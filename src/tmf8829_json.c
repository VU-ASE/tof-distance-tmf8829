/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include "tmf8829_json.h"
#include "tmf8829.h"
#include "tmf8829_shim.h"

/* ============================================================================
 * Internal Thread Functions
 * ============================================================================ */

/**
 * @brief Internal function to write frame data to JSON file (called from writer thread)
 */
static int tmf8829_json_write_frame_internal(tmf8829_json_logger_t *logger,
                                               json_frame_data_t *frame_data);

/**
 * @brief Writer thread function - processes frames from queue and writes to JSON file
 */
static void* json_writer_thread(void *arg)
{
    tmf8829_json_logger_t *logger = (tmf8829_json_logger_t *)arg;
    json_frame_data_t frame_data;
    json_frame_data_t *queue_slot;
    int frame_num = 0;
    int have_frame = 0;
    int queue_tail_idx;

    PRINT_INFO("JSON writer thread started\n");
    
    /* Set thread to allow cancellation */
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    
    while (1)
    {
        /* Wait for frames to be available or stop request */
        pthread_mutex_lock(&logger->queue_mutex);
        
        /* Wait only if queue is empty AND not stopping */
        while (logger->queue_count == 0 && !logger->stop_requested)
        {
            pthread_cond_wait(&logger->queue_cond, &logger->queue_mutex);
        }
        
        /* Check if stop was requested */
        if (logger->stop_requested && logger->queue_count == 0)
        {
            pthread_mutex_unlock(&logger->queue_mutex);
            //PRINT_DEBUG("JSON writer thread: stop requested and queue empty, exiting...\n");
            break;
        }
        
        /* Get frame from queue */
        have_frame = 0;
        if (logger->queue_count > 0)
        {
            /* Save queue tail index to update pointers later */
            queue_tail_idx = logger->queue_tail;
            /* Get reference to queue slot to update pointers after writing */
            queue_slot = &logger->frame_queue[queue_tail_idx];
            frame_data = *queue_slot;  /* Copy the data */
#if ENABLE_HISTOGRAM
            PRINT_DEBUG("Dequeued frame %d, pixelResults=%p, histograms=%p, histogramsHA=%p\n",
                       frame_data.frame.frameNumber, (void*)frame_data.pixelResults,
                       (void*)frame_data.histograms, (void*)frame_data.histogramsHA);
#else
            PRINT_DEBUG("Dequeued frame %d, pixelResults=%p\n",
                       frame_data.frame.frameNumber, (void*)frame_data.pixelResults);
#endif
            logger->queue_tail = (logger->queue_tail + 1) % JSON_QUEUE_SIZE;
            logger->queue_count--;
            have_frame = 1;

            /* Signal that queue is not full anymore */
            pthread_cond_signal(&logger->queue_full_cond);
        }

        pthread_mutex_unlock(&logger->queue_mutex);

        /* Write the frame outside of mutex to avoid blocking producers */
        if (have_frame)
        {
            frame_num++;

            /* Sanity check: ensure frame data is valid */
            if (frame_data.pixelResults == NULL)
            {
                PRINT_DEBUG("ERROR: frame_data.pixelResults is NULL for frame %d\n", frame_data.frame.frameNumber);
                continue;
            }

            uint32_t start = getSysTick();
            int kbWritten = tmf8829_json_write_frame_internal(logger, &frame_data);
            uint32_t end = getSysTick();

            /* Free dynamically allocated memory for this frame */
            if (frame_data.pixelResults != NULL)
            {
                free(frame_data.pixelResults);
            }
#if ENABLE_HISTOGRAM
            if (frame_data.histograms != NULL)
            {
                free(frame_data.histograms);
            }
            if (frame_data.histogramsHA != NULL)
            {
                free(frame_data.histogramsHA);
            }
#endif

            /* Update queue slot pointers to NULL */
            /* We need to use mutex here because the slot might be reused by producer */
            pthread_mutex_lock(&logger->queue_mutex);
            queue_slot->pixelResults = NULL;
#if ENABLE_HISTOGRAM
            queue_slot->histograms = NULL;
            queue_slot->histogramsHA = NULL;
#endif
            pthread_mutex_unlock(&logger->queue_mutex);

            //if (frame_num % 10 == 0)
            {
                uint32_t writeTimeMs = (end - start) / 1000.0;
                uint32_t bps = writeTimeMs > 0 ? kbWritten / writeTimeMs : 0.0;
                PRINT_INFO("Frame #%d saved, %dkB/%dms=%dkB/ms, queue size %d\n",
                       frame_data.frame.frameNumber, kbWritten, writeTimeMs, bps, logger->queue_count);
            }
        }
        else if (logger->stop_requested)
        {
            /* No more frames and stop requested */
            break;
        }
    }
    
    //PRINT_DEBUG("JSON writer thread stopped (total %d frames written\n", frame_num);
    return NULL;
}

/**
 * @brief Internal function to write frame data to JSON file (called from writer thread)
 */
static int tmf8829_json_write_frame_internal(tmf8829_json_logger_t *logger,
                                               json_frame_data_t *frame_data)
{
    int row, col, i;
    tmf8829PixelResult_t *pixel;
    tmf8829FrameData_t *frame;
    int numPeaks;
    int useNoise, useXtalk, useSignal;
#if ENABLE_HISTOGRAM
    int bin;
    tmf8829Histogram_t *histogram = NULL;
    int binSize;
#endif
    z_off_t startPos, endPos;
    int kbWritten;

    if (logger == NULL || !logger->is_open || frame_data == NULL)
    {
        return 0;
    }

    startPos = gztell(logger->file);
    
    frame = &frame_data->frame;
    
    /* Parse result format */
    numPeaks = frame_data->resultFormat & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
    if (numPeaks == 0) numPeaks = 1;
    useNoise = (frame_data->resultFormat & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK) ? 1 : 0;
    useXtalk = (frame_data->resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK) ? 1 : 0;
    useSignal = (frame_data->resultFormat & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK) ? 1 : 0;
    
    /* Write frame header - add comma if this is not the first frame */
    gzprintf(logger->file,
             "        %s{\n"
             "            \"info\": {\n"
             "                \"frame_number\": %d,\n"
             "                \"read_time\": %u,\n"
             "                \"systick_t0\": %u,\n"
             "                \"systick_t1\": %u,\n"
             "                \"temperature\": %d,\n"
             "                \"warnings\": %d\n"
             "            },\n",
             logger->frame_count > 0 ? "," : "",
             frame->frameNumber, frame->systick, frame->t0Integration,
             frame->t1Integration, frame->temperature, frame->warnings);
    
#if ENABLE_HISTOGRAM
    /* Determine bin size based on mode */
    if (frame->numRows == 8 && frame->numCols == 8)
    {
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_8x8;
    }
    else
    {
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_OTHER;
    }
    
    /* Write histograms if available */
    if (frame_data->histogram_ready)
    {
        gzprintf(logger->file, "            \"mp_histo\": [\n");

        for (row = 0; row < frame->numRows; row++)
        {
            gzprintf(logger->file, "                [\n");
            for (col = 0; col < frame->numCols; col++)
            {
                if (frame_data->histograms != NULL)
                {
                    histogram = &frame_data->histograms[row * frame->numCols + col];
                    gzprintf(logger->file,
                             "                    {\n"
                             "                        \"bin\": [");
                    for (bin = 0; bin < binSize; bin++)
                    {
                        gzprintf(logger->file, "%u%s", histogram->bin[bin],
                                 bin < binSize - 1 ? "," : "");
                    }
                    gzprintf(logger->file,
                             "]\n"
                             "                    }%s\n",
                             col < frame->numCols - 1 ? "," : "");
                }
                else
                {
                    PRINT_DEBUG("ERROR: histograms is NULL at row=%d, col=%d\n", row, col);
                    /* Write empty histogram to avoid breaking JSON format */
                    gzprintf(logger->file,
                             "                    {\n"
                             "                        \"bin\": []\n"
                             "                    }%s\n",
                             col < frame->numCols - 1 ? "," : "");
                }
            }
            gzprintf(logger->file,
                     "                ]%s\n",
                     row < frame->numRows - 1 ? "," : "");
        }
        gzprintf(logger->file, "            ],\n");
        
        /* Write HA histograms if available (dual mode) */
        if (frame_data->histogram_ha_ready)
        {
            gzprintf(logger->file, "            \"mp_histo_HA\": [\n");
            
            for (row = 0; row < frame->numRows; row++)
            {
                gzprintf(logger->file, "                [\n");
                for (col = 0; col < frame->numCols; col++)
                {
                    histogram = &frame_data->histogramsHA[row * frame->numCols + col];
                    gzprintf(logger->file,
                             "                    {\n"
                             "                        \"bin\": [");
                    for (bin = 0; bin < binSize; bin++)
                    {
                        gzprintf(logger->file, "%u%s", histogram->bin[bin],
                                 bin < binSize - 1 ? "," : "");
                    }
                    gzprintf(logger->file,
                             "]\n"
                             "                    }%s\n",
                             col < frame->numCols - 1 ? "," : "");
                }
                gzprintf(logger->file,
                         "                ]%s\n",
                         row < frame->numRows - 1 ? "," : "");
            }
            gzprintf(logger->file, "            ],\n");
        }
        else
        {
            /* No HA histogram data - write empty array */
            gzprintf(logger->file,
                     "            \"mp_histo_HA\": [\n"
                     "            ],\n");
        }
    }
    else
    {
        /* No histogram data - write empty arrays */
        gzprintf(logger->file,
                 "            \"mp_histo\": [\n"
                 "            ],\n"
                 "            \"mp_histo_HA\": [\n"
                 "            ],\n");
    }
#else
    /* Histogram disabled - write empty array */
    gzprintf(logger->file,
             "            \"mp_histo\": [\n"
             "            ],\n");
#endif
    
    /* Write reference histogram (empty) and results header */
    gzprintf(logger->file,
             "            \"ref_histo\": [\n"
             "            ],\n"
             "            \"results\": [\n");
    
    /* Write results */
    for (row = 0; row < frame->numRows; row++)
    {
        gzprintf(logger->file, "                [\n");

        for (col = 0; col < frame->numCols; col++)
        {
            pixel = &frame_data->pixelResults[row * frame->numCols + col];
            
            gzprintf(logger->file,
                     "                    {\n"
                     "                        \"noise\": %d,\n"
                     "                        \"peaks\": [\n",
                     useNoise ? pixel->noise : 0);
            
            for (i = 0; i < numPeaks; i++)
            {
                gzprintf(logger->file,
                         "                            {\n"
                         "                                \"distance\": %d,\n"
                         "                                \"signal\": %d,\n"
                         "                                \"snr\": %d,\n"
                         "                                \"x\": \"%.2f\",\n"
                         "                                \"y\": \"%.2f\",\n"
                         "                                \"z\": \"%.2f\"\n"
                         "                        }%s\n",
                         pixel->peaks[i].distance,
                         useSignal ? pixel->peaks[i].signal : 0,
                         pixel->peaks[i].snr,
                         pixel->peaks[i].x, pixel->peaks[i].y, pixel->peaks[i].z,
                         i < numPeaks - 1 ? "," : "");
            }
            
            gzprintf(logger->file,
                     "                        ],\n"
                     "                        \"xtalk\": %d\n"
                     "                    }%s\n",
                     useXtalk ? pixel->xtalk : 0,
                     col < frame->numCols - 1 ? "," : "");
        }
        
        gzprintf(logger->file,
                 "                ]%s\n",
                 row < frame->numRows - 1 ? "," : "");
    }
    
    /* Close frame object */
    gzprintf(logger->file,
             "            ]\n"
             "        }\n");

    endPos = gztell(logger->file);
    kbWritten = (int)((endPos - startPos) >> 10);

    logger->frame_count++;
    //gzflush(logger->file, Z_SYNC_FLUSH);

    return kbWritten;
}

void tmf8829_json_get_timestamp(char *buffer, int size)
{
    time_t now;
    struct tm *tm_info;
    
    time(&now);
    tm_info = localtime(&now);
    
    snprintf(buffer, size, "%04d-%02d-%02d-%02d-%02d-%02d",
             tm_info->tm_year + 1900,
             tm_info->tm_mon + 1,
             tm_info->tm_mday,
             tm_info->tm_hour,
             tm_info->tm_min,
             tm_info->tm_sec);
}

int tmf8829_json_init(tmf8829_json_logger_t *logger, const char *prefix,
                      uint32_t serial_number, const uint8_t fw_version[4])
{
    char timestamp[64];
    int ret;
    
    if (logger == NULL)
    {
        return -1;
    }
    
    memset(logger, 0, sizeof(tmf8829_json_logger_t));
    logger->serial_number = serial_number;
    memcpy(logger->fw_version, fw_version, 4);
    logger->frame_count = 0;
    logger->thread_running = 0;
    logger->stop_requested = 0;
    logger->queue_head = 0;
    logger->queue_tail = 0;
    logger->queue_count = 0;

    /* Initialize queue slots to NULL */
    int i;
    for (i = 0; i < JSON_QUEUE_SIZE; i++)
    {
        logger->frame_queue[i].pixelResults = NULL;
#if ENABLE_HISTOGRAM
        logger->frame_queue[i].histograms = NULL;
        logger->frame_queue[i].histogramsHA = NULL;
        logger->frame_queue[i].histogram_ready = 0;
        logger->frame_queue[i].histogram_ha_ready = 0;
#endif
    }

    /* Initialize mutex and condition variables */
    if (pthread_mutex_init(&logger->queue_mutex, NULL) != 0)
    {
        PRINT_INFO("Error: Failed to initialize queue mutex\n");
        return -1;
    }

    if (pthread_cond_init(&logger->queue_cond, NULL) != 0)
    {
        PRINT_INFO("Error: Failed to initialize queue condition variable\n");
        pthread_mutex_destroy(&logger->queue_mutex);
        return -1;
    }

    if (pthread_cond_init(&logger->queue_full_cond, NULL) != 0)
    {
        PRINT_INFO("Error: Failed to initialize queue full condition variable\n");
        pthread_mutex_destroy(&logger->queue_mutex);
        pthread_cond_destroy(&logger->queue_cond);
        return -1;
    }
    
    tmf8829_json_get_timestamp(timestamp, sizeof(timestamp));
    
    /* File name format: tmf8829_UID<number>-<timestamp>.json.gz */
    snprintf(logger->file_path, JSON_MAX_FILE_PATH,
             "%s_UID%u-%s.json.gz",
             prefix,
             serial_number,
             timestamp);
    
    logger->file = gzopen(logger->file_path, "wb1");
    if (logger->file == NULL)
    {
        PRINT_INFO("Cannot create JSON file: %s\n", strerror(errno));
        pthread_mutex_destroy(&logger->queue_mutex);
        pthread_cond_destroy(&logger->queue_cond);
        pthread_cond_destroy(&logger->queue_full_cond);
        return -1;
    }
    
    logger->is_open = 1;
    
    /* Write file header - start with Result_Set array */
    gzprintf(logger->file, "{\n");
    gzprintf(logger->file, "    \"Result_Set\": [\n");
    
    /* Start the writer thread */
    logger->thread_running = 1;
    logger->stop_requested = 0;
    ret = pthread_create(&logger->writer_thread, NULL, json_writer_thread, logger);
    if (ret != 0)
    {
        PRINT_INFO("Failed to create JSON writer thread: %d\n", ret);
        gzclose(logger->file);
        logger->is_open = 0;
        pthread_mutex_destroy(&logger->queue_mutex);
        pthread_cond_destroy(&logger->queue_cond);
        pthread_cond_destroy(&logger->queue_full_cond);
        return -1;
    }

    PRINT_INFO("JSON logger initialized: %s (async mode)\n", logger->file_path);
    
    return 0;
}

void tmf8829_json_close(tmf8829_json_logger_t *logger)
{
    int i;

    if (logger == NULL || !logger->is_open)
    {
        return;
    }

    //PRINT_DEBUG("JSON logger closing...\n");

    /* Flush any remaining data */
    gzflush(logger->file, Z_FINISH);

    /* Close file */
    gzclose(logger->file);
    logger->is_open = 0;

    /* Free dynamically allocated memory in queue */
    for (i = 0; i < JSON_QUEUE_SIZE; i++)
    {
        if (logger->frame_queue[i].pixelResults != NULL)
        {
            free(logger->frame_queue[i].pixelResults);
            logger->frame_queue[i].pixelResults = NULL;
        }
#if ENABLE_HISTOGRAM
        if (logger->frame_queue[i].histograms != NULL)
        {
            free(logger->frame_queue[i].histograms);
            logger->frame_queue[i].histograms = NULL;
        }
        if (logger->frame_queue[i].histogramsHA != NULL)
        {
            free(logger->frame_queue[i].histogramsHA);
            logger->frame_queue[i].histogramsHA = NULL;
        }
#endif
    }

    /* Free histogram buffers */
#if ENABLE_HISTOGRAM
    if (logger->histograms != NULL)
    {
        free(logger->histograms);
        logger->histograms = NULL;
    }
    if (logger->histogramsHA != NULL)
    {
        free(logger->histogramsHA);
        logger->histogramsHA = NULL;
    }
#endif

    /* Destroy mutex and condition variables */
    pthread_mutex_destroy(&logger->queue_mutex);
    pthread_cond_destroy(&logger->queue_cond);
    pthread_cond_destroy(&logger->queue_full_cond);

    PRINT_INFO("JSON logger closed: %s (%d frames saved)\n",
           logger->file_path, logger->frame_count);
}

void tmf8829_json_save_config(tmf8829_json_logger_t *logger, tmf8829_cfg_t *cfg)
{
    if (logger == NULL || cfg == NULL)
    {
        return;
    }
    memcpy(&logger->cfg, cfg, sizeof(tmf8829_cfg_t));
}

void tmf8829_json_flush(tmf8829_json_logger_t *logger)
{
    if (logger == NULL || !logger->is_open)
    {
        return;
    }
    
    /* Signal writer thread to stop */
    if (logger->thread_running)
    {
        PRINT_DEBUG("Waiting for JSON writer thread to finish before flush...\n");
        
        pthread_mutex_lock(&logger->queue_mutex);
        logger->stop_requested = 1;
        pthread_cond_signal(&logger->queue_cond);
        pthread_mutex_unlock(&logger->queue_mutex);
        
        /* Wait for writer thread to finish processing all frames */
        /* Give it some time to process remaining frames */
        usleep(500000);  /* 500ms delay */
        
        /* Now join the thread */
        int ret = pthread_join(logger->writer_thread, NULL);
        if (ret != 0)
        {
            PRINT_INFO("Warning: pthread_join failed during flush (%d)\n", ret);
        }
        logger->thread_running = 0;

        PRINT_DEBUG("Writer thread stopped, remaining frames in queue: %d\n", logger->queue_count);
    }
    
    /* Close Result_Set array */
    gzprintf(logger->file, "    ],\n");
    
    /* Write configuration */
    gzprintf(logger->file, "    \"configuration\": {\n");
    gzprintf(logger->file, "        \"confidence_threshold\": %d,\n", logger->cfg.conf_threshold);
    gzprintf(logger->file, "        \"dead_time\": %d,\n", logger->cfg.deadtime);
    gzprintf(logger->file, "        \"dual_mode\": %d,\n", logger->cfg.dualMode);
    gzprintf(logger->file, "        \"fp_mode\": %d,\n", logger->cfg.fpMode);
    gzprintf(logger->file, "        \"high_accuracy_iterations\": %d,\n", logger->cfg.shortIteration);
    gzprintf(logger->file, "        \"histograms\": %d,\n", logger->cfg.histogram_dump);
    gzprintf(logger->file, "        \"iterations\": %d,\n", logger->cfg.iteration);
    gzprintf(logger->file, "        \"nr_peaks\": %d,\n", logger->cfg.resultFormat & 0x07);
    gzprintf(logger->file, "        \"period\": %d,\n", logger->cfg.period);
    gzprintf(logger->file, "        \"noise_strength\": %d,\n", (logger->cfg.resultFormat & 0x10) ? 1 : 0);
    gzprintf(logger->file, "        \"signal_strength\": %d,\n", (logger->cfg.resultFormat & 0x08) ? 1 : 0);
    gzprintf(logger->file, "        \"xtalk\": %d\n", (logger->cfg.resultFormat & 0x20) ? 1 : 0);
    gzprintf(logger->file, "    },\n");
    
    /* Write info */
    gzprintf(logger->file, "    \"info\": [\n");
    gzprintf(logger->file, "        {\n");
    gzprintf(logger->file, "            \"fw version\": [\n");
    gzprintf(logger->file, "                %d,\n", logger->fw_version[0]);
    gzprintf(logger->file, "                %d,\n", logger->fw_version[1]);
    gzprintf(logger->file, "                %d,\n", logger->fw_version[2]);
    gzprintf(logger->file, "                %d\n", logger->fw_version[3]);
    gzprintf(logger->file, "            ],\n");
    gzprintf(logger->file, "            \"logger version\": \"%s\",\n", JSON_LOGGER_VERSION);
    gzprintf(logger->file, "            \"serial number\": %u\n", logger->serial_number);
    gzprintf(logger->file, "        }\n");
    gzprintf(logger->file, "    ]\n");
    gzprintf(logger->file, "}\n");
    
    gzflush(logger->file, Z_SYNC_FLUSH);
}

int tmf8829_json_queue_frame(tmf8829_json_logger_t *logger,
                               tmf8829FrameParser_t *parser,
                               uint8_t resultFormat)
{
    int next_head;
    json_frame_data_t *frame_data;

    if (logger == NULL || !logger->is_open || parser == NULL)
    {
        return -1;
    }
    
    pthread_mutex_lock(&logger->queue_mutex);
    
    /* Wait if queue is full (with timeout to avoid blocking indefinitely) */
    next_head = (logger->queue_head + 1) % JSON_QUEUE_SIZE;
    if (logger->queue_count >= JSON_QUEUE_SIZE)
    {
        PRINT_INFO("Warning: JSON queue full, dropping frame %d\n", parser->frame.frameNumber);
        pthread_mutex_unlock(&logger->queue_mutex);
        return -1;
    }
    
    /* Get frame data slot */
    frame_data = &logger->frame_queue[logger->queue_head];

#if ENABLE_HISTOGRAM
    PRINT_DEBUG("Queue slot %d: pixelResults=%p, histograms=%p, histogramsHA=%p (before alloc)\n",
           logger->queue_head, (void*)frame_data->pixelResults,
           (void*)frame_data->histograms, (void*)frame_data->histogramsHA);
#else
    PRINT_DEBUG("Queue slot %d: pixelResults=%p (before alloc)\n",
           logger->queue_head, (void*)frame_data->pixelResults);
#endif

    /* Calculate number of pixels based on resolution */
    int numPixels = parser->frame.numRows * parser->frame.numCols;

    /* Allocate pixel results array if needed */
    if (frame_data->pixelResults == NULL)
    {
        PRINT_DEBUG("Allocating pixelResults for frame %d at slot %d\n",
                   parser->frame.frameNumber, logger->queue_head);
        frame_data->pixelResults = (tmf8829PixelResult_t *)malloc(
            sizeof(tmf8829PixelResult_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (frame_data->pixelResults == NULL)
        {
            PRINT_INFO("Error: Failed to allocate pixel results for frame %d\n",
                   parser->frame.frameNumber);
            pthread_mutex_unlock(&logger->queue_mutex);
            return -1;
        }
        PRINT_DEBUG("Allocated pixelResults at %p for frame %d\n",
                   (void*)frame_data->pixelResults, parser->frame.frameNumber);
    }

#if ENABLE_HISTOGRAM
    /* Allocate histogram arrays if needed */
    if (frame_data->histograms == NULL)
    {
        frame_data->histograms = (tmf8829Histogram_t *)malloc(
            sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (frame_data->histograms == NULL)
        {
            PRINT_INFO("Error: Failed to allocate histograms for frame %d\n",
                   parser->frame.frameNumber);
            pthread_mutex_unlock(&logger->queue_mutex);
            return -1;
        }
    }

    if (frame_data->histogramsHA == NULL)
    {
        frame_data->histogramsHA = (tmf8829Histogram_t *)malloc(
            sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (frame_data->histogramsHA == NULL)
        {
            PRINT_INFO("Error: Failed to allocate HA histograms for frame %d\n",
                   parser->frame.frameNumber);
            pthread_mutex_unlock(&logger->queue_mutex);
            return -1;
        }
    }
#endif

    /* Copy frame data */
    frame_data->frame = parser->frame;
    frame_data->resultFormat = resultFormat;

    /* Update frame pixelResults pointer to point to our allocated array */
    /* This is critical because parser->frame.pixelResults may point to parser's static array */
    frame_data->frame.pixelResults = frame_data->pixelResults;

    /* Copy pixel results */
    memcpy(frame_data->pixelResults, parser->pixelResults,
           sizeof(tmf8829PixelResult_t) * numPixels);

#if ENABLE_HISTOGRAM
    /* Copy histogram data if available */
    frame_data->histogram_ready = logger->histogram_ready;
    frame_data->histogram_ha_ready = logger->histogram_ha_ready;

    /* Copy default histograms */
    int histogram_ready_copy = logger->histogram_ready;
    if (histogram_ready_copy && logger->histograms != NULL)
    {
        memcpy(frame_data->histograms, logger->histograms,
               sizeof(tmf8829Histogram_t) * numPixels);
    }

    /* Copy HA histograms if available (dual mode) */
    int histogram_ha_ready_copy = logger->histogram_ha_ready;
    if (histogram_ha_ready_copy && logger->histogramsHA != NULL)
    {
        memcpy(frame_data->histogramsHA, logger->histogramsHA,
               sizeof(tmf8829Histogram_t) * numPixels);
    }

    /* Update frame histogram pointers to point to our allocated arrays */
    /* This is important to avoid dangling pointers */
    if (histogram_ha_ready_copy)
    {
        frame_data->frame.histograms = frame_data->histogramsHA;
    }
    else if (histogram_ready_copy)
    {
        frame_data->frame.histograms = frame_data->histograms;
    }
    else
    {
        /* No histograms available, set to NULL */
        frame_data->frame.histograms = NULL;
    }

    /* Clear histogram ready flags after copying both */
    if (histogram_ready_copy)
    {
        logger->histogram_ready = 0;
    }
    if (histogram_ha_ready_copy)
    {
        logger->histogram_ha_ready = 0;
    }
#endif
    
    /* Update queue pointers */
    logger->queue_head = next_head;
    logger->queue_count++;
    
    /* Signal the writer thread */
    pthread_cond_signal(&logger->queue_cond);
    
    pthread_mutex_unlock(&logger->queue_mutex);
    
    return 0;
}

void tmf8829_json_write_frame(tmf8829_json_logger_t *logger,
                               tmf8829FrameParser_t *parser,
                               uint8_t resultFormat)
{
    /* This function now queues the frame for async writing */
    tmf8829_json_queue_frame(logger, parser, resultFormat);
}

#if ENABLE_HISTOGRAM
void tmf8829_json_cache_histogram(tmf8829_json_logger_t *logger,
                                   tmf8829FrameParser_t *parser,
                                   int subFrame)
{
    int row, col, bin;
    int numRows, numCols;
    int fovRows, fovCols;
    int pixelRowsPerMp, pixelColsPerMp;
    int leftFovOffset, pixelRowOffset, pixelColumnOffset;
    int targetRow, targetCol;
    int numFramesNeeded;
    tmf8829FrameData_t *frame;
    int binSize;
    int isHAHistogram = 0;
    
    if (logger == NULL || !logger->is_open || parser == NULL)
    {
        return;
    }

    PRINT_DEBUG("JSON cache_histogram: subFrame=%d, histograms=%p, histogramsHA=%p\n",
           subFrame, (void*)logger->histograms, (void*)logger->histogramsHA);

    /* Store dual mode setting from parser */
    logger->dualMode = parser->dualMode;
    
    /* Determine if this is HA or Default histogram in dual mode
     * Use the value directly from frameparser which tracks this correctly */
    if (parser->dualMode != 0)
    {
        isHAHistogram = parser->isHAHistogram;
        PRINT_DEBUG("JSON histogram: subFrame=%d, dualMode=%d, isHAHistogram=%d, dualHistoCount=%d\n",
               subFrame, parser->dualMode, isHAHistogram, parser->dualHistoCount);
    }
    
    frame = &parser->frame;
    numRows = frame->numRows;
    numCols = frame->numCols;
    
    /* Determine bin size */
    if (numRows == 8 && numCols == 8)
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_8x8;
    else
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_OTHER;
    
    /* Allocate histogram buffers if not already done */
    if (logger->histograms == NULL)
    {
        logger->histograms = (tmf8829Histogram_t *)malloc(
                sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (logger->histograms == NULL)
        {
            PRINT_INFO("Failed to allocate histogram buffer\n");
            return;
        }
        memset(logger->histograms, 0,
                   sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
    }

    if (logger->histogramsHA == NULL && parser->dualMode != 0)
    {
        logger->histogramsHA = (tmf8829Histogram_t *)malloc(
                sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (logger->histogramsHA == NULL)
        {
            PRINT_INFO("Failed to allocate HA histogram buffer\n");
            return;
        }
        memset(logger->histogramsHA, 0,
                   sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
    }
    
    /* Calculate number of histogram frames needed per measurement
     * Note: For low-res modes (8x8, 16x16), the frame contains fovRows x (fovCols/2) histograms,
     * but subFrame (layout) values are 0 and 1 to cover left and right halves of FOV.
     * For high-res modes, multiple layout values are used to cover the full FOV.
     *
     * We need to track unique subFrame (layout) values, not total histogram frames.
     * - 8x8/16x16: layout 0 (left half) + layout 1 (right half) = 2 layouts
     * - 32x32: layout 0-3 (sub-frame 0 cycle) + layout 4-7 (sub-frame 1 cycle) = 8 layouts total per full cycle
     * - 48x32: layout 0-5 (sub-frame 0 cycle) + layout 6-11 (sub-frame 1 cycle) = 12 layouts total per full cycle
     *
     * For dual mode, numFramesNeeded is the number of layouts per HA or Default phase:
     * - 8x8/16x16 dual: 2 layouts per phase
     * - 32x32 dual: 4 layouts per phase (0-3 for sub0, 4-7 for sub1)
     * - 48x32 dual: 6 layouts per phase (0-5 for sub0, 6-11 for sub1)
     */
    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        /* High resolution modes */
        if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
            numFramesNeeded = 6;  /* layouts per phase: 0-5 for sub0, 6-11 for sub1 */
        else
            numFramesNeeded = 4;  /* layouts per phase: 0-3 for sub0, 4-7 for sub1 */
    }
    else
    {
        /* Low resolution modes: layout 0 (left half) + layout 1 (right half) */
        numFramesNeeded = 2;
    }
    
    /* For dual mode, we need both HA and Default histograms (each with same layouts) */
    if (parser->dualMode != 0)
    {
        /* In dual mode, each layout appears twice: first HA, then Default.
         * But we track them separately using histogram_subframes_received and histogram_subframes_received_ha.
         * So numFramesNeeded stays the same - it's the number of layouts per HA or Default phase. */
    }
    
    /* Calculate FOV dimensions and scaling factors */
    if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_8x8B)
    {
        fovRows = 8;
        fovCols = 8;  /* Full FOV width, but frame has only fovCols/2 columns */
    }
    else
    {
        fovRows = 16;
        fovCols = 16;
    }
    
    pixelRowsPerMp = numRows / fovRows;
    pixelColsPerMp = numCols / fovCols;
    
    /* Frame actually contains fovRows x (fovCols/2) histograms */
    int frameCols = fovCols / 2;
    
    /* Calculate position offsets based on layout (subFrame number) */
    leftFovOffset = 0;
    pixelRowOffset = 0;
    pixelColumnOffset = 0;
    
    /* Odd layout: right half of FOV */
    if (subFrame % 2 != 0)
    {
        leftFovOffset = numCols / 2;
    }
    
    if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
    {
        /* 48x32 mode: layout in [2,3,8,9] -> colOffset=1, [4,5,10,11] -> colOffset=2 */
        if (subFrame == 2 || subFrame == 3 || subFrame == 8 || subFrame == 9)
        {
            pixelColumnOffset = 1;
        }
        if (subFrame == 4 || subFrame == 5 || subFrame == 10 || subFrame == 11)
        {
            pixelColumnOffset = 2;
        }
        if (subFrame > 5)
        {
            pixelRowOffset = 1;
        }
    }
    else if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        /* 32x32 mode: layout in [2,3,6,7] -> colOffset=1, layout > 3 -> rowOffset=1 */
        if (subFrame == 2 || subFrame == 3 || subFrame == 6 || subFrame == 7)
        {
            pixelColumnOffset = 1;
        }
        if (subFrame > 3)
        {
            pixelRowOffset = 1;
        }
    }
    
    /* Check if this is the start of a new measurement cycle
     *
     * For low-res modes (8x8, 16x16):
     *   - subFrame 0: left half of FOV
     *   - subFrame 1: right half of FOV
     *   - New cycle starts when both subFrames are received (histogram_ready is set)
     *
     * For high-res modes (32x32, 48x32):
     *   - subFrame 0 cycle: subFrames 0-3 (or 0-5 for 48x32)
     *   - subFrame 1 cycle: subFrames 4-7 (or 6-11 for 48x32)
     *   - New cycle starts when moving from first group to second group
     *
     * We detect a new cycle by checking:
     *   - For low-res: histogram_ready is already set (previous cycle complete)
     *   - For high-res: subFrame value jumps from lower group to higher group
     */

    /* Check if we should start a new cycle based on mode */
    int startNewCycle = 0;

    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        /* High resolution mode - track sub-frame 0 and sub-frame 1 separately */
        int subFramesPerCycle;
        if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
            subFramesPerCycle = 6;
        else
            subFramesPerCycle = 4;

        int isFirstSubframeGroup = (subFrame < subFramesPerCycle);

        /* Check if we should start a new cycle */
        if (isFirstSubframeGroup)
        {
            /* First group (0-3 or 0-5) */
            /* For high resolution dual mode, only start new cycle if all histograms are ready */
            if (parser->dualMode != 0)
            {
                /* High resolution dual mode: wait for both sub-frames and both phases */
                if (logger->histogram_ready && logger->histogram_ha_ready)
                {
                    /* All histograms complete and written, start new cycle */
                    startNewCycle = 1;
                }
                else if (isHAHistogram && (logger->histogram_subframes_received_ha & 0x01) == 0)
                {
                    /* First HA histogram of a new cycle (sub-frame 0) */
                    startNewCycle = 1;
                }
                /* Note: For dual mode, don't start new cycle for Default subFrame=0
                 * Wait for HA phase to complete first, and wait for both sub-frames */
            }
            else
            {
                /* Non-dual mode */
                if (logger->histogram_subframe0_ready)
                {
                    /* Previous sub-frame 0 cycle complete but not yet written, start new cycle */
                    startNewCycle = 1;
                }
                else if ((logger->histogram_subframes_received & 0x01) == 0)
                {
                    /* First histogram of a new cycle (sub-frame 0) */
                    startNewCycle = 1;
                }
            }
        }
        else
        {
            /* Second group (4-7 or 6-11) - sub-frame 1 data
             * Don't start new cycle here - continue accumulating sub-frame 1 data
             * New cycle will be started when both sub-frames are complete and written */
            startNewCycle = 0;
        }
    }
    else
    {
        /* Low resolution mode */
        /* IMPORTANT: For dual mode, we need both HA and Default histograms before starting new cycle
         * Only start new cycle if BOTH histogram_ready AND histogram_ha_ready are set */
        if (parser->dualMode != 0)
        {
            /* Dual mode: need both HA and Default complete */
            if (logger->histogram_ready && logger->histogram_ha_ready)
            {
                /* Both phases complete, start new cycle */
                startNewCycle = 1;
            }
            else if (subFrame == 0 && isHAHistogram && (logger->histogram_subframes_received_ha & 0x01) == 0)
            {
                /* First HA histogram of a new cycle - only if no HA data received yet */
                startNewCycle = 1;
            }
        }
        else
        {
            /* Non-dual mode */
            if (logger->histogram_ready)
            {
                /* Previous cycle complete but not yet written, start new cycle */
                startNewCycle = 1;
            }
            else if (subFrame == 0 && (logger->histogram_subframes_received & 0x01) == 0)
            {
                /* First frame of a cycle (subFrame 0) */
                startNewCycle = 1;
            }
        }
    }
    
    if (startNewCycle)
    {
        PRINT_DEBUG("JSON histogram: starting new cycle (subFrame=%d, isHA=%d)\n", subFrame, isHAHistogram);
        PRINT_DEBUG("JSON histogram: histograms=%p, histogramsHA=%p\n",
               (void*)logger->histograms, (void*)logger->histogramsHA);
        if (logger->histograms != NULL)
        {
            PRINT_DEBUG("JSON histogram: clearing histograms buffer\n");
            memset(logger->histograms, 0,
                   sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        }
        if (logger->histogramsHA != NULL)
        {
            PRINT_DEBUG("JSON histogram: clearing histogramsHA buffer\n");
            memset(logger->histogramsHA, 0,
                   sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        }
        logger->histogram_subframes_received = 0;
        logger->histogram_subframes_received_ha = 0;
        logger->histogram_ready = 0;
        logger->histogram_ha_ready = 0;
        logger->histogram_subframe0_ready = 0;
        logger->histogram_subframe1_ready = 0;
        logger->histogram_subframe0_ready_ha = 0;
        logger->histogram_subframe1_ready_ha = 0;
        logger->dualCurrentPhase = 0;  /* Start with HA phase */

        /* Determine the expected mask for this cycle */
        if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16 && parser->dualMode != 0)
        {
            /* High resolution dual mode */
            if (subFrame < numFramesNeeded)
            {
                /* First group (0-3 or 0-5) */
                logger->histogram_expected_subframes = (1 << numFramesNeeded) - 1;
            }
            else
            {
                /* Second group (4-7 or 6-11) */
                logger->histogram_expected_subframes = ((1 << numFramesNeeded) - 1) << numFramesNeeded;
            }
            PRINT_DEBUG("JSON histogram: set histogram_expected_subframes=0x%x for subFrame=%d\n",
                   logger->histogram_expected_subframes, subFrame);
        }
        else
        {
            /* Low resolution or non-dual mode */
            logger->histogram_expected_subframes = (1 << numFramesNeeded) - 1;
        }
        PRINT_DEBUG("JSON histogram: new measurement cycle starting\n");
    }
    
    /* If histogram is already ready for this cycle, skip */
    if ((isHAHistogram && logger->histogram_ha_ready) ||
        (!isHAHistogram && logger->histogram_ready))
    {
        PRINT_DEBUG("JSON histogram: skipping subFrame=%d (data already complete for this cycle)\n", subFrame);
        return;
    }

        PRINT_DEBUG("JSON histogram: before update, ha_mask=0x%x, def_mask=0x%x, numFramesNeeded=%d\n",
               logger->histogram_subframes_received_ha, logger->histogram_subframes_received, numFramesNeeded);

        PRINT_DEBUG("JSON histogram: subFrame=%d, leftFov=%d, rowOff=%d, colOff=%d, isHA=%d, ready=%d, ha_ready=%d\n",
               subFrame, leftFovOffset, pixelRowOffset, pixelColumnOffset, isHAHistogram,
               logger->histogram_ready, logger->histogram_ha_ready);
    
    /* Copy histogram data from parser to logger buffer */
    int pixelsCopied = 0;
    for (row = 0; row < fovRows; row++)
    {
        for (col = 0; col < frameCols; col++)
        {
            /* Calculate target position in full resolution grid */
            targetRow = row * pixelRowsPerMp + pixelRowOffset;
            targetCol = col * pixelColsPerMp + leftFovOffset + pixelColumnOffset;
            
            /* Bounds check */
            if (targetRow >= numRows || targetCol >= numCols)
            {
                continue;
            }
            
            /* Select source histogram based on parser's histogram pointer */
            tmf8829Histogram_t *srcHist = parser->frame.histograms;
            if (srcHist == NULL)
            {
                continue;
            }
            srcHist = &srcHist[targetRow * numCols + targetCol];
            
            /* Select destination buffer based on histogram type */
            tmf8829Histogram_t *dstHist;
            if (isHAHistogram && parser->dualMode != 0)
            {
                if (logger->histogramsHA == NULL)
                {
                    PRINT_INFO("JSON histogram: WARNING - histogramsHA is NULL, skipping pixel\n");
                    continue;
                }
                dstHist = &logger->histogramsHA[targetRow * numCols + targetCol];
            }
            else
            {
                if (logger->histograms == NULL)
                {
                    PRINT_INFO("JSON histogram: WARNING - histograms is NULL, skipping pixel\n");
                    continue;
                }
                dstHist = &logger->histograms[targetRow * numCols + targetCol];
            }
            
            /* Copy bins directly */
            for (bin = 0; bin < binSize; bin++)
            {
                dstHist->bin[bin] = srcHist->bin[bin];
            }
            pixelsCopied++;
        }
    }

        PRINT_DEBUG("JSON histogram: copied %d pixels\n", pixelsCopied);
    
    /* Track which sub-frames have been received */
    if (isHAHistogram && parser->dualMode != 0)
    {
        logger->histogram_subframes_received_ha |= (1 << subFrame);
        PRINT_DEBUG("JSON histogram: HA subFrame=%d, mask=0x%x\n", subFrame, logger->histogram_subframes_received_ha);
    }
    else
    {
        logger->histogram_subframes_received |= (1 << subFrame);
        PRINT_DEBUG("JSON histogram: Default subFrame=%d, mask=0x%x\n", subFrame, logger->histogram_subframes_received);
    }
    
    /* For high resolution dual mode, track phase changes
     * For 32x32: each phase (HA or Default) has 4 layouts (0-3 or 4-7)
     * For 48x32: each phase has 6 layouts (0-5 or 6-11)
     * For low resolution (8x8/16x16): each phase has 1 layout (0 or 1)
     */
    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16 && parser->dualMode != 0)
    {
        /* Calculate frames per phase based on mode */
        int framesPerPhase;
        if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
            framesPerPhase = 6;  /* layouts 0-5 or 6-11 */
        else
            framesPerPhase = 4;  /* layouts 0-3 or 4-7 */
        
        int allFramesMaskPhase = (1 << framesPerPhase) - 1;
        
        if (isHAHistogram && 
            (logger->histogram_subframes_received_ha & allFramesMaskPhase) == allFramesMaskPhase)
        {
            /* Completed HA phase, switch to Default phase */
            logger->dualCurrentPhase = 1;
        }
        else if (!isHAHistogram && 
                 (logger->histogram_subframes_received & allFramesMaskPhase) == allFramesMaskPhase)
        {
            /* Completed Default phase, switch back to HA phase */
            logger->dualCurrentPhase = 0;
        }
    }
    else if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_16x16 && parser->dualMode != 0)
    {
        /* Low resolution dual mode: each phase has 1 layout (0 or 1) */
        /* Phase switching is handled by dualHistoCount in the parser, but we need to 
         * track when we complete both HA and Default phases for the same layout */
    }
    
    /* Check if all frames received */
    /* Calculate mask based on number of unique layout values needed
     * For non-dual mode: need all layouts (0 to numFramesNeeded-1)
     * For dual mode: each phase (HA/Default) needs all layouts
     *
     * For low-res modes (8x8/16x16): layouts 0 and 1 cover left and right halves
     * For high-res modes: multiple layouts cover different parts of FOV
     */
    uint32_t allFramesMask;

    /* Check if high resolution dual mode */
    if (parser->dualMode != 0 && parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        /* High resolution dual mode: track sub-frame 0 and sub-frame 1 separately for both HA and Default */
        int subFramesPerCycle = (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32) ? 6 : 4;
        uint32_t subframe0Mask = (1 << subFramesPerCycle) - 1;  /* 0x0F or 0x3F */
        uint32_t subframe1Mask = subframe0Mask << subFramesPerCycle;  /* 0xF0 or 0xFC0 */

        PRINT_DEBUG("JSON histogram: checking complete (high-res dual), ha_mask=0x%x, def_mask=0x%x, sf0_mask=0x%x, sf1_mask=0x%x\n",
               logger->histogram_subframes_received_ha,
               logger->histogram_subframes_received,
               subframe0Mask, subframe1Mask);

        /* Sub-frame 0 HA */
        if ((logger->histogram_subframes_received_ha & subframe0Mask) == subframe0Mask &&
            !logger->histogram_subframe0_ready_ha)
        {
            logger->histogram_subframe0_ready_ha = 1;
            PRINT_DEBUG("JSON histogram: Sub-frame 0 HA histogram ready\n");
            
            /* Check if all histograms are ready */
            if (logger->histogram_subframe0_ready_ha && logger->histogram_subframe1_ready_ha &&
                logger->histogram_subframe0_ready && logger->histogram_subframe1_ready)
            {
                logger->histogram_ready = 1;
                logger->histogram_ha_ready = 1;
                PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames HA and Default histograms ready\n");
            }
        }

        /* Sub-frame 1 HA */
        if ((logger->histogram_subframes_received_ha & subframe1Mask) == subframe1Mask &&
            !logger->histogram_subframe1_ready_ha)
        {
            logger->histogram_subframe1_ready_ha = 1;
            PRINT_DEBUG("JSON histogram: Sub-frame 1 HA histogram ready\n");
            
            /* Check if all histograms are ready */
            if (logger->histogram_subframe0_ready_ha && logger->histogram_subframe1_ready_ha &&
                logger->histogram_subframe0_ready && logger->histogram_subframe1_ready)
            {
                logger->histogram_ready = 1;
                logger->histogram_ha_ready = 1;
                PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames HA and Default histograms ready\n");
            }
        }

        /* Check Default histograms */
        /* Sub-frame 0 Default */
        if ((logger->histogram_subframes_received & subframe0Mask) == subframe0Mask &&
            !logger->histogram_subframe0_ready)
        {
            logger->histogram_subframe0_ready = 1;
            PRINT_DEBUG("JSON histogram: Sub-frame 0 Default histogram ready\n");

            /* Check if all histograms are ready */
            PRINT_DEBUG("JSON histogram: checking COMPLETE (sf0_ha=%d, sf1_ha=%d, sf0_def=%d, sf1_def=%d\n)",
                   logger->histogram_subframe0_ready_ha, logger->histogram_subframe1_ready_ha,
                   logger->histogram_subframe0_ready, logger->histogram_subframe1_ready);
            if (logger->histogram_subframe0_ready_ha && logger->histogram_subframe1_ready_ha &&
                logger->histogram_subframe0_ready && logger->histogram_subframe1_ready)
            {
                logger->histogram_ready = 1;
                logger->histogram_ha_ready = 1;
                PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames HA and Default histograms ready\n");
            }
        }

        /* Sub-frame 1 Default */
        if ((logger->histogram_subframes_received & subframe1Mask) == subframe1Mask &&
            !logger->histogram_subframe1_ready)
        {
            logger->histogram_subframe1_ready = 1;
            PRINT_DEBUG("JSON histogram: Sub-frame 1 Default histogram ready\n");

            /* Check if all histograms are ready */
            PRINT_DEBUG("JSON histogram: checking COMPLETE (sf0_ha=%d, sf1_ha=%d, sf0_def=%d, sf1_def=%d\n)",
                   logger->histogram_subframe0_ready_ha, logger->histogram_subframe1_ready_ha,
                   logger->histogram_subframe0_ready, logger->histogram_subframe1_ready);
            if (logger->histogram_subframe0_ready_ha && logger->histogram_subframe1_ready_ha &&
                logger->histogram_subframe0_ready && logger->histogram_subframe1_ready)
            {
                logger->histogram_ready = 1;
                logger->histogram_ha_ready = 1;
                PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames HA and Default histograms ready\n");
            }
        }
    }
    else if (parser->dualMode != 0)
    {
        /* Low resolution dual mode */
        /* Use the histogram_expected_subframes that was set at the start of the cycle */
        allFramesMask = logger->histogram_expected_subframes;

        PRINT_DEBUG("JSON histogram: checking complete (low-res dual), ha_mask=0x%x(0x%x), def_mask=0x%x(0x%x), allMask=0x%x\n",
               logger->histogram_subframes_received_ha, allFramesMask,
               logger->histogram_subframes_received, allFramesMask, allFramesMask);

        /* Dual mode needs both HA and Default histograms */
        if ((logger->histogram_subframes_received_ha & allFramesMask) == allFramesMask &&
            (logger->histogram_subframes_received & allFramesMask) == allFramesMask)
        {
            logger->histogram_ready = 1;
            logger->histogram_ha_ready = 1;
            PRINT_DEBUG("JSON histogram: COMPLETE! Both HA and Default histograms ready\n");
        }
    }
    else
    {
        /* Non-dual mode: all layouts needed */
        allFramesMask = (1 << numFramesNeeded) - 1;

        /* Check if high resolution mode */
        if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
        {
            /* High resolution: track sub-frame 0 and sub-frame 1 separately */
            /* Check which sub-frames have been received */
            int subframe0Mask = allFramesMask;  /* 0x0F or 0x3F */
            int subFramesPerCycle = (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32) ? 6 : 4;
            int subframe1Mask = subframe0Mask << subFramesPerCycle;  /* 0xF0 or 0xFC0 */

            /* Check if sub-frame 0 cycle completed */
            if ((logger->histogram_subframes_received & subframe0Mask) == subframe0Mask &&
                !logger->histogram_subframe0_ready)
            {
                /* Sub-frame 0 cycle completed */
                logger->histogram_subframe0_ready = 1;
                PRINT_DEBUG("JSON histogram: Sub-frame 0 Default histogram ready\n");

                /* Check if sub-frame 1 is also ready */
                if (logger->histogram_subframe1_ready)
                {
                    logger->histogram_ready = 1;
                    PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames Default histograms ready\n");
                }
            }

            /* Check if sub-frame 1 cycle completed */
            if ((logger->histogram_subframes_received & subframe1Mask) == subframe1Mask &&
                !logger->histogram_subframe1_ready)
            {
                /* Sub-frame 1 cycle completed */
                logger->histogram_subframe1_ready = 1;
                PRINT_DEBUG("JSON histogram: Sub-frame 1 Default histogram ready\n");

                /* Check if sub-frame 0 is also ready */
                if (logger->histogram_subframe0_ready)
                {
                    logger->histogram_ready = 1;
                    PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames Default histograms ready\n");
                }
            }
        }
        else
        {
            /* Low resolution mode: single cycle */
            if ((logger->histogram_subframes_received & allFramesMask) == allFramesMask && !logger->histogram_ready)
            {
                logger->histogram_ready = 1;
                PRINT_DEBUG("JSON histogram: complete (all %d frames received, mask=0x%x)\n",
                       numFramesNeeded, allFramesMask);
            }
        }
    }
}
#endif /* ENABLE_HISTOGRAM */
