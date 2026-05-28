/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef TMF8829_JSON_H
#define TMF8829_JSON_H

#include <stdio.h>
#include <stdint.h>
#include <zlib.h>
#include <pthread.h>
#include <semaphore.h>
#include "tmf8829_frameparser.h"

#define JSON_MAX_FILE_PATH      256
#define JSON_LOGGER_VERSION     "1"
#define JSON_MAX_HISTO_BINS     256
#define JSON_MAX_MP_COUNT       1536  /* 48x32 = 1536 pixels max */
#define JSON_QUEUE_SIZE         64   /* Number of frames in write queue - increased for high throughput */

/**
 * @brief Frame data structure for queued JSON writes
 */
typedef struct json_frame_data
{
    tmf8829FrameData_t frame;
    tmf8829PixelResult_t *pixelResults;  /* Dynamically allocated pixel results array */
#if ENABLE_HISTOGRAM
    int histogram_ready;
    int histogram_ha_ready;  /* High accuracy histogram ready flag */
    tmf8829Histogram_t *histograms;      /* Dynamically allocated default/regular histograms */
    tmf8829Histogram_t *histogramsHA;    /* Dynamatically allocated high accuracy histograms */
#endif
    uint8_t resultFormat;
} json_frame_data_t;

/**
 * @brief JSON logger context
 */
typedef struct tmf8829_json_logger
{
    char file_path[JSON_MAX_FILE_PATH];
    gzFile file;
    int is_open;
    int frame_count;
    uint32_t serial_number;
    uint8_t fw_version[4];
    tmf8829_cfg_t cfg;
    
#if ENABLE_HISTOGRAM
    /* Histogram buffering - store histograms for current measurement cycle */
    int histogram_ready;                    /**< flag: histogram data is ready for next result */
    int histogram_ha_ready;                 /**< flag: high accuracy histogram data is ready */
    int histogram_subframes_received;       /**< bitmask of received histogram sub-frames */
    int histogram_subframes_received_ha;    /**< bitmask of received HA histogram sub-frames */
    int histogram_expected_subframes;       /**< expected bitmask (0x03 for 2 sub-frames, 0x0F for 4) */
    tmf8829Histogram_t *histograms;         /**< pointer to default/regular histogram buffer */
    tmf8829Histogram_t *histogramsHA;       /**< pointer to high accuracy histogram buffer */
    int dualMode;                           /**< Current dual mode setting */
    int dualCurrentPhase;                   /**< Current phase in dual mode: 0=HA, 1=Default */
    /* High resolution sub-frame tracking */
    int histogram_subframe0_ready;          /**< flag: sub-frame 0 histograms ready */
    int histogram_subframe1_ready;          /**< flag: sub-frame 1 histograms ready */
    int histogram_subframe0_ready_ha;       /**< flag: sub-frame 0 HA histograms ready */
    int histogram_subframe1_ready_ha;       /**< flag: sub-frame 1 HA histograms ready */
#endif
    
    /* Threading support */
    pthread_t writer_thread;                /**< JSON writer thread handle */
    pthread_mutex_t queue_mutex;           /**< mutex for queue access */
    pthread_cond_t queue_cond;              /**< condition variable for queue signaling */
    pthread_cond_t queue_full_cond;         /**< condition variable for queue full signaling */
    json_frame_data_t frame_queue[JSON_QUEUE_SIZE];  /**< circular buffer for frames */
    int queue_head;                         /**< head index of circular buffer */
    int queue_tail;                         /**< tail index of circular buffer */
    int queue_count;                        /**< number of frames in queue */
    volatile int thread_running;             /**< flag: thread should be running */
    volatile int stop_requested;            /**< flag: stop requested */
} tmf8829_json_logger_t;

/**
 * @brief Initialize the JSON logger
 * @param logger Pointer to the logger structure
 * @param prefix File name prefix (e.g., "tmf8829")
 * @param serial_number Device serial number
 * @param fw_version Firmware version (4 bytes: app_id, major, minor, patch)
 * @return 0 on success, -1 on failure
 */
int tmf8829_json_init(tmf8829_json_logger_t *logger, const char *prefix,
                      uint32_t serial_number, const uint8_t fw_version[4]);

/**
 * @brief Close the JSON logger and finalize the file
 * @param logger Pointer to the logger structure
 */
void tmf8829_json_close(tmf8829_json_logger_t *logger);

/**
 * @brief Save configuration for later writing
 * @param logger Pointer to the logger structure
 * @param cfg Pointer to configuration structure
 */
void tmf8829_json_save_config(tmf8829_json_logger_t *logger, tmf8829_cfg_t *cfg);

/**
 * @brief Write a parsed frame to JSON file
 * @param logger Pointer to the logger structure
 * @param parser Pointer to the frame parser with parsed data
 * @param resultFormat Result format configuration byte
 */
void tmf8829_json_write_frame(tmf8829_json_logger_t *logger, 
                               tmf8829FrameParser_t *parser,
                               uint8_t resultFormat);

/**
 * @brief Queue a frame for asynchronous JSON writing
 * @param logger Pointer to the logger structure
 * @param parser Pointer to the frame parser with parsed data
 * @param resultFormat Result format configuration byte
 * @return 0 on success, -1 if queue is full
 */
int tmf8829_json_queue_frame(tmf8829_json_logger_t *logger, 
                               tmf8829FrameParser_t *parser,
                               uint8_t resultFormat);

#if ENABLE_HISTOGRAM
/**
 * @brief Cache histogram sub-frame data for later JSON write (called for each histogram sub-frame)
 * @param logger Pointer to the logger structure
 * @param parser Pointer to the frame parser with parsed histogram data
 * @param subFrame Sub-frame number (0, 1, 2, etc.)
 */
void tmf8829_json_cache_histogram(tmf8829_json_logger_t *logger,
                                   tmf8829FrameParser_t *parser,
                                   int subFrame);
#endif

/**
 * @brief Flush and finalize JSON file
 * @param logger Pointer to the logger structure
 */
void tmf8829_json_flush(tmf8829_json_logger_t *logger);

/**
 * @brief Get current timestamp string
 * @param buffer Buffer to store timestamp string
 * @param size Buffer size
 */
void tmf8829_json_get_timestamp(char *buffer, int size);

#endif /* TMF8829_JSON_H */
