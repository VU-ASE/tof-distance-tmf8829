/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef FRAMEPARSER_H
#define FRAMEPARSER_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/types.h>
#include <stdint.h>

/* ============================================================================
 * Compile-time Configuration
 * ============================================================================ */

/**
 * ENABLE_HISTOGRAM - Enable histogram data parsing and storage
 * 
 * When enabled, histogram data is parsed and stored in memory.
 * When disabled, histogram frames are skipped to save memory:
 *   - Saves ~310KB for histogram accumulation buffer
 *   - Saves ~300KB for histogram storage (48x32 mode max)
 *   - Total savings: ~600KB RAM
 * 
 * To enable: compile with -DENABLE_HISTOGRAM or define in Makefile
 */
#ifdef ENABLE_HISTOGRAM
  /* Feature is enabled */
#else
  #define ENABLE_HISTOGRAM 0
#endif

/* Frame sizes */
#define TMF8829_MAX_PEAKS           4
#define TMF8829_MAX_ROWS            32  /* Max rows (Y direction) */
#define TMF8829_MAX_COLS            48  /* Max columns (X direction) */
#if ENABLE_HISTOGRAM
#define TMF8829_HISTOGRAM_BIN_SIZE_8x8    256  /* Number of bins per histogram for 8x8 mode */
#define TMF8829_HISTOGRAM_BIN_SIZE_OTHER  64   /* Number of bins per histogram for other modes */
#endif

/* Frame status flags */
#define TMF8829_FRAME_VALID                     0x01
#define TMF8829_FRAME_ABORTED                   0xC0
#define TMF8829_FRAME_WARNING_HV_CP_OVERLOAD    0x08
#define TMF8829_FRAME_WARNING_VCDRV_OVERLOAD    0x10
#define TMF8829_FRAME_WARNING_VCDRV_BURST_EXCEEDED 0x20

/**
 * @brief Frame header structure - 16 bytes
 * See TMF8829_Host_Driver_Communication_AN001096 Section 4.6.2
 */
typedef struct __attribute__ ((packed)) tmf8829FrameHeader
{
    uint8_t  id;                    /**< frame identifier: 0x10 for result, 0x20 for histogram */
    uint8_t  layout;                /**< result: result format; histogram: sub-frame number */
    uint16_t payload;               /**< length of frame in bytes (excluding id, layout and payload field) */
    uint32_t fNumber;               /**< frame number (running index) */
    int8_t   temperature[3];        /**< temperature in degree celsius from 3 sensors */
    uint8_t  bdv;                   /**< BDV value (internal, ignore) */
    uint16_t refPos0;               /**< optical reference peak position 1st measurement */
    uint16_t refPos1;               /**< optical reference peak position last measurement */
} tmf8829FrameHeader_t;

/**
 * @brief Frame footer structure - 12 bytes
 * See TMF8829_Host_Driver_Communication_AN001096 Section 4.6.3
 */
typedef struct __attribute__ ((packed)) tmf8829FrameFooter
{
    uint32_t t0Integration;         /**< internal timestamp when t0 integration started */
    uint32_t t1Integration;         /**< internal timestamp when t-last integration started */
    uint8_t  frameStatus;           /**< frame validity and warning flags */
    uint8_t  reserved;              /**< reserved */
    uint16_t eof;                   /**< end of frame marker (0xE0F7) */
} tmf8829FrameFooter_t;

/**
 * @brief Single peak data structure
 */
typedef struct tmf8829Peak
{
    uint16_t distance;              /**< distance in mm */
    uint8_t  snr;                   /**< signal-to-noise ratio / confidence */
    uint16_t signal;                /**< signal strength (optional) */
    float    x;                     /**< x coordinate in mm */
    float    y;                     /**< y coordinate in mm */
    float    z;                     /**< z coordinate in mm */
} tmf8829Peak_t;

/**
 * @brief Single pixel result data
 */
typedef struct tmf8829PixelResult
{
    uint16_t noise;                 /**< noise strength (optional) */
    uint16_t xtalk;                 /**< crosstalk (optional) */
    uint8_t  numPeaks;              /**< number of valid peaks */
    tmf8829Peak_t peaks[TMF8829_MAX_PEAKS]; /**< peak data array */
} tmf8829PixelResult_t;

/**
 * @brief Single histogram data
 * Each histogram contains up to 256 bin values (8x8 mode uses 256, others use 64)
 */
#if ENABLE_HISTOGRAM
typedef struct tmf8829Histogram
{
    uint32_t bin[TMF8829_HISTOGRAM_BIN_SIZE_8x8]; /**< histogram bin values */
} tmf8829Histogram_t;
#endif

/**
 * @brief Parsed frame data structure
 * Holds the parsed information from a result frame
 */
typedef struct tmf8829FrameData
{
    /* Frame identification */
    uint8_t  frameType;             /**< frame type: TMF8829_FID_RESULTS or TMF8829_FID_HISTOGRAMS */
    uint8_t  fpMode;                /**< focal plane mode (0-5) */
    uint8_t  layout;                /**< layout byte: result format or histogram sub-frame number */
    
    /* Frame header info */
    uint32_t frameNumber;           /**< frame number */
    uint32_t systick;               /**< systick from pre-header */
    int8_t   temperature;           /**< temperature from sensor 2 (or average) */
    
    /* Frame footer info */
    uint32_t t0Integration;         /**< t0 integration timestamp */
    uint32_t t1Integration;         /**< t1 integration timestamp */
    uint8_t  frameStatus;           /**< frame status flags */
    uint8_t  warnings;              /**< warning flags extracted from status */
    
    /* Result data dimensions */
    int      numRows;               /**< number of rows (Y direction) */
    int      numCols;               /**< number of columns (X direction) */
    
    /* Parsed pixel results - stored as 2D array [row][col] */
    tmf8829PixelResult_t *pixelResults; /**< pointer to pixel results array */
    
#if ENABLE_HISTOGRAM
    /* Histogram data */
    int      hasHistogram;          /**< flag indicating histogram data is available */
    int      histogramSubFrame;     /**< histogram sub-frame number (0 or 1) */
    tmf8829Histogram_t *histograms; /**< pointer to histogram array [row][col] */
#endif
} tmf8829FrameData_t;

/* Maximum histogram data buffer size */
/* 48x32 mode (max): 32 rows * 48 cols * 64 bins * 3 bytes + 4 ref pixels * 64 bins * 3 bytes = 294912 + 768 = 295680 bytes */
/* Also need to account for frame footer (12 bytes) */
#if ENABLE_HISTOGRAM
#define TMF8829_HISTOGRAM_BUFFER_SIZE  310000
#endif

/**
 * @brief Frame parser context
 * Maintains state for frame parsing and buffering
 */
typedef struct tmf8829FrameParser
{
    /* Current frame state */
    int      state;                 /**< parser state */
    int      dataLen;               /**< current data length in buffer */
    
    /* Sub-frame handling for high resolution modes */
    int      expectedSubFrames;     /**< expected number of sub-frames */
    int      currentSubFrame;       /**< current sub-frame index */
    uint8_t  subFrameReceived;      /**< bitmask of received sub-frames */

    /* Dual mode tracking */
    int      dualMode;                 /**< dual mode setting: 0=disabled, 1=regular, 2=long-range */
    int      dualCurrentPhase;         /**< current phase in dual mode: 0=HA phase, 1=Default phase */
    int      dualHistoCount;           /**< histogram count in current dual mode phase */
    int      isHAHistogram;            /**< flag: current histogram frame is HA histogram (dual mode only) */

    /* Parsed frame data */
    tmf8829FrameData_t frame;       /**< current frame data */

    /* Pixel result storage (pre-allocated) */
    tmf8829PixelResult_t pixelResults[TMF8829_MAX_ROWS * TMF8829_MAX_COLS];

    /* Result data accumulation buffer */
    uint8_t *resultBuffer;          /**< buffer for accumulating result data */
    int      resultBufferAllocated; /**< size of allocated result buffer */

    /* Result sub-frame tracking for high resolution modes */
    int      resultSubFrameReceived;    /**< bitmask of received result sub-frames */
    int      resultReady;               /**< flag: result data is complete */
    uint32_t lastFrameNumber;          /**< last valid frame number for continuity check */

    /* FPS statistics */
    uint32_t fpsFirstSystick;           /**< systick of first complete result frame */
    uint32_t fpsLastSystick;            /**< systick of last complete result frame */
    int      fpsFrameCount;            /**< count of complete result frames */

    /* Crosstalk statistics */
    uint16_t maxXtalk;                  /**< maximum crosstalk value in current frame */
    float    xtalkPercentage;           /**< max crosstalk as percentage */

    /* FOV correction (bits 0-1: X correction, bits 2-3: Y correction) */
    uint8_t  fovCorrection;              /**< field of view correction value */

#ifdef ENABLE_KEYSTONE
    /* Keystone angle calculation */
    int      keystoneEnabled;           /**< flag: keystone angle calculation enabled */
    double   keystoneAngleX;           /**< calculated X angle in degrees */
    double   keystoneAngleY;           /**< calculated Y angle in degrees */
    double   keystoneAngleZ;           /**< calculated Z angle in degrees */
#endif

#if ENABLE_HISTOGRAM
    /* Histogram storage for dual mode (dynamically allocated when needed) */
    tmf8829Histogram_t *histograms;      /**< pointer to default/regular histogram array */
    tmf8829Histogram_t *histogramsHA;    /**< pointer to high accuracy histogram array */
    int      histogramsAllocated;        /**< number of allocated histograms */
    int      histogramsHAAllocated;      /**< number of allocated HA histograms */

    /* Histogram data accumulation buffer (dynamically allocated) */
    uint8_t *histogramBuffer;       /**< buffer for accumulating histogram data */
    int      histogramBufferAllocated; /**< size of allocated histogram buffer */
    int      histogramDataLen;      /**< current length of accumulated histogram data */
#endif
} tmf8829FrameParser_t;

/**
 * @brief Configuration structure
 */
typedef struct tmf8829_cfg
{
    int conf_threshold;
    int fpMode;
    int deadtime;
    int period;
    int resultFormat;
    int iteration;
    int shortIteration;
    int histogram_dump;
    int dualMode;
} tmf8829_cfg_t;

/* ============================================================================
 * Frame Parser Functions
 * ============================================================================ */

/**
 * @brief Initialize the frame parser
 * @param parser Pointer to parser structure
 */
void tmf8829FrameParserInit(tmf8829FrameParser_t *parser);

/**
 * @brief Cleanup frame parser resources
 * @param parser Pointer to parser structure
 */
void tmf8829FrameParserCleanup(tmf8829FrameParser_t *parser);

/**
 * @brief Parse frame header from raw data
 * @param parser Pointer to parser structure
 * @param data Raw frame data (including pre-header)
 * @return 0 on success, -1 on error
 */
int tmf8829ParseFrameHeader(tmf8829FrameParser_t *parser, uint8_t *data);

/**
 * @brief Parse frame footer from raw data
 * @param parser Pointer to parser structure
 * @param data Raw frame data
 * @param totalLen Total frame length
 * @return 0 on success, -1 on error
 */
int tmf8829ParseFrameFooter(tmf8829FrameParser_t *parser, uint8_t *data, int totalLen);

/**
 * @brief Parse result frame pixel data
 * @param parser Pointer to parser structure
 * @param data Raw frame data
 * @param resultFormat Result format configuration
 * @return 0 on success, -1 on error
 */
int tmf8829ParseResultData(tmf8829FrameParser_t *parser, uint8_t *data, uint8_t resultFormat);

#if ENABLE_HISTOGRAM
/**
 * @brief Parse histogram frame data
 * @param parser Pointer to parser structure
 * @param data Raw frame data
 * @param subFrame Sub-frame number (0 or 1)
 * @return 0 on success, -1 on error
 */
int tmf8829ParseHistogramData(tmf8829FrameParser_t *parser, uint8_t *data, int subFrame);
#endif

/**
 * @brief Get resolution for a given focal plane mode
 * @param fpMode Focal plane mode
 * @param rows Pointer to store row count
 * @param cols Pointer to store column count
 */
void tmf8829GetResolution(int fpMode, int *rows, int *cols);

/**
 * @brief Calculate pixel data size from result format
 * @param resultFormat Result format byte
 * @return Size in bytes per pixel
 */
int tmf8829GetPixelDataSize(uint8_t resultFormat);

/**
 * @brief Get the number of histogram frames between two result frames
 * @param fpMode Focal plane mode
 * @param dualMode Dual mode setting (0=disabled, 1=regular, 2=long-range)
 * @return Total number of histogram frames before a result frame
 */
int tmf8829GetHistogramsPerResult(int fpMode, int dualMode);

/* ============================================================================
 * Frame Handler Callbacks (called from tmf8829.c)
 * ============================================================================ */

/**
 * @brief Handle received frame header data (callback from driver)
 * @param dptr Pointer to chip structure
 * @param data Raw data buffer
 */
void handleReceivedFrameHeaderData(void *dptr, uint8_t *data);

/**
 * @brief Handle received result data chunk (callback from driver)
 * @param dptr Pointer to chip structure
 * @param data Data chunk
 * @param size Chunk size
 */
void handleReceivedResultData(void *dptr, uint8_t *data, uint16_t size);

/**
 * @brief Handle received histogram data chunk (callback from driver)
 * @param dptr Pointer to chip structure
 * @param data Data chunk
 * @param size Chunk size
 * @note When ENABLE_HISTOGRAM is 0, this function does nothing
 */
void handleReceivedHistogramData(void *dptr, uint8_t *data, uint16_t size);

/**
 * @brief Handle end of result data reception (callback from driver)
 * @param dptr Pointer to chip structure
 */
void handleReceivedResultDataEnd(void *dptr);

/**
 * @brief Handle end of histogram data reception (callback from driver)
 * @param dptr Pointer to chip structure
 * @note When ENABLE_HISTOGRAM is 0, this function does nothing
 */
void handleReceivedHistogramDataEnd(void *dptr);

/* ============================================================================
 * Print and Output Functions
 * ============================================================================ */

/**
 * @brief Print parsed frame results
 * @param parser Pointer to parser structure
 */
void tmf8829PrintFrameResults(tmf8829FrameParser_t *parser);

/**
 * @brief Print frame info
 * @param parser Pointer to parser structure
 */
void tmf8829PrintFrameInfo(tmf8829FrameParser_t *parser);

/**
 * @brief Print FPS statistics
 * @param parser Pointer to parser structure
 */
void tmf8829PrintFpsStats(tmf8829FrameParser_t *parser);

#endif /* FRAMEPARSER_H */
