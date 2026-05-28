/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef TMF8829_KEYSTONE_H
#define TMF8829_KEYSTONE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Keystone configuration */
#define KEYSTONE_MAX_ZONE_NUMBER        (48*32)     /* Maximum number of zones (pixels) */
#define KEYSTONE_LINEAR_NUMBER          2           /* Number of linear dimensions (X,Y) */
#define KEYSTONE_PI                     (3.14159265)
#define KEYSTONE_DENOISE_AVERAGE_COUNT  5           /* Number of frames for averaging */
#define KEYSTONE_DENOISE_PEAK_COUNT     1           /* Number of peaks to remove for denoising */
#define KEYSTONE_MAX_FIT_DATA           (48*32)     /* Maximum number of points for plane fitting */

/* Structure for pixel information */
typedef struct keystonePixelInfo {
    int channel;            /* Pixel channel index */
    int confidence;         /* Pixel confidence */
    int distance_mm;        /* Distance in mm */
    float x;                /* X coordinate in mm */
    float y;                /* Y coordinate in mm */
    float z;                /* Z coordinate in mm */
} keystonePixelInfo_t;

/* Structure for keystone calculation context */
typedef struct keystoneContext {
    /* Configuration */
    int enableDenoise;      /* Enable distance denoising */
    int snrThreshold;       /* SNR threshold for all-zones mode */

    /* Current state */
    int rows;               /* Number of rows in current frame */
    int cols;               /* Number of columns in current frame */
    int frameSize;          /* Current frame size (rows * cols) */

    /* Denoising buffers */
    double fitRawDataX[KEYSTONE_MAX_FIT_DATA][KEYSTONE_DENOISE_AVERAGE_COUNT];
    double fitRawDataY[KEYSTONE_MAX_FIT_DATA][KEYSTONE_DENOISE_AVERAGE_COUNT];
    double fitRawDataZ[KEYSTONE_MAX_FIT_DATA][KEYSTONE_DENOISE_AVERAGE_COUNT];
    int denoiseIndex;       /* Current index in denoising buffer */

    /* Zone selection mode */
    int useZoneBits;        /* 0=use all zones with SNR filter, 1=use predefined static pattern */

    /* Pixel array pointers for different resolutions */
    const int *pixelArray8x8;     /* Pointer to static array */
    const int *pixelArray16x16;
    const int *pixelArray32x32;
    const int *pixelArray48x32;

    /* Last calculated angles */
    double angles[3];       /* [0]=angleX, [1]=angleY, [2]=angleZ */
} keystoneContext_t;

/**
 * @brief Initialize keystone context
 * @param ctx Pointer to keystone context
 * @return 0 on success, -1 on error
 */
int keystoneInit(keystoneContext_t* ctx);

/**
 * @brief Clean up keystone context
 * @param ctx Pointer to keystone context
 */
void keystoneCleanup(keystoneContext_t* ctx);

/**
 * @brief Set zone pattern mode
 * @param ctx Pointer to keystone context
 * @param useZonePattern If true, use predefined static pixel_array_xxx pattern;
 *                       If false (default), use all zones with SNR filtering
 * @return 0 on success, -1 on error
 */
int keystoneSetZonePattern(keystoneContext_t* ctx, bool useZonePattern);

/**
 * @brief Process frame data and calculate angles
 * @param ctx Pointer to keystone context
 * @param rows Number of rows
 * @param cols Number of columns
 * @param xData X coordinate array [row*col]
 * @param yData Y coordinate array [row*col]
 * @param zData Z coordinate array [row*col]
 * @param confidenceData Confidence array [row*col] (can be NULL)
 * @return 0 on success, -1 on error
 */
int keystoneProcessFrame(keystoneContext_t* ctx, int rows, int cols,
                         float* xData, float* yData, float* zData,
                         int* confidenceData);

/**
 * @brief Get calculated angles
 * @param ctx Pointer to keystone context
 * @param angleX Output: X angle in degrees
 * @param angleY Output: Y angle in degrees
 * @param angleZ Output: Z angle in degrees (optional, can be NULL)
 */
void keystoneGetAngles(keystoneContext_t* ctx, double* angleX, double* angleY, double* angleZ);

/**
 * @brief Set denoising mode
 * @param ctx Pointer to keystone context
 * @param enable 1 to enable denoising, 0 to disable
 */
void keystoneSetDenoise(keystoneContext_t* ctx, int enable);

#ifdef __cplusplus
}
#endif

#endif /* TMF8829_KEYSTONE_H */