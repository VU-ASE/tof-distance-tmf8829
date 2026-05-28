/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#include "tmf8829_keystone.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* Predefined zone patterns for different resolutions */
static const int pixel_array_8x8[] =
{
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 1, 1, 0, 1, 0,
    0, 1, 0, 1, 1, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
};


static const int pixel_array_16x16[] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};


static const int pixel_array_32x32[] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};


static const int pixel_array_48x32[] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};


/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Cholesky decomposition for linear regression
 */
static int cholesky(double a[], int n, int m, double d[])
{
    int i, j, k, u, v;
    
    if ((a[0] + 1.0 == 1.0) || (a[0] < 0.0)) {
        return -1;
    }
    a[0] = sqrt(a[0]);
    for (j = 1; j <= n - 1; j++)
        a[j] = a[j] / a[0];
    for (i = 1; i <= n - 1; i++) {
        u = i * n + i;
        for (j = 1; j <= i; j++) {
            v = (j - 1) * n + i;
            a[u] = a[u] - a[v] * a[v];
        }
        if ((a[u] + 1.0 == 1.0) || (a[u] < 0.0)) {
            return -2;
        }
        a[u] = sqrt(a[u]);
        if (i != (n - 1)) {
            for (j = i + 1; j <= n - 1; j++) {
                v = i * n + j;
                for (k = 1; k <= i; k++)
                    a[v] = a[v] - a[(k - 1) * n + i] * a[(k - 1) * n + j];
                a[v] = a[v] / a[u];
            }
        }
    }
    for (j = 0; j <= m - 1; j++) {
        d[j] = d[j] / a[0];
        for (i = 1; i <= n - 1; i++) {
            u = i * n + i;
            v = i * m + j;
            for (k = 1; k <= i; k++)
                d[v] = d[v] - a[(k - 1) * n + i] * d[(k - 1) * m + j];
            d[v] = d[v] / a[u];
        }
    }
    for (j = 0; j <= m - 1; j++) {
        u = (n - 1) * m + j;
        d[u] = d[u] / a[n * n - 1];
        for (k = n - 1; k >= 1; k--) {
            u = (k - 1) * m + j;
            for (i = k; i <= n - 1; i++) {
                v = (k - 1) * n + i;
                d[u] = d[u] - a[v] * d[i * m + j];
            }
            v = (k - 1) * n + k - 1;
            d[u] = d[u] / a[v];
        }
    }
    return 0;
}

/**
 * @brief Linear regression using ordinary least squares
 */
static void linearRegression(double x[][KEYSTONE_MAX_ZONE_NUMBER], double y[], 
                                  int m, int n, double a[], double dt[], double v[])
{
    int i, j, k, mm;
    double q, e, u, p, yy, s, r, pp;
    double b[50] = { 0.0 };
    
    mm = m + 1;
    b[mm * mm - 1] = n;
    for (j = 0; j <= m - 1; j++) {
        p = 0.0;
        for (i = 0; i <= n - 1; i++)
            p = p + x[j][i];
        b[m * mm + j] = p;
        b[j * mm + m] = p;
    }
    for (i = 0; i <= m - 1; i++)
        for (j = i; j <= m - 1; j++) {
            p = 0.0;
            for (k = 0; k <= n - 1; k++)
                p = p + x[i][k] * x[j][k];
            b[j * mm + i] = p;
            b[i * mm + j] = p;
        }
    a[m] = 0.0;
    for (i = 0; i <= n - 1; i++)
        a[m] = a[m] + y[i];
    for (i = 0; i <= m - 1; i++) {
        a[i] = 0.0;
        for (j = 0; j <= n - 1; j++)
            a[i] = a[i] + x[i][j] * y[j];
    }
    
    cholesky(b, mm, 1, a);
    
    yy = 0.0;
    for (i = 0; i <= n - 1; i++)
        yy = yy + y[i] / n;
    q = 0.0;
    e = 0.0;
    u = 0.0;
    for (i = 0; i <= n - 1; i++) {
        p = a[m];
        for (j = 0; j <= m - 1; j++)
            p = p + a[j] * x[j][i];
        q = q + (y[i] - p) * (y[i] - p);
        e = e + (y[i] - yy) * (y[i] - yy);
        u = u + (yy - p) * (yy - p);
    }
    s = sqrt(q / n);
    r = sqrt(1.0 - q / e);
    for (j = 0; j <= m - 1; j++) {
        p = 0.0;
        for (i = 0; i <= n - 1; i++) {
            pp = a[m];
            for (k = 0; k <= m - 1; k++)
                if (k != j)
                    pp = pp + a[k] * x[k][i];
            p = p + (y[i] - pp) * (y[i] - pp);
        }
        v[j] = sqrt(1.0 - q / p);
    }
    dt[0] = q;
    dt[1] = s;
    dt[2] = r;
    dt[3] = u;
}

/**
 * @brief Calculate vector infinity norm (Euclidean norm)
 */
static double vectorNorm(double x[], int n)
{
    double b = 0.0;
    int i;
    
    for(i = 0; i < n; i++) {
        b = b + (x[i] * x[i]);
    }
    
    b = sqrt(b);
    
    return b;
}

/**
 * @brief Calculate dot product of two vectors
 */
static double vectorDot(double a[], double b[], int n)
{
    int i;
    double sum = 0.0;
    
    for (i = 0; i < n; i++)
        sum += a[i] * b[i];
    
    return sum;
}

/**
 * @brief Calculate plane angles from plane normal vector
 */
static void calcPlaneAngle(double plane_n[KEYSTONE_LINEAR_NUMBER+1], 
                                double angles[KEYSTONE_LINEAR_NUMBER+1])
{
    int vec_len = KEYSTONE_LINEAR_NUMBER+1;
    double base_vectors[3][3] = {
        { 1.000, 0.000, 0.000 },
        { 0.000, 1.000, 0.000 },
        { 0.000, 0.000, 1.000 },
    };
    
    angles[0] = 90 - acos(vectorDot(plane_n, base_vectors[0], vec_len) /
                          (vectorNorm(plane_n, vec_len) *
                           vectorNorm(base_vectors[0], vec_len))) * (180 / KEYSTONE_PI);
    angles[1] = acos(vectorDot(plane_n, base_vectors[1], vec_len) /
                     (vectorNorm(plane_n, vec_len) *
                      vectorNorm(base_vectors[1], vec_len))) * (180 / KEYSTONE_PI) - 90;
    angles[2] = 90 - acos(vectorDot(plane_n, base_vectors[2], vec_len) /
                          (vectorNorm(plane_n, vec_len) *
                           vectorNorm(base_vectors[2], vec_len))) * (180 / KEYSTONE_PI);
}

/**
 * @brief Insertion sort for denoising
 */
static void insertSort(double* arr, int sz)
{
    int i = 0;
    for (i = 0; i < sz - 1; i++)
    {
        int end = i;
        double tmp = arr[end + 1];
        while (end >= 0)
        {
            if (arr[end] > tmp)
            {
                arr[end + 1] = arr[end];
                end--;
            }
            else
            {
                break;
            }
        }
        
        arr[end+1] = tmp;
    }
}

/**
 * @brief Get denoised distance value
 */
static double denoiseGetDistance(double raw_distances[], int len)
{
    double distance_sum = 0;
    int i;
    int size = 0;
    double distances[KEYSTONE_DENOISE_AVERAGE_COUNT];
    double result = 0.0;
    
    if (len > KEYSTONE_DENOISE_AVERAGE_COUNT)
        len = KEYSTONE_DENOISE_AVERAGE_COUNT;
    
    memset(distances, 0, sizeof(distances));
    
    /* Debug: print input data */
    // printf("[DENOISE] Input len=%d: ", len);
    // for (i = 0; i < len; i++) {
    //     printf("%.2f ", raw_distances[i]);
    // }
    // printf("\n");
    
    for (i = 0; i < len; i++)
    {
        if (raw_distances[i] != 0)
        {
            distances[size] = raw_distances[i];
            size++;
        }
    }
    
    // printf("[DENOISE] Non-zero count: size=%d\n", size);
    
    /* Not enough valid data - return 0 to wait for more frames */
    if (size <= (KEYSTONE_DENOISE_PEAK_COUNT * 2))
    {
        // printf("[DENOISE] Returning 0 (not enough data: %d < %d)\n",
        //        size, (KEYSTONE_DENOISE_PEAK_COUNT * 2));
        return 0;
    }
    
    
    insertSort(distances, size);
    
    // printf("[DENOISE] After sorting: ");
    // for (i = 0; i < size; i++) {
    //     printf("%.2f ", distances[i]);
    // }
    // printf("\n");
    
    if (size <= (KEYSTONE_DENOISE_PEAK_COUNT * 2))
    {
        for (i = 0; i < size; i++)
            distance_sum = distance_sum + distances[i];
        
        result = distance_sum/size;
        // printf("[DENOISE] Returning sorted average: %.2f\n", result);
        return result;
    }
    
    for (i = KEYSTONE_DENOISE_PEAK_COUNT; i < (size - KEYSTONE_DENOISE_PEAK_COUNT); i++)
        distance_sum = distance_sum + distances[i];
    
    result = distance_sum/(size - KEYSTONE_DENOISE_PEAK_COUNT * 2);
    // printf("[DENOISE] Returning trimmed average: %.2f (removed %d values from each end)\n",
    //        result, KEYSTONE_DENOISE_PEAK_COUNT);
    return result;
}

/**
 * @brief Perform plane fitting on point data
 */
static void doPlaneFit(keystonePixelInfo_t avg_data[], double angles[], int zone_number)
{
    int i;
    int valid_num = 0;
    double coef[KEYSTONE_LINEAR_NUMBER+1] = {0.0, 0, 0};
    double plane_n[KEYSTONE_LINEAR_NUMBER+1] = {0.0};
    double v[3] = {0.0};
    double dt[4] = {0.0};
    double coef_sqrt = 0.0;
    double x[KEYSTONE_MAX_ZONE_NUMBER], y[KEYSTONE_MAX_ZONE_NUMBER], z[KEYSTONE_MAX_ZONE_NUMBER];
    double xy[KEYSTONE_LINEAR_NUMBER][KEYSTONE_MAX_ZONE_NUMBER], zz[KEYSTONE_MAX_ZONE_NUMBER];
    
    // printf("[PLANEFIT] zone_number=%d\n", zone_number);
    
    memset(x, 0, sizeof(x));
    memset(y, 0, sizeof(y));
    memset(z, 0, sizeof(z));
    
    /* Filter invalid data */
    for (i = 0; i < zone_number; i++)
    {
        if (avg_data[i].x == 0 && avg_data[i].y == 0 && avg_data[i].z == 0) {
            // printf("[PLANEFIT] Point %d skipped (x=y=z=0)\n", i);
            continue;
        }
        
        // printf("[PLANEFIT] Point %d: x=%.2f, y=%.2f, z=%.2f\n",
        //        i, avg_data[i].x, avg_data[i].y, avg_data[i].z);
        
        xy[0][valid_num] = avg_data[i].x;
        xy[1][valid_num] = avg_data[i].y;
        zz[valid_num] = avg_data[i].z;
        valid_num++;
    }
    
    // printf("[PLANEFIT] valid_num=%d\n", valid_num);
    
    if (valid_num < 3) {
        /* Not enough points for plane fitting */
        // printf("[PLANEFIT] ERROR: Not enough valid points (< 3), setting angles to 0\n");
        angles[0] = angles[1] = angles[2] = 0.0;
        return;
    }
    
    linearRegression(xy, zz, KEYSTONE_LINEAR_NUMBER, valid_num, coef, dt, v);
    
    coef_sqrt = sqrt(coef[0] * coef[0] + coef[1] * coef[1] + 1);
    
    plane_n[0] = -1 * coef[0] / coef_sqrt;
    plane_n[1] = -1 * coef[1] / coef_sqrt;
    plane_n[2] = 1 / coef_sqrt;
    
    calcPlaneAngle(plane_n, angles);
}

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

int keystoneInit(keystoneContext_t* ctx)
{
    if (ctx == NULL) {
        return -1;
    }

    memset(ctx, 0, sizeof(keystoneContext_t));

    /* Point to predefined static patterns */
    ctx->pixelArray8x8 = pixel_array_8x8;
    ctx->pixelArray16x16 = pixel_array_16x16;
    ctx->pixelArray32x32 = pixel_array_32x32;
    ctx->pixelArray48x32 = pixel_array_48x32;

    /* Default: use all zones with SNR filtering */
    ctx->useZoneBits = 0;
    ctx->snrThreshold = 20;
    ctx->enableDenoise = 0;

    return 0;
}

void keystoneCleanup(keystoneContext_t* ctx)
{
    if (ctx != NULL) {
        /* Currently no dynamic allocation in context */
    }
}

int keystoneSetZonePattern(keystoneContext_t* ctx, bool useZonePattern)
{
    if (ctx == NULL) {
        return -1;
    }

    if (useZonePattern) {
        /* Use predefined static zone pattern */
        ctx->useZoneBits = 1;
    } else {
        /* Use all zones with SNR filtering (default) */
        ctx->useZoneBits = 0;
    }

    return 0;
}

int keystoneProcessFrame(keystoneContext_t* ctx, int rows, int cols,
                         float* xData, float* yData, float* zData,
                         int* confidenceData)
{
    keystonePixelInfo_t fit_data[KEYSTONE_MAX_FIT_DATA];
    int size = rows * cols;
    const int *array_ptr = NULL;
    int index = 0;
    int number = 0;
    int i, j;

    // printf("[KEYSTONE] ProcessFrame: rows=%d, cols=%d, size=%d, denoise=%d, useZoneBits=%d\n",
    //        rows, cols, size, ctx->enableDenoise, ctx->useZoneBits);

    if (ctx == NULL || xData == NULL || yData == NULL || zData == NULL) {
        // printf("[KEYSTONE] ERROR: NULL parameter\n");
        return -1;
    }

    /* Select zone pattern (only used when not using all zones) */
    if (size == 64) {
        array_ptr = ctx->pixelArray8x8;
    } else if (size == 256) {
        array_ptr = ctx->pixelArray16x16;
    } else if (size == 1024) {
        array_ptr = ctx->pixelArray32x32;
    } else if (size == 1536) {
        array_ptr = ctx->pixelArray48x32;
    }

    /* Check if frame size changed, reset buffers if needed */
    if (ctx->frameSize != size) {
        memset(ctx->fitRawDataX, 0, sizeof(ctx->fitRawDataX));
        memset(ctx->fitRawDataY, 0, sizeof(ctx->fitRawDataY));
        memset(ctx->fitRawDataZ, 0, sizeof(ctx->fitRawDataZ));
        ctx->denoiseIndex = 0;
        ctx->frameSize = size;
    }

    ctx->rows = rows;
    ctx->cols = cols;

    /* Collect data for denoising */
    number = 0;
    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < cols; j++)
        {
            int skip = 0;

            if (!ctx->useZoneBits) {
                /* All-zones mode: use any non-zero distance pixel with sufficient SNR */
                /* Check if distance is valid (non-zero) */
                if (xData[index] == 0 && yData[index] == 0 && zData[index] == 0) {
                    skip = 1;
                }
                /* Check SNR threshold if confidence data is available */
                if (!skip && confidenceData != NULL && confidenceData[index] < ctx->snrThreshold) {
                    skip = 1;
                }
            } else {
                /* Zone-pattern mode: only pixels selected by the zone pattern */
                if (!array_ptr[index]) {
                    skip = 1;
                }
            }

            if (!skip) {
                ctx->fitRawDataX[number][ctx->denoiseIndex] = xData[index];
                ctx->fitRawDataY[number][ctx->denoiseIndex] = yData[index];
                ctx->fitRawDataZ[number][ctx->denoiseIndex] = zData[index];
                number++;
            }

            if (number >= KEYSTONE_MAX_FIT_DATA)
                break;
            index++;
        }
        if (number >= KEYSTONE_MAX_FIT_DATA)
            break;
    }

    ctx->denoiseIndex++;
    if (ctx->denoiseIndex >= KEYSTONE_DENOISE_AVERAGE_COUNT)
        ctx->denoiseIndex = 0;

    /* Prepare fit data */
    number = 0;
    index = 0;
    int number_in_zone = 0;  /* Count of points in zone pattern */

    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < cols; j++)
        {
            int skip = 0;

            if (!ctx->useZoneBits) {
                /* All-zones mode: use any non-zero distance pixel with sufficient SNR */
                if (xData[index] == 0 && yData[index] == 0 && zData[index] == 0) {
                    skip = 1;
                }
                if (!skip && confidenceData != NULL && confidenceData[index] < ctx->snrThreshold) {
                    skip = 1;
                }
            } else {
                /* Zone-pattern mode: only pixels selected by the zone pattern */
                if (!array_ptr[index]) {
                    skip = 1;
                }
            }

            if (!skip) {
                number_in_zone++;

                if (ctx->enableDenoise) {
                    double denoisedX = denoiseGetDistance(ctx->fitRawDataX[number],
                                                               KEYSTONE_DENOISE_AVERAGE_COUNT);
                    double denoisedY = denoiseGetDistance(ctx->fitRawDataY[number],
                                                               KEYSTONE_DENOISE_AVERAGE_COUNT);
                    double denoisedZ = denoiseGetDistance(ctx->fitRawDataZ[number],
                                                               KEYSTONE_DENOISE_AVERAGE_COUNT);

                    /* Fallback: if denoising returned 0 (not enough data yet), use current frame data */
                    if (denoisedX == 0 && denoisedY == 0 && denoisedZ == 0) {
                        // printf("[KEYSTONE] Point %d: denoising not ready, using current frame data\n", number);
                        fit_data[number].x = xData[index];
                        fit_data[number].y = yData[index];
                        fit_data[number].z = zData[index];
                    } else {
                        fit_data[number].x = denoisedX;
                        fit_data[number].y = denoisedY;
                        fit_data[number].z = denoisedZ;
                    }
                } else {
                    fit_data[number].x = xData[index];
                    fit_data[number].y = yData[index];
                    fit_data[number].z = zData[index];
                }

                if (confidenceData != NULL) {
                    fit_data[number].confidence = confidenceData[index];
                }
                fit_data[number].channel = number;
                number++;
            }

            if (number >= KEYSTONE_MAX_FIT_DATA)
                break;
            index++;
        }
        if (number >= KEYSTONE_MAX_FIT_DATA)
            break;
    }

    // printf("[KEYSTONE] number_in_zone=%d, valid_fit_data=%d\n", number_in_zone, number);

    /* Perform plane fitting and calculate angles */
    memset(ctx->angles, 0, sizeof(ctx->angles));
    doPlaneFit(fit_data, ctx->angles, number);

    return 0;
}

void keystoneGetAngles(keystoneContext_t* ctx, double* angleX, double* angleY, double* angleZ)
{
    if (ctx == NULL || angleX == NULL || angleY == NULL) {
        // printf("[KEYSTONE] ERROR: NULL parameter in GetAngles\n");
        return;
    }
    
    // printf("[KEYSTONE] GetAngles: X=%.2f, Y=%.2f, Z=%.2f\n",
    //        ctx->angles[0], ctx->angles[1], ctx->angles[2]);
    
    *angleX = ctx->angles[0];
    *angleY = ctx->angles[1];
    
    if (angleZ != NULL) {
        *angleZ = ctx->angles[2];
    }
}

void keystoneSetDenoise(keystoneContext_t* ctx, int enable)
{
    if (ctx != NULL) {
        ctx->enableDenoise = enable;
    }
}
