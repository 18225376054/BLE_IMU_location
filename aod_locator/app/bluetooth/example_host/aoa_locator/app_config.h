/***************************************************************************//**
 * @file
 * @brief Application configuration values.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "aoa_board.h"

#define AOD_ANGLE 1
// Maximum number of asset tags handled by the application.
// For the case of AoD, the host application will connect to the ncp mode tag 
// for get the IQ sample for angle calculation.
// So the maximum number of asset for each host application can be 1.
#define AOA_MAX_TAGS                   100

// Measurement interval expressed as the number of connection events.
#define CTE_SAMPLING_INTERVAL          3

// Minimum CTE length requested in 8 us units. Ranges from 16 to 160 us.
#define CTE_MIN_LENGTH                 20

// Maximum number of sampled CTEs in each advertising interval.
// 0: Sample and report all available CTEs.
#define CTE_COUNT                      0

// Switching and sampling slots in us (1 or 2).
#define CTE_SLOT_DURATION              1


#if 1 //Cheng
#define AOX_ARRAY_TYPE                 SL_RTL_AOX_ARRAY_TYPE_4x4_URA

// Maximum number of locators handled by the application.
#define MAX_NUM_LOCATORS        10

// Maximum number of asset tags handled by the application.
#define MAX_NUM_TAGS            50

// Location estimation mode.
// #define ESTIMATION_MODE         SL_RTL_LOC_ESTIMATION_MODE_THREE_DIM_HIGH_ACCURACY
#define ESTIMATION_MODE         SL_RTL_LOC_ESTIMATION_MODE_TWO_DIM_HIGH_ACCURACY

// Estimation interval in seconds.
// This value should approximate the time interval between two consecutive CTEs.
#define ESTIMATION_INTERVAL_SEC 0.1f

// Filter weight applied on the location coordinates. Ranges from 0 to 1.
#define FILTERING_AMOUNT        0.1f

// -----------------------------------------------------------------------------
// Secondary configuration values based on primary values.

#if (AOX_ARRAY_TYPE == SL_RTL_AOX_ARRAY_TYPE_4x4_URA)
#define AOA_NUM_SNAPSHOTS       (4)
#define AOA_NUM_ARRAY_ELEMENTS  (4 * 4)
#define AOA_REF_PERIOD_SAMPLES  (7)
#define SWITCHING_PATTERN       { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }
#elif (AOX_ARRAY_TYPE == SL_RTL_AOX_ARRAY_TYPE_3x3_URA)
#define AOA_NUM_SNAPSHOTS       (4)
#define AOA_NUM_ARRAY_ELEMENTS  (3 * 3)
#define AOA_REF_PERIOD_SAMPLES  (7)
#define SWITCHING_PATTERN       { 1, 2, 3, 5, 6, 7, 9, 10, 11 }
#elif (AOX_ARRAY_TYPE == SL_RTL_AOX_ARRAY_TYPE_1x4_ULA)
#define AOA_NUM_SNAPSHOTS       (18)
#define AOA_NUM_ARRAY_ELEMENTS  (1 * 4)
#define AOA_REF_PERIOD_SAMPLES  (7)
#define SWITCHING_PATTERN       { 0, 1, 2, 3 }
#endif
#endif

#endif // APP_CONFIG_H


