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

// Maximum number of locators handled by the application.
#define MAX_NUM_LOCATORS        6

// Maximum number of asset tags handled by the application.
#define MAX_NUM_TAGS            50

// Maximum number of incomplete sequence ids.
#define MAX_NUM_SEQUENCE_IDS    6

// Maximum sequence range where data does not reset.
#define MAX_SEQUENCE_DIFF       20

// Location estimation mode.
#define ESTIMATION_MODE         SL_RTL_LOC_ESTIMATION_MODE_THREE_DIM_HIGH_ACCURACY

// Estimation interval in seconds.
// This value should approximate the time interval between two consecutive CTEs.
#define ESTIMATION_INTERVAL_SEC 0.1f

// Filter weight applied on the location coordinates. Ranges from 0 to 1.
#define FILTERING_AMOUNT        0.1f

#endif // APP_CONFIG_H
