/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

 /** @cond To make doxygen skip this file */
 
/** @file

 * @brief LED Handling prototypes
 *
 */

#ifndef LED_H__
#define LED_H__

#include "boards/ras_1_6.h"
#include "devkit\devkit_constant.h"

/**@brief   Function to start GREEN LED. */
void green_led_on(void);

/**@brief   Function to stop GREEN LED. */
void green_led_off(void);

/**@brief   Function to start BLUE LED. */
void blue_led_on(void);

/**@brief   Function to stop BLUE LED. */
void blue_led_off(void);


void all_led_on(void);
void all_led_off(void);

#endif // LED_H__

/** @} */
/** @endcond */
