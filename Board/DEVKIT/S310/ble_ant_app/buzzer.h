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

 * @brief BUZZER Handling prototypes
 *
 */

#ifndef BUZZER_H__
#define BUZZER_H__

#include "boards/ras_1_6.h"
#include "smartlock\smartlock_constant.h"

/**@brief   Function to start BUZZER.
 * @details This will start the TIMER2 and enable the GPIOTE task that toggles the BUZZER.
 *          The PPI and GPIOTE configurations done by this app will make this action result in the
 *          beeping of BUZZER
 * @pre Can only be called after the SoftDevice is enabled - uses nrf_soc API
 */
void buzzer_start(void);

/**@brief   Function to stop BUZZER.
 * @details This will stop the TIMER2 and disable the GPIOTE task that toggles the BUZZER.
 *          The PPI and GPIOTE configurations done by this app will
 *          make this action result in the turning off the BUZZER.
 */
void buzzer_stop(void);

#endif // BUZZER_H__

/** @} */
/** @endcond */
