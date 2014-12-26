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

/** @file
 *
 * @defgroup  buzzer.c

 * @brief BUZZER control for the SmartLock example application
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_delay.h"
#include "buzzer.h"
#include "app_error.h"

#define MAX_SAMPLE_LEVELS 							(1500UL)     																/**< Maximum number of sample levels. */
#define TIMER_PRESCALERS 								6U           																/**< Prescaler setting for timer. */

#define PPI_CHAN2_TO_TOGGLE_LED              2            /*!< The PPI Channel that connects CC0 compare event to the GPIOTE Task that toggles the BUZZER. */
#define GPIOTE_CHAN2_FOR_BUZZER_TASK         2            /*!< The GPIOTE Channel used to perform write operation on the BUZZER pin. */
#define TIMER2_PRESCALER                     7            /*!< Prescaler setting for timer. */

extern uint8_t      buzzer_freq;

/* @brief Timer 2 initialisation function.
 *
 * @details This function will initialise Timer 2 peripheral. This timer is used only to
 *          generate capture compare events that toggle the buzzer @1/2/3/4KHz.
 */






/**
 * @}
 */
