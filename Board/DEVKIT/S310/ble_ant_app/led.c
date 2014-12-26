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
 * @defgroup  led.c

 * @brief LED control for the SmartLock example application
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "boards/ras_1_6.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "led.h"
#include "app_util.h"
#include "nrf_delay.h"
#include "main.h"

unsigned char led_mask=0;

#define FACT_RST_CNT					2

#define DELAY_TIME_SR_PARALL	1

//------------------------------------------------------------------------------
// Turn ON GREEN LED's
//------------------------------------------------------------------------------	
void green_led_on(void)
{
	nrf_gpio_pin_set(GREEN_LED_PIN_NUMBER);		
}
//------------------------------------------------------------------------------
// Turn OFF GREEN LED's
//------------------------------------------------------------------------------	
void green_led_off(void)
{
	nrf_gpio_pin_clear(GREEN_LED_PIN_NUMBER);	
}
//------------------------------------------------------------------------------
// Turn ON BLUE LED's
//------------------------------------------------------------------------------	
void blue_led_on(void)
{
	nrf_gpio_pin_set(BLUE_LED_PIN_NUMBER);			
}
//------------------------------------------------------------------------------
// Turn OFF BLUE LED's
//------------------------------------------------------------------------------	
void blue_led_off(void)
{
	nrf_gpio_pin_clear(BLUE_LED_PIN_NUMBER);
}

void all_led_on(void)
{
	nrf_gpio_pin_set(GREEN_LED_PIN_NUMBER);	
	nrf_gpio_pin_set(BLUE_LED_PIN_NUMBER);
}

void all_led_off(void)
{
	nrf_gpio_pin_clear(BLUE_LED_PIN_NUMBER);
	nrf_gpio_pin_clear(GREEN_LED_PIN_NUMBER);	
}
/**
 * @}
 */
