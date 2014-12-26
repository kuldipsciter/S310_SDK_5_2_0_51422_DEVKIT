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
 * @defgroup battery.c

 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble_bas.h"
#include "co2_sensor.h"
#include "app_util.h"
#include "nrf_delay.h"
#include "simple_uart.h"
#include "main.h"

/**@brief Battery Volatge Measurement.
 * @details  This function will returns battery volatge in mV.
 *
 */
extern uint8_t dbgStr[50];

uint16_t get_co2_voltage(void)
{	
		uint16_t    adc_result;
		//uint16_t    batt_lvl_in_milli_volts;

    NRF_ADC->ENABLE     		= ADC_ENABLE_ENABLE_Enabled;		// Enable ADC
    NRF_ADC->TASKS_START 		= 1;				// Start ADC Cobversion
		while(NRF_ADC->BUSY);		  				  // Wait Till ADC is busy in conversion
		NRF_ADC->EVENTS_END     = 0;
		adc_result              = NRF_ADC->RESULT;
		NRF_ADC->TASKS_STOP     = 1;
    NRF_ADC->ENABLE     		= ADC_ENABLE_ENABLE_Disabled;		// Disable ADC
	
//		batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);

//		return batt_lvl_in_milli_volts;
		#ifdef RUN_DBG
			sprintf(dbgStr, " co2 = %d ", adc_result);
			simple_uart_putstring(dbgStr);
		#endif
		return adc_result;
}

/**
 * 
 */
