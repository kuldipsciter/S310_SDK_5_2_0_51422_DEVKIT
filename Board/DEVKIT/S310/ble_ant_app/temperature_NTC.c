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
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble_bas.h"
#include "main.h"
#include "temperature_NTC.h"
#include "app_util.h"
#include "nrf_delay.h"
#include "simple_uart.h"

#define HYS_BAT_LEVEL		5									// 5% hysterisis level

/**@brief External reference to the Battery Service. */
extern ble_bas_t                             bas;

/**@brief External reference to the Battery Service. */
extern ble_bas_t                             bas;

extern uint8_t dbgStr[50];
/**@brief Actual Capacity from voltage measurement.
 *
 */
/** @brief ADC count to temperature conversion function
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 6V returns 100% battery level. The limit for power failure is 4.0V and
 *           is considered to be the lower boundary (0%).
 *
 *           The discharge curve for Alkaline is non-linear. In this model it is split into
 *           5 linear sections: 
 *           - Section 1: 6.0V - 5.3V = 100% - 60% (40% drop on 700 mV)
 *           - Section 2: 5.3V - 4.7V = 60% - 30%  (30% drop on 600 mV)
 *           - Section 3: 4.7V - 4.3V = 30% - 20%  (10% drop on 400 mV)
 *           - Section 4: 4.3V - 4.1V = 20% - 10%  (10% drop on 200 mV)
 *           - Section 5: 4.1V - 4.0V = 10% - 0%  (10% drop on 100 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
*/
float ADC_count_to_temp_convertor(uint16_t adc_result)
{
    float Temp;
	
    if (adc_result > 11 && adc_result <65)
    {
        Temp=(adc_result-111)/1.98;
    }
    else if (adc_result >= 66 && adc_result <216)
    {
        Temp=(adc_result-220)/6.13;
    }
		else if (adc_result >= 217 && adc_result <537)
    {
        Temp=(adc_result-236)/10.92;
    }
		else if (adc_result >= 538 && adc_result <788)
    {
        Temp=(adc_result-307)/8.8;
    }
		else if(adc_result > 788)
		{
			Temp=55;
		}
		else
		{
			Temp = -50;
		}
		
    return Temp;
}

/**@brief Temperature measure Measurement.
 * @details  This function will returns capacity of battery in 0-100%.
 *
 */
uint16_t get_temperature_count(void)
{	
		uint16_t    adc_result;
		//float      current_temperature;

    NRF_ADC->ENABLE     		= ADC_ENABLE_ENABLE_Enabled;		// Enable ADC
    NRF_ADC->TASKS_START 		= 1;														// Start ADC Cobversion
		while(NRF_ADC->BUSY);		  				  										// Wait Till ADC is busy in conversion
		NRF_ADC->EVENTS_END     = 0;
		adc_result              = NRF_ADC->RESULT;
		NRF_ADC->TASKS_STOP     = 1;
    NRF_ADC->ENABLE     		= ADC_ENABLE_ENABLE_Disabled;		// Disable ADC
	
		//current_temperature = ADC_count_to_temp_convertor(adc_result);

		//return current_temperature;
		return adc_result;
}

/**@brief Temperature measure Measurement.
 * @details  This function will returns capacity of battery in 0-100%.
 *
 */
float get_temperature(void)
{
		uint16_t	avg_result=0;
		float     current_temperature;
	
		
	for(uint8_t i=0; i<50 ; i++)
		{
			avg_result+=get_temperature_count();
			nrf_delay_us(10); 
		}
		avg_result=(uint16_t) (avg_result/50);
		
		#ifdef RUN_DBG
			sprintf(dbgStr, "\r\ntemp = %d ", avg_result);
			simple_uart_putstring(dbgStr);
		#endif
		current_temperature = ADC_count_to_temp_convertor(avg_result);
		return current_temperature;
}

/**
 * @}
 */
