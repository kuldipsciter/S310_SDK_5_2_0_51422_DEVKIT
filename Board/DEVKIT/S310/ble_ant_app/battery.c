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
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble_stack_handler.h"
#include "ble_bas.h"
#include "battery.h"
#include "main.h"
#include "app_util.h"
#include "simple_uart.h"

#define HYS_BAT_LEVEL		5									// 5% hysterisis level

/**@brief External reference to the Battery Service. */
extern ble_bas_t                             m_bas;

/**@brief Actual Capacity from voltage measurement.
 *
 */
/** @brief Converts the input voltage (in milli volts) into percentage of 6 Volts.
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
uint8_t battery_level_in_percent(uint16_t mvolts)
{
    uint8_t battery_level;

		#ifdef ACTUAL_BAT_MEASUREMENT
    if (mvolts >= 6000)
    {
        battery_level = 100;
    }
    else if (mvolts > 5300)
    {
        battery_level = 100 - ((6000 - mvolts) * 40) / 700;
    }
    else if (mvolts > 4700)
    {
        battery_level = 60 - ((5300 - mvolts) * 30) / 600;
    }
    else if (mvolts > 4300)
    {
        battery_level = 30 - ((4700 - mvolts) * 10) / 400;
    }
    else if (mvolts > 4100)
    {
        battery_level = 20 - ((4300 - mvolts) * 10) / 200;
    }
    else if (mvolts > 4000)
    {
        battery_level = 10 - ((4100 - mvolts) * 10) / 100;
    }		
    else
    {
        battery_level = 0;
    }
		#else
				battery_level = DUMMY_BATTERY_LEVEL;
		#endif
		
    return battery_level;
}

/**@brief Battery Volatge Measurement.
 * @details  This function will returns battery volatge in mV.
 *
 */
uint16_t get_battery_voltage(void)
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

//	batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
//														DIODE_FWD_VOLT_DROP_MILLIVOLTS;

	return adc_result;
	//return batt_lvl_in_milli_volts;
}
/**@brief Battery Capacity Measurement.
 * @details  This function will returns capacity of battery in 0-100%.
 *
 */
uint8_t get_battery_level(void)
{
	uint16_t    adc_result;
	uint16_t    batt_lvl_in_milli_volts;
	uint8_t     percentage_batt_lvl;
	uint32_t    err_code;

	NRF_ADC->ENABLE     		= ADC_ENABLE_ENABLE_Enabled;		// Enable ADC
	NRF_ADC->TASKS_START 		= 1;				// Start ADC Cobversion
	while(NRF_ADC->BUSY);		  				  // Wait Till ADC is busy in conversion
	NRF_ADC->EVENTS_END     = 0;
	adc_result              = NRF_ADC->RESULT;
	NRF_ADC->TASKS_STOP     = 1;
	NRF_ADC->ENABLE     		= ADC_ENABLE_ENABLE_Disabled;		// Disable ADC

	batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
														DIODE_FWD_VOLT_DROP_MILLIVOLTS;
	percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

	if(percentage_batt_lvl > gst_BleConfigDynamicParams.mu8_BatteryLevel)
	{
		 if((percentage_batt_lvl - gst_BleConfigDynamicParams.mu8_BatteryLevel ) >=  HYS_BAT_LEVEL)
		 {
				gst_BleConfigDynamicParams.mu8_BatteryLevel = percentage_batt_lvl;		
			 //				if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
//					BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
		 }
	}
	else if (percentage_batt_lvl < gst_BleConfigDynamicParams.mu8_BatteryLevel)
	{
			gst_BleConfigDynamicParams.mu8_BatteryLevel = percentage_batt_lvl;		
//			if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
//				BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;		
	}

	gst_BleConfigDynamicParams.mu8_BatteryLevel = 100;
	err_code = ble_bas_battery_level_update(&m_bas, gst_BleConfigDynamicParams.mu8_BatteryLevel);
	if (
			(err_code != NRF_SUCCESS)
			&&
			(err_code != NRF_ERROR_INVALID_STATE)
			&&
			(err_code != BLE_ERROR_NO_TX_BUFFERS)
			&&
			(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
	)
	{
			APP_ERROR_HANDLER(err_code);
	}		

	return percentage_batt_lvl;
}



/**
 * @}
 */
