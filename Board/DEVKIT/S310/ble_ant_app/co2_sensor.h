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
 *
 * @defgroup Battery Level Hardware Handling

 * @brief Battery Level Hardware Handling prototypes
 *
 */

#ifndef CO2_SENSOR_H__
#define CO2_SENSOR_H__

#include <stdint.h>

#define MAX_COUNT_10_BIT_RESOLUTION					 1023
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS        1100                           /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION         9                              /**< The ADC is configured to use VDD input, no prescaling */
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS       250                            /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 * @retval     Result converted to millivolts.
 */
 
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / MAX_COUNT_10_BIT_RESOLUTION) * ADC_PRE_SCALING_COMPENSATION)
				
/**@brief Caluclate mV 
 */
uint16_t get_co2_voltage(void);

#endif // CO2_SENSOR_H__

/** @} */
/** @endcond */
