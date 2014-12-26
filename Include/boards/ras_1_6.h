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
#ifndef DEVKIT_H__
#define DEVKIT_H__

#include "nrf_gpio.h"
#include "main.h"
/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/

////Define for NS_BLE module	
//#define STATUS_SW1_NS_BLE								28
//#define STATUS_SW2_NS_BLE								29
//#define MOTOR_OP_POS_PIN_NS_BLE							14
//#define MOTOR_OP_NEG_PIN_NS_BLE							10

//#define TX_PIN_NUMBER_NS_BLE							17

////Define for N548 module
//#define STATUS_SW1_N548									30
//#define STATUS_SW2_N548									31
//#define MOTOR_OP_POS_PIN_N548							3
//#define MOTOR_OP_NEG_PIN_N548							2
//#define TX_PIN_NUMBER_N548								21

//#define MOTION_DETECT_PIN_NUMBER						9
//#define WATER_SENSOR_PIN_NUMBER							6
//#define FACTORY_RESET									15

//#define SOLENOID_CTR_PIN_NUMBER							13
//#define RELAY_1_CTR_PIN_NUMBER							12
//#define RELAY_2_CTR_PIN_NUMBER							8
//#define GREEN_LED_PIN_NUMBER							24
//#define BLUE_LED_PIN_NUMBER								25
//#define PWM_OUTPUT_PIN_NUMBER							23
//#define WDT_PIN_NO						  				19					//Generate clock for WDT hardware

//#define CTS_PIN_NUMBER 									16    			// UART Clear To Send pin number. Not used if HWFC is set to false
//#define RTS_PIN_NUMBER 									17    			// Not used if HWFC is set to false
//#define RX_PIN_NUMBER  									18    			// UART RX pin number.
//#define HWFC           			  					   false 				// UART hardware flow control
//	
//#define I2C_DATA_PIN_NUMBER								11
//#define I2C_CLK_PIN_NUMBER								20


//Define for NS_BLE module	
#define STATUS_SW1_NS_BLE								30
#define STATUS_SW2_NS_BLE								29
#define MOTOR_OP_POS_PIN_NS_BLE							3
#define MOTOR_OP_NEG_PIN_NS_BLE							2

#define TX_PIN_NUMBER_NS_BLE							17

//Define for N548 module
#define STATUS_SW1_N548									30

#ifdef BLE_PA
#define STATUS_SW2_N548									29
#else
#define STATUS_SW2_N548									31
#endif

#define MOTOR_OP_POS_PIN_N548							3
#define MOTOR_OP_NEG_PIN_N548							2
#define TX_PIN_NUMBER_N548								21

#define MOTION_DETECT_PIN_NUMBER						9
#define WATER_SENSOR_PIN_NUMBER							6
#define FACTORY_RESET									15

#define SOLENOID_CTR_PIN_NUMBER							13
#define RELAY_1_CTR_PIN_NUMBER							12
#define RELAY_2_CTR_PIN_NUMBER							8
#define GREEN_LED_PIN_NUMBER							24
#define BLUE_LED_PIN_NUMBER								25
#define PWM_OUTPUT_PIN_NUMBER							23

#ifdef BLE_PA
#define WDT_PIN_NO						  				14					//Generate clock for WDT hardware
#else 
#define WDT_PIN_NO						  				19					//Generate clock for WDT hardware
#endif

#define CTS_PIN_NUMBER 									16    			// UART Clear To Send pin number. Not used if HWFC is set to false
#define RTS_PIN_NUMBER 									17    			// Not used if HWFC is set to false

#ifdef BLE_PA
#define RX_PIN_NUMBER  									22    			// UART RX pin number.
#else
#define RX_PIN_NUMBER  									18    			// UART RX pin number.
#endif

#define HWFC           			  					   false 				// UART hardware flow control
	
#define I2C_DATA_PIN_NUMBER								11

#ifdef BLE_PA
#define I2C_CLK_PIN_NUMBER								28
#else
#define I2C_CLK_PIN_NUMBER								20
#endif

#define ANT_SEL											17
#define CSD												18
#define CPS												19

//#define RX_PIN_NUMBER  		24	    // UART RX pin number.
//	#define TX_PIN_NUMBER  		12   	// UART TX pin number.
/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/
#endif 
