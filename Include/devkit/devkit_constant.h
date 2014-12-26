 /* Copyright (c) 2013 Belwith Products. All Rights Reserved.
 *
 */

#ifndef SMARTLOCK_CONSTANT_H
#define SMARTLOCK_CONSTANT_H
#include <stdbool.h>

#define DEVICE_1
//#define DEVICE_2
//#define DEVICE_3
//#define DEVICE_4

//#define DEVICE_5
//#define DEVICE_6
//#define DEVICE_7
//#define DEVICE_8
//#define DEVICE_9
//#define DEVICE_A
//#define DEVICE_B
//#define DEVICE_C
//#define DEVICE_D

/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/

/****************************************************************/
//ANT
#define NO_ANT_MSG  			0
#define ANT_MSG_RCVD			1
#define ANT_MSG_ACK_CMPLTD		2


/****************************************************************/

#define DEVKIT_DEVICE_TYPE   0
#define DEVKIT_OP_TYPE   	 1
#define DEVKIT_OP_VALUE		 2

#define CURRENT_TIME_POS					2
#define HEATING_SET_POINT_POS				4
#define COOLING_SET_POINT_POS				4
#define HEATING_ACTION_TIME_POS 			5
#define COOLING_ACTION_TIME_POS				5

#define HARDWARE_TYPE_POS					2
#define HUMIDITY_SENSOR_POS					3					//To identify humidity sensor is presence or not on hardware
#define TX_POWER_POS						4

/*
#define DEVKIT_DEVICE_TYPE   2
#define DEVKIT_OP_TYPE   	 3
#define DEVKIT_OP_VALUE		 4

#define CURRENT_TIME_POS					4
#define HEATING_SET_POINT_POS				6
#define COOLING_SET_POINT_POS				6
#define HEATING_ACTION_TIME_POS 			7
#define COOLING_ACTION_TIME_POS				7

#define HARDWARE_TYPE_POS					4
#define HUMIDITY_SENSOR_POS					5					//To identify humidity sensor is presence or not on hardware
*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Application timer index
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MAX_APP_TIMER					16  //	13 in DEVKIT				// Total timers

#define FACT_RST_TIMER					0
#define RELAY_OFF_TIMER					1//0
#define WATCH_DOG_TIMER					2//1
#define MOTOR_OFF_TIMER					3//2
#define SOLENOID_OFF_TIMER				4//3
#define TEMP_SCAN_TIMER					5//4
#define ACCE_UPDATE_TIMER				6//5
#define MOTION_UPDATE_TIMER			    7//6
#define CO2_SENSOR_UPDATE_TIMER	        8//7
#define ONE_MIN_TIMER				    9//8
#define THERMOSTAT_TRG_TIMER            10//9				//Thermostate relay trigger timer
#define HUMIDITY_TEMP_TIMER			   11//10
#define BAT_VOLT_UPDATE_TIMER		   12//11
#define HW_INFO_UPDATE_TIMER		   13//12
#define SYSTEM_RESET_TIMER			   14//13
#define UART_ACT_TIMER				   15//14	

//#define HW_INFO_UPDATE_TIMER		12

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Timer values
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define RELAY_OFF_TIME				50				// In 0.1 Second
#define RELAY_OFF_TIME					10				// In 0.1 Second
#define WATCH_DOG_TIME					1					// In 0.1 Second
#define MOTOR_OFF_TIME				    20				// In 0.1 Second
#define SOLENOID_OFF_TIME 				30 				// In 0.1 Second
#define TEMP_SCAN_TIME					10				// In 0.1 Second
#define ACCE_UPDATE_TIME				5					// In 0.1 Second
#define MOTION_UPDATE_TIME				10				// In 0.1 Second
#define CO2_SENSOR_UPDATE_TIME			20				// In 0.1 Second
#define ONE_MIN_TIME					600				// In 0.1 Second
#define THERMOSTAT_TRG_TIME	    		20				//Thermostate relay trigger timer
#define HUMIDITY_TEMP_TIME				30				// In 0.1 Second //It will multiply with 3, So update time is 3 second
#define BAT_VOLT_UPDATE_TIME			10
#define HW_INFO_UPDATE_TIME				10				//Reset the hardware after setting the hardware configuration

#define SYSTEM_RESET_TIME				20

#define UART_ACT_TIME					300				// In 0.1 Second

#define FACT_RST_TIME					30				// In 0.1 Second
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Device type ( Category )
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define TEMP_SENSOR								1
#define SOLENOID_DEV							2
#define MOTOR									3
#define LIGHT_CONTROL							4
#define RELAY									5
#define ACCELEROMETER							6
#define CO2_SENSOR								7
#define SW_STATUS								8
#define DFU_OTA_MODE							9
#define MOTION_SENSOR							10
#define THERMOSTAT 								11
#define HUMIDITY_SENSOR							12
#define WATER_SENSOR							13
#define BAT_VOLTAGE								14
#define HW_CONFIG								15
#define ONLINE_MODE_SET							16
#define DEV_KIT_SRNO_CFG						17

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Operation type
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define SOLENOID_OP								1
#define MOTOR_FW									3
#define MOTOR_RW									4
#define RELAY_MOMENTORY						8
#define RELAY_PUSH_RELEASE				9
#define CO2_ALARM_ON  						1
#define CO2_ALARM_OFF							0
#define LIGHT_ON									5
#define LIGHT_OFF									6
#define LIGHT_INTENSITY						7
#define READ_STATUS								99
#define HEATING										10
#define COOLING										11
#define HW_CONFIG_SET         		12
#define ONLINE_MODE_RELAY					13
#define SR_NO_SET									14


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Parameter value
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define	HIGH								1
#define	LOW									0

/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Device Name 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define CAB_NAME                    "FW_0000022"                           			/**< Name of device. Will be included in the advertising data. */
#define LOCK_NAME                   "FW_0000444"                           			/**< Name of device. Will be included in the advertising data. */
																							
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Battery Capacity Threshold 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define UPPER_THR								20				// Battery % capacity
#define LOWER_THR								10				// Battery % capacity


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Feature Enable Disable Macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define ACTUAL_BAT_MEASUREMENT

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Macro for enable debug log on UART
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define SESSION_ID_DBG
#define OPERATION_REQ_DBG
//#define INIT_DBG
#define RECEIVED_REQ_DBG			//Received on BLE via phone application
//#define PSTORAGE_DBG
//#define NOTIFICATION_DBG



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Buzzer Count For Different Operation
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define SECURITY_LEVEL_0					0
#define SECURITY_LEVEL_1					1
#define SECURITY_LEVEL_2					2
#define SECURITY_LEVEL_3					3


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Application timer index
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define AUTHO_TIMER							3
//#define UART_ACT_TIMER						4	

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Application timer value( 1 second = x(value of time) * 0.5)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define BUTTON_TEST_ACK_TIME		20				// In 0.5 second
//#define BUTTON_TEST_FAIL_TIME		20				// In 0.5 Second
//#define BUTTON_TEST_TIME				10				// In 0.5 Second
//#define FACT_RST_TIME						6					// In 0.5 Second
//#define KEY_DETECT_IND_TIME			4					// In 0.5 Second
//#define AUTHO_TOUT_TIME					40	  		// In 0.5 Second
//#define UART_ACT_TIME						60				// In 0.5 Second
//#define BUTTON_RST_TIME					10 				// In 0.5 Second
//#define MOTION_CHECK_TIME				10				// In 0.5 Second
//#define KEY_TEST_TIME						4					// In 0.5 Second
//#define MOTION_CHECK_TIME_RE		4					// In 0.5 Second
//#define SYSTEM_RESET_TIME			6
//#define DISCC_TIME    					180				// In 0.5 Second

//#define ADV_TIME							4			// Start Advertising After this time (In second) 
//														// once bluetooth link is disconnected
//																					
//#define BAT_WARN_TIME						3600		// In Second (30 Minutes)															
//																					
//#define  LED_OFF_TIME   					3
//#define  LED_ON_TIME    					1																					

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Operation Status Related Macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define OP_STATUS_IDLE								0
#define OP_STATUS_SUCCESS							1
#define OP_STATUS_DEVICE_NOT_INITIALIZED			2
#define OP_STATUS_SR_NUM_MISMATCH     	       		3
#define OP_STATUS_SECURE_TOKEN_MISMATCH             4
#define OP_STATUS_INVALID_OP_REQ					5
#define OP_STATUS_OPERATION_TIMER_EXPIRED			6
#define OP_STATUS_USER_NOT_FOUND					7
#define OP_STATUS_NUMBER_OF_USER_LIMIT_EXCEEDED		8
#define OP_STATUS_VERY_LOW_BATTERY					9
#define OP_STATUS_SESSION_ID_MISMATCH				10
#define OP_STATUS_INVALID_COMMAND_ID_REQ			11
#define OP_STATUS_SESSION_ID_NOT_REQUIRED			12
#define OP_STATUS_NO_AUTORISED_USER_CONNECTED		13
#define OP_STATUS_MANUAL_CONNECTION					14	// Application is not auto connected


#define OP_STATUS_USER_UNAUTHORIZED		        	0x05
#define OP_STATUS_DUPLICATE_ENTRY 			        0x06
#define OP_STATUS_USER_ALREADY_EXISTS				0x0B
#define	OP_STATUS_USER_ADMIN_INIT 					0x0D	
#define	OP_STATUS_USER_INVITE_REQUIRED 				0x0E	
#define OP_STATUS_OPERATION_INITIATED				0x10
#define	OP_STATUS_USER_ALREADY_PAIRED				0x16

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Operation Request Related Macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define DFU_REQ								16
#define READ_CONFIG_REQ						1
#define WRITE_CONFIG_REQ					2
#define PAIRING_REQ							4
#define UPDATE_USER_DEVICE_ID_REQ			5
#define AUTHORIZE_CONN_REQ					6
#define READ_AUDIT_TRAIL_COUNTER_REQ		7
#define READ_AUDIT_TRAIL_REQ				8
#define READ_SESSION_ID_REQ					9
#define WRITE_SESSION_ID_REQ				10
#define WRITE_RANGE_IN_OR_OUT_REQ			11
#define REMOTE_OPERATION_REQ				12
#define STATUS_REQ							13
#define CHANGE_SECURITY_LEVEL_REQ			14
#define ADD_NETWORK_ID_REQ					15
#define DEVKIT_OPERATION_REQUEST        	30
#define REMOTE_OPERATION_RES				10
#define AUDIT_TRAIL_1_7_REQ					0
#define AUDIT_TRAIL_8_10_REQ				1

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Configuration para Buffer index
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define CONFIGURATION_BYTE0_INDEX		0
#define RESERVED_BYTE1_INDEX			1
#define RESERVED_BYTE2_INDEX			2
#define AUTOLOCK_TIMER_INDEX			3
#define TX_POWER_INDEX					4
#define ADV_INTERVAL_INDEX				5
#define CALIBRATION_MSB_INDEX			6
#define CALIBRATION_LSB_INDEX			7

#define MOTION_SENSE_POS				1
#define AUTO_LOCK_POS					2
#define AUTO_UNLOCK_POS					4
#define EXT_SMART_BUTTON_POS			8


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Command Buffer index
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define COMMAND_ID_INDEX				0
#define COMMAND_ID_LENGTH_INDEX			1
#define COMMAND_ID_PAYLOAD_INDEX		2
#define CONFIG_PARA_INDEX				2
#define AUDIT_TRAIL_REQ_INDEX			1			// if req type 0x00 than provide 1-7 audit trail
													// if req type 0x01 than provide 8-10 audit trail
#define AUDIT_TRAIL_COUNTER_INDEX		2
#define AUDIT_TRAIL_LENGTH_INDEX		3
#define AUDIT_TRAIL_PAYLOAD_INDEX		4
													
#define OPERATION_REQ_ID_INDEX			1
#define OPERATION_REQ_LENGTH_INDEX		2
#define OPERATION_REQ_PAYLOAD_INDEX		3
#define USER_SR_NO_INDEX				COMMAND_ID_PAYLOAD_INDEX							//USER_SR_NO_INDEX = 2
#define USER_SECURITY_TOCKEN_INDEX		USER_SR_NO_INDEX + SERIAL_NUM_BYTES					//USER_SECURITY_TOCKEN_INDEX = 16+2 = 18
#define USER_DEVICE_ID_INDEX			USER_SECURITY_TOCKEN_INDEX + SECURITY_TOCKEN_BYTES	//USER_DEVICE_ID_INDEX = 18+10 = 28
#define USER_TIME_INDEX					USER_DEVICE_ID_INDEX + USER_DEVICE_ID_BYTES			//USER_TIME_INDEX = 28+16 = 44
#define AUTO_MANU_CONN_STATUS_INDEX		USER_TIME_INDEX + TIME_STAMP_BYTES					//AUTO_MANU_CONN_STATUS_INDEX = 44+5 = 49

#define  DEVKIT_COMMAND_INDEX 48

#define LOCAL_AUTH_FLAG_INDEX			USER_TIME_INDEX + TIME_STAMP_BYTES					//AUTO_MANU_CONN_STATUS_INDEX = 51+5 = 56
#define ALLOW_DIRECT_FLAG_INDEX			LOCAL_AUTH_FLAG_INDEX + 1
#define INTERNET_CON_STATUS_INDEX		ALLOW_DIRECT_FLAG_INDEX + 1
#define COM_RET_BUNDLE_UPDATE_ID_INDEX	INTERNET_CON_STATUS_INDEX + 1


#define REMOVE_USER_DEVICE_ID_INDEX		USER_TIME_INDEX + TIME_STAMP_BYTES					//REMOVE_USER_DEVICE_ID_INDEX = 51+5 = 56
#define RANGE_IN_OUT_INDEX				USER_TIME_INDEX + TIME_STAMP_BYTES					//RANGE_IN_OUT_INDEX = 51+5 = 56
#define SECURITY_LEVEL_INDEX			USER_TIME_INDEX + TIME_STAMP_BYTES					//SECURITY_LEVEL_INDEX = 51+5 = 56
#define NETWORK_ID_INDEX				USER_TIME_INDEX + TIME_STAMP_BYTES					//NETWORK_ID_INDEX = 51+5 = 56
#define TARGET_ANT_SR_NO_INDEX			USER_TIME_INDEX + TIME_STAMP_BYTES					//NETWORK_ID_INDEX = 51+5 = 56

#define REMOTE_OP_DEVICE_TYPE_INDEX		TARGET_ANT_SR_NO_INDEX + ANT_SERIAL_NUMBER_BYTES					
#define REMOTE_OP_OPERA_TYPE_INDEX		REMOTE_OP_DEVICE_TYPE_INDEX + 1		

#define SESSION_ID_LENGTH_INDEX			1
#define SESSION_ID_INDEX				2
#define AUDIT_TRIAL_COUNTER_INDEX		2
#define SECURITY_LEVEL_ADV_PACKET_INDEX	16					//Security level index in advertisment packet
#define BIT_FIELD_ADV_PACKET_INDEX		17					//Bit fields index in advertisment packet
															//--------------------------------
															// 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
															// - | - | - | - | - | - | - |FCT|
															//--------------------------------
															
#define AUDIT_TRIAL_COUNTER_ADV_PACKET_INDEX	18			//Audit trail record index in advertisment packet

//Macro for remote operation
#define OPERATION_TYPE_INDEX			4
#define OPERATION_STATUS_INDEX			3
#define LOCK_STATUS_INDEX				2

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Macro for IN & OUT range based on RSSI
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define IN_RANGE						1
#define OUT_RANGE						0

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set The Transmit Power Of Radio
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RANGE_PROXIMITY			1   // Neg30dBm
#define RANGE_LOW     			2	// Neg20dBm
#define RANGE_MEDIUM			3	// 0dBm
#define RANGE_HIGH				4	// Pos4dBm 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Advertising Interval
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define ADV_INT_25MSEC			1		//	40
#define ADV_INT_50MSEC      	2		//  80
#define ADV_INT_100MSEC			3		//  160
#define ADV_INT_200MSEC     	4		//  320
#define ADV_INT_500MSEC			5		//  800
#define ADV_INT_1000MSEC		6 	//  1600

#define ADV_CNT_25MSEC		  	40
#define ADV_CNT_50MSEC      	80
#define ADV_CNT_100MSEC			160
#define ADV_CNT_200MSEC     	320
#define ADV_CNT_500MSEC			800
#define ADV_CNT_1000MSEC		1600

//********************************************************
// Flash Operation
//********************************************************
//#define BLE_DYN_PARA_FLASH_NO_OPERATION			0
//#define BLE_DYN_PARA_FLASH_OP_PENDING			1
//#define BLE_DYN_PARA_FLASH_OP_START				2

#define NO_BLE_FLASH_WRITE_OPERATION		0x00
#define BLE_FLASH_WRITE_INITIAT				0x01
#define BLE_FLASH_WRITE_IN_PROGRESS			0x02

//********************************************************
// Constant definition
//********************************************************
#define TWO_SECOND_DLY_COUNT		400000				// 2 sec delay measured on scope_with no deadbolt.	

#define ONE_SECOND_DLY_COUNT		200000				// 2 sec delay measured on scope_with no deadbolt.	

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Configuration Parameter Related macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define	PERMANENT								1
#define	BLE_DYNAMIC								2
#define	ANT_DYNAMIC								3
#define	BOTH									4

#define PERMENENT_PARA_SIZE_IN_PAGES			1

#define	MAX_USER								10
#define MAX_AUDIT_LOG							10

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Parameter Size
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define CONFIG_PARA_BYTES						9
#define TIME_STAMP_BYTES						4
#define	PHONE_NUMBER_BYTES						7
#define	USER_NAME_BYTES							12
#define ANT_SERIAL_NUMBER_BYTES					4
#define	SERIAL_NUM_BYTES						16
#define NETWORK_KEY_BYTES						10
#define MANUFACTURE_NAME_BYTES					20
#define USER_DEVICE_ID_BYTES					16
#define HW_VER_BYTES							8
#define SW_VER_BYTES							8
#define MODEL_NUM_BYTES							8
#define MANUFACTURE_DATE_BYTES					8
#define FCC_CERTI_BYTES							16
#define BUILD_ID_BYTES							2
#define AES_KEY_BYTES							16
#define SECURITY_TOCKEN_BYTES					10
#define SESSION_ID_BYTES						16
#define DEVICE_NAME_BYTES						10

#define EMAIL_ADD_SIZE							40

#define USER_RECORD_CHAR_SIZE					20

#define OP_TYPE_SIZE							1
#define OP_STATUS_SIZE							1

#define SIGNLE_AUDIT_LOG_SIZE					(TIME_STAMP_BYTES + USER_NAME_BYTES + OP_TYPE_SIZE + OP_STATUS_SIZE)
#define SIGNLE_DEBUG_LOG_SIZE					20

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Default Value
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define SERIAL_NUM								"DEVKIT-TEST00001"						
#define SERIAL_NUM								"DEVKIT-000000001"
//#define SERIAL_NUM								"DEVKIT-000000032"
//#define SERIAL_NUM								"DEVKIT-000000046"
//#define SERIAL_NUM								"DEVKIT-000000090"

//#define ANT_SERIAL_NUM							{0x99,0x00,0x00,0x01}
#define MANUFACTURE_NAME						" BELWITH PRODUCTION "
#define MODEL_NUM								"RAS_1_6 "
#define NETWORK_KEY								"4444444444"
#define HARDWARE_VERSION						"02.01.01"
#define SOFTWARE_VERSION						"02.01.03"

//#define DEFAULT_AES_KEY_1						0x0000000000000001
//#define DEFAULT_AES_KEY_2						0x0000000000000002
//#define DEFAULT_AES_KEY_3						0x0000000000000003

#define DEFAULT_AES_KEY_1						{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01}
#define DEFAULT_AES_KEY_2						{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02}
#define DEFAULT_AES_KEY_3						{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03}

#define SECURITY_TOCKEN							"6666666666"
#define DEVICE_NAME								"FW_0000999"
#define MANUFACTURE_DATE						"21/07/14"
#define FCC_CERTI								"8888888888888888"
#define BUILD_ID								"99"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Auto Lock/Unlock enable
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define AUTO_DEACTIVE							0
#define AUTO_UNLOCK_ONLY_ACTIVE			  		1
#define AUTO_LOCK_ONLY_ACTIVE					2
#define AUTO_ACTIVE								3

#define AUTO_DEAC_MOTION_ACTV					8

#define MOTION_POS								8
#define MANUAL_AUTO_UNLOCK_POS					4

#define FACT_RST_COND							(MOTION_POS + MANUAL_AUTO_UNLOCK_POS)


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// User Specification Information
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define AUTO_UNLOCK_ENABLE					0x01			// Bit-0
#define AUTO_LOCK_ENABLE		  			0x02			// Bit-1
#define ADMIN_RIGHT							0x04			// Bit-2
#define PIN_REQUIRED						0x08			// Bit-3
#define EMAIL_NOTIFICATION_ENABLE			0x10			// Bit-4
#define GLBL_EMAIL_NOTIFICATION  			0x20			// Bit-5
#define EXT_BUTN_UNLOCK_ENABLE  			0x40			// Bit-6
#define LOW_BAT_REPETE_WARN_ENABLE  		0x80			// Bit-7

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Misc. macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define DEBUG_BUFFER_SIZE					100
#define RX_BUFFER_SIZE						150

#define DUMMY_BATTERY_LEVEL					100

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Define Manual Connection or Auto Connection Macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MANUAL_CONN							0
#define AUTO_CONN							1

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Status Reporting Macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define	ACTIVE								1
#define	NON_ACTIVE							0


#define SUCCESS     						1
#define FAIL        						0

#define TRUE								1
#define FALSE								0

#define ON  								1
#define OFF  								0

#define PROD_SUCESS							0
#define PROD_FAIL 							1

#define LEFT_POS							1
#define RIGHT_POS							2

#endif
