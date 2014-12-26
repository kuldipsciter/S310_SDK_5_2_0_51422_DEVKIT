 /* Copyright (c) 2013 Belwith Products. All Rights Reserved.
 *
 */

#ifndef SMARTLOCK_CONFIG_H
#define SMARTLOCK_CONFIG_H


#include <stdint.h>
#include <string.h>
#include "ble_srv_common.h"
#include "pstorage.h"

#define TX_BUF_SIZE								30

extern char	tx_buf[TX_BUF_SIZE];

#define REQUEST_FIELD_POS         		2
#define NUM_OF_CFG_PARA_POS       		3
#define RAS_GPIO_TEST_POS         		3
#define NUM_OF_PINS_POS           		3
#define PARA_ID_POS				  		3
#define PARA_VALUE_POS 			  		3
#define LED_TYPE_POS			  		3
#define LHS_RHS_CMD_POS			  		3
#define BUTTON_TYPE_POS					3
#define BUTTON_TEST_TYPE_POS			4
#define BUTTON_STATUS_POS				4
#define LOCK_REQ_TYPE_POS				3
#define LOCK_UNLOCK_ACK_POS				3
#define OPTO_TYPE_POS					3
#define OPTO_ACK_POS					4


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Button test type using production tool
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define BUTTON_TEST_SINGLE_CLICK  0		//button test type single click
#define BUTTON_TEST_PRESS_HOLD		1		//button test type press and hold for some seconds

#define LED_STATUS_POS			  4

#define ALL_LED_TYPE			  0			
#define BLUE_LED_TYPE			  1		
#define RED_LED_TYPE			  2		
#define GREEN_LED_TYPE			  3		
#define AMBER_LED_TYPE			  4

#define LED_OFF_CMD				  0
#define LED_ON_CMD				  1

#define SET_CONFIG_PARA           0x01
#define RANGE_TES_REQ             0x02
#define UART_TES_REQ              0x03
#define RANGE_TES_IPHONE_REQ      0x04
#define GPIO_TEST_OUTPUT_REQ	  0x05
#define GPIO_TEST_INPUT_REQ	  	  0x06
#define BAT_VOL_MEASURE_REQ	  	  0x07

#define TEMP_ANT_FREQ ``	  	  0x07


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// OPTO test type using production tool
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define OPTO_LEFT									0
#define OPTO_RIGHT								1

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// lOCK REQUEST type using production tool
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define SL_SERIALNUM_ID					1
#define SL_AUTOLOCK_TMR_ID				2
#define SL_TOTAL_AUDIT_LOG_ID			3
#define SL_TOTAL_USER_ID				4
#define SL_MANUFACT_NAME_ID				5
#define SL_MODELNUM_ID					6
#define SL_SWVERSION_ID					7
#define SL_HWVERSION_ID					8
#define SL_FCCCERTI_ID					9
#define SL_MANUFAC_DATE_ID				10
#define SL_DEVICE_NAME_ID				11
#define SL_CONFIG_STT_ID				12
#define SL_KEYFOB_INVITE_ID		   		13
#define SL_KEYFOB_SECKEY_ID				14 
#define SL_CONFIG_BUILD_ID				15
#define SL_ANT_SERIAL_NUMBER_ID			16		//RAS 1.6
#define SL_AES_KEY_1_ID					17		//RAS 1.6
#define SL_AES_KEY_2_ID					18		//RAS 1.6
#define SL_AES_KEY_3_ID					19		//RAS 1.6
#define SL_SECURITY_TOKEN_ID			20		//RAS 1.6
#define SL_SECURITY_LEVEL_ID			21		//RAS 1.6
#define SL_DEVICE_TYPE_ID				22		//RAS 1.6

/** configure permenanet and dynamic parameter

*/
void smartlock_configuration(void);

/** 

*/
void bw_configParamRead(int paramType);

/** 

*/
void setDefaultConfigParams(int paramType);

/** 

*/
void bw_configParamUpdate(int paramType);

/** 

*/
void bw_flashWrite(uint32_t * flashstart, char *buf, int length);

/** 

*/
void sl_config_by_uart(void);

/** 

*/
uint8_t cal_CRC(uint8_t StartIndex,uint8_t Count,char *msg);

/** 

*/
void sl_store_config_para(char *msg,uint8_t pos);


/** 

*/
void SmartDeviceEncryptSerialNumber(void);
/**

*/
void SmartDeviceSerialNumberEncryptBy_AES_CBC(void);
/** 

*/
uint32_t deserialize(uint8_t *buffer);

/** 

*/
void serialize(uint8_t *buffer,uint32_t value);


/** 

*/
void sl_read_config_para(uint8_t id);

/** 

*/
void sl_reply_to_pctool(void);

#endif
