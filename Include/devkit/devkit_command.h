 /* Copyright (c) 2013 Belwith Products. All Rights Reserved.
 *
 */

#ifndef SMARTLOCK_COMMAND_H
#define SMARTLOCK_COMMAND_H

#include "devkit\devkit_variable.h"

//#define RUN_DBG 1
typedef volatile struct 
{	
	uint8_t mu8_BleFlashUpdate : 2;
	uint8_t	mu8_BleConnectionState : 1;
	uint8_t	mu8_AntCH0_CloseByBLE : 1;
	uint8_t	mu8_AntCH1_CloseByBLE : 1;
	
}st_Blebitfield_t;

extern st_Blebitfield_t BleBits;

/**@brief Read characteristic value and update in local database.
	* @param[in]   paramUUID  UUID of characteristic.
	* @param[in]   *data  		point to value of characteristic.
	* @param[in]   len  			legnth of characteristic.
 */
void paramUpdate(uint32_t paramUUID, uint8_t *data, uint8_t len);


/**@brief Execute command received from phone application

*/
void bleP0_BleAppDevkitCommandExecution(void);


/**@brief Execute command received from phone application

*/
uint32_t bleP1_CommandExecuter(uint8_t *);

/**@brief store auditlog parameters

*/
void blePx_AuditLog( uint8_t *msg , uint8_t button_type);

/**@brief Generate Session ID

*/
void bleP2_GenerateSessionID(void);

/**@brief Encrypt the data

*/
void blePx_EncryptData(uint8_t *aes_key,uint8_t *data_buffer,uint8_t length, uint8_t index);

/**@brief Decrypt the data

*/
void blePx_DecryptData(uint8_t *aes_key,uint8_t *data_buffer,uint8_t length);

/**@brief Shuffel the data for sending to phone application

*/
void bleP3_ShuffelTheDataBuffer(uint8_t *SuffelBuffer,uint8_t DataLength);

/**@brief Deshuffel the data sent by phone application

*/
void bleP2_DeshuffelTheDataBuffer(uint8_t *SuffelBuffer,uint8_t DataLength);

/*

*/
void bleP3_UserAuthenticationProcess(uint8_t *);

/*

*/
void bleP3_OperationReqVerificationProcess(uint8_t *);

/*

**/
void bleP2_CommandIDReqVerificationProcess(uint8_t *);

/**@brief find record index for user operation

*/
int8_t blePx_FindUserDeviceID(uint8_t *);

/**@brief factory reset of smartlock configuration parameter

*/
void sl_factory_reset(void); 

/*
*/
void reset_all_the_timer_and_flg(void);
/*

*/
void devkitCommandExecution(void);
/*

*/
void update_water_sensor_status(void);


uint8_t  bleP0_ReceviedBleMsgOnAnt(uint8_t *);

void decode_op_req_msg(uint8_t *data);

#endif
