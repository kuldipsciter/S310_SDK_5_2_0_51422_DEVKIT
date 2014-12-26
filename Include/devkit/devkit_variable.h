 /* Copyright (c) 2013 Belwith Products. All Rights Reserved.
 *
 */

#ifndef SMARTLOCK_VARIABLE_H
#define SMARTLOCK_VARIABLE_H

#include <stdint.h>
#include "devkit\devkit_constant.h"


typedef struct 
{
	
	uint8_t		mu8ar_UserDeviceId[USER_DEVICE_ID_BYTES];
	
}st_UserProfile_t;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Permanent parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct										// data type
{
	
	uint8_t 	mu8_FlashStatus;
	uint8_t 	mu8ar_SerialNumber[SERIAL_NUM_BYTES];	
	uint8_t 	mu8ar_AntSerialNumber[ANT_SERIAL_NUMBER_BYTES];
	uint8_t 	mu8ar_ManufactName[MANUFACTURE_NAME_BYTES];
	uint8_t 	mu8ar_ModelNumber[MODEL_NUM_BYTES];
	uint8_t 	mu8ar_NetworkKey[NETWORK_KEY_BYTES];		
	uint8_t 	mu8ar_HardwareVersion[HW_VER_BYTES];
	uint8_t 	mu8ar_SoftwareVersion[SW_VER_BYTES];
	uint8_t		mu8ar_AesKey1[AES_KEY_BYTES];
	uint8_t		mu8ar_AesKey2[AES_KEY_BYTES];
	uint8_t		mu8ar_AesKey3[AES_KEY_BYTES];
	uint8_t		mu8ar_SecurityTocken[SECURITY_TOCKEN_BYTES];
	uint8_t		mu8ar_DeviceName[DEVICE_NAME_BYTES];
	uint8_t 	mu8ar_ManufactDate[MANUFACTURE_DATE_BYTES];		// “MM.DD.YY”
	uint8_t 	mu8ar_FccCertificate[FCC_CERTI_BYTES];
	uint8_t 	mu8ar_BuildId[BUILD_ID_BYTES];
	
}__attribute__((aligned(4)))st_FlashPermanentParams_t; 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Dynamic parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef struct
{
	
	uint8_t 	mu8_Month;
	uint8_t 	mu8_Day;
	uint8_t 	mu8_Year;
	uint8_t 	mu8_Hour;
//	uint8_t 	mu8_Minute;
	
}st_DateAndTime_t;


typedef struct
{
	
	st_DateAndTime_t	mst_TimeStamp; 
	uint8_t 			mu8_UserDeviceId[USER_DEVICE_ID_BYTES];	
	uint8_t				mu8_OperationType;
	uint8_t				mu8_OperationStatus;
	
}st_AuditLogProfile_t;

typedef struct
{
	
	uint8_t 				mu8_FlashStatus;
	uint8_t 				mu8_NumOfUser;
	uint8_t					mu8_UserRecordUpdateCounter;
	uint8_t 				mu8_SrDevStatus;
	uint8_t  				mu8_TotalAuditTrailLogRecord;
	uint8_t					mu8_AuditTrailRecordRunning;
	uint8_t   				mu8_AutoLockTimer;
	uint8_t       			mu8_BatteryLevel;
	uint8_t 				mu8_TxPower;	
	uint8_t       			mu8_AdvertiseInterval;
	uint8_t					mu8_Securitylevel;
	uint8_t					mu8_DeviceType;
	uint8_t					mu8ar_ConfigurationParameter[CONFIG_PARA_BYTES];
	
	/*******DEVKIT***********/
	uint8_t 				light_intensity;
	uint8_t 				light_status;
	int8_t					heating_set_point_flash;
	int8_t					cooling_set_point_flash;
	uint8_t 				hardware_type_flash;
	uint8_t 				humidity_sensor_presence_flash;									//Identifying humidity sensor is presence or not
	uint8_t 				relay_op_online_mode_flash;
	uint8_t 				flash_Status_flg;
	uint8_t					dev_kit_sr_no_set_flg_flash;
	/**********************/
	
	st_UserProfile_t		mst_UserRecord[MAX_USER];	
	st_AuditLogProfile_t 	mst_AuditLogRecord[MAX_AUDIT_LOG];	

}__attribute__((aligned(4)))st_BleFlashDynamicParams_t; 


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Dynamic parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef struct
{
	
	uint8_t mu8_flash_Status;
	uint8_t mu8ar_serial_num_str[SERIAL_NUM_BYTES + 1];
	uint8_t mu8ar_ant_serial_num_str[ANT_SERIAL_NUMBER_BYTES+1];		//RAS 1.6
	uint8_t mu8ar_aes_key_1[AES_KEY_BYTES+1];							//RAS 1.6
	uint8_t mu8ar_aes_key_2[AES_KEY_BYTES+1];							//RAS 1.6
	uint8_t mu8ar_aes_key_3[AES_KEY_BYTES+1];							//RAS 1.6
	uint8_t mu8ar_security_token[SECURITY_TOCKEN_BYTES+1];				//RAS 1.6
	uint8_t mu8_device_type;											//RAS 1.6
	uint8_t mu8_security_level;											//RAS 1.6
	
}__attribute__((aligned(4)))st_FlashDataOfPreviousVersion_t;


#endif
