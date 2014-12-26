#include <string.h>
#include <stdio.h>
#include "simple_uart.h"
#include "AntEventHandler.h"
#include "ble_sr.h"
#include "development_kit_config.h"
#include "main.h"
#include "devkit\devkit_variable.h"
#include "devkit\devkit_config.h"
#include "devkit\devkit_constant.h"

/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/

extern pstorage_handle_t    mp_flash_PermPara;                      ///< Pointer to flash location to write next System Attribute information. */
extern uint32_t light_intensity_value;
extern bool light_status_flg;
extern uint8_t dev_kit_sr_no_set_flg;
extern int8_t heating_set_point;
extern int8_t cooling_set_point;
extern uint8_t hardware_type;
extern uint8_t humidity_sensor_presence;
extern uint8_t tx_power;
extern uint8_t relay_op_online_mode;
extern uint8_t flash_Status;
extern uint8_t DevKit_SrNo[DEV_KIT_SR_NO_BYTES];

/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/

st_AntNodeFlashParams_t *gstp_SlaveInfo; 

uint8_t *gu8p_AntConfigParamAddr = NULL;
uint8_t *gu8p_BLEConfigParamAddr = NULL;
uint8_t *gu8p_PeramParaConfigParamAddr = NULL;
uint8_t *gu8p_FlashDataOfPreviousVersion = NULL;

uint8_t gu8_AntNoOfUnRegisterNode=0;

#ifdef DEVICE_1
//CD2470D1
const uint8_t gu8ar_ANTDefaultSrNo[SR_NO_LENGTH]={0xCD,0x24,0x70,0xD1};
	//const uint8_t gu8ar_ANTDefaultSrNo[SR_NO_LENGTH]={0x88,0x00,0x00,0x01};
#endif

#ifdef DEVICE_2
	const uint8_t gu8ar_ANTDefaultSrNo[SR_NO_LENGTH]={0x88,0x00,0x00,0x02};
#endif
	
#ifdef DEVICE_3
	const uint8_t gu8ar_ANTDefaultSrNo[SR_NO_LENGTH]={0x88,0x00,0x00,0x03};
#endif
	
#ifdef DEVICE_4
	const uint8_t gu8ar_ANTDefaultSrNo[SR_NO_LENGTH]={0x88,0x00,0x00,0x04};
#endif
	
#ifdef DEVICE_5
	const uint8_t gu8ar_ANTDefaultSrNo[SR_NO_LENGTH]={0x88,0x00,0x00,0x05};
#endif

//*****************************************************************************
// devkit Configuration 
//*****************************************************************************

void devkit_Configuration(void)
{
	ReadPermConfiguration();
	antP2_ReadDynaConfiguration();
	bleP2_ReadDynaConfiguration();
}

void ReadPermConfiguration(void)
{
	// Start address for configuration parameter storage area
	gu8p_PeramParaConfigParamAddr = (uint8_t *)mp_flash_PermPara.block_id; 
	
	// Check Flash has been initialzied or not? (BLE Application)
	memcpy((uint8_t *)&gst_ConfigPermanentParams,gu8p_PeramParaConfigParamAddr,sizeof(st_FlashPermanentParams_t));
	
	if(gst_ConfigPermanentParams.mu8_FlashStatus != 0x01)
	{
		#ifdef INIT_DBG
			DEBUG("\r\n**************************************");
			DEBUG("\r\n** Write Default parameter in Flash **");
			DEBUG("\r\n**************************************");
		#endif
		//*******************************************************************//
		// Flash Not Initialized so set Default Permanent Parameters to Flash
		//*******************************************************************//
		memset((void *) &gst_ConfigPermanentParams, 0, sizeof(gst_ConfigPermanentParams));
		gst_ConfigPermanentParams.mu8_FlashStatus = 1;
		memcpy(gst_ConfigPermanentParams.mu8ar_SerialNumber, SERIAL_NUM, SERIAL_NUM_BYTES);
		memcpy(gst_ConfigPermanentParams.mu8ar_AntSerialNumber, gu8ar_ANTDefaultSrNo, ANT_SERIAL_NUMBER_BYTES);
		memcpy(gst_ConfigPermanentParams.mu8ar_ManufactName, MANUFACTURE_NAME, MANUFACTURE_NAME_BYTES);
		memcpy(gst_ConfigPermanentParams.mu8ar_ModelNumber, MODEL_NUM, MODEL_NUM_BYTES);
		memcpy(gst_ConfigPermanentParams.mu8ar_NetworkKey, NETWORK_KEY, NETWORK_KEY_BYTES);
		memcpy(gst_ConfigPermanentParams.mu8ar_HardwareVersion, HARDWARE_VERSION, HW_VER_BYTES);
		memcpy(gst_ConfigPermanentParams.mu8ar_SoftwareVersion, SOFTWARE_VERSION, SW_VER_BYTES);
		
//		memcpy(gst_ConfigPermanentParams.mu8ar_AesKey1, (char*)DEFAULT_AES_KEY_1, AES_KEY_BYTES);
//		memcpy(gst_ConfigPermanentParams.mu8ar_AesKey2, DEFAULT_AES_KEY_2, AES_KEY_BYTES);
//		memcpy(gst_ConfigPermanentParams.mu8ar_AesKey3, DEFAULT_AES_KEY_3, AES_KEY_BYTES);
		
		memset(gst_ConfigPermanentParams.mu8ar_AesKey1,0,AES_KEY_BYTES);
		gst_ConfigPermanentParams.mu8ar_AesKey1[15] = 0x01;
		memset(gst_ConfigPermanentParams.mu8ar_AesKey2,0,AES_KEY_BYTES);
		gst_ConfigPermanentParams.mu8ar_AesKey2[15] = 0x02;
		memset(gst_ConfigPermanentParams.mu8ar_AesKey3,0,AES_KEY_BYTES);
		gst_ConfigPermanentParams.mu8ar_AesKey3[15] = 0x03;
		
		memcpy(gst_ConfigPermanentParams.mu8ar_SecurityTocken, SECURITY_TOCKEN, SECURITY_TOCKEN_BYTES);
		memcpy(gst_ConfigPermanentParams.mu8ar_DeviceName, DEVICE_NAME, DEVICE_NAME_BYTES);
		
//		memcpy(gst_ConfigPermanentParams.mu8ar_ManufactDate, MANUFACTURE_DATE, MANUFACTURE_DATE_BYTES);		
//		memcpy(bw_configPermanentParams.manufact_date_str, "01.04.13", sizeof(bw_configPermanentParams.manufact_date_str));
		gst_ConfigPermanentParams.mu8ar_ManufactDate[0]=25;
		gst_ConfigPermanentParams.mu8ar_ManufactDate[1]=07;
		gst_ConfigPermanentParams.mu8ar_ManufactDate[2]=20;
		gst_ConfigPermanentParams.mu8ar_ManufactDate[3]=14;
		gst_ConfigPermanentParams.mu8ar_ManufactDate[4]=1;
		gst_ConfigPermanentParams.mu8ar_ManufactDate[5]=1;
		gst_ConfigPermanentParams.mu8ar_ManufactDate[6]=1;
		gst_ConfigPermanentParams.mu8ar_ManufactDate[7]=6;
		
		memcpy(gst_ConfigPermanentParams.mu8ar_FccCertificate, FCC_CERTI, FCC_CERTI_BYTES);
		memcpy(gst_ConfigPermanentParams.mu8ar_BuildId, BUILD_ID, BUILD_ID_BYTES);
		
		FlashWrite(&mp_flash_PermPara,(uint8_t *)&gst_ConfigPermanentParams,sizeof(st_FlashPermanentParams_t));

	}
	else
	{		
		//Copy the Permenant Parameter from ROM to RAM. (BLE+ANT Application)
		memcpy(&gst_ConfigPermanentParams,gu8p_PeramParaConfigParamAddr,sizeof(st_FlashPermanentParams_t));
	}
	
	
	//********************************************************************************************//
	
	uint8_t PageNumber = 237;												//237 * 1024 = 0x3B400
	uint16_t PageSize = NRF_FICR->CODEPAGESIZE;
	gu8p_FlashDataOfPreviousVersion = (uint8_t *) (PageNumber * PageSize);	//0x3B400
	
	memcpy((uint8_t *)&gst_FlashDataOfPreviousVersion,gu8p_FlashDataOfPreviousVersion,sizeof(st_FlashDataOfPreviousVersion_t));
	
	if(gst_FlashDataOfPreviousVersion.mu8_flash_Status == 1)
	{
		memcpy((uint8_t *)gst_ConfigPermanentParams.mu8ar_SerialNumber,gst_FlashDataOfPreviousVersion.mu8ar_serial_num_str,SERIAL_NUM_BYTES);
		memcpy((uint8_t *)gst_ConfigPermanentParams.mu8ar_AntSerialNumber,gst_FlashDataOfPreviousVersion.mu8ar_ant_serial_num_str,ANT_SERIAL_NUMBER_BYTES);
		memcpy((uint8_t *)gst_ConfigPermanentParams.mu8ar_AesKey1,gst_FlashDataOfPreviousVersion.mu8ar_aes_key_1,AES_KEY_BYTES);
		memcpy((uint8_t *)gst_ConfigPermanentParams.mu8ar_AesKey2,gst_FlashDataOfPreviousVersion.mu8ar_aes_key_2,AES_KEY_BYTES);
		memcpy((uint8_t *)gst_ConfigPermanentParams.mu8ar_AesKey3,gst_FlashDataOfPreviousVersion.mu8ar_aes_key_3,AES_KEY_BYTES);
		memcpy((uint8_t *)gst_ConfigPermanentParams.mu8ar_SecurityTocken,gst_FlashDataOfPreviousVersion.mu8ar_security_token,SECURITY_TOCKEN_BYTES);
		gst_BleConfigDynamicParams.mu8_DeviceType = gst_FlashDataOfPreviousVersion.mu8_device_type;
		gst_BleConfigDynamicParams.mu8_Securitylevel = gst_FlashDataOfPreviousVersion.mu8_security_level;
		
		FlashWrite(&mp_flash_PermPara,(uint8_t *)&gst_ConfigPermanentParams,sizeof(st_FlashPermanentParams_t));
	}
		
	memcpy((uint8_t *)DevKit_SrNo,gst_FlashDataOfPreviousVersion.mu8ar_serial_num_str,SERIAL_NUM_BYTES);  //[KC]
}

//*******************************************************************//
// Print BLE Dynamic Parameter
//*******************************************************************//
void bleP2_ReadDynaConfiguration(void)
{	
	// Start address for configuration parameter storage area
	gu8p_BLEConfigParamAddr = (uint8_t *)mp_flash_BLE_DynPara.block_id;	 	
	
	// Check Flash has been initialzied or not? (BLE Application)
	memcpy((uint8_t *)&gst_BleConfigDynamicParams,gu8p_BLEConfigParamAddr,sizeof(st_BleFlashDynamicParams_t));
	
	if(gst_BleConfigDynamicParams.mu8_FlashStatus != 0x01)
	{
		#ifdef INIT_DBG
			DEBUG("\r\n**Write Default parameter in Flash**");
		#endif
		//*******************************************************************//
		// Flash Not Initialized so set Default BLE Dynamic Parameters to Flash
		//*******************************************************************//
		memset(&gst_BleConfigDynamicParams, 0, sizeof(gst_BleConfigDynamicParams));
		
		gst_BleConfigDynamicParams.mu8_FlashStatus = 1;
		
		gst_BleConfigDynamicParams.mu8_NumOfUser = 0;
		gst_BleConfigDynamicParams.mu8_UserRecordUpdateCounter=0;
		gst_BleConfigDynamicParams.mu8_SrDevStatus = 0;
		gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord = 0;
		gst_BleConfigDynamicParams.mu8_AuditTrailRecordRunning = 0;
		gst_BleConfigDynamicParams.mu8_AutoLockTimer = 0;
//		gst_BleConfigDynamicParams.mu8_BatteryLevel = 0;
		gst_BleConfigDynamicParams.mu8_TxPower = 4;	
		gst_BleConfigDynamicParams.mu8_AdvertiseInterval = 0;
		gst_BleConfigDynamicParams.mu8_Securitylevel = 2;
		gst_BleConfigDynamicParams.mu8_DeviceType = 0;		
		
		gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[0] = 0x0F;
		gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[1] = 0x00;
		gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[2] = 0x00;
		gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[3] = 0x01;
		gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[4] = 0x04;
		gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[5] = 0x01;
		gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[6] = 0x00;
		gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[7] = 0x00;		

		gst_BleConfigDynamicParams.light_intensity = 0;
		gst_BleConfigDynamicParams.light_status = 0;
		gst_BleConfigDynamicParams.heating_set_point_flash = 0;
		gst_BleConfigDynamicParams.cooling_set_point_flash = 0;
		gst_BleConfigDynamicParams.hardware_type_flash = 0;
		gst_BleConfigDynamicParams.relay_op_online_mode_flash =0;
		gst_BleConfigDynamicParams.dev_kit_sr_no_set_flg_flash = 0;
										
		FlashWrite(&mp_flash_BLE_DynPara,(uint8_t *)&gst_BleConfigDynamicParams,sizeof(st_BleFlashDynamicParams_t));
	}
	else
	{
		//Copy the BLE Dynamic Parameter from ROM to RAM. (BLE Application) --------------------------
		memcpy((uint8_t *)&gst_BleConfigDynamicParams,gu8p_BLEConfigParamAddr,sizeof(st_BleFlashDynamicParams_t));
	}
		light_intensity_value =gst_BleConfigDynamicParams.light_intensity; 	//Light intensity 0-100%
	
		light_status_flg = (light_intensity_value == 0)?false:gst_BleConfigDynamicParams.light_status;
	
		heating_set_point = gst_BleConfigDynamicParams.heating_set_point_flash;
		cooling_set_point = gst_BleConfigDynamicParams.cooling_set_point_flash;
		hardware_type     = gst_BleConfigDynamicParams.hardware_type_flash;
		humidity_sensor_presence = gst_BleConfigDynamicParams.humidity_sensor_presence_flash;
		tx_power = gst_BleConfigDynamicParams.mu8_TxPower;
		relay_op_online_mode = gst_BleConfigDynamicParams.relay_op_online_mode_flash;
		flash_Status = gst_BleConfigDynamicParams.flash_Status_flg;
		dev_kit_sr_no_set_flg = gst_BleConfigDynamicParams.dev_kit_sr_no_set_flg_flash;		
		
		if(dev_kit_sr_no_set_flg != 0 && dev_kit_sr_no_set_flg != 1)
		{
			dev_kit_sr_no_set_flg=0;
		}
}


void antP2_ClearDynaConfiguration(void)
{
	uint16_t lu16_temp=0;
	
	lu16_temp = gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum;
	
	memset(&gst_AntFlashParams,0x00,sizeof(gst_AntFlashParams));
		
	//Prepare RAM Copy with initial values to first time write in to flash.
	gst_AntFlashParams.mst_AntFlashHeader.mu8_FlashStatus=0;
	gst_AntFlashParams.mst_AntFlashHeader.mu8_NodePoweredOption=BATTERY_POWERED;
	gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum=lu16_temp;
	gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevType=CHANNEL_0_CHAN_ID_DEV_TYPE_0;
	gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevTransType=CHANNEL_0_CHAN_ID_TRANS_TYPE;
	gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress=0;
	memcpy((uint8_t *)&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],gu8ar_ANTDefaultSrNo,SR_NO_LENGTH);
	memset(&gst_AntFlashParams.mst_AntNodeFlashParams,0x00,sizeof(st_AntNodeFlashParams_t) * MAX_NODES_IN_NEXT_LAYER);

	gu8_NextSharedAddress=1;
	gu8_AddedSlaveNodeIndex=0;

	if(gu8_AntFlashWriteState==NO_ANT_FLASH_WRITE_OPERATION)
	{
		gu8_AntFlashWriteState=ANT_FLASH_WRITE_INITIAT;
		#ifdef ANT_FLASH_DEBUG_ENABLE
		DEBUG("NO_ANT_FLASH_WRITE_OPERATION\r\n");
		#endif
	}
	else
	{
		#ifdef ANT_FLASH_DEBUG_ENABLE
		DEBUG("ANT_FLASH_WRITE_IN_PROGRESS\r\n");
		#endif
	}
	gu8_AntFlashWriteReason=CLEAR_DYNA_PARA;
	
	#ifdef ANT_FLASH_DEBUG_ENABLE
	DEBUG("Clear ANT Parameters\r\n");
	DEBUG("\r\nFLASH WRITE REASON:");
	gen_PrintHex(gu8_AntFlashWriteReason);
	CRLF;
	#endif
}

void antP2_ReadDynaConfiguration(void)
{
	uint8_t lu8_i=0;
	
	// Start address for configuration parameter storage area
	gu8p_AntConfigParamAddr = (uint8_t *)mp_flash_ANT_DynPara.block_id;  	
	
	// Check Flash has been initialzied or not? (BLE Application)
	memcpy((uint8_t *)&gst_AntFlashParams.mst_AntFlashHeader,gu8p_AntConfigParamAddr,sizeof(st_AntFlashHeader_t));
	
	if(gst_AntFlashParams.mst_AntFlashHeader.mu8_FlashStatus == 0xFF)
	{
		#ifdef INIT_DBG
			DEBUG("\r\n**Write Default parameter in Flash**");
		#endif
		//*******************************************************************//
		// Flash Not Initialized so set Default BLE Dynamic Parameters to Flash
		//*******************************************************************//
		memset(&gst_AntFlashParams,0x00,sizeof(gst_AntFlashParams));
		
		//Prepare RAM Copy with initial values to first time write in to flash.
		gst_AntFlashParams.mst_AntFlashHeader.mu8_FlashStatus=0;
		gst_AntFlashParams.mst_AntFlashHeader.mu8_NodePoweredOption=BATTERY_POWERED;//MAINS_POWERED;
		gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum= CHANNEL_0_CHAN_ID_DEV_NUM;   // 0x01AC;
		gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevType=CHANNEL_0_CHAN_ID_DEV_TYPE_0;
		gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevTransType=CHANNEL_0_CHAN_ID_TRANS_TYPE;
		gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress=0;
		memcpy((uint8_t *)&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],gu8ar_ANTDefaultSrNo,SR_NO_LENGTH);
		memset(&gst_AntFlashParams.mst_AntNodeFlashParams,0x00,sizeof(st_AntNodeFlashParams_t) * MAX_NODES_IN_NEXT_LAYER);
		
		
		gu8_NextSharedAddress=1;
		gu8_AddedSlaveNodeIndex=0;
		
		#ifdef INIT_DBG
		DEBUG("Storing Default ANT Parameter\r\n");
		#endif
		
		if(gu8_AntFlashWriteState==NO_ANT_FLASH_WRITE_OPERATION)
		{
			gu8_AntFlashWriteState=ANT_FLASH_WRITE_INITIAT;
			#ifdef ANT_FLASH_DEBUG_ENABLE
			DEBUG("NO_ANT_FLASH_WRITE_OPERATION\r\n");
			#endif
		}
		else
		{
			#ifdef ANT_FLASH_DEBUG_ENABLE
			DEBUG("ANT_FLASH_WRITE_IN_PROGRESS\r\n");
			#endif
		}
		gu8_AntFlashWriteReason=DEFAULT_PARA_WRITE;
	
		#ifdef ANT_FLASH_DEBUG_ENABLE
		DEBUG("\r\nFLASH WRITE REASON:");
		gen_PrintHex(gu8_AntFlashWriteReason);
		CRLF;
		#endif
	}
	else
	{
		//Copy the ANT Dynamic Parameter from ROM to RAM. (ANT Application) --------------------------
		memcpy((uint8_t *)&gst_AntFlashParams,gu8p_AntConfigParamAddr,sizeof(st_AntFlashParmas_t));
		
		//Find # of Unregistered Node to GW
		gu8_AntNoOfUnRegisterNode=0;
		for(lu8_i=0;lu8_i<MAX_NODES_IN_NEXT_LAYER;lu8_i++)
		{
			if(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_RegisterStatus==E_DEV_NOT_IN_DB)
			{
				gu8_AntNoOfUnRegisterNode++;
				b.mu8_AntSendNodeInfoToMaster=1;
			}
		}
		
		//Creat NextLayerNodesSharedAddress Bitmap
		gu16_AntNextLayerNodesPollStatus=0;
		for(lu8_i=0;lu8_i<MAX_NODES_IN_NEXT_LAYER;lu8_i++)
		{
			if(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_ParentLayerNumber==(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber+1))
			{
				gu16_AntNextLayerNodesPollStatus |= (1<<gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_DeviceSharedAddress);
			}
		}
		
		//Get Next Available Free Shared Address
		antI4_GetNextSharedAddress();
		
		#ifdef ANT_DEBUG_ENABLE
		DEBUG("REG_STATE=");
		gen_PrintHex(gu8_AntRegState);
		CRLF;
		#endif
		
		//Get current Empty Structure index: it will be used to store newly discovered slave.
		if(FAIL == antI4_GetNodeParamsWrIndx()) 
		{

		}
	}
		
	
	#ifdef DISP_DEVICE_INFO
		//Just for Testing Array Size	
//		DEBUG("SizeOf gu8ar_AntBurstTxBuffer=");
//		gen_PrintDec(sizeof(st_AntFlashParmas_t));
//		CRLF;
		CRLF;CRLF;
		DEBUG("\r\n****************************");
		DEBUG("\r\n** ANT Dynamic Parameters **");
		DEBUG("\r\n****************************");
		DEBUG("\r\nSelf Info\r\n");

		DEBUG("SrNo=0x");
		gen_PrintHexStr(&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],4);
		
		DEBUG(", SA=");
		gen_PrintDec(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress);

		DEBUG(", PSrNo=0x");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[3]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[2]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[1]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[0]);
		
		DEBUG(", PSA=");
		gen_PrintDec(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentSharedAddress);

		DEBUG(", PLayerNo=");
		gen_PrintDec(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber);
		CRLF;
		
		DEBUG("Ch ID=0x");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum>>8);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum);
		DEBUG("  ");
		gen_PrintDec(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevType);
		DEBUG("  ");
		gen_PrintDec(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevTransType);
		CRLF;
		
		DEBUG("Operating Frequency CH0:");
		gen_PrintDec(CH_FREQUENCY0);
		CRLF;
		
		DEBUG("Operating Frequency CH1:");
		gen_PrintDec(CH_FREQUENCY1);
		CRLF;
		
		DEBUG("NextSA=");
		gen_PrintDec(gu8_NextSharedAddress);
		CRLF;
		
		DEBUG("POWER OPTION=");
		if(gst_AntFlashParams.mst_AntFlashHeader.mu8_NodePoweredOption==MAINS_POWERED)
		{
			DEBUG("MAINS\r\n");
		}
		else
		{
			DEBUG("BATTERY\r\n");
		}
		
		//For Slave Device is Master on CH1
		for(lu8_i=0;lu8_i<MAX_NODES_IN_NEXT_LAYER;lu8_i++)
		{
			gen_PrintDec(lu8_i+1);
			
			DEBUG(".SA=");
			gen_PrintDec(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_DeviceSharedAddress);
			
			DEBUG(", SrNo=0x");
			gen_PrintHexStr(&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_DeviceSrNo[0],4);
			
			DEBUG(", PSA=");
			gen_PrintDec(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_ParentSharedAddress);
			
			DEBUG(", PSrNo=");
			gen_PrintHexStr(&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_ParentSrNo[0],4);
				
			DEBUG(", PLayerNo=");
			gen_PrintDec(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_ParentLayerNumber);
			
			DEBUG(", Reg=");
			gen_PrintDec(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_RegisterStatus);

			CRLF;
		}
		
		DEBUG("NoOfUnreg Node=");
		gen_PrintDec(gu8_AntNoOfUnRegisterNode);
		CRLF;
		
		DEBUG("NEXT Store IND=");
		gen_PrintDec(gu8_AddedSlaveNodeIndex);
		CRLF;
		
		DEBUG("Next Layer Shared Address mapping=0x");
		gen_PrintHex(gu16_AntNextLayerNodesPollStatus>>8);
		gen_PrintHex(gu16_AntNextLayerNodesPollStatus);
		CRLF;
	#endif
}

//*****************************************************************************
//  Update configuration Parameter in flash
//*****************************************************************************
void dev_kit_configParamUpdate(void)
{
		gst_BleConfigDynamicParams.light_intensity=light_intensity_value; 	
		gst_BleConfigDynamicParams.light_status=light_status_flg;
		gst_BleConfigDynamicParams.hardware_type_flash = hardware_type;
		gst_BleConfigDynamicParams.humidity_sensor_presence_flash = humidity_sensor_presence;
		gst_BleConfigDynamicParams.heating_set_point_flash = heating_set_point;
		gst_BleConfigDynamicParams.cooling_set_point_flash = cooling_set_point;
		gst_BleConfigDynamicParams.relay_op_online_mode_flash = relay_op_online_mode;
		gst_BleConfigDynamicParams.flash_Status_flg = true;
		gst_BleConfigDynamicParams.dev_kit_sr_no_set_flg_flash = dev_kit_sr_no_set_flg; 
		gst_BleConfigDynamicParams.mu8_TxPower = tx_power; 
	
	if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
			BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
	
		//memcpy(&gst_ConfigPermanentParams.mu8ar_SerialNumber,&DevKit_SrNo[0],DEV_KIT_SR_NO_BYTES);
		//FlashWrite(&mp_flash_BLE_DynPara,(uint8_t *)&gst_BleConfigDynamicParams,sizeof(st_BleFlashDynamicParams_t));
}
