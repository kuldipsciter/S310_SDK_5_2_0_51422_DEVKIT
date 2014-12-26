#include <string.h>
#include <stdio.h>
#include "simple_uart.h"
#include "AntEventHandler.h"
#include "devkit\devkit_debug.h"
#include "devkit\devkit_variable.h"
#include "main.h"
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Extern variable
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


extern st_FlashPermanentParams_t gst_ConfigPermanentParams;
extern st_BleFlashDynamicParams_t gst_BleConfigDynamicParams;
#define INIT_DBG 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// local variable
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

char	debugstr[DEBUG_BUFFER_SIZE];

void gen_PrintRcvData(uint8_t * p_event_message_buffer)
{
	uint8_t lu8_i,lu8_k,Tmp[4]={0};
	for(lu8_i=0; lu8_i<(p_event_message_buffer[ANT_LENGH_ID]+2); lu8_i++)
	{
		Tmp[0] = ( ( p_event_message_buffer[lu8_i]>> 4 ) & 0xF);
		Tmp[1] = ( ( p_event_message_buffer[lu8_i]) & 0xF);	

		for(lu8_k=0; lu8_k<2; lu8_k++)
		{
			if ( Tmp[lu8_k] < 10 )
			{
				Tmp[lu8_k] = Tmp[lu8_k] + 0x30;
			}
			else
			{
				Tmp[lu8_k] = Tmp[lu8_k] + 0x37;
			}
		}
		
		simple_uart_put(Tmp[0]);
		simple_uart_put(Tmp[1]);
		simple_uart_put('.');							
	}
}

void gen_PrintHex(uint8_t adata)
{
	uint8_t k,Tmp[4]={0};

	Tmp[0] = ((adata>>4) & 0xF);
	Tmp[1] = (adata & 0xF);	

	for ( k = 0; k < 2; k++ )
	{
		if ( Tmp[k] < 10 )
		{
			Tmp[k] = Tmp[k] + 0x30;
		}
		else
		{
			Tmp[k] = Tmp[k] + 0x37;
		}
	}
	simple_uart_put(Tmp[0]);
	simple_uart_put(Tmp[1]);						
}

void gen_PrintDec(uint8_t adata)
{
	uint8_t Tmp[3]={0};
	
	Tmp[2]	= (adata%10) + 0x30;
	adata/=10;
	
	Tmp[1]	= (adata%10) + 0x30;
	adata/=10;
	
	Tmp[0]	= (adata%10) + 0x30;
	
	simple_uart_put(Tmp[0]);
	simple_uart_put(Tmp[1]);	
	simple_uart_put(Tmp[2]);	
}

void gen_PrintHexStr(uint8_t *str, uint8_t NoOfByte)
{
	while(NoOfByte)
	{
		gen_PrintHex(*str);
		str++;
		NoOfByte--;
		if(NoOfByte) simple_uart_put('.');
	}					
}

void gen_PrintCharStr(uint8_t *str, uint8_t NoOfByte)
{
	uint8_t lu8_i;
	
	for (lu8_i = 0; lu8_i < NoOfByte; lu8_i++)
	{
		simple_uart_put(*str);
		str++;
	}				
}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Display Configuration Parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void display_config_data(void)
{
			uint8_t lu8_i,lu8_UserCount;
			
	#ifdef INIT_DBG
	
		// Flash has been initialzied already, so just read back for syncing
		DEBUG("\r\n**Permanent Parameters**");
		
		DEBUG("\r\nFlash_Status = ");
		gen_PrintDec(gst_ConfigPermanentParams.mu8_FlashStatus);
		
		DEBUG("\r\nSerial_number = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_SerialNumber[0],SERIAL_NUM_BYTES);
		//gen_PrintCharStr(&gst_BleConfigDynamicParams.mu8ar_SerialNumber[0],SERIAL_NUM_BYTES);

		DEBUG("\r\nAntSerialNumber = ");
		gen_PrintHexStr(&gst_ConfigPermanentParams.mu8ar_AntSerialNumber[0],ANT_SERIAL_NUMBER_BYTES);

		DEBUG("\r\nManufactName = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_ManufactName[0],MANUFACTURE_NAME_BYTES);
		
		DEBUG("\r\nModelNumber = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_ModelNumber[0],MODEL_NUM_BYTES);

		DEBUG("\r\nNetworkKey = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_NetworkKey[0],NETWORK_KEY_BYTES);

		DEBUG("\r\nHardwareVersion = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_HardwareVersion[0],HW_VER_BYTES);

		DEBUG("\r\nSoftwareVersion = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_SoftwareVersion[0],SW_VER_BYTES);

		DEBUG("\r\nAES_KEY_1 = ");
		gen_PrintHexStr(&gst_ConfigPermanentParams.mu8ar_AesKey1[0],AES_KEY_BYTES);

		DEBUG("\r\nAES_KEY_2 = ");
		gen_PrintHexStr(&gst_ConfigPermanentParams.mu8ar_AesKey2[0],AES_KEY_BYTES);

		DEBUG("\r\nAES_KEY_3 = ");
		gen_PrintHexStr(&gst_ConfigPermanentParams.mu8ar_AesKey3[0],AES_KEY_BYTES);

		DEBUG("\r\nSecurityTocken = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_SecurityTocken[0],SECURITY_TOCKEN_BYTES);

		DEBUG("\r\nDeviceName = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_DeviceName[0],DEVICE_NAME_BYTES);

		DEBUG("\r\nManufactDate = ");
		gen_PrintHexStr(&gst_ConfigPermanentParams.mu8ar_ManufactDate[0],MANUFACTURE_DATE_BYTES);

		DEBUG("\r\nFccCertificate = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_FccCertificate[0],FCC_CERTI_BYTES);

		DEBUG("\r\nBuildId = ");
		gen_PrintCharStr(&gst_ConfigPermanentParams.mu8ar_BuildId[0],BUILD_ID_BYTES);
	
	#endif
	
	#ifdef INIT_DBG
	
		DEBUG("\r\n**BLE Dynamic Parameters**");

		DEBUG("\r\nNumOfUser = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_NumOfUser);
		
		DEBUG("\r\nUserRecordUpdateCounter = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_UserRecordUpdateCounter);
		
		DEBUG("\r\nSrDevStatus = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_SrDevStatus);
		
		DEBUG("\r\nTotalAuditTrailLogRecord = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord);
		
		DEBUG("\r\nAuditTrailRecordRunning = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_AuditTrailRecordRunning);

		DEBUG("\r\nAutoLockTimer = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_AutoLockTimer);
		
		DEBUG("\r\nBatteryLevel = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_BatteryLevel);
		
		DEBUG("\r\nTxPower = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_TxPower);
		
		DEBUG("\r\nAdvertiseInterval = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_AdvertiseInterval);
		
		DEBUG("\r\nSecuritylevel = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_Securitylevel);
		
		DEBUG("\r\nDeviceType = ");
		gen_PrintDec(gst_BleConfigDynamicParams.mu8_DeviceType);
				
		DEBUG("\r\n**User Record**");
		
		lu8_UserCount = (gst_BleConfigDynamicParams.mu8_NumOfUser > MAX_USER) ? MAX_USER: gst_BleConfigDynamicParams.mu8_NumOfUser;

		for (lu8_i = 0; lu8_i < lu8_UserCount; lu8_i++)
		{
			DEBUG("\r\nUser Device ID_");
			gen_PrintDec(lu8_i+1);
			DEBUG(" = ");
			gen_PrintHexStr(&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[0],USER_DEVICE_ID_BYTES);
			DEBUG("\r\n");
		}
		
		lu8_UserCount = (gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord > MAX_USER) ? MAX_USER: gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord;
		
		DEBUG("\r\n**Audit Log Record**");
		for(lu8_i = 0; lu8_i < lu8_UserCount; lu8_i++)
		{		
			DEBUG("\r\nAuditLogRecord[");
			gen_PrintDec(lu8_i+1);
			DEBUG("].timeStamp(M:D:Y:H:m) = ");
			gen_PrintDec(gst_BleConfigDynamicParams.mst_AuditLogRecord[lu8_i].mst_TimeStamp.mu8_Month);
			simple_uart_put(':');
			gen_PrintDec(gst_BleConfigDynamicParams.mst_AuditLogRecord[lu8_i].mst_TimeStamp.mu8_Day);
			simple_uart_put(':');
			gen_PrintDec(gst_BleConfigDynamicParams.mst_AuditLogRecord[lu8_i].mst_TimeStamp.mu8_Year);
			simple_uart_put(':');
			gen_PrintDec(gst_BleConfigDynamicParams.mst_AuditLogRecord[lu8_i].mst_TimeStamp.mu8_Hour);
			simple_uart_put(':');
//			gen_PrintDec(gst_BleConfigDynamicParams.mst_AuditLogRecord[lu8_i].mst_TimeStamp.mu8_Minute);	
			
			DEBUG("\r\nUser Device ID_");
			gen_PrintDec(lu8_i+1);
			DEBUG(" = ");
			gen_PrintHexStr(&gst_BleConfigDynamicParams.mst_AuditLogRecord[lu8_i].mu8_UserDeviceId[0],USER_DEVICE_ID_BYTES);	

			DEBUG("\r\nOperation Type ");
			gen_PrintDec(lu8_i+1);
			DEBUG(" = ");
			gen_PrintHexStr(&gst_BleConfigDynamicParams.mst_AuditLogRecord[lu8_i].mu8_OperationType,1);
			
			DEBUG("\r\nOperation Status ");
			gen_PrintDec(lu8_i+1);
			DEBUG(" = ");
			gen_PrintHexStr(&gst_BleConfigDynamicParams.mst_AuditLogRecord[lu8_i].mu8_OperationStatus,1);
			DEBUG("\r\n");
		}
	#endif
		//Just for Testing Array Size	
//		DEBUG("SizeOf gu8ar_AntBurstTxBuffer=");
//		gen_PrintDec(sizeof(st_AntFlashParmas_t));
//		CRLF;
		CRLF;CRLF;
		DEBUG("\r\n****************************");
		DEBUG("\r\n** ANT Dynamic Parameters **");
		DEBUG("\r\n****************************");
		DEBUG("\r\nSelf Info\r\n");

		DEBUG("SrNo=");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[1]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[2]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[3]);

		DEBUG(", SA=");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress);

		DEBUG(", PSrNo=");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[3]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[2]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[1]);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[0]);
		
		DEBUG(", PSA=");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentSharedAddress);

		DEBUG(", PLayerNo=");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber);
		CRLF;
		
		DEBUG("Ch ID=");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum>>8);
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum);
		DEBUG("  ");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevType);
		DEBUG("  ");
		gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevTransType);
		CRLF;
		
		DEBUG("Operating Frequency CH0:");
		gen_PrintDec(CH_FREQUENCY0);
		CRLF;
		
		DEBUG("Operating Frequency CH1:");
		gen_PrintDec(CH_FREQUENCY1);
		CRLF;
		
		DEBUG("NextSA=");
		gen_PrintHex(gu8_NextSharedAddress);
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
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_DeviceSharedAddress);
			
			DEBUG(", SrNo=");
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_DeviceSrNo[0]);
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_DeviceSrNo[1]);
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_DeviceSrNo[2]);
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_DeviceSrNo[3]);
			
			DEBUG(", PSA=");
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_ParentSharedAddress);
			DEBUG(", PSrNo=");
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_ParentSrNo[0]);
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_ParentSrNo[1]);
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_ParentSrNo[2]);
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_ParentSrNo[3]);
			
			DEBUG(", PLayerNo=");
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_ParentLayerNumber);
			
			DEBUG(", Reg=");
			gen_PrintHex(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_RegisterStatus);

			CRLF;
		}
		
		DEBUG("NoOfUnreg Node=");
		gen_PrintHex(gu8_AntNoOfUnRegisterNode);
		CRLF;
		
		DEBUG("NEXT Store IND=");
		gen_PrintHex(gu8_AddedSlaveNodeIndex);
		CRLF;
		
		DEBUG("Next Layer Shared Address mapping=");
		gen_PrintHex(gu16_AntNextLayerNodesPollStatus>>8);
		gen_PrintHex(gu16_AntNextLayerNodesPollStatus);
		CRLF;
				
	
}

