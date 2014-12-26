/*................................................................................/
Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.

* The information contained herein is property of Nordic Semiconductor ASA.
* Terms and conditions of usage are described in detail in NORDIC
* SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
*
* Licensees are granted free, non-transferable use of the information. NO
* WARRANTY of ANY KIND is provided. This heading must NOT be removed from
* the file.


File Name : AntEventHandler.c

File type : C source File

Compiler Version : Keil Uvision Ver 4.73.0.0

Created on : 17/08/2014

File Description : Handles all the events related to ANT
*.................................................................................*/

#include <string.h>
#include "ant_parameters.h"
#include "ant_interface.h"
#include "ant_error.h"
#include "nrf_error.h"
#include "simple_uart.h"
#include "AntEventHandler.h"
#include "devkit/devkit_constant.h"
#include "nrf_gpio.h"
#include "boards/ras_1_6.h"
#include "main.h"

#include "aes\aes.h"
#include "led.h"
#include "pwm.h"

//***************************************************************************************
//								variables Definations
//***************************************************************************************

/* ANT event message buffer */
uint8_t event_message_buffer[ANT_EVENT_MSG_BUFFER_MIN_SIZE]; 

/* ANT Network Key */
//const uint8_t NetworkKey[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
st_Antbitfield_t  b;
st_AntFlashParmas_t gst_AntFlashParams;
st_AntNodeFlashParams_t gst_AntTempPara;

/* Channel TX buffer */
uint8_t gu8ar_AntCHTxbuffer[NO_OF_CHANNEL][PAYLOAD_BUFFER_SIZE]={0};
uint8_t gu8ar_AntBurstRxBuffer[NO_OF_CHANNEL][BURST_BUFFER_SIZE]={0};  			///< Primary data for Burst receive buffer on CH0
uint8_t gu8ar_AntBurstTxBuffer[NO_OF_CHANNEL][BURST_BUFFER_SIZE]={0};  			///< Primary data for Burst transmit buffer on CH0
uint8_t gu8ar_AntTempBuffer1[50];  												///< Common Use Buffer

uint8_t gu8_AntAddrAssignState=0;
uint8_t gu8_AntRegState = ADD_AVAILABEL;
uint8_t gu8_BurstRcvInd[NO_OF_CHANNEL]={0};
uint8_t gu8_BurstSendCnt=0;
uint8_t gu8_BurstMsgInProgress=0;
uint8_t gu8_StatusUpdateTimer=10;
uint8_t gu8_tempSharedAddr=0,gu8_tempPSharedAddr=0,gu8_tempPLayerNo=0,gu8ar_tempSrNo[4];
uint8_t gu8_L2NodePollingInd=0;
uint8_t gu8_AntFlashWriteReason=0;
uint8_t gu8_AntBurstMsgProgressState[NO_OF_CHANNEL]={0};
uint8_t gu8_AntGetEpochTimeMsgState=0;
uint8_t gu8_AntTxBurstMsgSize[NO_OF_CHANNEL]={0};
uint8_t gu8_AddrAssignTimeout[NO_OF_CHANNEL]={0};
uint8_t gu8_AdditionOfIndirectNodes=0;
uint16_t gu16_AntNextLayerNodesPollStatus=0; //Initialy(0x0000000X) bit map variable holds the status of Immidiate next layer Node : Shared address of device is same as bit position */
										     //value of any bit 1: Node is Active & 0:Node is Invactive.
uint8_t gu8ar_NodePollingCount[ANT_MAX_NODE_IN_IMMIDIATE_NEXT_LAYER]={0};
uint8_t gu8_NextSharedAddress=0;		//Hold the shared address to be used by master for assinging Shared address.	
const uint8_t gu8ar_NullSrNumber[4] = {0x00,0x00,0x00,0x00};
uint8_t gu8_AddedSlaveNodeIndex=0;
uint8_t gu8_PollingSharedAddr=0;
uint8_t gu8_AntSelfConnectionTimeout=0;
uint8_t gu8_Ch0_CloseFlag=0;
uint8_t gu8_AntDeletNextLayerNode[2]={0};
uint32_t gu8_EpochTime=0;
uint8_t gu8_AntReqSharedAddrAtCh1=0;
uint8_t gu8_AntReqSrNoAtCh1[4]={0};
uint8_t	gu8_AntReqTypeAtCh1=0;

uint8_t gu8_AntBurstTryCntr[NO_OF_CHANNEL]={0};
//uint8_t gu8_AntSendResToMasterTryCntr=0;
//uint8_t gu8_AntFwdMsgToMasterTryCntr=0;
//uint8_t gu8_AntSendNodeInfoToMasterTryCntr=0;
//uint8_t gu8_AntSendMobileMsgToMasterCntr_ch0=0;
//uint8_t gu8_AntSendMobileMsgToMasterCntr_ch1=0;
//uint8_t gu8_AntSendMsgToSlaveTryCntr=0;

uint8_t gu8_AntStatusPollingRcv=0;
uint8_t gu8_AntStatusUpdateCount=0;
uint8_t *gu8p_AntBleMsgptr;
uint8_t gu8p_AntBleMsglen=0;
uint8_t gu8_RemoteOpStatus=0;
uint8_t gu8_RemoteOpRequest=0;
uint8_t gu8_AntFlashWriteState=0;

uint8_t gu8_AntLockCurrentStatus = 0,gu8_AntLockLastStatus = 0;

uint8_t gu8_AntAsyncStatus = 0;

#ifdef ANT_OPERATION_COUNT_ENABLE
volatile uint16_t gu8_AntBurstRcvCount=0,gu8_AntBurstRetryCount=0,gu8_AntBurstFailCount=0,gu8_AntBurstOKCount=0;
#endif

const uint8_t gu8_AntTestmessage[]="Sciter Technology";


//***************************************************************************************
//								Devkit
//***************************************************************************************
uint8_t ant_request = NO_ANT_MSG; 
uint8_t ant_relay_status = 2;
extern bool relay_status_flg;
extern bool solenoid_status;
extern bool light_status_flg;
extern uint32_t light_intensity_value;
extern uint32_t STATUS_SW1;
//***************************************************************************************
//Shivsatya
//***************************************************************************************
//								Function Declaration
//***************************************************************************************

void antPx_ProcessBroadCastMessageAtCH0(uint8_t * p_event_message_buffer)
{
	uint32_t err_code;
	
	if(gu8_AntStatusUpdateCount>5) 
		gu8_AntStatusUpdateCount=0;
	
	switch(p_event_message_buffer[APP_MSG_IDX_ID])
	{
		case ADD_AVAILABEL: 
			
			if(!gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress)
			{
				if(!gu8_AntAddrAssignState)
				{
					//Store temperary Serial and Shared address
					gu8_tempSharedAddr=p_event_message_buffer[10];
					gu8_tempPLayerNo=p_event_message_buffer[9];
					gu8ar_tempSrNo[0]=p_event_message_buffer[8];
					gu8ar_tempSrNo[1]=p_event_message_buffer[7];
					gu8ar_tempSrNo[2]=p_event_message_buffer[6];
					gu8ar_tempSrNo[3]=p_event_message_buffer[5];
					
					//Send Request for Shared Address
					gu8ar_AntCHTxbuffer[CH_0][0]=0x00;
					gu8ar_AntCHTxbuffer[CH_0][1]=REQUEST_ADD;
					gu8ar_AntCHTxbuffer[CH_0][2]=0x00;
					gu8ar_AntCHTxbuffer[CH_0][3]=0x00;
					gu8ar_AntCHTxbuffer[CH_0][4]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0];
					gu8ar_AntCHTxbuffer[CH_0][5]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[1];
					gu8ar_AntCHTxbuffer[CH_0][6]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[2];
					gu8ar_AntCHTxbuffer[CH_0][7]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[3];
					err_code=sd_ant_acknowledge_message_tx(CHANNEL_0,8,&gu8ar_AntCHTxbuffer[CH_0][0]);
					//APP_ERROR_CHECK(err_code);

					#ifdef ANT_DEBUG_ENABLE
					DEBUG("S1,T0\r\n");
					#endif
					
					#ifdef ANT_DATA_DEBUG_ENABLE
					gen_PrintHexStr(&gu8ar_AntCHTxbuffer[CH_0][0],8);
					CRLF;
					#endif

					gu8_AntAddrAssignState=1;
					
					gu8_AddrAssignTimeout[CH_0]=10;
				}
			}
				
		break;
			
		case BUSY_ACQUIRING:  
			
			if(!gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress)
			{
				if(gu8_AntAddrAssignState==2)
				{
					if(!memcmp(&p_event_message_buffer[SR_NO_IDX_ID],&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],SR_NO_LENGTH))
					{
						gu8_tempPSharedAddr=p_event_message_buffer[6];
						
						//Send Request for Confirm Aquire
						gu8ar_AntCHTxbuffer[CH_0][0]=0x00;
						gu8ar_AntCHTxbuffer[CH_0][1]=CONFIRM_ACQUIRE;
						gu8ar_AntCHTxbuffer[CH_0][2]=0x00;
						gu8ar_AntCHTxbuffer[CH_0][3]=0x00;
						gu8ar_AntCHTxbuffer[CH_0][4]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0];
						gu8ar_AntCHTxbuffer[CH_0][5]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[1];
						gu8ar_AntCHTxbuffer[CH_0][6]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[2];
						gu8ar_AntCHTxbuffer[CH_0][7]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[3];
						err_code=sd_ant_acknowledge_message_tx(CHANNEL_0,8,&gu8ar_AntCHTxbuffer[CH_0][0]);
						//APP_ERROR_CHECK(err_code);
						
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("S3,T0\r\n");
						#endif
						
						#ifdef ANT_DATA_DEBUG_ENABLE
						gen_PrintHexStr(&gu8ar_AntCHTxbuffer[CH_0][0],8);
						CRLF;
						#endif
						
						gu8_AntAddrAssignState=3;
						
						gu8_AddrAssignTimeout[CH_0]=10;
					}
					else
					{
						gu8_AntAddrAssignState=0;
						
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("S0\r\n");
						#endif
					}
				}
			}
				
		break;
		
		case ADD_FULL: 
			
			if(!gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress)
			{
				if(!gu8_Ch0_CloseFlag)
				{
					#ifdef ANT_DEBUG_ENABLE
					DEBUG("CH0 CLOSE CMD\r\n");
					#endif
					
					/* Close Channel */
					if(b.mu8_AntChannelOpenState_ch0==1)
					{
						err_code = sd_ant_channel_close(CHANNEL_0);
						APP_ERROR_CHECK(err_code);
					
						gu8_Ch0_CloseFlag = 1;
					}
				}
			}
		
		break;
		
		default:

		break;
	}
}

void antPx_ProcessAckMessageAtCH0(uint8_t * p_event_message_buffer)
{
	uint32_t err_code;
	uint8_t lu8_DevSharedAdd=0;
	
	switch(p_event_message_buffer[APP_MSG_IDX_ID])
	{
		case STATUS_UPDATE_REQ: 
			
			if(!memcmp(&p_event_message_buffer[SR_NO_IDX_ID],&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],SR_NO_LENGTH))
			{
				gu8_AntSelfConnectionTimeout=SELF_CONNECTION_TIMEOUT;
								
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("SELF POLL\r\n");
				#endif
				
				gu8_AntStatusPollingRcv=1;
				
				gu8_AntStatusUpdateCount++;
				
				if(b.mu8_AntSendAsyncMsgToMaster==1)
				{
					if(gu8_AntBurstMsgProgressState[CH_0] == NO_BURST_MESSAGE)
					{
						antPx_SendAsyncMsgToMaster();								//Send Asynchronous Event message to Master
					}
				}
				else if(b.mu8_AntSendResToMasterForRemoteReq==1)   //Send ACK to Master for request received from Remote
				{
					if(gu8_AntBurstMsgProgressState[CH_0] == NO_BURST_MESSAGE)
					{
						//gu8_AntBurstTryCntr[CH_0]--;
						//if(!gu8_AntBurstTryCntr[CH_0])
						antPx_SendResToMasterForRemoteReq(gu8_RemoteOpRequest);		//Send Ack to Master for Remote Request
					}
					#ifdef ANT_DEBUG_ENABLE
					DEBUG("B2\r\n");
					#endif
				}
				else if(b.mu8_AntFwdMsgToMaster==1)
				{
					if(gu8_AntBurstMsgProgressState[CH_0] == NO_BURST_MESSAGE)
					{
						antPx_FwdMsgToMaster();										//Forward message to Master
					}
				}
				else if((b.mu8_AntSendMobileMsgToMaster_ch0==1) || (b.mu8_AntSendMobileMsgToMaster_ch1==1))
				{
					if(gu8_AntBurstMsgProgressState[CH_0] == NO_BURST_MESSAGE)
					{
						err_code = sd_ant_burst_handler_request(CHANNEL_0, gu8_AntTxBurstMsgSize[CH_0] , &gu8ar_AntBurstTxBuffer[CH_0][0], BURST_SEGMENT_START | BURST_SEGMENT_END);
						APP_ERROR_CHECK(err_code);	

						gu8_AntBurstMsgProgressState[CH_0] = BURST_IN_PROGRESS;
						
						gu8_AntBurstTryCntr[CH_0]=BURST_MSG_RETRY_CH0_COUNTER;
					}
				}
				else if(b.mu8_AntSendNodeInfoToMaster==1)	
				{ 
					if(gu8_AntBurstMsgProgressState[CH_0] == NO_BURST_MESSAGE)
					{
						antPx_SendAddNodeRequest();								//Send Node add request
					}
				}
				else if(gu8_AntDeletNextLayerNode[NEAR_DEVICE]==MESSAGE_INITIATED)
				{
					//Send Node Delete message to Upper Layer
					err_code=sd_ant_acknowledge_message_tx(CHANNEL_0,8,&gu8ar_AntCHTxbuffer[CH_0][0]);
					APP_ERROR_CHECK(err_code);
					
					#ifdef ANT_DATA_DEBUG_ENABLE
					gen_PrintHexStr(&gu8ar_AntCHTxbuffer[CH_0][0],8);
					CRLF;
					#endif
					
					gu8_AntDeletNextLayerNode[NEAR_DEVICE]=MESSAGE_IN_PROGRESS;
				}
				else if(gu8_AntDeletNextLayerNode[FAR_DEVICE]==MESSAGE_INITIATED)
				{
					//Send Node Delete message to Upper Layer
					err_code=sd_ant_acknowledge_message_tx(CHANNEL_0,8,&gu8ar_AntCHTxbuffer[CH_0][0]);
					APP_ERROR_CHECK(err_code);
					
					#ifdef ANT_DATA_DEBUG_ENABLE
					gen_PrintHexStr(&gu8ar_AntCHTxbuffer[CH_0][0],8);
					CRLF;
					#endif
					
					gu8_AntDeletNextLayerNode[FAR_DEVICE]=MESSAGE_IN_PROGRESS;
				}
				else if(b.mu8_AntGetEpochTime==1)
				{
					if(gu8_AntGetEpochTimeMsgState==MESSAGE_INITIATED)
					{
						memset(&gu8ar_AntCHTxbuffer[CH_0][0],0,PAYLOAD_BUFFER_SIZE);
						
						//Shared Address
						gu8ar_AntCHTxbuffer[CH_0][0] = gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
						
						//Command ID
						gu8ar_AntCHTxbuffer[CH_0][1] = EPOCH_TIME_REQ;		//Command ID
						
						//Send Epoch request to SR Bridge
						err_code=sd_ant_acknowledge_message_tx(CHANNEL_0,8,&gu8ar_AntCHTxbuffer[CH_0][0]);
						APP_ERROR_CHECK(err_code);
						
						#ifdef ANT_EPOCH_LOG_ENABLE
						DEBUG("EPOCH REQ SEND @ CH0:");
						gen_PrintHexStr(&gu8ar_AntCHTxbuffer[CH_0][0],8);
						CRLF;
						#endif
						
						gu8_AntGetEpochTimeMsgState=MESSAGE_IN_PROGRESS;
					}
				}
			}
		
		break;
			
		case EPOCH_TIME_RES:
			
			if(p_event_message_buffer[6]==1)
			{
				b.mu8_EpochTimeValidFlag=1;
				memcpy((uint8_t*)&gu8_EpochTime,&p_event_message_buffer[7],4);
			}		
			
			#ifdef ANT_EPOCH_LOG_ENABLE
			DEBUG("EPOCH RES @ CH0:");
			gen_PrintHexStr(&p_event_message_buffer[3],8);
			CRLF;
			#endif
			
		break;
		
		case DELETE_NODE_REQ:
			
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("DELETE_NODE:");
			gen_PrintHexStr(&p_event_message_buffer[3],8);
			CRLF;
			#endif
		
			if(!memcmp(&p_event_message_buffer[SR_NO_IDX_ID],&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],SR_NO_LENGTH))
			{
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("FOR SELF\r\n");
				#endif
				
				flash_ClearPage(mp_flash_ANT_DynPara);		
				b.mu8_AntDataEraseFlag=1;
			}
			else
			{
				if(SUCCESS==antP1_SearchDevice(&lu8_DevSharedAdd, &p_event_message_buffer[SR_NO_IDX_ID]))
				{
					gu8_AntReqSharedAddrAtCh1=lu8_DevSharedAdd;
					memcmp(&gu8_AntReqSrNoAtCh1[0],&p_event_message_buffer[SR_NO_IDX_ID],SR_NO_LENGTH);
					gu8_AntReqTypeAtCh1=DELETE_NODE_REQ;
				}
				else
				{
					#ifdef ANT_DEBUG_ENABLE
					DEBUG("  NDL\r\n");
					#endif
				}
			}
			
		break;
			
		default:
			
		break;
	}
}

void antPx_SendBleMsgOnAnt(uint8_t *buffer,uint8_t lu8_msgLengh,uint8_t *TrgSrNo)
{
	uint8_t lu8_i=0;
		
	//Check if message is intended for any registered node
	for(lu8_i = 0;lu8_i < MAX_NODES_IN_NEXT_LAYER; lu8_i++)
	{
		//
		if(memcmp(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_DeviceSrNo,&buffer[TARGET_ANT_SR_NO_INDEX-1],ANT_SERIAL_NUMBER_BYTES) == 0)
		{
			break;
		}
	}
	
	//Intended node found from registered node list
	if(lu8_i < MAX_NODES_IN_NEXT_LAYER)
	{
		#ifdef ANT_DEBUG_ENABLE
		DEBUG("  NF:");
		gen_PrintHex(gu8_AntBurstMsgProgressState[CH_1]);
		CRLF;
		#endif
		
		if(gu8_AntBurstMsgProgressState[CH_1] == NO_BURST_MESSAGE)
		{
			//Clear Buffer
			memset(&gu8ar_AntBurstTxBuffer[CH_1][0],0x00,sizeof(gu8ar_AntBurstTxBuffer[CH_1]));

			//Shared Address
			gu8ar_AntBurstTxBuffer[CH_1][BRUST_CMD_SHARED_ID_INDEX] = gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_DeviceSharedAddress;

			//Command ID
			gu8ar_AntBurstTxBuffer[CH_1][BURST_CMD_ID_INDEX] = BLE_OP_REQ;		//Command ID
			
			//Fill Target Sr. No.
			memcpy(&gu8ar_AntBurstTxBuffer[CH_1][BURST_BLE_TAR_SR_NO],TrgSrNo,ANT_SERIAL_NUMBER_BYTES);

			//Fill Target Sr. No.
			memcpy(&gu8ar_AntBurstTxBuffer[CH_1][BURST_BLE_SRC_SR_NO],&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],ANT_SERIAL_NUMBER_BYTES);
			
			//Fill Payload
			memcpy(&gu8ar_AntBurstTxBuffer[CH_1][BURST_BLE_PAYLOAD],buffer,lu8_msgLengh);
			
			//Add padding bytes by 0x00 if message is not multiple of 8
			lu8_msgLengh+=10;
			while(lu8_msgLengh%8)
			{
				gu8ar_AntBurstTxBuffer[CH_1][lu8_msgLengh++]=0x00;
			}
			
			gu8_AntTxBurstMsgSize[CH_1] = lu8_msgLengh;
			
			//Burst Initialize
			gu8_AntBurstMsgProgressState[CH_1] = BURST_INITIATED;

			gu8_AntBurstTryCntr[CH_1]=BURST_MSG_RETRY_CH1_COUNTER;
			
			#ifdef ANT_DATA_DEBUG_ENABLE
			gen_PrintHexStr(&gu8ar_AntBurstTxBuffer[CH_1][0],lu8_msgLengh);
			#endif
		}
	}
	else   //Intended node doesn't found,so forwared to Master node
	{
		#ifdef ANT_DEBUG_ENABLE
		DEBUG("  NDF,F2M:");
		gen_PrintHex(gu8_AntBurstMsgProgressState[CH_1]);
		CRLF;
		#endif
		
		if(gu8_AntBurstMsgProgressState[CH_0] == NO_BURST_MESSAGE)
		{
			//Clear Buffer
			memset(&gu8ar_AntBurstTxBuffer[CH_0][0],0x00,sizeof(gu8ar_AntBurstTxBuffer[CH_0]));
		
			//Shared Address
			gu8ar_AntBurstTxBuffer[CH_0][BRUST_CMD_SHARED_ID_INDEX] = gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;

			//Command ID
			gu8ar_AntBurstTxBuffer[CH_0][BURST_CMD_ID_INDEX] = BLE_OP_REQ;		//Command ID
			
			//Fill Target Sr. No.
			memcpy(&gu8ar_AntBurstTxBuffer[CH_0][BURST_BLE_TAR_SR_NO],TrgSrNo,ANT_SERIAL_NUMBER_BYTES);

			//Fill Target Sr. No.
			memcpy(&gu8ar_AntBurstTxBuffer[CH_0][BURST_BLE_SRC_SR_NO],&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],ANT_SERIAL_NUMBER_BYTES);
			
			//Fill Payload
			memcpy(&gu8ar_AntBurstTxBuffer[CH_0][BURST_BLE_PAYLOAD],buffer,lu8_msgLengh);
						
			//Add padding bytes by 0x00 if message is not multiple of 8
			lu8_msgLengh+=10;
			while(lu8_msgLengh%8)
			{
				gu8ar_AntBurstTxBuffer[CH_0][lu8_msgLengh++]=0x00;
			}
			
			gu8_AntTxBurstMsgSize[CH_0] = lu8_msgLengh;
				
			b.mu8_AntSendMobileMsgToMaster_ch0=1;
			
			gu8_AntBurstTryCntr[CH_0]=BURST_MSG_RETRY_CH0_COUNTER;
			
			#ifdef ANT_DATA_DEBUG_ENABLE
			gen_PrintHexStr(&gu8ar_AntBurstTxBuffer[CH_0][0],lu8_msgLengh);
			#endif
		}
	}
}

void antPx_SendResToMasterForRemoteReq(uint8_t lu8_RemoteOpRequest)
{
	uint32_t err_code;
	uint8_t lu8_msglengh=0;
	
	memset(&gu8ar_AntBurstTxBuffer[CH_0][0],0x00,sizeof(gu8ar_AntBurstTxBuffer[CH_0]));
	
	//Shared Address
	gu8ar_AntBurstTxBuffer[CH_0][BRUST_CMD_SHARED_ID_INDEX] = gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
	
	//Command ID
	gu8ar_AntBurstTxBuffer[CH_0][BURST_CMD_ID_INDEX] = REMOTE_OP_RES;		//Command ID
	
	//Make Header
	memcpy(&gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_MSB],&gu8ar_AntBurstRxBuffer[CH_0][BURST_MSG_LEN_MSB],25);
	gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_TYPE] = gu8ar_AntBurstRxBuffer[CH_0][BURST_MSG_TYPE] + 0x80;
	
	switch(lu8_RemoteOpRequest)
	{
		case E_OPERATION_REQUEST:
							
			gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_MSB] = 0;
			gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_LSB] = 25;
			
			gu8ar_AntBurstTxBuffer[CH_0][BURST_REQUEST_STATUS] = gu8_RemoteOpStatus;		//Status of Request, 0x00-Success,>0x00-Fail with Reason Code
			lu8_msglengh=32;
		
		break;
		
		case E_STATUS_REQUEST:
			
			gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_MSB] = 0;
			gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_LSB] = 26;
			
			gu8ar_AntBurstTxBuffer[CH_0][BURST_REQUEST_STATUS] = 0x00;		//Status of Request, 0x00-Success,>0x00-Fail with Reason Code
			gu8ar_AntBurstTxBuffer[CH_0][BURST_DEVICE_STATUS] = gu8_RemoteOpStatus;		//Status of Device, 0x00-Lock, 0x01-UnLock
		
			lu8_msglengh=32;
					
		break;
		
		case E_ADD_USER_ACCESS_DEVICE_REQUEST:

			gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_MSB] = 0;
			gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_LSB] = 25;
			
			gu8ar_AntBurstTxBuffer[CH_0][BURST_REQUEST_STATUS] = gu8_RemoteOpStatus;		//Status of Request, 0x00-Success,>0x00-Fail with Reason Code
			lu8_msglengh=32;
		
		break;
		
		case E_REMOVE_USER_ACCESS_DEVICE_REQUEST:

			gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_MSB] = 0;
			gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_LSB] = 25;
			
			gu8ar_AntBurstTxBuffer[CH_0][BURST_REQUEST_STATUS] = gu8_RemoteOpStatus;		//Status of Request, 0x00-Success,>0x00-Fail with Reason Code
			lu8_msglengh=32;
		
		break;
		
		default :

			#ifdef ANT_DEBUG_ENABLE
			DEBUG("E_DEF_REQ\n\r");
			#endif
		
		break;		
	}
	
	gu8_AntTxBurstMsgSize[CH_0] = lu8_msglengh;
	err_code = sd_ant_burst_handler_request(CHANNEL_0, gu8_AntTxBurstMsgSize[CH_0] , &gu8ar_AntBurstTxBuffer[CH_0][0], BURST_SEGMENT_START | BURST_SEGMENT_END);
	APP_ERROR_CHECK(err_code);	
		
	gu8_AntBurstMsgProgressState[CH_0] = BURST_IN_PROGRESS;
	
	ant_request = ANT_MSG_ACK_CMPLTD;
	//gu8_AntBurstTryCntr[CH_0]=BURST_MSG_RETRY_CH0_COUNTER;
}

void antPx_FwdMsgToMaster(void)
{
	uint32_t err_code;
	
	memset(&gu8ar_AntBurstTxBuffer[CH_0][0],0x00,sizeof(gu8ar_AntBurstTxBuffer[CH_0]));
	
	//Copy receive Burst Message from CH1 to gu8ar_AntBurstTxBuffer to send on CH0
	memcpy(&gu8ar_AntBurstTxBuffer[CH_0][0],gu8ar_AntTempBuffer1,gu8_AntTxBurstMsgSize[CH_0]);
	
	//Shared Address
	gu8ar_AntBurstTxBuffer[CH_0][BRUST_CMD_SHARED_ID_INDEX] = gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
	
	err_code = sd_ant_burst_handler_request(CHANNEL_0, gu8_AntTxBurstMsgSize[CH_0] , &gu8ar_AntBurstTxBuffer[CH_0][0], BURST_SEGMENT_START | BURST_SEGMENT_END);
	APP_ERROR_CHECK(err_code);	
		
	gu8_AntBurstMsgProgressState[CH_0] = BURST_IN_PROGRESS;
	
	gu8_AntBurstTryCntr[CH_0]=BURST_MSG_RETRY_CH0_COUNTER;
}

void antPx_SendAsyncMsgToMaster(void)
{
	uint32_t err_code;
	
	memset(&gu8ar_AntBurstTxBuffer[CH_0][0],0x00,sizeof(gu8ar_AntBurstTxBuffer[CH_0]));

	//Shared Address
	gu8ar_AntBurstTxBuffer[CH_0][BRUST_CMD_SHARED_ID_INDEX] = gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
	
	//Command ID
	gu8ar_AntBurstTxBuffer[CH_0][BURST_CMD_ID_INDEX] = ASYNC_EVENT_REQ;		//Command ID
	
	//Make Header
	gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_MSB] = 0;		
	
	gu8ar_AntBurstTxBuffer[CH_0][BURST_DEVICE_TECH] = 0x02;	
	gu8ar_AntBurstTxBuffer[CH_0][BURST_DEVICE_TYPE] = 0x0E;	
	memcpy(&gu8ar_AntBurstTxBuffer[CH_0][BURST_TAR_NODE_FULL_SR_NO],&gst_ConfigPermanentParams.mu8ar_SerialNumber[0],SERIAL_NUM_BYTES);
	memcpy(&gu8ar_AntBurstTxBuffer[CH_0][BURST_TAR_NODE_ANT_SR_NO],&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],ANT_SERIAL_NUMBER_BYTES);
	gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_TYPE] = E_AYNCHRONOUS_EVENT_REPORT_RESP;
	
	//Make Payload
	gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_TYPE] = gu8_AntAsyncStatus;//gu8_AntLockLastStatus;
	
	#ifdef ASYNC_MSG_UNENCRYPTED
	//------------ Unencrypted Message
	gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_LSB] = 32;	
	gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_ENCRY_TYPE] = 0x00;		//0x00=Unencrypted, 0x01=Encrypted
	
	gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_DATA] = 6;
	gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_DATA+1] = gu8_AntAsyncStatus;
	memcpy(&gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_DATA+2],&gu8_EpochTime,4);
	
	gu8_AntTxBurstMsgSize[CH_0]=40;
	
	#else
	//------------ Encrypted Message	
	gu8ar_AntBurstTxBuffer[CH_0][BURST_MSG_LEN_LSB] = 42;	
	gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_ENCRY_TYPE] = 0x01;		//0x00=Unencrypted, 0x01=Encrypted
	
	gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_DATA] = 16;
	gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_DATA+1] = gu8_AntAsyncStatus;
	memcpy(&gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_DATA+2],&gu8_EpochTime,4);
	memset(&gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_DATA+6],0x00,10);

	blePx_EncryptData(gst_ConfigPermanentParams.mu8ar_AesKey3,&gu8ar_AntBurstTxBuffer[CH_0][BURST_ASYNC_EVENT_DATA],16,0);

	gu8_AntTxBurstMsgSize[CH_0]=48;
	#endif
	
	err_code = sd_ant_burst_handler_request(CHANNEL_0, gu8_AntTxBurstMsgSize[CH_0] , &gu8ar_AntBurstTxBuffer[CH_0][0], BURST_SEGMENT_START | BURST_SEGMENT_END);
	APP_ERROR_CHECK(err_code);	
	
	gu8_AntBurstMsgProgressState[CH_0] = BURST_IN_PROGRESS;
	
	gu8_AntBurstTryCntr[CH_0]=BURST_MSG_RETRY_CH0_COUNTER;
}

void antPx_ProcessBurstMessageAtCH0(uint8_t * p_event_message_buffer)
{
	uint8_t lu8_i=0,lu8_j=0,lu8_k=0,lu8_matchCnt=0;
	uint8_t lu8_DevSharedAdd=0;
	
	//Fill the seven bytes from received payload
	memcpy(&gu8ar_AntBurstRxBuffer[CH_0][gu8_BurstRcvInd[CH_0]],&p_event_message_buffer[3],8);
	gu8_BurstRcvInd[CH_0] += 8;
		
	if(p_event_message_buffer[2] & 0x80)
	{
		#ifdef ANT_DATA_DEBUG_ENABLE
		gen_PrintHexStr(&gu8ar_AntBurstRxBuffer[CH_0][0],gu8_BurstRcvInd[CH_0]);	
		CRLF;
		#endif
		
		#ifdef ANT_BURST_TEST_ENABLE
		DEBUG("BURST RCVD CH0:");
		gen_PrintHex(gu8ar_AntBurstRxBuffer[CH_0][BURST_CMD_ID_INDEX]);	
		CRLF;
		#endif 
		
		switch(gu8ar_AntBurstRxBuffer[CH_0][BURST_CMD_ID_INDEX])
		{
			case REMOTE_OP_REQ:
				
				//Check if message is intended for self or other
				if(!memcmp(&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],&gu8ar_AntBurstRxBuffer[CH_0][BURST_TAR_NODE_ANT_SR_NO],SR_NO_LENGTH))
				{
					#ifdef ANT_DEBUG_ENABLE
					DEBUG("  SELF\r\n");
					#endif

					//Decrypt Operation Key
					antPx_DecryptData(gst_ConfigPermanentParams.mu8ar_AesKey3,&gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_IV],&gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_DATA],16);
									
					#ifdef ANT_BURST_TEST_ENABLE
					DEBUG("DECRYPT DATA=");
					gen_PrintHexStr(&gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_DATA],16);	
					CRLF;
					#endif 
					
					gu8_RemoteOpStatus=0;
					
					if(gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_DATA]==16) lu8_matchCnt|=0x01;
					if(!memcmp(&gst_ConfigPermanentParams.mu8ar_SerialNumber[5],&gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_DATA+1],5)) 
					{
						lu8_matchCnt|=0x02;
					}
					else
					{
						gu8_RemoteOpStatus = 12;
					}
					
					if(!memcmp(&gst_ConfigPermanentParams.mu8ar_SecurityTocken[2],&gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_DATA+6],4)) lu8_matchCnt|=0x04;
					
					if(gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_DATA+10]==gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_TYPE]) 
					{
						lu8_matchCnt|=0x08;
					}
					else
					{
						gu8_RemoteOpStatus = 13;
					}
					
					if(!memcmp(&gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_TIME],&gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_DATA+11],4)) 
					{
						lu8_matchCnt|=0x10;
					}
					else
					{
						gu8_RemoteOpStatus = 11;
					}
						
					if(gu8ar_AntBurstRxBuffer[CH_0][BURST_OP_EXPIRE_TIME]==gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_KEY_DATA+15]) lu8_matchCnt|=0x20;
					
					b.mu8_AntSendResToMasterForRemoteReq=1;
					gu8_AntBurstTryCntr[CH_0]=BURST_MSG_RETRY_CH0_COUNTER;
					
					gu8_RemoteOpRequest=gu8ar_AntBurstRxBuffer[CH_0][BURST_MSG_TYPE];
					
					#ifdef ANT_BURST_TEST_ENABLE
					DEBUG("MISMATCH=");
					gen_PrintHex(lu8_matchCnt);	
					CRLF;
					#endif 
					
					if(lu8_matchCnt==0x3F)
					{
						//Find Command From Remote
						switch(gu8_RemoteOpRequest)
						{
							case E_OPERATION_REQUEST:
								
								#ifdef ANT_OPERATION_COUNT_ENABLE
								//For Testing Debug, remove in final -------
								DEBUG("RCV:");
								gen_PrintHex(gu8_AntBurstRcvCount>>8);
								gen_PrintHex(gu8_AntBurstRcvCount);
								DEBUG(" OK:");
								gen_PrintHex(gu8_AntBurstOKCount>>8);
								gen_PrintHex(gu8_AntBurstOKCount);
								DEBUG(" RTRY:");
								gen_PrintHex(gu8_AntBurstRetryCount>>8);
								gen_PrintHex(gu8_AntBurstRetryCount);
								DEBUG(" FAIL:");
								gen_PrintHex(gu8_AntBurstFailCount>>8);
								gen_PrintHex(gu8_AntBurstFailCount);
								DEBUG("\r\n");
															
								gu8_AntBurstRcvCount++;
								#endif
								//-------------------------------------------
							
								if(gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_TYPE] != E_RELAY_STATUS)
								{
									gu8_RemoteOpStatus = 0;
								}
								ant_request = ANT_MSG_RCVD;
								switch(gu8ar_AntBurstRxBuffer[CH_0][BURST_OPERATION_TYPE])
								{
									
									case E_DOOR_UNLOCK:
										
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("UNLOCK or OPEN Req\n\r");
										#endif
										gu8_AntAsyncStatus = 0;
									
									break;

									case E_DOOR_LOCK:
										
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("LOCK or CLOSE Req\n\r");
										#endif
										gu8_AntAsyncStatus = 1;
									
									break;
								
									case E_OPERATE_RELAY_MOMENTARY:
										
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("\n\rE_OPERATE_RELAY_MOMENTARY\n\r");
										#endif
										nrf_gpio_pin_set(RELAY_1_CTR_PIN_NUMBER);
										green_led_on();
										sl_app_timer[RELAY_OFF_TIMER]=RELAY_OFF_TIME;
									
									break;
									
									case E_OPERATE_RELAY_MAINTAIN:
										
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("\n\rE_OPERATE_RELAY_MAINTAIN\n\r");
										#endif
										if(!relay_status_flg)
										{
											ant_relay_status = 1;
											nrf_gpio_pin_set(RELAY_2_CTR_PIN_NUMBER);
											green_led_on();
											relay_status_flg=true;
										}
										else
										{
											ant_relay_status = 2;
											nrf_gpio_pin_clear(RELAY_2_CTR_PIN_NUMBER);
											green_led_off();
											relay_status_flg=false;
										}
										
									break;	
									
									case E_OPERATE_SOLANOID:
										
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("\n\rE_OPERATE_SOLANOID\n\r");
										#endif
										nrf_gpio_pin_set(SOLENOID_CTR_PIN_NUMBER);
										green_led_on();
										sl_app_timer[SOLENOID_OFF_TIMER]=SOLENOID_OFF_TIME;
										solenoid_status = true;
										
									break;
									
									case E_MOTOR_FORWARD:
										
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("\n\rE_MOTOR_FORWARD\n\r");
										#endif
									
										motor_forward();
										green_led_on();
										sl_app_timer[MOTOR_OFF_TIMER]=MOTOR_OFF_TIME;
										
									break;
									
									case E_MOTOR_REVERSE:

										#ifdef ANT_DEBUG_ENABLE
										DEBUG("\n\rE_MOTOR_REVERSE\n\r");
										#endif
										motor_reverse();
										green_led_on();
										sl_app_timer[MOTOR_OFF_TIMER]=MOTOR_OFF_TIME;								
										
									break;
									
									case E_RELAY_STATUS:
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("\n\rE_RELAY_STATUS\n\r");
										#endif										
										gu8_RemoteOpStatus = ant_relay_status;
									break;
									
//									case E_LIGHT_ON:
//									
//										#ifdef ANT_DEBUG_ENABLE
//										DEBUG("E_LIGHT_ON\n\r");
//										#endif
//									
//										light_status_flg=true;
//										if(light_intensity_value != 0)
//										{
//											DEBUG("light_intensity_value\n\r");
//											set_intensity(light_intensity_value);
//										}				
//										
//									break;
									
//									case E_LIGHT_OFF:

//										#ifdef ANT_DEBUG_ENABLE
//										DEBUG("E_LIGHT_OFF\n\r");
//										#endif
//									
//										light_status_flg=false;
//										set_intensity(0);	
//									
//									break;
//									
//									case E_SWITCH_STATUS:
//										
//										if(nrf_gpio_pin_read(STATUS_SW1) == LOW)
//										{
//											gu8_AntLockLastStatus = LOW;
//										}
//										else
//										{
//											gu8_AntLockLastStatus = HIGH;
//										}								
//									break;
																
									default :
										
										ant_request = NO_ANT_MSG;
										gu8_RemoteOpStatus = E_OP_STATUS_INVALID_OP_REQ;
									
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("DEF_OPREQ\n\r");
										#endif
									
									break;
								}							
							break;
							
							case E_STATUS_REQUEST:
								#ifdef ANT_DEBUG_ENABLE
								DEBUG("E_STAT_REQ\n\r");
								#endif		
								//gu8_RemoteOpStatus = sl_current_status();
							break;
							
							case E_ADD_USER_ACCESS_DEVICE_REQUEST:
								#ifdef ANT_DEBUG_ENABLE
								DEBUG("E_AUADR\n\r");
								#endif
							break;
							
							case E_REMOVE_USER_ACCESS_DEVICE_REQUEST:
								#ifdef ANT_DEBUG_ENABLE
								DEBUG("E_RUADR\n\r");
								#endif
							break;
							
							default :
								b.mu8_AntSendResToMasterForRemoteReq=0;
								gu8_AntBurstTryCntr[CH_0]=0;
							
								#ifdef ANT_DEBUG_ENABLE
								DEBUG("E_DEF_REQ\n\r");
								#endif
							break;
						}
					}
					
					break;
				}
				
				if(SUCCESS==antP1_SearchDevice(&lu8_DevSharedAdd, &gu8ar_AntBurstRxBuffer[CH_0][BURST_TAR_NODE_ANT_SR_NO]))
				{
					if(gu8_AntBurstMsgProgressState[CH_1] == NO_BURST_MESSAGE)
					{						
						memcpy(&gu8ar_AntBurstTxBuffer[CH_1][0],&gu8ar_AntBurstRxBuffer[CH_0][0],gu8_BurstRcvInd[CH_0]);
						
						gu8ar_AntBurstTxBuffer[CH_1][BRUST_CMD_SHARED_ID_INDEX]=lu8_DevSharedAdd;
						
						gu8_AntTxBurstMsgSize[CH_1] = gu8_BurstRcvInd[CH_0];
						
						gu8_AntBurstMsgProgressState[CH_1] = BURST_INITIATED;
						
						gu8_AntBurstTryCntr[CH_1]=BURST_MSG_RETRY_CH1_COUNTER;
					}
				}
				else
				{
					#ifdef ANT_BURST_TEST_ENABLE
					DEBUG("  NDL\r\n");
					#endif
				}
			
			break;
			
			case BLE_OP_REQ: 

				//Check if message is intended for self
				if(!memcmp(&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],&gu8ar_AntBurstRxBuffer[CH_0][BURST_BLE_TAR_SR_NO],SR_NO_LENGTH))
				{
					#ifdef ANT_DEBUG_ENABLE
					DEBUG("  SELF\r\n");
					#endif
					
					gu8_BleMsgReceivedOnAnt=TRUE;
					gu8p_AntBleMsgptr = &gu8ar_AntBurstRxBuffer[CH_0][BURST_BLE_PAYLOAD];
					gu8p_AntBleMsglen = gu8_BurstRcvInd[CH_0]-BURST_BLE_PAYLOAD;
					memcpy(&gu8_TargetAntSerialNumber[0],&gu8ar_AntBurstRxBuffer[CH_0][BURST_BLE_SRC_SR_NO],ANT_SERIAL_NUMBER_BYTES);
				}
				else
				{
					//Check if message is intended any registered node
					for(lu8_i = 0;lu8_i < MAX_NODES_IN_NEXT_LAYER; lu8_i++)
					{
						//this loop will delete all the Child node entries from RAM copy, whose parent removal request received.
						if(memcmp(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_DeviceSrNo,&gu8ar_AntBurstRxBuffer[CH_0][BURST_BLE_TAR_SR_NO],SR_NO_LENGTH) == 0)
						{
							break;
						}
					}
					
					//Intended node found from registered node list
					if(lu8_i < MAX_NODES_IN_NEXT_LAYER)
					{
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("  NFL\r\n");
						#endif
						
						if(gu8_AntBurstMsgProgressState[CH_1] == NO_BURST_MESSAGE)
						{
							memcpy(&gu8ar_AntBurstTxBuffer[CH_1][0],&gu8ar_AntBurstRxBuffer[CH_0][0],gu8_BurstRcvInd[CH_0]);
							gu8ar_AntBurstTxBuffer[CH_1][BRUST_CMD_SHARED_ID_INDEX]=gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_DeviceSharedAddress;
														
							gu8_AntTxBurstMsgSize[CH_1] = gu8_BurstRcvInd[CH_0];
							
							gu8_AntBurstMsgProgressState[CH_1] = BURST_INITIATED;
							
							gu8_AntBurstTryCntr[CH_1]=BURST_MSG_RETRY_CH1_COUNTER;
						}
					}
					else   //Intended node doesn't found,so forwared to Master node
					{
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("  NDFL\r\n");
						#endif
					}
				}
				
			break;
			
			case ADD_NODE_RES:
				
				for(lu8_i = 0;lu8_i < gu8ar_AntBurstRxBuffer[CH_0][BURST_NUM_OF_DEV_INDEX]; lu8_i++)
				{
					lu8_j=BRUST_INFO_START_INDEX + (lu8_i * BRUST_SINGE_DEV_INFO_LEN);
					if(gu8ar_AntBurstRxBuffer[CH_0][lu8_j+11] == E_DEV_NOT_IN_DB)
					{
						//Search the Node and Delete the Entry. 
						for(lu8_k = 0;lu8_k < MAX_NODES_IN_NEXT_LAYER; lu8_k++)
						{
							if(!memcmp(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_k].mu8ar_DeviceSrNo,&gu8ar_AntBurstRxBuffer[CH_0][lu8_j],SR_NO_LENGTH))
							{
								break;
							}					
						}
						
						//Delete Status polling time out Node
						antI3_DeleteNode(lu8_k);
					}
				}
					
				//Find Next Shared Address from status Update Node
				antI4_GetNextSharedAddress();
				
				//Update SlaveTrack Index.
				if(FAIL == antI4_GetNodeParamsWrIndx())
				{
					
				}
			
			break;
				
			case ASYNC_EVENT_RES:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("\n\rASYNC EVENT RESP\n\r");
				#endif
				
			break;
			
			default:
			
				#ifdef ANT_BURST_TEST_ENABLE
				DEBUG("DEFAULT CASE\r\n");
				#endif 
			
			break;
		}

		gu8_BurstRcvInd[CH_0]=0;
	}
}

void antPx_TaskOnRxAtCH0(uint8_t * p_event_message_buffer)        
{
   	switch (p_event_message_buffer[ANT_MSG_IDX_ID])
    {
        /* Broadcast data recieved */
        case MESG_BROADCAST_DATA_ID:  
		
			antPx_ProcessBroadCastMessageAtCH0(p_event_message_buffer);

            break;
				
		/* Ack data recieved */
        case MESG_ACKNOWLEDGED_DATA_ID:  
	
			antPx_ProcessAckMessageAtCH0(p_event_message_buffer);
						
            break;
				
		/* Burst data recieved */
        case MESG_BURST_DATA_ID:  

			antPx_ProcessBurstMessageAtCH0(p_event_message_buffer);

            break;
				
		/* Advanced Burst data recieved */
        case MESG_ADV_BURST_DATA_ID:  
						
            break;
				
        default:      
            break;
    }
}

/** 
 * @brief Handle ANT RX channel events 
 *
 * @param[in] p_event_message_buffer is the ANT event message buffer
 *
 */
void antPx_TaskOnRxAtCH1(uint8_t * p_event_message_buffer)        
{
   	switch (p_event_message_buffer[ANT_MSG_IDX_ID])
    {
        /* Broadcast data recieved */
        case MESG_BROADCAST_DATA_ID:  
		
			antPx_ProcessBroadCastMessageAtCH1(p_event_message_buffer);

            break;
				
		/* Ack data recieved */
        case MESG_ACKNOWLEDGED_DATA_ID:  
		
			antPx_ProcessAckMessageAtCH1(p_event_message_buffer);
						
            break;
				
		/* Burst data recieved */
        case MESG_BURST_DATA_ID:  
		
			antPx_ProcessBurstMessageAtCH1(p_event_message_buffer);

            break;
				
		/* Advanced Burst data recieved */
        case MESG_ADV_BURST_DATA_ID:  
			
            break;
				
        default:      
            break;
    }
}

void antPx_ProcessBroadCastMessageAtCH1(uint8_t * p_event_message_buffer)
{
	//uint32_t err_code;
		
	switch(p_event_message_buffer[APP_MSG_IDX_ID])
	{		
		default:
		
		break;
	}
}

void antPx_ProcessAckMessageAtCH1(uint8_t * p_event_message_buffer)
{
	st_AntNodeFlashParams_t	lst_AntNode;
	uint8_t lu8_NodeIndex = 0;
	
	switch(p_event_message_buffer[APP_MSG_IDX_ID])
	{
		case REQUEST_ADD: 
			
			if(!gu8_AddrAssignTimeout[CH_1])
			{		
				//Copy the device serial number for search operation.
				memcpy(&lst_AntNode.mu8ar_DeviceSrNo[0],&p_event_message_buffer[SR_NO_IDX_ID],SR_NO_LENGTH);

				//Check if Node is already registered and we get registration request for same.   
				if(NODE_ALREADY_REGISTERED == antI3_SearchDuplicateNode(&lst_AntNode, &lu8_NodeIndex))
				{		
					#ifdef ANT_NODE_ADD_REMOVE_ENABLE
					DEBUG("DUPLICATE NODE\r\n");
					#endif
					
					//Delete current Entry of Node,
					antI3_DeleteNode(lu8_NodeIndex);

					antI4_GetNextSharedAddress();
					//gu8_NextSharedAddress = lst_AntNode.mu8_DeviceSharedAddress;
					//gu8_SlaveTrackIndex = antI3_FlashGetWriteIndex(&lst_AntNode);
					
					//gu8_AntRegState = ADD_AVAILABEL;
					
					#ifdef ANT_DEBUG_ENABLE
					DEBUG("NAR\r\n");
					#endif

					return;
				}
				//Copy for next stage verification.(this Sr number should match throughout Registration process.)
				gst_AntTempPara.mu8_DeviceSharedAddress=gu8_NextSharedAddress;
				memcpy(&gst_AntTempPara.mu8ar_DeviceSrNo[0],&p_event_message_buffer[SR_NO_IDX_ID],SR_NO_LENGTH);
				gst_AntTempPara.mu8_ParentSharedAddress=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
				memcpy(&gst_AntTempPara.mu8ar_ParentSrNo[0],&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],SR_NO_LENGTH);
				gst_AntTempPara.mu8_ParentLayerNumber=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber+1;
				
				//Change the Slave registration as GW gets Assign address request,
				gu8_AntRegState=BUSY_ACQUIRING;			
						
				gu8_AddrAssignTimeout[CH_1]=10;		//Set 5 second Timeout
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("ST2:REQ ADD\r\n");
				#endif
			}
			
		break;
		
		case CONFIRM_ACQUIRE: 
			
			if(!memcmp(&p_event_message_buffer[SR_NO_IDX_ID],&gst_AntTempPara.mu8ar_DeviceSrNo[0],SR_NO_LENGTH))
			{
				//stop BLE Advertisement
				//advertising_stop();
				gu8_AddrAssignTimeout[CH_1]=0;
				
				gst_AntTempPara.mu8_RegisterStatus=E_DEV_NOT_IN_DB;
				memcpy((uint8_t *)&gst_AntFlashParams.mst_AntNodeFlashParams[gu8_AddedSlaveNodeIndex],(uint8_t *)&gst_AntTempPara,sizeof(gst_AntTempPara));
				
				#ifdef ANT_NODE_ADD_REMOVE_ENABLE
				DEBUG("\r\nNODE ADDED:");
				gen_PrintHexStr(&gst_AntTempPara.mu8ar_DeviceSrNo[0],4);
				DEBUG(" @ Index=");
				gen_PrintHex(gu8_AddedSlaveNodeIndex);
				CRLF;
				#endif
				
				//Update NextLayer Shared address bitmapping  
				gu16_AntNextLayerNodesPollStatus |= (1<<gu8_NextSharedAddress);
				antI4_GetNextSharedAddress();
				
				//Update SlaveTrack Index.
				if(FAIL == antI4_GetNodeParamsWrIndx())
				{
					
				}
				
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
				gu8_AntFlashWriteReason=DIRECT_ANT_NODE_INFO_ADD;

				#ifdef ANT_FLASH_DEBUG_ENABLE
				DEBUG("\r\nFLASH WRITE REASON:");
				gen_PrintHex(gu8_AntFlashWriteReason);
				CRLF;
				DEBUG("ST4: CNFRM_DEV\r\n");
				DEBUG("Store N Info\r\n");
				#endif
			}
			
		break;		
		
		case EPOCH_TIME_REQ:

			#ifdef ANT_EPOCH_LOG_ENABLE
			DEBUG("EPOCH REQ RCVD @ CH1:");
			gen_PrintHexStr(&p_event_message_buffer[3],8);
			CRLF;
			#endif
		
			gu8_AntReqSharedAddrAtCh1=p_event_message_buffer[3];
			gu8_AntReqTypeAtCh1=EPOCH_TIME_REQ;
		
		break;
			
		/*case REMOVE_L2_NODE_REQ:
			
			//Copy the device serial number for search operation.
			memcpy(&lst_AntNode.mu8ar_DeviceSrNo[0],&p_event_message_buffer[SR_NO_IDX_ID],SR_NO_LENGTH);
			
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("DEL Node SrNo:");
			gen_PrintHexStr(lst_AntNode.mu8ar_DeviceSrNo,11);
			CRLF;
			#endif
		
			//Check if Node is already registered and we get registration request for same.   
			if(NODE_ALREADY_REGISTERED == antI3_SearchDuplicateNode(&lst_AntNode, &lu8_NodeIndex))
			{
				//Delete current Entry of Node,
				antI3_DeleteNode(lu8_NodeIndex);
				
				//Find next Write Index in database
				if(FAIL == antI4_GetNodeParamsWrIndx())
				{	

				}
			}
			
			//Send Delete Node message to Upper layer also
			memcpy(&gu8ar_AntCHTxbuffer[CH_0][0],&p_event_message_buffer[3],8);
			gu8_AntDeletNextLayerNode[FAR_DEVICE]=MESSAGE_INITIATED;
			
		break;
		*/
		
		default:
		
		break;
	}
}

void antPx_ProcessBurstMessageAtCH1(uint8_t * p_event_message_buffer)
{
	uint8_t lu8_i=0;
	uint8_t lu8_InfoIndex=0,lu8_Offset=0;
	st_AntNodeFlashParams_t lstp_NodeInfo;		
	
	//Fill Burst RX buffer from received payload
	memcpy(&gu8ar_AntBurstRxBuffer[CH_1][gu8_BurstRcvInd[CH_1]],&p_event_message_buffer[3],8);
	gu8_BurstRcvInd[CH_1] += 8;
		
	if(p_event_message_buffer[2] & 0x80)
	{
		#ifdef ANT_DATA_DEBUG_ENABLE
		gen_PrintHexStr(&gu8ar_AntBurstRxBuffer[CH_1][0],gu8_BurstRcvInd[CH_1]);	
		#endif
		
		#ifdef ANT_BURST_TEST_ENABLE
		DEBUG("BURST RCVD CH1:");
		gen_PrintHex(gu8ar_AntBurstRxBuffer[CH_1][BURST_CMD_ID_INDEX]);	
		CRLF;
		#endif
		
		switch(gu8ar_AntBurstRxBuffer[CH_1][BURST_CMD_ID_INDEX])
		{
			case ADD_NODE_REQ:
							
				for(lu8_i = 0;lu8_i < gu8ar_AntBurstRxBuffer[CH_1][BURST_NUM_OF_DEV_INDEX];lu8_i++)
				{
					memcpy(&lstp_NodeInfo,&gu8ar_AntBurstRxBuffer[CH_1][BRUST_INFO_START_INDEX + lu8_Offset],sizeof(st_AntNodeFlashParams_t));
					
					//Check for duplicate node info in data base
					if(NODE_ALREADY_REGISTERED == antI3_SearchDuplicateNode(&lstp_NodeInfo,&lu8_InfoIndex))			
					{
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("DUPLICATE L2 NODE\r\n");
						#endif
						
						//Delete Duplicate Entry for over writing new entry
						antI3_DeleteNode(lu8_InfoIndex);
						
						//for overwrting previous value the writing index should be updated
						if(FAIL == antI4_GetNodeParamsWrIndx())
						{	

						}
					}
					//Register Device.
					//Add The Far Device(who are not connected to Master Bridge Directely), Copy into Ram copy data base.
					memcpy((uint8_t *)&gst_AntFlashParams.mst_AntNodeFlashParams[gu8_AddedSlaveNodeIndex],(uint8_t *)&lstp_NodeInfo,sizeof(st_AntNodeFlashParams_t));

					//New Node Added
					gu8_AdditionOfIndirectNodes++;
					
					//Write back everything to Flash
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
					gu8_AntFlashWriteReason=INDIRECT_ANT_NODE_INFO_ADD;
					
					#ifdef ANT_FLASH_DEBUG_ENABLE
					DEBUG("\r\nFLASH WRITE REASON:");
					gen_PrintHex(gu8_AntFlashWriteReason);
					CRLF;
					#endif
					
					#ifdef ANT_DEBUG_ENABLE
					DEBUG("L2 NODE ADD:TrckIdx =");
					gen_PrintHex(gu8_AddedSlaveNodeIndex);CRLF;
					#endif

					//Get next free index for Next new node 
					if(FAIL == antI4_GetNodeParamsWrIndx())
					{	
						
					}
		
					//Get New Device Info Offset for Buffer
					lu8_Offset += gu8ar_AntBurstRxBuffer[CH_1][BRUST_SINGE_DEV_INFO_LEN];
				}
				
//				for(lu8_i=0;lu8_i<gu8ar_AntBurstRxBuffer[CH_1][3];lu8_i++)
//				{
//					if(gst_AntFlashParams.mst_AntFlashHeader.mu8_AddedSlaveNodeIndex < MAX_NODES_IN_NEXT_LAYER)
//					{
//						memcpy((uint8_t *)&gst_AntFlashParams.mst_AntNodeFlashParams[gst_AntFlashParams.mst_AntFlashHeader.mu8_AddedSlaveNodeIndex],&gu8ar_AntBurstRxBuffer[CH_1][4],gu8ar_AntBurstRxBuffer[CH_1][3]);
//						gst_AntFlashParams.mst_AntFlashHeader.mu8_AddedSlaveNodeIndex++;
//						lu8_NodeAdd=1;
//						
//						//New Node Added
//						gu8_AdditionOfIndirectNodes++;
//					}
//				}
//				
//				if(lu8_NodeAdd==1)
//				{
//					if(gu8_AntFlashWriteState==NO_ANT_FLASH_WRITE_OPERATION)
//					{
//						gu8_AntFlashWriteState=ANT_FLASH_WRITE_INITIAT;
//						#ifdef ANT_FLASH_DEBUG_ENABLE
//						DEBUG("NO_ANT_FLASH_WRITE_OPERATION\r\n");
//						#endif
//					}
//					else
//					{
//						#ifdef ANT_FLASH_DEBUG_ENABLE
//						DEBUG("ANT_FLASH_WRITE_IN_PROGRESS\r\n");
//						#endif
//					}
//					gu8_AntFlashWriteReason=INDIRECT_ANT_NODE_INFO_ADD;
//				}
			
			break;
			
			case REMOTE_OP_RES: 
			
				//Send Ack for Remote Request to Master
				b.mu8_AntFwdMsgToMaster=1;
				
				gu8_AntTxBurstMsgSize[CH_0]=gu8_BurstRcvInd[CH_1];
			
				memcpy(&gu8ar_AntTempBuffer1[0],&gu8ar_AntBurstRxBuffer[CH_1][0],gu8_BurstRcvInd[CH_1]);
			
			break;	
			
			case ASYNC_EVENT_REQ: 
			
				//Forward Asynch Event message to Master
				b.mu8_AntSendAsyncMsgToMaster=1;
			
				gu8_AntTxBurstMsgSize[CH_0]=gu8_BurstRcvInd[CH_1];
			
				memcpy(&gu8ar_AntTempBuffer1[0],&gu8ar_AntBurstRxBuffer[CH_1][0],gu8_BurstRcvInd[CH_1]);
			
			break;	
			
			case BLE_OP_REQ: 
				
				//Check if message is intended for self
				if(!memcmp(&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],&gu8ar_AntBurstRxBuffer[CH_1][BURST_BLE_TAR_SR_NO],SR_NO_LENGTH))
				{
					#ifdef ANT_DEBUG_ENABLE
					DEBUG("  SELF\r\n");
					#endif
					
					gu8_BleMsgReceivedOnAnt=TRUE;
					gu8p_AntBleMsgptr = &gu8ar_AntBurstRxBuffer[CH_1][BURST_BLE_PAYLOAD];
					gu8p_AntBleMsglen = gu8_BurstRcvInd[CH_1]-BURST_BLE_PAYLOAD;
					
					memcpy(&gu8_TargetAntSerialNumber[0],&gu8ar_AntBurstRxBuffer[CH_1][BURST_BLE_SRC_SR_NO],ANT_SERIAL_NUMBER_BYTES);
				}
				else
				{
					//Check if message is intended any registered node
					for(lu8_i = 0;lu8_i < MAX_NODES_IN_NEXT_LAYER; lu8_i++)
					{
						//this loop will delete all the Child node entries from RAM copy, whose parent removal request received.
						if(memcmp(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8ar_DeviceSrNo,&gu8ar_AntBurstRxBuffer[CH_1][BURST_BLE_TAR_SR_NO],SR_NO_LENGTH) == 0)
						{
							break;
						}
					}
					
					//Intended node found from registered node list
					if(lu8_i < MAX_NODES_IN_NEXT_LAYER)
					{
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("  NFL\r\n");
						#endif
						
						if(gu8_AntBurstMsgProgressState[CH_1] == NO_BURST_MESSAGE)
						{
							memcpy(&gu8ar_AntBurstTxBuffer[CH_1][0],&gu8ar_AntBurstRxBuffer[CH_1][0],gu8_BurstRcvInd[CH_1]);
							gu8ar_AntBurstTxBuffer[CH_1][BRUST_CMD_SHARED_ID_INDEX]=gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_DeviceSharedAddress;
							
							gu8_AntTxBurstMsgSize[CH_1] = gu8_BurstRcvInd[CH_1];
							
							gu8_AntBurstMsgProgressState[CH_1] = BURST_INITIATED;
							
							gu8_AntBurstTryCntr[CH_1]=BURST_MSG_RETRY_CH1_COUNTER;
						}
					}
					else   //Intended node doesn't found,so forwared to Master node
					{
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("  NDF,F2M\r\n");
						#endif
						
						if(gu8_AntBurstMsgProgressState[CH_0] == NO_BURST_MESSAGE)
						{
							b.mu8_AntSendMobileMsgToMaster_ch1=1;

							memcpy(&gu8ar_AntBurstTxBuffer[CH_0][0],&gu8ar_AntBurstRxBuffer[CH_1][0],gu8_BurstRcvInd[CH_1]);
							
							gu8ar_AntBurstTxBuffer[CH_0][BRUST_CMD_SHARED_ID_INDEX]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
							
							gu8_AntTxBurstMsgSize[CH_0] = gu8_BurstRcvInd[CH_1];
						}
					}
				}
				
			break;
				
			default:
			
			break;
		}

		
		gu8_BurstRcvInd[CH_1]=0;
	}
}

//For master Device....
void on_ant_evt(ant_evt_t * p_ant_evt)
{	
	uint32_t err_code; 
	
	memcpy(&event_message_buffer[0],p_ant_evt->evt_buffer,ANT_STACK_EVT_MSG_BUF_SIZE);

    //if (p_ant_evt->channel == 0)
    {//if channel is 0th channel
        switch (p_ant_evt->event)
        { 
            case EVENT_TX:
				
				//antPx_HandleEventTx(event_message_buffer);
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:

					break;
					
					case CHANNEL_1:
					
						if(gu8_AntBurstMsgProgressState[CH_1] == BURST_INITIATED)
						{
							#ifdef ANT_BURST_TEST_ENABLE
							DEBUG("CH1 BURST_INITIATED\r\n");
							#endif
							
							err_code = sd_ant_burst_handler_request(CHANNEL_1,gu8_AntTxBurstMsgSize[CH_1],&gu8ar_AntBurstTxBuffer[CH_1][0],BURST_SEGMENT_START | BURST_SEGMENT_END);
							APP_ERROR_CHECK(err_code);	
														
							gu8_AntBurstMsgProgressState[CH_1] = BURST_IN_PROGRESS;
						}
						else if(gu8_AntBurstMsgProgressState[CH_1] == BURST_IN_PROGRESS)
						{
							#ifdef ANT_BURST_TEST_ENABLE
							DEBUG("CH1 BURST_IN_PROGRESS\r\n");
							#endif
						}
						else if(gu8_AntReqTypeAtCh1)
						{
							switch(gu8_AntReqTypeAtCh1)
							{
								case EPOCH_TIME_REQ:
									
									#ifdef ANT_EPOCH_LOG_ENABLE
									DEBUG("EPOCH RES SEND @ CH1:");
									#endif
									
									memset(&gu8ar_AntCHTxbuffer[CH_1][0],0,PAYLOAD_BUFFER_SIZE);
									gu8ar_AntCHTxbuffer[CH_1][0]=gu8_AntReqSharedAddrAtCh1;
									gu8ar_AntCHTxbuffer[CH_1][1]=gu8_AntReqTypeAtCh1 + 0x80;
									gu8ar_AntCHTxbuffer[CH_1][2]=0x00;
									
									if(b.mu8_EpochTimeValidFlag==1)
									{
										gu8ar_AntCHTxbuffer[CH_1][3]=0x01;
										memcpy(&gu8ar_AntCHTxbuffer[CH_1][4],(uint8_t*)&gu8_EpochTime,4);
									}
																
									err_code=sd_ant_acknowledge_message_tx(CHANNEL_1,8,&gu8ar_AntCHTxbuffer[CH_1][0]);
									APP_ERROR_CHECK(err_code);
							
								break;
								
								case DELETE_NODE_REQ:
									
									memset(&gu8ar_AntCHTxbuffer[CH_1][0],0,PAYLOAD_BUFFER_SIZE);
									gu8ar_AntCHTxbuffer[CH_1][0]=gu8_AntReqSharedAddrAtCh1;
									gu8ar_AntCHTxbuffer[CH_1][1]=DELETE_NODE_REQ;
									memcmp(&gu8ar_AntCHTxbuffer[CH_1][4],&gu8_AntReqSrNoAtCh1[0],SR_NO_LENGTH);
								
									err_code=sd_ant_acknowledge_message_tx(CHANNEL_1,8,&gu8ar_AntCHTxbuffer[CH_1][0]);
									APP_ERROR_CHECK(err_code);
								
								break;
							}
							
							//gu8_AntReqTypeAtCh1=0;
							//gu8_AntReqSharedAddrAtCh1=0;
						}
						else
						{
							antPx_SetDataOnCH1(gu8_AntRegState);
						}

					break;
				}
					
			break;
					
			case EVENT_RX:
				
				//antPx_HandleEventRx(event_message_buffer);
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:
						
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("R0=");
						#endif
					
						#ifdef ANT_DATA_DEBUG_ENABLE
					    gen_PrintHexStr(&event_message_buffer[3],8);
						CRLF;
						#endif

						antPx_TaskOnRxAtCH0(event_message_buffer);
						
					break;
					
					case CHANNEL_1:
						
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("R0=");
						#endif
					
						#ifdef ANT_DATA_DEBUG_ENABLE
					    gen_PrintHexStr(&event_message_buffer[3],8);
						CRLF;
						#endif
					
						antPx_TaskOnRxAtCH1(event_message_buffer);
					
					break;
				}

			break;
								
			case EVENT_CHANNEL_CLOSED:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("\r\nCLOSE CH:");
				gen_PrintHex(p_ant_evt->channel);
				CRLF;
				#endif
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:
						
						b.mu8_AntChannelOpenState_ch0=0;					
						//Clear Channel 0 Close flag
						gu8_Ch0_CloseFlag=0;
					
						if(BleBits.mu8_AntCH0_CloseByBLE)
						{
							
						}
						else if(b.mu8_AntChannelCloseByANT)
						{
							
						}
						else
						{
							if(gu8_AntAddrAssignState==4)
							{
								//Get Channel ID parameter of CHANNEL_0
								sd_ant_channel_id_get (CHANNEL_0,&gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum,&gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevType,&gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevTransType);
								
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
								gu8_AntFlashWriteReason=SHARED_ADDR_ADD;
								
								#ifdef ANT_FLASH_DEBUG_ENABLE
								DEBUG("\r\nFLASH WRITE REASON:");
								gen_PrintHex(gu8_AntFlashWriteReason);
								CRLF;
								#endif
								
								#ifdef ANT_DEBUG_ENABLE
								DEBUG("STORE SA\r\n");
								#endif
							}
							else
							{
								#ifdef ANT_DEBUG_ENABLE
								DEBUG("ReOpen CH0\r\n");
								#endif
								
								if(!gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress)
								{
									if(!b.mu8_AntchangeFreq)
									{
										b.mu8_AntchangeFreq=1;
										
										/* Set Channel ID */
										err_code = sd_ant_channel_id_set(CHANNEL_0, 
																		  gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum, 
																		  CHANNEL_0_CHAN_ID_DEV_TYPE_1, 
																		  CHANNEL_0_CHAN_ID_TRANS_TYPE);
										APP_ERROR_CHECK(err_code);
										
										#ifndef ANT_FREQ_AGILITY_ENABLE
										/* Set Channel Radio Frequency */
										err_code = sd_ant_channel_radio_freq_set(CHANNEL_0,CH_FREQUENCY1);
										APP_ERROR_CHECK(err_code);
										#else
										/* Set Frequency Agility */
										err_code = sd_ant_auto_freq_hop_table_set(CHANNEL_0,CH_FREQ1_AGLTY1,CH_FREQ1_AGLTY2,CH_FREQ1_AGLTY3);
										APP_ERROR_CHECK(err_code);
										#endif
										
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("F1\r\n");
										#endif
									}
									else
									{
										b.mu8_AntchangeFreq=0;
										
										/* Set Channel ID */
										err_code = sd_ant_channel_id_set(CHANNEL_0, 
																		  gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum, 
																		  CHANNEL_0_CHAN_ID_DEV_TYPE_0, 
																		  CHANNEL_0_CHAN_ID_TRANS_TYPE);
										APP_ERROR_CHECK(err_code);

										#ifndef ANT_FREQ_AGILITY_ENABLE
										/* Set Channel Radio Frequency */
										err_code = sd_ant_channel_radio_freq_set(CHANNEL_0,CH_FREQUENCY0);
										APP_ERROR_CHECK(err_code);
										#else
										/* Set Frequency Agility */
										err_code = sd_ant_auto_freq_hop_table_set(CHANNEL_0,CH_FREQ0_AGLTY1,CH_FREQ0_AGLTY2,CH_FREQ0_AGLTY3);
										APP_ERROR_CHECK(err_code);
										#endif
										
										#ifdef ANT_DEBUG_ENABLE
										DEBUG("F0\r\n");
										#endif
									}
								}
								else//If Already Registered Node then Make it Factory default
								{
									//Clear Timeout Timer
									gu8_AntSelfConnectionTimeout=0;
									
									#ifdef ANT_FLASH_DEBUG_ENABLE
									DEBUG("EADD2AUN\r\n");
									#endif
									
									//Clear All ANT Parameter
									antP2_ClearDynaConfiguration();
									
									gu8_AntStatusPollingRcv=0;
								}

								//ReOpen Channel 0 agian
								err_code = sd_ant_channel_open(CHANNEL_0);
								APP_ERROR_CHECK(err_code);
								
								b.mu8_AntChannelOpenState_ch0=1;
							}
						}
						
					break;
						
					case CHANNEL_1:
					
						b.mu8_AntChannelOpenState_ch1=0;
					
						//ReOpen Channel 0 agian
						if(BleBits.mu8_AntCH1_CloseByBLE)
						{
							
						}
						else if(b.mu8_AntChannelCloseByANT)
						{
							
						}
						else
						{
							#ifdef ANT_DEBUG_ENABLE
							DEBUG("EADD2AUN\r\n");
							#endif
							
							err_code = sd_ant_channel_open(CHANNEL_1);
							APP_ERROR_CHECK(err_code);
						
							b.mu8_AntChannelOpenState_ch1=1;
						}
					break;
				}
				
			break;
							
			case EVENT_TRANSFER_TX_COMPLETED:
				
//				#ifdef ANT_DEBUG_ENABLE
//				DEBUG("\r\nEVENT_TRANSFER_TX_COMPLETED at CH:");
//				gen_PrintHex(p_ant_evt->channel);
//				DEBUG("->");
//				gen_PrintRcvData(event_message_buffer);
//				#endif
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:
					
						if(!gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress)
						{
							if(gu8_AntAddrAssignState==1)
							{
								gu8_AntAddrAssignState=2;
								
								#ifdef ANT_DEBUG_ENABLE
								DEBUG("S2\r\n");
								#endif
							}
							else if(gu8_AntAddrAssignState==3)
							{
								gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress=gu8_tempSharedAddr;
								gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentSharedAddress=gu8_tempPSharedAddr;
								gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[0]=gu8ar_tempSrNo[0];
								gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[1]=gu8ar_tempSrNo[1];
								gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[2]=gu8ar_tempSrNo[2];
								gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_ParentSrNo[3]=gu8ar_tempSrNo[3];
								gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber=gu8_tempPLayerNo;
								
								gu8_AntAddrAssignState=4;
								
								/* Close Channel */
								err_code = sd_ant_channel_close(CHANNEL_0);
								APP_ERROR_CHECK(err_code);
								
								gu8_AddrAssignTimeout[CH_0]=0;
								
								#ifdef ANT_DEBUG_ENABLE
								DEBUG("S4:SA RECV,Close CH0\r\n");
								#endif
							}
						}
						else
						{
							if(gu8_AntBurstMsgProgressState[CH_0]==BURST_IN_PROGRESS)
							{
								gu8_AntBurstMsgProgressState[CH_0]=NO_BURST_MESSAGE;
							
								gu8_AntBurstTryCntr[CH_0]=0;
								
								#ifdef ANT_BURST_TEST_ENABLE
								DEBUG("CH0 BURST SEND OK\r\n");
								#endif					
								
								if(b.mu8_AntSendAsyncMsgToMaster==1)
								{
									b.mu8_AntSendAsyncMsgToMaster=0;
								}
								else if(b.mu8_AntSendResToMasterForRemoteReq==1)
								{
									#ifdef ANT_OPERATION_COUNT_ENABLE
									gu8_AntBurstOKCount++;
									#endif
									
									b.mu8_AntSendResToMasterForRemoteReq=0;									
								}
								else if(b.mu8_AntFwdMsgToMaster==1)
								{
									b.mu8_AntFwdMsgToMaster=0;
								}
								else if(b.mu8_AntSendMobileMsgToMaster_ch0==1)
								{
									b.mu8_AntSendMobileMsgToMaster_ch0=0;
								}
								else if(b.mu8_AntSendMobileMsgToMaster_ch1==1)
								{
									b.mu8_AntSendMobileMsgToMaster_ch1=0;
								}
								else if(b.mu8_AntSendNodeInfoToMaster==1)
								{
									b.mu8_AntSendNodeInfoToMaster=0;
									
									gu8_AntNoOfUnRegisterNode=0;

									for(uint8_t i=0;i<MAX_NODES_IN_NEXT_LAYER;i++)
									{
										if(gst_AntFlashParams.mst_AntNodeFlashParams[i].mu8_RegisterStatus==E_DEV_NOT_IN_DB)
										{
											gst_AntFlashParams.mst_AntNodeFlashParams[i].mu8_RegisterStatus=E_DEV_IN_DB;
										}
									}

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
									gu8_AntFlashWriteReason=ANT_NODE_REG_ADD;
									
									#ifdef ANT_FLASH_DEBUG_ENABLE
									DEBUG("\r\nFLASH WRITE REASON:");
									gen_PrintHex(gu8_AntFlashWriteReason);
									CRLF;
									#endif
								}

								//Broadcast With Shared address to Stack
								gu8ar_AntCHTxbuffer[CH_0][0]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
								gu8ar_AntCHTxbuffer[CH_0][1]=0x00;
								gu8ar_AntCHTxbuffer[CH_0][2]=0x00;
								gu8ar_AntCHTxbuffer[CH_0][3]=0x00;
								gu8ar_AntCHTxbuffer[CH_0][4]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0];
								gu8ar_AntCHTxbuffer[CH_0][5]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[1];
								gu8ar_AntCHTxbuffer[CH_0][6]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[2];
								gu8ar_AntCHTxbuffer[CH_0][7]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[3];
								
								err_code = sd_ant_broadcast_message_tx(CHANNEL_0,8,&gu8ar_AntCHTxbuffer[CH_0][0]);
								APP_ERROR_CHECK(err_code);
								
								#ifdef ANT_DATA_DEBUG_ENABLE
								gen_PrintHexStr(&gu8ar_AntCHTxbuffer[CH_0][0],8);
								CRLF;
								#endif	
							}
							else if(gu8_AntDeletNextLayerNode[NEAR_DEVICE]==MESSAGE_IN_PROGRESS)
							{
								gu8_AntDeletNextLayerNode[NEAR_DEVICE]=NO_MESSAGE;
							}
							else if(gu8_AntDeletNextLayerNode[FAR_DEVICE]==MESSAGE_IN_PROGRESS)
							{
								gu8_AntDeletNextLayerNode[FAR_DEVICE]=NO_MESSAGE;
							}
							else if(b.mu8_AntGetEpochTime==1)
							{
								if(gu8_AntGetEpochTimeMsgState==MESSAGE_IN_PROGRESS)
								{
									gu8_AntGetEpochTimeMsgState=NO_MESSAGE;
									b.mu8_AntGetEpochTime=0;
									
									#ifdef ANT_EPOCH_LOG_ENABLE
									DEBUG("EPOCH SEND OK @ CH0\r\n");
									#endif
								}
							}
						}
						
					break;
					
					case CHANNEL_1:
												
						if(gu8_AntBurstMsgProgressState[CH_1] == BURST_INITIATED)
						{
							#ifdef ANT_BURST_TEST_ENABLE
							DEBUG("CH1 BURST_INITIATED\r\n");
							#endif
							
							err_code = sd_ant_burst_handler_request(CHANNEL_1,gu8_AntTxBurstMsgSize[CH_1],&gu8ar_AntBurstTxBuffer[CH_1][0],BURST_SEGMENT_START | BURST_SEGMENT_END);
							APP_ERROR_CHECK(err_code);	
														
							gu8_AntBurstMsgProgressState[CH_1] = BURST_IN_PROGRESS;
						}
						else if(gu8_AntBurstMsgProgressState[CH_1]==BURST_IN_PROGRESS)
						{
							gu8_AntBurstMsgProgressState[CH_1]=NO_BURST_MESSAGE;
							gu8_AntBurstTryCntr[CH_1]=0;
							
							#ifdef ANT_BURST_TEST_ENABLE
							DEBUG("CH1 BURST SEND OK\r\n");
							#endif
						}
						else if(gu8_AntReqTypeAtCh1)
						{
							switch(gu8_AntReqTypeAtCh1)
							{
								case EPOCH_TIME_REQ:
									
									#ifdef ANT_EPOCH_LOG_ENABLE
									DEBUG("EPOCH SEND OK @ CH1\r\n");
									#endif
								
								break;
								
								case DELETE_NODE_REQ:
									
									#ifdef ANT_EPOCH_LOG_ENABLE
									DEBUG("DELETE SEND OK @ CH1\r\n");
									#endif
								
								break;
							}
							
							gu8_AntReqSharedAddrAtCh1=0;
							gu8_AntReqTypeAtCh1=0;
						}
						else
						{						
							#ifdef ANT_DEBUG_ENABLE
							DEBUG("POLL OK=");
							gen_PrintHex(gu8_PollingSharedAddr);
							CRLF;
							#endif
							
							gu8ar_NodePollingCount[gu8_PollingSharedAddr-1] = 0;
							
							antPx_SetDataOnCH1(gu8_AntRegState);
						}
							
					break;
				}
				
			break;
										
			case EVENT_TRANSFER_TX_FAILED:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("\r\nTX_FAILED@CH:");
				gen_PrintHex(p_ant_evt->channel);
				//gen_PrintRcvData(event_message_buffer);
				CRLF;
				#endif

				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:
					
						gu8_AntAddrAssignState=0;
					
						if(gu8_AntBurstMsgProgressState[CH_0]==BURST_IN_PROGRESS)
						{
							gu8_AntBurstMsgProgressState[CH_0]=NO_BURST_MESSAGE;
							
							#ifdef ANT_BURST_TEST_ENABLE
							DEBUG("CH0 BURST FAIL 2 M\r\n");
							#endif
							
							if(gu8_AntBurstTryCntr[CH_0])
							{
								gu8_AntBurstTryCntr[CH_0]--;
								
								if(b.mu8_AntSendResToMasterForRemoteReq==1)
								{
									#ifdef ANT_OPERATION_COUNT_ENABLE
									gu8_AntBurstRetryCount++; //For Debug purpose
									#endif
								}	
									
								#ifdef ANT_BURST_TEST_ENABLE
								DEBUG("CH0 BURST RETRY\r\n");
								#endif
							}
							else
							{
								if(b.mu8_AntSendAsyncMsgToMaster==1)
								{
									b.mu8_AntSendAsyncMsgToMaster=0;
								}
								else if(b.mu8_AntSendResToMasterForRemoteReq==1)
								{
									#ifdef ANT_OPERATION_COUNT_ENABLE
									gu8_AntBurstFailCount++; //For Debug purpose
									#endif
									
									b.mu8_AntSendResToMasterForRemoteReq=0;
								}
								else if(b.mu8_AntFwdMsgToMaster==1)
								{
									b.mu8_AntFwdMsgToMaster=0;
								}
								else if(b.mu8_AntSendMobileMsgToMaster_ch0==1)
								{
									b.mu8_AntSendMobileMsgToMaster_ch0=0;
								}
								else if(b.mu8_AntSendMobileMsgToMaster_ch1==1)
								{
									b.mu8_AntSendMobileMsgToMaster_ch1=0;
								}
								else if(b.mu8_AntSendNodeInfoToMaster==1)
								{
									b.mu8_AntSendNodeInfoToMaster=0;
								}
							}

							//Broadcast With Shared address to Stack
							gu8ar_AntCHTxbuffer[CH_0][0]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
							gu8ar_AntCHTxbuffer[CH_0][1]=0x00;
							gu8ar_AntCHTxbuffer[CH_0][2]=0x00;
							gu8ar_AntCHTxbuffer[CH_0][3]=0x00;
							gu8ar_AntCHTxbuffer[CH_0][4]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0];
							gu8ar_AntCHTxbuffer[CH_0][5]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[1];
							gu8ar_AntCHTxbuffer[CH_0][6]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[2];
							gu8ar_AntCHTxbuffer[CH_0][7]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[3];
							
							err_code = sd_ant_broadcast_message_tx(CHANNEL_0,8,&gu8ar_AntCHTxbuffer[CH_0][0]);
							APP_ERROR_CHECK(err_code);

							#ifdef ANT_DATA_DEBUG_ENABLE
							gen_PrintHexStr(&gu8ar_AntCHTxbuffer[CH_0][0],8);
							CRLF;
							#endif	
						}
						else if(gu8_AntDeletNextLayerNode[NEAR_DEVICE]==MESSAGE_IN_PROGRESS)
						{
							gu8_AntDeletNextLayerNode[NEAR_DEVICE]=MESSAGE_INITIATED;
						}
						else if(gu8_AntDeletNextLayerNode[FAR_DEVICE]==MESSAGE_IN_PROGRESS)
						{
							gu8_AntDeletNextLayerNode[FAR_DEVICE]=MESSAGE_INITIATED;
						}
						else if(b.mu8_AntGetEpochTime==1)
						{
							if(gu8_AntGetEpochTimeMsgState==MESSAGE_IN_PROGRESS)
							{
								gu8_AntGetEpochTimeMsgState=MESSAGE_INITIATED;
								DEBUG("EPOCH SEND FAIL @ CH0,TRY AGAIN\r\n");
							}
						}

					break;
					
					case CHANNEL_1:
											
						if(gu8_AntBurstMsgProgressState[CH_1] == BURST_INITIATED)
						{
							#ifdef ANT_BURST_TEST_ENABLE
							DEBUG("CH1 BURST_INITIATED\r\n");
							#endif
							
							err_code = sd_ant_burst_handler_request(CHANNEL_1,gu8_AntTxBurstMsgSize[CH_1],&gu8ar_AntBurstTxBuffer[CH_1][0],BURST_SEGMENT_START | BURST_SEGMENT_END);
							APP_ERROR_CHECK(err_code);	
														
							gu8_AntBurstMsgProgressState[CH_1] = BURST_IN_PROGRESS;
						}
						else if(gu8_AntBurstMsgProgressState[CH_1]==BURST_IN_PROGRESS)
						{
							if(gu8_AntBurstTryCntr[CH_1])
							{
								gu8_AntBurstTryCntr[CH_1]--;

								#ifdef ANT_BURST_TEST_ENABLE
								DEBUG("CH1 BURST FAIL,TRY AGAIN\r\n");
								#endif
								
								//gu8_AntBurstMsgProgressState[CH_1]=BURST_INITIATED;
								
								err_code = sd_ant_burst_handler_request(CHANNEL_1,gu8_AntTxBurstMsgSize[CH_1],&gu8ar_AntBurstTxBuffer[CH_1][0],BURST_SEGMENT_START | BURST_SEGMENT_END);
								APP_ERROR_CHECK(err_code);	
							}
							else
							{
								gu8_AntBurstMsgProgressState[CH_1]=NO_BURST_MESSAGE;
								
								#ifdef ANT_BURST_TEST_ENABLE
								DEBUG("CH1 BURST FAILED\r\n");
								#endif
							}
						}
						else if(gu8_AntReqTypeAtCh1)
						{
							switch(gu8_AntReqTypeAtCh1)
							{
								case EPOCH_TIME_REQ:
									
									#ifdef ANT_EPOCH_LOG_ENABLE
									DEBUG("EPOCH SEND FAIL @ CH1\r\n");
									#endif

									memset(&gu8ar_AntCHTxbuffer[CH_1][0],0,PAYLOAD_BUFFER_SIZE);
									gu8ar_AntCHTxbuffer[CH_1][0]=gu8_AntReqSharedAddrAtCh1;
									gu8ar_AntCHTxbuffer[CH_1][1]=gu8_AntReqTypeAtCh1;
									gu8ar_AntCHTxbuffer[CH_1][2]=0x00;
									
									if(b.mu8_EpochTimeValidFlag==1)
									{
										gu8ar_AntCHTxbuffer[CH_1][3]=0x01;
										memcpy(&gu8ar_AntCHTxbuffer[CH_1][4],(uint8_t*)&gu8_EpochTime,4);
									}
																
									err_code=sd_ant_acknowledge_message_tx(CHANNEL_1,8,&gu8ar_AntCHTxbuffer[CH_1][0]);
									APP_ERROR_CHECK(err_code);
								
								break;
								
								case DELETE_NODE_REQ:
									
									memset(&gu8ar_AntCHTxbuffer[CH_1][0],0,PAYLOAD_BUFFER_SIZE);
									gu8ar_AntCHTxbuffer[CH_1][0]=gu8_AntReqSharedAddrAtCh1;
									gu8ar_AntCHTxbuffer[CH_1][1]=DELETE_NODE_REQ;
									memcmp(&gu8ar_AntCHTxbuffer[CH_1][4],&gu8_AntReqSrNoAtCh1[0],SR_NO_LENGTH);
								
									err_code=sd_ant_acknowledge_message_tx(CHANNEL_1,8,&gu8ar_AntCHTxbuffer[CH_1][0]);
									APP_ERROR_CHECK(err_code);
								
								break;
							}
							
							//gu8_AntReqSharedAddrAtCh1=0;
							//gu8_AntReqTypeAtCh1=0;
							
						}
						else
						{
							#ifdef ANT_DEBUG_ENABLE
							DEBUG("POLL FAIL=");
							gen_PrintHex(gu8_PollingSharedAddr);
							CRLF;
							#endif
							
							antPx_SetDataOnCH1(gu8_AntRegState);
						}
						
					break;
				}

			break;
					
			case EVENT_TRANSFER_RX_FAILED:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("\r\nTX_RX_FAILED@CH:");
				gen_PrintHex(p_ant_evt->channel);
				//gen_PrintRcvData(event_message_buffer);
				#endif
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:
					
						gu8_AntAddrAssignState=0;
						gu8_BurstRcvInd[CH_0]=0;
					
					break;
					
					case CHANNEL_1:

						gu8_BurstRcvInd[CH_1]=0;
					
					break;
				}
				
			break;
				
			case EVENT_TRANSFER_NEXT_DATA_BLOCK:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("TX_NEXT_DATA_BLOCK@CH:");	
				gen_PrintHex(p_ant_evt->channel);
				CRLF;
				#endif		
				
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:
					
//						if(gu8_AntBurstMsgProgressState[CH_0]==BURST_IN_PROGRESS)
//						{
//							if(gu8_AntTxBurstMsgBlock[CH_0]) 
//							{
//								gu8_AntTxBurstMsgBlock[CH_0]--;

//								gu8_AntTxBurstMsgSize[CH_0] += 8;
//								
//								#ifdef ANT_DEBUG_ENABLE
//								gen_PrintHexStr(&gu8ar_AntBurstTxBuffer[CH_0][gu8_AntTxBurstMsgSize[CH_0]],8);
//								CRLF;
//								#endif		
//								
//								if(gu8_AntTxBurstMsgBlock[CH_0])
//								{
//									err_code = sd_ant_burst_handler_request(CHANNEL_0, 8 , &gu8ar_AntBurstTxBuffer[CH_0][gu8_AntTxBurstMsgSize[CH_0]], BURST_SEGMENT_CONTINUE);
//								}
//								else
//								{
//									err_code = sd_ant_burst_handler_request(CHANNEL_0, 8 , &gu8ar_AntBurstTxBuffer[CH_0][gu8_AntTxBurstMsgSize[CH_0]], BURST_SEGMENT_END);
//								}
//								APP_ERROR_CHECK(err_code);
//							}
//						}
					
					break;
					
					case CHANNEL_1:
					
//						if(gu8_AntBurstMsgProgressState[CH_1]==BURST_IN_PROGRESS)
//						{
//							if(gu8_AntTxBurstMsgBlock[CH_1]) 
//							{
//								gu8_AntTxBurstMsgBlock[CH_1]--;

//								gu8_AntTxBurstMsgSize[CH_1] += 8;
//								
//								if(gu8_AntTxBurstMsgBlock[CH_1])
//								{
//									err_code = sd_ant_burst_handler_request(CHANNEL_1, 8 , &gu8ar_AntBurstTxBuffer[CH_1][gu8_AntTxBurstMsgSize[CH_1]], BURST_SEGMENT_CONTINUE);
//								}
//								else
//								{
//									err_code = sd_ant_burst_handler_request(CHANNEL_1, 8 , &gu8ar_AntBurstTxBuffer[CH_1][gu8_AntTxBurstMsgSize[CH_1]], BURST_SEGMENT_END);
//								}
//								APP_ERROR_CHECK(err_code);
//							}
//						}
						
					break;
				}
				
			break;	
			
			case EVENT_TRANSFER_TX_START:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("TX_START@CH:");	
				gen_PrintHex(p_ant_evt->channel);
				//gen_PrintRcvData(event_message_buffer);
				CRLF;
				#endif
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:

					break;
					
					case CHANNEL_1:

					break;
				}
				
			break;	
				
			case EVENT_RX_SEARCH_TIMEOUT:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("\r\nRX_SEARCH_TIMEOUT@CH:");
				gen_PrintHex(p_ant_evt->channel);
				CRLF;
				#endif
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:

					break;
					
					case CHANNEL_1:

					break;
				}
				
			break;
									
			case EVENT_RX_FAIL:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("RX_FAIL@CH:");
				gen_PrintHex(p_ant_evt->channel);
				CRLF;
				#endif
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:

					break;
					
					case CHANNEL_1:

					break;
				}
				
			break;

			case EVENT_RX_FAIL_GO_TO_SEARCH:
				
				#ifdef ANT_DEBUG_ENABLE
				DEBUG("\r\nRX_FAIL_GO_TO_SEARCH@CH:");
				gen_PrintHex(p_ant_evt->channel);
				CRLF;
				#endif
			
				switch(p_ant_evt->channel)
				{
					case CHANNEL_0:

					break;
					
					case CHANNEL_1:
					
					break;
				}
				
			break;
			
			default:
				break;
        }
    }			
}

/**
 * @brief Set Broadcast data CH 1
 */
void antPx_SetDataOnCH1(uint8_t Type)
{
	uint32_t err_code;
	static uint8_t slu8_toggleByte=0,gu8_BroadCastPollingIndx=1;
	uint8_t lu8_LoopIndex=0;
	
	memset(&gu8ar_AntCHTxbuffer[CH_1][0],0,PAYLOAD_BUFFER_SIZE);
	
	switch(Type)
	{
		case ADD_AVAILABEL:
		case ADD_FULL:
			
			gu8_AddrAssignTimeout[CH_1]=0;
		
			if((gu16_AntNextLayerNodesPollStatus) && (slu8_toggleByte>1))
			{
				for(lu8_LoopIndex = 0;lu8_LoopIndex < ANT_MAX_NODE_IN_IMMIDIATE_NEXT_LAYER;lu8_LoopIndex++,gu8_BroadCastPollingIndx++)
				{
					if(gu8_BroadCastPollingIndx > ANT_MAX_NODE_IN_IMMIDIATE_NEXT_LAYER)
					{
						gu8_BroadCastPollingIndx = 1;
					}
					
					if(gu16_AntNextLayerNodesPollStatus & (1<<gu8_BroadCastPollingIndx))
					{	
						break;
					}
				}
				
				//Store address for status update purpose.
				gu8_PollingSharedAddr = gu8_BroadCastPollingIndx;
				
				if(slu8_toggleByte>=3)
				{
					slu8_toggleByte = 1;
					gu8_BroadCastPollingIndx++;							//Increment so that next time another shared address will be used.
				}
				else
				{
					slu8_toggleByte++;
				}
				
				gu8ar_AntCHTxbuffer[CH_1][SHARED_ADD_IDX] = gu8_PollingSharedAddr;
				gu8ar_AntCHTxbuffer[CH_1][POLL_CMD_ID_INDX] = STATUS_UPDATE_REQ;
				
				//Add Serial # of polled Node
				for(lu8_LoopIndex = 0;lu8_LoopIndex <MAX_NODES_IN_NEXT_LAYER;lu8_LoopIndex++)
				{
					if((gst_AntFlashParams.mst_AntNodeFlashParams[lu8_LoopIndex].mu8_DeviceSharedAddress == gu8_PollingSharedAddr)\
						&&(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_LoopIndex].mu8_ParentLayerNumber == (gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber+1)))
					{	
						memcpy(&gu8ar_AntCHTxbuffer[CH_1][POLL_SLAVE_DEV_ID],gst_AntFlashParams.mst_AntNodeFlashParams[lu8_LoopIndex].mu8ar_DeviceSrNo,SR_NO_LENGTH);
					}
				}
				
				//Check if Number of Poll Retrial is more than  limit then take Acion. 
				if(gu8ar_NodePollingCount[gu8_PollingSharedAddr-1] > MAX_POLLING_COUNT_VALUE)
				{
					antI2_UpdatePollingStatus();
				}
				else
				{
					gu8ar_NodePollingCount[gu8_PollingSharedAddr-1]++;
				}

				err_code = sd_ant_acknowledge_message_tx(CHANNEL_1, PAYLOAD_BUFFER_SIZE, &gu8ar_AntCHTxbuffer[CH_1][0]);
				APP_ERROR_CHECK(err_code);
			}
			else
			{
				//Toggle the flag to Poll Request after every four Address available request. 
				//slu8_toggleByte++;
				//if(slu8_toggleByte>1) slu8_toggleByte = 0;
				slu8_toggleByte++;
		
				gu8ar_AntCHTxbuffer[CH_1][SHARED_ADD_IDX] = GLOBAL_SHARED_ADD;
				
				if(Type==ADD_AVAILABEL)
					gu8ar_AntCHTxbuffer[CH_1][APP_MSG_ID_IDX] = ADD_AVAILABEL;
				else
					gu8ar_AntCHTxbuffer[CH_1][APP_MSG_ID_IDX] = ADD_FULL;
				
				memcpy(&gu8ar_AntCHTxbuffer[CH_1][2],&gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0],SR_NO_LENGTH);
				gu8ar_AntCHTxbuffer[CH_1][6]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber+1;
				gu8ar_AntCHTxbuffer[CH_1][NXT_SHR_ADD_IDX]=gu8_NextSharedAddress;	
				
				err_code = sd_ant_broadcast_message_tx(CHANNEL_1, PAYLOAD_BUFFER_SIZE, &gu8ar_AntCHTxbuffer[CH_1][0]);
				APP_ERROR_CHECK(err_code);
			}
			
		break;
		
		case BUSY_ACQUIRING:

			gu8ar_AntCHTxbuffer[CH_1][SHARED_ADD_IDX]=GLOBAL_SHARED_ADD;
			gu8ar_AntCHTxbuffer[CH_1][APP_MSG_ID_IDX]=BUSY_ACQUIRING;
			gu8ar_AntCHTxbuffer[CH_1][3]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
			memcpy(&gu8ar_AntCHTxbuffer[CH_1][4],&gst_AntTempPara.mu8ar_DeviceSrNo[0],SR_NO_LENGTH);		
			
			err_code = sd_ant_broadcast_message_tx(CHANNEL_1, PAYLOAD_BUFFER_SIZE, &gu8ar_AntCHTxbuffer[CH_1][0]);
			APP_ERROR_CHECK(err_code);
		
		break;
			
		default:
			
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("Invalid Case\r\n");	
			#endif
				
		break;
	}

	#ifdef ANT_DEBUG_ENABLE
	DEBUG("T1=");
	#endif
	
	#ifdef ANT_DATA_DEBUG_ENABLE
	gen_PrintHexStr(&gu8ar_AntCHTxbuffer[CH_1][0],8);
	CRLF;
	#endif
}

/**
 * @brief Send Layer2 Device Info to GW
 */
void antPx_SendAddNodeRequest(void)
{
	uint32_t err_code;
	uint8_t lu8_i=0;
	uint8_t lu8_Ind=0;
	
	//Clear Buffer
	memset(&gu8ar_AntBurstTxBuffer[CH_0][0],0x00,sizeof(gu8ar_AntBurstTxBuffer[CH_0]));
	
	//Shared Address
	gu8ar_AntBurstTxBuffer[CH_0][BRUST_CMD_SHARED_ID_INDEX] = gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
	
	//Command ID
	gu8ar_AntBurstTxBuffer[CH_0][BURST_CMD_ID_INDEX] = ADD_NODE_REQ;
	
	//Copy a Buffer to another buffer to send it to GW using Burst data transfer
	gu8ar_AntBurstTxBuffer[CH_0][BURST_NUM_OF_DEV_INDEX] = gu8_AntNoOfUnRegisterNode;			//# of Node Info in message
	gu8ar_AntBurstTxBuffer[CH_0][BRUST_SINGE_DEV_INFO_LEN] = sizeof(st_AntNodeFlashParams_t);	//Size of Node Info message
	lu8_Ind=BRUST_INFO_START_INDEX;
	
	for(lu8_i=0;lu8_i<MAX_NODES_IN_NEXT_LAYER;lu8_i++)
	{
		if(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i].mu8_RegisterStatus==E_DEV_NOT_IN_DB)
		{
			memcpy(&gu8ar_AntBurstTxBuffer[CH_0][lu8_Ind],(uint8_t *)&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_i],sizeof(st_AntNodeFlashParams_t));
			lu8_Ind += sizeof(st_AntNodeFlashParams_t);
		}
	}
	
	while(lu8_Ind%8)
	{
		gu8ar_AntBurstTxBuffer[CH_0][lu8_Ind++]=0x00;
	}
	
	gu8_AntTxBurstMsgSize[CH_0]=lu8_Ind;
	err_code = sd_ant_burst_handler_request(CHANNEL_0, gu8_AntTxBurstMsgSize[CH_0] , &gu8ar_AntBurstTxBuffer[CH_0][0], BURST_SEGMENT_START | BURST_SEGMENT_END);
	APP_ERROR_CHECK(err_code);	
		
	gu8_AntBurstMsgProgressState[CH_0] = BURST_IN_PROGRESS;
	
	gu8_AntBurstTryCntr[CH_0]=BURST_NODE_ADD_SEND_RETRY_COUNTER;
}

/**
*   @Description : 
*   When parsing UART message if found the message is intended to operate Slave device in Bridge network,
*   then search layer1 (Slave device which is directly connected to Bridge(ANT).)
*
*  @params[IN]  : lu8p_DestDevice - Destination device ID,
*  @params[out] : lu8p_DevSharedAdd - shared Address of the node to be searched will be written if found
*  @return : SUCCESS or FAIL
*/
uint8_t antP1_SearchDevice(uint8_t *lu8p_DevSharedAdd, uint8_t *lu8p_DestDeviceSrNo)
{
	uint8_t lu8_DevIndex;
	uint8_t lu8ar_DeviceSrNo[4] = {0};
	
	memcpy(lu8ar_DeviceSrNo,lu8p_DestDeviceSrNo,4);
	*lu8p_DevSharedAdd = 0xff; //Invalid device.
	
	for(lu8_DevIndex = 0;lu8_DevIndex < MAX_NODES_IN_NEXT_LAYER; lu8_DevIndex++)
	{
		if(memcmp(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_DevIndex].mu8ar_DeviceSrNo,lu8ar_DeviceSrNo,4) == 0)
		{
			if(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_DevIndex].mu8_ParentLayerNumber == (gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber+1))
			{
				//Near Device Found(message will be forwarded to this device.)
				*lu8p_DevSharedAdd = gst_AntFlashParams.mst_AntNodeFlashParams[lu8_DevIndex].mu8_DeviceSharedAddress;
				return SUCCESS;
			}
			else
			{//Device is not in layer1 so keep searching using parent device serial #
				memcpy(lu8ar_DeviceSrNo,gst_AntFlashParams.mst_AntNodeFlashParams[lu8_DevIndex].mu8ar_ParentSrNo,4);
				lu8_DevIndex = 0xff;
			}
		}
	}
	return FAIL;
}

/** @brief      - update polling status of Next immidiate nodes.
*   @Params[in] - lu8p_SharedAddress - shared address of node
*   @Params[in] - lu8_status - status of node
* 	@returns    - None.
*/
void antI2_UpdatePollingStatus(void)
{ 
	#ifdef ANT_NODE_ADD_REMOVE_ENABLE
	DEBUG("\r\nPOLL TOUT:");
	gen_PrintHex(gu8_PollingSharedAddr);
	CRLF;
	#endif
	
	//Clear Shared Address bit for perticular Node
	gu16_AntNextLayerNodesPollStatus &= ~(1<<gu8_PollingSharedAddr);
	
	//Search the Node and Delete the Entry. :Modify search function
	for(uint8_t lu8_Index = 0;lu8_Index < MAX_NODES_IN_NEXT_LAYER; lu8_Index++)
	{
		if((gst_AntFlashParams.mst_AntNodeFlashParams[lu8_Index].mu8_ParentLayerNumber == (gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber+1))
		&&(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_Index].mu8_DeviceSharedAddress ==  gu8_PollingSharedAddr))
		{
			if(gu8_AntDeletNextLayerNode[NEAR_DEVICE]==NO_MESSAGE)
			{
				gu8ar_NodePollingCount[gu8_PollingSharedAddr-1]=0;
			
				gu8ar_AntCHTxbuffer[CH_0][0]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
				gu8ar_AntCHTxbuffer[CH_0][1]=REMOVE_L2_NODE_REQ;
				gu8ar_AntCHTxbuffer[CH_0][2]=0x00;
				gu8ar_AntCHTxbuffer[CH_0][3]=0x00;
				gu8ar_AntCHTxbuffer[CH_0][4]=gst_AntFlashParams.mst_AntNodeFlashParams[lu8_Index].mu8ar_DeviceSrNo[0];
				gu8ar_AntCHTxbuffer[CH_0][5]=gst_AntFlashParams.mst_AntNodeFlashParams[lu8_Index].mu8ar_DeviceSrNo[1];
				gu8ar_AntCHTxbuffer[CH_0][6]=gst_AntFlashParams.mst_AntNodeFlashParams[lu8_Index].mu8ar_DeviceSrNo[2];
				gu8ar_AntCHTxbuffer[CH_0][7]=gst_AntFlashParams.mst_AntNodeFlashParams[lu8_Index].mu8ar_DeviceSrNo[3];
							
				//Delete Status polling time out Node
				antI3_DeleteNode(lu8_Index);
				
				//Find Next Shared Address from status Update Node
				antI4_GetNextSharedAddress();
				
				//Update SlaveTrack Index.
				if(FAIL == antI4_GetNodeParamsWrIndx())
				{
					
				}
				
				gu8_AntDeletNextLayerNode[NEAR_DEVICE]=MESSAGE_INITIATED;
			}
			
			return;
		}
	}
}

/** @brief      - function used to find next free Shared Address from device Database.
*   @Params[in] - None
*   @returns    - None.
*/
void antI4_GetNextSharedAddress(void)
{
	uint8_t lu8_SearchIndex;
	for(lu8_SearchIndex = 1;lu8_SearchIndex <= ANT_MAX_NODE_IN_IMMIDIATE_NEXT_LAYER ;lu8_SearchIndex++)
	{
		//Get Next Shared Address
		if(!(gu16_AntNextLayerNodesPollStatus & (1<< lu8_SearchIndex)))
		{
			gu8_NextSharedAddress = lu8_SearchIndex;
			gu8_AntRegState=ADD_AVAILABEL;
			
			#ifdef ANT_NODE_ADD_REMOVE_ENABLE
			DEBUG("NextSA:");
			gen_PrintHex(gu8_NextSharedAddress);
			CRLF;
			#endif
			
			return;
		} 
	}
	
	gu8_NextSharedAddress = 0xFF;
	gu8_AntRegState=ADD_FULL;
	
	#ifdef ANT_NODE_ADD_REMOVE_ENABLE
	DEBUG("SharedAddrFull\r\n");
	#endif
}

/** @brief      - function used to find next free Flash space Index.
*   @Params[in] - None
*	@returns    - None.
*/
uint8_t antI4_GetNodeParamsWrIndx(void)  
{
	uint8_t lu8_SearchIndex;
	for(lu8_SearchIndex = 0;lu8_SearchIndex < MAX_NODES_IN_NEXT_LAYER ;lu8_SearchIndex++)
	{
		//Get next Free space for writing into Flash
		if(memcmp(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_SearchIndex].mu8ar_DeviceSrNo,gu8ar_NullSrNumber,4) == 0)
		{	
			//Found Free Device.
			gu8_AddedSlaveNodeIndex = lu8_SearchIndex;
			
			#ifdef ANT_NODE_ADD_REMOVE_ENABLE
			DEBUG("NEXT Store IND:");
			gen_PrintHex(gu8_AddedSlaveNodeIndex);
			CRLF;
			#endif
			
			return SUCCESS;
		}
	}
	gu8_AntRegState=ADD_FULL;
	
	#ifdef ANT_NODE_ADD_REMOVE_ENABLE
	DEBUG("IND FULL=");
	gen_PrintHex(gu8_AddedSlaveNodeIndex);
	CRLF;
	#endif
	
	return FAIL;
}

/** @brief      - This function is used to delete device info from flash
*   @Params[in] - lu8_NodeIndex : Index of Ram coy of Device Data base to be deleted.	
*	@returns    - None.
	@Note       - This Function will work only if number of Network layer is less than 2.
*/
void antI3_DeleteNode(uint8_t lu8_NodeIndex)
{
	uint8_t lu8ar_NodeSrNumber[4] = {0};
	uint8_t lu8_LoopIndex=0;
	
	memcpy(lu8ar_NodeSrNumber,&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_NodeIndex].mu8ar_DeviceSrNo,4);
	
	#ifdef ANT_NODE_ADD_REMOVE_ENABLE
	DEBUG("DEL:");
	gen_PrintHexStr(&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_NodeIndex].mu8ar_DeviceSrNo[0],4);
	DEBUG(" @Ind=");
	gen_PrintHex(lu8_NodeIndex);
	CRLF;
	#endif
	
	//if the node removal request comes because of status polling timeout, then need to remove all the child of this node.
	if(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_NodeIndex].mu8_ParentLayerNumber==(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber+1))
	{
		for(lu8_LoopIndex = 0;lu8_LoopIndex < MAX_NODES_IN_NEXT_LAYER; lu8_LoopIndex++)
		{
			//this loop will delete all the Child node entries from RAM copy, whose parent removal request received.
			if(memcmp(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_LoopIndex].mu8ar_ParentSrNo,lu8ar_NodeSrNumber,SR_NO_LENGTH) == 0)
			{
				memset(&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_LoopIndex],0x00,sizeof(st_AntNodeFlashParams_t));
			}
		}
		
		//update the Layer1 Node status Bit map varible...
		gu16_AntNextLayerNodesPollStatus &= ~(1 << gst_AntFlashParams.mst_AntNodeFlashParams[lu8_NodeIndex].mu8_DeviceSharedAddress);
	}
	
	//Delete Node itself
	memset(&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_NodeIndex],0x00,sizeof(st_AntNodeFlashParams_t));
	
	//Write updated info in to flash.
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
	gu8_AntFlashWriteReason=ANT_NODE_DEL_WRITE;
	
	#ifdef ANT_FLASH_DEBUG_ENABLE
	DEBUG("\r\nFLASH WRITE REASON:");
	gen_PrintHex(gu8_AntFlashWriteReason);
	CRLF;
	#endif
}

/** @brief       - IF New device to be added this function verifies whether it is aleady present or not
*   @Params[in]  - *lstp_NodeInfo - structure pointer of st_AntNodeInfo_t :pointing to Node data
*	@Params[out] - *lu8p_NodeIndex -	pointer to Node index found in search procedure
*   @returns     - NODE_ALREADY_REGISTERED : Device info already in database.
*				 - NODE_NOT_REGISTERED 	   : Device Not in database.
*/
uint8_t antI3_SearchDuplicateNode(st_AntNodeFlashParams_t *lstp_NodeInfo ,uint8_t *lu8p_NodeIndex)
{
	uint8_t lu8_NodeIndex;
	
	if(!memcmp(lstp_NodeInfo->mu8ar_DeviceSrNo,gu8ar_NullSrNumber,SR_NO_LENGTH))
		return NODE_NOT_REGISTERED;
	
	for(lu8_NodeIndex = 0;lu8_NodeIndex < MAX_NODES_IN_NEXT_LAYER; lu8_NodeIndex++)
	{
		if(memcmp(&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_NodeIndex].mu8ar_DeviceSrNo[0],\
		       lstp_NodeInfo->mu8ar_DeviceSrNo,SR_NO_LENGTH) == NODE_ALREADY_REGISTERED)
		{		
			*lu8p_NodeIndex = lu8_NodeIndex;
			
			//If device is in Layer 1 then Copy back the info for taking other decision
			//if(gst_AntFlashParams.mst_AntNodeFlashParams[lu8_NodeIndex].mu8_ParentLayerNumber == 0)
			{
				//Write Back structure in case of device is in layer 1.
				memcpy(lstp_NodeInfo,&gst_AntFlashParams.mst_AntNodeFlashParams[lu8_NodeIndex],sizeof(st_AntNodeFlashParams_t));
			}
			return NODE_ALREADY_REGISTERED;
		}
	}
	return NODE_NOT_REGISTERED;
}

//**************************************************************//
// Decrypt the data buffer
//**************************************************************//
void antPx_DecryptData(uint8_t *aes_key,uint8_t *iv,uint8_t *data_buffer,uint8_t length)
{
	uint8_t lu8_length=0;
	lu8_length = length % 16;
	
	if(length > 16)
	{
		length = length + (16 - lu8_length);
	}
	else
	{
		length = 16;
	}
	aes_128_cbc_decrypt(aes_key, iv,data_buffer, length);
}

/**@brief ANT CHANNEL_CLOSED event handler.
 */
//void on_ant_evt_channel_closed(void)
//{
// 
//	return;
//        //open_ant_and_ble_channel();
//}
