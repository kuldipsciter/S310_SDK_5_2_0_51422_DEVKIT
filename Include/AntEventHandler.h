/*................................................................................/
Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.

* The information contained herein is property of Nordic Semiconductor ASA.
* Terms and conditions of usage are described in detail in NORDIC
* SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
*
* Licensees are granted free, non-transferable use of the information. NO
* WARRANTY of ANY KIND is provided. This heading must NOT be removed from
* the file.


File Name : AntEventHandler.h

File type : Header File

Compiler Version : Keil Uvision Ver 4.73.0.0

Created on : 17/08/2014

File Description : Handles all the events related to ANT
*.................................................................................*/



#ifndef _ANT_EVNT_HNDLR_H
#define _ANT_EVNT_HNDLR_H

#include "ant_stack_handler_types.h" 
#include "ble_srv_common.h"
#include "main.h"

//**********************************************************************************
//	Debug macros.
//**********************************************************************************
#define DISP_DEVICE_INFO
#define ANT_DEBUG_ENABLE  
#define ANT_DATA_DEBUG_ENABLE 
#define ANT_BURST_TEST_ENABLE
#define ANT_NODE_ADD_REMOVE_ENABLE
#define ANT_FLASH_DEBUG_ENABLE
#define ANT_EPOCH_LOG_ENABLE
//#define ANT_OPERATION_COUNT_ENABLE

//**********************************************************************************
//	Functional macros.
//**********************************************************************************
#define ASYNC_MSG_UNENCRYPTED
//#define ANT_FREQ_AGILITY_ENABLE 

//**********************************************************************************
//	ANT related macros.
//**********************************************************************************

#define ANT_BUFFER_INDEX_MESG_ID        0x01                      /**< Index for Message ID. */
#define ANT_BUFFER_INDEX_MESG_DATA      0x03                      /**< Index for Data. */

#define FLASH_PAGE_PERM_PARA          	241
#define FLASH_PAGE_DYN_PARA           	242

#define ANT_NETWORK_KEY         		{0,0,0,0,0,0,0,0}

#define ANT_CHANNEL_DEFAULT_NETWORK     ((uint8_t) 0x01)      ///< ANT Channel Network

/* Channel Configuration */
#define CHANNEL_0                       ((uint8_t) 0x00)      ///< ANT Channel 0
#define CHANNEL_0_TX_CHANNEL_PERIOD     ((uint16_t)8192)     ///< Channel period 1 Hz
#ifndef ANT_FREQ_AGILITY_ENABLE
	#define CHANNEL_0_ANT_EXT_ASSIGN        ((uint8_t) 0x00)      ///< ANT Ext Assign
#else
	#define CHANNEL_0_ANT_EXT_ASSIGN        ((uint8_t) EXT_PARAM_FREQUENCY_AGILITY)      ///< ANT Ext Assign
#endif



#define CHANNEL_1                       ((uint8_t) 0x01)      ///< ANT Channel 1
#define CHANNEL_1_TX_CHANNEL_PERIOD     ((uint16_t)8192)     ///< Channel period 1 Hz
#ifndef ANT_FREQ_AGILITY_ENABLE
	#define CHANNEL_1_ANT_EXT_ASSIGN        ((uint8_t) 0x00)      ///< ANT Ext Assign
#else
	#define CHANNEL_1_ANT_EXT_ASSIGN        ((uint8_t) EXT_PARAM_FREQUENCY_AGILITY)      ///< ANT Ext Assign
#endif

/* Channel ID configuration */
#define CHANNEL_0_CHAN_ID_DEV_NUM       ((uint16_t) 0x0000)   ///< Device number
#define CHANNEL_0_CHAN_ID_DEV_TYPE_0    ((uint8_t) 0x04)      ///< Device type 0
#define CHANNEL_0_CHAN_ID_DEV_TYPE_1    ((uint8_t) 0x05)      ///< Device type 1
#define CHANNEL_0_CHAN_ID_TRANS_TYPE    ((uint8_t) 0x02)      ///< Transmission type 

#define CHANNEL_1_CHAN_ID_DEV_NUM       ((uint16_t) 0x0000)     ///< Device number
#define CHANNEL_1_CHAN_ID_DEV_TYPE      ((uint8_t) 0x05)      ///< Device type 
#define CHANNEL_1_CHAN_ID_TRANS_TYPE    ((uint8_t) 0x02)      ///< Transmission type

//#define STATUS_POLLING_TIME   			(5)					  ///< Status Polling Time = STATUS_POLLING_TIME times Channel Period

//**********************************************************************************
//	Device Info Related Macro for ANT
//**********************************************************************************
#define MAINS_POWERED					0
#define BATTERY_POWERED					1

#define CH_FREQUENCY0					25//10
#define CH_FREQUENCY1					25//10

#define CH_FREQ0_AGLTY1					CH_FREQUENCY0
#define CH_FREQ0_AGLTY2					40
#define CH_FREQ0_AGLTY3					70

#define CH_FREQ1_AGLTY1					CH_FREQUENCY1
#define CH_FREQ1_AGLTY2					40
#define CH_FREQ1_AGLTY3					70

//**********************************************************************************
//	Index in Application Message  
//**********************************************************************************
#define SHARED_ADD_IDX					0x00
#define APP_MSG_ID_IDX					0x01
#define NXT_SHR_ADD_IDX					0x07
#define GLOBAL_SHARED_ADD_IDX			0x00

// Macro For status poll message
#define POLL_CMD_ID_INDX				0x01
#define POLL_SLAVE_DEV_ID				0x04

#define GLOBAL_SHARED_ADD				0x00
//**********************************************************************************
//	Index in Stack Message  
//**********************************************************************************
#define CHANNEL_IDX_ID					0x02
#define APP_MSG_IDX_ID					0x04
#define SR_NO_IDX_ID					0x07

//**********************************************************************************
//	Command used at Application Level  
//**********************************************************************************
#define REMOTE_OP_REQ					0x01				//Burst Message (Master -> Slave)
#define REMOTE_OP_RES					(REMOTE_OP_REQ + 0x80) //Burst Message (Slave -> Master)
#define ADD_NODE_REQ					0x02		//Burst Message (Slave -> Master)
#define ADD_NODE_RES					(ADD_NODE_REQ + 0x80)		//Burst Message (Master -> Slave)
#define REMOVE_L2_NODE_REQ				0x03		//Ack Message (Slave -> Master)
#define STATUS_UPDATE_REQ				0x04		//Broadcast Message (Master -> Slave)
#define ASYNC_EVENT_REQ					0x05		//Burst Message (Slave -> Master)
#define ASYNC_EVENT_RES					(ASYNC_EVENT_REQ + 0x80)		//Burst Message (Master -> Slave)
#define BLE_OP_REQ						0x06		//Burst Message (Slave -> Master)
#define EPOCH_TIME_REQ					0x07		//Ack Message (Slave -> Master)
#define EPOCH_TIME_RES					(EPOCH_TIME_REQ + 0x80)		//Ack Message (Master -> Slave)
#define DELETE_NODE_REQ					0x08		//Ack Message (Master -> Slave)

//For Address Assignment
#define ADD_AVAILABEL					0xFF		//Broadcast Message (Master -> Slave)
#define BUSY_ACQUIRING					0xFE		//Ack Message (Slave -> Master)
#define REQUEST_ADD						0xFD		//Broadcast Message (Master -> Slave)
#define CONFIRM_ACQUIRE					0xFC		//Ack Message (Slave -> Master)
#define ADD_FULL						0xFB		//Broadcast Message (Master -> Slave)


//Remote Requests
typedef enum
{
	E_ADD_DEV_REQUEST = 1,
	E_REMOVE_DEV_REQUEST,
	E_OPERATION_REQUEST,
	E_STATUS_REQUEST,
	E_ADD_USER_ACCESS_DEVICE_REQUEST,
	E_REMOVE_USER_ACCESS_DEVICE_REQUEST,
	E_AYNCHRONOUS_EVENT_REPORT_RESP,
	E_GET_TIME_RESP,
	E_RESET_ANT_MASTER,
	E_START_ANT_NETWORK,
	E_GET_CONFIG_INFO
	
}en_srpRequest_t;

//Operation Table opCodes
typedef enum
{
	E_DOOR_UNLOCK   = 1,
	E_GARAGE_OPEN   = 1,
	E_GARAGE_CLOSE  = 2,
	E_DOOR_LOCK     = 2,
	E_OPERATE_RELAY_MOMENTARY,
	E_OPERATE_RELAY_MAINTAIN,
	E_OPERATE_SOLANOID,
	E_MOTOR_FORWARD,
	E_MOTOR_REVERSE,
	E_READ_TEMPERATURE,
	E_LIGHT_ON,
	E_LIGHT_OFF,
	E_INTENSITY_OF_LIGHT,
	E_READ_ACCELEROMETER,
	E_READ_BATTERY,
	E_WATER_SENSOR_STATUS,
	E_MOTION_DETECTION,
	E_READ_HUMIDITY,
	E_SWITCH_STATUS,
	E_RELAY_STATUS = 18
}en_Opcode_t;

typedef enum
{
	E_OP_STATUS_SUCCESS = 1,
	E_OP_STATUS_DEVICE_NOT_INITIALIZED,
	E_OP_STATUS_SR_NUM_MISMATCH,
	E_OP_STATUS_SECURE_KEY_MISMATCH,
	E_OP_STATUS_INVALID_OP_REQ  = 5,
	E_OP_STATUS_OPERATION_TIMER_EXPIRED,
	E_OP_STATUS_LOCK_BUSY,
	E_OP_STATUS_USER_NOT_FOUND,
	E_OP_STATUS_NUMBER_OF_USER_LIMIT_EXCEEDED = 12,
	E_OP_STATUS_ALREADY_LOCKED = 17,
	E_OP_STATUS_ALREADY_UNLOCKED,
	E_OP_STATUS_MOTION_DETECTED = 20,
	E_OP_STATUS_FAIL,
	E_OP_STATUS_VERY_LOW_BATTERY,
	E_OP_STATUS_EXT_BUTN_UNLOCK_DISABLED,
	E_OP_STATUS_USER_NOT_IN_PROXIMITY,
	E_OP_STATUS_WAKEUP_SWT_ERROR,
	E_OP_STATUS_OPTO_ERROR,
	E_OP_STATUS_SMART_BUTTON_LOCK_SUCCESS,
	E_OP_STATUS_SMART_BUTTON_UNLOCK_SUCCESS,
	E_OP_STATUS_NO_AUTORISED_USER_CONNECTED,
	E_OP_STATUS_MANUAL_CONNECTION,
	E_OP_STATUS_SESSION_ID_MISMATCH = 64,
	E_OP_STATUS_INVALID_COMMAND_ID_REQ,
	E_OP_STATUS_SESSION_ID_NOT_REQUIRED
	
}en_UartRespCode_t;

//**********************************************************************************
//	Message sending status Using Burst 
//**********************************************************************************
#define NO_BURST_MESSAGE				0x00
#define BURST_INITIATED					0x01
#define BURST_IN_PROGRESS				0x02

#define BURST_MSG_RETRY_CH0_COUNTER		1//3
#define BURST_MSG_RETRY_CH1_COUNTER		1//3
#define BURST_NODE_ADD_SEND_RETRY_COUNTER			5

//**********************************************************************************
//	Index in Burst Message  
//**********************************************************************************
#define BRUST_CMD_SHARED_ID_INDEX		0x00	
#define BURST_CMD_ID_INDEX				(BRUST_CMD_SHARED_ID_INDEX + 1)

//For ADD_NODE_INFO 
#define BURST_NUM_OF_DEV_INDEX			(BURST_CMD_ID_INDEX + 1)				
#define BRUST_SINGE_DEV_INFO_LEN		(BURST_NUM_OF_DEV_INDEX + 1)
#define BRUST_INFO_START_INDEX			(BRUST_SINGE_DEV_INFO_LEN + 1)

//For REMOTE_RESPONSE 
#define BURST_NODE_SR_NO  				(BURST_CMD_ID_INDEX + 2)				

//For BLE Message --------------------------------------------
#define BURST_BLE_TAR_SR_NO				(BURST_CMD_ID_INDEX + 1)
#define BURST_BLE_SRC_SR_NO				(BURST_BLE_TAR_SR_NO + 4)
#define BURST_BLE_PAYLOAD				(BURST_BLE_SRC_SR_NO + 4)

//For Remote Message --------------------------------------------
#define BURST_MSG_LEN_MSB 				(BURST_CMD_ID_INDEX + 1)	
#define BURST_MSG_LEN_LSB 				(BURST_MSG_LEN_MSB + 1)
#define BURST_DEVICE_TECH 				(BURST_MSG_LEN_LSB + 1)
#define BURST_DEVICE_TYPE 				(BURST_DEVICE_TECH + 1)
#define BURST_TAR_NODE_FULL_SR_NO		(BURST_DEVICE_TYPE + 1)
#define BURST_TAR_NODE_ANT_SR_NO		(BURST_TAR_NODE_FULL_SR_NO + 16)
#define BURST_MSG_TYPE					(BURST_TAR_NODE_ANT_SR_NO + 4)
#define BURST_MSG_PAYLOAD 				(BURST_MSG_TYPE + 1)

//For Operation Request -----------------------------------------
#define BURST_OPERATION_TYPE			(BURST_MSG_TYPE + 1)
#define BURST_OP_EXPIRE_TIME			(BURST_OPERATION_TYPE + 1)	//In Minutes
#define BURST_OPERATION_TIME			(BURST_OP_EXPIRE_TIME + 1)
#define BURST_OPERATION_KEY_IV			(BURST_OPERATION_TIME + 4)
#define BURST_OPERATION_KEY_DATA		(BURST_OPERATION_KEY_IV + 16)

//For Operation Response
#define BURST_REQUEST_STATUS			(BURST_MSG_TYPE + 1)

//For Status Response -------------------------------------------
#define BURST_DEVICE_STATUS				(BURST_REQUEST_STATUS + 1)

//For Add/Remove User Access Device Request ----------------------------
#define BURST_USER_DEVICE_ID			(BURST_MSG_TYPE + 1)

//For Asynchronous Event -------------------------------------------
#define BURST_ASYNC_EVENT_TYPE			(BURST_MSG_TYPE + 1)
#define BURST_ASYNC_EVENT_ENCRY_TYPE	(BURST_ASYNC_EVENT_TYPE + 1)
#define BURST_ASYNC_EVENT_DATA			(BURST_ASYNC_EVENT_ENCRY_TYPE + 1)

//For Mobile Message 
#define BURST_SRC_NODE_SR_NO			(BURST_MSG_TYPE + 1)
//#define BURST_MSG_PAYLOAD 				(BURST_SRC_NODE_SR_NO + 4)

//**********************************************************************************
//	Message sending status
//**********************************************************************************
#define NO_MESSAGE						0x00
#define MESSAGE_INITIATED				0x01
#define MESSAGE_IN_PROGRESS				0x02

//**********************************************************************************
//	Flash Operation status
//**********************************************************************************
#define NO_ANT_FLASH_WRITE_OPERATION	0x00
#define ANT_FLASH_WRITE_INITIAT			0x01
#define ANT_FLASH_WRITE_IN_PROGRESS		0x02

//**********************************************************************************
//	Flash Write Reason Code
//**********************************************************************************
#define NOTHING_TO_ADD					0x00
#define DEFAULT_PARA_WRITE				0x01
#define SHARED_ADDR_ADD					0x02
#define DIRECT_ANT_NODE_INFO_ADD		0x03
#define INDIRECT_ANT_NODE_INFO_ADD		0x04
#define ANT_NODE_DEL_WRITE				0x05
#define ANT_NODE_REG_ADD				0x06
#define DEVICE_NO_ADD					0x07
#define CLEAR_DYNA_PARA					0x08

//**********************************************************************************
//	Other Macro
//**********************************************************************************
typedef enum
{
	E_DEV_UNKNOWN = 0,
	E_DEV_ONLY_REGISTERED,
	E_DEV_ACTIVE,
	E_DEV_INACTIVE,
	E_DEV_NOT_IN_DB,
	E_DEV_IN_DB,
	
}en_DevCurrentStatus;

#define NODE_ALREADY_REGISTERED     	0
#define NODE_NOT_REGISTERED         	1

#define NEAR_DEVICE     				0
#define FAR_DEVICE			         	1

#define BROADCAST_MSG     				0
#define ACK_MSG			         		1

//**********************************************************************************
//	Application macros.
//**********************************************************************************
#define ANT_MSG_IDX_ID                  1                   ///< ANT message id index
#define ANT_LENGH_ID                    0                   ///< ANT message lengh index
#define ANT_EVENT_MSG_BUFFER_MIN_SIZE   32                  ///< Minimum size of ANT event message buffer

#define PAYLOAD_BUFFER_SIZE      		8                   ///< Size of the broadcast data buffer
#define BURST_BUFFER_SIZE	      		100                 ///< Size of the broadcast data buffer

#define FALSE							0
#define TRUE							1

#define NO								0
#define YES								1

#define NO_OF_CHANNEL 					2
#define CH_0 							0
#define CH_1 							1

#define SR_NO_LENGTH					4

#define SELF_CONNECTION_TIMEOUT			15//60

#define MAX_POLLING_COUNT_VALUE			40

#define MAX_NETWORK_LAYER	  					2

#define ANT_MAX_NODE_IN_IMMIDIATE_NEXT_LAYER	4
#define MAX_NODES_IN_NEXT_LAYER					ANT_MAX_NODE_IN_IMMIDIATE_NEXT_LAYER

//***************************************************************************************
//								variables Definations
//***************************************************************************************

/* Structure Declarations */

/* Channel ID parameter */
typedef struct									
{
	uint16_t mu16_DevNum;
	uint8_t mu8_DevType;
	uint8_t mu8_DevTransType;

}st_ChannelIDPara_t;

/* Device Serial Number and shared Address on network */
typedef struct						
{
	uint8_t mu8ar_DeviceSrNo[4];    		//Serial # of Device
	uint8_t mu8ar_ParentSrNo[4];    		//Serial # of Device's parent
	uint8_t mu8_DeviceSharedAddress;    	//Shared address of Device for Point to point communication.
	uint8_t mu8_ParentSharedAddress;		//Shared address of Device's Parent. 0x00: if Parent is Gateway
	uint8_t mu8_ParentLayerNumber;  		//Layer # of Device's parent
	uint8_t mu8_RegisterStatus;				//Registration Stat to GW
	
}st_AntNodeFlashParams_t;

/* Device Channel ID Parameter on network */
typedef struct 
{
	uint8_t mu8_FlashStatus;			//0xFF in this byte indicate flash is empty
	uint8_t mu8_NodePoweredOption;	
	uint8_t Reserve1;					
	uint8_t Reserve2;	
	st_ChannelIDPara_t mst_ChannelIDPara;
	st_AntNodeFlashParams_t mst_NodeInfo;
	
}st_AntFlashHeader_t;


typedef struct 
{
	st_AntFlashHeader_t         mst_AntFlashHeader;
	st_AntNodeFlashParams_t     mst_AntNodeFlashParams[MAX_NODES_IN_NEXT_LAYER];
	
}st_AntFlashParmas_t;	

typedef struct
{
    uint8_t                     flash_page_num_permenent_para;            			  	  /**< Flash page number to use for storing Bonding Information. */
    uint8_t                     flash_page_num_dynamic_para;        				  	  /**< Flash page number to use for storing System Attributes. */
    ble_srv_error_handler_t     error_handler;                  	   			  		  /**< Function to be called in case of an error. */
} app_flashmngr_init_t;

typedef volatile struct 
{
	uint8_t mu8_AntFwdMsgToMaster : 1; 
	uint8_t mu8_AntSendAsyncMsgToMaster : 1; 
	uint8_t mu8_AntSendResToMasterForRemoteReq : 1; 
	uint8_t mu8_AntSendNodeInfoToMaster : 1; 
	uint8_t mu8_AntSendMobileMsgToMaster_ch0 : 1; 
	uint8_t mu8_AntSendMobileMsgToMaster_ch1 : 1; 
	uint8_t mu8_AntStartNetwork : 1; 
	uint8_t mu8_AntChannelOpenState_ch0 : 1;
	uint8_t mu8_AntChannelOpenState_ch1 : 1;
	uint8_t mu8_AntchangeFreq : 1; 
	uint8_t mu8_AntDataEraseFlag : 1;
	uint8_t mu8_AntNWIDReceived : 1;
	uint8_t mu8_AntChannelCloseByANT : 1;
	uint8_t mu8_AntResetChip : 1;
	uint8_t mu8_AntGetEpochTime : 1;
	uint8_t mu8_EpochTimeValidFlag : 1;
	
}st_Antbitfield_t;

extern uint8_t gu8ar_AntBurstRxBuffer[NO_OF_CHANNEL][BURST_BUFFER_SIZE];  					///< Primary data for Burst receive buffer
extern uint8_t gu8ar_AntBurstTxBuffer[NO_OF_CHANNEL][BURST_BUFFER_SIZE];  					///< Primary data for Burst transmit buffer
extern uint8_t gu8ar_AntTempBuffer1[50];  							///< Common Use Buffer
extern uint8_t gu8ar_AntCHTxbuffer[NO_OF_CHANNEL][PAYLOAD_BUFFER_SIZE];
extern st_AntFlashParmas_t gst_AntFlashParams;
extern uint8_t *gu8p_AntConfigParamAddr;
extern uint8_t *gu8p_BleConfigParamAddr;
extern uint8_t gu8_AntFlashWriteReason;
extern uint8_t gu8_AntNoOfUnRegisterNode;
extern uint8_t gu8_AntBurstMsgProgressState[NO_OF_CHANNEL];
extern uint8_t gu8_AntGetEpochTimeMsgState;
extern uint8_t gu8_AntTxBurstMsgSize[NO_OF_CHANNEL];
extern uint8_t gu8_AddrAssignTimeout[NO_OF_CHANNEL];
extern uint8_t gu8_AntRegState;
extern uint8_t gu8_AntAddrAssignState;
extern uint8_t gu8_AdditionOfIndirectNodes;
extern uint8_t gu8_AntSendAckToMasterForRemote[NO_OF_CHANNEL];
extern uint16_t gu16_AntNextLayerNodesPollStatus;
extern uint8_t gu8ar_NodePollingCount[ANT_MAX_NODE_IN_IMMIDIATE_NEXT_LAYER];	
extern uint8_t gu8_NextSharedAddress;
extern uint8_t gu8_AddedSlaveNodeIndex;
extern uint8_t gu8_AntSelfConnectionTimeout;
extern uint8_t gu8_PollingSharedAddr;
extern uint8_t gu8_Ch0_CloseFlag;
extern uint8_t gu8_AntDeletNextLayerNode[2];
extern uint32_t gu8_EpochTime;
extern uint8_t gu8_AntReqSharedAddrAtCh1;
extern uint8_t gu8_AntReqSrNoAtCh1[4];
extern uint8_t	gu8_AntReqTypeAtCh1;
extern uint8_t gu8_AntBurstTryCntr[NO_OF_CHANNEL];

//extern uint8_t gu8_AntSendResToMasterTryCntr;
//extern uint8_t gu8_AntFwdMsgToMasterTryCntr;
//extern uint8_t gu8_AntSendNodeInfoToMasterTryCntr;
//extern uint8_t gu8_AntSendMobileMsgToMasterCntr_ch0;
//extern uint8_t gu8_AntSendMobileMsgToMasterCntr_ch1;
//extern uint8_t gu8_AntSendMsgToSlaveTryCntr;

extern uint8_t gu8_AntStatusPollingRcv;
extern uint8_t gu8_AntStatusUpdateCount;
extern uint8_t *gu8p_AntBleMsgptr;
extern uint8_t gu8p_AntBleMsglen;
extern uint8_t gu8_RemoteOpStatus;
extern uint8_t gu8_RemoteOpRequest;
extern uint8_t gu8_AntFlashWriteState;

extern uint8_t gu8_AntLockCurrentStatus,gu8_AntLockLastStatus;

extern volatile uint16_t gu8_AntBurstRcvCount,gu8_AntBurstRetryCount,gu8_AntBurstFailCount,gu8_AntBurstOKCount;

extern unsigned char write_cmd[WRITE_BUFFER_SIZE];

extern st_Antbitfield_t  b;
//***************************************************************************************
//								Function Declaration
//***************************************************************************************

void on_ant_evt(ant_evt_t * p_ant_evt);
//void on_ant_evt_channel_closed(void);
extern void antP2_ClearDynaConfiguration(void);
void antPx_HandleEventTx(uint8_t *);
void antPx_HandleEventRx(uint8_t *);
void antPx_TaskOnRxAtCH0(uint8_t * p_event_message_buffer);  
void antPx_ProcessBroadCastMessageAtCH0(uint8_t *);
void antPx_ProcessAckMessageAtCH0(uint8_t *);
void antPx_ProcessBurstMessageAtCH0(uint8_t *);
void antPx_TaskOnRxAtCH1(uint8_t * p_event_message_buffer);
void antPx_ProcessBroadCastMessageAtCH1(uint8_t *);
void antPx_ProcessAckMessageAtCH1(uint8_t *);
void antPx_ProcessBurstMessageAtCH1(uint8_t *);
void antPx_SetDataOnCH1(uint8_t);
void antPx_SendAddNodeRequest(void);
void antPx_SendResToMasterForRemoteReq(uint8_t);
void antPx_FwdMsgToMaster(void);
void antPx_SendAsyncMsgToMaster(void);
void antPx_SendBleMsgOnAnt(uint8_t *,uint8_t,uint8_t *);
void antI2_UpdatePollingStatus(void);
void antI4_GetNextSharedAddress(void);
void antI3_DeleteNode(uint8_t lu8_NodeIndex);
uint8_t antP1_SearchDevice(uint8_t *lu8p_DevSharedAdd, uint8_t *lu8p_DestDeviceSrNo);
uint8_t antI4_GetNodeParamsWrIndx(void);
uint8_t antI3_SearchDuplicateNode(st_AntNodeFlashParams_t *lstp_NodeInfo ,uint8_t *lu8p_NodeIndex);
void antPx_DecryptData(uint8_t *aes_key,uint8_t *iv,uint8_t *data_buffer,uint8_t length);

#endif
