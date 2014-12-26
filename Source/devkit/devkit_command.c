#include <string.h>
#include <stdio.h>
#include "nrf_soc.h"
#include "ant_interface.h"
#include "simple_uart.h"
#include "ble_srv_common.h"
#include "AntEventHandler.h"
#include "devkit\devkit_variable.h"
#include "devkit\devkit_command.h"
#include "devkit\devkit_constant.h"
#include "main.h"
#include <nrf.h>
#include "aes/aes-cbc_enc_dec.h"  
#include "aes/aes.h"
#include "ble_sr.h"
#include "boards/ras_1_6.h"
#include "led.h"
#include "temperature_NTC.h"
#include "co2_sensor.h"
#include "Accelerometer.h"
#include "battery.h"
#include "simple_uart.h"
#include "pwm.h"

st_Blebitfield_t BleBits;
extern unsigned char write_cmd[WRITE_BUFFER_SIZE];
extern unsigned char read_cmd[READ_BUFFER_SIZE];

uint8_t	gu8_TargetAntSerialNumber[ANT_SERIAL_NUMBER_BYTES];
uint8_t	gu8_DeviceType;
uint8_t gu8_OperationType;

extern ble_sr_t		m_sr;                                    	 	 /**< Structure used to identify the SecuRemote service. */

uint8_t ble_app_sl_request = FALSE;						/**indicates whether operation is pending or not. */
uint8_t SessionIDReceiveOKFlg = FALSE;
uint8_t UserAuthenticatedFlg =FALSE;
uint8_t gu8_RangeInOrOut;
uint8_t gu8_AutoManualConnStatus;

volatile uint8_t gu8_AntMsgReceivedOnBle;
volatile uint8_t gu8_BleMsgReceivedOnAnt;

uint8_t gu8_CommandIDReqVerificationFlg=FALSE;
uint8_t gu8_OperationReqVerificationFlg = FALSE;
uint8_t gu8_UserAlreadyPaired=FALSE;
uint8_t gu8_UserAlreadyAdded=FALSE;
uint8_t op_status = OP_STATUS_SUCCESS;
/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/
extern uint8_t ant_request; 

extern uint8_t UartDataRxFlg;

extern void adc_NTC_init(void);
extern void adc_CO2_init(void);
extern void adc_bat_volt_init(void);

extern uint32_t light_intensity_value;
extern uint32_t update_light_intensity_value;
extern bool co2_last_status;
extern bool co2_notify_on_connect_flg;
extern uint32_t STATUS_SW1;

extern uint8_t TWI_RxData[30];													//TWI_RxData contain values received by Accelerometer
extern volatile bool twi_operation_complete, twi_ack_received, twi_blocking_enabled;

bool water_sensor_flg;
bool light_status_flg;
bool temperature_notify_flag;
bool accelerometer_update_flg;
bool co2_notify_flag;
bool humidity_temperature_update_flg;
bool bat_voltage_update_flg;
bool motion_update_flg;
bool motion_detect_flg;
bool hw_info_update_flg;
bool solenoid_status;
bool relay_status_flg;
bool thermostat_update_flag;
bool water_sensor_update_flg;
bool Hardware_config_flg;
bool one_min_timer_flg;

bool heating_relay_status_on_flag;
bool cooling_relay_status_on_flag;
bool msg_rec_on_ble_for_UART_com_flg;

uint8_t 		operation_type;
uint8_t 		device_type;

uint8_t 		dev_kit_sr_no_set_flg;

uint8_t 		sw_previous_status;
uint8_t			dev_kit_op_req[OP_REQ_BUFF_LEN];
uint8_t 		dev_kit_op_status[OP_STATUS_BUFF_LEN];
uint8_t			dev_kit_UART_msg[UART_BUFF_LEN];
uint8_t			DevKit_SrNo[DEV_KIT_SR_NO_BYTES];
uint8_t		    hardware_type;
uint8_t 		humidity_sensor_presence;
uint8_t 		tx_power = 4;
uint8_t			relay_op_online_mode;
uint16_t		current_time=0;																//save the current time for thermostat category
uint16_t		heating_action_time=0;												//heating action time in thermostat category
uint16_t		cooling_action_time=0;												//Cooling action time in thermostat category
int8_t			heating_set_point=0;													//Heating set point in thermostat category
int8_t			cooling_set_point=0;													//Cooling set point in thermostat category

bool repeted_start_flg;

float  temperature;
int8_t   	thermostat_temp;
uint16_t 	updated_temperature;
float humidity;
float battery_voltage;



uint8_t 	dbgStr[50];	
/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/

//********************************************************
//Clear all the flags and timer to zero.
//When new commond for any category will receive.
//********************************************************
void reset_all_the_timer_and_flg(void)
{
	temperature_notify_flag=false;						//To stop the notification of temperature
	accelerometer_update_flg=false;						//To stop the notification of the accelerometer reading
	co2_notify_flag=false;
	humidity_temperature_update_flg=false;
	bat_voltage_update_flg = false;
	motion_update_flg = false;
	motion_detect_flg = false;

	if(device_type!=THERMOSTAT)
	{
		sl_app_timer[TEMP_SCAN_TIMER]=0;
	}
	
	if(device_type!=SOLENOID_DEV)
	{
		nrf_gpio_pin_clear(SOLENOID_CTR_PIN_NUMBER);
		green_led_off();
	}
	
	sl_app_timer[ACCE_UPDATE_TIMER]=0;
	sl_app_timer[CO2_SENSOR_UPDATE_TIMER]=0;
	sl_app_timer[HUMIDITY_TEMP_TIMER] = 0;
	sl_app_timer[BAT_VOLT_UPDATE_TIMER] = 0;
	sl_app_timer[MOTION_UPDATE_TIMER] = 0;
}
//**************************************************************//
// Generate Session ID
//**************************************************************//
void bleP2_GenerateSessionID(void)
{
	uint8_t lu8_BytesAvailable,lu8_i;

	//Get number of random bytes available to the application.
	sd_rand_application_bytes_available_get(&lu8_BytesAvailable);
	
	for(lu8_i=0;lu8_i<SESSION_ID_BYTES;lu8_i++)
	{
		//Get random bytes from the application pool.
		sd_rand_application_vector_get(SessionID,lu8_BytesAvailable);
//		SessionID[lu8_i] = lu8_i;
	}
	
	#ifdef SESSION_ID_DBG
		DEBUG("\r\nAuto Generated Session ID = ");
		gen_PrintHexStr(SessionID,SESSION_ID_BYTES);
		DEBUG("\r\n");
	#endif
	
	//Encrypt the session_ID by AES KEY 1
	blePx_EncryptData(gst_ConfigPermanentParams.mu8ar_AesKey1,SessionID,SESSION_ID_BYTES,0);
	
	#ifdef SESSION_ID_DBG
		DEBUG("\r\nAfter encryption data = ");
		gen_PrintHexStr(read_cmd,SESSION_ID_BYTES);
		DEBUG("\r\n");	
	#endif
	
	// Shuffel the encrypted data which is in read_cmd buffer
	bleP3_ShuffelTheDataBuffer(read_cmd,SESSION_ID_BYTES);
	
	#ifdef SESSION_ID_DBG
		DEBUG("\r\nShuffled : data = ");
		gen_PrintHexStr(read_cmd,SESSION_ID_BYTES);
		DEBUG("\r\n");
	#endif
	
	for(lu8_i=SESSION_ID_BYTES+1; lu8_i>0; lu8_i--)
	{
		read_cmd[lu8_i] = read_cmd[lu8_i-2];
	} 
	read_cmd[COMMAND_ID_INDEX] = READ_SESSION_ID_REQ;
	read_cmd[COMMAND_ID_LENGTH_INDEX] = SESSION_ID_BYTES;
}
//**************************************************************//
// Shuffel the data buffer
//**************************************************************//
void bleP3_ShuffelTheDataBuffer(uint8_t *SuffelBuffer,uint8_t DataLength)
{
	//Shuffel the data buffer
	uint8_t lu8_SwapCount=0,lu8_i=0,lu8_j,lu8_temp;
	lu8_SwapCount = (uint8_t)(DataLength/2);
	
	for(lu8_j=0; lu8_j<lu8_SwapCount; lu8_j++)
	{
		lu8_temp 				= SuffelBuffer[lu8_i];
		SuffelBuffer[lu8_i]		= SuffelBuffer[lu8_i+1];
		SuffelBuffer[lu8_i+1] 	= lu8_temp;
		lu8_i+=2;
	}
}

//**************************************************************//
//Deshuffel the data buffer
//**************************************************************//
void bleP2_DeshuffelTheDataBuffer(uint8_t *SuffelBuffer,uint8_t DataLength)
{
	//Shuffel the data buffer
	uint8_t lu8_SwapCount=0,lu8_i=0,lu8_j,lu8_temp;
	lu8_SwapCount = (uint8_t)(DataLength/2);
	
	for(lu8_j=0; lu8_j<lu8_SwapCount; lu8_j++)
	{
		lu8_temp 				= SuffelBuffer[lu8_i];
		SuffelBuffer[lu8_i]		= SuffelBuffer[lu8_i+1];
		SuffelBuffer[lu8_i+1] 	= lu8_temp;
		lu8_i+=2;
	}
}

//**************************************************************//
// Encrypt the data buffer
//**************************************************************//
void blePx_EncryptData(uint8_t *aes_key, uint8_t *data_buffer, uint8_t length, uint8_t index)
{
	uint8_t lu8_length=0,lu8_i;	
	
	//Copy Data to read buffer for encryption
	memcpy(&read_cmd[index], &data_buffer[index], length);
	
	if(length > 16)
	{
		lu8_length = length % 16;
		for(lu8_i=0; lu8_i<lu8_length; lu8_i++)
		{
			read_cmd[index + length++] = 0;
		}
		length = length + (16 - lu8_length);
	}
	else
	{
		lu8_length = 16 - length;
		for(lu8_i=0; lu8_i<lu8_length; lu8_i++)
		{
			read_cmd[index + length++] = 0;
		}
		length = 16;
	}
	aes_128_cbc_encrypt(aes_key, iv, &read_cmd[index], length);
}

//**************************************************************//
// Decrypt the data buffer
//**************************************************************//
void blePx_DecryptData(uint8_t *aes_key,uint8_t *data_buffer,uint8_t length)
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

//**************************************************************//
// BLE characteristic's events
//**************************************************************//
void paramUpdate(uint32_t paramUUID, uint8_t *data, uint8_t len)
{
	
	switch(paramUUID)
	{
		case BLE_UUID_DEV_KIT_OP_REQ:
			#ifdef DEBUGLOG_ENABLE
				DEBUG("BLE_UUID_DEV_KIT_OP_REQ\r\n");
			#endif
			memcpy(write_cmd, data, len);
			ble_app_sl_request=TRUE;
			//ble_sr_device_read_buffer_update(&m_sr, &data[0]);
			break;
			
		case BLE_UUID_READ_SL_DATA:
			
			break;
		
		case BLE_UUID_DEV_KIT_OP_STATUS:
			
			hw_info_update_flg =true;
			break;
		case BLE_UUID_OPERATION_STATUS:	

			break;
		
//		case BLE_UUID_LOCK_STATUS:

//			break;
		
		default:
			DEBUG("NOT Valid UUID\r\n");
			break;
	}
}

/*****************************************************************************
* Print message(command) received from phone app
*****************************************************************************/
void bleP1_PrintReceivedData(uint8_t *DataBuffer, uint8_t lu8_i)
{
//	DEBUG("\r\nRAW DAta = ");
//	gen_PrintHexStr(&write_cmd[COMMAND_ID_INDEX],100);
	if((write_cmd[COMMAND_ID_INDEX]!=READ_SESSION_ID_REQ) && (write_cmd[COMMAND_ID_INDEX]!=WRITE_SESSION_ID_REQ))
	{
		#ifdef OPERATION_REQ_DBG
			DEBUG("\r\nCommand ID = ");
			gen_PrintHexStr(&write_cmd[COMMAND_ID_INDEX - lu8_i],1);

			DEBUG("\r\nLength = ");
			gen_PrintHexStr(&write_cmd[OPERATION_REQ_LENGTH_INDEX - lu8_i],1);

			DEBUG("\r\nSerial Number = ");
			gen_PrintHexStr(&write_cmd[USER_SR_NO_INDEX - lu8_i],SERIAL_NUM_BYTES);

			DEBUG("\r\nSecurity tocken = ");
			gen_PrintHexStr(&write_cmd[USER_SECURITY_TOCKEN_INDEX - lu8_i],SECURITY_TOCKEN_BYTES);

			DEBUG("\r\nUser device ID = ");
			gen_PrintHexStr(&write_cmd[USER_DEVICE_ID_INDEX - lu8_i],USER_DEVICE_ID_BYTES);

			DEBUG("\r\nTime (MM:DD:YY:HH:MM) = ");
			gen_PrintHexStr(&write_cmd[USER_TIME_INDEX - lu8_i],TIME_STAMP_BYTES);
		#endif
	}
}

int8_t sl_setTxPower(uint8_t value)
{
  int8_t range;
 
  switch (value)
  {
    case RANGE_PROXIMITY:
      range = RADIO_TXPOWER_TXPOWER_Neg30dBm;
      break;

    case RANGE_LOW:
      range = RADIO_TXPOWER_TXPOWER_Neg20dBm;
      break;
    
    case RANGE_MEDIUM:
      range = RADIO_TXPOWER_TXPOWER_0dBm;
      break;

    case RANGE_HIGH:
      range = RADIO_TXPOWER_TXPOWER_Pos4dBm;
      break;

    default:
      range = RADIO_TXPOWER_TXPOWER_Pos4dBm;
      break;
  }
  sd_ble_gap_tx_power_set(range); 
  
  return range; 
}

/*****************************************************************************
* Process message(command) from phone app
*****************************************************************************/
void bleP0_BleAppDevkitCommandExecution(void)
{
	uint8_t lu8_index=0,lu8_Length=0;
	
//	#ifdef RECEIVED_REQ_DBG
//		DEBUG("\r\nWRITE DATA = ");
//		gen_PrintHexStr(&write_cmd[0],WRITE_BUFFER_SIZE);		
//	#endif
	

	if((gst_BleConfigDynamicParams.mu8_Securitylevel >= SECURITY_LEVEL_2)  && (write_cmd[COMMAND_ID_INDEX]!=WRITE_SESSION_ID_REQ))
	{
		lu8_Length = write_cmd[COMMAND_ID_LENGTH_INDEX];
		if((gst_BleConfigDynamicParams.mu8_Securitylevel >= SECURITY_LEVEL_2) && (lu8_Length>0))
		{
			blePx_DecryptData(gst_ConfigPermanentParams.mu8ar_AesKey3,&write_cmd[COMMAND_ID_PAYLOAD_INDEX],lu8_Length);
		}
		
		#ifdef RECEIVED_REQ_DBG
			DEBUG("\r\nDecrypted DATA = ");		
			gen_PrintHexStr(&write_cmd[0],WRITE_BUFFER_SIZE);
		#endif
		
		lu8_index = 0;
		bleP1_PrintReceivedData(&write_cmd[0], lu8_index);
	}		
	bleP1_PrintReceivedData(&write_cmd[0], lu8_index);
	bleP1_CommandExecuter(&write_cmd[0]);
//	if(BleBits.mu8_BleFlashUpdate == BLE_FLASH_WRITE_INITIAT)
//	{
//		FlashWrite(&mp_flash_BLE_DynPara,(uint8_t *)&gst_BleConfigDynamicParams,sizeof(st_BleFlashDynamicParams_t));
//		BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_IN_PROGRESS;
//	}
}
/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/
//*******************************************************
//Decode reveived msg by phone.
//By which we have to do operation according to command
//*******************************************************
void decode_op_req_msg(uint8_t *data)
{	
	ant_request = NO_ANT_MSG;
	device_type = data[DEVKIT_DEVICE_TYPE];						//index 2
	operation_type = data[DEVKIT_OP_TYPE];						//index 3
	reset_all_the_timer_and_flg();
	switch(device_type)
	{
		case TEMP_SENSOR:
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					adc_NTC_init();													//ADC used for Temperature measurement(NTC)
					temperature_notify_flag=true;
					sl_app_timer[TEMP_SCAN_TIMER]=TEMP_SCAN_TIME;
				}
				break;
				
		case SOLENOID_DEV:
				if((data[DEVKIT_OP_TYPE] == SOLENOID_OP) && (!solenoid_status))
				{
					nrf_gpio_pin_set(SOLENOID_CTR_PIN_NUMBER);
					green_led_on();
					sl_app_timer[SOLENOID_OFF_TIMER]=SOLENOID_OFF_TIME;
					solenoid_status = true;
					
					dev_kit_op_status[0]=SOLENOID_DEV;
					dev_kit_op_status[1]=solenoid_status;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
				}				
				break;
				
		case MOTOR:

		      //gen_PrintHex(data[DEVKIT_OP_TYPE]);
				if(data[DEVKIT_OP_TYPE] == MOTOR_FW)
				{
					motor_forward();
					green_led_on();
					sl_app_timer[MOTOR_OFF_TIMER]=MOTOR_OFF_TIME;
				}	
				else if(data[DEVKIT_OP_TYPE] == MOTOR_RW)
				{				
					motor_reverse();
					green_led_on();
					sl_app_timer[MOTOR_OFF_TIMER]=MOTOR_OFF_TIME;
				}	
				break;
				
		case LIGHT_CONTROL:
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]=LIGHT_CONTROL;
					dev_kit_op_status[1]=light_status_flg;
					dev_kit_op_status[2]=light_intensity_value;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
					if(light_status_flg)
					{
						if(light_intensity_value != 0)
						{
							set_intensity(light_intensity_value);
							//PWM_TIMER->TASKS_START = 1;
						}
					}
				}
				else if(data[DEVKIT_OP_TYPE] == LIGHT_ON)
				{
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]=LIGHT_CONTROL;
					dev_kit_op_status[1]=LIGHT_ON_STAT;
					dev_kit_op_status[2]=light_intensity_value;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
					light_status_flg=true;
					if(light_intensity_value != 0)
					{
						set_intensity(light_intensity_value);
					}
				}	
				else if(data[DEVKIT_OP_TYPE] == LIGHT_OFF)
				{				
					light_status_flg=false;
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]=LIGHT_CONTROL;
					dev_kit_op_status[1]=LIGHT_OFF_STAT;
					dev_kit_op_status[2]=light_intensity_value;					
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
					set_intensity(0);	
				}	
				else if(data[DEVKIT_OP_TYPE] == LIGHT_INTENSITY)
				{
					light_intensity_value = data[DEVKIT_OP_VALUE];
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]=LIGHT_CONTROL;
					dev_kit_op_status[1]=light_status_flg;
					dev_kit_op_status[2]=light_intensity_value;					
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
					if(light_status_flg)
					{
					 	set_intensity(light_intensity_value);	
					}
				}
				break; 	
				
		case RELAY:
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]=RELAY;
					dev_kit_op_status[1]=relay_status_flg;
					dev_kit_op_status[2]=relay_op_online_mode;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);					
				}
			
				else if(data[DEVKIT_OP_TYPE] == RELAY_MOMENTORY)
				{
					nrf_gpio_pin_set(RELAY_1_CTR_PIN_NUMBER);
					green_led_on();
					sl_app_timer[RELAY_OFF_TIMER]=RELAY_OFF_TIME;
					
					dev_kit_op_status[0]=RELAY;
					dev_kit_op_status[1]=0;
					dev_kit_op_status[2]=relay_op_online_mode;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
				}
				
				else if(data[DEVKIT_OP_TYPE] == RELAY_PUSH_RELEASE)
				{	
					if(!relay_status_flg)
					{
						nrf_gpio_pin_set(RELAY_2_CTR_PIN_NUMBER);
						green_led_on();
						relay_status_flg=true;
					}
					else
					{
						nrf_gpio_pin_clear(RELAY_2_CTR_PIN_NUMBER);
						green_led_off();
						relay_status_flg=false;
					}
					
					dev_kit_op_status[0]=RELAY;
					dev_kit_op_status[1]=relay_status_flg;
					dev_kit_op_status[2]=relay_op_online_mode;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
				}
				break;
		
		case ACCELEROMETER:
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					//Initialize_Accelerometer();
					sl_app_timer[ACCE_UPDATE_TIMER]=ACCE_UPDATE_TIME;
				}
				break;
				
		case CO2_SENSOR:	
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{					
					adc_CO2_init();																						//ADC used for CO2 measurement
					co2_notify_flag=true;
					co2_notify_on_connect_flg=true;
					co2_last_status=false;
					sl_app_timer[CO2_SENSOR_UPDATE_TIMER]=CO2_SENSOR_UPDATE_TIME;					
				}			
				break;
				
		case SW_STATUS:
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					if((nrf_gpio_pin_read(STATUS_SW1) == LOW))
					{
						sw_previous_status = LOW;
						memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
						dev_kit_op_status[0]=SW_STATUS;
						dev_kit_op_status[1]=SWITCH_PRESSED_STATE;					
						ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
					}
					else
					{
						sw_previous_status = HIGH;
						memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
						dev_kit_op_status[0]=SW_STATUS;
						dev_kit_op_status[1]=SWITCH_UNPRESSED_STATE;					
						ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
					}
				}
				break;
				
		case DFU_OTA_MODE:		
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
				  dev_kit_op_status[0]=DFU_OTA_MODE;
				  dev_kit_op_status[1]=0;					
				  ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
					
					memset(&data[0], 0, OP_REQ_BUFF_LEN);
					green_led_off();
					blue_led_off();
					green_led_on();
					blue_led_on();
					nrf_delay_us(100000);
					nrf_delay_us(100000);
					green_led_off();
					blue_led_off();
					sd_power_gpregret_set(1);								//resistor value compare in bootloader code 
					NVIC_SystemReset();											//RESET the device after write value in resistor
				}		
				break;
				
		case MOTION_SENSOR:
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					motion_update_flg = true;
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]=MOTION_SENSOR;
					dev_kit_op_status[1]=nrf_gpio_pin_read(MOTION_DETECT_PIN_NUMBER);					
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
					if(nrf_gpio_pin_read(MOTION_DETECT_PIN_NUMBER) == HIGH)
					{
						motion_detect_flg = true;
					}
					sl_app_timer[MOTION_UPDATE_TIMER]=MOTION_UPDATE_TIME;
				}
				break;
				
		case THERMOSTAT:
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					adc_NTC_init();
					temperature_notify_flag=true;
					if(current_time == 0)
					{
						heating_action_time=0;
						cooling_action_time=0;
						heating_set_point=0;
						cooling_set_point=0;
					}
				}
				else if(data[DEVKIT_DEVICE_TYPE] == THERMOSTAT)
				{	
					current_time=0;
					
					current_time				= (uint16_t) (data[CURRENT_TIME_POS] << 8);
					current_time        = (uint16_t) ((current_time) | (data[CURRENT_TIME_POS+1]));
					
					if(data[DEVKIT_OP_TYPE] == HEATING)
					{
						heating_set_point		= 0;
						heating_set_point   = (int8_t)   (data[HEATING_SET_POINT_POS]);
						
						heating_action_time	=	0;
						heating_action_time = (uint16_t) (data[HEATING_ACTION_TIME_POS] << 8);
						heating_action_time = (uint16_t) ((heating_action_time) | (data[HEATING_ACTION_TIME_POS+1]));
					}
					
					if(data[DEVKIT_OP_TYPE] == COOLING)
					{
						cooling_set_point		=	0;
						cooling_set_point   = (int8_t)   (data[COOLING_SET_POINT_POS]);
						
						cooling_action_time	=	0;
						cooling_action_time = (uint16_t) (data[COOLING_ACTION_TIME_POS] << 8);
						cooling_action_time = (uint16_t) ((cooling_action_time) | (data[COOLING_ACTION_TIME_POS+1]));
					}
					
					gst_BleConfigDynamicParams.heating_set_point_flash = heating_set_point;
					gst_BleConfigDynamicParams.cooling_set_point_flash = cooling_set_point;					
				}
				
				if(heating_action_time!=0)
				{
					if(heating_action_time > current_time)
					{
						sl_app_timer[ONE_MIN_TIMER] = ONE_MIN_TIME;												
						heating_action_time -= current_time;						
						thermostat_update_flag = true;
					}
					#ifdef RUN_DBG
					sprintf(dbgStr, "\r\nCurrent time = %i \r\n ", current_time);
					simple_uart_putstring(dbgStr);
					sprintf(dbgStr, "\r\nHeating set point = %i \r\n ", heating_set_point);
					simple_uart_putstring(dbgStr);
					sprintf(dbgStr, "\r\nHeating action time = %i \r\n ", heating_action_time);
					simple_uart_putstring(dbgStr);
					#endif
				}
				
				if(cooling_action_time!=0)
				{
					if(cooling_action_time > current_time)
					{
						sl_app_timer[ONE_MIN_TIMER] = ONE_MIN_TIME;												
						cooling_action_time -= current_time;
						thermostat_update_flag = true;
					}
					#ifdef RUN_DBG
					sprintf(dbgStr, "\r\nCurrent time = %i \r\n ", current_time);
					simple_uart_putstring(dbgStr);
					sprintf(dbgStr, "\r\nCooling set point = %i \r\n ", cooling_set_point);
					simple_uart_putstring(dbgStr);
					sprintf(dbgStr, "\r\nCooling action time = %i \r\n ", cooling_action_time);
					simple_uart_putstring(dbgStr);
					#endif
				}		
			  break;
		case HUMIDITY_SENSOR:
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					humidity_temperature_update_flg=true;
					sl_app_timer[HUMIDITY_TEMP_TIMER] = 5;
				}				
			  break;
				
		case WATER_SENSOR:
				water_sensor_update_flg=true;
				break;
		
		case BAT_VOLTAGE:
				adc_bat_volt_init();
				sl_app_timer[BAT_VOLT_UPDATE_TIMER] = BAT_VOLT_UPDATE_TIME;
				break;
		
		case HW_CONFIG:
				if(data[DEVKIT_OP_TYPE] == HW_CONFIG_SET)
				{	
					Hardware_config_flg = true;
						
					hardware_type = data[HARDWARE_TYPE_POS];
					humidity_sensor_presence = data[HUMIDITY_SENSOR_POS];
					tx_power = data[TX_POWER_POS];
			
					gst_BleConfigDynamicParams.hardware_type_flash = hardware_type;
					gst_BleConfigDynamicParams.humidity_sensor_presence_flash = humidity_sensor_presence;	
					gst_BleConfigDynamicParams.mu8_TxPower = tx_power;
					
					if(hardware_type==HARDWARE_NS_BLE)
					{
						simple_uart_putstring("\r\nHardware type NS_BLE\r\n");	
					}
					else if(hardware_type==HARDWARE_NS_548)
					{
						simple_uart_putstring("\r\nHardware type NS548\r\n");	
					}
					
					if(humidity_sensor_presence==HUMIDITY_CONNECTED)
					{
						simple_uart_putstring("humidity sensor is connected\r\n");	
					}
					else if(humidity_sensor_presence==HUMIDITY_NOT_CONNECTED)
					{
						simple_uart_putstring("humidity sensor is not connected\r\n");	
					}
					
					sl_setTxPower(gst_BleConfigDynamicParams.mu8_TxPower);
										
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]=HW_CONFIG;
					dev_kit_op_status[1]=hardware_type;
					dev_kit_op_status[2]=humidity_sensor_presence;
					dev_kit_op_status[3]=tx_power;					
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
				}
				break;
				
		case ONLINE_MODE_SET:																		//16
				if(data[DEVKIT_OP_TYPE] == ONLINE_MODE_RELAY)
				{
					relay_op_online_mode = data[DEVKIT_OP_VALUE];					
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]=ONLINE_MODE_SET;
					dev_kit_op_status[1]=relay_op_online_mode;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
				}	
				else if(data[DEVKIT_OP_TYPE] == SR_NO_SET)
				{
					dev_kit_sr_no_set_flg = TRUE;
					memcpy(&DevKit_SrNo[0],&data[DEVKIT_OP_VALUE],DEV_KIT_SR_NO_BYTES);
				}
				break;
				
		case DEV_KIT_SRNO_CFG:																		//17
				if(data[DEVKIT_OP_TYPE] == READ_STATUS)
				{
					if(dev_kit_sr_no_set_flg==1)
					{
						memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
						dev_kit_op_status[0]=DEV_KIT_SRNO_CFG;
						memcpy(&dev_kit_op_status[1],&DevKit_SrNo[0],DEV_KIT_SR_NO_BYTES);
						ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], DEV_KIT_SR_NO_BYTES);
					}
				}
				break;
	}
}
/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/

/*****************************************************************************
* Process All message type as per RAS_ICD Document
*****************************************************************************/
uint32_t bleP1_CommandExecuter(uint8_t *cmd_buffer)
{	
		uint8_t lu8_i,lu8_j;
		uint32_t err_code;
	switch (cmd_buffer[COMMAND_ID_INDEX])
	{
		case DFU_REQ:
			sd_power_gpregret_set(1);
			sl_app_timer[SYSTEM_RESET_TIMER]=SYSTEM_RESET_TIME;
			//NVIC_SystemReset();
			break;
		
		case READ_CONFIG_REQ:																	//1
			
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nREAD_CONFIG_REQ");
			#endif
		
			if(UserAuthenticatedFlg == TRUE)
			{
				memset(read_cmd,0,sizeof(read_cmd));
				read_cmd[COMMAND_ID_INDEX] = READ_CONFIG_REQ;
				read_cmd[COMMAND_ID_LENGTH_INDEX] = CONFIG_PARA_BYTES;
				gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[8] = gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord;
				memcpy(&read_cmd[CONFIG_PARA_INDEX], &gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[0], CONFIG_PARA_BYTES);
				if(gst_BleConfigDynamicParams.mu8_Securitylevel >= SECURITY_LEVEL_2)
				{
					blePx_EncryptData(gst_ConfigPermanentParams.mu8ar_AesKey3, read_cmd, CONFIG_PARA_BYTES, CONFIG_PARA_INDEX);
				}
				ble_sr_device_read_buffer_update(&m_sr, read_cmd);									//Update Read buffer
				ble_sr_operation_status_update(&m_sr,SUCCESS);				
			}			
			break;
		
		case WRITE_CONFIG_REQ:																	//2
			
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nWRITE_CONFIG_REQ");
			#endif
		
			if(UserAuthenticatedFlg == TRUE)
			{
				memcpy(&gst_BleConfigDynamicParams.mu8ar_ConfigurationParameter[0], &cmd_buffer[CONFIG_PARA_INDEX], CONFIG_PARA_BYTES);
				ble_sr_operation_status_update(&m_sr,SUCCESS);
				if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
				{
					BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
				}
			}
			break;
			
		case DEVKIT_OPERATION_REQUEST:                                                           //30
						
			#ifdef OPERATION_REQ_DBG
				DEBUG(" DEVKIT_OPERATION_REQUEST\r\n");
			#endif
			bleP2_CommandIDReqVerificationProcess(&cmd_buffer[0]);
		
				if(UserAuthenticatedFlg == TRUE)
				{
				ble_sr_operation_status_update(&m_sr,SUCCESS);
				DEBUG("\r\nDEVKIT DATA = ");
				gen_PrintHexStr(&cmd_buffer[DEVKIT_COMMAND_INDEX],10);	
				
				decode_op_req_msg(&cmd_buffer[DEVKIT_COMMAND_INDEX]);
				}
			#ifdef OPERATION_REQ_DBG
				DEBUG(" DEVKIT_OPERATION_REQUEST\r\n");
			#endif
		break;
			
		case PAIRING_REQ:																		//4
				
			#ifdef OPERATION_REQ_DBG			
				DEBUG(" PAIRING_REQ\r\n");
			#endif
			gu8_UserAlreadyAdded = FALSE;
			if((SessionIDReceiveOKFlg == TRUE) || (gst_BleConfigDynamicParams.mu8_Securitylevel == SECURITY_LEVEL_0))
			{	
				if(memcmp(&cmd_buffer[USER_SR_NO_INDEX],&gst_ConfigPermanentParams.mu8ar_SerialNumber[0],SERIAL_NUM_BYTES) == 0)
				{
					if(memcmp(&cmd_buffer[USER_SECURITY_TOCKEN_INDEX],&gst_ConfigPermanentParams.mu8ar_SecurityTocken[0],SECURITY_TOCKEN_BYTES) == 0)
					{
						if(gst_BleConfigDynamicParams.mu8_NumOfUser > 0)
						{
							for(lu8_i=0; lu8_i < gst_BleConfigDynamicParams.mu8_NumOfUser; lu8_i++)
							{
								if(memcmp(&cmd_buffer[USER_DEVICE_ID_INDEX],&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[0],USER_DEVICE_ID_BYTES) == 0)
								{
									gu8_UserAlreadyAdded = TRUE;
									if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
									{
										BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
									}
									break;
								}
							}
						}
						if(gu8_UserAlreadyAdded!=TRUE)
						{	
							memcpy(&gst_BleConfigDynamicParams.mst_UserRecord[gst_BleConfigDynamicParams.mu8_NumOfUser].mu8ar_UserDeviceId[0], &cmd_buffer[USER_DEVICE_ID_INDEX], USER_DEVICE_ID_BYTES);
							gst_BleConfigDynamicParams.mu8_NumOfUser++;
							if(gst_BleConfigDynamicParams.mu8_NumOfUser >= MAX_USER)
							{
								gst_BleConfigDynamicParams.mu8_NumOfUser = MAX_USER-1;
							}
							
							gst_BleConfigDynamicParams.mu8_SrDevStatus = ACTIVE;
							UserAuthenticatedFlg = TRUE;
							ble_sr_operation_status_update(&m_sr,OP_STATUS_SUCCESS);
							if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
							{
								BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
							}
						}
						else
						{
							#ifdef OPERATION_REQ_DBG			
								DEBUG(" User already paired\r\n");
							#endif
							UserAuthenticatedFlg = TRUE;						
							ble_sr_operation_status_update(&m_sr,OP_STATUS_SUCCESS);						
						}
					}
					else
					{
						ble_sr_operation_status_update(&m_sr,OP_STATUS_SECURE_TOKEN_MISMATCH);
					}
				}
				else
				{
					ble_sr_operation_status_update(&m_sr,OP_STATUS_SR_NUM_MISMATCH);
				}
			}
			break;
			
		case UPDATE_USER_DEVICE_ID_REQ:															//5
						
			#ifdef OPERATION_REQ_DBG
				DEBUG(" UPDATE_USER_DEVICE_ID_REQ\r\n");
			#endif
			
			if(UserAuthenticatedFlg == TRUE)
			{
				bleP2_CommandIDReqVerificationProcess(&cmd_buffer[0]);
				if(gu8_CommandIDReqVerificationFlg == TRUE)
				{
					for(lu8_i=0; lu8_i < gst_BleConfigDynamicParams.mu8_NumOfUser; lu8_i++)
					{
						if(memcmp(&cmd_buffer[REMOVE_USER_DEVICE_ID_INDEX],&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[0],USER_DEVICE_ID_BYTES) == 0)
						{
							#ifdef OPERATION_REQ_DBG
								DEBUG("\r\nRemoved User Device ID_");
								gen_PrintDec(lu8_i+1);
								DEBUG(" = ");
								gen_PrintHexStr(&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[0],USER_DEVICE_ID_BYTES);
								DEBUG("\r\n");								
							#endif
							
							for( ; lu8_i< gst_BleConfigDynamicParams.mu8_NumOfUser; lu8_i++)
							{
								for(lu8_j=0; lu8_j < USER_DEVICE_ID_BYTES; lu8_j++)
								{
									memcpy(&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[lu8_j],
											&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i+1].mu8ar_UserDeviceId[lu8_j],
												USER_DEVICE_ID_BYTES);
								}
							}
//								lu8_i++;
							memset(&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[0], 0,USER_DEVICE_ID_BYTES);
							
							if(gst_BleConfigDynamicParams.mu8_NumOfUser > 0)
							{
								gst_BleConfigDynamicParams.mu8_NumOfUser--;
							}
							else
							{
								gst_BleConfigDynamicParams.mu8_NumOfUser=0;
							}
							if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
							{
								BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
							}
							ble_sr_operation_status_update(&m_sr,OP_STATUS_SUCCESS);
						}
					}
					UserAuthenticatedFlg = FALSE;
				}					
			}
			break;
			
		case AUTHORIZE_CONN_REQ:																//6
			
			#ifdef OPERATION_REQ_DBG
				DEBUG(" AUTHORIZE_CONN_REQ\r\n");
			#endif
		//UserAuthenticatedFlg = TRUE;
	//	ble_sr_operation_status_update(&m_sr,SUCCESS);
			bleP3_UserAuthenticationProcess(cmd_buffer);
			break;
		

		
		case READ_AUDIT_TRAIL_COUNTER_REQ:														//4
			
				 #ifdef OPERATION_REQ_DBG
					DEBUG("\r\nREAD_AUDIT_TRAIL_COUNTER_REQ");
				 #endif
		
				 ble_sr_operation_status_update(&m_sr,cmd_buffer[AUDIT_TRIAL_COUNTER_INDEX]);
				 
				 break;
		
		case READ_AUDIT_TRAIL_REQ:																//5
				 
			#ifdef OPERATION_REQ_DBG
			DEBUG("\r\nREAD_AUDIT_TRAIL_REQ");
			#endif
			if(UserAuthenticatedFlg==TRUE)
			{
//				if(gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord > 0)
				{
					if(cmd_buffer[AUDIT_TRAIL_REQ_INDEX] == AUDIT_TRAIL_1_7_REQ)
					{
						read_cmd[COMMAND_ID_INDEX] = READ_AUDIT_TRAIL_REQ;
						read_cmd[AUDIT_TRAIL_REQ_INDEX] = AUDIT_TRAIL_1_7_REQ;
						read_cmd[AUDIT_TRAIL_COUNTER_INDEX] = gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord;
						read_cmd[AUDIT_TRAIL_LENGTH_INDEX] = ((gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord > 7) ? 7:gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord) * sizeof(st_AuditLogProfile_t);
						memcpy(&read_cmd[AUDIT_TRAIL_PAYLOAD_INDEX], &gst_BleConfigDynamicParams.mst_AuditLogRecord[0], read_cmd[AUDIT_TRAIL_LENGTH_INDEX]);
						if(gst_BleConfigDynamicParams.mu8_Securitylevel >= SECURITY_LEVEL_2)
						{
							blePx_EncryptData(gst_ConfigPermanentParams.mu8ar_AesKey3, read_cmd, read_cmd[AUDIT_TRAIL_LENGTH_INDEX], AUDIT_TRAIL_PAYLOAD_INDEX);
						}
					}
					else if(cmd_buffer[AUDIT_TRAIL_REQ_INDEX] == AUDIT_TRAIL_8_10_REQ)
					{
						read_cmd[COMMAND_ID_INDEX] = READ_AUDIT_TRAIL_REQ;
						read_cmd[AUDIT_TRAIL_REQ_INDEX] = AUDIT_TRAIL_8_10_REQ;
						read_cmd[AUDIT_TRAIL_COUNTER_INDEX] = gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord - 7;
						read_cmd[AUDIT_TRAIL_LENGTH_INDEX] = read_cmd[AUDIT_TRAIL_COUNTER_INDEX] * sizeof(st_AuditLogProfile_t);
						memcpy(&read_cmd[AUDIT_TRAIL_PAYLOAD_INDEX], &gst_BleConfigDynamicParams.mst_AuditLogRecord[7], read_cmd[AUDIT_TRAIL_LENGTH_INDEX]);
						if(gst_BleConfigDynamicParams.mu8_Securitylevel >= SECURITY_LEVEL_2)
						{
							blePx_EncryptData(gst_ConfigPermanentParams.mu8ar_AesKey3, read_cmd, read_cmd[AUDIT_TRAIL_LENGTH_INDEX], AUDIT_TRAIL_PAYLOAD_INDEX);
						}
					}
					
					ble_sr_device_read_buffer_update(&m_sr, read_cmd);								//Update Read buffer
					ble_sr_operation_status_update(&m_sr,SUCCESS);									//Notification Success			
				}	
			}	 			 
			break;
		
		case READ_SESSION_ID_REQ:																//6
			
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nREAD_SESSION_ID_REQ");
			#endif
		
			if(gst_BleConfigDynamicParams.mu8_Securitylevel >= SECURITY_LEVEL_1)
			{
				bleP2_GenerateSessionID();
				ble_sr_device_read_buffer_update(&m_sr, read_cmd);								//Update Read buffer
				ble_sr_operation_status_update(&m_sr,SUCCESS);									//Notification Success
			}
			else
			{
				ble_sr_operation_status_update(&m_sr,OP_STATUS_SESSION_ID_NOT_REQUIRED);			//Notification
			}			
			break;
		
		case WRITE_SESSION_ID_REQ:																//7
			
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nWRITE_SESSION_ID_REQ");
			#endif
			if(cmd_buffer[SESSION_ID_LENGTH_INDEX]==SESSION_ID_BYTES)
			{				
				memcpy(read_cmd, &cmd_buffer[SESSION_ID_INDEX], SESSION_ID_BYTES);
				bleP2_DeshuffelTheDataBuffer(read_cmd,SESSION_ID_BYTES);
				blePx_DecryptData(gst_ConfigPermanentParams.mu8ar_AesKey2,read_cmd,SESSION_ID_BYTES);
				if(memcmp(read_cmd,SessionID,SESSION_ID_BYTES) == 0)
				{	
					#ifdef OPERATION_REQ_DBG
						DEBUG("\r\nSession ID Received OK");
					#endif
					SessionIDReceiveOKFlg = TRUE;
					ble_sr_operation_status_update(&m_sr,SUCCESS);
				}
				else
				{
					#ifdef OPERATION_REQ_DBG
						DEBUG("\r\nWrong Session ID");
					#endif
					ble_sr_operation_status_update(&m_sr,OP_STATUS_SESSION_ID_MISMATCH);
				}				
			}
			else
			{
				ble_sr_operation_status_update(&m_sr,OP_STATUS_SESSION_ID_MISMATCH);
			}
			break;
		
		case WRITE_RANGE_IN_OR_OUT_REQ:															//8
			
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nWRITE_RANGE_IN_OR_OUT_REQ");
			#endif
		
			if(UserAuthenticatedFlg==TRUE)
			{
				bleP2_CommandIDReqVerificationProcess(&cmd_buffer[0]);
				if(gu8_CommandIDReqVerificationFlg == TRUE)
				{
					gu8_RangeInOrOut = cmd_buffer[RANGE_IN_OUT_INDEX];
					ble_sr_operation_status_update(&m_sr,SUCCESS);
					if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
					{
						BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
					}
					gu8_CommandIDReqVerificationFlg = FALSE;
				}			
			}				
			break;
		
		case REMOTE_OPERATION_REQ:																//9
			
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nREMOTE_OPERATION_REQ");
			#endif
		
			if((UserAuthenticatedFlg==TRUE) && (gu8_AntMsgReceivedOnBle==FALSE))
			{
				//Send data to ANT than give success reply
				memcpy(gu8_TargetAntSerialNumber,&cmd_buffer[TARGET_ANT_SR_NO_INDEX-1],ANT_SERIAL_NUMBER_BYTES);
				gu8_DeviceType = cmd_buffer[REMOTE_OP_DEVICE_TYPE_INDEX-1];
				gu8_OperationType = cmd_buffer[REMOTE_OP_OPERA_TYPE_INDEX-1];
				
				
				#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nTarget ANT SrNo=");
				gen_PrintHex(gu8_TargetAntSerialNumber[0]);
				gen_PrintHex(gu8_TargetAntSerialNumber[1]);
				gen_PrintHex(gu8_TargetAntSerialNumber[2]);
				gen_PrintHex(gu8_TargetAntSerialNumber[3]);
				
				DEBUG("\r\nDevice Type =");
				gen_PrintHex(gu8_DeviceType);
				
				DEBUG("\r\nOperation Type =");
				gen_PrintHex(gu8_OperationType);
				
				#endif
				
				ble_sr_operation_status_update(&m_sr,SUCCESS);
				gu8_CommandIDReqVerificationFlg = FALSE;
				gu8_AntMsgReceivedOnBle = TRUE;
			}			
			break;

		
		case CHANGE_SECURITY_LEVEL_REQ:															//11
			
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nSECURITY_LEVEL_REQ");
			#endif
		
			if(UserAuthenticatedFlg==TRUE)
			{
				bleP2_CommandIDReqVerificationProcess(&cmd_buffer[0]);
				if(gu8_CommandIDReqVerificationFlg == TRUE)
				{
					gst_BleConfigDynamicParams.mu8_Securitylevel = cmd_buffer[SECURITY_LEVEL_INDEX];
					ble_sr_operation_status_update(&m_sr,SUCCESS);
					if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
					{
						BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
					}
					gu8_CommandIDReqVerificationFlg = FALSE;
				}
			}			
			break;		
		
		case ADD_NETWORK_ID_REQ:																//15

			if(UserAuthenticatedFlg == TRUE)
			{				
				#ifdef OPERATION_REQ_DBG
					DEBUG(" ADD_NETWORK_ID_REQ\r\n");
				#endif
				bleP2_CommandIDReqVerificationProcess(&cmd_buffer[0]);
				if(gu8_CommandIDReqVerificationFlg == TRUE)
				{
					DEBUG("\r\n\r\nNW ID=");
					gen_PrintHex(cmd_buffer[NETWORK_ID_INDEX-1]);
					gen_PrintHex(cmd_buffer[NETWORK_ID_INDEX]);					
					gen_PrintHex(cmd_buffer[NETWORK_ID_INDEX+1]);
					DEBUG("\r\n\r\n");
					
					if(!gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum)
					{
						memcpy(&gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum,&cmd_buffer[NETWORK_ID_INDEX],2);
						
						//Start ANT Network as Device #(Network ID) received
						//b.mu8_AntStartNetwork=1;
												
						if(gu8_AntFlashWriteState==NO_ANT_FLASH_WRITE_OPERATION)
						{
							gu8_AntFlashWriteState=ANT_FLASH_WRITE_INITIAT;
						}
						gu8_AntFlashWriteReason=DEVICE_NO_ADD;
						
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("\r\nFLASH WRITE REASON:");
						gen_PrintHex(gu8_AntFlashWriteReason);
						CRLF;
						#endif
					}
					//If Device #(Network ID) modified
					else if(memcmp(&gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum,&cmd_buffer[NETWORK_ID_INDEX],2))
					{
						memcpy(&gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum,&cmd_buffer[NETWORK_ID_INDEX],2);
						
						if(b.mu8_AntChannelOpenState_ch0==1)
						{
							b.mu8_AntChannelCloseByANT = TRUE;
							err_code = sd_ant_channel_close(CHANNEL_0);
							APP_ERROR_CHECK(err_code);
						}
						
						if(b.mu8_AntChannelOpenState_ch1==1)
						{
							b.mu8_AntChannelCloseByANT = TRUE;
							err_code = sd_ant_channel_close(CHANNEL_1);
							APP_ERROR_CHECK(err_code);				
						}

						if(gu8_AntFlashWriteState==NO_ANT_FLASH_WRITE_OPERATION)
						{
							gu8_AntFlashWriteState=ANT_FLASH_WRITE_INITIAT;
						}
						gu8_AntFlashWriteReason=DEVICE_NO_ADD;
						
						#ifdef ANT_DEBUG_ENABLE
						DEBUG("\r\nFLASH WRITE REASON:");
						gen_PrintHex(gu8_AntFlashWriteReason);
						CRLF;
						#endif
					}
					
//					#ifdef OPERATION_REQ_DBG
						DEBUG("Ch ID=");					
						gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum>>8);					
						gen_PrintHex(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum);
//					#endif
					
					ble_sr_operation_status_update(&m_sr,SUCCESS);
					gu8_CommandIDReqVerificationFlg = FALSE;
				}				
			}
			break;
		case STATUS_REQ:																		//10
			
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nSTATUS_REQ");
			#endif
			//if(UserAuthenticatedFlg==TRUE)
			{
					ble_sr_operation_status_update(&m_sr,SUCCESS);
			}				
						#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nSTATUS_REQ OUT");
			#endif
			break;
			
		default :
				#ifdef OPERATION_REQ_DBG
					DEBUG("\r\nUndefine Command ID Request");
				#endif		
				ble_sr_operation_status_update(&m_sr,OP_STATUS_INVALID_COMMAND_ID_REQ);			
		     break;			
	}
	
	return 0;
}

//***************************************************************
// User Authentication Process
// Verify #SR, Security Tocken, User Device ID
//***************************************************************
void bleP3_UserAuthenticationProcess(uint8_t *data_buffer)
{
	uint8_t lu8_i;
	
	UserAuthenticatedFlg = FALSE;
	
	if(memcmp(&data_buffer[USER_SR_NO_INDEX],&gst_ConfigPermanentParams.mu8ar_SerialNumber[0],SERIAL_NUM_BYTES) == 0)
	{
		if(memcmp(&data_buffer[USER_SECURITY_TOCKEN_INDEX],&gst_ConfigPermanentParams.mu8ar_SecurityTocken[0],SECURITY_TOCKEN_BYTES) == 0)
		{
			for(lu8_i=0; lu8_i < gst_BleConfigDynamicParams.mu8_NumOfUser; lu8_i++)
			{
				if(memcmp(&data_buffer[USER_DEVICE_ID_INDEX],&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[0],USER_DEVICE_ID_BYTES) == 0)
				{
					gu8_AutoManualConnStatus = data_buffer[AUTO_MANU_CONN_STATUS_INDEX];
					UserAuthenticatedFlg = TRUE;
					ble_sr_operation_status_update(&m_sr,SUCCESS);
					break;
				}
			}			
			if(UserAuthenticatedFlg!=TRUE)
			{
				#ifdef OPERATION_REQ_DBG
					DEBUG("\r\nAUTHORIZE_CONN_REQ FAIL");
				 #endif
				ble_sr_operation_status_update(&m_sr,OP_STATUS_USER_NOT_FOUND);
			}
		}
		else
		{
			ble_sr_operation_status_update(&m_sr,OP_STATUS_SECURE_TOKEN_MISMATCH);
		}
	}
	
	else
	{
		ble_sr_operation_status_update(&m_sr,OP_STATUS_SR_NUM_MISMATCH);
	}
}

//***************************************************************
// Command ID request verification
// Verify #SR, Security Tocken, User Device ID
//***************************************************************
void bleP2_CommandIDReqVerificationProcess(uint8_t *data_buffer)
{		
	uint8_t lu8_i;
	
	UserAuthenticatedFlg = FALSE;
	
	if(memcmp(&data_buffer[USER_SR_NO_INDEX],&gst_ConfigPermanentParams.mu8ar_SerialNumber[0],SERIAL_NUM_BYTES) == 0)
	{
		if(memcmp(&data_buffer[USER_SECURITY_TOCKEN_INDEX],&gst_ConfigPermanentParams.mu8ar_SecurityTocken[0],SECURITY_TOCKEN_BYTES) == 0)
		{
			for(lu8_i=0; lu8_i < gst_BleConfigDynamicParams.mu8_NumOfUser; lu8_i++)
			{
				if(memcmp(&data_buffer[USER_DEVICE_ID_INDEX],&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[0],USER_DEVICE_ID_BYTES) == 0)
				{
					UserAuthenticatedFlg = TRUE;
					gu8_CommandIDReqVerificationFlg = TRUE;
					break;
				}
				else if(lu8_i == (gst_BleConfigDynamicParams.mu8_NumOfUser - 1))
				{
					ble_sr_operation_status_update(&m_sr,OP_STATUS_USER_NOT_FOUND);
				}
			}			
			if(UserAuthenticatedFlg!=TRUE)
			{
				#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nCommand ID verification process fail : USER_NOT_FOUND\r\n");
				#endif
				ble_sr_operation_status_update(&m_sr,OP_STATUS_USER_NOT_FOUND);
			}
		}
		else
		{
			#ifdef OPERATION_REQ_DBG
				DEBUG("\r\nSecurity Token Mismatch\r\n");
			#endif
			ble_sr_operation_status_update(&m_sr,OP_STATUS_SECURE_TOKEN_MISMATCH);
		}
	}
	
	else
	{
		#ifdef OPERATION_REQ_DBG
			DEBUG("\r\nSerial Number Mismatch\r\n");
		#endif
		ble_sr_operation_status_update(&m_sr,OP_STATUS_SR_NUM_MISMATCH);
	}
}

/*****************************************************************************
* Find User Number
*****************************************************************************/
int8_t blePx_FindUserDeviceID(uint8_t *UserID)
{
	uint8_t lu8_i;

	for ( lu8_i = 0; lu8_i < gst_BleConfigDynamicParams.mu8_NumOfUser; lu8_i++)
	{
		if(memcmp(&gst_BleConfigDynamicParams.mst_UserRecord[lu8_i].mu8ar_UserDeviceId[0], UserID, USER_DEVICE_ID_BYTES) == 0)
		{
				return lu8_i;
		}
	}
	return -1;
}

/*****************************************************************************
* Factory Reset the configuration parameter
*****************************************************************************/
void sl_factory_reset(void)
{
		uint8_t i;
		
		gst_BleConfigDynamicParams.mu8_NumOfUser = 0;
		gst_BleConfigDynamicParams.mu8_SrDevStatus = 0;
	
		gst_BleConfigDynamicParams.mu8_UserRecordUpdateCounter = 0;	
	
		for ( i = 0; i < MAX_USER; i++)
		{
				memset((void *) &gst_BleConfigDynamicParams.mst_UserRecord[i], 0, sizeof(gst_BleConfigDynamicParams.mst_UserRecord[i]));
		}

		gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord = 0;
		
		for ( i = 0; i < MAX_AUDIT_LOG; i++)
		{
				memset((void *) &gst_BleConfigDynamicParams.mst_AuditLogRecord[i], 0, sizeof(gst_BleConfigDynamicParams.mst_AuditLogRecord[i]));
		}
			
		
//		FlashWrite(&mp_flash_BLE_DynPara,(uint8_t *)&gst_BleConfigDynamicParams,sizeof(st_BleFlashDynamicParams_t));
		if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
		{
			BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;
		}
		
		#ifdef RESET_DBG
				simple_uart_putstring("\r\n**Fact Dyn para**");
		#endif
		
		FlashWrite(&mp_flash_BLE_DynPara,(uint8_t *)&gst_BleConfigDynamicParams,sizeof(st_BleFlashDynamicParams_t));
//		bw_configParamUpdate(DYNAMIC);
//		memset((char *)&g_ReqMsg, 0, sizeof(g_ReqMsg));	
		
		#ifdef RESET_DBG
				simple_uart_putstring("\r\n**Fact rst cmpl**");
		#endif
}

/*****************************************************************************
* Store Audit Log
*****************************************************************************/
void blePx_AuditLog(uint8_t *msg , uint8_t button_type)
{
	int32_t index = 0, temp;

	if(gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord >= MAX_AUDIT_LOG)
	{
		//Shift record by one to maintain FIFO
		for (index = 0; index < (MAX_AUDIT_LOG - 1) ; index++)
		{
			gst_BleConfigDynamicParams.mst_AuditLogRecord[index] = gst_BleConfigDynamicParams.mst_AuditLogRecord[index + 1];	
		}			
	}
	else
	{
		// Storing Audit log information
		index = (gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord) % MAX_AUDIT_LOG;				
	}

	// Reset record before storing any new data
	memset((void *) &gst_BleConfigDynamicParams.mst_AuditLogRecord[index], 0, sizeof(gst_BleConfigDynamicParams.mst_AuditLogRecord[index]));		

	if(button_type==NULL)
	{
//		memcpy(&gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mst_TimeStamp, &msg[USER_TIME_INDEX], TIME_STAMP_BYTES);
		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mst_TimeStamp.mu8_Month = msg[USER_TIME_INDEX];
		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mst_TimeStamp.mu8_Day = msg[USER_TIME_INDEX+1];
		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mst_TimeStamp.mu8_Year = msg[USER_TIME_INDEX+2];
		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mst_TimeStamp.mu8_Hour = msg[USER_TIME_INDEX+3];
//		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mst_TimeStamp.mu8_Minute = msg[USER_TIME_INDEX+4];

		temp = blePx_FindUserDeviceID(&msg[USER_DEVICE_ID_INDEX]);
		if(temp >= 0)
		{
			memcpy(&gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mu8_UserDeviceId[0], &gst_BleConfigDynamicParams.mst_UserRecord[temp].mu8ar_UserDeviceId[0], USER_DEVICE_ID_BYTES);
		}
		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mu8_OperationType = msg[OPERATION_REQ_ID_INDEX];
	}
	else
	{
		memset(&gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mst_TimeStamp,0,sizeof(gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mst_TimeStamp));
		memset(&gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mu8_UserDeviceId[0],0, USER_DEVICE_ID_BYTES);
		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mu8_OperationType = button_type;
	}
	
	if((gst_BleConfigDynamicParams.mu8_BatteryLevel<=UPPER_THR) && (gst_BleConfigDynamicParams.mu8_BatteryLevel>LOWER_THR))
	{
		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mu8_OperationStatus = (op_status | 0x80);
	}		
	else
	{
		gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mu8_OperationStatus = op_status;
	}
	
	gst_BleConfigDynamicParams.mst_AuditLogRecord[index].mu8_OperationType = msg[OPERATION_REQ_ID_INDEX];
	gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord++;
		
	if (gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord >= MAX_AUDIT_LOG)
	{
		gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord = MAX_AUDIT_LOG;
	}
	if(BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION)
		BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_INITIAT;				
}


/*****************************************************************************
Covert axis value in to G force
*****************************************************************************/

void convert_in_g_force(uint8_t *data)
{
		uint8_t i=0;
		uint8_t j=1;
		int16_t value_of_g;
		float calculation_of_g[3];
	
		//Clear the notification data buffer
		memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
	
		while(i < 3)
		{
			if(data[i]<=31)
			{
				calculation_of_g[i] = (data[i]) * (0.047);			
				value_of_g = (int16_t ) (1000*calculation_of_g[i]);
				dev_kit_op_status[j] = (value_of_g>>8);
				dev_kit_op_status[++j] = value_of_g;
			}
			else if(32<=data[i] || data[i]<=63)
			{
				calculation_of_g[i] = (data[i]-64) * (0.047);
				value_of_g = (int16_t ) (1000*calculation_of_g[i]);
				dev_kit_op_status[j] = (value_of_g>>8);
				dev_kit_op_status[++j] = value_of_g;
			}
			++i;
			++j;
		}
		
		dev_kit_op_status[0]=ACCELEROMETER;				
		ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
		#ifdef RUN_DBG
			sprintf(dbgStr, "\r\nX = %f \r\nY = %f \r\nZ = %f \r\n", calculation_of_g[0], calculation_of_g[1], calculation_of_g[2]);
			simple_uart_putstring(dbgStr);
		#endif
}

/*
Send water sensor level
*/
void update_water_sensor_status()
{
		memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
		dev_kit_op_status[0]=WATER_SENSOR;
		dev_kit_op_status[1]=nrf_gpio_pin_read(WATER_SENSOR_PIN_NUMBER);					
		ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
}

/**@brief
*/
uint32_t ble_UART_message_update(ble_sr_t * p_mtr, uint8_t * data , uint16_t len)
{
    uint32_t err_code = NRF_SUCCESS;
		uint8_t op_status[10];
		
		memcpy(op_status, data, len);

		// Save new operation status value
		//p_mtr->operation_status_last = operation_status;
		
		// Update database
		err_code = sd_ble_gatts_value_set(p_mtr->UART_data_handle.value_handle,
																	 0,
																	 &len,
																	 &op_status[0]);
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}

		// Send value if connected and notifying
		if ((p_mtr->conn_handle != BLE_CONN_HANDLE_INVALID) && p_mtr->is_notification_supported)
		{
				ble_gatts_hvx_params_t hvx_params;
				
				memset(&hvx_params, 0, sizeof(hvx_params));
				len = sizeof(uint8_t);
				
				hvx_params.handle   = p_mtr->UART_data_handle.value_handle;
				hvx_params.type 		= BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset   = 0;
				hvx_params.p_len    = &len;
				hvx_params.p_data   = &op_status[0];
				
				err_code = sd_ble_gatts_hvx(p_mtr->conn_handle, &hvx_params);

				#ifdef RUN_DBG								
//						sprintf(dbgStr, "\r\nerr_code = %d operation_status = %d %s %d", err_code, operation_status, __FILE__, __LINE__);
//						simple_uart_putstring(dbgStr);					
				#endif
		}
		else
		{
				err_code = NRF_ERROR_INVALID_STATE;
		}

    return err_code;
}

/*****************************************************************************
* DEVKIT COMMAND EXECUTION
*****************************************************************************/
void devkitCommandExecution(void)	
{
			uint16_t co2_level;
			uint8_t  lu8_i;	
		//*****************************************************************
		//Temperature
		//*****************************************************************
		if(temperature_notify_flag)
		{	
			temperature = get_temperature();
			thermostat_temp = (int8_t) temperature;
			updated_temperature =(int16_t )(100*temperature);
			
				//sprintf(dbgStr, "\r\n C = %ul \r\n",updated_temperature);
				//simple_uart_putstring(dbgStr);
			
			//Clear the notification data buffer
			memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
			dev_kit_op_status[0]= TEMP_SENSOR;
			dev_kit_op_status[1]= (updated_temperature>>8);
			dev_kit_op_status[2]= updated_temperature;
			ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
			temperature_notify_flag=false;
			sl_app_timer[TEMP_SCAN_TIMER]=TEMP_SCAN_TIME;
			
			#ifdef RUN_DBG
				//sprintf(dbgStr, "\r\n C = %f \r\n", temperature);
				//simple_uart_putstring(dbgStr);
			#endif
		}

		//*****************************************************************
		//Accelerometer
		//*****************************************************************
		if(accelerometer_update_flg && (!repeted_start_flg))
		{
			Read_Coordinate_Data();
			Tilt_Reg_Data();
			#ifdef RUN_DBG
				sprintf(dbgStr, "\r\nX = %d \r\nY = %d \r\nZ = %d \r\nTilt = %d \r\n", TWI_RxData[0], TWI_RxData[1], TWI_RxData[2], TWI_RxData[3]);
				simple_uart_putstring(dbgStr);
			#endif
			convert_in_g_force(&TWI_RxData[0]);
			accelerometer_update_flg=false;
		}

		//*****************************************************************
		//Co2 sensor
		//*****************************************************************
		if(co2_notify_flag)
		{
			adc_CO2_init();
			co2_level = get_co2_voltage();
			if(co2_level>300)
			{
				green_led_on();
				nrf_gpio_pin_set(SOLENOID_CTR_PIN_NUMBER);
				if((co2_last_status==false) || (co2_notify_on_connect_flg==true))
				{
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]= CO2_SENSOR;
					dev_kit_op_status[1]= CO2_SENSOR_DETECTED;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
				}
				co2_last_status=true;
			}					
			else if(co2_level<=300)
			{					
				green_led_off();
				nrf_gpio_pin_clear(SOLENOID_CTR_PIN_NUMBER);
				if((co2_last_status==true) || (co2_notify_on_connect_flg==true))
				{
					memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
					dev_kit_op_status[0]= CO2_SENSOR;
					dev_kit_op_status[1]= CO2_SENSOR_REMOVED;
					ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
				}
				co2_last_status=false;
			}
			co2_notify_flag=false;
			co2_notify_on_connect_flg=false;
		}

		//*****************************************************************
		//Thermostat
		//*****************************************************************
		if(thermostat_update_flag)
		{	
			if(one_min_timer_flg)
			{
				if(heating_action_time>0)
				{
					--heating_action_time;
					
					if((heating_action_time==0) && (temperature < heating_set_point))
					{
						nrf_gpio_pin_set(RELAY_1_CTR_PIN_NUMBER);
						nrf_gpio_pin_set(GREEN_LED_PIN_NUMBER);
						heating_relay_status_on_flag=true;
						sl_app_timer[THERMOSTAT_TRG_TIMER] = THERMOSTAT_TRG_TIME;
						memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
						dev_kit_op_status[0]= THERMOSTAT;
						dev_kit_op_status[1]= 1;
						ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
						#ifdef RUN_DBG
							simple_uart_putstring("\r\n Heating relay ON");
						#endif
					}	
					else if(heating_action_time == 0)
					{
						simple_uart_putstring("\r\n Heating action time out");
					}
				}
				
				if(cooling_action_time>0)
				{
					--cooling_action_time;
					if((cooling_action_time==0) && (temperature > cooling_set_point))
					{
						nrf_gpio_pin_set(RELAY_2_CTR_PIN_NUMBER);
						green_led_on();
						cooling_relay_status_on_flag=true;
						sl_app_timer[THERMOSTAT_TRG_TIMER] = THERMOSTAT_TRG_TIME;
						memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
						dev_kit_op_status[0]= THERMOSTAT;
						dev_kit_op_status[1]= 0;
						ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
						#ifdef RUN_DBG
							simple_uart_putstring("\r\n Cooling relay ON");
						#endif
					}
					else if(cooling_action_time == 0)
					{
						simple_uart_putstring("\r\n Cooling action time out");
					}
				}						
				one_min_timer_flg=false;
		  }
			
			if(heating_relay_status_on_flag || cooling_relay_status_on_flag)
			{
				if((heating_action_time==0) && (temperature >= heating_set_point) && (heating_relay_status_on_flag))
				{
					nrf_gpio_pin_set(RELAY_1_CTR_PIN_NUMBER);
					green_led_on();
					sl_app_timer[THERMOSTAT_TRG_TIMER] = THERMOSTAT_TRG_TIME;
					heating_relay_status_on_flag=false;
					#ifdef RUN_DBG
							simple_uart_putstring("\r\n H Ok");
					#endif
				}
				
				if((cooling_action_time==0) && (temperature <= cooling_set_point) && (cooling_relay_status_on_flag))
				{
					nrf_gpio_pin_set(RELAY_2_CTR_PIN_NUMBER);
					green_led_on();
					sl_app_timer[THERMOSTAT_TRG_TIMER] = THERMOSTAT_TRG_TIME;
					cooling_relay_status_on_flag=false;
					#ifdef RUN_DBG
							simple_uart_putstring("\r\n C Ok");
					#endif
				}
			}
		} 

		//********************************************************************
		//Humidity sensor
		//********************************************************************
		if(humidity_temperature_update_flg)
		{
			humidity_temperature_update_flg=false;
			Humidity_sensor_data();						
		}			

		if(twi_operation_complete && twi_ack_received && TWI_RxData[0]!=0 && TWI_RxData[1]!=0 && device_type==HUMIDITY_SENSOR)
		//if(twi_operation_complete && twi_ack_received && device_type==HUMIDITY_SENSOR)	
		{					
			humidity=0;
			humidity = TWI_RxData[1] | (TWI_RxData[0]<<8);
			humidity = ((125 * (humidity))/65536) - 6;
			
			#ifdef RUN_DBG
				sprintf(dbgStr, "\r\nHumidity = %f\r\n",humidity);
				simple_uart_putstring(dbgStr);
			#endif
			
			TWI_RxData[0]=0;
			TWI_RxData[1]=0;
			
			memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
			dev_kit_op_status[0]= HUMIDITY_SENSOR;
			dev_kit_op_status[1]= (uint8_t) humidity;
			dev_kit_op_status[2]= updated_temperature >> 8;
			dev_kit_op_status[3]= updated_temperature;
			ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
			humidity=0;
		}
		//**************************************************************
		//Water sensor
		//**************************************************************
		if(water_sensor_update_flg)
		{
			update_water_sensor_status();
			water_sensor_update_flg=false;		
			if(nrf_gpio_pin_read(WATER_SENSOR_PIN_NUMBER))
			{
				water_sensor_flg=true;
				nrf_gpio_pin_set(SOLENOID_CTR_PIN_NUMBER);
				green_led_on();
			}		
		}

		//**************************************************************
		//Battery voltage measurement
		//**************************************************************
		if(bat_voltage_update_flg)
		{
			bat_voltage_update_flg = false;
			battery_voltage = get_battery_voltage();
			
			#ifdef RUN_DBG
				sprintf(dbgStr, "\r\nADC count = %f\r\n",battery_voltage);
				simple_uart_putstring(dbgStr);
			#endif
			
			battery_voltage = (battery_voltage * 11.75) / 1000;
			
			
			#ifdef RUN_DBG
				sprintf(dbgStr, "\r\nbattery_voltage = %f\r\n",battery_voltage);
				simple_uart_putstring(dbgStr);
			#endif
			
			battery_voltage *= 10; 
			memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
			dev_kit_op_status[0]= BAT_VOLTAGE;
			dev_kit_op_status[1]= (uint8_t) battery_voltage;
			ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
		}

		//**************************************************************
		//Battery voltage measurement
		//**************************************************************
		if(motion_update_flg)
		{								  
			motion_update_flg = false;
			if((nrf_gpio_pin_read(MOTION_DETECT_PIN_NUMBER) == HIGH) && (!motion_detect_flg))
			{
				motion_detect_flg=true;						
				memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
				dev_kit_op_status[0]=MOTION_SENSOR;
				dev_kit_op_status[1]=MOTION_DETECT;					
				ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
			}					
		}

		if(hw_info_update_flg)
		{
			//sprintf(dbgStr, "\r\nhw_info_update_flg\r\n");
			hw_info_update_flg = false;
			memset(&dev_kit_op_status[0], 0, OP_STATUS_BUFF_LEN);
			dev_kit_op_status[0]=HW_CONFIG;
			dev_kit_op_status[1]=hardware_type;
			dev_kit_op_status[2]=humidity_sensor_presence;	
			dev_kit_op_status[3]=tx_power;			
			ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
		}
		if(msg_rec_on_ble_for_UART_com_flg==TRUE)
		{
			
			simple_uart_putstring("\r\n");
			for(lu8_i=0; lu8_i< UART_BUFF_LEN; lu8_i++)
			{
				simple_uart_put(dev_kit_UART_msg[lu8_i]);
			}
			simple_uart_putstring("\r\n In HEX : ");
			gen_PrintHexStr(dev_kit_UART_msg,UART_BUFF_LEN);
			msg_rec_on_ble_for_UART_com_flg = FALSE;
		}
		/*
		if(UartDataRxFlg == TRUE)
		{
			simple_uart_putstring((uint8_t *)"sent Data = ");
			for (uint8_t i = 0; i< UART_BUFF_LEN; i++ )
			{
				simple_uart_put(rx_buf[i]);
			}
			ble_UART_message_update(&m_sr, &rx_buf[0], UART_BUFF_LEN);
			
			UartDataRxFlg = FALSE;
		}*/
}

uint8_t bleP0_ReceviedBleMsgOnAnt(uint8_t *buffer)
{
	uint8_t ret_value;
	
	#ifdef RECEIVED_REQ_DBG
		DEBUG("\r\nData received on ANT = ");
		gen_PrintHexStr(&write_cmd[0],WRITE_BUFFER_SIZE);		
	#endif	
	
	bleP2_CommandIDReqVerificationProcess(&buffer[0]);
	if(gu8_CommandIDReqVerificationFlg == TRUE)
	{		
		gu8_OperationType = buffer[REMOTE_OP_OPERA_TYPE_INDEX-1];

//		if(gu8_OperationType == LOCK_REQ)
//		{
//			ret_value = sl_lock();
//		}
//		else if(gu8_OperationType == UNLOCK_REQ)
//		{
//			ret_value = sl_unlock();
//		}
		gu8_CommandIDReqVerificationFlg = FALSE;
	}
	return ret_value;
}
