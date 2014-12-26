#include <string.h>
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "devkit\devkit_config.h"
#include "devkit\devkit_variable.h"
#include "main.h"
#include "simple_uart.h"
#include "nrf_gpio.h"
#include "battery.h"
#include "led.h"
#include "ble_flash.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "pstorage.h"

char tx_buf[TX_BUF_SIZE]={0};
extern unsigned char read_cmd[READ_BUFFER_SIZE];

extern uint32_t err_code;

extern uint8_t		gu8_SerialNumberEncrypted[SERIAL_NUM_BYTES+4];

st_FlashPermanentParams_t gst_ConfigPermanentParams;
st_BleFlashDynamicParams_t gst_BleConfigDynamicParams;
st_FlashDataOfPreviousVersion_t gst_FlashDataOfPreviousVersion;

//*****************************************************************************
//  configure smartlock by UART (smartlock/securemote production tool)
//*****************************************************************************
void sl_config_by_uart(void)
{
// Need to implement
	
	    uint16_t msg_len,bat_volt;
    uint8_t crcVal;
    
    msg_len = (rx_buf[1]+(rx_buf[0]*256));        
    if(msg_len==(sl_rcvcntr - 2))                           // Check length       
    {
        crcVal = cal_CRC(0,(sl_rcvcntr-1),&rx_buf[0]);
        if(crcVal == rx_buf[sl_rcvcntr-1])                  // Check CRC
        {
            switch(rx_buf[REQUEST_FIELD_POS])            		// Check Request Type   
            {
                case SET_CONFIG_PARA:            						//  Request to set configuration para
					if(gst_BleConfigDynamicParams.mu8_SrDevStatus==NON_ACTIVE)
					{
							sl_store_config_para(rx_buf,NUM_OF_CFG_PARA_POS);
							sl_reply_to_pctool();
	//						ble_sl_app_stop(); 								 // Stop all smart lock functionality before disabling the SoftDevice
	//						ble_stack_stop();				  				 // Disable the S110 stack	
							FlashWrite(&mp_flash_PermPara,(uint8_t *)&gst_ConfigPermanentParams,sizeof(st_FlashPermanentParams_t));
							FlashWrite(&mp_flash_BLE_DynPara,(uint8_t *)&gst_BleConfigDynamicParams,sizeof(st_BleFlashDynamicParams_t));
//							bw_configParamUpdate(BOTH);				 // Update In Flash		
							//tunr_on_all_LED();
	//						nrf_delay_us(10000);
	//						NVIC_SystemReset();	
							sl_app_timer[SYSTEM_RESET_TIMER]=SYSTEM_RESET_TIME;
					}
					else if(gst_BleConfigDynamicParams.mu8_SrDevStatus==ACTIVE)
					{
							tx_buf[0]=0x00;
							tx_buf[1]=0x03;
							tx_buf[REQUEST_FIELD_POS] = 0xA0 | rx_buf[REQUEST_FIELD_POS];
							tx_buf[3]= 1;    // 0=SUCCESS and 1=FAIL
							tx_buf[4]=cal_CRC(0,4,&tx_buf[0]);
							sl_xmit_data(&tx_buf[0],5);  										
					}											
				break;
					
				case RANGE_TES_REQ:
					
				break;
				
                case UART_TES_REQ:             				  //  Request to verify UART of Smart Lock
					
					sl_read_config_para(rx_buf[PARA_ID_POS]);
				
				break;								

													
                case BAT_VOL_MEASURE_REQ:               //  Request to verify BATTERY MEASUREMENT of Smart Lock
					bat_volt=get_battery_voltage();
					tx_buf[0]=0x00;
					tx_buf[1]=0x04;
					tx_buf[REQUEST_FIELD_POS] = 0xA0 | rx_buf[REQUEST_FIELD_POS];		
					tx_buf[3] = (bat_volt & 0xff00) >> 8;
					tx_buf[4] = (bat_volt & 0x00ff) ;	
					tx_buf[5]=cal_CRC(0,5,&tx_buf[0]);
					sl_xmit_data(&tx_buf[0],6);   								
				break;
							
			}
		}
	}
    memset(rx_buf,0x00,sizeof(rx_buf));    
    sl_rcvcntr=0;  	


}

//
//------------------------------------------------------------------------------
// Read Configuration Parameter as per ID from SmartLock and send back to production tool
//------------------------------------------------------------------------------
void sl_read_config_para(uint8_t id)
{
    uint8_t paramLength=0;
	
		tx_buf[0]=0x00;
		tx_buf[REQUEST_FIELD_POS] = 0xA0 | rx_buf[REQUEST_FIELD_POS];			
	
		switch(id)
		{
				case SL_SERIALNUM_ID:
							paramLength = SERIAL_NUM_BYTES;
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_SerialNumber,paramLength);  
							break;
				case SL_AUTOLOCK_TMR_ID:
							paramLength = 1;					
							tx_buf[PARA_VALUE_POS]=gst_BleConfigDynamicParams.mu8_AutoLockTimer;
							break;
 				case SL_MANUFACT_NAME_ID:
							paramLength = MANUFACTURE_NAME_BYTES;	
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_ManufactName,paramLength);  				
							break;
				case SL_MODELNUM_ID:
							paramLength = MODEL_NUM_BYTES;	
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_ModelNumber,paramLength);  					
							break;
				case SL_SWVERSION_ID:
							paramLength = SW_VER_BYTES;	
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_SoftwareVersion,paramLength); 					
							break;                    
				case SL_HWVERSION_ID:
							paramLength = HW_VER_BYTES;	
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_HardwareVersion,paramLength); 
							break;
				case SL_FCCCERTI_ID:
							paramLength = FCC_CERTI_BYTES;	
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_FccCertificate,paramLength); 					
							break;
				case SL_MANUFAC_DATE_ID:
							paramLength = MANUFACTURE_DATE_BYTES;	
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_ManufactDate,paramLength); 					
							break;                    
				case SL_DEVICE_NAME_ID:
							paramLength = DEVICE_NAME_BYTES;	
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_DeviceName,paramLength); 						
							break;
				case SL_CONFIG_STT_ID:
							paramLength = 1;					
							tx_buf[PARA_VALUE_POS]=gst_BleConfigDynamicParams.mu8_SrDevStatus;					
							break;
				case SL_CONFIG_BUILD_ID:
							paramLength = BUILD_ID_BYTES;					
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_BuildId,paramLength); 				
							break;
				
				//RAS 1.6
				case SL_ANT_SERIAL_NUMBER_ID:
							paramLength = ANT_SERIAL_NUMBER_BYTES;
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_AntSerialNumber,paramLength);
							break;
				case SL_AES_KEY_1_ID:
							paramLength = AES_KEY_BYTES;
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_AesKey1,paramLength);
							break;
				case SL_AES_KEY_2_ID:
							paramLength = AES_KEY_BYTES;
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_AesKey2,paramLength);
							break;
				case SL_AES_KEY_3_ID:
							paramLength = AES_KEY_BYTES;
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_AesKey3,paramLength);
							break;
				case SL_SECURITY_TOKEN_ID:
							paramLength = SECURITY_TOCKEN_BYTES;
							memcpy(&tx_buf[PARA_VALUE_POS],gst_ConfigPermanentParams.mu8ar_SecurityTocken,paramLength);
							break;
				case SL_SECURITY_LEVEL_ID:
							paramLength = 1;
							tx_buf[PARA_VALUE_POS]=gst_BleConfigDynamicParams.mu8_Securitylevel;
							break;
				case SL_DEVICE_TYPE_ID:
							paramLength = 1;
							tx_buf[PARA_VALUE_POS]=gst_BleConfigDynamicParams.mu8_DeviceType;
							break;
    }
		tx_buf[1] = (2 + paramLength);
		tx_buf[PARA_VALUE_POS + paramLength] = cal_CRC(0,(PARA_VALUE_POS + paramLength),&tx_buf[0]);
		sl_xmit_data(&tx_buf[0],(PARA_VALUE_POS + paramLength + 1));   		
}

//------------------------------------------------------------------------------
// Store Configuration Parameter Received from SmartLock/SecuRemote Production tool
//------------------------------------------------------------------------------
void sl_store_config_para(char *msg,uint8_t pos)
{
    uint8_t paramID,paramLength;  
    uint8_t num_of_confg_para,index;
      
    num_of_confg_para = *(msg + pos);
    index = (pos+1);	              //first parameter starts from here

    while(num_of_confg_para!=0)
    {
        paramID = *(msg+index);
        index++;
        paramLength = *(msg+index);
        index++;
        switch(paramID)
        {
            case SL_SERIALNUM_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_SerialNumber,&msg[index],paramLength);  
                  break;
            case SL_AUTOLOCK_TMR_ID:
                  gst_BleConfigDynamicParams.mu8_AutoLockTimer=*(msg+index);
                  break;
            case SL_TOTAL_AUDIT_LOG_ID:
                  break;  
            case SL_TOTAL_USER_ID:
                  break;                    
            case SL_MANUFACT_NAME_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_ManufactName,&msg[index],paramLength);
                  break;
            case SL_MODELNUM_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_ModelNumber,&msg[index],paramLength);
                  break;
            case SL_SWVERSION_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_SoftwareVersion,&msg[index],paramLength);
                  break;                    
            case SL_HWVERSION_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_HardwareVersion,&msg[index],paramLength);
                  break;
            case SL_FCCCERTI_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_FccCertificate,&msg[index],paramLength);
                  break;
            case SL_MANUFAC_DATE_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_ManufactDate,&msg[index],paramLength);
                  break;                    
            case SL_DEVICE_NAME_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_DeviceName,&msg[index],paramLength);
                  break;
            case SL_CONFIG_STT_ID:
                  gst_BleConfigDynamicParams.mu8_SrDevStatus=*(msg+index);							
                  break;	
            case SL_KEYFOB_INVITE_ID:
                  break;	
            case SL_KEYFOB_SECKEY_ID:
                  break;							
            case SL_CONFIG_BUILD_ID:
                  memcpy(gst_ConfigPermanentParams.mu8ar_BuildId,&msg[index],paramLength);					
                  break;			
			
			//RAS 1.6
			case SL_ANT_SERIAL_NUMBER_ID:
						memcpy(gst_ConfigPermanentParams.mu8ar_AntSerialNumber,&msg[index],paramLength);
						break;
			
			case SL_AES_KEY_1_ID:
						memcpy(gst_ConfigPermanentParams.mu8ar_AesKey1,&msg[index],paramLength);
						break;
			
			case SL_AES_KEY_2_ID:
						memcpy(gst_ConfigPermanentParams.mu8ar_AesKey2,&msg[index],paramLength);
						break;
			
			case SL_AES_KEY_3_ID:
						memcpy(gst_ConfigPermanentParams.mu8ar_AesKey3,&msg[index],paramLength);
						break;
			
			case SL_SECURITY_TOKEN_ID:
						memcpy(gst_ConfigPermanentParams.mu8ar_SecurityTocken,&msg[index],paramLength);
						break;
			
			case SL_SECURITY_LEVEL_ID:
						gst_BleConfigDynamicParams.mu8_Securitylevel=*(msg+index);
						break;
			
			case SL_DEVICE_TYPE_ID:
						gst_BleConfigDynamicParams.mu8_DeviceType=*(msg+index);
						break;		
        }        
        index += paramLength;
        num_of_confg_para--;
    }
}

//------------------------------------------------------------------------------
// Send Response to PC tool
//------------------------------------------------------------------------------
void sl_reply_to_pctool(void)
{
    switch(rx_buf[REQUEST_FIELD_POS])                // Check Request Type   
    {
        case SET_CONFIG_PARA:                        // Reply to confirmation				
			  tx_buf[0]=0x00;
              tx_buf[1]=0x03;
              tx_buf[REQUEST_FIELD_POS] = 0xA0 | rx_buf[REQUEST_FIELD_POS];
              tx_buf[3]= 0;    // 0=SUCCESS and 1=FAIL
              tx_buf[4]=cal_CRC(0,4,&tx_buf[0]);
              sl_xmit_data(&tx_buf[0],5);              
              break;
    }   
}
		
//------------------------------------------------------------------------------
// Calculate CRC of Data
//------------------------------------------------------------------------------
uint8_t cal_CRC(uint8_t StartIndex,uint8_t Count,char *msg)
{
    uint32_t Total = 0;
    uint8_t i;
    for (i = StartIndex; i < Count; i++)
    {
        Total += (uint32_t)*(msg + i);
    }
    Total = Total & 0xFF;
    Total = ((~Total) + 1) & 0xFF;
    
    return (uint8_t)(Total);
}


//------------------------------------------------------------------------------
// Encryption Of Serial Number
// Key : 0xd1d2a3a4b5e8f5c6e23f4a780f4e6cf9
//------------------------------------------------------------------------------

uint32_t k0=0xd1d2a3a4;
uint32_t k1=0xb5e8f5c6;
uint32_t k2=0xe23f4a78;
uint32_t k3=0x0f4e6cf9;

void SmartDeviceEncryptSerialNumber(void)
{
		uint32_t v_1,v_2,sum=0, i;
		uint32_t delta=0x9e3779b9;	
 		v_1 = deserialize(&gu8_SerialNumberEncrypted[0]);
 		v_2 = deserialize(&gu8_SerialNumberEncrypted[4]);
		for (i=0; i < 32; i++)
		{
				sum += delta;
				v_1 += ((v_2<<4) + k0) ^ (v_2 + sum) ^ ((v_2>>5) + k1);
				v_2 += ((v_1<<4) + k2) ^ (v_1 + sum) ^ ((v_1>>5) + k3);
		}	
 		serialize(&gu8_SerialNumberEncrypted[0],v_1);
 		serialize(&gu8_SerialNumberEncrypted[4],v_2);
		
		sum=0;

 		v_1 = deserialize(&gu8_SerialNumberEncrypted[8]);
 		v_2 = deserialize(&gu8_SerialNumberEncrypted[12]);
		for (i=0; i < 32; i++)
		{
				sum += delta;
				v_1 += ((v_2<<4) + k0) ^ (v_2 + sum) ^ ((v_2>>5) + k1);
				v_2 += ((v_1<<4) + k2) ^ (v_1 + sum) ^ ((v_1>>5) + k3);
		}	
 		serialize(&gu8_SerialNumberEncrypted[8],v_1);
 		serialize(&gu8_SerialNumberEncrypted[12],v_2);		
		
		sum=0;
		
		v_1 = deserialize(&gu8_SerialNumberEncrypted[16]);
		for (i=0; i < 32; i++)
		{
				sum += delta;
				v_1 += ((v_2<<4) + k0) ^ (v_2 + sum) ^ ((v_2>>5) + k1);
		}	
 		serialize(&gu8_SerialNumberEncrypted[16],v_1);
	
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

uint32_t deserialize(uint8_t *buffer)
{
		uint32_t value = 0;

		value |= buffer[0] << 24;
		value |= buffer[1] << 16;
		value |= buffer[2] << 8;
		value |= buffer[3];
	
		return value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void serialize(uint8_t *buffer,uint32_t value)
{
		buffer[0] = (value & 0xff000000) >> 24;
		buffer[1] = (value & 0x00ff0000) >> 16;
		buffer[2] = (value & 0x0000ff00) >> 8;
		buffer[3] = (value & 0x000000ff) ;	
}

void SmartDeviceSerialNumberEncryptBy_AES_CBC(void)
{
	blePx_EncryptData(gst_ConfigPermanentParams.mu8ar_AesKey1,&gu8_SerialNumberEncrypted[0],SERIAL_NUM_BYTES,0);
	memcpy(&gu8_SerialNumberEncrypted[0],&read_cmd[0],SERIAL_NUM_BYTES);
}

