#include <string.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf51_bitfields.h"
#include "boards/ras_1_6.h"
#include "app_util.h"
#include "led.h"
#include "simple_uart.h"
#include "led.h"
#include "nrf_delay.h"
#include "devkit\devkit_variable.h"
#include "devkit\devkit_motor.h"
#include "devkit\devkit_command.h"
#include "devkit\devkit_application.h"
#include "devkit\devkit_config.h"
#include "ble_conn_params.h"
#include "nrf_soc.h"
#include "ble_conn_params.h"
#include "ble_sr.h"
#include "battery.h"
#include "main.h"
#include "ble_advdata.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Extern variable
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
extern st_BleFlashDynamicParams_t gst_BleConfigDynamicParams;
extern uint8_t 	sl_rcvtimeout;
extern uint8_t 	sl_rcvmsg_ok;
extern uint8_t  sl_conn_state;

extern uint8_t 	uart_state;
extern char		tx_buf[TX_BUF_SIZE];
extern uint8_t 	op_status;
extern ble_sr_t  m_sr; 

/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/
extern uint32_t STATUS_SW1,STATUS_SW2,DEVKIT_MOTOR_OP_POS_PIN,DEVKIT_MOTOR_OP_NEG_PIN;
extern uint8_t DEVKIT_TX_PIN_NUMBER;

extern uint8_t hardware_type;

extern bool cc0_turn;

extern uint8_t ant_request; 
/** Used in Timer Activity **/
extern bool solenoid_status;
extern bool temperature_notify_flag;
extern bool accelerometer_update_flg;
extern bool co2_notify_flag;
extern bool motion_update_flg;
extern bool one_min_timer_flg;
extern bool humidity_temperature_update_flg;
extern bool bat_voltage_update_flg;
extern bool Hardware_config_flg;
extern bool water_sensor_flg;
extern bool motion_detect_flg;

extern uint8_t 		operation_type;
extern uint8_t 		dev_kit_op_status[OP_STATUS_BUFF_LEN];
extern uint8_t 		device_type;
extern uint8_t 		sw_previous_status;
/** ** **/


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// local variable
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint16_t sl_app_timer[MAX_APP_TIMER]={0};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize GPIO Of Development_kit
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//*****************************************************************
//Configure for NS_BLE hardware
//*****************************************************************
void config_hardware_for_NS_BLE(void)
{
	STATUS_SW1 = STATUS_SW1_NS_BLE;					
	STATUS_SW2 = STATUS_SW2_NS_BLE;					
	DEVKIT_MOTOR_OP_POS_PIN = MOTOR_OP_POS_PIN_NS_BLE;		
	DEVKIT_MOTOR_OP_NEG_PIN = MOTOR_OP_NEG_PIN_NS_BLE;		
	DEVKIT_TX_PIN_NUMBER = TX_PIN_NUMBER_NS_BLE;
}

//*****************************************************************
//Configure for N548 hardware
//*****************************************************************
void config_hardware_for_N548(void)
{
	STATUS_SW1 = STATUS_SW1_N548;					
	STATUS_SW2 = STATUS_SW2_N548;					
	DEVKIT_MOTOR_OP_POS_PIN = MOTOR_OP_POS_PIN_N548;			
	DEVKIT_MOTOR_OP_NEG_PIN = MOTOR_OP_NEG_PIN_N548;			
	DEVKIT_TX_PIN_NUMBER = TX_PIN_NUMBER_N548;	
}


void development_kit_gpio_init(void)
{
	if(hardware_type == HARDWARE_NS_BLE)
	{
		config_hardware_for_NS_BLE();				
	}
	else if(hardware_type == HARDWARE_NS_548)
	{		
		config_hardware_for_N548();								
	}
	else
	{
		config_hardware_for_NS_BLE();
		hardware_type = 0;
	}
	
	//Serial to parallel convertor 
	nrf_gpio_cfg_output(DEVKIT_MOTOR_OP_POS_PIN);		
	nrf_gpio_cfg_output(DEVKIT_MOTOR_OP_NEG_PIN);
	nrf_gpio_cfg_output(SOLENOID_CTR_PIN_NUMBER);
	nrf_gpio_cfg_output(RELAY_1_CTR_PIN_NUMBER);
	nrf_gpio_cfg_output(RELAY_2_CTR_PIN_NUMBER);
	nrf_gpio_cfg_output(GREEN_LED_PIN_NUMBER);
	nrf_gpio_cfg_output(BLUE_LED_PIN_NUMBER);
	nrf_gpio_cfg_output(WDT_PIN_NO);
	nrf_gpio_cfg_output(PWM_OUTPUT_PIN_NUMBER);
	
//	nrf_gpio_cfg_input(29,NRF_GPIO_PIN_NOPULL);
//	nrf_gpio_pin_dir_set(29, NRF_GPIO_PIN_DIR_INPUT);
	nrf_gpio_cfg_input(STATUS_SW2,NRF_GPIO_PIN_NOPULL);
	
	nrf_gpio_pin_dir_set(STATUS_SW2, NRF_GPIO_PIN_DIR_INPUT);
	nrf_gpio_pin_dir_set(STATUS_SW1, NRF_GPIO_PIN_DIR_INPUT);
	nrf_gpio_pin_dir_set(MOTION_DETECT_PIN_NUMBER, NRF_GPIO_PIN_DIR_INPUT);
	nrf_gpio_pin_dir_set(FACTORY_RESET, NRF_GPIO_PIN_DIR_INPUT);
}
	

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Activities need to perform at one second completion
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void app_timer_activity(void)
{
    uint8_t i,status;
	  uint32_t uclcl_i;
	
		for(i=0;i<MAX_APP_TIMER;i++)
		{
			switch(i)
			{		
			case FACT_RST_TIMER: 
				if(sl_app_timer[FACT_RST_TIMER]!=0)
				{
					sl_app_timer[FACT_RST_TIMER]--;
					status = nrf_gpio_pin_read(FACTORY_RESET);								
					if(sl_app_timer[FACT_RST_TIMER]==0)
					{
						if(status==LOW)
						{
							#ifdef RESET_DBG
							simple_uart_putstring("\r\n**Fact rst str**");
							#endif											

							if ( sl_conn_state == TRUE )
							{
								#ifdef RESET_DBG
								simple_uart_putstring("\r\n**Disconnect**");
								#endif
								sl_host_disconnection();	
							}
							else
							{

//								if ( Adv_Status == TRUE )    //[KC]
//								{
//									advertising_stop();					
//									for ( uclcl_i = 0; uclcl_i < 500000; uclcl_i++ )
//									{

//									}
//								}

								//ble_sl_app_stop(); 								// Stop all smart lock functionality before disabling the SoftDevice
								//ble_stack_stop();				  				// Disable the S110 stack
								all_led_on();
								for ( uclcl_i = 0; uclcl_i < 500000; uclcl_i++ )
								{

								}						
								all_led_off();
								for ( uclcl_i = 0; uclcl_i < 500000; uclcl_i++ )
								{

								}						
								all_led_on();
								for ( uclcl_i = 0; uclcl_i < 500000; uclcl_i++ )
								{

								}						
								all_led_off();
								nrf_delay_us(100000);
								nrf_delay_us(100000);
	
								#ifdef RESET_DBG
								simple_uart_putstring("\r\n**Fact all led**");
								#endif
								sl_factory_reset();
								sd_power_gpregret_clr(0xFF);
								//								nrf_delay_us(100000);								// uSec	
								//								NVIC_SystemReset();
								sl_app_timer[SYSTEM_RESET_TIMER]=SYSTEM_RESET_TIME;
							}

						}

					}
					else
					{
						if(status==HIGH)
						{							
							sl_app_timer[FACT_RST_TIMER]=0;		// If PIN is released in between													
						}
					}
				}
			break;
					
					
					
					
					case SOLENOID_OFF_TIMER: 
								if(sl_app_timer[SOLENOID_OFF_TIMER]!=0)
								{
										--sl_app_timer[SOLENOID_OFF_TIMER];
										if(sl_app_timer[SOLENOID_OFF_TIMER]==0)
										{
											nrf_gpio_pin_clear(SOLENOID_CTR_PIN_NUMBER);
											green_led_off();
											solenoid_status = false;
										}
								}
								break;
						
					case MOTOR_OFF_TIMER: 
								if(sl_app_timer[MOTOR_OFF_TIMER]!=0)
								{
										--sl_app_timer[MOTOR_OFF_TIMER];
										if(sl_app_timer[MOTOR_OFF_TIMER]==0)
										{
											motor_stop();
											green_led_off();
											if(operation_type == MOTOR_FW)
											{
													dev_kit_op_status[0]=MOTOR;
													dev_kit_op_status[1]=0;	
													if(ant_request == NO_ANT_MSG)
													{
														ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
													}
													
											}
											else if(operation_type == MOTOR_RW)
											{
													dev_kit_op_status[0]=MOTOR;
													dev_kit_op_status[1]=1;		
													if(ant_request == NO_ANT_MSG)
													{														
														ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
													}
											}
										}
								}
								if(ant_request == ANT_MSG_ACK_CMPLTD)
								{
									ant_request = NO_ANT_MSG;
								}
								break;
								
					case RELAY_OFF_TIMER: 
								if(sl_app_timer[RELAY_OFF_TIMER]!=0)
								{
										--sl_app_timer[RELAY_OFF_TIMER];
										if(sl_app_timer[RELAY_OFF_TIMER]==0)
										{
											nrf_gpio_pin_clear(RELAY_1_CTR_PIN_NUMBER);
											green_led_off();
										}
								}
								break;
						
					case TEMP_SCAN_TIMER: 
								if(sl_app_timer[TEMP_SCAN_TIMER]!=0)
								{
										--sl_app_timer[TEMP_SCAN_TIMER];
										if(sl_app_timer[TEMP_SCAN_TIMER]==0)
										{
											sl_app_timer[TEMP_SCAN_TIMER]=TEMP_SCAN_TIME;
											temperature_notify_flag=true;
										}
								}	
								break;
								
					case ACCE_UPDATE_TIMER:
								if(sl_app_timer[ACCE_UPDATE_TIMER]!=0)
								{
										--sl_app_timer[ACCE_UPDATE_TIMER];
										if(sl_app_timer[ACCE_UPDATE_TIMER]==0)
										{
											accelerometer_update_flg=true;
											sl_app_timer[ACCE_UPDATE_TIMER]=ACCE_UPDATE_TIME;											
										}
									}
								
								break;
						
					case MOTION_UPDATE_TIMER:
								if(sl_app_timer[MOTION_UPDATE_TIMER]!=0)
								{
										--sl_app_timer[MOTION_UPDATE_TIMER];
										if(sl_app_timer[MOTION_UPDATE_TIMER]==0)
										{
											motion_update_flg = true;
											sl_app_timer[MOTION_UPDATE_TIMER] = MOTION_UPDATE_TIME;
										}
								}
								break;
					case CO2_SENSOR_UPDATE_TIMER:
								if(sl_app_timer[CO2_SENSOR_UPDATE_TIMER]!=0)
								{
										--sl_app_timer[CO2_SENSOR_UPDATE_TIMER];
										if(sl_app_timer[CO2_SENSOR_UPDATE_TIMER]==0)
										{
											co2_notify_flag=true;
											sl_app_timer[CO2_SENSOR_UPDATE_TIMER]=CO2_SENSOR_UPDATE_TIME;
										}
								}
								break;
								
					case WATCH_DOG_TIMER: 
								//nrf_gpio_pin_toggle(WDT_PIN_NO);
								break;
						
					case ONE_MIN_TIMER:
								if(sl_app_timer[ONE_MIN_TIMER]!=0)
								{
										--sl_app_timer[ONE_MIN_TIMER];
										if(sl_app_timer[ONE_MIN_TIMER]==0)
										{
											sl_app_timer[ONE_MIN_TIMER] = ONE_MIN_TIME;
											one_min_timer_flg=true;											
										}
								}
							  break;
								
					case THERMOSTAT_TRG_TIMER:
								if(sl_app_timer[THERMOSTAT_TRG_TIMER]!=0)
								{
										--sl_app_timer[THERMOSTAT_TRG_TIMER];
										if(sl_app_timer[THERMOSTAT_TRG_TIMER]==0)
										{
											nrf_gpio_pin_clear(RELAY_1_CTR_PIN_NUMBER);							//Heting
											nrf_gpio_pin_clear(RELAY_2_CTR_PIN_NUMBER); 						//AC(Cooling)
											green_led_off();
										}
								}
							  break;
								
					case HUMIDITY_TEMP_TIMER:
								if(sl_app_timer[HUMIDITY_TEMP_TIMER]!=0)
								{
									--sl_app_timer[HUMIDITY_TEMP_TIMER];
									if(sl_app_timer[HUMIDITY_TEMP_TIMER]==0)
									{
										sl_app_timer[HUMIDITY_TEMP_TIMER] = HUMIDITY_TEMP_TIME;
										humidity_temperature_update_flg=true;
									}
								}
							  break;
								
					case BAT_VOLT_UPDATE_TIMER:
								if(sl_app_timer[BAT_VOLT_UPDATE_TIMER]!=0)
								{
									--sl_app_timer[BAT_VOLT_UPDATE_TIMER];
									if(sl_app_timer[BAT_VOLT_UPDATE_TIMER]==0)
									{
										sl_app_timer[BAT_VOLT_UPDATE_TIMER] = BAT_VOLT_UPDATE_TIME;
										bat_voltage_update_flg = true;
									}
								}
							  break;
								
					case HW_INFO_UPDATE_TIMER:
								if(sl_app_timer[HW_INFO_UPDATE_TIMER]!=0)
								{
									--sl_app_timer[HW_INFO_UPDATE_TIMER];
									if(sl_app_timer[HW_INFO_UPDATE_TIMER]==0)
									{
										//hw_info_update_flg = true;
										Hardware_config_flg = false;
										//NVIC_SystemReset();											
									}
								}
								break;		
								
					case SYSTEM_RESET_TIMER:
								if(sl_app_timer[SYSTEM_RESET_TIMER]!=0)
								{
									if(--sl_app_timer[SYSTEM_RESET_TIMER]==0)
									{
										NVIC_SystemReset();
									}
								}
								break;
								
					case UART_ACT_TIMER: 
								if(sl_app_timer[UART_ACT_TIMER]!=0)
								{
									if(--sl_app_timer[UART_ACT_TIMER]==0)
									{
										#ifndef KEEP_UART_ACTIVATED											
											action_uart_deactivate();
										#endif
									}
								}
							break;
				}
		}
		
	if(sl_rcvtimeout!=0)
	{
		if(--sl_rcvtimeout==0)
		{
			sl_rcvmsg_ok = TRUE;	    // Just set flag and call routine from while(1)		
		}
	}
		
		if(device_type == SW_STATUS)
		{
			if((sw_previous_status == HIGH) && ((nrf_gpio_pin_read(STATUS_SW2) == LOW)))
			{
				sw_previous_status = LOW;
				dev_kit_op_status[0]=SW_STATUS;
				dev_kit_op_status[1]=1;					
				ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
			}
			else if((sw_previous_status == LOW) && ((nrf_gpio_pin_read(STATUS_SW2) == HIGH)))
			{
				sw_previous_status = HIGH;
				dev_kit_op_status[0]=SW_STATUS;
				dev_kit_op_status[1]=0;					
				ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
			}
					
		}		
		//check motion in sensor is detected or not
	  if((!nrf_gpio_pin_read(MOTION_DETECT_PIN_NUMBER)) && motion_detect_flg)
		{
			dev_kit_op_status[0]=MOTION_SENSOR;
			dev_kit_op_status[1]=MOTION_REMOVED;					
			ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
			motion_detect_flg=false;			
		}
		
		//check water level is detected or not
	  if((!nrf_gpio_pin_read(WATER_SENSOR_PIN_NUMBER)) && water_sensor_flg)
		{
			dev_kit_op_status[0]=WATER_SENSOR;
			dev_kit_op_status[1]=WATER_LEVEL_NOTDETETED;					
			ble_mtr_operation_status_update(&m_sr, &dev_kit_op_status[0], OP_STATUS_BUFF_LEN);
			water_sensor_flg=false;			
		}

}
