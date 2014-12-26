#include "AntAsyncEvent.h"

extern uint8_t STATUS_SW2;
extern uint8_t gu8_AntAsyncStatus;

uint8_t Ant_sw_prvs_status = HIGH;
uint8_t Ant_motion_prvs_status = LOW;
uint8_t Ant_water_prvs_status = LOW;

//  ANT ASCHYNC EVENTS HANDLING
void AntAschyncEvent(void)
{
//SWITCH
	if((Ant_sw_prvs_status == HIGH) && ((nrf_gpio_pin_read(STATUS_SW2) == LOW)))
	{
		DEBUG((const unsigned char*)"\r\n Switch pressed \n\r");
		Ant_sw_prvs_status = LOW;
		gu8_AntAsyncStatus = SWITCH_PRESSED;
		b.mu8_AntSendAsyncMsgToMaster=1;
	}
	else if((Ant_sw_prvs_status == LOW) && ((nrf_gpio_pin_read(STATUS_SW2) == HIGH)))
	{
		DEBUG((const unsigned char*)"\r\n Switch released \n\r");
		Ant_sw_prvs_status = HIGH;
		gu8_AntAsyncStatus = SWITCH_RELEASED;
		b.mu8_AntSendAsyncMsgToMaster=1;	
	}
//MOTION
//	if((Ant_motion_prvs_status == LOW) && ((nrf_gpio_pin_read(MOTION_DETECT_PIN_NUMBER) == HIGH)))
//	{
//		DEBUG((const unsigned char*)"\r\n Motion Detected \n\r");
//		Ant_motion_prvs_status = HIGH;
//		gu8_AntAsyncStatus = MOTION_DETECTED;
//		b.mu8_AntSendAsyncMsgToMaster=1;
//	}
//	else if((Ant_motion_prvs_status == HIGH) && ((nrf_gpio_pin_read(MOTION_DETECT_PIN_NUMBER) == LOW)))
//	{
//		DEBUG((const unsigned char*)"\r\n Motion Stoped \n\r");
//		Ant_motion_prvs_status = LOW;
//		gu8_AntAsyncStatus = MOTION_STOP;
//		b.mu8_AntSendAsyncMsgToMaster=1;
//	}
//WATER
	if((Ant_water_prvs_status == HIGH) && ((nrf_gpio_pin_read(WATER_SENSOR_PIN_NUMBER) == LOW)))
	{
		DEBUG((const unsigned char*)"\r\n Water not detected \n\r");
		Ant_water_prvs_status = LOW;
		gu8_AntAsyncStatus = WATER_NOT_DETECTED;
		b.mu8_AntSendAsyncMsgToMaster=1;
	
	}
	else if((Ant_water_prvs_status == LOW) && ((nrf_gpio_pin_read(WATER_SENSOR_PIN_NUMBER) == HIGH)))
	{
		DEBUG((const unsigned char*)"\r\n Water detected \n\r");
	    Ant_water_prvs_status = HIGH;
		gu8_AntAsyncStatus = WATER_DETECTED;
		b.mu8_AntSendAsyncMsgToMaster=1;
	}
}
