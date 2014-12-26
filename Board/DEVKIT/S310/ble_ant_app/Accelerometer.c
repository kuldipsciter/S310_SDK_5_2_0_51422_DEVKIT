#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble_bas.h"
#include "main.h"
#include "app_util.h"
//#include "twi_master.h"
#include "twi_master_int.h"
#include "Accelerometer.h"
#include "simple_uart.h"

uint8_t TWI_TXData[30];
uint8_t TWI_RxData[30];

uint8_t TWI_TXAddr[2];

extern bool timeout_flg;
extern volatile bool twi_operation_complete, twi_ack_received, twi_blocking_enabled;
uint8_t i=0;


void Initialize_Accelerometer(void)
{
  //////////////////////// Initialization Start //////////////////////////////////////////////
	
	TWI_TXAddr[0] = (START_CONDITION);// << 1);

	TWI_TXData[0] = MODE_REG_ADDRESS;
	TWI_TXData[1] = 0x00;     // MODE register Stand by mode 

	//twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2,true);
	
	twi_master_write(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2);
	
	while(!twi_operation_complete);
	
	TWI_TXData[0] = SPCNT_REG_ADDRESS;
	TWI_TXData[1] = 0x05;     // SPCNT register ( Sleep count zero )
	
//	twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2,true);
	twi_master_write(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2);
	
	TWI_TXData[0] = INTSU_REG_ADDRESS;
	TWI_TXData[1] = 0x00;     // INTSU register ( No interrupts enabled ) 

	while(!twi_operation_complete);
//	twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2,true);
	twi_master_write(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2);

	TWI_TXData[0] = PDET_REG_ADDRESS;
	TWI_TXData[1] = 0xE0;     // PDET register ( Tap detection disabled )

	while(!twi_operation_complete);
//	twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2,true);
	twi_master_write(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2);
	
	TWI_TXData[0] = SR_REG_ADDRESS;
	TWI_TXData[1] = 0x00;     // SR register ( 8 samples per second and Tap detection disabled )

	while(!twi_operation_complete);
//	twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2,true);
	twi_master_write(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2);
	
	TWI_TXData[0] = PD_REG_ADDRESS;
	TWI_TXData[1] = 0x00;     // PD register ( No Tap detection debounce count enabled  )

	while(!twi_operation_complete);
//	twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2,true);
	twi_master_write(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2);

	TWI_TXData[0] = MODE_REG_ADDRESS;
	TWI_TXData[1] = 0x01;     // MODE register ( Active mode )

	while(!twi_operation_complete);
//	twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2,true);
	twi_master_write(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],2);
	
	while(!twi_operation_complete);
  //////////////////////// Initialization End //////////////////////////////////////////////
	
}

void Humidity_sensor_data(void)
{
	TWI_TXAddr[0] = HUMIDITY_SENSOR_ADD;

	TWI_TXData[0] = HUMIDITY_REG_ADDRESS_HOLD;

	twi_master_write_read(TWI_TXAddr[0], (uint8_t *)&TWI_TXData[0], 1, (uint8_t *)&TWI_RxData[0], 2); 
}

void Temperature_sensor_data(void)
{
	TWI_TXAddr[0] = HUMIDITY_SENSOR_ADD;

	TWI_TXData[0] = TEMPERATURE_REG_ADDRESS_HOLD;

	twi_master_write_read(TWI_TXAddr[0], (uint8_t *)&TWI_TXData[0], 1, (uint8_t *)&TWI_RxData[0], 2);
}


void Read_Coordinate_Data(void)
{	
	//------------------ X OUT data -----------------------------//
	
	TWI_TXAddr[0] = (START_CONDITION);

	TWI_TXData[0] = XOUT_REG_ADDRESS;

	
	twi_master_write_read(TWI_TXAddr[0], (uint8_t *)&TWI_TXData[0], 1,(uint8_t *)&TWI_RxData[0], 1);
	while(!twi_operation_complete)
	{
	}
	//------------------ Y OUT data -----------------------------//
	
	while(!twi_operation_complete)
	{
	}
	
	TWI_TXAddr[0] = (START_CONDITION);

	TWI_TXData[0] = YOUT_REG_ADDRESS;
	
		
	twi_master_write_read(TWI_TXAddr[0], (uint8_t *)&TWI_TXData[0], 1,(uint8_t *)&TWI_RxData[1], 1);
	while(!twi_operation_complete)
	{
	}
	
	//------------------ Z OUT data -----------------------------//
	while(!twi_operation_complete)
	{
	}

	TWI_TXAddr[0] = (START_CONDITION);

	TWI_TXData[0] = ZOUT_REG_ADDRESS;

	twi_master_write_read(TWI_TXAddr[0], (uint8_t *)&TWI_TXData[0], 1,(uint8_t *)&TWI_RxData[2], 1);
	while(!twi_operation_complete)
	{
	}
}

void Tilt_Reg_Data(void)
{/*
	
	TWI_TXAddr[0] = (START_CONDITION << 1);

	TWI_TXData[0] = TILT_REG_ADDRESS;

	twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_TXData[0],1,false);

  TWI_TXAddr[0] = ( (START_CONDITION << 1) | 0x01 );

	twi_master_transfer(TWI_TXAddr[0],(uint8_t *)&TWI_RxData[3],1,true);
		*/
}
