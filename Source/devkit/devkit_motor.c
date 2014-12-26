#include <string.h>
#include <stdio.h>
#include "simple_uart.h"
#include "ble_srv_common.h"
#include "ble_dis.h"
#include "devkit\devkit_motor.h"
#include "boards/ras_1_6.h"
#include "ble_sr.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "led.h"
#include "main.h"
#include "battery.h"

/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/
extern uint32_t DEVKIT_MOTOR_OP_POS_PIN,DEVKIT_MOTOR_OP_NEG_PIN;


/**@brief run motor in forward direction
 *
 */
void motor_forward(void)
{
	nrf_gpio_pin_set(DEVKIT_MOTOR_OP_POS_PIN);
	nrf_gpio_pin_clear(DEVKIT_MOTOR_OP_NEG_PIN);
}
/**@brief run motor in reverse direction
 *
 */
void motor_reverse(void)
{
	nrf_gpio_pin_clear(DEVKIT_MOTOR_OP_POS_PIN);
	nrf_gpio_pin_set(DEVKIT_MOTOR_OP_NEG_PIN);
}
/**@brief stop motor
 *
 */
void motor_stop(void)
{
	nrf_gpio_pin_clear(DEVKIT_MOTOR_OP_POS_PIN);
	nrf_gpio_pin_clear(DEVKIT_MOTOR_OP_NEG_PIN);
}

/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/


