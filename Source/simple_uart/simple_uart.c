/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdint.h>

#include "nrf.h"
#include "simple_uart.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_util.h"
#include "devkit\devkit_constant.h"
#include "main.h"

#define TIME_BTWN_TWO_CHAR	2;			// 2 second

char 	rx_buf[RX_BUFFER_SIZE]={0};
uint8_t 	uart_state =	TRUE;
uint8_t 	sl_rcvcntr = 0;
uint8_t 	sl_rcvtimeout=0;
uint8_t 	sl_rcvmsg_ok=FALSE;
uint8_t   rx_msg_len=0;
uint8_t UartDataRxFlg = FALSE;
/*@brief Action to disable the UART
 */
void action_uart_deactivate(void)
{
		if(uart_state == TRUE)
		{
			NRF_UART0->TASKS_STOPTX = 1;
			NRF_UART0->TASKS_STOPRX = 1;
			NRF_UART0->ENABLE       = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);	

			NVIC_ClearPendingIRQ(UART0_IRQn);
			NVIC_DisableIRQ(UART0_IRQn);	
			NRF_UART0->INTENCLR 		= 0xffffffffUL;	// Disable All Interrupt

			uart_state = FALSE;	
		}
}
	
/*@brief Action to enable the UART
 */
void action_uart_activate(void)
{
	if(uart_state == FALSE)
	{	
		NRF_UART0->ENABLE       = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);	
		NRF_UART0->TASKS_STARTTX = 1;
		NRF_UART0->TASKS_STARTRX = 1;

		uart_state = TRUE;
	}
}

uint8_t simple_uart_get(void)
{
  while (NRF_UART0->EVENTS_RXDRDY != 1)
  {
    // Wait for RXD data to be received
  }
  
  NRF_UART0->EVENTS_RXDRDY = 0;
  return (uint8_t)NRF_UART0->RXD;
}

bool simple_uart_get_with_timeout(int32_t timeout_ms, uint8_t *rx_data)
{
  bool ret = true;
  
  while (NRF_UART0->EVENTS_RXDRDY != 1)
  {
    if (timeout_ms-- >= 0)
    {
      // wait in 1ms chunk before checking for status
      nrf_delay_us(1000);
    }
    else
    {
      ret = false;
      break;
    }
  }  // Wait for RXD data to be received

  if (timeout_ms >= 0)
  {
    // clear the event and set rx_data with received byte
      NRF_UART0->EVENTS_RXDRDY = 0;
      *rx_data = (uint8_t)NRF_UART0->RXD;
  }

  return ret;
}

void simple_uart_put(uint8_t cr)
{
  NRF_UART0->TXD = (uint8_t)cr;

  while (NRF_UART0->EVENTS_TXDRDY!=1)
  {
    // Wait for TXD data to be sent
  }

  NRF_UART0->EVENTS_TXDRDY=0;
}

void simple_uart_putstring(const uint8_t *str)
//void simple_uart_putstring(char *str)
{
	action_uart_activate();
  uint_fast8_t i = 0;
  uint8_t ch = str[i++];
  while (ch != '\0')
  {
    simple_uart_put(ch);
    ch = str[i++];
  }
}

void simple_uart_config(  uint8_t rts_pin_number,
                          uint8_t txd_pin_number,
                          uint8_t cts_pin_number,
                          uint8_t rxd_pin_number,
                          bool    hwfc)
{
	/** @snippet [Configure UART RX and TX pin] */
	nrf_gpio_cfg_output(txd_pin_number);
	nrf_gpio_cfg_input(rxd_pin_number, NRF_GPIO_PIN_NOPULL);  

	NRF_UART0->PSELTXD = txd_pin_number;
	NRF_UART0->PSELRXD = rxd_pin_number;
	/** @snippet [Configure UART RX and TX pin] */
	if (hwfc)
	{
		nrf_gpio_cfg_output(rts_pin_number);
		nrf_gpio_cfg_input(cts_pin_number, NRF_GPIO_PIN_NOPULL);
		NRF_UART0->PSELCTS = cts_pin_number;
		NRF_UART0->PSELRTS = rts_pin_number;
		NRF_UART0->CONFIG  = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
	}

  NRF_UART0->BAUDRATE         = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);
  NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
  NRF_UART0->TASKS_STARTTX    = 1;
  NRF_UART0->TASKS_STARTRX    = 1;
  NRF_UART0->EVENTS_RXDRDY    = 0;
	
	// Enable UART interrupt
	NRF_UART0->INTENCLR = 0xffffffffUL;
	NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
							(UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos);

	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_HIGH);
	NVIC_EnableIRQ(UART0_IRQn);
}

/**@brief UART Interrupt handler.
 *
 * @details UART interrupt handler to process RX Ready when a byte
 *          is received, or in case of error when receiving a byte.
 */
void UART0_IRQHandler(void)
{
	// Handle reception
	if (NRF_UART0->EVENTS_RXDRDY != 0)
	{
		// Clear UART RX event flag
		NRF_UART0->EVENTS_RXDRDY = 0;

		if(sl_rcvmsg_ok==FALSE)                           // Protect Against RxBuffer OverWrite 
		{
			sl_rcvtimeout = TIME_BTWN_TWO_CHAR;      			// Time out between two char
			rx_buf[sl_rcvcntr] = (uint8_t)NRF_UART0->RXD;	// Write received byte 
			sl_rcvcntr++;
			if(sl_rcvcntr==2)
			{
				rx_msg_len=(rx_buf[1]+(rx_buf[0]*256)); 
			}
			if(sl_rcvcntr>=3)
			{
				if(sl_rcvcntr == (rx_msg_len + 2))
				{
					sl_rcvmsg_ok = TRUE;	    // Just set flag and call routine from while(1)		
					sl_rcvtimeout = 0;
				}
			}
		}		
	}
	// Handle errors.
	if (NRF_UART0->EVENTS_ERROR != 0)
	{
		// Clear UART ERROR event flag.
		NRF_UART0->EVENTS_ERROR = 0;
	}
}
//------------------------------------------------------------------------------
// Calculate CRC of Data
//------------------------------------------------------------------------------
void sl_xmit_data(char *str,uint8_t num)
{
	action_uart_activate();
	uint8_t i = 0;
	while (i<num)
	{
		simple_uart_put(str[i++]);
	}
}
