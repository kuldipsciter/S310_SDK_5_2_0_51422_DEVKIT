/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
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

#include "ble_debug_assert_handler.h"
#include <string.h>
#include <stdio.h>
#include "simple_uart.h"
#include "nrf51.h"
#include "ble_error_log.h"
#include "nordic_common.h"
#include "app_util.h"
#include "devkit\devkit_config.h"
#include "main.h"

#define MAX_LENGTH_FILENAME 128     /**< Max length of filename to copy for the debug error handlier. */

extern char s8g_buffer[BLE_DBG_BUF_SIZE];

// WARNING - DO NOT USE THIS FUNCTION IN END PRODUCT. - WARNING
// WARNING -         FOR DEBUG PURPOSES ONLY.         - WARNING
void ble_debug_assert_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // Copying parameters to static variables because parameters may not be accessible in debugger.
    static volatile uint8_t  s_file_name[MAX_LENGTH_FILENAME];
    static volatile uint16_t s_line_num;
    static volatile uint32_t s_error_code;   
		
	uint16_t lu16_delay;
        
    strncpy((char *)s_file_name, (const char *)p_file_name, MAX_LENGTH_FILENAME - 1);
    s_file_name[MAX_LENGTH_FILENAME - 1] = '\0';
    s_line_num                           = line_num;
    s_error_code                         = error_code;
    UNUSED_VARIABLE(s_file_name);
    UNUSED_VARIABLE(s_line_num);
    UNUSED_VARIABLE(s_error_code);    

    // WARNING: The PRIMASK register is set to disable ALL interrups during writing the error log.
    // 
    // Do not use __disable_irq() in normal operation.
    __disable_irq();

    // This function will write error code, filename, and line number to the flash.
    // In addition, the Cortex-M0 stack memory will also be written to the flash.
    //(void) ble_error_log_write(error_code, p_file_name, line_num);

	sprintf(s8g_buffer, "\r\nerror_code = %d  %s %d", error_code,  p_file_name, line_num);
	simple_uart_putstring((const uint8_t *)s8g_buffer);			
		
    // For debug purposes, this function never returns.
    // Attach a debugger for tracing the error cause.
    for (lu16_delay=0;lu16_delay<60000;lu16_delay++)
    {
        // Do nothing.
    }
}

