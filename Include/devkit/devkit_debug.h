 /* Copyright (c) 2013 Belwith Products. All Rights Reserved.
 *
 */

#ifndef DEVKIT_DEBUG_H
#define DEVKIT_DEBUG_H
#include <string.h>
#include <stdio.h>
#include "simple_uart.h"

void gen_PrintRcvData(uint8_t * p_event_message_buffer);
void gen_PrintHex(uint8_t adata);
void gen_PrintDec(uint8_t adata);
void gen_PrintHexStr(uint8_t *str, uint8_t NoOfByte);
void gen_PrintCharStr(uint8_t *str, uint8_t NoOfByte);

/** display permenanent and dynamic data

*/
void display_config_data(void);

#endif
