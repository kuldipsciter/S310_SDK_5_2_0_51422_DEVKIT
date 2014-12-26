/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 */

#ifndef __AES_CBC_ENC_DEC_H__
#define __AES_CBC_ENC_DEC_H__
#include <stdint.h>

#define   BYTES         33

uint8_t ekey[16] = {0x27,0xec,0x2f,0x24,0xef,0x7d,0xe1,0xc2,0xa8,0x2e,0xe0,0x68,0x45,0xe6,0x29,0xd1};
//uint8_t ekey[16] = {0x27,0xec,0x2b,0x24,0xef,0x7d,0xe1,0x2d,0xa8,0x2e,0x05,0x68,0x45,0xe6,0x29,0xb2};
uint8_t iv[16] = {0X00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};
uint8_t dumy_data[32]={0};

//*****************************************************************************
//
// Define string for sending request to server
//
//*****************************************************************************


static const char g_Test_5008[] =                          
{
    //"I am kd. This is bipin. how r u? "
	"Sciter Technology Private Limited"
};


//*****************************************************************************
//
// This is prototype defination.
//
//*****************************************************************************
void ENCDEC(void);


#endif /* __AES-CBC_ENC_DEC_H__ */
