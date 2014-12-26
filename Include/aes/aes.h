/*
 * * AES functions
 * * Copyright (c) 2003-2006, Jouni Malinen <j@w1.fi>
 * *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 as
 * * published by the Free Software Foundation.
 * *
 * * Alternatively, this software may be distributed under the terms of BSD
 * * license.
 * *
 * * See README and COPYING for more details.
 * */

#ifndef AES_H
#define AES_H

#include <stdlib.h>
#include "aes/common.h"



#define AES_BLOCK_SIZE 16

void * aes_encrypt_init(const unsigned char *key, size_t len);
void aes_encrypt(void *ctx, const unsigned char *plain, unsigned char *crypt);
void aes_encrypt_deinit(void *ctx);

void * aes_decrypt_init(const unsigned char *key, size_t len);
void aes_decrypt(void *ctx, const unsigned char *crypt, unsigned char *plain);
void aes_decrypt_deinit(void *ctx);

int aes_128_cbc_encrypt	(const unsigned char *, const unsigned char *, unsigned char *, short);	
int aes_128_cbc_decrypt(const unsigned char *, const unsigned char *, unsigned char *, short );
#endif /* AES_H */

