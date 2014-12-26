/*
* AES-128 CBC
*
* Copyright (c) 2003-2007, Jouni Malinen <j@w1.fi>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* Alternatively, this software may be distributed under the terms of BSD
* license.
*
* See README and COPYING for more details.
*/

//Ref: http://docs.ros.org/fuerte/api/wpa_supplicant_node/html/aes-cbc_8c_source.html

#include "aes\aes.h"

int aes_128_cbc_encrypt(const unsigned char *key, const unsigned char *iv, unsigned char *data, short data_len)
{
    void                        *ctx;
    unsigned char               cbc[AES_BLOCK_SIZE];
    unsigned char               *pos = data;
    int                         i, j, blocks;
 
    ctx = aes_encrypt_init(key, 16);
    if(ctx == NULL)
        return -1;    
    
    for(i=0; i<AES_BLOCK_SIZE; i++)
        cbc[i] = *iv++;
 
    blocks = data_len / AES_BLOCK_SIZE;
    
    for (i=0; i<blocks; i++) 
    {
        for(j=0; j<AES_BLOCK_SIZE; j++)
            cbc[j] ^= pos[j];
        
        aes_encrypt(ctx, cbc, cbc);
          
        for(j=0; j<AES_BLOCK_SIZE; j++)
            pos[j] = cbc[j];
        
        pos += AES_BLOCK_SIZE;
    }
    
    aes_encrypt_deinit(ctx);
    
    return 0;
}



int aes_128_cbc_decrypt(const unsigned char *key, const unsigned char *iv, unsigned char *data, short data_len)
{
    void                        *ctx;
    unsigned char                          cbc[AES_BLOCK_SIZE], tmp[AES_BLOCK_SIZE];
    unsigned char                          *pos = data;
    unsigned char                          *first_pos;
    int                         i, j, blocks;

    ctx = aes_decrypt_init(key, 16);
    if (ctx == NULL)
        return -1;
    
    for(i=0; i<AES_BLOCK_SIZE; i++)
        cbc[i] = *iv++;

    blocks = data_len / AES_BLOCK_SIZE;
    
    for(i=0; i<blocks; i++)
    {
        first_pos = pos;
        for(j=0; j<AES_BLOCK_SIZE; j++)
            tmp[j] = *pos++;
        pos = first_pos; 

        aes_decrypt(ctx, pos, pos);
        
        for(j=0; j<AES_BLOCK_SIZE; j++)
            pos[j] ^= cbc[j];
        
        for(j=0; j<AES_BLOCK_SIZE; j++)
            cbc[j] = tmp[j];

        pos += AES_BLOCK_SIZE;
    }

    aes_decrypt_deinit(ctx);
    
    return 0;
}

