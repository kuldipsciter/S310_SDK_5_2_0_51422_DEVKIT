/*
 * * AES (Rijndael) cipher - decrypt
 * *
 * * Modifications to public domain implementation:
 * * - support only 128-bit keys
 * * - cleanup
 * * - use C pre-processor to make it easier to change S table access
 * * - added option (AES_SMALL_TABLES) for reducing code size by about 8 kB at
 * *   cost of reduced throughput (quite small difference on Pentium 4,
 * *   10-25% when using -O1 or -O2 optimization)
 * *
 * * Copyright (c) 2003-2005, Jouni Malinen <j@w1.fi>
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

#include "aes/common.h"
#include <stdlib.h>
#include "aes/aes_i.h"


unsigned long s0, s1, s2, s3, t0, t1, t2, t3;


void rijndaelKeySetupDec(unsigned long rk[], const unsigned char cipherKey[])
{	
    int                         Nr = 10, i, j;
    unsigned long               temp;
	
    /* expand the cipher key: */
    rijndaelKeySetupEnc(rk, cipherKey);
    
    /* invert the order of the round keys: */
    for(i=0, j=4*Nr; i<j; i+=4, j-=4) 
    {
        temp = rk[i    ]; rk[i    ] = rk[j    ]; rk[j    ] = temp;
        temp = rk[i + 1]; rk[i + 1] = rk[j + 1]; rk[j + 1] = temp;
        temp = rk[i + 2]; rk[i + 2] = rk[j + 2]; rk[j + 2] = temp;
        temp = rk[i + 3]; rk[i + 3] = rk[j + 3]; rk[j + 3] = temp;
    }
    
    /* apply the inverse MixColumn transform to all round keys but the
     * first and the last: */
    for(i=1; i<Nr; i++) 
    {
        rk += 4;
        for(j=0; j<4; j++) 
        {
            rk[j] = TD0_(TE4((rk[j] >> 24)       )) ^
                    TD1_(TE4((rk[j] >> 16) & 0xff)) ^
                    TD2_(TE4((rk[j] >>  8) & 0xff)) ^
                    TD3_(TE4((rk[j]      ) & 0xff));
        }
    }
}

void * aes_encrypt_init(const unsigned char *key, size_t len)
{
    unsigned long               rk[AES_PRIV_SIZE];
    void                        *tmp;
    unsigned long               i;
    
    if (len != 16)
        return NULL;
    
    for(i=0; i<AES_PRIV_SIZE; i++)
        rk[i] = 0;
    
    rijndaelKeySetupEnc(rk, key);      
    tmp = rk;
    return tmp;
}

void * aes_decrypt_init(const unsigned char *key, size_t len)
{
    unsigned long                         rk[AES_PRIV_SIZE];
    void                        *tmp;
    unsigned long                         i;
	
    if(len != 16)
        return NULL;
    
    for(i=0; i<AES_PRIV_SIZE; i++)
        rk[i] = 0;
	
    rijndaelKeySetupDec(rk, key);
    tmp = rk;
    return tmp;
}


void rijndaelEncrypt(const unsigned long rk[/*44*/], const unsigned char pt[16], unsigned char ct[16])
{
    const int                   Nr = 10;
    #ifndef FULL_UNROLL
    int                         r;
    #endif /* ?FULL_UNROLL */

    /*
    * and add initial round key:
    */
    s0 = GETU32(pt     ) ^ rk[0];
    s1 = GETU32(pt +  4) ^ rk[1];
    s2 = GETU32(pt +  8) ^ rk[2];
    s3 = GETU32(pt + 12) ^ rk[3];
 
    #define ROUND(i,d,s) \
        d##0 = TE0(s##0) ^ TE1(s##1) ^ TE2(s##2) ^ TE3(s##3) ^ rk[4 * i]; \
        d##1 = TE0(s##1) ^ TE1(s##2) ^ TE2(s##3) ^ TE3(s##0) ^ rk[4 * i + 1]; \
        d##2 = TE0(s##2) ^ TE1(s##3) ^ TE2(s##0) ^ TE3(s##1) ^ rk[4 * i + 2]; \
        d##3 = TE0(s##3) ^ TE1(s##0) ^ TE2(s##1) ^ TE3(s##2) ^ rk[4 * i + 3]
 
        #ifdef FULL_UNROLL
            ROUND(1,t,s);
            ROUND(2,s,t);
            ROUND(3,t,s);
            ROUND(4,s,t);
            ROUND(5,t,s);
            ROUND(6,s,t);
            ROUND(7,t,s);
            ROUND(8,s,t);
            ROUND(9,t,s);
     
            rk += Nr << 2;
        #else /* !FULL_UNROLL */
              /* Nr - 1 full rounds: */
            r = Nr >> 1;
            for (;;) 
            {
                ROUND(1,t,s);
                
                rk += 8;
                if (--r == 0)
                    break;
                
                ROUND(0,s,t);
            }
        #endif /* ?FULL_UNROLL */
    #undef ROUND
            
    /*
    * apply last round and
    * map cipher state to byte array block:
    */
            
    s0 = (unsigned long) TE41(t0) ^ (unsigned long) TE42(t1) ^ (unsigned long) TE43(t2) ^ (unsigned long) TE44(t3) ^ (unsigned long) rk[0];
    PUTU32(ct     , s0);
    s1 = TE41(t1) ^ TE42(t2) ^ TE43(t3) ^ TE44(t0) ^ rk[1];
    PUTU32(ct +  4, s1);
    s2 = TE41(t2) ^ TE42(t3) ^ TE43(t0) ^ TE44(t1) ^ rk[2];
    PUTU32(ct +  8, s2);
    s3 = TE41(t3) ^ TE42(t0) ^ TE43(t1) ^ TE44(t2) ^ rk[3];
    PUTU32(ct + 12, s3);
}
            

static void rijndaelDecrypt(const unsigned long rk[], const unsigned char ct[16], unsigned char pt[16])
{
    const int                   Nr = 10;
    #ifndef FULL_UNROLL
    int                         r;
    #endif /* ?FULL_UNROLL */
	
    /*
    * * map byte array block to cipher state
    * * and add initial round key:
    * */
    s0 = GETU32(ct     ) ^ rk[0];
    s1 = GETU32(ct +  4) ^ rk[1];
    s2 = GETU32(ct +  8) ^ rk[2];
    s3 = GETU32(ct + 12) ^ rk[3];
	
    #define ROUND(i,d,s) \
        d##0 = TD0(s##0) ^ TD1(s##3) ^ TD2(s##2) ^ TD3(s##1) ^ rk[4 * i]; \
        d##1 = TD0(s##1) ^ TD1(s##0) ^ TD2(s##3) ^ TD3(s##2) ^ rk[4 * i + 1]; \
        d##2 = TD0(s##2) ^ TD1(s##1) ^ TD2(s##0) ^ TD3(s##3) ^ rk[4 * i + 2]; \
        d##3 = TD0(s##3) ^ TD1(s##2) ^ TD2(s##1) ^ TD3(s##0) ^ rk[4 * i + 3]


        #ifdef FULL_UNROLL    
            ROUND(1,t,s);
            ROUND(2,s,t);
            ROUND(3,t,s);
            ROUND(4,s,t);
            ROUND(5,t,s);
            ROUND(6,s,t);
            ROUND(7,t,s);
            ROUND(8,s,t);
            ROUND(9,t,s);

            rk += Nr << 2;	
        #else /* !FULL_UNROLL */		
              /* Nr - 1 full rounds: */
            r = Nr >> 1;
            for(;;) 
            {
                ROUND(1,t,s);

                rk += 8;
                if (--r == 0)
                    break;
                
                ROUND(0,s,t);
            }
        #endif /* ?FULL_UNROLL */    
    #undef ROUND
	
    /*
    * * apply last round and
    * * map cipher state to byte array block:
    * */

    s0 = (unsigned long) TD41(t0) ^ (unsigned long) TD42(t3) ^ (unsigned long) TD43(t2) ^ (unsigned long) TD44(t1) ^ (unsigned long) rk[0];
    PUTU32(pt     , s0);
    s1 = TD41(t1) ^ TD42(t0) ^ TD43(t3) ^ TD44(t2) ^ rk[1];
    PUTU32(pt +  4, s1);
    s2 = TD41(t2) ^ TD42(t1) ^ TD43(t0) ^ TD44(t3) ^ rk[2];
    PUTU32(pt +  8, s2);
    s3 = TD41(t3) ^ TD42(t2) ^ TD43(t1) ^ TD44(t0) ^ rk[3];
    PUTU32(pt + 12, s3);
}
	
void aes_encrypt(void *ctx, const unsigned char *plain, unsigned char *crypt)
{
    rijndaelEncrypt(ctx, plain, crypt);
}

void aes_decrypt(void *ctx, const unsigned char *crypt, unsigned char *plain)
{
    rijndaelDecrypt(ctx, crypt, plain);
}

void aes_encrypt_deinit(void *ctx)
{
    unsigned long                         i;
    unsigned long                         *tmp = (unsigned long*)ctx;
    
    for(i=0; i<AES_PRIV_SIZE; i++)
    {
        *tmp++ = 0;
    }
}

void aes_decrypt_deinit(void *ctx)
{
    unsigned long                         i;
    unsigned long                         *tmp = (unsigned long*)ctx;
    
    for(i=0; i<AES_PRIV_SIZE; i++)
    {
        *tmp++ = 0;
    }
}

