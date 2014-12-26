/*
 * * AES (Rijndael) cipher
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

#ifndef AES_I_H
#define AES_I_H
	
#include "aes.h"
	
/* #define FULL_UNROLL */
#define AES_SMALL_TABLES

extern const unsigned long Te0[256];
extern const unsigned long Te1[256];
extern const unsigned long Te2[256];
extern const unsigned long Te3[256];
extern const unsigned long Te4[256];

extern const unsigned long Td0[256];
extern const unsigned long Td1[256];
extern const unsigned long Td2[256];
extern const unsigned long Td3[256];
extern const unsigned long Td4[256];
extern const unsigned long rcon[10];
extern const unsigned char Td4s[256];
extern const unsigned char rcons[10];

#ifndef AES_SMALL_TABLES	
    #define RCON(i) rcon[(i)]

    #define TE0(i) Te0[((i) >> 24) & 0xff]
    #define TE1(i) Te1[((i) >> 16) & 0xff]
    #define TE2(i) Te2[((i) >> 8) & 0xff]
    #define TE3(i) Te3[(i) & 0xff]
    #define TE41(i) (Te4[((i) >> 24) & 0xff] & 0xff000000)
    #define TE42(i) (Te4[((i) >> 16) & 0xff] & 0x00ff0000)
    #define TE43(i) (Te4[((i) >> 8) & 0xff] & 0x0000ff00)
    #define TE44(i) (Te4[(i) & 0xff] & 0x000000ff)
    #define TE421(i) (Te4[((i) >> 16) & 0xff] & 0xff000000)
    #define TE432(i) (Te4[((i) >> 8) & 0xff] & 0x00ff0000)
    #define TE443(i) (Te4[(i) & 0xff] & 0x0000ff00)
    #define TE414(i) (Te4[((i) >> 24) & 0xff] & 0x000000ff)
    #define TE4(i) (Te4[(i)] & 0x000000ff)

    #define TD0(i) Td0[((i) >> 24) & 0xff]
    #define TD1(i) Td1[((i) >> 16) & 0xff]
    #define TD2(i) Td2[((i) >> 8) & 0xff]
    #define TD3(i) Td3[(i) & 0xff]
    #define TD41(i) (Td4[((i) >> 24) & 0xff] & 0xff000000)
    #define TD42(i) (Td4[((i) >> 16) & 0xff] & 0x00ff0000)
    #define TD43(i) (Td4[((i) >> 8) & 0xff] & 0x0000ff00)
    #define TD44(i) (Td4[(i) & 0xff] & 0x000000ff)
    #define TD0_(i) Td0[(i) & 0xff]
    #define TD1_(i) Td1[(i) & 0xff]
    #define TD2_(i) Td2[(i) & 0xff]
    #define TD3_(i) Td3[(i) & 0xff]
#else /* AES_SMALL_TABLES */	
    #define RCON(i) ((unsigned long) rcons[(i)] << 24)

    static inline unsigned long rotr(unsigned long val, int bits)
    {
        return (val >> bits) | (val << (32 - bits));
    }

    #define TE0(i) Te0[((i) >> 24) & 0xff]
    #define TE1(i) rotr((unsigned long) Te0[((i) >> 16) & 0xff], 8)
    #define TE2(i) rotr((unsigned long) Te0[((i) >> 8) & 0xff], 16)
    #define TE3(i) rotr((unsigned long) Te0[(i) & 0xff], 24)
    
    #define TE41(i) ((unsigned long) (Te0[((i) >> 24) & 0xff] << 8) & 0xff000000)
    #define TE42(i) ((unsigned long) Te0[((i) >> 16) & 0xff] & 0x00ff0000)
    #define TE43(i) ((unsigned long) Te0[((i) >> 8) & 0xff] & 0x0000ff00)
    #define TE44(i) ((unsigned long) (Te0[(i) & 0xff] >> 8) & 0x000000ff)
    #define TE421(i) ((unsigned long) (Te0[((i) >> 16) & 0xff] << 8) & 0xff000000)
    #define TE432(i) ((unsigned long) Te0[((i) >> 8) & 0xff] & 0x00ff0000)
    #define TE443(i) ((unsigned long) Te0[(i) & 0xff] & 0x0000ff00)
    #define TE414(i) ((unsigned long) (Te0[((i) >> 24) & 0xff] >> 8) & 0x000000ff)
    #define TE4(i) ((unsigned long) (Te0[(i)] >> 8) & 0x000000ff)

    #define TD0(i) Td0[((i) >> 24) & 0xff]
    #define TD1(i) rotr((unsigned long) Td0[((i) >> 16) & 0xff], 8)
    #define TD2(i) rotr((unsigned long) Td0[((i) >> 8) & 0xff], 16)
    #define TD3(i) rotr((unsigned long) Td0[(i) & 0xff], 24)
    #define TD41(i) ((unsigned long) Td4s[((i) >> 24) & 0xff] << 24)
    #define TD42(i) ((unsigned long) Td4s[((i) >> 16) & 0xff] << 16)
    #define TD43(i) ((unsigned long) Td4s[((i) >> 8) & 0xff] << 8)
    #define TD44(i) ((unsigned long) Td4s[(i) & 0xff])
    #define TD0_(i) Td0[(i) & 0xff]
    #define TD1_(i) rotr((unsigned long) Td0[(i) & 0xff], 8)
    #define TD2_(i) rotr((unsigned long) Td0[(i) & 0xff], 16)
    #define TD3_(i) rotr((unsigned long) Td0[(i) & 0xff], 24)		
#endif /* AES_SMALL_TABLES */
	
#ifdef _MSC_VER
    #define SWAP(x) (_lrotl(x, 8) & 0x00ff00ff | _lrotr(x, 8) & 0xff00ff00)
    #define GETU32(p) SWAP(*((unsigned long *)(p)))
    #define PUTU32(ct, st) { *((unsigned long *)(ct)) = SWAP((st)); }
#else
    #define GETU32(pt) (((unsigned long)(pt)[0] << 24) ^ ((unsigned long)(pt)[1] << 16) ^ \
        ((unsigned long)(pt)[2] <<  8) ^ ((unsigned long)(pt)[3]))
    #define PUTU32(ct, st) { \
        (ct)[0] = (unsigned char)((st) >> 24); (ct)[1] = (unsigned char)((st) >> 16); \
        (ct)[2] = (unsigned char)((st) >>  8); (ct)[3] = (unsigned char)(st); }
#endif
	
#define AES_PRIV_SIZE (4 * 44)

void rijndaelKeySetupEnc(unsigned long rk[], const unsigned char cipherKey[]);
	
#endif /* AES_I_H */

