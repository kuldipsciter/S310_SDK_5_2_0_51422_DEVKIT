/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
#ifndef Devkit_CONFIG_H
#define Devkit_CONFIG_H

#include <stdint.h>
#include <string.h>
#include "ble_srv_common.h"
#include "pstorage.h"

extern pstorage_handle_t        mp_flash_PermPara;                      			 /**< Pointer to flash location to write next System Attribute information. */
extern pstorage_handle_t        mp_flash_ANT_DynPara;                   			 /**< Pointer to flash location to write next Bonding Information. */
extern pstorage_handle_t        mp_flash_BLE_DynPara;                   			 /**< Pointer to flash location to write next Bonding Information. */

/*.................. Function Declarations ................*/
void devkit_Configuration(void);
void ReadPermConfiguration(void);
void antP2_ReadDynaConfiguration(void);
void bleP2_ReadDynaConfiguration(void);

void dev_kit_configParamUpdate(void);

//#ifdef ANT 
void antP2_ClearDynaConfiguration(void);
//#endig


#endif
