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
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "devkit\devkit_config.h"
#include "devkit\devkit_debug.h"
#include "devkit\devkit_application.h"
#include "devkit\devkit_variable.h"
#include "devkit\devkit_command.h"
#include "devkit\devkit_motor.h"
#include "devkit\devkit_constant.h"
#include "ble_srv_common.h"
#include "nrf_delay.h"
#include "pstorage.h"

#define BLE_DBG_BUF_SIZE  10

#define DEBUG(x)     simple_uart_putstring((const uint8_t *)(x))
#define CRLF		 DEBUG("\r\n")


//#define BLE_PA  1
#define DEBUGLOG_ENABLE 1
/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/

//**********************************************************************************
//	Device Info Related Macro
//**********************************************************************************
#define ADV_DEVICE_NAME                 "SRDev_kit "								/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "BELWITH PRODUCTS"							/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER                    "SRD731"									/**< Model Number. Will be passed to Device Information Service. */
#define HW_VERSION			            "12.01.01"									/**< HW Version. Will be passed to Device Information Service. */
#define SW_VERSION			            "01.01.01"									/**< SW Version. Will be passed to Device Information Service. */


//**********************************************************************************
//	Application Timer related Macro
//**********************************************************************************
                                        /**< Size of timer operation queues. */

#define MAX_SAMPLE_LEVELS 							(1500UL)     																/**< Maximum number of sample levels. */
#define TIMER_PRESCALERS 								6U  
#define INTENSITY_MULTIPLIER							15

/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/



//**********************************************************************************
//	BLE Related Macro
//**********************************************************************************
#define APP_ADV_INTERVAL                1600                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define ALWAYS_ADVERTISE				0                                           /**< The advertise always */

#define SECOND_1_25_MS_UNITS            800                                         /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                         /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 100)                  /**< Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS/10)                      /**< Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                    /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                              /**< Maximum number of users of the GPIOTE handler. */
//**********************************************************************************
//	Application Timer related Macro
//**********************************************************************************
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         6   

#define CH0_CONNECT_TIMEOUT(M)     		APP_TIMER_TICKS(M, APP_TIMER_PRESCALER) 	/**< channel 0 connect timeout (ticks). */

#define SEC_PARAM_TIMEOUT               30                                          	 /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  0                                           	 /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           	 /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        	 /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           	 /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           	 /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          	 /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)       /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

extern pstorage_handle_t        		mp_flash_PermPara;                      			 /**< Pointer to flash location to write next System Attribute information. */
static ble_gap_sec_params_t             m_sec_params;                               	 /**< Security requirements for this application. */
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Misc. macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define READ_BUFFER_SIZE				200
#define WRITE_BUFFER_SIZE				100


/**@brief External reference to application timer. */
extern uint16_t sl_app_timer[MAX_APP_TIMER];
extern uint8_t SessionID[SESSION_ID_BYTES];
extern pstorage_handle_t        mp_flash_ANT_DynPara;                   			 /**< Pointer to flash location to write next Bonding Information. */
extern pstorage_handle_t        mp_flash_BLE_DynPara;                   			 /**< Pointer to flash location to write next Bonding Information. */
//-----------------------------------------------------------
/*.................. Function Declarations ................*/
//-----------------------------------------------------------
void antPx_NodeSetUp(void);
static void antPx_Ch0SetUpAsSlave(void);
static void antPx_Ch1SetUpAsMaster(void);

void gen_PrintRcvData(uint8_t *);
void gen_PrintHex(uint8_t adata);
void gen_PrintDec(uint8_t adata);
void gen_PrintHexStr(uint8_t *str, uint8_t NoOfByte);
void gen_PrintCharStr(uint8_t *str, uint8_t NoOfByte);

void FlashWrite(pstorage_handle_t * handle,uint8_t *write_buffer, uint16_t size);
void flash_read(uint8_t *read_buffer, uint16_t num_bytes_to_read);
void flash_init(void);
void flash_ClearPage(pstorage_handle_t);
static void sys_evt_dispatch(uint32_t sys_evt);
void advertising_start(void);
void advertising_stop(void);
void timers_start(void);
void timers_stop(void);

void ppi_init(void);
void Ul_gpio_int(void);

extern char rx_buf[RX_BUFFER_SIZE];
extern uint8_t sl_rcvcntr;
extern uint8_t sl_rcvtimeout;
extern uint8_t sl_rcvmsg_ok;
extern uint8_t SessionIDReceiveOKFlg;
extern uint8_t UserAuthenticatedFlg;
extern uint8_t gu8_RangeInOrOut;
extern uint8_t gu8_AutoManualConnStatus;

extern volatile uint8_t gu8_AntMsgReceivedOnBle;
extern volatile uint8_t gu8_BleMsgReceivedOnAnt;
extern uint8_t	gu8_TargetAntSerialNumber[ANT_SERIAL_NUMBER_BYTES];

extern char rx_buf[RX_BUFFER_SIZE];
extern uint8_t sl_rcvcntr;
extern uint8_t sl_rcvtimeout;
extern uint8_t sl_rcvmsg_ok;

extern st_FlashPermanentParams_t gst_ConfigPermanentParams;
extern st_BleFlashDynamicParams_t gst_BleConfigDynamicParams;
extern st_FlashDataOfPreviousVersion_t gst_FlashDataOfPreviousVersion;

/**@brief External reference to operation request. */
extern uint8_t ble_app_sl_request;

#endif
