/* Copyright (c) 2013 Belwith Products. All Rights Reserved.
 *

 *
 */

/** @file
 *

 * @brief SecuRemote Service module.
 *

 */

#ifndef BLE_SR_H__
#define BLE_SR_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"


/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Configuration Parameter Related macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define FLASH_PAGE_PERM_PARA          241//237
//#define FLASH_PAGE_DYN_PARA           238

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//macros
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define OP_REQ_BUFF_LEN			10
#define OP_STATUS_BUFF_LEN		20
#define UART_BUFF_LEN			20

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Parameter Size
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MANUFACTURE_NAME_SIZE					20
#define HW_VER_SIZE										8
#define SW_VER_SIZE										8
#define MODEL_NUM_SIZE								9
#define	SERIAL_NUM_BYTES							16
#define DEV_KIT_SR_NO_BYTES						16   //[KC]  change from 8 t0 16

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Status macro
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define LIGHT_ON_STAT							1
#define LIGHT_OFF_STAT						 	0
#define SWITCH_PRESSED_STATE					1
#define SWITCH_UNPRESSED_STATE					0
#define MOTION_DETECT							1
#define MOTION_REMOVED							0
#define WATER_LEVEL_DETECTED					1
#define WATER_LEVEL_NOTDETETED					0
#define CO2_SENSOR_DETECTED						1
#define CO2_SENSOR_REMOVED						0
#define THERMOSTAT_RELAY_ON						1
#define THERMOSTAT_RELAY_OFF					0
#define HEATING_RELAY_ON						1
#define COOLING_RELAY_ON						0
#define HARDWARE_NS_BLE							0
#define HARDWARE_NS_548							1
#define HUMIDITY_CONNECTED						1
#define HUMIDITY_NOT_CONNECTED					0



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Permanent parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct 	_bw_flashPermanentParams_template								// data type
{
	uint8_t light_intensity;
	uint8_t light_status;
	int8_t	heating_set_point_flash;
	int8_t	cooling_set_point_flash;
	uint8_t hardware_type_flash;
	uint8_t humidity_sensor_presence_flash;									//Identifying humidity sensor is presence or not
	uint8_t relay_op_online_mode_flash;
	uint8_t flash_Status_flg;
	char manufact_name_str[MANUFACTURE_NAME_SIZE + 1];		
	char hw_rev_str[HW_VER_SIZE + 1];
	char sw_rev_str[SW_VER_SIZE + 1];
	char model_num_str[MODEL_NUM_SIZE + 1];
	uint8_t DevKit_SrNo_flash[DEV_KIT_SR_NO_BYTES];
	uint8_t	dev_kit_sr_no_set_flg_flash;
}__attribute__((aligned(8))); 

typedef struct _bw_flashPermanentParams_template	bw_flashPermanentParams_t;


/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/
extern uint8_t write_cmd_handle_value;

/**@brief SecuRemote Service event type. */
typedef enum
{
    BLE_SR_EVT_NOTIFICATION_ENABLED,                   /**< SecuRemote value notification enabled event. */
    BLE_SR_EVT_NOTIFICATION_DISABLED                   /**< SecuRemote value notification disabled event. */
} ble_sr_evt_type_t;

/**@brief SecuRemote Service event. */
typedef struct
{
    ble_sr_evt_type_t evt_type;                        /**< Type of event. */
} ble_sr_evt_t;

// Forward declaration of the ble_sr_t type. 
typedef struct ble_sr_s ble_sr_t;

/**@brief SecuRemote Service event handler type. */
typedef void (*ble_sr_evt_handler_t) (ble_sr_t * p_sr, ble_sr_evt_t * p_evt);

/**@brief SecuRemote Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
	ble_sr_evt_handler_t           evt_handler;                 /**< Event handler to be called for handling events in the SecuRemote Service. */
	bool                           support_notification;        /**< TRUE if notification of Operation Status is supported. */
	ble_srv_report_ref_t *         p_report_ref;	
	ble_srv_utf8_str_t             write_cmd;           					/**< Write Command String. */
	ble_srv_utf8_str_t             read_data;           					/**< Read Data String. */	
	
	ble_srv_utf8_str_t				UART_msg;
	
	
	uint8_t						   device_status;							/**< device status */
	uint8_t						   operation_status;						/**< Operation Status 1 byte long can have multiple combination to address various status of number of operations */
	uint8_t							 devkit_notify;
	ble_srv_cccd_security_mode_t   operation_status_char_attr_md;
	ble_srv_cccd_security_mode_t   device_state_char_attr_md;  
  ble_srv_cccd_security_mode_t   devkit_notify_char_attr_md;	
	uint8_t                        initial_device_state;
	uint8_t						   initial_operation_status;	
	ble_gap_conn_sec_mode_t        device_state_report_read_perm;
	ble_gap_conn_sec_mode_t		   operation_status_report_read_perm;	
	ble_gap_conn_sec_mode_t		   devkit_notify_report_read_perm;	
} ble_sr_init_t;

/**@brief SecuRemote Service structure. This contains various status information for the service. */
typedef struct ble_sr_s
{
	uint8_t                      smartdevice_uuid_type;	
	ble_sr_evt_handler_t         evt_handler;                                          /**< Event handler to be called for handling events in the SecuRemote Service. */
	bool                         is_notification_supported;       										 /**< TRUE if notification of Operation Status is supported. */  
	uint16_t                     report_ref_handle;               										 /**< Handle of the Report Reference descriptor. */
	ble_gatts_char_handles_t     device_state_handles;
	ble_gatts_char_handles_t	   operation_status_handles;	
	ble_gatts_char_handles_t	   devkit_notify_handles;
	ble_gatts_char_handles_t	 UART_data_handle;
	
	uint16_t                     service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
	uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
	uint8_t						 operation_status_last;	
	uint8_t						 device_state_last;	
	uint8_t                      initial_device_state;
	uint8_t						 initial_operation_status;	
} ble_sr_t;

/**@brief Initialize the SecuRemote Service.
 *
 * @param[out]  p_sr       SecuRemote Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_sr_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_sr_init(ble_sr_t * p_sr, ble_sr_init_t * p_sr_init);

/**@brief SecuRemote Service BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the SecuRemote Service.
 *
 * @param[in]   p_sr       SecuRemote Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_sr_on_ble_evt(ble_sr_t * p_sr, ble_evt_t * p_ble_evt);

/**@brief Update Lock Status
 *
 * @param[in]   p_sr           		  SecuRemote Service structure.
 * @param[in]   device_state  			New Device State value.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_sr_device_state_update(ble_sr_t * p_sr, uint8_t device_state);

/**@brief Update Lock Status
 *
 * @param[in]   p_sr           		  SecuRemote Service structure.
 * @param[in]   device_state  			New Device State value.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_sr_device_read_buffer_update(ble_sr_t * p_sr, uint8_t *u8_buffer);

/**@brief Update Operation Status
 *
 * @details The application calls this function after having performed an operation . If
 *          notification has been enabled, the operation status characteristic is sent to the client.
 * @param[in]   p_sr           		SecuRemote Service structure.
 * @param[in]   operation_status  New Operation Status value.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_sr_operation_status_update(ble_sr_t * p_sr, uint8_t operation_status);

/**@brief Restore Used Characteristic to default
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t sr_restore_modified_char_to_default(void);



/**@brief Update Operation Status
 *
 * @details The application calls this function after having performed an operation . If
 *          notification has been enabled, the operation status characteristic is sent to the client.
 * @param[in]   p_mtr           		Motor operation Service structure.
 * @param[in]   operation_status  New Operation Status value.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_mtr_operation_status_update(ble_sr_t * p_mtr, uint8_t * data , uint16_t len);

#endif // BLE_SR_H__

/** @} */
