/* Copyright (c) 2013 Belwith Products. All Rights Reserved.
 *
 *
 */

#include "ble_sr.h"
#include <string.h>
#include <stdio.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_error.h"
#include "ble_gatt.h"
#include "main.h"
/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/
uint8_t 		handle_value;
uint8_t			UART_char_handle_value;

/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/



//static ble_gatts_char_handles_t device_state_handles;
ble_gatts_char_handles_t write_cmd_handles;
static ble_gatts_char_handles_t read_data_handles;

uint8_t write_cmd_handle_value;

/**@brief Connect event handler.
 *
 * @param[in]   p_sr        SecuRemote Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_sr_t * p_sr, ble_evt_t * p_ble_evt)
{
    p_sr->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Disconnect event handler.
 *
 * @param[in]   p_sr        SecuRemote Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_sr_t * p_sr, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sr->conn_handle = BLE_CONN_HANDLE_INVALID;
}

void ble_sr_on_ble_evt(ble_sr_t * p_sr, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sr, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sr, p_ble_evt);
            break;
            
        default:
            break;
    }
}

/**@brief Add characteristic as a single byte character
 *
 * @param[in]   p_sr           securemote Service structure.
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
/* 
static uint32_t sr_char_add(ble_sr_t *		 p_sr,
							ble_sr_init_t * 							p_sr_init,
							uint16_t                        uuid,
							uint8_t                         p_char_value)
{
    ble_gatts_char_md_t char_md;
		ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md; 
	
		memset(&cccd_md, 0, sizeof(cccd_md));
    
		// According to BAS_SPEC_V10, the read operation on cccd should be possible without
		// authentication.
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		cccd_md.write_perm = p_sr_init->device_state_char_attr_md.cccd_write_perm;
		cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	
    memset(&char_md, 0, sizeof(char_md));

		char_md.char_props.read  = 1;
		char_md.char_props.notify= (p_sr->is_notification_supported) ? 1 : 0;;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
		char_md.p_cccd_md        = (p_sr->is_notification_supported) ? &cccd_md : NULL;;
    char_md.p_sccd_md        = NULL;
    
	ble_uuid.type = p_sr->smartdevice_uuid_type;
	ble_uuid.uuid = uuid;		

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);		
		attr_md.read_perm  = p_sr_init->device_state_char_attr_md.read_perm;
    attr_md.write_perm = p_sr_init->device_state_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = &p_char_value;
    
    return sd_ble_gatts_characteristic_add(p_sr->service_handle, &char_md, &attr_char_value, &p_sr->device_state_handles);		
}
*/

/**@brief Add characteristic as a string 
 * @param[in]   p_sr        	 SecuRemote Service structure.
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[in]   char_len       Length of initial value. This will also be the maximum value.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t sr_char_add_string(ble_sr_t *											 p_sr,
									uint16_t                        uuid,
									uint8_t *                       p_char_value,
									uint16_t                        char_len,
									ble_gatts_char_handles_t *      p_handles)
{
	uint32_t 						err_code;
    ble_uuid_t          ble_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;

    APP_ERROR_CHECK_BOOL(p_char_value != NULL);
    APP_ERROR_CHECK_BOOL(char_len > 0);
    
    // The ble_gatts_char_md_t structure uses bit fields. So we reset the memory to zero.
    memset(&char_md, 0, sizeof(char_md));

	if(uuid==BLE_UUID_READ_SL_DATA)
	{
		char_md.char_props.read  = 1;
	}
	else if(uuid==BLE_UUID_DEV_KIT_OP_REQ || uuid == BLE_UUID_DEV_KIT_UART_MSG)
	{
		char_md.char_props.write  = 1;
		char_md.char_props.read   = 1;	
		char_md.char_ext_props.reliable_wr = 1;		
	}

    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

	ble_uuid.type = p_sr->smartdevice_uuid_type;
	ble_uuid.uuid = uuid;		
		
    memset(&attr_md, 0, sizeof(attr_md));

	if(uuid==BLE_UUID_DEV_KIT_OP_REQ || uuid == BLE_UUID_DEV_KIT_UART_MSG)
	{
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	}
		
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;		
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = char_len;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = char_len;
    attr_char_value.p_value      = p_char_value;
		
	err_code = sd_ble_gatts_characteristic_add(p_sr->service_handle, &char_md, &attr_char_value, p_handles);
	
	//get handle value of characteristics for reference
	write_cmd_handle_value = p_sr->operation_status_handles.value_handle;
//	write_cmd_handle_value = p_sr->write_cmd_handles.value_handle;
	
	return err_code;
}

/**@brief Add SecuRemote characteristic.
 *
 * @param[in]   p_sr        	 SecuRemote Service structure.
 * @param[in]   p_sr_init    	 Information needed to initialize the service.
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t sr_char_add_notify(ble_sr_t *		 p_sr,
																	 ble_sr_init_t * 							p_sr_init,
																	 uint8_t                      p_char_value)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_operation_status;
    
    // Add Battery Level characteristic
    if (p_sr->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));
    
        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_sr_init->operation_status_char_attr_md.cccd_write_perm;
        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    }
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = (p_sr->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_sr->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;
    
	ble_uuid.type = p_sr->smartdevice_uuid_type;
	ble_uuid.uuid = BLE_UUID_OPERATION_STATUS;		
    
    memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.read_perm  = p_sr_init->operation_status_char_attr_md.read_perm;
    attr_md.write_perm = p_sr_init->operation_status_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
	initial_operation_status = p_sr_init->initial_operation_status;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = &initial_operation_status;
    
    err_code = sd_ble_gatts_characteristic_add(p_sr->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_sr->operation_status_handles);																							 
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	p_sr->report_ref_handle = BLE_GATT_HANDLE_INVALID;

	return NRF_SUCCESS;
}



/**@brief Add SecuRemote characteristic.
 *
 * @param[in]   p_sr        	 SecuRemote Service structure.
 * @param[in]   p_sr_init    	 Information needed to initialize the service.
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t devkit_char_add_notify(ble_sr_t *		 p_sr,
																	 ble_sr_init_t * 							p_sr_init,
																	 uint8_t                      p_char_value)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_operation_status;
    
    // Add Battery Level characteristic
    if (p_sr->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));
    
        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_sr_init->devkit_notify_char_attr_md.cccd_write_perm;
        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    }
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = (p_sr->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_sr->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;
    
	ble_uuid.type = p_sr->smartdevice_uuid_type;
	ble_uuid.uuid = BLE_UUID_DEV_KIT_OP_STATUS;		
    
    memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.read_perm  = p_sr_init->devkit_notify_char_attr_md.read_perm;
    attr_md.write_perm = p_sr_init->devkit_notify_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
	initial_operation_status = p_sr_init->initial_operation_status;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = OP_STATUS_BUFF_LEN; //sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = OP_STATUS_BUFF_LEN; //sizeof(uint8_t);
    attr_char_value.p_value      = &initial_operation_status;
    
    err_code = sd_ble_gatts_characteristic_add(p_sr->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_sr->devkit_notify_handles);																							 
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	p_sr->report_ref_handle = BLE_GATT_HANDLE_INVALID;

	return NRF_SUCCESS;
}


uint32_t ble_sr_init(ble_sr_t * p_sr, ble_sr_init_t * p_sr_init)
{
    uint32_t      err_code;
	ble_uuid_t    service_uuid;
	ble_uuid128_t smartdevice_base_uuid =  {0xF5, 0x4C, 0x13, 0x1A, 0xC4, 0x70, 0x63, 0x98, 0x08, 0x4D, 0xA2, 0x45, 0x69, 0x64, 0x31, 0x60};

//{0xF5, 0x1E, 0xC0, 0x5A, 0x0D, 0x08, 0x91, 0xB3, 0x91, 0x4E, 0x81, 0x72, 0x3F, 0x40, 0x2F, 0xC0};
											//C02F-403E-7281-4E91-B391-080D-5AC0-1Ef5
	
	// Initialize service structure
	p_sr->evt_handler                 = p_sr_init->evt_handler;
	p_sr->is_notification_supported   = p_sr_init->support_notification;
	p_sr->conn_handle                 = BLE_CONN_HANDLE_INVALID;
	
	// Add service (smart device)
	err_code = sd_ble_uuid_vs_add(&smartdevice_base_uuid, &p_sr->smartdevice_uuid_type);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	service_uuid.type = p_sr->smartdevice_uuid_type;
	service_uuid.uuid = BLE_UUID_DEV_KIT_SERVICE;

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_sr->service_handle);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	

	if (p_sr_init->write_cmd.length > 0)
	{		
		err_code = sr_char_add_string(p_sr,
									BLE_UUID_DEV_KIT_OP_REQ,
									p_sr_init->write_cmd.p_str, 
									p_sr_init->write_cmd.length, 					
									&write_cmd_handles);					
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}
	}
		

	if (p_sr_init->read_data.length > 0)
	{		
		err_code = sr_char_add_string(p_sr,
									BLE_UUID_READ_SL_DATA,
									p_sr_init->read_data.p_str, 
									p_sr_init->read_data.length, 					
									&read_data_handles);					
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}
	}		
    // Add smart device's operation status characteristic
		err_code = sr_char_add_notify(p_sr,p_sr_init,p_sr_init->operation_status);		
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }	
		
		 // Add devkit notification status characteristic
		err_code = devkit_char_add_notify(p_sr,p_sr_init,p_sr_init->devkit_notify);		
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }	
	
		// Add UART message request characteristic
	
//	if (p_sr_init->UART_msg.length > 0)
//	{		
//		err_code = sr_char_add_string(p_sr,
//										BLE_UUID_DEV_KIT_UART_MSG,
//										p_sr_init->UART_msg.p_str, 
//										p_sr_init->UART_msg.length, 					
//										&p_sr->UART_data_handle);		      			
//		if (err_code != NRF_SUCCESS)
//		{
//			return err_code;
//		}
//		//get handle value of characteristics for reference
//		UART_char_handle_value = p_sr->UART_data_handle.value_handle;
//	}

		
    return NRF_SUCCESS;
}

/**@brief device status update 

 */
uint32_t ble_sr_device_state_update(ble_sr_t * p_sr, uint8_t device_state)   // Not used
{
    uint32_t err_code = NRF_SUCCESS;

		uint16_t len = sizeof(uint8_t);
		
		// Save new device state value
		p_sr->device_state_last = device_state;

		// Update database
		err_code = sd_ble_gatts_value_set(p_sr->device_state_handles.value_handle,
																	 0,
																	 &len,
																	 &device_state);
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}
		
		// Send value if connected and notifying
		if ((p_sr->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sr->is_notification_supported)
		{
				ble_gatts_hvx_params_t hvx_params;
				
				memset(&hvx_params, 0, sizeof(hvx_params));
				len = sizeof(uint8_t);
				
				hvx_params.handle   = p_sr->device_state_handles.value_handle;
				hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset   = 0;
				hvx_params.p_len    = &len;
				hvx_params.p_data   = &device_state;
				
				err_code = sd_ble_gatts_hvx(p_sr->conn_handle, &hvx_params);
		}
		else
		{
				err_code = NRF_ERROR_INVALID_STATE;
		}					

    return err_code;
}
/**@brief

 */
uint32_t ble_sr_operation_status_update(ble_sr_t * p_sr, uint8_t operation_status)    //Device operation status SUCCSESS ,ERROR CODe etc
{
    uint32_t err_code = NRF_SUCCESS;

		uint16_t len = sizeof(uint8_t);
//		uint8_t dbgStr[50];
		// Save new operation status value
		p_sr->operation_status_last = operation_status;
			
		// Update database
		err_code = sd_ble_gatts_value_set(p_sr->operation_status_handles.value_handle,
																	 0,
																	 &len,
																	 &operation_status);
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}

		// Send value if connected and notifying
		if ((p_sr->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sr->is_notification_supported)
		{
			ble_gatts_hvx_params_t hvx_params;
			
			memset(&hvx_params, 0, sizeof(hvx_params));
			len = sizeof(uint8_t);
			
			hvx_params.handle   = p_sr->operation_status_handles.value_handle;
			hvx_params.type 		= BLE_GATT_HVX_NOTIFICATION;
			hvx_params.offset   = 0;
			hvx_params.p_len    = &len;
			hvx_params.p_data   = &operation_status;
			
			err_code = sd_ble_gatts_hvx(p_sr->conn_handle, &hvx_params);
			
			#ifdef NOTIFICATION_DBG								
				sprintf(dbgStr, "\r\nerr_code = %d operation_status = %d %s %d", err_code, operation_status, __FILE__, __LINE__);
				simple_uart_putstring(dbgStr);					
			#endif
		}
		else
		{
				err_code = NRF_ERROR_INVALID_STATE;
		}

    return err_code;
}


/**@brief device status update 

 */
uint32_t ble_sr_device_read_buffer_update(ble_sr_t * p_sr, uint8_t *u8_buffer)    // READ_CONFIG_REQ etc..
{
    uint32_t err_code = NRF_SUCCESS;

		uint16_t len = READ_BUFFER_SIZE;
		
		// Save new device state value
		p_sr->device_state_last = u8_buffer[0];

		// Update database
		err_code = sd_ble_gatts_value_set(read_data_handles.value_handle,
																	 0,
																	 &len,
																	 &u8_buffer[0]);
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}
		
		// Send value if connected and notifying
		if ((p_sr->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sr->is_notification_supported)
		{
				ble_gatts_hvx_params_t hvx_params;
				
				memset(&hvx_params, 0, sizeof(hvx_params));
				len = sizeof(uint8_t);
				
				hvx_params.handle   = read_data_handles.value_handle;
				hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset   = 0;
				hvx_params.p_len    = &len;
				hvx_params.p_data   = &u8_buffer[0];
				
				err_code = sd_ble_gatts_hvx(p_sr->conn_handle, &hvx_params);
		}
		else
		{
				err_code = NRF_ERROR_INVALID_STATE;
		}					

    return err_code;
}

/**@brief

 */
uint32_t ble_mtr_operation_status_update(ble_sr_t * p_sr, uint8_t * data , uint16_t len)  // DEVKIT operation requests
{
    uint32_t err_code = NRF_SUCCESS;
		uint8_t op_status[10];
		
		memcpy(op_status, data, len);

		// Save new operation status value
		//p_mtr->operation_status_last = operation_status;
		
		// Update database
		err_code = sd_ble_gatts_value_set(p_sr->devkit_notify_handles.value_handle,
																	 0,
																	 &len,
																	 &op_status[0]);
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}

		// Send value if connected and notifying
		if ((p_sr->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sr->is_notification_supported)
		{
				ble_gatts_hvx_params_t hvx_params;
				
				memset(&hvx_params, 0, sizeof(hvx_params));
				len = sizeof(uint8_t);
				
				hvx_params.handle   = p_sr->devkit_notify_handles.value_handle;
				hvx_params.type 		= BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset   = 0;
				hvx_params.p_len    = &len;
				hvx_params.p_data   = &op_status[0];
				
				err_code = sd_ble_gatts_hvx(p_sr->conn_handle, &hvx_params);

				#ifdef RUN_DBG								
//						sprintf(dbgStr, "\r\nerr_code = %d operation_status = %d %s %d", err_code, operation_status, __FILE__, __LINE__);
//						simple_uart_putstring(dbgStr);					
				#endif
		}
		else
		{
				err_code = NRF_ERROR_INVALID_STATE;
		}

    return err_code;
}


/**@brief Restore modified char to default

 */
uint32_t sr_restore_modified_char_to_default(void)
{
    uint32_t err_code = NRF_SUCCESS;

	return err_code;
}
