/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup ble_sdk_app_ant_hrs_main main.c
 * @{
 * @ingroup ble_sdk_app_ant_hrs
 * @brief HRM sample application using both BLE and ANT.
 *
 * The application uses the BLE Heart Rate Service (and also the Device Information
 * services), and the ANT HRM RX profile.
 *
 * It will open a receive channel which will connect to an ANT	 HRM TX profile device when the
 * application starts. The received data will be propagated to a BLE central through the
 * BLE Heart Rate Service.
 *
 * The ANT HRM TX profile device simulator SDK application
 * (Board\pca10003\ant\ant_hrm\hrm_tx_buttons) can be used as a peer ANT device. By changing
 * ANT_HRMRX_NETWORK_KEY to the ANT+ Network Key, the application will instead be able to connect to
 * an ANT heart rate belt.
 *
 * @note The ANT+ Network Key is available for ANT+ Adopters. Please refer to
 *       http://thisisant.com to become an ANT+ Adopter and access the key.
 *
 * @note This application is based on the BLE Heart Rate Service Sample Application
 *       (Board\nrf6310\ble\ble_app_hrs). Please refer to this application for additional
 *       documentation.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ant_parameters.h"
#include "ant_interface.h"
#include "ant_error.h"
#include "app_error.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_sdm.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_sr.h"
#include "ble_conn_params.h"
#include "boards/ras_1_6.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_flash.h"
#include "nrf_delay.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "simple_uart.h"
#include "AntEventHandler.h"
#include "twi_master_int.h"
#include "Accelerometer.h"
#include "main.h"
#include "development_kit_config.h"
#include "led.h"
#include "devkit\devkit_constant.h"
#include "battery.h"
#include "nrf_soc.h"
#include "pwm.h"
#include "AntAsyncEvent.h"

uint32_t err_code;

/*........... Variable Declarations ............*/
unsigned char write_cmd[WRITE_BUFFER_SIZE];
unsigned char read_cmd[READ_BUFFER_SIZE];
uint8_t SessionID[SESSION_ID_BYTES];
uint8_t gu8_SerialNumberEncrypted[SERIAL_NUM_BYTES+4];				//Add one byte for security level indication

//static uint8_t m_ant_network_key[] = ANT_NETWORK_KEY; /**< ANT Network key. */
pstorage_handle_t        mp_flash_PermPara;                      			 /**< Pointer to flash location to write next System Attribute information. */
pstorage_handle_t        mp_flash_ANT_DynPara;                   			 /**< Pointer to flash location to write next Bonding Information. */
pstorage_handle_t        mp_flash_BLE_DynPara;                   			 /**< Pointer to flash location to write next Bonding Information. */

app_timer_id_t           m_second_timer_id;                              	 /**< Second Timeout timer. */
app_timer_id_t           m_GerneralPurposeTimer;      						 /**< General Purpose Timer */

//static app_flash2mngr_init_t 			m_appflashmngr_config;                       /**< Configuration as specified by the application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;     /**< Handle of the current connection. */
static ble_gap_adv_params_t             m_adv_params;                                /**< Parameters to be passed to the stack when starting advertising. */
ble_bas_t                        		m_bas;                                       /**< Structure used to identify the battery service. */
ble_sr_t                         		m_sr;                                  	 	 /**< Structure used to identify the SecuRemote service. */

/****************************************************************/
/* 			DEVKIT 												*/
/****************************************************************/
#define BUTTON_PULL    			NRF_GPIO_PIN_NOPULL
#define BUTTON_PULL_DOWN    	NRF_GPIO_PIN_PULLDOWN
#define UART_MSG											"00000000000000000000"

extern int intensity;
extern uint8_t 		sw_previous_status;
extern uint8_t hardware_type;
extern uint8_t humidity_sensor_presence;
extern uint8_t device_type;

extern bool temperature_notify_flag;
extern bool accelerometer_update_flg;
extern bool co2_notify_flag;
extern bool water_sensor_flg;
extern bool relay_status_flg;

bool co2_notify_on_connect_flg;
bool co2_last_status;
extern bool Hardware_config_flg;

uint8_t 	flash_Status;
bw_flashPermanentParams_t bw_configPermanentParams;

uint32_t STATUS_SW1,STATUS_SW2,DEVKIT_MOTOR_OP_POS_PIN,DEVKIT_MOTOR_OP_NEG_PIN;
uint8_t  DEVKIT_TX_PIN_NUMBER;


uint32_t 	light_intensity_value;
uint32_t 	update_light_intensity_value = 0;

typedef struct
{
    uint8_t                     flash_page_num_permenent_para;            			  	  /**< Flash page number to use for storing Bonding Information. */
    uint8_t                     flash_page_num_dynamic_para;        						  	  /**< Flash page number to use for storing System Attributes. */
    ble_srv_error_handler_t     error_handler;                  								  		/**< Function to be called in case of an error. */
} App_Flashmngr_init_t;


bool cc0_turn = false;


void adc_NTC_init(void);
void adc_CO2_init(void);


//static App_Flashmngr_init_t m_AppFlashMngr_config;                                	  /**< Configuration as specified by the application. */

/*************************************************/
/* 				"Function Definitations".		*/
/*************************************************/


/* @brief Function for initializing the Programmable Peripheral Interconnect peripheral.
 */

void ppi_init(void)
{
	nrf_pwm_config_t pwm_config = PWM_DEFAULT_CONFIG;  
    pwm_config.mode             = PWM_MODE_LED_1000;
    pwm_config.num_channels     = 1;
    pwm_config.gpio_num[0]      = PWM_OUTPUT_PIN_NUMBER;
   
    nrf_pwm_init(&pwm_config);  // Initialize the PWM library
}

/**@brief ADC initialization.
 *
 * @details Initializes ADC used by the application.
 */
void adc_NTC_init(void)
{
    // Configure ADC
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_10bit                       << ADC_CONFIG_RES_Pos)     |
													(ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling	<< ADC_CONFIG_INPSEL_Pos) |
													(ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling	<< ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_AnalogInput5               << ADC_CONFIG_PSEL_Pos)    |
													(ADC_CONFIG_EXTREFSEL_None									<< ADC_CONFIG_EXTREFSEL_Pos);
		
		NRF_ADC->EVENTS_END = 0;
}

void adc_CO2_init(void)
{	
		NRF_ADC->CONFIG     = (ADC_CONFIG_RES_10bit                       << ADC_CONFIG_RES_Pos)     |
													(ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling	<< ADC_CONFIG_INPSEL_Pos) |
													(ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling	<< ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_AnalogInput5               << ADC_CONFIG_PSEL_Pos)    |
													(ADC_CONFIG_EXTREFSEL_None									<< ADC_CONFIG_EXTREFSEL_Pos);
	
    NRF_ADC->EVENTS_END = 0;
}

void adc_bat_volt_init(void)
{	
		NRF_ADC->CONFIG     = (ADC_CONFIG_RES_10bit                       << ADC_CONFIG_RES_Pos)     |
													(ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling	<< ADC_CONFIG_INPSEL_Pos) |
													(ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling	<< ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_AnalogInput6               << ADC_CONFIG_PSEL_Pos)    |
													(ADC_CONFIG_EXTREFSEL_None									<< ADC_CONFIG_EXTREFSEL_Pos);
	
    NRF_ADC->EVENTS_END = 0;
}

/****************************************************************/
/****************************************************************/
/* 			END DEVKIT 											*/
/****************************************************************/
/****************************************************************/

/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
	DEBUG((const unsigned char*)"\r\nApp_Error_Handler\n\r");
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief Assert macro callback function.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Stop timers.
*/
void timers_stop(void)
{
    uint32_t err_code;
    
    // Stop application timers
    err_code = app_timer_stop(m_second_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**@brief Start advertising.
 */
void advertising_start(void)
{
    uint32_t err_code;
	
    // Initialise advertising parameters (used when starting advertising)
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = ALWAYS_ADVERTISE;
	
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
	
	
	#ifdef BLE_STACK_EVENT_DEBUG
	DEBUG("\r\nBLE ADVT START\r\n");
	#endif
}

/**@brief Stop advertising.
 */
void advertising_stop(void)
{
    uint32_t err_code;
    
    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);
	
	#ifdef BLE_STACK_EVENT_DEBUG
	DEBUG("\r\nBLE ADVT STOP\r\n");
	#endif
}

/**@brief Function for starting timers.
*/
void timers_start(void)
{	
	uint32_t err_code;
	
	// Start application timers
	err_code = app_timer_start(m_second_timer_id,CH0_CONNECT_TIMEOUT(100), NULL);//2000 Ticks.
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_start(m_GerneralPurposeTimer,CH0_CONNECT_TIMEOUT(1000), NULL);//1000 Ticks.
	APP_ERROR_CHECK(err_code);
}

/*****************************************************************************
* Static Timeout Handling Functions
*****************************************************************************/

/**@brief Second timer timeout handler.
 *
 * @details This function will be called each time the second timer expires.
 *          This function will check activity which is need at each second completion.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void second_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	app_timer_activity();
	get_battery_level();
}


/**@brief UART initialization.
 *
 * @details Initializes UART as per this application.
 */
static void uart_init(void)
{
	simple_uart_config(RTS_PIN_NUMBER, DEVKIT_TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
	DEBUG(((const uint8_t *)"\r\nANT+BLE SLAVE Firmware\n\r"));	
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void antIx_SecTimer(void *lvp_Parameter)
{
	if(b.mu8_EpochTimeValidFlag==1)
	{
		gu8_EpochTime++;
	}
	
	//Shared Address assignment Timeout on CH1
	if(gu8_AddrAssignTimeout[CH_1])
	{
		gu8_AddrAssignTimeout[CH_1]--;
		if(!gu8_AddrAssignTimeout[CH_1])
		{
			gu8_AntRegState=ADD_AVAILABEL;
			
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("ADDR ASSIGN TMOUT L2");	
			#endif
		}
	}
	
	//Shared Address assignment Timeout on CH0
	if(gu8_AddrAssignTimeout[CH_0])
	{
		gu8_AddrAssignTimeout[CH_0]--;
		if(!gu8_AddrAssignTimeout[CH_0])
		{
			gu8_AntAddrAssignState=0;
			
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("ADDR ASSIGN TMOUT GW");	
			#endif
			
			/* Close Channel */
			//if(b.mu8_AntChannelOpenState_ch0==1)
			{
				err_code = sd_ant_channel_close(CHANNEL_0);
				APP_ERROR_CHECK(err_code);
			}
		}
	}
		
	//Self Connection to Upper Layer Timeout
	if(gu8_AntSelfConnectionTimeout)
	{
		gu8_AntSelfConnectionTimeout--;
		if(!gu8_AntSelfConnectionTimeout)
		{
			gu8_AntStatusPollingRcv=0;
			
			#ifdef ANT_FLASH_DEBUG_ENABLE
			DEBUG("EADD2RUN\r\n");
			#endif	
			
			antP2_ClearDynaConfiguration();
		}
	}
	
}

/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
	
		// Create timers.
    err_code = app_timer_create(&m_second_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                second_timeout_handler);
		APP_ERROR_CHECK(err_code);
		
	err_code = app_timer_create(&m_GerneralPurposeTimer,
                                APP_TIMER_MODE_REPEATED,
                                antIx_SecTimer);
		APP_ERROR_CHECK(err_code);
}

/**@brief GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, ADV_DEVICE_NAME, strlen(ADV_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
	ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ble_advdata_manuf_data_t manuf_data;
	
	manuf_data.company_identifier = 0x1234;	
	manuf_data.data.size = sizeof(gu8_SerialNumberEncrypted);	
    manuf_data.data.p_data = (uint8_t *)gu8_SerialNumberEncrypted;
	
	// YOUR_JOB: Use UUIDs for service(s) used in your application.
	ble_uuid_t adv_uuids = (ble_uuid_t) {.uuid=BLE_UUID_DEV_KIT_SERVICE, .type=m_sr.smartdevice_uuid_type};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.p_manuf_specific_data   = &manuf_data;
    
	memset(&scanrsp, 0, sizeof(scanrsp));
	scanrsp.name_type               = BLE_ADVDATA_FULL_NAME;
	scanrsp.uuids_complete.uuid_cnt = 1;
	scanrsp.uuids_complete.p_uuids  = &adv_uuids;
	
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);	
}

/**@brief Initialize Device Information Service.
 */
static void dis_init(void)
{
    uint32_t       err_code;
    ble_dis_init_t dis_init;	
	
    //Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
	ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HW_VERSION);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, (char *)SW_VERSION);		

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief SecuRemote Service.
 */
static void dev_kit_service_init(void)
{
    uint32_t 			err_code;
	ble_sr_init_t   	sr_init;	
	
    // Initialize SecuRemote Service
  memset(&sr_init, 0, sizeof(sr_init));	
	
	memset(write_cmd,0,sizeof(write_cmd));
	memset(read_cmd,0,sizeof(read_cmd));

	ble_srv_ascii_to_utf8_len(&sr_init.write_cmd,(char *)write_cmd,sizeof(write_cmd));
	ble_srv_ascii_to_utf8_len(&sr_init.read_data,(char *)read_cmd,sizeof(read_cmd));	
	
	sr_init.UART_msg.length = UART_BUFF_LEN;
	ble_srv_ascii_to_utf8(&sr_init.UART_msg, UART_MSG);

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.operation_status_char_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.operation_status_char_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sr_init.operation_status_char_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.operation_status_report_read_perm);

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.device_state_char_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.device_state_char_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sr_init.device_state_char_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.device_state_report_read_perm);
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.devkit_notify_char_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.devkit_notify_char_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sr_init.devkit_notify_char_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sr_init.devkit_notify_report_read_perm);

	sr_init.evt_handler          		= NULL;
	sr_init.support_notification 		= true;
	sr_init.p_report_ref         		= NULL;		
	sr_init.initial_device_state   		= 0;
	sr_init.initial_operation_status 	= 0;		
	
    err_code = ble_sr_init(&m_sr, &sr_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	bas_init();				  // Battery Service initialization
	dis_init();				  // Device Information Service initialization	
	dev_kit_service_init();   //devkit service initialize
}

/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}
	
/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    switch (p_evt->evt_type)
    {
        case BLE_CONN_PARAMS_EVT_FAILED:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
            APP_ERROR_CHECK(err_code);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Application's Stack ANT event handler.
 *
 * @param[in]   p_ant_evt   Event received from the stack.
 */


/**@brief Application's Stack BLE event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    switch (p_ble_evt->header.evt_id)
    {
   case BLE_GAP_EVT_CONNECTED:
			blue_led_on();					
			simple_uart_putstring("\r\nConnect with central");
			co2_last_status=false;
			co2_notify_on_connect_flg = true;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			BleBits.mu8_BleConnectionState = TRUE;
		
			#ifdef BLE_STACK_EVENT_DEBUG
				DEBUG("\r\nBLE CONNECT\r\n");
			#endif
            break;
            
 case BLE_GAP_EVT_DISCONNECTED:
			blue_led_off();
			green_led_off();
			motor_stop();
			sl_app_timer[RELAY_OFF_TIMER]=0;
			sl_app_timer[TEMP_SCAN_TIMER]=0;
			relay_status_flg=false;
		
			nrf_gpio_pin_clear(RELAY_2_CTR_PIN_NUMBER);
			nrf_gpio_pin_clear(RELAY_1_CTR_PIN_NUMBER);
			nrf_gpio_pin_clear(SOLENOID_CTR_PIN_NUMBER);
		
			dev_kit_configParamUpdate();
			
			reset_all_the_timer_and_flg();
			simple_uart_putstring("\r\nDisconnect with central");

			m_conn_handle = BLE_CONN_HANDLE_INVALID;
			BleBits.mu8_BleConnectionState = FALSE;
			
			#ifdef BLE_STACK_EVENT_DEBUG
				DEBUG("\r\nBLE DISCONNECT\r\n");
			#endif
			if(Hardware_config_flg)
			{
				sl_app_timer[HW_INFO_UPDATE_TIMER] = HW_INFO_UPDATE_TIME; 
			}
			
			if(BleBits.mu8_BleFlashUpdate == BLE_FLASH_WRITE_INITIAT)
			{
				if(b.mu8_AntChannelOpenState_ch0==1)
				{
					BleBits.mu8_AntCH0_CloseByBLE = TRUE;
					err_code = sd_ant_channel_close(CHANNEL_0);
					APP_ERROR_CHECK(err_code);
				}
				if(b.mu8_AntChannelOpenState_ch1==1)
				{
					BleBits.mu8_AntCH1_CloseByBLE = TRUE;
					err_code = sd_ant_channel_close(CHANNEL_1);
					APP_ERROR_CHECK(err_code);				
				}
			}
//			else
//			{
				advertising_start();
//			}
			
            // Need to close the ANT channel to make it safe to write bonding information to flash
            
            // Note: Bonding information will be stored, advertising will be restarted and the
            //       ANT channel will be reopened when ANT event CHANNEL_CLOSED is received.
			
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, 
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {                 
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
				err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
				APP_ERROR_CHECK(err_code);
				break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Dispatches a stack event to all modules with a stack BLE event handler.
 *
 * @details This function is called from the Stack event interrupt handler after a stack BLE
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Stack Bluetooth event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
	
	ble_bas_on_ble_evt(&m_bas, p_ble_evt);
	ble_sr_on_ble_evt(&m_sr, p_ble_evt);
}

/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}

//Button handler
static void button_event_handler(uint8_t pin_no)
{
    switch (pin_no)
    {
		case WATER_SENSOR_PIN_NUMBER:
						if(device_type==WATER_SENSOR)
						{
							water_sensor_flg=true;
							update_water_sensor_status();
						}
					  break;
		case FACTORY_RESET:
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("\r\nFACT_RESET_SW");
			#endif
		DEBUG("\r\nFACT_RESET_SW");
			timers_start();

//			if((switch_type==FACTORY_RESET_SWITCH) && (test_by_production_tool))
//			{
//				sl_app_timer[BUTTON_TEST_TIMER]=BUTTON_TEST_TIME;
//				if(sl_app_timer[BUTTON_TEST_FAIL_TIMER] <= BUTTON_TEST_TIME)
//				{
//					sl_app_timer[BUTTON_TEST_FAIL_TIMER] = BUTTON_TEST_TIME+1;
//				}
//			}
//			else
			{
				sl_app_timer[FACT_RST_TIMER]=FACT_RST_TIME;
			}
            break;
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

/**@brief Initialize buttons.
 */
static void buttons_init(void)
{	
	    // Note: Array must be static because a pointer to it will be saved in the Button handler
    //       module.
    static app_button_cfg_t buttons[] =
    {			
				{WATER_SENSOR_PIN_NUMBER,true, BUTTON_PULL_DOWN, button_event_handler},
				{FACTORY_RESET, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
}    


/**@brief BLE + ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the stack event interrupt.
 */
static void ble_ant_stack_init(void)
{
    uint32_t err_code; 
	// Initialize SoftDevice
	
	#ifdef BLE_PA
	// Start 16 MHz crystal oscillator /
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->XTALFREQ = UICR_XTALFREQ_XTALFREQ_32MHz;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	// Wait for the external oscillator to start up /
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
	{
	}
  //  SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, false);
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
    #else
	 SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
	#endif
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
        
    // Subscribe for ANT events.
    err_code = softdevice_ant_evt_handler_set(on_ant_evt);
    APP_ERROR_CHECK(err_code);
	
	// Register with the SoftDevice handler module for system events
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code;
    
    // Wait for events    
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Application Flash storage initialization.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void af_pstorage_cb_handler(pstorage_handle_t * handle,
                                   uint8_t             op_code,
                                   uint32_t            result,
                                   uint8_t           * p_data,
                                   uint32_t            data_len)
{
	if(handle->module_id == mp_flash_PermPara.module_id)
	{
		#ifdef PSTORAGE_DBG
			DEBUG("Flash operation for permanent para\r\n");
		#endif
	}
	else if(handle->module_id == mp_flash_ANT_DynPara.module_id)
	{
		#ifdef ANT_FLASH_DEBUG_ENABLE
			DEBUG("ANT Flash Operation\r\n");
		#endif
		
		switch(op_code)
		{	
			case PSTORAGE_CLEAR_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					if(b.mu8_AntDataEraseFlag)
					{
						b.mu8_AntDataEraseFlag=0;
						
						#ifdef ANT_FLASH_DEBUG_ENABLE
							DEBUG("Page Erase OK for RESET\r\n");
						#endif
						b.mu8_AntResetChip=1;
						NVIC_SystemReset();
					}
					else
					{
						#ifdef ANT_FLASH_DEBUG_ENABLE
							DEBUG("Page Erase OK for Write\r\n");
						#endif
					}
				}
				else
				{
					#ifdef ANT_FLASH_DEBUG_ENABLE
						DEBUG("ANT Flash Page Erase Fail\r\n");
					#endif
				}
				
			break;

			case PSTORAGE_STORE_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef ANT_FLASH_DEBUG_ENABLE
						DEBUG("Store Sucessfully\r\n");
					#endif
					
					gu8_AntFlashWriteState=NO_ANT_FLASH_WRITE_OPERATION;
					
					switch(gu8_AntFlashWriteReason)
					{
						case SHARED_ADDR_ADD:
							
							#ifdef ANT_FLASH_DEBUG_ENABLE
							DEBUG("SA STORE SUCCESS,RESET NODE\r\n");
							#endif
							
							b.mu8_AntResetChip=1;
						
						break;
						
						case CLEAR_DYNA_PARA:
							
							#ifdef ANT_FLASH_DEBUG_ENABLE
							DEBUG("DYNA PARA CLEAR,RESET NODE\r\n");
							#endif
							b.mu8_AntResetChip=1;

						break;

						case DIRECT_ANT_NODE_INFO_ADD:
							
							//New Node Added
							gu8_AntNoOfUnRegisterNode++;
							b.mu8_AntSendNodeInfoToMaster=1;
						
						break;
						
						case INDIRECT_ANT_NODE_INFO_ADD:
							
							//New Node Added
							gu8_AntNoOfUnRegisterNode+=gu8_AdditionOfIndirectNodes;
							gu8_AdditionOfIndirectNodes=0;
						
						break;
						
						case ANT_NODE_REG_ADD:
							
							gu8_AntNoOfUnRegisterNode=0;
						
						break;
						
						case DEVICE_NO_ADD:

							#ifdef ANT_FLASH_DEBUG_ENABLE
							DEBUG("DEVICE_NO_ADD\r\n");
							#endif
						
							b.mu8_AntResetChip=1;
						
							//Start ANT Network as Device #(Network ID) received
							//b.mu8_AntStartNetwork=1;
						
						break;

						default:
							
						break;
					}
					
					gu8_AntFlashWriteReason=NOTHING_TO_ADD;
				}
				else
				{
					#ifdef ANT_FLASH_DEBUG_ENABLE
						DEBUG("Store Fail\r\n");
					#endif
				}
				
			break;
			
			case PSTORAGE_LOAD_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef ANT_FLASH_DEBUG_ENABLE
						DEBUG("Load Sucessfully\r\n");
					#endif
				}
				else
				{
					#ifdef ANT_FLASH_DEBUG_ENABLE
						DEBUG("Load Fail\r\n");
					#endif
				}
				
			break;
				
				
			case PSTORAGE_UPDATE_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef ANT_FLASH_DEBUG_ENABLE
						DEBUG("Update Sucessfully\r\n");
					#endif
				}
				else
				{
					#ifdef ANT_FLASH_DEBUG_ENABLE
						DEBUG("Update Fail\r\n");
					#endif
				}
				
			break;
				
			case PSTORAGE_ERROR_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef ANT_FLASH_DEBUG_ENABLE
						DEBUG("Error\r\n");
					#endif
				}
				else
				{
					// Store operation failed.
				}
				
			break;
		}	
	}
	else if(handle->module_id == mp_flash_BLE_DynPara.module_id)
	{
		#ifdef PSTORAGE_DBG
			DEBUG("BLE Flash operation\r\n");
		#endif
		switch(op_code)
		{	
			case PSTORAGE_CLEAR_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef PSTORAGE_DBG
						DEBUG("Page Erase Sucessfully\r\n");
					#endif
				}
				else
				{
					#ifdef PSTORAGE_DBG
						DEBUG("Page Erase Fail\r\n");
					#endif
				}
				
			break;

			case PSTORAGE_STORE_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef PSTORAGE_DBG
						DEBUG("Store Sucessfully\r\n");
					#endif

					BleBits.mu8_BleFlashUpdate = NO_BLE_FLASH_WRITE_OPERATION;
					
					if(BleBits.mu8_AntCH0_CloseByBLE == TRUE)
					{
						#ifdef BLE_STACK_EVENT_DEBUG
							DEBUG("Open CH0 again\r\n");
						#endif
						/* Open Channel 0 */
						err_code = sd_ant_channel_open(CHANNEL_0);
						APP_ERROR_CHECK(err_code);
						
						b.mu8_AntChannelOpenState_ch0=1;
						BleBits.mu8_AntCH0_CloseByBLE = 0;
					}
					
					if(BleBits.mu8_AntCH1_CloseByBLE == TRUE)
					{
						#ifdef BLE_STACK_EVENT_DEBUG
							DEBUG("Open CH1 again\r\n");
						#endif
						/* Open Channel 1 */
						err_code = sd_ant_channel_open(CHANNEL_1);
						APP_ERROR_CHECK(err_code);
						
						b.mu8_AntChannelOpenState_ch1=1;
						BleBits.mu8_AntCH1_CloseByBLE = 0;
					}
				}
				else
				{
					#ifdef PSTORAGE_DBG
					DEBUG("\r\nStore Fail\r\n");
					#endif
					BleBits.mu8_BleFlashUpdate = NO_BLE_FLASH_WRITE_OPERATION;
				}
				
			break;
			
			case PSTORAGE_LOAD_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef PSTORAGE_DBG
						DEBUG("Load Sucessfully\r\n");
					#endif
				}
				else
				{
					#ifdef PSTORAGE_DBG
						DEBUG("Load Fail\r\n");
					#endif
				}

			break;

			case PSTORAGE_UPDATE_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef PSTORAGE_DBG
						DEBUG("Update Sucessfully\r\n");
					#endif
				}
				else
				{
					#ifdef PSTORAGE_DBG
						DEBUG("Update Fail\r\n");
					#endif
				}
				
			break;
				
			case PSTORAGE_ERROR_OP_CODE:
				
				if (result == NRF_SUCCESS)
				{
					#ifdef PSTORAGE_DBG
						DEBUG("Error\r\n");
					#endif
				}
				else
				{
					// Store operation failed.
				}
				
			break;
		}
	}
}

/**@brief Application Flash storage initialization.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */

uint32_t flash_manager_init(app_flashmngr_init_t * p_init)
{
    pstorage_module_param_t param;
    uint32_t err_code;

    if (p_init->error_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
              
    param.block_size  = 1024;//sizeof(st_FlashPermanentParams_t);
    param.block_count = 1;
    param.cb          = af_pstorage_cb_handler;
    
    // Blocks are requested twice, once for bond information and once for system attributes.
    // The number of blocks requested has to be the maximum number of bonded devices that
    // need to be supported. However, the size of blocks can be different if the sizes of
    // system attributes and bonds are not too close.
    err_code = pstorage_register(&param, &mp_flash_PermPara);    
    APP_ERROR_CHECK(err_code);
    
//	param.block_size  = sizeof(st_BleFlashDynamicParams_t);	
	err_code = pstorage_register(&param, &mp_flash_BLE_DynPara);    
    APP_ERROR_CHECK(err_code);
	
//	param.block_size  = sizeof(st_AntFlashHeader_t);	
    err_code = pstorage_register(&param, &mp_flash_ANT_DynPara);    
    APP_ERROR_CHECK(err_code);   
    
    //m_appflashmngr_config = *p_init;
    
    return NRF_SUCCESS;
}

/**@brief Bond Manager module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void app_flashmanager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief FlashData Manager Init.
 *

 */
static void flash_data_manager_init(void)
{
    uint32_t             err_code;
    app_flashmngr_init_t app_init_data;	
	
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);
		
    // Initialize the Bond Manager
    app_init_data.flash_page_num_permenent_para     = FLASH_PAGE_PERM_PARA;
    app_init_data.flash_page_num_dynamic_para       = FLASH_PAGE_DYN_PARA;
    app_init_data.error_handler                     = app_flashmanager_error_handler;	
  
    err_code = flash_manager_init(&app_init_data);
    APP_ERROR_CHECK(err_code);
}

//Function to write ANT+BLE Permanant data to flash
void FlashWrite(pstorage_handle_t * handle,uint8_t *write_buffer, uint16_t size)
{
	uint32_t err_code;
	pstorage_handle_t dest_block;
	
	err_code = pstorage_clear(handle, size);
	APP_ERROR_CHECK(err_code);
	
	err_code = pstorage_block_identifier_get(handle,0,&dest_block);
	          
	// Write Permanent parameter Information
	err_code = pstorage_store(&dest_block,
			   (uint8_t *)write_buffer,
			   size,
			   0);
	APP_ERROR_CHECK(err_code);
}

void flash_ClearPage(pstorage_handle_t mp_flash_Para)
{
	uint32_t err_code = pstorage_clear(&mp_flash_Para,1024);
	APP_ERROR_CHECK(err_code);
}

/**@brief Initialize Variables
 *

 */
void variable_init(void)
{
    memset(&gst_AntFlashParams,0x00,sizeof(gst_AntFlashParams));
	memset(&gu8ar_AntBurstTxBuffer[CH_0][0], 0, sizeof(gu8ar_AntBurstTxBuffer[CH_0]));
	memset(&gu8ar_AntBurstRxBuffer[CH_0][0], 0, sizeof(gu8ar_AntBurstRxBuffer[CH_0]));
	memset(&gu8ar_AntBurstTxBuffer[CH_1][0], 0, sizeof(gu8ar_AntBurstTxBuffer[CH_1]));
	memset(&gu8ar_AntBurstRxBuffer[CH_1][0], 0, sizeof(gu8ar_AntBurstRxBuffer[CH_1]));
	memset(&gu8ar_AntTempBuffer1[0], 0, sizeof(gu8ar_AntTempBuffer1));
	
	//BLE
	BleBits.mu8_BleFlashUpdate = 0;
	BleBits.mu8_BleConnectionState = 0;
	BleBits.mu8_AntCH0_CloseByBLE = 0;
	BleBits.mu8_AntCH1_CloseByBLE = 0;
//#ifdef ANT 	
	//ANT
	b.mu8_AntFwdMsgToMaster = 0; 
	b.mu8_AntSendAsyncMsgToMaster = 0;
	b.mu8_AntSendResToMasterForRemoteReq = 0;
	b.mu8_AntSendNodeInfoToMaster = 0;
	b.mu8_AntSendMobileMsgToMaster_ch0 = 0;
	b.mu8_AntSendMobileMsgToMaster_ch1 = 0;
	b.mu8_AntStartNetwork = 0;
	b.mu8_AntChannelOpenState_ch0 = 0;
	b.mu8_AntChannelOpenState_ch1 = 0;
	b.mu8_AntchangeFreq = 0;
	b.mu8_AntDataEraseFlag = 0;
	b.mu8_AntNWIDReceived = 0;
	b.mu8_AntChannelCloseByANT = 0;
	b.mu8_AntResetChip = 0;
	b.mu8_EpochTimeValidFlag = 0;
	
	b.mu8_AntGetEpochTime = 1;
	gu8_AntGetEpochTimeMsgState=MESSAGE_INITIATED;
	b.mu8_AntSendResToMasterForRemoteReq=0;
//#endif
}

/**
 * @brief Setup ANT Node Channel 0
 *
 * Issues the following commands in a specific order:
 *
 * - assign channel
 * - set channel ID    
 * - open channel 
 */
//#ifdef ANT 	
static void antPx_Ch0SetUpAsSlave(void)
{
	uint32_t err_code;

	/* Set Network number and network Key */
	//err_code=sd_ant_network_address_set(ANT_CHANNEL_DEFAULT_NETWORK,&m_ant_network_key[0]);
	//APP_ERROR_CHECK(err_code);

	/* Set Channel Number */
	err_code = sd_ant_channel_assign(CHANNEL_0, 
									  CHANNEL_TYPE_SHARED_SLAVE, 
									  ANT_CHANNEL_DEFAULT_NETWORK, 
									  CHANNEL_0_ANT_EXT_ASSIGN);
	APP_ERROR_CHECK(err_code);

	/* Set Channel ID */
	err_code = sd_ant_channel_id_set(CHANNEL_0, 
									  gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum, 
									  gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevType, 
									  gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu8_DevTransType);
																		  
																		  
	APP_ERROR_CHECK(err_code);
	
	if(!gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress)
	{		
		#ifndef ANT_FREQ_AGILITY_ENABLE
		/* Set Channel Radio Frequency */
		err_code = sd_ant_channel_radio_freq_set(CHANNEL_0,CH_FREQUENCY0);
		APP_ERROR_CHECK(err_code);
		#else
		/* Set Frequency Agility */
		err_code = sd_ant_auto_freq_hop_table_set(CHANNEL_0,CH_FREQ0_AGLTY1,CH_FREQ0_AGLTY2,CH_FREQ0_AGLTY3);
		APP_ERROR_CHECK(err_code);
		#endif
		
		#ifdef ANT_DEBUG_ENABLE
		DEBUG("NO SA\r\n");
		#endif
	}
	else
	{
		#ifdef ANT_DEBUG_ENABLE
		DEBUG("Have SA\r\n");
		#endif
		
		/* If Parent Layer # is Zero then Set CH0_FREQUENCY */
		if(!gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber)
		{
			#ifndef ANT_FREQ_AGILITY_ENABLE
			/* Set Channel Radio Frequency */
			err_code = sd_ant_channel_radio_freq_set(CHANNEL_0,CH_FREQUENCY0);
			APP_ERROR_CHECK(err_code);
			#else
			/* Set Frequency Agility */
			err_code = sd_ant_auto_freq_hop_table_set(CHANNEL_0,CH_FREQ0_AGLTY1,CH_FREQ0_AGLTY2,CH_FREQ0_AGLTY3);
			APP_ERROR_CHECK(err_code);
			#endif
			
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("F0\r\n");
			#endif
		}
		else
		{
			#ifndef ANT_FREQ_AGILITY_ENABLE
				/* Set Channel Radio Frequency */
				err_code = sd_ant_channel_radio_freq_set(CHANNEL_0,CH_FREQUENCY1);
				APP_ERROR_CHECK(err_code);
			#else
				/* Set Frequency Agility */
				err_code = sd_ant_auto_freq_hop_table_set(CHANNEL_0,CH_FREQ1_AGLTY1,CH_FREQ1_AGLTY2,CH_FREQ1_AGLTY3);
				APP_ERROR_CHECK(err_code);
			#endif
			
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("F1\r\n");
			#endif
		}
	}
		
	/* Set Channel Period */
	err_code = sd_ant_channel_period_set(CHANNEL_0,CHANNEL_0_TX_CHANNEL_PERIOD);
	APP_ERROR_CHECK(err_code);
	
	/* Set TX Power level */
    err_code = sd_ant_channel_radio_tx_power_set(CHANNEL_0,RADIO_TX_POWER_LVL_4,RADIO_TXPOWER_TXPOWER_Pos4dBm);
	APP_ERROR_CHECK(err_code);
	
	/* Set RX Search Timeout */
	err_code = sd_ant_channel_rx_search_timeout_set(CHANNEL_0,4);  //4 * 2.5 Sec = 10 sec
	APP_ERROR_CHECK(err_code);
	
	/* Set RX Search Timeout */
	err_code = sd_ant_channel_low_priority_rx_search_timeout_set(CHANNEL_0,20);  //20 * 2.5 Sec = 50 sec
	APP_ERROR_CHECK(err_code);
	
    /* Open Channel */
    err_code = sd_ant_channel_open(CHANNEL_0);
    APP_ERROR_CHECK(err_code);
	
	b.mu8_AntChannelOpenState_ch0=1;
	
	#ifdef ANT_DEBUG_ENABLE
	DEBUG("CH0 SetUp\r\n");
	#endif
}
//#endif

/**
 * @brief Setup ANT Node Channel 1
 *
 * Issues the following commands in a specific order:
 *
 * - assign channel
 * - set channel ID    
 * - open channel 
 */
//#ifdef ANT 	
static void antPx_Ch1SetUpAsMaster(void)
{
	uint32_t err_code;

	/* Set Network number and network Key */
	/*err_code = sd_ant_network_address_set(ANT_CHANNEL_DEFAULT_NETWORK,&mu8ar_NetworkKey[0]);
	APP_ERROR_CHECK(err_code);
	*/
	
    /* Set Channel Number */
    err_code = sd_ant_channel_assign(CHANNEL_1, 
                                      CHANNEL_TYPE_SHARED_MASTER, 
                                      ANT_CHANNEL_DEFAULT_NETWORK, 
                                      CHANNEL_1_ANT_EXT_ASSIGN);
    APP_ERROR_CHECK(err_code);

    /* Set Channel ID */
    err_code = sd_ant_channel_id_set(CHANNEL_1, 
                                      gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum, 
                                      CHANNEL_1_CHAN_ID_DEV_TYPE, 
                                      CHANNEL_1_CHAN_ID_TRANS_TYPE);
    APP_ERROR_CHECK(err_code);

	#ifndef ANT_FREQ_AGILITY_ENABLE
	/* Set Channel Radio Frequency */
	err_code = sd_ant_channel_radio_freq_set(CHANNEL_1,CH_FREQUENCY1);
	APP_ERROR_CHECK(err_code);
	#else
	/* Set Frequency Agility */
	err_code = sd_ant_auto_freq_hop_table_set(CHANNEL_1,CH_FREQ1_AGLTY1,CH_FREQ1_AGLTY2,CH_FREQ1_AGLTY3);
	APP_ERROR_CHECK(err_code);
	#endif
	
	/* Set Channel Period */
    err_code = sd_ant_channel_period_set(CHANNEL_1,CHANNEL_1_TX_CHANNEL_PERIOD);
    APP_ERROR_CHECK(err_code);
	
	/* Set TX Power level */
	err_code = sd_ant_channel_radio_tx_power_set(CHANNEL_1,RADIO_TX_POWER_LVL_4,RADIO_TXPOWER_TXPOWER_Pos4dBm);
	APP_ERROR_CHECK(err_code);
	
    /* Open Channel */
    err_code = sd_ant_channel_open(CHANNEL_1);
    APP_ERROR_CHECK(err_code);
	
	b.mu8_AntChannelOpenState_ch1=1;
	
	#ifdef ANT_DEBUG_ENABLE
	DEBUG("CH1 SetUp\r\n");
	#endif
}
//#endif

/**
 * @brief Setup ANT Node Channel.
 *
 * Issues the following commands in a specific order:
 *
 * - assign channel
 * - set channel ID    
 * - open channel 
 */
//#ifdef ANT 	
void antPx_NodeSetUp(void)
{
	uint32_t err_code;
	
	antPx_Ch0SetUpAsSlave();
	
	if(gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress)
	{
		gu8ar_AntCHTxbuffer[CH_0][0]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_DeviceSharedAddress;
		gu8ar_AntCHTxbuffer[CH_0][1]=0x00;
		gu8ar_AntCHTxbuffer[CH_0][2]=0x00;
		gu8ar_AntCHTxbuffer[CH_0][3]=0x00;
		gu8ar_AntCHTxbuffer[CH_0][4]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[0];
		gu8ar_AntCHTxbuffer[CH_0][5]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[1];
		gu8ar_AntCHTxbuffer[CH_0][6]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[2];
		gu8ar_AntCHTxbuffer[CH_0][7]=gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8ar_DeviceSrNo[3];
		
		err_code = sd_ant_broadcast_message_tx(CHANNEL_0,8,&gu8ar_AntCHTxbuffer[CH_0][0]);
		APP_ERROR_CHECK(err_code);
		
		/* Open Channel_1 as a Master for Next Layer only if Device is Mains Powered */
		if((gst_AntFlashParams.mst_AntFlashHeader.mu8_NodePoweredOption==MAINS_POWERED) && (gst_AntFlashParams.mst_AntFlashHeader.mst_NodeInfo.mu8_ParentLayerNumber<(MAX_NETWORK_LAYER-1)))
		{
			/* Setup Channel_1 as a Master for Next Layer */
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("OPEN CH1\r\n");
			#endif
			
			antPx_Ch1SetUpAsMaster();
						
			antPx_SetDataOnCH1(gu8_AntRegState);
			
			//nrf_gpio_pin_set(LED_1);
		}
		else
		{
			//nrf_gpio_pin_set(LED_2);
		}
		
		gu8_AntSelfConnectionTimeout=SELF_CONNECTION_TIMEOUT;
	}	
	else
	{
//		nrf_gpio_pin_set(LED_0);
	}
}
//#endif
/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}

//************************************************************************
// Encrypt Serial Number for Advertisment
//************************************************************************
void EncryptSerialNumber()
{
	memset(&gu8_SerialNumberEncrypted[0], 0, sizeof(gu8_SerialNumberEncrypted));
	memcpy(gu8_SerialNumberEncrypted,gst_ConfigPermanentParams.mu8ar_SerialNumber,SERIAL_NUM_BYTES);
	gu8_SerialNumberEncrypted[SECURITY_LEVEL_ADV_PACKET_INDEX] = gst_BleConfigDynamicParams.mu8_Securitylevel;
	gu8_SerialNumberEncrypted[BIT_FIELD_ADV_PACKET_INDEX] = gu8_SerialNumberEncrypted[BIT_FIELD_ADV_PACKET_INDEX] | gst_BleConfigDynamicParams.mu8_SrDevStatus;
	gu8_SerialNumberEncrypted[AUDIT_TRIAL_COUNTER_ADV_PACKET_INDEX] = gst_BleConfigDynamicParams.mu8_TotalAuditTrailLogRecord;
	
	//SmartDeviceEncryptSerialNumber();			//TNY algorithem
	SmartDeviceSerialNumberEncryptBy_AES_CBC();	//
}

void Ul_gpio_int(void)
{	
	nrf_gpio_cfg_output(ANT_SEL);
	nrf_gpio_cfg_output(CSD);
	nrf_gpio_cfg_output(CPS);
	
	nrf_gpio_pin_set(ANT_SEL);
	nrf_gpio_pin_set(CSD);
	nrf_gpio_pin_set(CPS);
}

/**@brief Application main function.
 */
extern void decode_op_req_msg(uint8_t *data);

int main(void)
{
	uint32_t err_code; 
#ifdef BLE_PA
	Ul_gpio_int();
#endif
	variable_init();
	hardware_type = HARDWARE_NS_548;
	development_kit_gpio_init();  
	
	timers_init();
	gpiote_init();
	buttons_init();
	
    //Initialize S310 SoftDevice
	ble_ant_stack_init();
	
	//Inintialize pstorage(persisance )
	flash_data_manager_init();	
	
	// enable app_button as, the button events needed in both connected and disconnected state
	err_code = app_button_enable();		
	APP_ERROR_CHECK(err_code);
    	
	if(humidity_sensor_presence!=HUMIDITY_CONNECTED)
	{
		humidity_sensor_presence = HUMIDITY_NOT_CONNECTED;
	}	
	uart_init();				  
	devkit_Configuration();
	display_config_data();
	
	ppi_init();
	
	//Encrypt Serial Number for Advertisment
	EncryptSerialNumber();	
	
    //Initialize Bluetooth stack parameters.
    gap_params_init();    
    services_init();
	advertising_init();
    conn_params_init();
	sec_params_init();

	// Start Timer
	timers_start();			
	
	// ANT channel was closed due to a BLE disconnection, restart advertising
    advertising_start();
	
//#ifdef ANT
	// Setup ANT Node Channels
	if(gst_AntFlashParams.mst_AntFlashHeader.mst_ChannelIDPara.mu16_DevNum)
	{
		b.mu8_AntStartNetwork=1;
	}
//#endif
	
	//set GPREGRET resistor value to 0 after firmware upgrade completed
	sd_power_gpregret_set(0);   // [NTC]
	
	//Initialize TWI mater in mode
	twi_config_t my_twi_config = {.pinselect_scl = I2C_CLK_PIN_NUMBER,
								  .pinselect_sda = I2C_DATA_PIN_NUMBER,
								  .frequency     = TWI_FREQ_100KHZ,
								  .blocking_mode = TWI_BLOCKING_DISABLED};

	twi_master_init(&my_twi_config);
	
	//set GPREGRET resistor value to 0 after firmware upgrade completed
	sd_power_gpregret_set(0);
	/*****************************************************************/																	
	Initialize_Accelerometer();
	/*****************************************************************/
	co2_notify_flag=false;
	temperature_notify_flag= false;									  
	accelerometer_update_flg=false;
	device_type = 0;	  
    // Enter main loop.	
	DEBUG("\r\nUART Enabled");								  
    for (;;)
    {	
		devkitCommandExecution();
		
		if(b.mu8_AntStartNetwork==1)
		{
			if((BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION) && (gu8_AntFlashWriteState==NO_ANT_FLASH_WRITE_OPERATION))
			{
				antPx_NodeSetUp();
				b.mu8_AntStartNetwork=0;
			}
		}
		
		if(BleBits.mu8_BleConnectionState == FALSE)
		{
			if((b.mu8_AntChannelOpenState_ch0 == FALSE) && (b.mu8_AntChannelOpenState_ch1 == FALSE))
			{
				//BLE Flash Write 
				if(BleBits.mu8_BleFlashUpdate == BLE_FLASH_WRITE_INITIAT)
				{
					#ifdef BLE_STACK_EVENT_DEBUG
						DEBUG("\r\nBLE FLASH WRITE START\r\n");
					#endif
					FlashWrite(&mp_flash_BLE_DynPara,(uint8_t *)&gst_BleConfigDynamicParams,sizeof(st_BleFlashDynamicParams_t));
					BleBits.mu8_BleFlashUpdate = BLE_FLASH_WRITE_IN_PROGRESS;
				}
			}
			
			//ANT Flash Write 
			if(gu8_AntFlashWriteState==ANT_FLASH_WRITE_INITIAT)
			{
				#ifdef ANT_DEBUG_ENABLE
					DEBUG("\r\nANT FLASH WRITE START\r\n");
				#endif
				//Write Updated ANT Para to flash.
				FlashWrite(&mp_flash_ANT_DynPara,(uint8_t *)&gst_AntFlashParams,sizeof(st_AntFlashParmas_t));
				gu8_AntFlashWriteState=ANT_FLASH_WRITE_IN_PROGRESS;
			}
		}
		if(b.mu8_AntResetChip==1)
		{	
			#ifdef ANT_DEBUG_ENABLE
				DEBUG("SYSTEM RESET TRIGGER\r\n");
				DEBUG("FLASH STATE:");
				gen_PrintHex(BleBits.mu8_BleFlashUpdate);
				DEBUG("  ");
				gen_PrintHex(gu8_AntFlashWriteState);
				CRLF;
			#endif
			
			if((BleBits.mu8_BleConnectionState == FALSE) && (BleBits.mu8_BleFlashUpdate == NO_BLE_FLASH_WRITE_OPERATION) && (gu8_AntFlashWriteState==NO_ANT_FLASH_WRITE_OPERATION))
			{	
				b.mu8_AntResetChip=0;
				NVIC_SystemReset();
			}
		}
////  ANT ASCHYNC EVENTS
		AntAschyncEvent();
		//-------------------------------------------------------------	
		if(sl_rcvmsg_ok==TRUE)
		{
			sl_app_timer[UART_ACT_TIMER]=UART_ACT_TIME;					
			sl_config_by_uart();	
			sl_rcvmsg_ok=FALSE;		
		}
		else if(ble_app_sl_request==TRUE)
		{
			//decode_op_req_msg(&write_cmd[0]);
			bleP0_BleAppDevkitCommandExecution();
			ble_app_sl_request=FALSE;		
		}
		
		if(gu8_AntMsgReceivedOnBle==TRUE)
		{
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("Ble --> ANT\r\n");
			#endif
			antPx_SendBleMsgOnAnt(write_cmd,write_cmd[COMMAND_ID_LENGTH_INDEX]+2,gu8_TargetAntSerialNumber);
			gu8_AntMsgReceivedOnBle = FALSE;
		}
		else if(gu8_BleMsgReceivedOnAnt==TRUE)			//ANT --> BLE
		{
			#ifdef ANT_DEBUG_ENABLE
			DEBUG("ANT --> BLE\r\n");
			#endif
			memcpy(write_cmd,gu8p_AntBleMsgptr,gu8p_AntBleMsglen);
			switch(write_cmd[COMMAND_ID_INDEX])
			{
				case REMOTE_OPERATION_REQ: 

					if(write_cmd[REMOTE_OP_OPERA_TYPE_INDEX-1] != STATUS_REQ)
					{
						#ifdef BLE_STACK_EVENT_DEBUG
						DEBUG("while REMOTE_OPERATION_REQ\r\n");
						#endif

						write_cmd[OPERATION_STATUS_INDEX] = bleP0_ReceviedBleMsgOnAnt(write_cmd);						
					}
					else
					{
						#ifdef BLE_STACK_EVENT_DEBUG
						DEBUG("while REMOTE_STATUS_REQ\r\n");
						#endif
					}
					write_cmd[OPERATION_TYPE_INDEX] = write_cmd[REMOTE_OP_OPERA_TYPE_INDEX-1];
					//write_cmd[LOCK_STATUS_INDEX] = sl_current_status();
					write_cmd[COMMAND_ID_LENGTH_INDEX] = 3;
					write_cmd[COMMAND_ID_INDEX] = REMOTE_OPERATION_RES;

					gu8_AntMsgReceivedOnBle = TRUE;

					#ifdef ANT_DEBUG_ENABLE
					DEBUG("\r\nwhile Target ANT SrNo=");
					gen_PrintHexStr(&gu8_TargetAntSerialNumber[0],4);
					CRLF;
					#endif

					gu8_AntLockCurrentStatus = write_cmd[LOCK_STATUS_INDEX];

				break;
				
				case REMOTE_OPERATION_RES:
					
					ble_sr_device_state_update(&m_sr, write_cmd[LOCK_STATUS_INDEX]);
					
					if(write_cmd[OPERATION_TYPE_INDEX] != STATUS_REQ)
					{
						DEBUG("REMOTE_OPERATION_RES\r\n");
						DEBUG("REMOTE_OPERATION_RES\r\n");
						DEBUG("REMOTE_OPERATION_RES\r\n");
						DEBUG("REMOTE_OPERATION_RES\r\n");
						DEBUG("REMOTE_OPERATION_LOCK_UNLOCK_RES\r\n");
						ble_sr_operation_status_update(&m_sr, write_cmd[OPERATION_STATUS_INDEX]);
					}
					else
					{			
						#ifdef BLE_STACK_EVENT_DEBUG
						DEBUG("REMOTE_OPERATION_STATUS_RES\r\n");
						#endif
					}
					
				break;
				
				default:
					
					#ifdef BLE_STACK_EVENT_DEBUG
					DEBUG("ANT --> BLE fail\r\n");
					#endif

				break;
			}			
			
			gu8_BleMsgReceivedOnAnt = FALSE;
		}		
		power_manage();
	}
}


