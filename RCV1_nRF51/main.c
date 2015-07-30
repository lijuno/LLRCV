/*
The MIT License (MIT)

Copyright (c) 2015 Lijun (http://lijun.li/)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
 

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer_appsh.h"
#include "ble_error_log.h"
//#include "app_gpiote.h"
//#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "bsp.h"
#include "ble_gap.h"
#include "nrf_delay.h"
#include "ll_ble_otats.h"
#include "ll_common.h"
#include "ll_ble_rcv1s.h"
#include "nrf_pwm.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                     "LLRCV1"                             /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL_IN_MS        100
#define APP_ADV_INTERVAL_TICKS        APP_TIMER_TICKS(APP_ADV_INTERVAL_IN_MS, APP_TIMER_PRESCALER)    
#define APP_ADV_TIMEOUT_IN_SECONDS    60                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

//#define DEBUG_MODE     // uncomment this line to enable debugging LED indicators
// #define WAKEUP_BUTTON_PIN               8                                /**< Button used to wake up the application. */
#define ADVERTISING_LED_PIN_NO           9                                  /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN_NO            10                                   /**< Is on when device has connected. */

// Motor parameters
#define MOTOR_L   26
#define MOTOR_R   27
#define MOTOR_L_PPI_CH  0
#define MOTOR_R_PPI_CH  1
#define MOTION_JOB_QUEUE_SIZE  10
#define MOTION_PARAMS_DEFAULT {.properties=0x0, .duration=0, .motor_l_speed = 0, .motor_r_speed = 0}

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ll_ble_otats_t                   m_otats;
static ll_ble_rcv1s_t                    m_rcv1s;

static app_timer_id_t m_motor_pwm_duration_timer;  

static uint8_t ind_mj_q_head; // array index of the head element in motion job queue (the place the next element is to be dequeued)
static uint8_t ind_mj_q_tail; // array index of the tail element in motion job queue (the place the next element is to be enqueued to)
static uint8_t len_mj_q;   // current length of motion job queue; ind_mj_q_tail == (ind_mj_q_head + len_mj_q) % MOTION_JOB_QUEUE_SIZE;

// Data structure for motor moving
typedef struct
{
	uint8_t properties;   // moving directions for left and right motors; acceraltion, etc
	uint16_t duration;  // unit: 10 ms
	uint8_t motor_l_speed;  // 0~255
	uint8_t motor_r_speed;   // 0~255
} ll_rcv1_motor_params_t;


static ll_rcv1_motor_params_t motion_job_queue[MOTION_JOB_QUEUE_SIZE];
static uint8_t flag_pwm_timer_running = 0;  // 0 for full stop (no PWM timer running); 
                                           // 1 for PWM timer is still running (even if there are no PWM waveforms) so that PWM doesn't need to be re-initialized

// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);

/**@brief Function for error handling, which is called when an error has occurred.
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
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    //NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
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


// Queue methods; a naive implementation of queue
// Todo: use linked lists

static void motion_job_queue_clear(void)
{  // Queue initialization
	ind_mj_q_head = 0;
	ind_mj_q_tail = 0;
	len_mj_q = 0;
	uint8_t ii;
	for (ii=0;ii<MOTION_JOB_QUEUE_SIZE; ii++) 
	{
		motion_job_queue[ii].properties=0x0;
		motion_job_queue[ii].duration = 0;
		motion_job_queue[ii].motor_l_speed = 0;
		motion_job_queue[ii].motor_r_speed = 0;
	}
}

static bool mj_queue_is_empty(void)
{
	return (len_mj_q <=0);
}

static bool mj_queue_is_full(void)
{
	return (len_mj_q >=MOTION_JOB_QUEUE_SIZE);
}	

static uint32_t mj_enqueue(ll_rcv1_motor_params_t * p_rcv1_motion_params)
{
	if (!(mj_queue_is_full()))
	{  
		// kinda wasteful here since a full copy is being done
		motion_job_queue[ind_mj_q_tail].properties = p_rcv1_motion_params->properties;
		motion_job_queue[ind_mj_q_tail].duration = p_rcv1_motion_params->duration;
		motion_job_queue[ind_mj_q_tail].motor_l_speed = p_rcv1_motion_params->motor_l_speed;
		motion_job_queue[ind_mj_q_tail].motor_r_speed = p_rcv1_motion_params->motor_r_speed;
		len_mj_q ++;
		ind_mj_q_tail = (ind_mj_q_tail +1) % MOTION_JOB_QUEUE_SIZE;
		return 0;
	}
	else
		return 0xFFFFFFFF;
}

static uint32_t mj_dequeue(ll_rcv1_motor_params_t ** pp_mj)
{
	if (!(mj_queue_is_empty()))
	{
		*pp_mj = &motion_job_queue[ind_mj_q_head];
		len_mj_q --;
		ind_mj_q_head = (ind_mj_q_head + 1) % MOTION_JOB_QUEUE_SIZE; // ind_mj_q_tail doesn't change
		return 0;
	}
	else
		return 0xFFFFFFFF;
}
	
// Define OTA characteristic write handler, called by function in ll_ble_otats.c
static void otats_trig_char_write_handler(ll_ble_otats_t * p_otats, uint8_t * data)
{
	if (sizeof(data)/sizeof(data[0])==4) {
		  uint32_t cmd = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
			if (cmd == OTA_TRIG_CMD)
			{
					sd_ble_gap_disconnect( p_otats->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION );
					softdevice_handler_sd_disable();
				#ifdef DEBUG_MODE
				  nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);  // turn off connection indicator
				#endif
					nrf_delay_ms(4000);   // wait for BLE central to be disconnected.
					NVIC_SystemReset();
			}
	}
}

// RCV1 motor methods below

static void rcv1_motors_init(void)
{
	// Init PWM
		nrf_pwm_config_t pwm_config = PWM_DEFAULT_CONFIG;
    pwm_config.mode             = PWM_MODE_MTR_255;  // 8-bit resolution, 31kHz PWM frequency
    pwm_config.num_channels     = 2;
    pwm_config.gpio_num[MOTOR_L_PPI_CH]      = MOTOR_L;
	  pwm_config.gpio_num[MOTOR_R_PPI_CH]      = MOTOR_R;
	
    nrf_pwm_init(&pwm_config);
}

static void rcv1_motion_execute(void)
{
	uint32_t err_code;
	
	ll_rcv1_motor_params_t *p_mj;
	mj_dequeue(&p_mj);
	nrf_pwm_set_value(MOTOR_L_PPI_CH, p_mj->motor_l_speed);
	nrf_pwm_set_value(MOTOR_R_PPI_CH, p_mj->motor_r_speed);

	err_code = app_timer_start(m_motor_pwm_duration_timer, APP_TIMER_TICKS((p_mj->duration)*10, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
}


static void rcv1_motion_pause(void)
{}


static void rcv1_full_shutdown(void)
{
	// Fully shut down PWM immediately and clear the queue
	nrf_pwm_set_enabled(false);
	nrf_gpio_cfg_output(MOTOR_L);
	nrf_gpio_pin_clear(MOTOR_L);
	nrf_gpio_cfg_output(MOTOR_R);
	nrf_gpio_pin_clear(MOTOR_R);
	
	// Shut down PWM duration timer (in case it is still running)
	uint32_t err_code;
	err_code = app_timer_stop(m_motor_pwm_duration_timer);
	APP_ERROR_CHECK(err_code);	
	
	flag_pwm_timer_running = 0;
	
	motion_job_queue_clear();
}

// Define characteristic write handlers for RCV1 service 
static void rcv1s_mode_char_write_handler(ll_ble_rcv1s_t * p_rcv1s, uint8_t * data)
{
	if ((data[0]==RCV1S_MODE_CMD_EXECUTE_QUEUE) && !(mj_queue_is_empty())) // Execute the full queue if not empty
	{
		if (flag_pwm_timer_running==0) 
		{
			rcv1_motors_init();
			flag_pwm_timer_running = 1;
		}

		rcv1_motion_execute();  // Trigger executing job queue
	}	
	else if (data[0]==RCV1S_MODE_CMD_START_LIVE_STREAM)
	{  // Live stream code here (maintain queue size of 1; only update queue when asked to)
	}
	else if (data[0]==RCV1S_MODE_CMD_PAUSE)
	{
		rcv1_motion_pause();
	}
	else if (data[0]==RCV1S_MODE_CMD_FULL_SHUTDOWN)
  {  
    rcv1_full_shutdown();
	}	
}

static void rcv1s_moving_params_char_write_handler(ll_ble_rcv1s_t * p_rcv1s, uint8_t * data)
{
	// Data format (LSB to MSB): {cmd_type (high 4 bit) | direction (low 4 bit), 
	//                            left_speed (8 bit), right_speed (8 bit), 
	//                            left_time (16 bit), right_time (16 bit)}
	uint8_t cmd_type = data[0]>>4;
	if (cmd_type==0x00)
	{
		// Push motion parameters to the queue
		if (len_mj_q<MOTION_JOB_QUEUE_SIZE)
		{
			ll_rcv1_motor_params_t mj;
			mj.properties = data[0] & 0x0f;
			mj.duration = data[1] | (data[2]<<8);
			mj.motor_l_speed = data[3];
			mj.motor_r_speed = data[4];
			mj_enqueue(&mj);
		}
		else
		{  // some error handling
		}
	}
}

// Define motor PWM duration timer handlers
static void motor_pwm_timer_timeout_handler(void * p_context)
{ 
	if(!(mj_queue_is_empty())) rcv1_motion_execute();  
	else
	{
	  // Stop PWM waveform, but TIMER is still alive
	  nrf_pwm_set_value(MOTOR_L_PPI_CH, 0);
  	nrf_pwm_set_value(MOTOR_R_PPI_CH, 0);
	}
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void gpio_init(void)
{
	#ifdef DEBUG_MODE
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
	  nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
	  nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
	#endif
	nrf_gpio_cfg_output(MOTOR_L);
	nrf_gpio_cfg_output(MOTOR_R);
	nrf_gpio_pin_clear(MOTOR_L);
	nrf_gpio_pin_clear(MOTOR_R);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	uint32_t err_code;
	
    // Initialize timer module, making it use the scheduler
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
	
		err_code = app_timer_create(&m_motor_pwm_duration_timer, APP_TIMER_MODE_SINGLE_SHOT, motor_pwm_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);	
	
}


/**@brief Function for the GAP initialization.
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

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;   // not to include appearance to save for ADV data space
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
	
// Build scan response data
//    ble_advdata_t scanrsp;
//    ble_uuid_t adv_uuids[] = {{OTATS_UUID_SERVICE, m_otats.uuid_type}};  
//    memset(&scanrsp, 0, sizeof(scanrsp));
//    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
//    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
//    err_code = ble_advdata_set(&advdata, &scanrsp);
    err_code = ble_advdata_set(&advdata, NULL);   
		APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ll_ble_otats_init_t otats_init;
	  ll_ble_rcv1s_init_t  rcv1s_init;
    
    otats_init.otats_trig_char_write_handler = otats_trig_char_write_handler;
	  rcv1s_init.rcv1s_mode_char_write_handler = rcv1s_mode_char_write_handler;
	  rcv1s_init.rcv1s_moving_params_char_write_handler = rcv1s_moving_params_char_write_handler;
    
    err_code = ll_ble_otats_init(&m_otats, &otats_init);
    APP_ERROR_CHECK(err_code);
    err_code = ll_ble_rcv1s_init(&m_rcv1s, &rcv1s_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Function for handling the Connection Parameters Module.
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

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
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


/**@brief Function for starting timers.
*/
static void timers_start(void)
{
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL_TICKS;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
	#ifdef DEBUG_MODE
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
	#endif
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
		static ble_gap_master_id_t p_master_id;
		static ble_gap_sec_keyset_t keys_exchanged;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
				#ifdef DEBUG_MODE
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
				#endif
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
// Be careful, if disabling app_button_enable(), should also disable the GPIOTE functionality because app_button_enable() uses GPIOTE
//            err_code = app_button_enable();  
//            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
				#ifdef DEBUG_MODE
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
				#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

//            err_code = app_button_disable();
//            APP_ERROR_CHECK(err_code);
            
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params,&keys_exchanged);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0,BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            //p_enc_info = keys_exchanged.keys_central.p_enc_key
						
            if (p_master_id.ediv == p_ble_evt->evt.gap_evt.params.sec_info_request.master_id.ediv)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, &keys_exchanged.keys_central.p_enc_key->enc_info, &keys_exchanged.keys_central.p_id_key->id_info, NULL);
                APP_ERROR_CHECK(err_code);
								p_master_id.ediv = p_ble_evt->evt.gap_evt.params.sec_info_request.master_id.ediv;
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL,NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
							   advertising_start();
							// Uncomment below to use external wakeup on pin WAKEUP_BUTTON_PIN
//                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
//                // Configure buttons with sense level low as wakeup source.
//                nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
//                                         BUTTON_PULL,
//                                         NRF_GPIO_PIN_SENSE_LOW);
//                
//                // Go to system-off mode (this function will not return; wakeup will cause a reset)                
//                err_code = sd_power_system_off();
//                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	// Don't forget to register services here to receive BLE events
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ll_ble_otats_on_ble_evt(&m_otats, p_ble_evt);
	  ll_ble_rcv1s_on_ble_evt(&m_rcv1s, p_ble_evt);
}


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


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    // SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
	  SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_gap_addr_t addr;
    
    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
    sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


//static void button_event_handler(uint8_t pin_no, uint8_t button_action)
//{
//    uint32_t err_code;
//    
//    switch (pin_no)
//    {
//        case LEDBUTTON_BUTTON_PIN_NO:
//            err_code = ble_otats_on_button_change(&m_otats, button_action);
//            if (err_code != NRF_SUCCESS &&
//                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//                err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//            break;

//        default:
//            APP_ERROR_HANDLER(pin_no);
//            break;
//    }
//}

/**@brief Function for initializing the GPIOTE handler module.
 */
//static void gpiote_init(void)
//{
//    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
//}




/**@brief Function for initializing the button handler module.
 */
//static void buttons_init(void)
//{
//	// This is how to configure a button
//    // Note: Array must be static because a pointer to it will be saved in the Button handler
//    //       module.
//    static app_button_cfg_t buttons[] =
//    {
//        {WAKEUP_BUTTON_PIN, false, BUTTON_PULL, NULL},
//        {LEDBUTTON_BUTTON_PIN_NO, false, BUTTON_PULL, button_event_handler}
//    };

//    app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY);
//}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize
	  motion_job_queue_clear();
    gpio_init();
    timers_init();
//    gpiote_init();
//    buttons_init();
    ble_stack_init();
    scheduler_init();    
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();

    // Start execution
    timers_start();
    advertising_start();

    // Enter main loop
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

/**
 * @}
 */
