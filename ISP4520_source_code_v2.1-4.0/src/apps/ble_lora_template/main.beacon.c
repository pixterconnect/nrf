 /******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @version V2.0.0
 * @date    5-febuary-2019
 * @brief  Template project main file.
 *
 * Template of project of using BLE and LoRaWan. 
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "fds.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_ble_gatt.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_button.h"
#include "ble_advertising.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_err.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "board.h"
#include "loramachelper.h"

// Timer & scheduler
#define APP_TIMER_PRESCALER         0                                           ///<  Value of the RTC1 PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE     75                                          ///<  Size of timer operation queues.
#define SCHED_QUEUE_SIZE            20                                          ///< Maximum number of events in the scheduler queue.
#define SCHED_MAX_EVENT_DATA_SIZE   MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, NRF_SDH_BLE_EVT_BUF_SIZE)  ///<  Maximum size of scheduler events.

// BLE
#define DEAD_BEEF                       0xDEADBEEF                              ///< Value used as error code on stack dump, can be used to identify stack location on stack unwind.
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    ///< Reply when unsupported features are requested.
#define DEVICE_NAME                     "ISP_Template"                          ///< Name of device. Will be included in the advertising data.
#define APP_BLE_OBSERVER_PRIO           3                                       ///< Application's BLE observer priority. You shouldn't need to modify this value.
#define APP_BLE_CONN_CFG_TAG            1                                       ///< A tag identifying the SoftDevice BLE configuration. */
#define APP_ADV_INTERVAL             	480                                     ///< The advertising interval (in units of 0.625 ms. This value corresponds to 300 ms).
#define APP_ADV_TIMEOUT_IN_SECONDS      20                                       ///< The advertising timeout (in units of seconds).
#define MIN_CONN_INTERVAL_MS            100                                     ///< Minimum acceptable connection interval in seconds.
#define MAX_CONN_INTERVAL_MS            500                                     ///< Maximum acceptable connection interval in seconds.
#define SLAVE_LATENCY                   0                                       ///< Slave latency.
#define CONN_SUP_TIMEOUT_MS             3200                                    ///< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000)                   ///< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  ///< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       ///< Number of attempts before giving up the connection parameter negotiation.
#define BLE_TX_POWER_LEVEL              0                                       ///< Tx power in dBm. 

// LoRaWan
#define LORAWAN_APP_DATA_BUFF_SIZE	64                                      ///< Size of the data to be transmitted.
#define LORAWAN_APP_TX_DUTYCYCLE	10000                                   ///< Defines the application data transmission duty cycle. 10s, value in [ms].
#define LORAWAN_JOINREQ_NBTRIALS        3                                       ///< Number of trials for the join request.
#define LORAWAN_ADR_ON                  1                                       ///< LoRaWAN Adaptive Data Rate enabled (the end-device should be static here).
#define LORAWAN_ADR_OFF            	0                                       ///< LoRaWAN Adaptive Data Rate disabled.

// Foward declaration
static void lorawan_has_joined_handler (void);
static void lorawan_rx_handler (lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler (DeviceClass_t Class);
static void send_lora_frame (void);
static void tx_lora_periodic_handler (void * unused);

// Static vars
NRF_BLE_GATT_DEF(m_gatt);                                           ///< GATT module instance.
BLE_ADVERTISING_DEF(m_advertising);                                 ///< Advertising module instance.
APP_TIMER_DEF(lora_tx_timer_id);                                    ///< LoRa tranfer timer instance.

/* TO DO: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */


/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK, LORAWAN_JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {    BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                            lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler};

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];                              ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0 ,0, 0, 0};                   ///< Lora user application data structure.

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback (uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief LoRa function for handling HasJoined event.
 */
static void lorawan_has_joined_handler ()
{
#if (OVER_THE_AIR_ACTIVATION != 0)
    NRF_LOG_INFO("Network Joined");
#endif
    lmh_class_request(CLASS_A);
}

/**@brief LoRa function for handling received data from server
 *
 * @param[in] app_data  Pointer to rx data
 */
static void lorawan_rx_handler (lmh_app_data_t *app_data)
{
    NRF_LOG_DEBUG("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d", app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

    switch (app_data->port)
    {
        case 3:
            // Port 3 switches the class
            if (app_data->buffsize == 1)
            {
                switch (app_data->buffer[0])
                {
                    case 0:
                        lmh_class_request(CLASS_A);
                        break;

                    case 1:
                        lmh_class_request(CLASS_B);
                        break;

                    case 2:
                        lmh_class_request(CLASS_C);
                        break;
					
                    default:
                        break;
                }
            }
            break;

            case LORAWAN_APP_PORT:
                // TO DO: Take action on received LoRaWAN data 
                break;
		
            default:
                break;
    }
}

/**@brief LoRa function for handling class change
 *
 * @param[in] Class  class of the device
 */
static void lorawan_confirm_class_handler (DeviceClass_t Class)
{
    NRF_LOG_INFO("switch to class %c done", "ABC"[Class]);

    // Informs the server that switch has occurred ASAP
    m_lora_app_data.buffsize = 0;
    m_lora_app_data.port = LORAWAN_APP_PORT;
    lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
static void tx_lora_periodic_handler (void * unused)
{
    send_lora_frame();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static uint32_t timers_init (void)
{
    ret_code_t err_code;
	
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Initialize timer module.
    err_code = app_timer_init();
    VERIFY_SUCCESS(err_code);
	
    // Initialize timers
    err_code = app_timer_create(&lora_tx_timer_id, APP_TIMER_MODE_REPEATED, tx_lora_periodic_handler);
    VERIFY_SUCCESS(err_code);

     /* TO DO: Create any timers to be used by the application.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
     */
	
    return NRF_SUCCESS;
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler (ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
        } break;

        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static uint32_t ble_stack_init (void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    VERIFY_SUCCESS(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    VERIFY_SUCCESS(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    VERIFY_SUCCESS(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
	
    return NRF_SUCCESS;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static uint32_t gap_params_init (void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    VERIFY_SUCCESS(err_code);

  /* TO DO: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MSEC_TO_UNITS(MIN_CONN_INTERVAL_MS, UNIT_1_25_MS);
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS(MAX_CONN_INTERVAL_MS, UNIT_1_25_MS);;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(CONN_SUP_TIMEOUT_MS, UNIT_10_MS);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    VERIFY_SUCCESS(err_code);

    err_code = sd_ble_gap_tx_power_set(BLE_TX_POWER_LEVEL);
    VERIFY_SUCCESS(err_code);
	
    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    VERIFY_SUCCESS(err_code);
	 
    return NRF_SUCCESS;
}

/**@brief Function for initializing the GATT module.
 */
static uint32_t gatt_init (void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler (uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static uint32_t conn_params_init (void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    VERIFY_SUCCESS(err_code);
	
    return NRF_SUCCESS;
}

/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void adv_evt_handler (ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break; 

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Idle advertising.");
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static uint32_t advertising_init (void)
{
    ret_code_t err_code;
    ble_advertising_init_t init;
    // TO DO: Use UUIDs for service(s) used in your application.
    //ble_uuid_t adv_uuid =  {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE};

//    int8_t tx_power_level;

//    tx_power_level = -20;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type               	= BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      	= false;
    init.advdata.flags                   	= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    //init.advdata.uuids_complete.uuid_cnt 	= 0;
    //init.advdata.uuids_complete.p_uuids  	= &adv_uuid;
    init.config.ble_adv_fast_enabled  		= true;
    init.config.ble_adv_fast_interval 		= APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  		= APP_ADV_TIMEOUT_IN_SECONDS;
    init.evt_handler = adv_evt_handler;


//    init.advdata.p_tx_power_level 			= &tx_power_level;

    err_code = ble_advertising_init(&m_advertising, &init);
    VERIFY_SUCCESS(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
	
    return NRF_SUCCESS;
}

/**@brief Function for handling the YYY Service events.
 * TO DO implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/

/**@brief Function for initializing services that will be used by the application.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
static uint32_t services_init (void)
{
    /* TO DO: Add code to initialize the services used by the application.
       ret_code_t                         err_code;
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */

    return NRF_SUCCESS;
}

static void send_lora_frame (void)
{
    uint32_t err_code;

    if (lmh_join_status_get() != LMH_SET)
    {
        lmh_join();
        return; 
    }

    m_lora_app_data.port = LORAWAN_APP_PORT;
    /* TO DO: Fill m_lora_app_data with the payload to the LoRaWAN Network
     */
    strcpy(m_lora_app_data.buffer, "Hello world!");
    m_lora_app_data.buffsize = 12;
  
    lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter (void)
{
    ret_code_t err_code;

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init (void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for the Power manager.
 */
 #define FPU_EXCEPTION_MASK 0x0000009F 
static void power_manage (void)
{
    /* Clear exceptions and PendingIRQ from the FPU unit */
    __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));      
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);
            
    /* Wait for events */
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main (void)
{
    uint32_t err_code;

	
    // Initialize logs.
    log_init();
    NRF_LOG_INFO("BLE LoRa template started.");
	
    // Initialize clocks
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
	
    // Initialize Scheduler and timer
    err_code = timers_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("timers_init failed - %d", err_code);
        return err_code;
    }
		
    // Initialize BLE
    err_code = ble_stack_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("ble_stack_init failed - %d", err_code);
        return err_code;
    }

    err_code = gap_params_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("gap_params_init failed - %d", err_code);
        return err_code;
    }

//    err_code = conn_params_init();
//    if (err_code != NRF_SUCCESS)
//    {
//        NRF_LOG_ERROR("conn_params_init failed - %d", err_code);
//        return err_code;
//    }
	
//    err_code = services_init();
//    if (err_code != NRF_SUCCESS)
//    {
//        NRF_LOG_ERROR("services_init failed - %d", err_code);
//        return err_code;
//    }
	
    err_code = advertising_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("advertising_init failed - %d", err_code);
        return err_code;
    }
	
    // Initialize LoRa chip.
    err_code = lora_hardware_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("lora_hardware_init failed - %d", err_code);
        return err_code;
    }

    // Initialize LoRaWan
    err_code = lmh_init(&lora_callbacks, lora_param_init);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("lmh_init failed - %d", err_code);
        return err_code;
    }
	
    // Start LoRaWAN Join procedure
    //lmh_join();
    //app_timer_start(lora_tx_timer_id, APP_TIMER_TICKS(LORAWAN_APP_TX_DUTYCYCLE), NULL);
	
    // Start BLE
    ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	
    // Enter main loop.
    for (;;)
    {
        Radio.IrqProcess();
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}

