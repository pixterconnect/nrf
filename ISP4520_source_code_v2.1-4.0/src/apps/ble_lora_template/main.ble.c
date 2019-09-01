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
//#include "ble_hci.h"
#include "ble_err.h"
//#include "ble_srv_common.h"
#include "ble_advdata.h"
//#include "ble_conn_params.h"
//#include "ble_conn_state.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "board.h"
//#include "loramachelper.h"

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
#define APP_ADV_INTERVAL             	80                                      ///< The advertising interval (in units of 0.625 ms. This value corresponds to 50 ms).
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                       ///< The advertising timeout (in units of seconds).
//#define MIN_CONN_INTERVAL_MS            100                                     ///< Minimum acceptable connection interval in seconds.
//#define MAX_CONN_INTERVAL_MS            500                                     ///< Maximum acceptable connection interval in seconds.
//#define SLAVE_LATENCY                   0                                       ///< Slave latency.
//#define CONN_SUP_TIMEOUT_MS             3200                                    ///< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
//#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000)                   ///< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
//#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  ///< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
//#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       ///< Number of attempts before giving up the connection parameter negotiation.
#define BLE_TX_POWER_LEVEL              0                                       ///< Tx power in dBm. 

// Static vars
NRF_BLE_GATT_DEF(m_gatt);                                           ///< GATT module instance.
BLE_ADVERTISING_DEF(m_advertising);                                 ///< Advertising module instance.
APP_TIMER_DEF(lora_tx_timer_id);                                    ///< LoRa tranfer timer instance.


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
    //err_code = app_timer_create(&lora_tx_timer_id, APP_TIMER_MODE_REPEATED, tx_lora_periodic_handler);
    //VERIFY_SUCCESS(err_code);

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
    ble_gap_conn_sec_mode_t sec_mode;

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    VERIFY_SUCCESS(err_code);
    err_code = sd_ble_gap_tx_power_set(BLE_TX_POWER_LEVEL);
    VERIFY_SUCCESS(err_code);
	
    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    VERIFY_SUCCESS(err_code);
	 
    return NRF_SUCCESS;
}

/**@brief Function for initializing the GATT module.
 */
//static uint32_t gatt_init (void)
//{
//    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
//    VERIFY_SUCCESS(err_code);
//
//    return NRF_SUCCESS;
//}


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

//    tx_power_level = -20;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type               	= BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      	= false;
    init.advdata.flags                   	= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
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

	
    err_code = advertising_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("advertising_init failed - %d", err_code);
        return err_code;
    }

	
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

