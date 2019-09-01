 /******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    06-november-2018
 * @brief   LoRaMac classA project main file.
 *          This file contains initialization for starting device in LoRaMac classA 
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

// Standards
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// nRF
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"

// LoRa
#include "radio.h"
#include "board.h"

//logs
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// LoRaMac
#include "loramachelper.h"

#define SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60                              /**< Maximum number of events in the scheduler queue. */

#define LORAWAN_ADR_ON              1           /**< LoRaWAN Adaptive Data Rate enabled (the end-device should be static here). */  
#define LORAWAN_ADR_OFF             0           /**< LoRaWAN Adaptive Data Rate disabled. */  

#define LORAWAN_APP_DATA_BUFF_SIZE  64		/**< Size of the data to be transmitted. */  
#define LORAWAN_APP_TX_DUTYCYCLE    10000 	/**< Defines the application data transmission duty cycle. 10s, value in [ms]. */  
#define APP_TX_DUTYCYCLE_RND        1000 	/**< Defines a random delay for application data transmission duty cycle. 1s, value in [ms]. */  


// Foward declaration
static void lorawan_has_joined_handler (void);
static void lorawan_rx_handler (lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler (DeviceClass_t Class);
static void send_lora_frame (void);

APP_TIMER_DEF(lora_tx_timer_id);                                                    ///< LoRa tranfer timer instance.
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];                  ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0 ,0, 0, 0};       ///< Lora user application data structure.

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK, LORAWAN_DEFAULT_TX_POWER};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {    BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                            lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler};



/**@brief LoRa function for handling HasJoined event.
 */
static void lorawan_has_joined_handler (void)
{
#if (OVER_THE_AIR_ACTIVATION != 0)
    NRF_LOG_INFO("Network Joined");
#endif
    lmh_class_request(CLASS_A);
}

/**@brief Function for handling LoRaWan received data from Gateway
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
                // YOUR_JOB: Take action on received data 
                break;
		
            default:
                break;
    }
}

static void lorawan_confirm_class_handler (DeviceClass_t Class)
{
    NRF_LOG_INFO("switch to class %c done", "ABC"[Class]);

    // Informs the server that switch has occurred ASAP
    m_lora_app_data.buffsize = 0;
    m_lora_app_data.port = LORAWAN_APP_PORT;
    lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

static void send_lora_frame (void)
{
    if (lmh_join_status_get() != LMH_SET)
    {
        //Not joined, try joining again
        lmh_join();
        return;
    }

    uint32_t i = 0;
    m_lora_app_data.port = LORAWAN_APP_PORT;
    m_lora_app_data.buffer[i++] = 'H';
    m_lora_app_data.buffer[i++] = 'e';
    m_lora_app_data.buffer[i++] = 'l';
    m_lora_app_data.buffer[i++] = 'l';
    m_lora_app_data.buffer[i++] = 'o';
    m_lora_app_data.buffer[i++] = ' ';
    m_lora_app_data.buffer[i++] = 'w';
    m_lora_app_data.buffer[i++] = 'o';
    m_lora_app_data.buffer[i++] = 'r';
    m_lora_app_data.buffer[i++] = 'l';
    m_lora_app_data.buffer[i++] = 'd';
    m_lora_app_data.buffer[i++] = '!';
    m_lora_app_data.buffsize = i;
  
    lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
    if (error == LMH_SUCCESS)
    {	}
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
static void power_manage (void)
{
    __WFE();
    __SEV();
    __WFE();
}


/**@brief Function for application main entry.
 */
int main (void)
{	
    uint32_t err_code;
	
    // Initialize logs.
    log_init();
    NRF_LOG_INFO("LoRaWan Class A example started.");
	
    // Initialize clocks
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    nrf_drv_clock_hfclk_request(NULL);

    // Enable nRF52 DCDC
    NRF_POWER->DCDCEN = 1;

    // Initialize Scheduler and timer
    err_code = timers_init();
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("timers_init failed - %d", err_code);
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

    // Start Join procedure
    lmh_join();
    app_timer_start(lora_tx_timer_id, APP_TIMER_TICKS(LORAWAN_APP_TX_DUTYCYCLE), NULL);
		
    // Enter main loop.
    for (;;)
    {
        app_sched_execute(),
        Radio.IrqProcess();
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }		
    }
}

