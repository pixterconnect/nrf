 /**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "nrf_drv_clock.h"

//#include "boards.h"
#include "bsp.h"
//#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
//#include "ble_lbs_c.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "battery_voltage.h"
#include "flashcg.h"
#include "fds.h"

// LoRa
#include "radio.h"
#include "board.h"

// LoRaMac
#include "loramachelper.h"

// Timer & scheduler
#define APP_TIMER_PRESCALER         0                                           ///<  Value of the RTC1 PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE     75                                          ///<  Size of timer operation queues.
#define SCHED_QUEUE_SIZE            20                                          ///< Maximum number of events in the scheduler queue.
#define SCHED_MAX_EVENT_DATA_SIZE   MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, NRF_SDH_BLE_EVT_BUF_SIZE)  ///<  Maximum size of scheduler events.

//#define BLE_SCAN_DUTYCYCLE	60000                                   ///< Defines the application scan ble duty cycle. 60s, value in [ms].


//#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
//#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1                     /**< Connected LED will be on when the device is connected. */
//#define LEDBUTTON_LED                   BSP_BOARD_LED_2                     /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define SCAN_INTERVAL                   200//0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     180//0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                    20                             /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE                     2                                   /**< Size of a UUID, in bytes. */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

// LoRaWan
#define LORAWAN_APP_DATA_BUFF_SIZE	64                                      ///< Size of the data to be transmitted.
#define LORAWAN_APP_TX_DUTYCYCLE	10000                                   ///< Defines the application data transmission duty cycle. 10s, value in [ms].
#define LORAWAN_JOINREQ_NBTRIALS        3                                       ///< Number of trials for the join request.
#define LORAWAN_ADR_ON                  1                                       ///< LoRaWAN Adaptive Data Rate enabled (the end-device should be static here).
#define LORAWAN_ADR_OFF            	0                                       ///< LoRaWAN Adaptive Data Rate disabled.

// LoraWan Downlink Port
#define DWNLINK_CONFIG                  32                                      /// device reconfiguration

#define RESET_msg                       0x00
#define KEEPALIVE_msg                   0x10
#define BLE_COUNT_msg                   0x11
#define BLE_RSSI_msg                    0x12

// BLE scan
#define table_MAX 	40 		// max number of entries

// PicoTag operation mode
#define BEACON_MODE     0
#define SCANNER_MODE    1


// Foward declaration
static void lorawan_has_joined_handler (void);
static void lorawan_rx_handler (lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler (DeviceClass_t Class);
static void send_lora_frame (void);
static void tx_lora_periodic_handler (void * unused);
static void fds_evt_handler(fds_evt_t const * p_evt);

// Foward declaration
static void scan_start(void);

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK, LORAWAN_JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {    BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                            lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler};

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];                              ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0 ,0, 0, 0};                   ///< Lora user application data structure.

/////////////////////////////////////////////////////////////////////////////////////////////
/**@brief application variable declatation
*/
static     uint8_t  * p_payload;      /**< Pointer to payload. */



/////////////////////////////////////////////////////////////////////////////////////////////

/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;

// Static vars
APP_TIMER_DEF(ble_scan_timer_id);                                    ///< ble scan timer instance.
APP_TIMER_DEF(ble_adv_timer_id);                                    ///< ble advertising timer instance.
APP_TIMER_DEF(lora_tx_timer_id);                                    ///< LoRa tranfer timer instance.

//BLE_LBS_C_DEF(m_ble_lbs_c);                                     /**< Main structure used by the LBS client module. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */



/* Array to map FDS return values to strings. */
char const * fds_err_str[] =
{
    "FDS_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record
} m_delete_all;

/* BLE scan result structure definition */
typedef struct
{
    bool empty;
    int16_t tag_rssi;
    char     tag_id[6];
} tag_report_t;



static tag_report_t tag_table[table_MAX];


/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;

/* Global declaration */
static bool volatile trilateration;



static char const m_target_periph_name[] = "P ID ";           /**< Name of the device we try to connect to. This name is searched in the scan report data*/

/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t m_scan_params =
{
 //   .active   = 1,
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};



/* flash configuration data. */
//static configuration_t m_dummy_cfg =
//{
//    .boot_count  = 0x0,
//    .device_name = "dummy",
//};

//header_mask = (const uint8_t *)"XTER-";

/* flash Default configuration data. */
static char const default_tag_id[] = "P ID 654321";       
static mode_t mode_file =
{
    .mode = BEACON_MODE,
    .header_len = sizeof(default_tag_id),
    .header_mask = "X TR 123456",
//    .p_header_mask = &header_mask,
};

static scanner_t scanner_file =
{
    .period = 60,
    .interval = 200,
    .window = 180,
    .scanDuration = 20000,
    .rssiThreshold = -90,
    .maxTag = 6,
};

static beacon_t beacon_file =
{
    .keepAlive = 1,
    .interval = 100,
    .timeout = 4,
    .period = 20,
    .txpower = 0,
};



/* A record containing dummy configuration data. */
static fds_record_t const scanner_file_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CFG_SCANNER_KEY,
    .data.p_data       = &scanner_file,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(scanner_file) + 3) / sizeof(uint32_t),
};

static fds_record_t const beacon_file_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CFG_BEACON_KEY,
    .data.p_data       = &beacon_file,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(beacon_file) + 3) / sizeof(uint32_t),
};

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}



/**@brief LoRa function for handling HasJoined event.
 */
static void lorawan_has_joined_handler ()
{

#if (OVER_THE_AIR_ACTIVATION != 0)
    NRF_LOG_INFO("Network Joined");
#endif
    bsp_board_led_off(0);
    lmh_class_request(CLASS_A);

    m_lora_app_data.buffer[0] = RESET_msg;
    m_lora_app_data.buffer[1] = mode_file.mode;

    if (mode_file.mode == SCANNER_MODE)
    {
        // Start periodic timer for scanning
        app_timer_start(ble_scan_timer_id, APP_TIMER_TICKS(scanner_file.period*1000), NULL);
        // Send Reset Message
        memcpy(&m_lora_app_data.buffer[2], &scanner_file, sizeof(scanner_t));
        m_lora_app_data.buffsize = sizeof(scanner_t) + 2;
    }
    else
    {
        // Start periodic timer for avertising
        app_timer_start(ble_adv_timer_id, APP_TIMER_TICKS(beacon_file.period*1000), NULL);
        // Start periodic timer for Lora Keepalive
        app_timer_start(lora_tx_timer_id, APP_TIMER_TICKS(beacon_file.keepAlive*60000), NULL);
        // Send Reset Message
        memcpy(&m_lora_app_data.buffer[2], &beacon_file, sizeof(beacon_t));
        m_lora_app_data.buffsize = sizeof(beacon_t) + 2;

    }
           NRF_LOG_INFO("RST MSG SIZE : %d", m_lora_app_data.buffsize );
    send_lora_frame();




}

/**@brief LoRa function for handling received data from server
 *
 * @param[in] app_data  Pointer to rx data
 */
static void lorawan_rx_handler (lmh_app_data_t *app_data)
{
    ret_code_t err_code;

    //NRF_LOG_DEBUG("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d", app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);
    NRF_LOG_INFO("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d", app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

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

        case DWNLINK_CONFIG:
            if (app_data->buffsize != sizeof(scanner_t))
            //if (app_data->buffsize != 11)
            {
                NRF_LOG_INFO("DownLink Set Parameters scanner size mismatch %d, %d",app_data->buffsize, sizeof(scanner_t)  );
            }
            else
            {
                 /* Copy the configuration from lora into scanner_file. */
                memcpy(&scanner_file, app_data->buffer, sizeof(scanner_t));

                /* Update scanner File */
                fds_record_desc_t desc = {0};
                fds_find_token_t  tok  = {0};
                err_code = fds_record_find(CONFIG_FILE, CFG_SCANNER_KEY, &desc, &tok);
                if (err_code == FDS_SUCCESS)
                {
                     /* Write the updated record to flash. */
                     err_code = fds_record_update(&desc, &scanner_file_record);
                     APP_ERROR_CHECK(err_code);
                }
                else
                {
                    /* System config not found; write a new one. */
                     err_code = fds_record_write(&desc, &scanner_file_record);
                    APP_ERROR_CHECK(err_code);
                }
                NRF_LOG_INFO("scanner File updated");
                /* Reset */
                sd_nvic_SystemReset();
            }
            
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


static void send_lora_frame (void)
{
    if (lmh_join_status_get() != LMH_SET)
    {
        //Not joined, try joining again
        lmh_join();
        return;
    }
    
    m_lora_app_data.port = LORAWAN_APP_PORT;
    lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}


/**@brief Function for handling a LoRa tx timer timeout event.
 */
static void tx_lora_periodic_handler (void * unused)
{
    NRF_LOG_INFO("Lorawan KeepAlive");
    //send_lora_frame();
}






/**@brief Function for handling a LoRa tx timer timeout event.
 */
static void ble_scan_periodic_handler (void * unused)
{
    NRF_LOG_INFO("ble_scan handler");
    scan_start();
    battery_voltage_init();
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
static void ble_adv_periodic_handler (void * unused)
{
    NRF_LOG_INFO("ble_adv handler");
    //scan_start();
    //battery_voltage_init();
}

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
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
    APP_ERROR_CHECK(err_code);
	
    // Initialize timers
    err_code = app_timer_create(&ble_scan_timer_id, APP_TIMER_MODE_REPEATED, ble_scan_periodic_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&ble_adv_timer_id, APP_TIMER_MODE_REPEATED, ble_adv_periodic_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&lora_tx_timer_id, APP_TIMER_MODE_REPEATED, tx_lora_periodic_handler);
    VERIFY_SUCCESS(err_code);

     /* TO DO: Create any timers to be used by the application.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
     */

	
    return NRF_SUCCESS;
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;
    uint16_t i;

     bsp_board_led_on(2);

     // init tag_table
     for(i = 0 ; i < table_MAX ; i++)
     {
        tag_table[i].empty = true;
     }  

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     dev_name;

    int16_t    RSSI;      /**< RSSI */
    uint16_t   i;

    char name[32];

    // For readibility.
    ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
    ble_gap_addr_t const * peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

    // Initialize advertisement report for parsing
    adv_data.p_data   = (uint8_t *)p_gap_evt->params.adv_report.data;
    adv_data.data_len = p_gap_evt->params.adv_report.dlen;

    // Search for advertising names.
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);

    if (err_code == NRF_SUCCESS)
    {
        RSSI = p_gap_evt->params.adv_report.rssi;
        if (RSSI >= scanner_file.rssiThreshold)
        {
            if (strlen(m_target_periph_name) != 0)
            {
                memcpy(name, dev_name.p_data, dev_name.data_len);
                name[dev_name.data_len] = 0;
                if (memcmp(m_target_periph_name, dev_name.p_data, strlen(m_target_periph_name))== 0)
                {
                    NRF_LOG_INFO(" -> name : %s, rssi =%d dBm", &name[strlen(m_target_periph_name)], RSSI);

                    // Found 1 device matching, check if already exists
                    for(i = 0 ; i < table_MAX ; i++)
                    {
                        if (tag_table[i].empty == true)
                        {
                            // no match found, create the entry
                            tag_table[i].empty = false;
                            tag_table[i].tag_rssi = RSSI;
                            memcpy(tag_table[i].tag_id, &name[strlen(m_target_periph_name)], 6);
                            break;
                        }
                        else
                        {
                            if (memcmp(&tag_table[i].tag_id, &name[strlen(m_target_periph_name)], 6)== 0)
                            {
                                // entry matching
                                tag_table[i].tag_rssi = RSSI;
                                break;

                            } 
                        }
                    }  
                }
            }
        }
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    uint16_t i;
    uint16_t j, n;
    uint16_t index;
    uint16_t loop;
    int16_t higher;
    uint16_t pointer;
    uint16_t msb; 
    uint16_t lsb;
    char tag_id[6];
    uint16_t p_vbatt;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");
//            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c, p_gap_evt->conn_handle, NULL);
//            APP_ERROR_CHECK(err_code);

            //err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            //APP_ERROR_CHECK(err_code);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
 //           bsp_board_led_on(CENTRAL_CONNECTED_LED);
//            bsp_board_led_off(CENTRAL_SCANNING_LED);
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
           // scan_start();
        } break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            on_adv_report(p_ble_evt);
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("BLE scan timed out.");
                bsp_board_led_off(2);
                sd_ble_gap_scan_stop();
                //uint16_t vbatt;              // Variable to hold voltage reading
                //battery_voltage_get(&vbatt); // Get new battery voltage
                //printf("ADC result: %d\r\n", vbatt);

                for(index = 0 ; index < table_MAX ; index++)
                {
                    if (tag_table[index].empty == false)
                    {     
                        NRF_LOG_INFO(" tag_ID : %s, rssi =%d dBm", tag_table[index].tag_id, tag_table[index].tag_rssi);
                    }
                    else
                    {
                        break;
                    }
                }
                NRF_LOG_INFO("Nombre de tags lus : %d", index); 

                // Setup the lora messzage
                n = 0;
                if (trilateration == true)
                {
                    m_lora_app_data.buffer[n++] = BLE_RSSI_msg;
                }
                else
                {
                    m_lora_app_data.buffer[n++] = BLE_COUNT_msg;
                }
                
                battery_voltage_get(&p_vbatt);
                m_lora_app_data.buffer[n++] = (p_vbatt >> 8) & 0x0000000F;
                m_lora_app_data.buffer[n++] = (p_vbatt) & 0x000000FF;

                if (index > 0)
                {
                    if (index > scanner_file.maxTag)
                    {     
                        loop = scanner_file.maxTag;
                    }
                    else
                    {
                        loop = index;
                    }

                     m_lora_app_data.buffer[1] += ((loop << 4) & 0x000000F0);
               

                    for(j = 0 ; j < loop ; j++)
                    {
                        higher = -10000;
                        for(i = 0 ; i < index ; i++)
                        {
                            NRF_LOG_INFO("index : %d", i); 
                            if (tag_table[i].tag_rssi > higher)
                            {     
                                NRF_LOG_INFO("pointer : %d", pointer);
                                higher = tag_table[i].tag_rssi;
                                pointer = i;
                            }
                        }
                        NRF_LOG_INFO("pointer : %d", pointer);
  
                        msb = tag_table[pointer].tag_id[0];
                        lsb = tag_table[pointer].tag_id[1];
                        NRF_LOG_INFO("msb : %02X, lsb : %02X", msb, lsb);
                        if ( (msb & 0x40) != 0) { msb += 9; }
                        msb = (msb & 0x0F) << 4;
                        if ( (lsb & 0x40) != 0) { lsb += 9; }
                        lsb = (lsb & 0x0F) + msb;
                        NRF_LOG_INFO("hex : %02X", lsb);
                        m_lora_app_data.buffer[n++] = lsb;

                        msb = tag_table[pointer].tag_id[2];
                        lsb = tag_table[pointer].tag_id[3];
                         NRF_LOG_INFO("msb : %02X, lsb : %02X", msb, lsb);
                        if ( (msb & 0x40) != 0) { msb += 9; }
                        msb = (msb & 0x0F) << 4;
                        if ( (lsb & 0x40) != 0) { lsb += 9; }
                        lsb = (lsb & 0x0F) + msb;
                        NRF_LOG_INFO("hex : %02X", lsb);
                        m_lora_app_data.buffer[n++] = lsb;

                        msb = tag_table[pointer].tag_id[4];
                        lsb = tag_table[pointer].tag_id[5];
                         NRF_LOG_INFO("msb : %02X, lsb : %02X", msb, lsb);
                        if ( (msb & 0x40) != 0) { msb += 9; }
                        msb = (msb & 0x0F) << 4;
                        if ( (lsb & 0x40) != 0) { lsb += 9; }
                        lsb = (lsb & 0x0F) + msb;
                        NRF_LOG_INFO("hex : %02X", lsb);
                        m_lora_app_data.buffer[n++] = lsb;
                        
                        // Change Rssi to -10000 to remove it from the list
			tag_table[pointer].tag_rssi = -10000;

                        if (trilateration == true)
                        {
                            m_lora_app_data.buffer[n++] = (higher&0x0000FF00)>>8;
                            m_lora_app_data.buffer[n++] = higher&0x000000FF;
                        }
                    }
                }
                m_lora_app_data.buffsize = n;

                send_lora_frame();

            }
             
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the timer.
 */
//static void timer_init(void)
//{
//    ret_code_t err_code = app_timer_init();
//    APP_ERROR_CHECK(err_code);
//}

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

//static void power_manage (void)
//{
//    __WFE();
//    __SEV();
//    __WFE();
//}



/**@brief Function for initializing the Power manager. */
static void power_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


// Flash event handler
static void fds_evt_handler(fds_evt_t const * p_evt)
{
    NRF_LOG_INFO("Event: %s received (%s)",
                  fds_evt_str[p_evt->id],
                  fds_err_str[p_evt->result]);

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        case FDS_EVT_UPDATE:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}




/**@brief   Wait for fds to initialize. */
void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
        power_manage();
    }
}

int main(void)
{
    ret_code_t err_code;

    log_init();

    //power_init();

    // Initialize clocks
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
 


    bsp_board_leds_init();
    bsp_board_led_on(0);
   

    
	
    // Initialize Scheduler and timer
    err_code = timers_init();
    APP_ERROR_CHECK(err_code);

    ble_stack_init();
    gatt_init();

    // Enable nRF52 DCDC
    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    APP_ERROR_CHECK(err_code);


    // Initialize LoRa chip.
    err_code = lora_hardware_init();
    APP_ERROR_CHECK(err_code);

    // Initialize LoRaWan
    err_code = lmh_init(&lora_callbacks, lora_param_init);
    APP_ERROR_CHECK(err_code);
    

    // Flash init
 
    /* Register first to receive an event when initialization is complete. */
    (void) fds_register(fds_evt_handler);

    NRF_LOG_GREEN("Initializing fds...");

    err_code = fds_init();
    APP_ERROR_CHECK(err_code);

    /* Wait for fds to initialize. */
    wait_for_fds_ready();
    
    NRF_LOG_INFO("Reading flash usage statistics...");

    fds_stat_t stat = {0};

    err_code = fds_stat(&stat);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    
    
    // Read Config

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};
    err_code = fds_record_find(CONFIG_FILE, CFG_MODE_KEY, &desc, &tok);
    if (err_code == FDS_SUCCESS)
    {
        /* A config file is in flash */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        err_code = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(err_code);

        /* Copy the configuration from flash into config_file. */
        memcpy(&mode_file, config.p_data, sizeof(mode_t));

        /* Close the record when done reading. */
        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
    }

    if (mode_file.mode == SCANNER_MODE)
    {
        NRF_LOG_INFO("PicoTag : mode SCANNER");
        NRF_LOG_INFO("Header len : %d", mode_file.header_len);
        NRF_LOG_INFO("Header mask : %s", mode_file.header_mask);
        fds_record_desc_t desc = {0};
        fds_find_token_t  tok  = {0};
        err_code = fds_record_find(CONFIG_FILE, CFG_SCANNER_KEY, &desc, &tok);
        if (err_code == FDS_SUCCESS)
        {
            /* A config file is in flash */
            fds_flash_record_t config = {0};

            /* Open the record and read its contents. */
            err_code = fds_record_open(&desc, &config);
            APP_ERROR_CHECK(err_code);

            /* Copy the configuration from flash into config_file. */
            memcpy(&scanner_file, config.p_data, sizeof(scanner_t));

            /* Close the record when done reading. */
            err_code = fds_record_close(&desc);
            APP_ERROR_CHECK(err_code);
        }

        // updatestatic ble_gap_scan_params_t 

        m_scan_params.interval = scanner_file.interval;
        m_scan_params.window   = scanner_file.window;
        m_scan_params.timeout  = scanner_file.scanDuration / 1000;

        NRF_LOG_INFO("period %d", scanner_file.period);
        NRF_LOG_INFO("interval %d", scanner_file.interval);
        NRF_LOG_INFO("window %d", scanner_file.window);
        NRF_LOG_INFO("scanDuration %d", scanner_file.scanDuration);
        NRF_LOG_INFO("rssiThreshold %d", scanner_file.rssiThreshold);
        NRF_LOG_INFO("maxTag %d", scanner_file.maxTag & 0x0F);
                                                            // default 4
        if (scanner_file.maxTag & 0x80)
        {
            trilateration = true;
            NRF_LOG_INFO("trilateration enabled");
        }
        else
        {
            trilateration = false;
            NRF_LOG_INFO("trilateration disabled");
        }
    }
    else
    {
        NRF_LOG_INFO("PicoTag : mode BEACON");
        NRF_LOG_INFO("Tag ID len : %d", mode_file.header_len);
        NRF_LOG_INFO("Tag ID : %s", mode_file.header_mask);

        fds_record_desc_t desc = {0};
        fds_find_token_t  tok  = {0};
        err_code = fds_record_find(CONFIG_FILE, CFG_BEACON_KEY, &desc, &tok);
        if (err_code == FDS_SUCCESS)
        {
            /* A config file is in flash */
            fds_flash_record_t config = {0};

            /* Open the record and read its contents. */
            err_code = fds_record_open(&desc, &config);
            APP_ERROR_CHECK(err_code);

            /* Copy the configuration from flash into config_file. */
            memcpy(&beacon_file, config.p_data, sizeof(beacon_t));

            /* Close the record when done reading. */
            err_code = fds_record_close(&desc);
            APP_ERROR_CHECK(err_code);
        }

        NRF_LOG_INFO("keepAlive %d minutes", beacon_file.keepAlive);
        NRF_LOG_INFO("interval %d ms", beacon_file.interval);
        NRF_LOG_INFO("timeout %d seconds", beacon_file.timeout);
        NRF_LOG_INFO("period %d seconds", beacon_file.period);
        NRF_LOG_INFO("txpower %d", beacon_file.txpower);
     }



     // Start LoRaWAN Join procedure
    lmh_join();
    //app_timer_start(lora_tx_timer_id, APP_TIMER_TICKS(LORAWAN_APP_TX_DUTYCYCLE), NULL);

    //app_timer_start(ble_scan_timer_id, APP_TIMER_TICKS(BLE_SCAN_DUTYCYCLE), NULL);

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
    //for (;;)
    //{
    //    NRF_LOG_FLUSH();
    //    nrf_pwr_mgmt_run();
    //}
}
