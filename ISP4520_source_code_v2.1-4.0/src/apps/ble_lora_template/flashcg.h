/**
 * Copyright (c) 2017 - 2017, Nordic Semiconductor ASA
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

#ifndef FDS_EXAMPLE_H__
#define FDS_EXAMPLE_H__

#include <stdint.h>
#include "fds.h"

/* File ID and Key used for the configuration record. */
#define CONFIG_FILE           (0xF010)
#define CFG_SCAN_PARAM_KEY    (0x700F)
#define CFG_TAG_KEY           (0x7010)


/* File ID and Key used for the DeviceKey record. */
#define DEVICE_KEY__FILE     (0xF011)
#define DEVICE_REC_KEY  (0x7011)

/* Colors used to print on the console. */

#define COLOR_GREEN     "\033[1;32m"
#define COLOR_YELLOW    "\033[1;33m"
#define COLOR_CYAN      "\033[1;36m"

/* Macros to print on the console using colors. */

#define NRF_LOG_CYAN(...)   NRF_LOG_INFO(COLOR_CYAN   __VA_ARGS__)
#define NRF_LOG_YELLOW(...) NRF_LOG_INFO(COLOR_YELLOW __VA_ARGS__)
#define NRF_LOG_GREEN(...)  NRF_LOG_INFO(COLOR_GREEN  __VA_ARGS__)


/* A dummy structure to save in flash. */
typedef struct
{
    char     tag_id[12];
}   __attribute__ ((packed)) tagid_t;

typedef struct
{
    uint16_t interval;
    uint16_t window;
}  __attribute__ ((packed)) scan_param_t;

typedef struct
{
    uint8_t   mode;                   // Tag mode : BEACON_MODE or SCANNER_MODE
    uint16_t  adv_keepAlive;          // adv keepalive                -> in minutes
    uint16_t  adv_period;             // adv period                   -> in sec 
    uint16_t  scan_period;            // scan period                  -> in sec 
    uint16_t  adv_duration;           // adv duration                 -> in ms
    uint16_t  scan_duration;          // Scan duration                -> in mc
    uint16_t  adv_interval;           // adv interval                 -> in ms
    int16_t   rssiThreshold;          // Scan rssi threshold          -> in dbm
    int8_t   txpower;                // Adv transmission power       -> in dbm
    uint8_t   maxTag;                 // Scan maxTag reported
    char      header_value[6];
    uint8_t   match_len;              // Size of char to match from header_value[0] to header_value[match_len]
}  __attribute__ ((packed)) tag_config_t;

/* Defined in main.c */

//void delete_all_begin(void);

/* Defined in flashcg.c */

//void fds_evt_handler(fds_evt_t const * p_evt);
//void delete_all_begin(void);
//void delete_all_process(void);
//void wait_for_fds_ready(void);

/* Defined in cli.c */

//void cli_init(void);
//void cli_start(void);
//void cli_process(void);
//bool record_delete_next(void);


#endif
