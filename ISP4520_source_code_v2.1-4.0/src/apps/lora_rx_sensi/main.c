/******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    16-mars-2018
 * @brief   Rx sensitivity Main file.
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

// nRF
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_drv_clock.h"

// LoRa
#include "radio.h"
#include "board.h"

//logs
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#if defined( REGION_AS923 )
//#define RF_FREQUENCY                                923000000 // Hz
#define RF_FREQUENCY                                925000000 // Hz
#elif defined( REGION_AU915 )
#define RF_FREQUENCY                                915000000 // Hz
#elif defined( REGION_CN779 )
#define RF_FREQUENCY                                779000000 // Hz
#elif defined( REGION_EU868 )
#define RF_FREQUENCY                                868000000 // Hz
#elif defined( REGION_KR920 )
#define RF_FREQUENCY                                920000000 // Hz
#elif defined( REGION_IN865 )
#define RF_FREQUENCY                                865000000 // Hz
#elif defined( REGION_US915 )
#define RF_FREQUENCY                                915000000 // Hz
#elif defined( REGION_US915_HYBRID )
#define RF_FREQUENCY                                915000000 // Hz
#else
    #error "Please define a frequency band in the compiler options."
#endif

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define BUFFER_SIZE                                 64 		  // Define the payload size here

static RadioEvents_t RadioEvents;
static int8_t RssiValue = 0;
static int8_t SnrValue = 0;
static uint16_t BufferSize = BUFFER_SIZE;
static uint8_t Buffer[BUFFER_SIZE];



 /**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone (uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    BufferSize = size;
    memcpy(Buffer, payload, BufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    NRF_LOG_INFO("OnRxDone");
    NRF_LOG_INFO("RssiValue=%d dBm, SnrValue=%d", rssi, snr);
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
    // Initialize logs.
    log_init();
    NRF_LOG_INFO("Lora rx-sensi example started.");
	
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    nrf_drv_clock_hfclk_request(NULL);
    NRF_POWER->DCDCEN = 1;
	
    // Initialize LORA
    lora_hardware_init();
    RadioEvents.RxDone = OnRxDone;
    Radio.Init(&RadioEvents);
    Radio.SetChannel (RF_FREQUENCY);
    Radio.SetRxConfig   (MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                        0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
	
    // Start LORA
    Radio.Rx(0); // Continuous Rx

    // Enter main loop.
    for (;;)
    {
        Radio.IrqProcess( );

        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}

