/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

 /******************************************************************************
 * @file    board.h
 * @author  Insight SiP
 * @version V2.1.0
 * @date    07-may-2019 
 * @brief   Board (module) specific functions declaration.
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

#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"

#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "gpio-board.h"
#include "spi-board.h"

#include "radio.h"
#include "sx126x/sx126x.h"
#include "sx126x-board.h"


/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

#define LORA_RESET 	PIN_LORA_RESET
#define SPI_INSTANCE 	0

//nRF pin configuration
#define NRF_NUM_GPIO_PINS       32
#define PIN_OSC_XL1             0       // 32kHz XTAL
#define PIN_OSC_XL2             1       // 32kHz XTAL
#define PIN02                   2       // External pin
#define PIN03                   3       // External pin
#define PIN04                   4       // External pin
#define PIN05                   5       // External pin
#define PIN06                   6       // External pin
#define PIN07                   7       // External pin
#define PIN08                   8       // External pin
#define PIN09                   9       // External pin
#define PIN10                   10      // External pin
#define PIN_LORA_DIO_1          11      // LORA DIO_1
#define PIN12                   12      // External pin
#define PIN13                   13      // External pin
#define PIN14                   14      // External pin
#define PIN15                   15      // External pin
#define PIN16                   16      // External pin
#define PIN17                   17      // External pin
#define PIN18                   18      // External pin
#define PIN_LORA_RESET          19      // LORA RESET
#define PIN20                   20      // External pin
#define PIN_NRF_RST             21      // NRF RESET
#define PIN22                   22      // External pin
#define PIN_LORA_SCLK           23      // LORA SPI CLK
#define PIN_LORA_NSS            24      // LORA SPI CS
#define PIN_LORA_MISO           25      // LORA SPI MISO
#define PIN_LORA_MOSI           26      // LORA SPI MOSI
#define PIN_LORA_BUSY           27      // LORA SPI BUSY
#define PIN_28                  28      // External pin
#define PIN_29                  29      // External pin
#define PIN_30                  30      // External pin
#define PIN_31                  31      // External pin

// modif cg 1707
// LEDs definitions for XterTag
#define LEDS_NUMBER    3

#define LED_1          PIN12          // Rouge
#define LED_2          PIN10          // Vert
#define LED_3          PIN17          // Orange

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2, LED_3}

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3

#define BUTTONS_NUMBER  0

/**
 * Function for returning the state of an LED.
 *
 * @param led_idx LED index (starting from 0), as defined in the board-specific header.
 *
 * @return True if the LED is turned on.
 */
bool bsp_board_led_state_get(uint32_t led_idx);

/**
 * Function for turning on an LED.
 *
 * @param led_idx LED index (starting from 0), as defined in the board-specific header.
 */
void bsp_board_led_on(uint32_t led_idx);

/**
 * Function for turning off an LED.
 *
 * @param led_idx LED index (starting from 0), as defined in the board-specific header.
 */
void bsp_board_led_off(uint32_t led_idx);

/**
 * Function for inverting the state of an LED.
 *
 * @param led_idx LED index (starting from 0), as defined in the board-specific header.
 */
void bsp_board_led_invert(uint32_t led_idx);
/**
 * Function for turning off all LEDs.
 */
void bsp_board_leds_off(void);

/**
 * Function for turning on all LEDs.
 */
void bsp_board_leds_on(void);

/**
 * Function for initializing LEDs.
 */
void bsp_board_leds_init(void);

#ifdef BSP_LED_0
#define BSP_LED_0_MASK (1<<BSP_LED_0)
#else
#define BSP_LED_0_MASK 0
#endif
#ifdef BSP_LED_1
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#else
#define BSP_LED_1_MASK 0
#endif
#ifdef BSP_LED_2
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#else
#define BSP_LED_2_MASK 0
#endif
#ifdef BSP_LED_3
#define BSP_LED_3_MASK (1<<BSP_LED_3)
#else
#define BSP_LED_3_MASK 0
#endif
#ifdef BSP_LED_4
#define BSP_LED_4_MASK (1<<BSP_LED_4)
#else
#define BSP_LED_4_MASK 0
#endif
#ifdef BSP_LED_5
#define BSP_LED_5_MASK (1<<BSP_LED_5)
#else
#define BSP_LED_5_MASK 0
#endif
#ifdef BSP_LED_6
#define BSP_LED_6_MASK (1<<BSP_LED_6)
#else
#define BSP_LED_6_MASK 0
#endif
#ifdef BSP_LED_7
#define BSP_LED_7_MASK (1<<BSP_LED_7)
#else
#define BSP_LED_7_MASK 0
#endif

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | \
                        BSP_LED_2_MASK | BSP_LED_3_MASK | \
                        BSP_LED_4_MASK | BSP_LED_5_MASK | \
                        BSP_LED_6_MASK | BSP_LED_7_MASK)

#define LEDS_OFF(leds_mask) do {  ASSERT(sizeof(leds_mask) == 4);                     \
                        NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                        NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LEDS_ON(leds_mask) do {  ASSERT(sizeof(leds_mask) == 4);                     \
                       NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                       NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LED_IS_ON(leds_mask) ((leds_mask) & (NRF_GPIO->OUT ^ LEDS_INV_MASK) )

#define LEDS_INVERT(leds_mask) do { uint32_t gpio_state = NRF_GPIO->OUT;      \
                              ASSERT(sizeof(leds_mask) == 4);                 \
                              NRF_GPIO->OUTSET = ((leds_mask) & ~gpio_state); \
                              NRF_GPIO->OUTCLR = ((leds_mask) & gpio_state); } while (0)

#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
                                  ASSERT(sizeof(leds_mask) == 4);     \
                                  for (pin = 0; pin < 32; pin++)      \
                                      if ( (leds_mask) & (1 << pin) ) \
                                          nrf_gpio_cfg_output(pin); } while (0)

// endof modif cg

/**@brief Initializes the target board peripherals.
 */
uint32_t lora_hardware_init (void);

/**@brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void lora_hardware_uninit (void);

/**@brief Returns a pseudo random seed generated using the MCU Unique ID
 *
 * @retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed (void);

/**@brief Gets the board 64 bits unique ID
 *
 * @param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId (uint8_t *id);

/**@brief   Get batttery value TO BE IMPLEMENTED
 */
uint8_t BoardGetBatteryLevel (void);

/**@brief Disable interrupts
 *
 * @remark IRQ nesting is managed
 */
void BoardDisableIrq(void);

/**@brief Enable interrupts
 *
 * @remark IRQ nesting is managed
 */
void BoardEnableIrq(void);

#endif // __BOARD_H__
