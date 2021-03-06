INSIGHT SIP

LoRaWAN endpoint stack implementation and example projects.
=====================================

1. Introduction
----------------
The aim of this archive is to show examples of raw LoRa and LoRaWAN stack implementation.

This LoRaWAN stack implements Europe region (868MHz frequency) Class A endpoint.

Other simple examples are provided:
- a Ping-Pong application.
- a CW application.
- an RX sensitivity application. 
- a TX and RX with temperature data application (default firmware in the Dev Kits)
- a BLE + LoRaWan template

2. Acknowledgments
-------------------
The official LoRaWAN stack (http://stackforce.github.io/LoRaMac-doc/*) project was used at the beginning as source of inspiration.

This program uses the AES algorithm implementation (http://www.gladman.me.uk/) by Brian Gladman.

This program uses the CMAC algorithm implementation
(http://www.cse.chalmers.se/research/group/dcs/masters/contikisec/) by Lander Casado, Philippas Tsigas.

Implements LoRa Mac from Semtech/StackForce master branch (4.4.1 release)
https://github.com/Lora-net/LoRaMac-node/tree/master

3. Dependencies
----------------
This program depends on specific hardware platforms. Currently the supported platforms are:
    - ISP4520_EU Revision C
        MCU     : nRF52832 - 512K FLASH, 64K RAM, Timers, SPI, I2C, UART, DAC, ADC, DMA
        RADIO   : SX1261
        ANTENNA : internal circuit antenna

4. Usage
---------
Projects for Keil v5 and Segger Embedded Studio Environments are available.

One project is available per application and for each hardware platform in each
development environment. Different targets/configurations have been created in
the different projects in order to select different options such as the radio frequency band to be used.
The projects needs Nordic SDK 14.2.0 (not included in this archive).

5. Tree
--------
<YourDirectory>
├───hex
└───src
    ├───apps
    │   ├───ble_lora_template
    │   │   ├───ISP4520_EU
    │   │   │   ├───arm5_no_packs
    │   │   │   ├───config
    │   │   │   └───ses
    │   ├───lora_classA
    │   │   ├───ISP4520_EU
    │   │   │   ├───arm5_no_packs
    │   │   │   ├───config
    │   │   │   └───ses
    │   ├───lora_ping_pong
    │   │   ├───ISP4520_EU
    │   │   │   ├───arm5_no_packs
    │   │   │   ├───config
    │   │   │   └───ses
    │   ├───lora_rx_sensi
    │   │   ├───ISP4520_EU
    │   │   │   ├───arm5_no_packs
    │   │   │   ├───config
    │   │   │   └───ses
    │   ├───lora_rx_temp
    │   │   ├───ISP4520_EU
    │   │   │   ├───arm5_no_packs
    │   │   │   ├───config
    │   │   │   └───ses
    │   ├───lora_tx_temp
    │   │   ├───ISP4520_EU
    │   │   │   ├───arm5_no_packs
    │   │   │   ├───config
    │   │   │   └───ses
    │   └───lora_tx-cw
    │       └───ISP4520_EU
    │           ├───arm5_no_packs
    │           ├───config
    │           └───ses
    ├───lora
    │   ├───boards
    │   │   ├───isp4520
    │   │   │   └───cmsis
    │   │   ├───mcu
    │   │   │   └───nrf52832
    │   ├───mac
    │   │   └───region
    │   ├───radio
    │   │   ├───sx126x
    │   │   ├───sx1272
    │   │   └───sx1276
    │   └───system
    │       └───crypto
    └───nordic_sdk

7. Changelog
-------------
2019-05-07, v2.1.0
* Changed LoRaWan stack to release v4.4.1
* Fixed include path errors on soem projects

2019-04-09, v2.0.4
* Fixed building error on few examples

2019-03-29, v2.0.3
* Added ble_lora_template projects
* Simplified Boards.h

2019-03-18, v2.0.2
* Renamed target from isp4520 to ISP4520_EU

2019-03-08, v2.0.1
* Changed LoRaMacHelper.c to v1.0.1, now correctly send "confirmed" frames

2019-02-05, v2.0.0
* Release for customer

2018-06-28, v1.0.0
* Initial version.
