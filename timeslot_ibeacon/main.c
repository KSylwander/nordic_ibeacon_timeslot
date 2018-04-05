/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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

#include <stdint.h>
#include <string.h>

#include "advertiser_beacon.h"

#include "nordic_common.h"
#include "nrf.h"
#include "boards.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "bsp.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_pay_service.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG            2                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// Beacon advertisment 

#define APP_COMPANY_IDENTIFIER               0x004C                                     /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

// This should be defined in "non-reversed" order as oopposed to the UUID used in the service.
#define BEACON_UUID 0xB9, 0x40, 0x7F, 0x30, 0xF5, 0xF8, 0x46, 0x6E,\
                    0xAF, 0xF9, 0x25, 0x55, 0x6B, 0x57, 0xFF, 0xFF 

#define BEACON_ADV_INTERVAL                  100                                        /**< The Beacon's advertising interval, in milliseconds*/
#define BEACON_MAJOR                         0x1234                                     /**< The Beacon's Major*/
#define BEACON_MINOR                         0x5678                                     /**< The Beacon's Minor*/
#define BEACON_RSSI                          0xBD                                       /**< The Beacon's measured RSSI at 1 meter distance in dBm. */

static ble_beacon_init_t beacon_init;

static uint8_t m_adv_ibeacon = 0;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**
 * @brief Function for initializing the logging module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Initialized logs");
}

/**@brief Function for handling a BeaconAdvertiser error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void beacon_advertiser_error_handler(uint32_t nrf_error)
{
    NRF_LOG_INFO("Beacon error: %d", nrf_error);
    APP_ERROR_HANDLER(nrf_error);
}

static void beacon_adv_handler(bool is_advertising) 
{
    if (is_advertising) {
        NRF_LOG_INFO("Beacon handler - Started advertising beacon");
        bsp_board_led_on(BSP_BOARD_LED_0);
    } else {
        NRF_LOG_INFO("Beacon handler - Stopped advertising beacon");
        bsp_board_led_off(BSP_BOARD_LED_0);
    }
}

/**@brief Function for initializing Beacon advertiser.
 */
static void beacon_adv_init(void)
{
    static uint8_t beacon_uuid[] = {BEACON_UUID};

    memcpy(beacon_init.uuid.uuid128, beacon_uuid, sizeof(beacon_uuid));
    beacon_init.adv_interval  = BEACON_ADV_INTERVAL;
    beacon_init.major         = BEACON_MAJOR;
    beacon_init.minor         = BEACON_MINOR;
    beacon_init.manuf_id      = APP_COMPANY_IDENTIFIER;
    beacon_init.rssi          = BEACON_RSSI;
    beacon_init.error_handler = beacon_advertiser_error_handler;
    beacon_init.adv_handler   = beacon_adv_handler;

    uint32_t err_code = sd_ble_gap_addr_get(&beacon_init.beacon_addr);
    APP_ERROR_CHECK(err_code);

    // We have to have different addresses on both advertisers
    //beacon_init.beacon_addr.addr[3]++;
    
    app_beacon_init(&beacon_init);
}

/**
 * @brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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
    //NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    NRF_LOG_INFO("Initialized BLE stack");
}

static void bsp_event_callback(bsp_event_t bsp_event) {
    switch (bsp_event) {
        case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("Button 1 Pressed");
            if (!m_adv_ibeacon) {
                m_adv_ibeacon = 1;
                app_beacon_start();
            } else {
                m_adv_ibeacon = 0;
                app_beacon_stop();
            }
            break;
        default:
            break;
    }
}

/**
 * @brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(err_code);
    bsp_board_leds_off();
    err_code = bsp_buttons_enable();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Initialized LEDs");
}

/**
 * @brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Initialized timers");
}

/**
 * @brief Function for the Power Manager.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();
    leds_init();

    ble_stack_init();
    beacon_adv_init();

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
