/** @file
 *
 * @defgroup ble_sdk_adv_beacon Advertiser Module using the radio through the sd_radio_request SoftDevice API
 * @{
 * @ingroup ble_sdk_radio_time_slot_api
 * @brief Advertiser module. This module shows an example of using periodic timeslots on the radio when the SoftDevice is running
 *
 * @details This module implements an advertiser which can be run in parallel with the S110 (thus allowing, for example, to advertise while in a connection)
 *          This module shows an example of using periodic timeslots on the radio when the SoftDevice is running
 *
 * @note This module is experimental.
 *
 */
#ifndef ADVERTISER_BEACON_H__
#define ADVERTISER_BEACON_H__

#include <stdint.h>
#include <stdbool.h>

#include "ble_types.h"
#include "ble_gap.h"
#include "ble_srv_common.h"

typedef void (*adv_beacon_handler_t) (bool is_advertising);

typedef struct
{
    ble_uuid128_t           uuid;
    uint32_t                adv_interval;
    uint16_t                major;
    uint16_t                minor;
    uint16_t                manuf_id;
    uint8_t                 rssi;                               /** measured RSSI at 1 meter distance in dBm*/
    ble_gap_addr_t          beacon_addr;                        /** ble address to be used by the beacon*/    
    ble_srv_error_handler_t error_handler;                      /**< Function to be called in case of an error. */
    adv_beacon_handler_t	adv_handler;						/**< Function called when advertising is started and stopped. */
} ble_beacon_init_t;


/**@brief Function for handling system events.
 *
 * @details Handles all system events of interest to the Advertiser module. 
 *
 * @param[in]   event     received event.
 */
void app_beacon_on_sys_evt(uint32_t event);

/**@brief Function for initializing the advertiser module.
 *
 * @param[in]   p_init     structure containing advertiser configuration information.
 */
void app_beacon_init(ble_beacon_init_t * p_init);

/**@brief Function for starting the advertisement.
 *
 */
void app_beacon_start(void);

/**@brief Function for stopping the advertisement.
 * @note This functions returns immediately, but the advertisement is actually stopped after the next radio slot.
 *
 */
void app_beacon_stop(void);

#endif // ADVERTISER_BEACON_H__