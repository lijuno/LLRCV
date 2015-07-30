/*
The MIT License (MIT)

Copyright (c) 2015 Lijun (http://lijun.li/)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#ifndef LL_BLE_OTATS_H__
#define LL_BLE_OTATS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ll_common.h"


// Forward declaration of the ll_ble_otats_t type. 
typedef struct ll_ble_otats_s ll_ble_otats_t;

typedef void (*ll_ble_otats_trig_char_write_handler_t) (ll_ble_otats_t * p_otats, uint8_t *new_state);  

typedef struct
{
    ll_ble_otats_trig_char_write_handler_t otats_trig_char_write_handler;                    /**< Event handler to be called when LED characteristic is written. */
    // This event write handler is to be defined in the main.c. 
} ll_ble_otats_init_t;

/**@brief OTA Triggering Service structure. This contains various status information for the service. */
typedef struct ll_ble_otats_s
{
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    ota_trig_char_handles;
    uint8_t                     uuid_type;   // Reference https://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v8.x.x/doc/8.0.0/s110/html/a00864.html
	                                           // BLE_UUID_TYPE_BLE (0x01) for 16-bit UUID, BLE_UUID_TYPE_VENDOR_BEGIN (0x02) for 128-bit UUID
    uint16_t                    conn_handle;
    ll_ble_otats_trig_char_write_handler_t otats_trig_char_write_handler;
} ll_ble_otats_t;

/**@brief Function for initializing the OTA Triggering Service.
 *
 * @param[out]  p_otats       OTA Triggering Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_otats_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ll_ble_otats_init(ll_ble_otats_t * p_otats, const ll_ble_otats_init_t * p_otats_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the OTA Triggering Service.
 *
 *
 * @param[in]   p_otats      OTA Triggering Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ll_ble_otats_on_ble_evt(ll_ble_otats_t * p_otats, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 */
// uint32_t ble_otats_on_button_change(ll_ble_otats_t * p_otats, uint8_t button_state);

#endif // LL_BLE_OTATS_H__



