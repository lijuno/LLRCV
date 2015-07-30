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

#ifndef LL_BLE_RCV1S_H__
#define LL_BLE_RCV1S_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ll_common.h"

#define RCV1S_MODE_CMD_LEN  1
#define RCV1S_MOTION_CMD_LEN  5

// mode command list
#define RCV1S_MODE_CMD_FULL_SHUTDOWN  0
#define RCV1S_MODE_CMD_EXECUTE_QUEUE  1
#define RCV1S_MODE_CMD_START_LIVE_STREAM  2
#define RCV1S_MODE_CMD_PAUSE  3


// Design of service
// Characteristic 1: start/stop command; trigger a handler to start/stop the motors
// Characteristic 2: set moving parameters; speed of left and right

// Forward declaration of the ll_ble_rcv1s_t type. 
typedef struct ll_ble_rcv1s_s ll_ble_rcv1s_t;

typedef void (*ll_ble_rcv1s_mode_char_write_handler_t) (ll_ble_rcv1s_t * p_rcv1s, uint8_t *data);  
typedef void (*ll_ble_rcv1s_moving_params_char_write_handler_t) (ll_ble_rcv1s_t * p_rcv1s, uint8_t *data);  

typedef struct
{   // These event write handlers are to be defined in the main.c. 
    ll_ble_rcv1s_mode_char_write_handler_t rcv1s_mode_char_write_handler;
	  ll_ble_rcv1s_moving_params_char_write_handler_t rcv1s_moving_params_char_write_handler;
} ll_ble_rcv1s_init_t;

// rcv1s Service structure. 
typedef struct ll_ble_rcv1s_s
{
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    mode_char_handles;
	  ble_gatts_char_handles_t    moving_params_char_handles;
    uint8_t                     uuid_type;   // Reference https://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v8.x.x/doc/8.0.0/s110/html/a00864.html
	                                           // BLE_UUID_TYPE_BLE (0x01) for 16-bit UUID, BLE_UUID_TYPE_VENDOR_BEGIN (0x02) for 128-bit UUID
    uint16_t                    conn_handle;
    ll_ble_rcv1s_mode_char_write_handler_t rcv1s_mode_char_write_handler;
	  ll_ble_rcv1s_moving_params_char_write_handler_t rcv1s_moving_params_char_write_handler;
} ll_ble_rcv1s_t;

// Service initialization
// NRF_SUCCESS on successful initialization of service, otherwise an error code.
uint32_t ll_ble_rcv1s_init(ll_ble_rcv1s_t * p_rcv1s, const ll_ble_rcv1s_init_t * p_rcv1s_init);

void ll_ble_rcv1s_on_ble_evt(ll_ble_rcv1s_t * p_rcv1s, ble_evt_t * p_ble_evt);

#endif  // LL_BLE_RCV1S_H__


