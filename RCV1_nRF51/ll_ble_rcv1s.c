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

#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ll_ble_rcv1s.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_rcv1s       OTA Triggering Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ll_ble_rcv1s_t * p_rcv1s, ble_evt_t * p_ble_evt)
{
    p_rcv1s->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_rcv1s       OTA Triggering Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ll_ble_rcv1s_t * p_rcv1s, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_rcv1s->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_rcv1s       OTA Triggering Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write_mode_char(ll_ble_rcv1s_t * p_rcv1s, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_rcv1s->mode_char_handles.value_handle) &&
         (p_evt_write->len == RCV1S_MODE_CMD_LEN) &&   // length of received data array
        (p_rcv1s->rcv1s_mode_char_write_handler != NULL))   // if write handler is defined
    {
        p_rcv1s->rcv1s_mode_char_write_handler(p_rcv1s, p_evt_write->data);
    }
}


static void on_write_moving_params_char(ll_ble_rcv1s_t * p_rcv1s, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_rcv1s->moving_params_char_handles.value_handle) &&
        (p_evt_write->len == RCV1S_MOTION_CMD_LEN) &&   // length of received data array
        (p_rcv1s->rcv1s_moving_params_char_write_handler != NULL))   // if write handler is defined
    {
        p_rcv1s->rcv1s_moving_params_char_write_handler(p_rcv1s, p_evt_write->data);
    }
}


void ll_ble_rcv1s_on_ble_evt(ll_ble_rcv1s_t * p_rcv1s, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_rcv1s, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_rcv1s, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write_mode_char(p_rcv1s, p_ble_evt);
				    on_write_moving_params_char(p_rcv1s, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding characteristics.
 */
static uint32_t rcv1s_mode_char_add(ll_ble_rcv1s_t * p_rcv1s, const ll_ble_rcv1s_init_t * p_rcv1s_init)
{
    ble_gatts_char_md_t char_md;   // characteristic metadata
    ble_gatts_attr_t    attr_char_value;   // the value of characteristics (GATT attribute type)
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;  // attribute metadata

    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_rcv1s->uuid_type;
    ble_uuid.uuid = RCV1S_UUID_MODE_CHAR;
    
		// Setting the 2nd attribute (the value of characteristics)
		// First set up the attribute metadata, and then add it to attribute
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t)*RCV1S_MODE_CMD_LEN;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t)*RCV1S_MODE_CMD_LEN;  
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_rcv1s->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_rcv1s->mode_char_handles);
}


static uint32_t rcv1s_moving_params_char_add(ll_ble_rcv1s_t * p_rcv1s, const ll_ble_rcv1s_init_t * p_rcv1s_init)
{   
    ble_gatts_char_md_t char_md;   // characteristic metadata
    ble_gatts_attr_t    attr_char_value;   // the value of characteristics (GATT attribute type)
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;  // attribute metadata

    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_rcv1s->uuid_type;
    ble_uuid.uuid = RCV1S_UUID_MOVING_PARAMS_CHAR;
    
		// Setting the 2nd attribute (the value of characteristics)
		// First set up the attribute metadata, and then add it to attribute
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t)*RCV1S_MOTION_CMD_LEN;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t)*RCV1S_MOTION_CMD_LEN;  
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_rcv1s->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_rcv1s->moving_params_char_handles);
}



uint32_t ll_ble_rcv1s_init(ll_ble_rcv1s_t * p_rcv1s, const ll_ble_rcv1s_init_t * p_rcv1s_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_rcv1s->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_rcv1s->rcv1s_mode_char_write_handler = p_rcv1s_init->rcv1s_mode_char_write_handler;
	  p_rcv1s->rcv1s_moving_params_char_write_handler = p_rcv1s_init->rcv1s_moving_params_char_write_handler;
    
    // Add service
    ble_uuid128_t base_uuid = {LL_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_rcv1s->uuid_type); // Add a vendor specific ID; Reference http://bit.ly/1JLkeCY
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_rcv1s->uuid_type;
    ble_uuid.uuid = RCV1S_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_rcv1s->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    

    err_code = rcv1s_mode_char_add(p_rcv1s, p_rcv1s_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = rcv1s_moving_params_char_add(p_rcv1s, p_rcv1s_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}


