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


// Implementation of OTA DFU triggering service

#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ll_ble_otats.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_otats       OTA Triggering Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ll_ble_otats_t * p_otats, ble_evt_t * p_ble_evt)
{
    p_otats->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_otats       OTA Triggering Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ll_ble_otats_t * p_otats, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_otats->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_otats       OTA Triggering Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ll_ble_otats_t * p_otats, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_otats->ota_trig_char_handles.value_handle) &&
         (p_evt_write->len == OTA_TRIG_CMD_LEN) &&   // length of received data array
        (p_otats->otats_trig_char_write_handler != NULL))   // if write handler is defined
    {
        p_otats->otats_trig_char_write_handler(p_otats, p_evt_write->data);
    }
}


void ll_ble_otats_on_ble_evt(ll_ble_otats_t * p_otats, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_otats, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_otats, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_otats, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the LED characteristic.
 *
 */
static uint32_t ota_trig_char_add(ll_ble_otats_t * p_otats, const ll_ble_otats_init_t * p_otats_init)
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
    
    ble_uuid.type = p_otats->uuid_type;
    ble_uuid.uuid = OTATS_UUID_CHAR;
    
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
    attr_char_value.init_len     = sizeof(uint8_t)*4;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t)*4;  
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_otats->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_otats->ota_trig_char_handles);
}

/**@brief Function for adding the Button characteristic.
 *
 */
//static uint32_t button_char_add(ll_ble_otats_t * p_otats, const ll_ble_otats_init_t * p_otats_init)
//{
//   ble_gatts_char_md_t char_md;
//   ble_gatts_attr_md_t cccd_md;
//   ble_gatts_attr_t    attr_char_value;
//   ble_uuid_t          ble_uuid;
//   ble_gatts_attr_md_t attr_md;

//   memset(&cccd_md, 0, sizeof(cccd_md));

//   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//   cccd_md.vloc = BLE_GATTS_VLOC_STACK;
//   
//   memset(&char_md, 0, sizeof(char_md));
//   
//   char_md.char_props.read   = 1;
//   char_md.char_props.notify = 1;
//   char_md.p_char_user_desc  = NULL;
//   char_md.p_char_pf         = NULL;
//   char_md.p_user_desc_md    = NULL;
//   char_md.p_cccd_md         = &cccd_md;
//   char_md.p_sccd_md         = NULL;
//   
//   ble_uuid.type = p_otats->uuid_type;
//   ble_uuid.uuid = OTATS_UUID_BUTTON_CHAR;
//   
//   memset(&attr_md, 0, sizeof(attr_md));

//   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//   BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
//   attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//   attr_md.rd_auth    = 0;
//   attr_md.wr_auth    = 0;
//   attr_md.vlen       = 0;
//   
//   memset(&attr_char_value, 0, sizeof(attr_char_value));

//   attr_char_value.p_uuid       = &ble_uuid;
//   attr_char_value.p_attr_md    = &attr_md;
//   attr_char_value.init_len     = sizeof(uint8_t);
//   attr_char_value.init_offs    = 0;
//   attr_char_value.max_len      = sizeof(uint8_t);
//   attr_char_value.p_value      = NULL;
//   
//   return sd_ble_gatts_characteristic_add(p_otats->service_handle, &char_md,
//                                              &attr_char_value,
//                                              &p_otats->button_char_handles);
//}

uint32_t ll_ble_otats_init(ll_ble_otats_t * p_otats, const ll_ble_otats_init_t * p_otats_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_otats->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_otats->otats_trig_char_write_handler = p_otats_init->otats_trig_char_write_handler;
    
    // Add service
    ble_uuid128_t base_uuid = {LL_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_otats->uuid_type); // Add a vendor specific ID; Reference http://bit.ly/1JLkeCY
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_otats->uuid_type;
    ble_uuid.uuid = OTATS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_otats->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
//   err_code = button_char_add(p_otats, p_otats_init);
//   if (err_code != NRF_SUCCESS)
//   {
//       return err_code;
//   }
    
    err_code = ota_trig_char_add(p_otats, p_otats_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}


//// This is how to send a notification 
//uint32_t ble_otats_on_button_change(ll_ble_otats_t * p_otats, uint8_t button_state)
//{
//   ble_gatts_hvx_params_t params;
//   uint16_t len = sizeof(button_state);
//   
//   memset(&params, 0, sizeof(params));
//   params.type = BLE_GATT_HVX_NOTIFICATION;    // hanlde value operation; reference http://bit.ly/1AZPE3r
//   params.handle = p_otats->button_char_handles.value_handle;
//   params.p_data = &button_state;
//   params.p_len = &len;
//   
//   return sd_ble_gatts_hvx(p_otats->conn_handle, &params);  // reference: http://bit.ly/1Hg5lbn
//}
