/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */
#include "sdk_common.h"
#include "ble_smss.h"
#include "ble_srv_common.h"
#define NRF_LOG_MODULE_NAME "SMSS"
#include "nrf_log.h"
#include "app_error.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_smss_t * p_smss, ble_evt_t * p_ble_evt)
{
    p_smss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_smss_t * p_smss, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_smss->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_smss_t * p_smss, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	
	if((p_evt_write->handle == p_smss->app_write_handles.value_handle) &&
		(p_evt_write->len == 4) &&
		(p_smss->app_write_function != NULL))
	{
		p_smss->app_write_function(p_smss, p_evt_write->data);
	}
}


void ble_smss_on_ble_evt(ble_smss_t * p_smss, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_smss, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_smss, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_smss, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Button Characteristic.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_lbs_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t button_char_add(ble_smss_t * p_smss)
{
	uint32_t			err_code = 0;
	
	// Add custom characteristic UUID (2.A)
	ble_uuid_t			char_uuid;
	ble_uuid128_t		base_uuid = SMSS_UUID_BASE;
	char_uuid.uuid = SMSS_UUID_BUTTON_CHAR;
	sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
	APP_ERROR_CHECK(err_code);
	
	// Add read/write properties to characteristic (2.F)
	ble_gatts_char_md_t	char_md;
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;
	
	// Configure CCCD (3.A)
	ble_gatts_attr_md_t	cccd_md;
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	char_md.p_cccd_md = &cccd_md;
	char_md.char_props.notify = 1;
	
	// Configure attribute metadata (2.B)
	ble_gatts_attr_md_t	attr_md;
	memset(&attr_md, 0, sizeof(attr_md));
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	
	// Set read/write security level to characteristic (2.G)
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
	// Configure characteristic value attribute (2.C)
	ble_gatts_attr_t	attr_char_value;
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &char_uuid;
	attr_char_value.p_attr_md = &attr_md;
	
	// Set characteristic length (2.H)
	attr_char_value.max_len = 2;
	attr_char_value.init_len = 2;
	uint8_t value[2] = {0};
	attr_char_value.p_value = value;
	
	// Add new characteristic to the service
	err_code = sd_ble_gatts_characteristic_add(p_smss->service_handle,
												&char_md,
												&attr_char_value,
												&p_smss->button_char_handles);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEBUG("Button characteristic added\n\r");
	NRF_LOG_DEBUG("- service handle: %#x\n\r", p_smss->service_handle);
	NRF_LOG_DEBUG("- char value handle: %#x\r\n", p_smss->button_char_handles.value_handle);
	NRF_LOG_DEBUG("- char cccd handle: %#x\r\n\r\n", p_smss->button_char_handles.cccd_handle);
	
	return NRF_SUCCESS;
}




static uint32_t app_write_char_add(ble_smss_t * p_smss)
{
	uint32_t			err_code = 0;
	
	// Add custom characteristic UUID (2.A)
	ble_uuid_t			char_uuid;
	ble_uuid128_t		base_uuid = SMSS_UUID_BASE;
	char_uuid.uuid = SMSS_UUID_WRITE_CHAR;
	sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
	APP_ERROR_CHECK(err_code);
	
	// Add read/write properties to characteristic (2.F)
	ble_gatts_char_md_t	char_md;
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;
	
	// Configure CCCD (3.A)
	ble_gatts_attr_md_t	cccd_md;
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	char_md.p_cccd_md = &cccd_md;
	char_md.char_props.notify = 1;
	
	// Configure attribute metadata (2.B)
	ble_gatts_attr_md_t	attr_md;
	memset(&attr_md, 0, sizeof(attr_md));
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	
	// Set read/write security level to characteristic (2.G)
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
	// Configure characteristic value attribute (2.C)
	ble_gatts_attr_t	attr_char_value;
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &char_uuid;
	attr_char_value.p_attr_md = &attr_md;
	
	// Set characteristic length (2.H)
	attr_char_value.max_len = 4;
	attr_char_value.init_len = 4;
	uint8_t value[4] = {0};
	attr_char_value.p_value = value;
	
	// Add new characteristic to the service
	err_code = sd_ble_gatts_characteristic_add(p_smss->service_handle,
												&char_md,
												&attr_char_value,
												&p_smss->app_write_handles);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEBUG("Application update characteristic added\n\r");
	NRF_LOG_DEBUG("- service handle: %#x\n\r", p_smss->service_handle);
	NRF_LOG_DEBUG("- char value handle: %#x\r\n", p_smss->app_write_handles.value_handle);
	NRF_LOG_DEBUG("- char cccd handle: %#x\r\n\r\n", p_smss->app_write_handles.cccd_handle);
	
	return NRF_SUCCESS;
}



void ble_smss_init(ble_smss_t * p_smss, const ble_smss_init_t * p_smss_init)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // FROM_SERVICE_TUTORIAL: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = SMSS_UUID_BASE;
    service_uuid.uuid = SMSS_UUID_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    p_smss->conn_handle = BLE_CONN_HANDLE_INVALID;
	// Adding app update function pointer
	p_smss->app_write_function = p_smss_init->app_write_function;

    // FROM_SERVICE_TUTORIAL: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_smss->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    button_char_add(p_smss);
	app_write_char_add(p_smss);
}

uint32_t ble_smss_on_button_change(ble_smss_t * p_smss, uint16_t button_state)
{
	NRF_LOG_DEBUG("on button change...\r\n");
	if(p_smss->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		ble_gatts_hvx_params_t hvx_params;
		uint16_t len = sizeof(button_state);
		memset(&hvx_params, 0, sizeof(hvx_params));
		
		hvx_params.handle = p_smss->button_char_handles.value_handle;
		hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len = &len;
		hvx_params.p_data = (uint8_t*)&button_state;

		NRF_LOG_DEBUG("Sending: %#x to 0x%04x->0x%04x\r\n",
					hvx_params.p_data[0],
					p_smss->conn_handle,
					hvx_params.handle);
		
		return sd_ble_gatts_hvx(p_smss->conn_handle, &hvx_params);
	}
	return BLE_ERROR_INVALID_CONN_HANDLE;
}
