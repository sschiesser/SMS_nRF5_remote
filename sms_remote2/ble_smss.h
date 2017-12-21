/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_lbs LED Button Service Server
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief LED Button Service Server module.
 *
 * @details This module implements a custom LED Button Service with an LED and Button Characteristics.
 *          During initialization, the module adds the LED Button Service and Characteristics
 *          to the BLE stack database.
 *
 *          The application must supply an event handler for receiving LED Button Service
 *          events. Using this handler, the service notifies the application when the
 *          LED value changes.
 *
 *          The service also provides a function for letting the application notify
 *          the state of the Button Characteristic to connected peers.
 *
 * @note The application must propagate BLE stack events to the LED Button Service
 *       module by calling ble_lbs_on_ble_evt() from the @ref softdevice_handler callback.
*/

#ifndef BLE_SMSS_H__
#define BLE_SMSS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SMSS_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define SMSS_UUID_SERVICE     0x1C57
#define SMSS_UUID_BUTTON_CHAR 0x1C58
#define SMSS_UUID_PRESS_CHAR  0x1C59
#define SMSS_UUID_IMU_CHAR	  0x1C5A
#define SMSS_UUID_WRITE_CHAR 0x1C60
	
					  
typedef struct ble_smss_s ble_smss_t;
							  
typedef void (*ble_smss_app_write_handler_t)	(ble_smss_t * p_smss, uint8_t *data);
				
typedef struct
{
	ble_smss_app_write_handler_t app_write_function;
}ble_smss_init_t;	
							  
/**@brief LED Button Service structure. This structure contains various status information for the service. */
typedef struct ble_smss_s
{
    uint16_t                    	conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    uint16_t                    	service_handle;      /**< Handle of LED Button Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    	button_char_handles; /**< Handles related to the Button Characteristic. */
	ble_gatts_char_handles_t		press_char_handles;
	ble_gatts_char_handles_t		imu_char_handles;
	ble_gatts_char_handles_t		app_write_handles;
	ble_smss_app_write_handler_t	app_write_function;
	uint8_t							uuid_type;
}ble_smss_t;


//typedef void (*ble_smss_led_write_handler_t) (ble_smss_t * p_smss, uint8_t new_state);
//typedef void (*ble_smss_button_write_handler_t) (ble_smss_t * p_smss, uint8_t button_state);
//typedef void (*ble_smss_press_write_handler_t) (ble_smss_t * p_smss, uint8_t press_value);
//typedef void (*ble_smss_imu_write_handler_t) (ble_smss_t * p_smss, uint8_t imu_value);
							  

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_lbs      LED Button Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
void ble_smss_init(ble_smss_t * p_smss, const ble_smss_init_t * p_smss_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the LED Button Service.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_smss_on_ble_evt(ble_smss_t * p_smss, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_smss_on_button_change(ble_smss_t * p_smss, uint16_t button_state);

//uint32_t ble_smss_on_imu_value(ble_smss_t * p_smss, uint8_t * imu_value);
uint32_t ble_smss_on_imu_value(ble_smss_t * p_smss, uint32_t * imu_value);

uint32_t ble_smss_on_press_value(ble_smss_t * p_smss, int32_t * press_value);

#ifdef __cplusplus
}
#endif

#endif // BLE_SMSS_H__

/** @} */
