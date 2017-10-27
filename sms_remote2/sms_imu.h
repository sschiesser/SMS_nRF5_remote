#ifndef SMS_IMU_H_
#define SMS_IMU_H_

#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
//#include "softdevice_handler.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
//#include "app_timer.h"
#include "nrf_drv_timer.h"
//#include "SEGGER_RTT.h"
#include "bno055.h"

#define SMS_IMU_CHAR_LEN		16

#define SMS_IMU_DATAMSK_GRV		(1 << 6)
#define SMS_IMU_DATAMSK_LIA		(1 << 5)
#define SMS_IMU_DATAMSK_EULER	(1 << 4)
#define SMS_IMU_DATAMSK_QUAT	(1 << 3)
#define SMS_IMU_DATAMSK_MAG		(1 << 2)
#define SMS_IMU_DATAMSK_GYRO	(1 << 1)
#define SMS_IMU_DATAMSK_ACCEL	(1 << 0)

#define SMS_IMU_POLL_MS			19

#define SMS_IMU_SUPPLY_PIN		16
#define SMS_IMU_SW_ON			0
#define SMS_IMU_SW_OFF			1


#define PI						(3.1415926535897932384626433832795)

// IMU data mask:
// bit:  7  -  6  -  5  -   4   -   3  -  2  -   1  -   0 
// val: N/A - grv - lia - euler - quat - mag - gyro - accel
void imu_poll_data(uint8_t data_msk);
void imu_enable(void);

void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void twi_init(void);

#endif
