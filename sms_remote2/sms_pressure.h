#ifndef SMS_PRESSURE_H_
#define SMS_PRESSURE_H_

#include "ms58.h"

#define SMS_PRESSURE_PRESS_LEN		4
#define SMS_PRESSURE_TEMP_LEN		4
#define SMS_PRESSURE_CHAR_LEN		(SMS_PRESSURE_PRESS_LEN + SMS_PRESSURE_TEMP_LEN)

#define SMS_PRESSURE_POLL_MS		49

#define SMS_PRESSURE_SUPPLY_PIN		15
#define SMS_PRESSURE_SW_ON			1
#define SMS_PRESSURE_SW_OFF			0


//void pressure_startup(void);
void pressure_enable(void);
void pressure_poll_data(void);
//void pressure_calculate(void);


#endif
