/*
* ms58.h
*
* Created: 10.06.2016 15:00:58
*  Author: Sebastien Schiesser
*/
#ifndef MS58_H_
#define MS58_H_

/* === Includes ============================================================= */
#include <stdbool.h>
#include <stdint.h>

/* === Macros =============================================================== */
#define MS58_RESET_WAIT_MS             (10) // actually need 3...

//#define MS58_INIT_RETRY_MAX            (4)

//#define MS58_ENABLE                    (0)
//#define MS58_DISABLE                   (1)

#define MS58_RESET                     (0x1E)
#define MS58_CONV_D1_256               (0x40)
#define	MS58_CONV_D1_512               (0x42)
#define	MS58_CONV_D1_1024              (0x44)
#define	MS58_CONV_D1_2048              (0x46)
#define	MS58_CONV_D1_4096              (0x48)
#define	MS58_CONV_D2_256               (0x50)
#define	MS58_CONV_D2_512               (0x52)
#define	MS58_CONV_D2_1024              (0x54)
#define	MS58_CONV_D2_2048              (0x56)
#define	MS58_CONV_D2_4096              (0x58)
#define	MS58_ADC_READ                  (0x00)
#define	MS58_PROM_READ_0               (0xA0)
#define	MS58_PROM_READ_1               (0xA2)
#define	MS58_PROM_READ_2               (0xA4)
#define	MS58_PROM_READ_3               (0xA6)
#define	MS58_PROM_READ_4               (0xA8)
#define	MS58_PROM_READ_5               (0xAA)
#define	MS58_PROM_READ_6               (0xAC)
#define	MS58_PROM_READ_7               (0xAE)

#define MS58_PROM_VAL_ERR				(65000)
#define MS58_PROM_VAL_MAX				(8)
#define MS58_ADC_VAL_MAX				(2)
#define MS58_BUF_SIZE					(4)

/* === Types ================================================================ */
enum ms58_datatype_tag {
	MS58_TYPE_PRESS = 0,
	MS58_TYPE_TEMP
};

typedef struct {
	bool dev_start; // command to lauch startup & initialization procedure
	bool init_ok; // PROM value have been successfully read
	bool dev_en;
//    uint8_t osr;
}ms58_config_s;

typedef struct {
	uint16_t prom_values[MS58_PROM_VAL_MAX];
	uint32_t adc_values[MS58_ADC_VAL_MAX];
	bool complete;
	int32_t pressure;
	int32_t temperature;
}ms58_output_s;

typedef struct {
	bool enabled;
	volatile bool new_value;
	volatile bool rts;
}ms58_interrupt_s;

#endif
