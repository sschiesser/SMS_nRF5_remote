#include "sms_pressure.h"
#include "ms58.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "PRESSURE"
#include "nrf_log.h"
#include "app_error.h"

#define SMS_PRESS_RESET_MS		4

extern const nrf_drv_spi_t spi_master_instance;
extern volatile bool spi_xfer_done;
extern uint8_t m_tx_buf[];
extern uint8_t m_rx_buf[];

ms58_output_s ms58_output;
ms58_config_s ms58_config;
ms58_interrupt_s ms58_interrupt;

static void ms58_reset(void)
{
	NRF_LOG_DEBUG("Resetting MS58\n\r");

	uint8_t tx_len = 1;
	uint8_t rx_len = 0;
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_RESET;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done);
	nrf_delay_ms(SMS_PRESS_RESET_MS);
}

static void ms58_read_prom(void)
{
	uint8_t tx_len = 1;
	uint8_t rx_len = 3;
	
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_1;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[1] = (m_rx_buf[1] << 8 | m_rx_buf[2]);
		
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_2;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[2] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_3;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[3] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_4;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[4] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_5;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[5] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_6;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[6] = (m_rx_buf[1] << 8 | m_rx_buf[2]);


	// Send the first conversion command
	tx_len = 1;
	rx_len = 0;
	m_tx_buf[0] = MS58_CONV_D1_4096;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done);
}

static void pressure_calculate(void)
{
    /***************************************************************************
    * Calculated values...
    * Note: - ms58_prom_values[] are uint16_t
    *       - ms58_adc_values[] are uint32_t
    * Typical values given on the MS5003-01BA datasheets (March 25, 2013) are:
    * -------------------------------------------------------------------------
    * ms58_prom_values[]          |   ms58_adc_values[]
    * - C1 = 40'127 (SENSt1)      |   - D1 = 9'085'466 (Digital pressure)
    * - C2 = 36'924 (OFFt1)       |   - D2 = 8'569'150 (Digital temperature)
    * - C3 = 23'317 (TCS)         |
    * - C4 = 23'282 (TCO)         |
    * - C5 = 33'464 (Tref)        |
    * - C6 = 28'312 (TEMPSENS)    |
    **************************************************************************/
    int32_t deltaT;
    int64_t offset, sensitivity, tv1, tv2, tv3;

    /***************************
    * Temperature calculation *
    ***************************/
    /* dT = D2 - Tref = D2 - C5*2^8 */
    /* tv1: 33464 * 2^8 = 8566784 */
    tv1 = ((int64_t)(ms58_output.prom_values[5]) << 8);
    /* deltaT: 8569150 - 8566784 = 2366 */
    deltaT = (int32_t)((int64_t)ms58_output.adc_values[MS58_TYPE_TEMP] - tv1);

    /* TEMP = 20°C + dT*TEMPSENS = 2000 + dT * C6/2^23 */
    /* tv1: 28312 * 2366 = 66986192 */
    tv1 = ((int64_t)ms58_output.prom_values[6] * (int64_t)deltaT);
    /* tv2: 66986192 / 2^23 = 7(.985376358) */
    tv2 = (tv1 >> 23);
    /* temp: 7 + 2000 = 2007 */
    ms58_output.temperature = (int32_t)(tv2 + 2000);

    /************************
    * Pressure calculation *
    ************************/
    /* OFF = OFFt1 + TCO*dT = C2*2^16 + (C4*dT)/2^7 */
    /* tv1: 36924 * 2^16 = 2419851264 */
    tv1 = ((int64_t)(ms58_output.prom_values[2]) << 16);
    /* tv2: 23282 * 2366 = 55085212 */
    tv2 = ((int64_t)ms58_output.prom_values[4] * (int64_t)deltaT);
    /* tv3: 55085212 / 2^7 = 430353(.21875) */
    tv3 = (tv2 >> 7);
    /* offset: 2419851264 + 430353 = 2420281617 */
    offset = (tv1 + tv3);

    /* SENS = SENSt1 + TCS*dT = C1*2^15 + (C3*dT)/2^8 */
    /* tv1: 40127 * 2^15 = 1314881536 */
    tv1 = ((int64_t)(ms58_output.prom_values[1]) << 15);
    /* tv2: 23317 * 2366 = 55168022 */
    tv2 = ((int64_t)ms58_output.prom_values[3] * (int64_t)deltaT);
    /* tv3: 55168022 / 2^8 = 215500(.0859375) */
    tv3 = (tv2 >> 8);
    /* sensitivity: 1314881536 + 215500 = 1315097036 */
    sensitivity = (tv1 + tv3);

    /* P = D1*SENS - OFF = (D1*SENS/2^21 - OFF)/2^15 */
    /* tv1: (9085466 * 1315097036) / 2^21 = 5697378829(.612148284) */
    tv1 = (((int64_t)ms58_output.adc_values[MS58_TYPE_PRESS] * sensitivity) >> 21);
    /* tv2: 5697378829 - 2420281617 = 3277097212 */
    tv2 = tv1 - offset;
    /* press: 3277097212 / 2^15 = 100009(.070190) */
    ms58_output.pressure = (int32_t)(tv2 >> 15);
	NRF_LOG_DEBUG("Pressure/Temperature (10^2): %ld/%ld\n\r", ms58_output.pressure, ms58_output.temperature);
}


static void pressure_startup(void)
{
	// Initialize all ms58 struct values
	ms58_config.init_ok = true;
	ms58_config.dev_en = false;
	ms58_output.complete = false;
	ms58_output.pressure = 0;
	ms58_output.temperature = 0;
	for(uint8_t i = 0; i < MS58_PROM_VAL_MAX; i++) {
		ms58_output.prom_values[i] = 0;
	}
	for(uint8_t i = 0; i < MS58_ADC_VAL_MAX; i++) {
		ms58_output.adc_values[i] = 0;
	}
	ms58_interrupt.enabled = false;
	ms58_interrupt.new_value = false;
	ms58_interrupt.rts = false;
	
	// Reset device & read PROM factory settings
	ms58_reset();
	ms58_read_prom();
	
	// Roughly test PROM values
	for(uint8_t i = 1; i < 7; i++) {
		NRF_LOG_DEBUG("MS58 PROM value %d: 0x%x\n\r", i, ms58_output.prom_values[i]);
		if((ms58_output.prom_values[i] == 0) || (ms58_output.prom_values[i] == 0xff)) {
			NRF_LOG_INFO("MS58 PROM suspect value: 0x%#x\r\n", ms58_output.prom_values[i]);
			ms58_config.init_ok = false;
		}
	}
	
	ms58_config.dev_en = ms58_config.init_ok;
}

void pressure_poll_data(void)
{
	uint8_t tx_len = 4;
	uint8_t rx_len = 4;
	
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_ADC_READ;
	m_tx_buf[1] = MS58_ADC_READ;
	m_tx_buf[2] = MS58_ADC_READ;
	m_tx_buf[3] = MS58_ADC_READ;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance,
					m_tx_buf, tx_len,
					m_rx_buf, rx_len));
	while(!spi_xfer_done) {};

	if(ms58_output.complete) {
		ms58_output.adc_values[MS58_TYPE_PRESS] = \
				((m_rx_buf[1] << 16) | (m_rx_buf[2] << 8) | (m_rx_buf[3]));
//		NRF_LOG_DEBUG("Getting Pressure... 0x%x (%d)\n\r",
//				ms58_output.adc_values[MS58_TYPE_PRESS],
//				ms58_output.adc_values[MS58_TYPE_PRESS]);
		m_tx_buf[0] = MS58_CONV_D2_4096;
	}
	else {
		ms58_output.adc_values[MS58_TYPE_TEMP] = \
				((m_rx_buf[1] << 16) | (m_rx_buf[2] << 8) | (m_rx_buf[3]));
//		NRF_LOG_DEBUG("Getting Temperature...0x%x (%d)\n\r",
//				ms58_output.adc_values[MS58_TYPE_TEMP],
//				ms58_output.adc_values[MS58_TYPE_TEMP]);
		m_tx_buf[0] = MS58_CONV_D1_4096;
	}
	tx_len = 1;
	rx_len = 0;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance,
					m_tx_buf, tx_len,
					m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	
	if(ms58_output.complete) {
		ms58_output.complete = false;
		pressure_calculate();
		ms58_interrupt.rts = true;
	}
	else {
		ms58_output.complete = true;
	}
}

void pressure_enable(void)
{
//	NRF_LOG_DEBUG("Enabling pressure sensor...\n\r");
	pressure_startup();
//	NRF_LOG_DEBUG("MS58 enabled? %d\r\n\n", ms58_config.dev_en);
}

