#include "sms_imu.h"
//#include "nrf_drv_delay.h"
#define NRF_LOG_MODULE_NAME "IMU"
#include "nrf_log.h"

extern const nrf_drv_twi_t twi_master_instance;

extern volatile bool twi_xfer_done;

bno055_config_s bno055_config;
bno055_output_s bno055_output;
bno055_interrupt_s bno055_interrupt;

//extern const nrf_drv_timer_t TIMER_DELTA_US;

static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	uint8_t reg[2] = {subAddress, data};
	twi_xfer_done = false;
	ret_code_t err_code = nrf_drv_twi_tx(&twi_master_instance, address, reg, sizeof(reg), false);
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done) {};
}

static uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t reg[1] = {subAddress};
	twi_xfer_done = false;
	ret_code_t err_code = nrf_drv_twi_tx(&twi_master_instance, address, reg, sizeof(reg), false);
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done);
		
	uint8_t data;
	twi_xfer_done = false;
	err_code = nrf_drv_twi_rx(&twi_master_instance, address, &data, sizeof(data));
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done);
	
	return data;
}

static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
	uint8_t reg[1] = {subAddress};
	twi_xfer_done = false;
	ret_code_t err_code = nrf_drv_twi_tx(&twi_master_instance, address, reg, sizeof(reg), false);
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done);
	
	twi_xfer_done = false;
	err_code = nrf_drv_twi_rx(&twi_master_instance, address, dest, count);
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done);
}

static void bno055_reset(void)
{
	writeByte(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x20);
	nrf_delay_ms(500);
}


static uint8_t bno055_check(void)
{
	int ret = 0x0F;
	uint8_t c = readByte(BNO055_ADDRESS, BNO055_CHIP_ID);
	NRF_LOG_DEBUG("Device returned 0x%02x\r\n", c);
	if(c == 0xA0) ret = 0x07;
	nrf_delay_ms(500);
	c = readByte(BNO055_ADDRESS, BNO055_ACC_ID);
	NRF_LOG_DEBUG("Accelerometer returned 0x%02x\r\n", c);
	if(c == 0xFB) ret = 0x03;
	nrf_delay_ms(500);
	c = readByte(BNO055_ADDRESS, BNO055_MAG_ID);
	NRF_LOG_DEBUG("Magnetometer returned 0x%02x\r\n", c);
	if(c == 0x32) ret = 0x01;
	nrf_delay_ms(500);
	c = readByte(BNO055_ADDRESS, BNO055_GYRO_ID);
	NRF_LOG_DEBUG("Gyroscope returned 0x%02x\r\n", c);
	if(c == 0x0F) ret = 0;
	nrf_delay_ms(500);
	
	return ret;
}



static int bno055_test(void)
{
	// Check software revision ID
	uint8_t swlsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_LSB);
	uint8_t swmsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_MSB);
	NRF_LOG_INFO("Software revision ID: %02x.%02x\r\n", swmsb, swlsb);
	
	// Check bootloader version
	uint8_t blid = readByte(BNO055_ADDRESS, BNO055_BL_REV_ID);
	NRF_LOG_INFO("Bootloader version: %d\r\n", blid);
	
	// Check self-test results
	uint8_t selftest = readByte(BNO055_ADDRESS, BNO055_ST_RESULT);
	if(selftest & 0x01) {
		NRF_LOG_DEBUG("Accelerometer passed selftest\r\n");
	}
	if(selftest & 0x02) {
		NRF_LOG_DEBUG("Magnetometer passed selftest\r\n");
	}
	if(selftest & 0x04) {
		NRF_LOG_DEBUG("Gyroscope passed selftest\r\n");
	}
	if(selftest & 0x08) {
		NRF_LOG_INFO("MCU passed selftest\r\n");
	}
	
	nrf_delay_ms(1000);
}




static void bno055_init_config_values(void)
{
	bno055_config.g_pwr_mode = NormalG;	// Gyro power mode
	bno055_config.g_scale = GFS_250DPS;	// Gyro full scale
	bno055_config.g_bw = GBW_523Hz;		// Gyro bandwidth
	bno055_config.a_pwr_mode = NormalA;	// Accel power mode
	bno055_config.a_scale = AFS_2G;		// Accel full scale
	bno055_config.a_bw = ABW_1000Hz;	// Accel bandwidth, accel sample rate divided by ABW_divx
	bno055_config.m_pwr_mode = Normal;	// Select magnetometer power mode
	bno055_config.m_op_mode = Regular;	// Select magnetometer perfomance mode
	bno055_config.m_odr = MODR_30Hz;	// Select magnetometer ODR when in BNO055 bypass mode
	bno055_config.pwr_mode = Normalpwr;	// Select BNO055 power mode
	bno055_config.opr_mode = NDOF;		// specify operation mode for sensors
	NRF_LOG_DEBUG("BNO055 config values initialized\r\n");
}

static void bno055_calibrate_accel_gyro(float *dest1, float *dest2)
{
	uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii = 0, sample_count = 0;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	NRF_LOG_INFO("Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait......");
	nrf_delay_ms(4000);
  
	// Select page 0 to read sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
	// Select BNO055 system operation mode as AMG for calibration
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG);
   
	// In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz, set it the same here
	writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, bno055_config.a_pwr_mode << 5 | bno055_config.a_bw << 2 | AFS_4G );
	sample_count = 256;
	for(ii = 0; ii < sample_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		accel_bias[0]  += (int32_t) accel_temp[0];
		accel_bias[1]  += (int32_t) accel_temp[1];
		accel_bias[2]  += (int32_t) accel_temp[2];
		nrf_delay_ms(20);  // at 62.5 Hz ODR, new accel data is available every 16 ms
	}
	accel_bias[0]  /= (int32_t) sample_count;  // get average accel bias in mg
	accel_bias[1]  /= (int32_t) sample_count;
	accel_bias[2]  /= (int32_t) sample_count;
    
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) 1000;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) 1000;}

	dest1[0] = (float) accel_bias[0];  // save accel biases in mg for use in main program
	dest1[1] = (float) accel_bias[1];  // accel data is 1 LSB/mg
	dest1[2] = (float) accel_bias[2];          

	// In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, bno055_config.g_bw << 3 | GFS_2000DPS );
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, bno055_config.g_pwr_mode);
	
	for(ii = 0; ii < sample_count; ii++) {
		int16_t gyro_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;  // Form signed 16-bit integer for each sample in FIFO
		gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
		nrf_delay_ms(35);  // at 32 Hz ODR, new gyro data available every 31 ms
	}
	gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias in counts
	gyro_bias[1]  /= (int32_t) sample_count;
	gyro_bias[2]  /= (int32_t) sample_count;

	dest2[0] = (float) gyro_bias[0]/16.;  // save gyro biases in dps for use in main program
	dest2[1] = (float) gyro_bias[1]/16.;  // gyro data is 16 LSB/dps
	dest2[2] = (float) gyro_bias[2]/16.;          

	// Return to config mode to write accelerometer biases in offset register
	// This offset register is only used while in fusion mode when accelerometer full-scale is +/- 4g
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);

	//write biases to accelerometer offset registers ad 16 LSB/dps
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB, (int16_t)accel_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB, ((int16_t)accel_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB, (int16_t)accel_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB, ((int16_t)accel_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB, (int16_t)accel_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB, ((int16_t)accel_bias[2] >> 8) & 0xFF);

	// Check that offsets were properly written to offset registers
	NRF_LOG_INFO("Average accelerometer bias = %d, %d, %d\n",\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB)),\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB)),\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB)));
	
	//write biases to gyro offset registers
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB, (int16_t)gyro_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB, ((int16_t)gyro_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB, (int16_t)gyro_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB, ((int16_t)gyro_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB, (int16_t)gyro_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB, ((int16_t)gyro_bias[2] >> 8) & 0xFF);

	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, bno055_config.opr_mode );

	// Check that offsets were properly written to offset registers
	NRF_LOG_INFO("Average gyro bias = %d, %d, %d\n",\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB)),\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB)),\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB)));
	
	nrf_delay_ms(1000);
	
	NRF_LOG_INFO("Accel/Gyro Calibration done!\r\n");
}

static void bno055_calibrate_mag(float *dest1)
{
	uint8_t data[6]; // data array to hold mag x, y, z, data
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0};
	int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

	NRF_LOG_INFO("Mag Calibration: Wave device in a figure eight until done!");
	nrf_delay_ms(4000);

	// Select page 0 to read sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
	// Select BNO055 system operation mode as NDOF for calibration
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG );

	// In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
	sample_count = 256;
	for(ii = 0; ii < sample_count; ii++) {
		int16_t mag_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
		mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		for (int jj = 0; jj < 3; jj++) {
			if (ii == 0) {
				mag_max[jj] = mag_temp[jj]; // Offsets may be large enough that mag_temp[i] may not be bipolar! 
				mag_min[jj] = mag_temp[jj]; // This prevents max or min being pinned to 0 if the values are unipolar...
			} else {
				if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
		}
		nrf_delay_ms(105);  // at 10 Hz ODR, new mag data is available every 100 ms
	}

	NRF_LOG_INFO("mag x min/max: %d, %d\r\n", mag_min[0], mag_max[0]);
	NRF_LOG_INFO("mag y min/max: %d, %d\r\n", mag_min[1], mag_max[1]);
	NRF_LOG_INFO("mag z min/max: %d, %d\r\n", mag_min[2], mag_max[2]);
	
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	dest1[0] = (float) mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
	dest1[1] = (float) mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
	dest1[2] = (float) mag_bias[2] / 1.6;          

	// Return to config mode to write mag biases in offset register
	// This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);

	//write biases to accelerometer offset registers as 16 LSB/microTesla
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB, (int16_t)mag_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB, ((int16_t)mag_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB, (int16_t)mag_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB, ((int16_t)mag_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB, (int16_t)mag_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB, ((int16_t)mag_bias[2] >> 8) & 0xFF);

	// Check that offsets were properly written to offset registers
	NRF_LOG_INFO("Average magnetometer bias = %d, %d, %d\n",\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB)),\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB)),\
		(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB)));
	
	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, bno055_config.opr_mode);
	nrf_delay_ms(25);

	nrf_delay_ms(1000);
	
	NRF_LOG_INFO("Mag Calibration done!\r\n");
}



// Read accel data
static void read_accel_data(int16_t *destination)
{
	// x/y/z accel register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers into data array
	readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];      
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4]; 
}
// Read gyro data
static void read_gyro_data(int16_t *destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4]; 
}
// Read temperature data
//static int16_t read_gyrotemp_data(void)
//{
//  return readByte(BNO055_ADDRESS, BNO055_TEMP);  // Read the two raw data registers sequentially into data array 
//}

// Read magnetometer data
static void read_mag_data(int16_t *destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

// Read quaternion data
static void read_quat_data(int16_t * destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[8];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	destination[3] = ((int16_t)rawData[7] << 8) | rawData[6];
}

// Read Euler angle data
static void read_euler_data(int16_t * destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

// Read linear acceleration data
static void read_lia_data(int16_t * destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

// Read gravity vector
static void read_grv_data(int16_t * destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}


static void imu_startup(void)
{
	// Initialize all bno055 struct values
	bno055_config.init_ok = true;
	bno055_config.dev_en = false;
	bno055_config.comp_mask = 0x0F;
	for(uint8_t i = 0; i < 3; i++) {
		bno055_config.gyro_bias[i] = 0;
		bno055_config.accel_bias[i] = 0;
		bno055_config.mag_bias[i] = 0;
		bno055_output.accel[i].val = 0;
		bno055_output.gyro[i].val = 0;
		bno055_output.mag[i].val = 0;
		bno055_output.lia[i].val = 0;
		bno055_output.grv[i].val = 0;
	}
	for(uint8_t i = 0; i < 4; i++) {
		bno055_output.quat[i].val = 0;
	}
	bno055_output.temp.val = 0;
	bno055_output.yaw.val = 0;
	bno055_output.roll.val = 0;
	bno055_output.pitch.val = 0;
	bno055_output.ts_us = 0;
	
	// Reset device & check which component is present
	bno055_reset();
	bno055_config.comp_mask = bno055_check();
	if(bno055_config.comp_mask == 0x0F) {
		bno055_config.init_ok = false;
		bno055_config.dev_en = false;
	}
	else {
		bno055_config.init_ok = true;
		bno055_config.dev_en = true;
	}
}


static void imu_configure(void)
{
	bno055_test();
	bno055_init_config_values();
//	bno055_calibrate_accel_gyro(bno055_config.accel_bias,
//								bno055_config.gyro_bias);
//	bno055_calibrate_mag(bno055_config.mag_bias);
}


static void imu_check_cal(void)
{
	bno055_config.cal_state = readByte(BNO055_ADDRESS, BNO055_CALIB_STAT);
}

static void imu_initialize(void)
{
	// Select BNO055 config mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);
	// Select page 1 to configure sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x01);
	// Configure ACC
	writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, bno055_config.a_pwr_mode << 5 | bno055_config.a_bw << 2 | bno055_config.a_scale);
	// Configure GYR
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, bno055_config.g_bw << 3 | bno055_config.g_scale);
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, bno055_config.g_pwr_mode);
	// Configure MAG
	writeByte(BNO055_ADDRESS, BNO055_MAG_CONFIG, bno055_config.m_pwr_mode << 5 | bno055_config.m_op_mode << 3 | bno055_config.m_odr);

//	// Configure interrupt settings...
//	// Accelerometer any motion interrupt (4:2)
//	writeByte(BNO055_ADDRESS, BNO055_ACC_INT_SETTINGS, 0x1C);
//	writeByte(BNO055_ADDRESS, BNO055_ACC_AM_THRES, 0x00);
//	// Gyroscope any motion interrupt (
//	writeByte(BNO055_ADDRESS, BNO055_GYR_INT_SETTINGS, 0x47);
//	writeByte(BNO055_ADDRESS, BNO055_GYR_AM_THRESH, 0x00);
//	writeByte(BNO055_ADDRESS, BNO055_GYR_AM_SET, 0x00);
//	// Set interrupt mask (6 -> ACC_AM, 2 -> GYR_AM)
//	writeByte(BNO055_ADDRESS, BNO055_INT_MSK, 0x44);
//	// Enable interrupt
//	writeByte(BNO055_ADDRESS, BNO055_INT_EN, 0x44);
	
	// Select page 0 to read sensors & configure orientation
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
	
	// Set BNO055 axis configuration P3 (0x21 / 0x02)
	writeByte(BNO055_ADDRESS, BNO055_AXIS_MAP_CONFIG, 0x21);
	writeByte(BNO055_ADDRESS, BNO055_AXIS_MAP_SIGN, 0x02);

	// Select BNO055 gyro temperature source 
	writeByte(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01 );

	// Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
	writeByte(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01 );
	
	// Select BNO055 system power mode
	writeByte(BNO055_ADDRESS, BNO055_PWR_MODE, bno055_config.pwr_mode);

	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, bno055_config.opr_mode);
	nrf_delay_ms(25);
}

//static void bno055_int_reset(void)
//{
//	writeByte(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x40);
//}


void imu_poll_data(uint8_t data_msk)
{
	float ax, ay, az, gx, gy, gz, mx, my, mz;

	if(data_msk & SMS_IMU_DATAMSK_ACCEL) {
		int16_t data[3];
		read_accel_data(data);  // Read the x/y/z adc values
		// Now we'll calculate the accleration value into actual mg's
		ax = (float)data[0]; // - accelBias[0];  // subtract off calculated accel bias
		ay = (float)data[1]; // - accelBias[1];
		az = (float)data[2]; // - accelBias[2];
		// Fill the sensor output table for later sending
		bno055_output.accel[0].val = ax;
		bno055_output.accel[1].val = ay;
		bno055_output.accel[2].val = az;
		NRF_LOG_DEBUG("Raw acceleration (10^6): %x, %x, %x\n\r",
				(int32_t)(ax*1000000.),
				(int32_t)(ay*1000000.),
				(int32_t)(az*1000000.));
	}
	
	if(data_msk & SMS_IMU_DATAMSK_GYRO) {
		int16_t data[3];
		read_gyro_data(data);  // Read the x/y/z adc values
		// Calculate the gyro value into actual degrees per second
		gx = (float)data[0]/16.; // - gyroBias[0];  // subtract off calculated gyro bias
		gy = (float)data[1]/16.; // - gyroBias[1];  
		gz = (float)data[2]/16.; // - gyroBias[2];  
		// Fill the sensor output table for later sending
		bno055_output.gyro[0].val = gx;
		bno055_output.gyro[1].val = gy;
		bno055_output.gyro[2].val = gz;
		NRF_LOG_DEBUG("Raw gyroscope (10^6)   : %ld, %ld, %ld\n\r",
				(int32_t)(gx*1000000.),
				(int32_t)(gy*1000000.),
				(int32_t)(gz*1000000.));
	}

	if(data_msk & SMS_IMU_DATAMSK_MAG) {
		int16_t data[3];
		read_mag_data(data);  // Read the x/y/z adc values   
		// Calculate the magnetometer values in milliGauss
		mx = (float)data[0]/1.6; // - magBias[0];  // get actual magnetometer value in mGauss 
		my = (float)data[1]/1.6; // - magBias[1];  
		mz = (float)data[2]/1.6; // - magBias[2];   
		// Fill the sensor output table for later sending
		bno055_output.mag[0].val = mx;
		bno055_output.mag[1].val = my;
		bno055_output.mag[2].val = mz;
		NRF_LOG_DEBUG("Raw magnetometer (10^6): %ld, %ld, %ld\n\r",
				(int32_t)(mx*1000000.),
				(int32_t)(my*1000000.),
				(int32_t)(mz*1000000.));
	}
    
	if(data_msk & SMS_IMU_DATAMSK_QUAT) {
		float q[4];
		int16_t data[4];
		read_quat_data(data);  // Read the x/y/z adc values   
		// Calculate the quaternion values  
		q[0] = (float)(data[0])/16384.;    
		q[1] = (float)(data[1])/16384.;  
		q[2] = (float)(data[2])/16384.;   
		q[3] = (float)(data[3])/16384.;   
		// Fill the sensor output table for later sending
		bno055_output.quat[0].val = q[0];
		bno055_output.quat[1].val = q[1];
		bno055_output.quat[2].val = q[2];
		bno055_output.quat[3].val = q[3];
		NRF_LOG_DEBUG("Raw quaternions (10^6) : %ld, %ld, %ld, %ld\n\r",
				(int32_t)(q[0]*1000000.),
				(int32_t)(q[1]*1000000.),
				(int32_t)(q[2]*1000000.),
				(int32_t)(q[3]*1000000.));
	}
	
	if(data_msk & SMS_IMU_DATAMSK_EULER) {
		float yaw, roll, pitch;
		int16_t data[3];
		read_euler_data(data);  // Read the x/y/z adc values   
		// Calculate the Euler angles values in degrees
		yaw = (float)data[0]/16.;  
		roll = (float)data[1]/16.;  
		pitch = (float)data[2]/16.;   
		// Fill the sensor output table for later sending
		bno055_output.yaw.val = yaw;
		bno055_output.roll.val = roll;
		bno055_output.pitch.val = pitch;
		NRF_LOG_DEBUG("Raw euler angles (10^6): %ld, %ld, %ld\n\r",
				(int32_t)(yaw*1000000.),
				(int32_t)(roll*1000000.),
				(int32_t)(pitch*1000000.));
	}
	
	if(data_msk & SMS_IMU_DATAMSK_LIA) {
		float lia[3];
		int16_t data[3];
		read_lia_data(data);  // Read the x/y/z adc values   
		// Calculate the linear acceleration (sans gravity) values in mg
		lia[0] = (float)data[0];  
		lia[1] = (float)data[1];  
		lia[2] = (float)data[2];   
		// Fill the sensor output table for later sending
		bno055_output.lia[0].val = lia[0];
		bno055_output.lia[1].val = lia[1];
		bno055_output.lia[2].val = lia[2];
		NRF_LOG_DEBUG("Raw LIA (10^6)         : %ld, %ld, %ld\n\r",
				(int32_t)(lia[0]*1000000.),
				(int32_t)(lia[1]*1000000.),
				(int32_t)(lia[2]*1000000.));
	}
	
	if(data_msk & SMS_IMU_DATAMSK_GRV) {
		float grv[3];
		int16_t data[3];
		read_grv_data(data);  // Read the x/y/z adc values   
		// Calculate the linear acceleration (sans gravity) values in mg
		grv[0] = (float)data[0];  
		grv[1] = (float)data[1];  
		grv[2] = (float)data[2];   
		// Fill the sensor output table for later sending
		bno055_output.grv[0].val = grv[0];
		bno055_output.grv[1].val = grv[1];
		bno055_output.grv[2].val = grv[2];
		NRF_LOG_DEBUG("Raw gravity (10^6)     : %ld, %ld, %ld\n\r",
				(int32_t)(grv[0]*1000000.),
				(int32_t)(grv[1]*1000000.),
				(int32_t)(grv[2]*1000000.));
	}
	
	//Chose (once) between unprecise but low-power ms app timer
	// and precise current-demanding us drv timer 
//	static uint32_t last_time_ms = 0;
//	uint32_t now_ms = app_timer_cnt_get();
//	uint32_t delta_ms = (now_ms - last_time_ms)/33;
//	last_time_ms = now_ms;
//	static uint32_t last_time_us = 0;
//	uint32_t delta_us;
//	uint32_t now_us = nrf_drv_timer_capture(&TIMER_DELTA_US, NRF_TIMER_CC_CHANNEL0);
//	if(now_us > last_time_us) {
//		delta_us = now_us - last_time_us;
//	}
//	else {
//		delta_us = 0xFFFFFFFF - last_time_us + now_us;
//	}
//	last_time_us = now_us;
//	// Fill the sensor output table for later sending
//	bno055_output.ts_us = now_us;
//	NRF_LOG_INFO("Timestamp/Delta (us)   : %ld/%ld\n\r", bno055_output.ts_us, delta_us);
	
	bno055_interrupt.rts = true;
}


void imu_enable(void)
{
	imu_startup();
//	NRF_LOG_DEBUG("BNO055 enabled? %d\r\n\n", bno055_config.dev_en);
	if(bno055_config.dev_en) {
		imu_configure();
		imu_check_cal();
		NRF_LOG_DEBUG("System calibration: %d\n\r",
					((0xC0 & bno055_config.cal_state) >> 6));
		NRF_LOG_DEBUG("Gyro   calibration: %d\n\r",
					((0x30 & bno055_config.cal_state) >> 4));
		NRF_LOG_DEBUG("Accel  calibration: %d\n\r",
					((0x0C & bno055_config.cal_state) >> 2));
		NRF_LOG_DEBUG("Mag    calibration: %d\n\r",
					((0x03 & bno055_config.cal_state) >> 0));
		imu_initialize();
	}
}
