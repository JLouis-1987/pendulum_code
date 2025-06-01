/*
 * lsm6dso.c
 *
 *  Created on: May 25, 2025
 *      Author: gaspr
 */


#include "lsm6dso.h"

#define I2C_ADDR				(0b1101011)   //SDO is tied to VDD
/*Memory Addresses of Registers */
#define WHO_AM_I_R				(0x0F)
#define CTRL1_XL				(0x10)
#define CTRL2_G					(0x11)

/*Settings */
#define ODR_XL					(0b0100) 	// with XL_HM_MODE = 0, sets data rate to 104 Hz
#define FS_XL					(0b10)		// Full scale = 4g (table 45 of data sheet)
#define LPF2_XL_EN				(0b0) 		// disable LPF2 for accelerometer
#define ODR_G					(0b0100)	// with G_HM_MODE = 0, set data rate to 104 Hz
#define FS_G					(0b01)		// Full scall = 500 dps (table 47 of data sheet)
#define HP_EN_G					(0b1)		// Enable high pass filter
#define HPM_G					(0b10)		// Set high pass filter to 3 seconds settling time

typedef struct imu_data_t {
  uint8_t* buffer;
  int16_t XL_x;
  uint32_t read_index;
  uint32_t write_index;
} imu_data_t;

bool check_IMU(void)
{
	/*
	 * Function: check_IMU
	 * ----------------------
	 * This function checks the device ID of the lsm6dso to verify it is online and
	 * i2c communications are working correctly.
	 *
	 * Parameters:
	 *   None
	 *
	 * Returns:
	 *   boolean - true if Address is read correctly, false if address is not correct.
	 */
	char DeviceID;

	I2C1_Read(I2C_ADDR, WHO_AM_I_R, 1, &DeviceID);

	if(DeviceID == 0x6C){
		return true;
	}
	else{
		return false;
	}
}

void init_lsm6dso(void)
{

	char new_register_val;
	/* Accelerometer set up  CTRL1_XL
	 * set CTRL1_XL register - Contains 3 settings
	 * ODR_XL, FS_XL & LPF2_XL_EN
	 * Note the following default settings are used:
	 * XL_HM_MODE = 0 to enable high performance mode
	 * HP_SLOPE_XL_EN = 0 to just have the LPF1 Digital LP filter enabled
	 * LPF2_XL_EN = 0  to just have the LPF1 Digital LP filter enabled
	 * See tables 8 and 9 in the application note.
	 */
	new_register_val = (ODR_XL << 4) | (FS_XL << 2) | (LPF2_XL_EN << 1);
	I2C1_Write(I2C_ADDR, CTRL1_XL, 1, &new_register_val);


	/* Gyro Setup CTRL2_G
	 * set data rate of both sensors
	 * Set ODR_G to 0100 and G_HM_MODE to 0
	 * - Give 104 Hz sample rate in High performance mode
	 * See tables 8 and 9 in the application note.*/
	new_register_val = (ODR_G << 4) | (FS_G << 2);
	I2C1_Write(I2C_ADDR, CTRL1_XL, 1, &new_register_val);


	/*set Gyro filtering CTRL7_G
	 * High Pass filter (to limit drift)
	 * HP_EN_G = 1
	 * HPM_G[1:0] = 10 -> 3 seconds settling time (see table 13 of app note)
	 */
	new_register_val = (HP_EN_G << 6) | (HPM_G << 4);
	I2C1_Write(I2C_ADDR, CTRL1_XL, 1, &new_register_val);

}
