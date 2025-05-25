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
#define LPF2_XL_EN				(0b0) 		//disable LPF2 for accelerometer
#define XL_HM_MODE				(0b0)		//enable High Performance mode
#define XL_ULP_EN				(0b0)		//disable ultra low power mode




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
	char read_register_val;
	/*enable accelerometer and gyro*/

	/*set CTRL1_XL register - Contains 3 settings
	 * ODR_XL, FS_XL & LPF2_XL_EN
	 */
	new_register_val = (ODR_XL << 4) | (FS_XL << 2) | (LPF2_XL_EN << 1);
	I2C1_Write(I2C_ADDR, CTRL1_XL, 1, &new_register_val);

	I2C1_Read(I2C_ADDR, CTRL1_XL, 1, &read_register_val);

	if(new_register_val == read_register_val){
		printf("Write successful\n\r");
	}




	/*set ranges of both sensors*/
	/*set data rate of both sensors
	 * Set ODR_G to 0100 and G_HM_MODE to 0
	 * - Give 104 Hz sample rate in High performance mode
	 * Set ODR_XL to 0100 and XL_HM_MODE to 0 and LPF2_XL_EN to 0
	 * - Give 104 Hz sample rate in High performance mode
	 * See tables 8 and 9 in the application note.*/

	/*set accelerometer filtering
	 * HP_SLOPE_XL_EN = 0 to just have the LPF1 Digital LP filter enabled
	 * LPF2_XL_EN = 0  to just have the LPF1 Digital LP filter enabled
	 * Set Table 10 in the application note
	 */

	/*set Gyro filtering
	 * High Pass filter (to limit drift)
	 * HP_EN_G = 1
	 * HPM_G[1:0] = 10 -> 3 seconds settling time (see talbe 13 of app note)
	 * LPF1_SEL_G = 0 to disable LPF1 - leave just LPF2 enabled based on ODR_G
	 *
	 */
	/*set butter/fifo to just return latest value*/

}
