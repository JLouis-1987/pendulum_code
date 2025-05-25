/*
 * lsm6dso.h
 *
 *  Created on: May 25, 2025
 *      Author: gaspr
 */

#ifndef LSM6DSO_H_
#define LSM6DSO_H_

#include <stdbool.h>
#include <stdint.h>
#include "i2c.h"

bool check_IMU(void);
void init_lsm6dso(void);

#endif /* LSM6DSO_H_ */
