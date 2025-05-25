/*
 * i2c.h
 *
 *  Created on: May 24, 2025
 *      Author: gaspr
 */

#ifndef I2C_H_
#define I2C_H_

void I2C1_Init(void);
void I2C1_byteRead(char paddr, char maddr, char* data);
void I2C1_Read(char paddr, char maddr, int n, char* data);
void I2C1_Write(char paddr, char maddr, int n, char* data);

#endif /* I2C_H_ */
