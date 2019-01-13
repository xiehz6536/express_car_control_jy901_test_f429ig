#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "i2c.h"

#define IMU_ADDRESS 0xA0

void BSP_I2C_ShortToChar(short sData,unsigned char cData[]);
short BSP_I2C_CharToShort(unsigned char cData[]);

#endif
