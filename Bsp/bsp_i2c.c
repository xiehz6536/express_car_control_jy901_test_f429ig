#include "bsp_i2c.h"

void BSP_I2C_ShortToChar(short sData,unsigned char cData[])
{
	cData[0]=sData&0xff;
	cData[1]=sData>>8;
}


short BSP_I2C_CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}
