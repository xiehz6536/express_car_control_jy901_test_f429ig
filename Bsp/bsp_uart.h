#ifndef BSP_UART_H
#define BSP_UART_H


#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "bsp_serial_print.h"
#include "bsp_i2c.h"
#include "bsp_moto_drive.h"
#include "bsp_control.h"
#include "bsp_move.h"

typedef union
{
	uint8_t date_u8[4];
	int date_int;
}int_u8_union;

typedef union
{
	uint8_t data_u8[4];
	float data_float;
}float_u8_union;

typedef union
{
	uint8_t data_u8[8];
	double data_double;
}double_u8_union;

typedef struct
{
	uint8_t head;
	float_u8_union linear;
	float_u8_union angular;
	
}tx_message;

void BSP_UART_Transmit(tx_message* tx_msg);

#endif
