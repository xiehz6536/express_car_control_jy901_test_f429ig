#ifndef BSP_MOVE_H
#define BSP_MOVE_H

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
/* USER CODE END Includes */



#define target_spd 16*51

#define MOVE_FORWARD(i) 			set_spd[0]=i;set_spd[1]=i;
#define MOVE_BACK(i) 					set_spd[0]=-i;set_spd[1]=-i;
#define TURN_LEFT(i) 					set_spd[0]=-i;set_spd[1]=i;
#define TURN_RIGHT(i) 				set_spd[0]=i;set_spd[1]=-i;
#define STOP 									set_spd[0]=0;set_spd[1]=0;

uint8_t BSP_Compare_Msg(uint8_t* rx_msg);

#endif
