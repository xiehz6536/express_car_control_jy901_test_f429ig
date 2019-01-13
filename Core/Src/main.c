
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
#include "bsp_uart.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint8_t imu_data[24];          //记录imu元数据
uint8_t raw_height[2];
float acc[3];                  //imu加速度
float omiga[3];                //imu角速度
float magnetic[3];              
float angle[3];                //imu角度
float height;                  //imu高度
float omiga_z;                 //z轴角速度
int_u8_union rx_spd;           //串口接受的数据,控制电机转速
//double_u8_union tx_linear,tx_angular;
tx_message tx_msg;
uint8_t rx_ctrl_msg[4];

extern moto_pid bsp_pid[2];     //pid结构体
int set_spd[2]={0, 0};//设定速度 13*51表示 1 r/s
double last_encoder[2]={0,0};    //记录上次编码器数值，滤波
double now_encoder[2];					 //记录当前编码器数值，滤波
double current_spd[2];           //
double last_spd[2];              //
double speed_lpf[2];             //滤波后的速度


float lpf_index = 0.18;          //越小，越平滑，但是相应越man  lpf=0.25
double move_target_spd=300;

uint8_t msg[4]={0x00,0x01,0x02,0x03};
uint8_t cnt=0;                   //控制频率  T=cnt*1ms
uint8_t control_period=6;
uint8_t set_pwm_period=1;
uint8_t init_flags=1;
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);//电机1的编码器
	HAL_Delay(5);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//电机2的编码器
	HAL_TIM_Base_Start_IT(&htim4);//定时器中断
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//PWM输出
	//PID初始�?
	BSP_Pid_Init(&bsp_pid[0],pid_id_1,5.0,1,1.1);
	BSP_Pid_Init(&bsp_pid[1],pid_id_2,5.0,1,1.1);
	//p=0.3,i=0.08 at7:30
	
	
	HAL_Delay(10);
	
	//MOVE_FORWARD;
	

	//printf("working...\n");
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&rx_ctrl_msg[0],4);
	//rx_spd.spd_int=-800;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//printf("%.2lf,%.2lf,%.2lf,%.2lf,%.2f\n",current_spd[moto_ID_2],speed_lpf[moto_ID_2],speed_lpf[moto_ID_2] - bsp_pid[moto_ID_2].target,bsp_pid[moto_ID_2].target,bsp_pid[moto_ID_2].out);
		//printf("%.2f,%.2f,%.2f,%d.\n",speed_lpf[moto_ID_1],speed_lpf[moto_ID_2],bsp_pid[0].target,0);
		//HAL_UART_Transmit(&huart1,(uint8_t *)&rx_spd.spd_uch[0],sizeof(rx_spd),50);
//		test OK
//		printf("start...\n");
//		HAL_I2C_Mem_Read(&hi2c1,IMU_ADDRESS,0x34,I2C_MEMADD_SIZE_8BIT,imu_data,24,50);
//		//printf("roll:%d pitch:%d yaw:%d.\n",imu_data[6],imu_data[8],imu_data[10]);
//		
//		acc[0]=(float)BSP_I2C_CharToShort(&imu_data[0])/32678*16*9.8;
//		acc[1]=(float)BSP_I2C_CharToShort(&imu_data[2])/32678*16*9.8;
//		acc[2]=(float)BSP_I2C_CharToShort(&imu_data[4])/32678*16*9.8;
//		printf("acc_x: %f    acc_y: %f    acc_z:%f.\n",acc[0],acc[1],acc[2]);
//		
//		omiga[0]=(float)BSP_I2C_CharToShort(&imu_data[6])/32678*2000;
//		omiga[1]=(float)BSP_I2C_CharToShort(&imu_data[8])/32678*2000;
//		omiga[2]=(float)BSP_I2C_CharToShort(&imu_data[10])/32678*2000;
//		printf("omiga_x: %f    omiga_y: %f    omiga_z:%f.\n",omiga[0],omiga[1],omiga[2]);
//		
//		magnetic[0]=(float)BSP_I2C_CharToShort(&imu_data[12]);
//		magnetic[1]=(float)BSP_I2C_CharToShort(&imu_data[14]);
//		magnetic[2]=(float)BSP_I2C_CharToShort(&imu_data[16]);
//		printf("magnetic_x: %f    magnetic_y: %f    magnetic_z:%f.\n",magnetic[0],magnetic[1],magnetic[2]);
//		
//		angle[0] = (float)BSP_I2C_CharToShort(&imu_data[18])/32768*180;   
//	  angle[1] = (float)BSP_I2C_CharToShort(&imu_data[20])/32768*180;
//    angle[2] = (float)BSP_I2C_CharToShort(&imu_data[22])/32768*180;
//		printf("roll: %f    pitch: %f    yaw: %f.\n",angle[0],angle[1],angle[2]);
//		
//		printf("end...\n");
//		HAL_Delay(500);


//		Let us try to plot a heart

//    for(float i=-2;i<2;i+=0.01)
//    {
//        float y1=sqrt(2*sqrt(i*i)-i*i);
//        float y2=asin(sqrt(i*i)-1)-3.1416/2;
//        float y3=sqrt(sqrt(i*i))-sqrt(cos(i))*cos(60*i);
//        if(i>1.57||i<-1.57)
//        {
//            y3=1.2;
//        }
//        printf("%f\n",y3);
//				//printf("%f,%f\n",y1,y2);
//        HAL_Delay(5);
//    }

//		get height...test error
//		HAL_I2C_Mem_Read(&hi2c1,IMU_ADDRESS,0x47,I2C_MEMADD_SIZE_8BIT,raw_height,2,50);
//		height=(float)BSP_I2C_CharToShort(&raw_height[0]);
//		printf("%f.\n",height);
//		HAL_Delay(500);

		//current_spd[0]=BSP_Moto_GetSpeed(&htim1);
		//BSP_PWM_SetValue(&htim9,TIM_CHANNEL_1,1000*3-1);
		//printf("OK.\n");
		//HAL_Delay(500);
		
		//HAL_UART_Receive_IT(&huart1,(uint8_t *)&rx_ctrl_msg[0],sizeof(rx_ctrl_msg));
		
		HAL_I2C_Mem_Read(&hi2c1,IMU_ADDRESS,0x34,I2C_MEMADD_SIZE_8BIT,imu_data,24,50);
		omiga_z=(float)BSP_I2C_CharToShort(&imu_data[10])/32678*2000/180*3.142;
//		HAL_Delay(10);
//		angle[0] = (float)BSP_I2C_CharToShort(&imu_data[18])/32768*180;   
//		angle[1] = (float)BSP_I2C_CharToShort(&imu_data[20])/32768*180;
//		angle[2] = (float)BSP_I2C_CharToShort(&imu_data[22])/32768*180;
//		printf("roll: %.2f    pitch: %.2f    yaw: %.2f.\n",angle[0],angle[1],angle[2]);
		
		
		tx_msg.head=0x01;
		tx_msg.angular.data_float=omiga_z;
		tx_msg.linear.data_float=(float)(current_spd[moto_ID_1]+current_spd[moto_ID_2])/13.0/51.0/2.0*125*3.142/180;
		BSP_UART_Transmit(&tx_msg);
		//printf("angular:%.2f,  linear:%.2f.\n",tx_msg.angular.data_float,tx_msg.linear.data_float);
		HAL_Delay(100);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		
		BSP_Pid_SetTarget(&bsp_pid[0],set_spd[0]);
		BSP_Pid_SetTarget(&bsp_pid[1],set_spd[1]);
		//HAL_UART_Transmit(&huart1,&msg[2],2,50);
		
		if(cnt==control_period)
		{
			
			/****************************/
			//当前编码器数值的变化
			now_encoder[moto_ID_1] = (BSP_Moto_GetValue(moto_ID_1)-last_encoder[moto_ID_1])*(100 / cnt);
			now_encoder[moto_ID_2] = (BSP_Moto_GetValue(moto_ID_2)-last_encoder[moto_ID_2])*(100 / cnt);
			//滤去 越过65536时编码器的数据
			if(now_encoder[moto_ID_1] > -20000 && now_encoder[moto_ID_1] < 20000) 
					current_spd[moto_ID_1]=now_encoder[moto_ID_1];
			if(now_encoder[moto_ID_2] > -20000 && now_encoder[moto_ID_2] < 20000) 
					current_spd[moto_ID_2]=now_encoder[moto_ID_2];
	
			//编码器速度滤波
			speed_lpf[moto_ID_1] = lpf_index * current_spd[moto_ID_1] + (1-lpf_index) * last_spd[moto_ID_1];
			speed_lpf[moto_ID_2] = lpf_index * current_spd[moto_ID_2] + (1-lpf_index) * last_spd[moto_ID_2];
			last_spd[moto_ID_1] =  speed_lpf[moto_ID_1];
			last_spd[moto_ID_2] =  speed_lpf[moto_ID_2];
			//记录 当前编码器数据 作为 下次读取时 的 上次数据
			last_encoder[moto_ID_1]=BSP_Moto_GetValue(moto_ID_1);
			last_encoder[moto_ID_2]=BSP_Moto_GetValue(moto_ID_2);
			
			//将滤波后的速度传入pid
			BSP_Pid_SetCurrent(&bsp_pid[moto_ID_1],speed_lpf[moto_ID_1]);
			BSP_Pid_SetCurrent(&bsp_pid[moto_ID_2],speed_lpf[moto_ID_2]);
			//pid计算
			BSP_Pid_calculate(&bsp_pid[moto_ID_1]); 
			BSP_Pid_calculate(&bsp_pid[moto_ID_2]);
			//驱动
			//BSP_Moto_Control(40,40);
			//BSP_PWM_SetValue(&htim8,TIM_CHANNEL_1,2000);
			//BSP_Moto_Control(bsp_pid[moto_ID_1].out,bsp_pid[moto_ID_2].out);
			HAL_UART_Receive_IT(&huart1,(uint8_t *)&rx_ctrl_msg[0],4);
			cnt=0;
			/*****************************/
			
			//printf("%d,%d,%d,%d,%d\n",current_spd[0], current_spd[1],bsp_pid[moto_ID_1].out, bsp_pid[moto_ID_2].out,13*51);
//			printf("pid :\n	err:%d,err_last:%d,err_pre:%d.\n	p:%d,i:%d,d:%d.\n",bsp_pid[moto_ID_1].err,bsp_pid[moto_ID_1].last_err,bsp_pid[moto_ID_1].pre_err,
//															bsp_pid[moto_ID_1].proportion,bsp_pid[moto_ID_2].integer,bsp_pid[moto_ID_1].differential);
//			printf("pwm: %d.\n",bsp_pid[moto_ID_1].out);
			//HAL_UART_Transmit(&huart1,msg,2,50);
		}
		
		BSP_Moto_SpeedControlOutput(&bsp_pid[moto_ID_1],cnt);
		BSP_Moto_SpeedControlOutput(&bsp_pid[moto_ID_2],cnt);
		BSP_Moto_Control(bsp_pid[moto_ID_1].out,bsp_pid[moto_ID_2].out);
		
		cnt++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		//printf("%d.\n",rx_spd.spd_int);
		//printf("UART_RECEIVE_IT\n");
		switch(BSP_Compare_Msg((uint8_t *)&rx_ctrl_msg[0]))
		//switch(rx_ctrl_msg[0])
		{
			case 0x00:
				STOP;break;
			case 0x01:
				MOVE_FORWARD(move_target_spd);break;
			case 0x02:
				MOVE_BACK(move_target_spd);break;
			case 0x03:
				TURN_LEFT(move_target_spd);break;
			case 0x04:
				TURN_RIGHT(move_target_spd);break;
		}

		HAL_UART_Receive_IT(&huart1,(uint8_t *)&rx_ctrl_msg[0],4);
		
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
