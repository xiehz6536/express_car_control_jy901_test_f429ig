#include "bsp_move.h"

uint8_t compare_msg[5]={0x00,0x01,0x02,0x03,0x04};
uint8_t last_msg=0x00;
uint8_t move_msg[3]={0x00,0x00,0x00};
uint8_t ret_msg=0x00;

/**********************************************
Function name:   BSP_Compare_Msg
Features:        比较串口接收到的消息
Parameter:       rx_msg---待比较消息指针
Return value:    last_msg---比较所得结果
**********************************************/
uint8_t BSP_Compare_Msg(uint8_t* rx_msg)
{
	
	int i=0;
	for(i=0;i<3;i++)
	{
		if(rx_msg[i] != rx_msg[i+1])
		{
			return last_msg;
		}
	}
	for(i=0;i<5;i++)
	{
		if(rx_msg[0] == compare_msg[i])
		{
			last_msg = compare_msg[i];
			return compare_msg[i];
		}
	}
	return last_msg;
	
	
//	//uint8_t ret_msg=0x00;
//	move_msg[2]=rx_msg[0];
//	if(move_msg[1]==move_msg[0] && move_msg[1]==move_msg[2])
//	{
//		ret_msg = move_msg[1];
//	}
//	else if(move_msg[0]==move_msg[2])
//	{
//		move_msg[1]=move_msg[0];
//		ret_msg=move_msg[1];
//	}
//	else
//	{
//		move_msg[2]=move_msg[1]=move_msg[0];
//		ret_msg=move_msg[0];
//	}
//	return ret_msg;
	
}
