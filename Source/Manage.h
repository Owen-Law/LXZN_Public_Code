#ifndef __MANAGE__H
#define __MANAGE__H

#include "main.h"

typedef enum
{
	READ_SN = 0x10,////读SN信息
	WRITE_SN = 0x11,///写SN信息
}manage_type_e;


typedef struct{
	uint8_t SYS_NUM;		///2  	=0x09 设备管理信息	=0xA9 表示设备应答数据
	manage_type_e DEV_CMD;		///3		=0x10：读SN信息（byte4-byte13填0）
									//=0x11：写SN信息
	uint8_t SN[10];			///4-13	10字节设备SN，高位在前，用字符串传输
	uint8_t NO_USE[17];		///14-30	备用
}device_manage_t;

typedef enum
{
	DEV_MANAGE = 0x09,////0x09 设备管理信息	
	DEV_MANAGE_ACK = 0xA9,///0xA9表示设备应答数据
}sys_num_type_e;



void Receive_Manage_Info(uint8_t * dat);
void Manage_Info_Ack(uint8_t mode, device_para_state_t dev);


#endif

