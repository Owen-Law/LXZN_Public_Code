#ifndef __MANAGE__H
#define __MANAGE__H

#include "main.h"

typedef enum
{
	READ_SN = 0x10,////��SN��Ϣ
	WRITE_SN = 0x11,///дSN��Ϣ
}manage_type_e;


typedef struct{
	uint8_t SYS_NUM;		///2  	=0x09 �豸������Ϣ	=0xA9 ��ʾ�豸Ӧ������
	manage_type_e DEV_CMD;		///3		=0x10����SN��Ϣ��byte4-byte13��0��
									//=0x11��дSN��Ϣ
	uint8_t SN[10];			///4-13	10�ֽ��豸SN����λ��ǰ�����ַ�������
	uint8_t NO_USE[17];		///14-30	����
}device_manage_t;

typedef enum
{
	DEV_MANAGE = 0x09,////0x09 �豸������Ϣ	
	DEV_MANAGE_ACK = 0xA9,///0xA9��ʾ�豸Ӧ������
}sys_num_type_e;



void Receive_Manage_Info(uint8_t * dat);
void Manage_Info_Ack(uint8_t mode, device_para_state_t dev);


#endif

