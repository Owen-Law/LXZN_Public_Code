#include "manage.h"
//#include "main.h"


//23 5F 09 11 32 30 32 31 30 35 31 32 33 34 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0E //2021051234
//23 5F 09 11 32 30 32 31 30 36 30 30 30 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 06 //2021060001

void Receive_Manage_Info(uint8_t * dat)
{
	if((HEAD0 == dat[0])&&(HEAD1 == dat[1]))
		{
		if( dat[31] == MakeChk(dat+2,29))
			{
				device_manage_t *dmt = (device_manage_t *)(dat+2);

				if(DEV_MANAGE == dmt->SYS_NUM)///设备类型
					{
						switch(dmt->DEV_CMD)
							{
							case READ_SN:
								gb_var.SendManageMode = READ_SN;
								break;
							case WRITE_SN:
								memcpy(device_para_state.DEV_SN,dmt->SN,sizeof(dmt->SN));
								device_para_state.DEV_SN[10] = 0;
								WriteIni(&device_para_state);
								gb_var.SendManageMode = WRITE_SN;
								break;
							default:

								break;
							}
						
					}
			}
		}
	gb_var.rx2_have_data = 0;
}

void Manage_Info_Ack(uint8_t mode, device_para_state_t dev)
{
	uint8_t buffer[29];

	memset(buffer, 0, sizeof(buffer));
	device_manage_t * dmt = (device_manage_t *)buffer;
	dmt->SYS_NUM = DEV_MANAGE_ACK;
	dmt->DEV_CMD = mode;
	memcpy(dmt->SN, dev.DEV_SN, sizeof(dmt->SN));
	
	Start_Send_PC( buffer, sizeof(buffer));

	gb_var.SendManageMode = no_data;
}

