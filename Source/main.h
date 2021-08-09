#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x_conf.h"
#include "stm32f10x.h"

#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include <stdarg.h>

//#define USART_TEST	///串口调试模式，
#define MOTOR_TEST	////电机调试
#define PRINTF_DEBUG//
#define CWS			///社区分类站
#define USE_TCP232		//使用tcp转串口模块
#define USE_NEWMOTOR	///使用新电机

//#define USE_TEST_BKSTG		///使用测试后台
//#define USE_BTN_OPEN	////使用按键开盖


#ifdef PRINTF_DEBUG
	#define d_debug(...) printf(__VA_ARGS__) 
#else
	#define d_debug(...)
#endif


#define BIT(b)           (1<<b)
#define SET(a,b)         a|=(1<<b)
#define CLR(a,b)         a&=~(1<<b)
#define CPL(a,b)         a^=(1<<b)
#define CHK(a,b)         (a&(1<<b))


#define MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(b))  | ((uint16_t)((uint8_t)(a))) << 8))//a=高位
#define MAKELONG(a, b)  ((uint32_t)(((uint16_t)(b)) | ((uint32_t)((uint16_t)(a))) << 16))

#define LOWORD(l)       ((uint16_t)(l))
#define HIWORD(l)       ((uint16_t)(((uint32_t)(l) >> 16) & 0xFFFF))

#define LOBYTE(x)       ((uint8_t)(x & 0x00FF))
#define HIBYTE(x)       ((uint8_t)((x & 0xFF00) >>8))

#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))

#define HTONS(n) 		(uint16_t)((((uint16_t) (n)) << 8) | (((uint16_t) (n)) >> 8))//HTONS是将高8位与低8位调换

#define NOP()	__nop()
#define TIMERFUNC_0S5		50
#define MOTOR_NUM_MAX		4
#define MOTOR_WORK_TIME_MAX	200	//电机运行时间,暂时=5s
#define DOOR_KEEP_OPEN_TIME	400
#define USART1__TIMEOUT		5			///50ms超时
#define USART2__TIMEOUT		5			///50ms超时
#define USART3__TIMEOUT		5			///50ms超时

#define TIMERFUNC_1S		100
#define REPORT_TIME_MAX		30			///30s
#define HEAD0	0x23
#define HEAD1	0x5F
#define TCP_RECHK_TIME		1000		///1s
#define TCP_SEND_LOGINFO_DELY	1000	///1s

#define FLASH_ADDR		0x08009000 //编号地址该地址不能小于0x08004000具体原因未知
#define INITLEN			64

#define HARMFUL_OVER_LINE	1600//2000
#define OTHER_OVER_LINE		800//1000


#define DEV_TYPE	0x0A
#define DEV_REPORT	0x01
#define DEV_ACK		0x0E
#define APP_ACK		0x0F

#define DEV_CODE	0xAB	//系统编码0xAB 表示公共垃圾分类站，0xBC 表示垃圾分类服务端
#define SEV_CODE	0xBC	//系统服务器编码

#define AD_OVER_HARMFUL		ADC_Channel_10	//有害
#define AD_OVER_KITCHEN		ADC_Channel_11	//餐厨
#define AD_OVER_RECYCLE		ADC_Channel_12	//可回收
#define AD_OVER_OTHER		ADC_Channel_13	//其他


typedef union {
    uint32_t    x;
    float       f;
    uint8_t     b[4];
    uint16_t    w[2];
} i2b_def;


typedef enum
{
	V_Harmful_waste = 1,
	V_Kitchen_waste,
	V_Recycle_waste,
	V_Other_waste,
	V_Scan_code_sucess,///5
	V_Scan_Qrcode_fail,///扫码失败专属二维码是否激活
	V_Welcome,
	V_Closeing_Be_careful,
	V_Error_beep,
	V_Door_open,///10
	V_Power_down,
	V_Fire_warning,
	V_Full_harmful,
	V_Full_kitchen,
	V_Full_recycle,///15
	V_Full_other,
	V_Door_close,
	V_Scan_code_fail_try_agin,
	V_Qrcode_invalid,
	V_Qrcode_used,////20
	V_Music_A,
	V_Error_tipsA,
	V_Error_tipsB,
	V_Error_tipsC,
	V_TipsA,///25
	V_TipsB,
	V_Err_time,////27
}voice_pormpt_e;

#ifndef USE_TCP232

#else
typedef enum
{
	//dev_logon = 0x00,////设备注册
	//dev_report = 0x03,////垃圾箱满溢信息上报
	dev_opendoor = 0x04,////开投递门
	dev_hwinfo = 0x05,///垃圾站硬件设置信息下行指令
	dev_keepopendoor = 0x06,////四桶常开常关，业务数据1位，1常关 2常开
	dev_new_connect = 0xff,///新连接
}cmd_type_e;

typedef enum
{
	send_logon = 0x00,////设备注册
	send_report = 0x03,////硬件状态值按配置上报
	//send_qr_code = 0x04,////发送二维码数据
	no_data = 0xff,///不发送
}send_info_mode_e;


#endif



typedef enum
{
	Motor_harmful = 0,
	Motor_kitchen,
	Motor_recycle,
	Motor_other,
}door_motor_e;

typedef enum
{
	Harmful_Waste = 3,
	Kitchen_Waste = 1,
	Recycle_Waste = 2,
	Other_Waste = 4,
	Undefined_Waste = 0,
}door_waste_e;


typedef enum {CLOSE = 0, OPEN = !CLOSE} motor_dir;
typedef enum {STOP = 0, RUN = !STOP} motor_state;


typedef struct{
	uint8_t DEV_ID[4];
	uint8_t OPENDOOR_TIME;
	uint8_t OVERFLOW_WRNING;
	uint16_t OVERFLOW_VAL[MOTOR_NUM_MAX];//满溢值
	uint8_t PWRDN_DOOR_STATE;
	uint16_t NETPORT;
	uint8_t  IPADDR[4];	
	uint8_t DEV_SN[11];
	uint8_t LONGITUDE[13];///经度比如11400.0518,E(E or W)
	uint8_t LATITUDE[12];///纬度比如2235.9058,N(N or S)
	
	
}device_para_state_t;//设备参数与状态
extern device_para_state_t device_para_state;

#define FILTER_N	10
typedef struct{
	uint16_t data_in;
	uint32_t data_sum;
	uint8_t filter_cnt;
	uint16_t data_out;
}filter_data_t;
extern filter_data_t overflow_value[4];

typedef struct{
	motor_state bMotor_operation;
	uint16_t Operation_time;//运行时间
	uint16_t Operation_count;///运行时间计数器
	motor_dir Direction;///方向
	uint16_t Door_Keep_Open_Time;///持续打开时间
	bool bDoorKeepOpen;
}motor_operation_t;


typedef struct {
	bool bKeyScan;
	bool bTimerFunc;
	uint16_t timer_function_count;
	uint8_t test1,test2;
	volatile uint16_t ADCConvertedValue[10][4];//用来存放ADC转换结果，也是DMA的目标地址,4通道，每通道采集10次后面取平均数
	//float ADC_Value[4];
	uint8_t rx1_have_data;
	uint16_t rx1_count;
	uint8_t rx_buffer[128];
	uint8_t tx_buffer[128];
	uint8_t rx1_timeout;
	motor_dir test_motor_dir;
	uint16_t device_report_count;
	bool bReportData;
	uint16_t ADC_Value[MOTOR_NUM_MAX];

	bool bUsrInitOK;
	bool bChkTcp;
	bool bTcpConnectOK;///tcp连接状态
	uint16_t tcp_chk_connect_delay_cnt;
	uint16_t tcp_send_logoninfo_cnt;
	//bool bSendLogonInfo;
	uint8_t SendInfoMode;
	uint8_t Undefined_Waste_cnt;
	uint8_t Undefined_Waste_Time_cnt;

	uint8_t rx2_buffer[64];///接收管理信息指令
	uint8_t rx2_have_data;
	uint16_t rx2_count; 
	uint8_t rx2_timeout;
	uint8_t SendManageMode;

	uint8_t rx3_buffer[64];///接收管理信息指令
	uint8_t rx3_have_data;
	uint16_t rx3_count; 
	uint8_t rx3_timeout;
	//uint8_t SendManageMode;

	bool bBtnScan;
	bool bBtnOk;
	uint8_t BtnNo;

	uint8_t wellcom_voice_delay_cnt;
	bool bMp3_Busy;//
	uint16_t Mp3_delay_cnt;

}global_variable_t;
extern global_variable_t gb_var;

typedef enum{
	C_door_close = 1,
	C_door_keep_open,
}door_keep_e;

//#ifndef USE_TCP232
#if 0
typedef struct{
	uint8_t SYS_NUM;		///2  	0x0A 表示公共垃圾分类站，0x0B 表示垃圾分类服务端，0x0C 表示售货机，
	uint8_t DEV_ID[4];		///3-6	设备ID，高位在前
	cmd_type_e CMD_TYPE;	///7  	=0x10：设备参数设置
							//=0x07：垃圾分类站门锁控制（包括清运门锁，投递门锁）
	uint8_t DATA[23];		///8....	
}device_type_t;

typedef struct{
	uint8_t OPENDOOR_TIME;		///8		设置投递门打开状态持续时间，单位=秒，最大时间4min
	uint8_t PWRDN_DOOR_STATE;	///9		断电状态投递门状态（0断电常开，1断电常关）
	uint8_t OVERFLOW_WRNING;	///10	满溢预警值，比如：=80则设置80%为预警值
}device_setpara_t;

typedef struct{
	uint8_t COUNTER_DOOR;	///8  	货柜机门锁，每2个bit位代表一个门锁，=00 代表对应门关闭，=01代表对应门打开，=11代表对应门状态不改变，
	uint8_t CLEAR_DOOR;		///9		清运门锁，
							//Bit0-bit1：有害垃圾
							//Bit2-bit3：餐厨垃圾
							//Bit4-bit5：可回收垃圾
							//Bit6-bit7：其他垃圾
							//数据含义同上
	uint8_t DROP_DOOR;		//投递门锁，
							//Bit0-bit1：有害垃圾
							//Bit2-bit3：餐厨垃圾
							//Bit4-bit5：可回收垃圾
							//Bit6-bit7：其他垃圾
							//数据含义同上
}device_door_ctrl_t;


#else
typedef struct{
	uint8_t SYS_NUM;		///0  	系统编号 0xAB 表示公共垃圾分类站，0xBC 表示垃圾分类服务端
	cmd_type_e CMD_TYPE;	///1		指令标识=0 设备注册,=3垃圾箱满溢信息上报,=4开投递门,=5 垃圾站硬件设置信息下行指令
	uint8_t D_LEN[4];			///2-5     业务数据长度
	uint8_t DATA[64];
}device_type_t;

typedef struct{
	uint8_t OPENDOOR_TIME;		///6		设置投递门打开状态持续时间，单位=秒，最大时间4min
	uint8_t PWRDN_DOOR_STATE;	///7		断电状态投递门状态（0断电常开，1断电常关）
	uint8_t OVERFLOW_WRNING;	///8		满溢预警值，比如：=80则设置80%为预警值
	uint8_t crc_val[4];			///9-12    crc校验
}device_setpara_t;

typedef struct{
	uint8_t DOOR_CTRL;			///6		开投递门，业务数据1位，1厨余垃圾袋 2可回收垃圾袋 3有害垃圾袋 4其他垃圾        
	uint8_t crc_val[4];			///7-10    crc校验
}device_door_ctrl_t;

typedef struct{
	uint8_t NEW_CONNECT;			///6		=0xff 新建立连接，后台告诉硬件需要重新注册,其他数据无效
	uint8_t crc_val[4];			///7-10    crc校验
}device_new_connect_t;

typedef struct{
	uint8_t DOOR_STATE;			///6		1常关 2常开
	uint8_t crc_val[4];			///7-10    crc校验
}device_keep_open_t;


typedef struct{
	uint8_t SN[10];
	uint8_t FLG;
	uint8_t KITCHEN_STATE;	///6		厨余类满溢值百分比，数据含义同上
	uint8_t RECYCLE_STATE;	///7		可回收类满溢值百分比，数据含义同上
	uint8_t HARMFUL_STATE;	///8		有害类满溢值百分比，比如=80，则是当前已满80%
	uint8_t OTHER_STATE;	///9		其他类满溢值百分比，数据含义同上
	//uint8_t crc_val[4];		///10-13    crc校验
}device_report_t;


#endif








typedef struct{
	uint8_t SYS_NUM;		///2  	0x0A 表示公共垃圾分类站，0x0B 表示垃圾分类服务端，0x0C 表示售货机，
	uint8_t DEV_ID[4];		///3-6	设备ID，高位在前
	uint8_t DEV_CMD;		///7		=0x10：设备参数设置
								  //=0x07：垃圾分类站门锁控制（包括清运门锁，投递门锁）
								  //=0x01：设备状态上报
	uint8_t ACK;			///8 	=1，硬件收到数据
}device_send_ack_t;




////引脚定义////

///测试脚定义////
#define TP1_Pin		GPIO_Pin_4
#define TP1_Port	GPIOC

#define TP2_Pin		GPIO_Pin_5
#define TP2_Port	GPIOC

#define TP3_Pin		GPIO_Pin_6
#define TP3_Port	GPIOC

#define TP4_Pin		GPIO_Pin_7
#define TP4_Port	GPIOC

///电机控制引脚定义////
#define M1_DIR_Pin	GPIO_Pin_1
#define M1_DIR_Port	GPIOA
#define M1_DIR_UP	GPIO_SetBits(M1_DIR_Port, M1_DIR_Pin)
#define M1_DIR_DN	GPIO_ResetBits(M1_DIR_Port, M1_DIR_Pin)


#define M1_STP_Pin	GPIO_Pin_0
#define M1_STP_Port	GPIOA
#define M1_STP_UP	GPIO_SetBits(M1_STP_Port, M1_STP_Pin)
#define M1_STP_DN	GPIO_ResetBits(M1_STP_Port, M1_STP_Pin)

#define M1_EN_Pin	GPIO_Pin_8
#define M1_EN_Port	GPIOA
#define M1_EN_UP	GPIO_SetBits(M1_EN_Port, M1_EN_Pin)
#define M1_EN_DN	GPIO_ResetBits(M1_EN_Port, M1_EN_Pin)
#define M1_EN(x)	(((x) == RUN) ? M1_EN_UP : M1_EN_DN)

#define M2_DIR_Pin	GPIO_Pin_12
#define M2_DIR_Port	GPIOA
#define M2_DIR_UP	GPIO_SetBits(M2_DIR_Port, M2_DIR_Pin)
#define M2_DIR_DN	GPIO_ResetBits(M2_DIR_Port, M2_DIR_Pin)

#define M2_STP_Pin	GPIO_Pin_11
#define M2_STP_Port	GPIOA
#define M2_STP_UP	GPIO_SetBits(M2_STP_Port, M2_STP_Pin)
#define M2_STP_DN	GPIO_ResetBits(M2_STP_Port, M2_STP_Pin)

#define M2_EN_Pin	GPIO_Pin_2
#define M2_EN_Port	GPIOD
#define M2_EN_UP	GPIO_SetBits(M2_EN_Port, M2_EN_Pin)
#define M2_EN_DN	GPIO_ResetBits(M2_EN_Port, M2_EN_Pin)
#define M2_EN(x)	(((x) == RUN) ? M2_EN_UP : M2_EN_DN)


#define M3_DIR_Pin	GPIO_Pin_3
#define M3_DIR_Port	GPIOB
#define M3_DIR_UP	GPIO_SetBits(M3_DIR_Port, M3_DIR_Pin)
#define M3_DIR_DN	GPIO_ResetBits(M3_DIR_Port, M3_DIR_Pin)

#define M3_STP_Pin	GPIO_Pin_2
#define M3_STP_Port	GPIOB
#define M3_STP_UP	GPIO_SetBits(M3_STP_Port, M3_STP_Pin)
#define M3_STP_DN	GPIO_ResetBits(M3_STP_Port, M3_STP_Pin)

#define M3_EN_Pin	GPIO_Pin_4
#define M3_EN_Port	GPIOB
#define M3_EN_UP	GPIO_SetBits(M3_EN_Port, M3_EN_Pin)
#define M3_EN_DN	GPIO_ResetBits(M3_EN_Port, M3_EN_Pin)
#define M3_EN(x)	(((x) == RUN) ? M3_EN_UP : M3_EN_DN)


#define M4_DIR_Pin	GPIO_Pin_6
#define M4_DIR_Port	GPIOB
#define M4_DIR_UP	GPIO_SetBits(M4_DIR_Port, M4_DIR_Pin)
#define M4_DIR_DN	GPIO_ResetBits(M4_DIR_Port, M4_DIR_Pin)

#define M4_STP_Pin	GPIO_Pin_5
#define M4_STP_Port	GPIOB
#define M4_STP_UP	GPIO_SetBits(M4_STP_Port, M4_STP_Pin)
#define M4_STP_DN	GPIO_ResetBits(M4_STP_Port, M4_STP_Pin)

#define M4_EN_Pin	GPIO_Pin_7
#define M4_EN_Port	GPIOB
#define M4_EN_UP	GPIO_SetBits(M4_EN_Port, M4_EN_Pin)
#define M4_EN_DN	GPIO_ResetBits(M4_EN_Port, M4_EN_Pin)
#define M4_EN(x)	(((x) == RUN) ? M4_EN_UP : M4_EN_DN)


////led灯引脚定义////
#define L_TCP_Pin	GPIO_Pin_8
#define L_TCP_Port	GPIOB

#define L_TCP_ON	GPIO_ResetBits(L_TCP_Port, L_TCP_Pin)
#define L_TCP_OFF	GPIO_SetBits(L_TCP_Port, L_TCP_Pin)

#ifndef CWS
#define L_SYS_Pin	GPIO_Pin_9
#define L_SYS_Port	GPIOB

#define L_SYS_ON	GPIO_ResetBits(L_SYS_Port, L_SYS_Pin)
#define L_SYS_OFF	GPIO_SetBits(L_SYS_Port, L_SYS_Pin)
#endif

///按键引脚定义////
#define KEY_Pin		GPIO_Pin_12
#define KEY_Port	GPIOB

#define RDKEY()		GPIO_ReadInputDataBit(KEY_Port,KEY_Pin)


///红外测距引脚定义////
#define DM1_Pin		GPIO_Pin_0		///adc10
#define DM1_Port	GPIOC

#define DM2_Pin		GPIO_Pin_1		//adc11
#define DM2_Port	GPIOC

#define DM3_Pin		GPIO_Pin_2		//adc12
#define DM3_Port	GPIOC

#define DM4_Pin		GPIO_Pin_3		//adc13
#define DM4_Port	GPIOC


///限位开关定义////
#define M1_LIMIT_A_Pin	GPIO_Pin_8		//开盖限位
#define M1_LIMIT_A_Port	GPIOC

#define M1_LIMIT_B_Pin	GPIO_Pin_9		//关盖限位
#define M1_LIMIT_B_Port	GPIOC

#define M2_LIMIT_A_Pin	GPIO_Pin_10		//开盖限位
#define M2_LIMIT_A_Port	GPIOC

#define M2_LIMIT_B_Pin	GPIO_Pin_11		//关盖限位
#define M2_LIMIT_B_Port	GPIOC

#define M3_LIMIT_A_Pin	GPIO_Pin_12		//开盖限位
#define M3_LIMIT_A_Port	GPIOC

#define M3_LIMIT_B_Pin	GPIO_Pin_13		//关盖限位
#define M3_LIMIT_B_Port	GPIOC

#define M4_LIMIT_A_Pin	GPIO_Pin_14		//开盖限位
#define M4_LIMIT_A_Port	GPIOC

#define M4_LIMIT_B_Pin	GPIO_Pin_15		//关盖限位
#define M4_LIMIT_B_Port	GPIOC

#define Get_M1_Open_Limit()		GPIO_ReadInputDataBit(M1_LIMIT_A_Port,M1_LIMIT_A_Pin)//获取m1开盖限位
#define Get_M1_Close_Limit()	GPIO_ReadInputDataBit(M1_LIMIT_B_Port,M1_LIMIT_B_Pin)//获取m1关盖限位

#define Get_M2_Open_Limit()		GPIO_ReadInputDataBit(M2_LIMIT_A_Port,M2_LIMIT_A_Pin)//获取m1开盖限位
#define Get_M2_Close_Limit()	GPIO_ReadInputDataBit(M2_LIMIT_B_Port,M2_LIMIT_B_Pin)//获取m1关盖限位

#define Get_M3_Open_Limit()		GPIO_ReadInputDataBit(M3_LIMIT_A_Port,M3_LIMIT_A_Pin)//获取m1开盖限位
#define Get_M3_Close_Limit()	GPIO_ReadInputDataBit(M3_LIMIT_B_Port,M3_LIMIT_B_Pin)//获取m1关盖限位

#define Get_M4_Open_Limit()		GPIO_ReadInputDataBit(M4_LIMIT_A_Port,M4_LIMIT_A_Pin)//获取m1开盖限位
#define Get_M4_Close_Limit()	GPIO_ReadInputDataBit(M4_LIMIT_B_Port,M4_LIMIT_B_Pin)//获取m1关盖限位


///USR_TCP232_T2模块引脚定义
#define USR_USART	USART1

#ifndef CWS
#define USR_REST_Pin	GPIO_Pin_13
#define USR_REST_PORT	GPIOB
#define USR_REST_UP		GPIO_SetBits(USR_REST_PORT, USR_REST_Pin)
#define USR_REST_DN		GPIO_ResetBits(USR_REST_PORT, USR_REST_Pin)

#define USR_CFG_Pin		GPIO_Pin_14
#define USR_CFG_PORT	GPIOB
#define USR_CFG_UP		GPIO_SetBits(USR_CFG_PORT, USR_CFG_Pin)
#define USR_CFG_DN		GPIO_ResetBits(USR_CFG_PORT, USR_CFG_Pin)
#else
#define USR_REST_Pin	GPIO_Pin_0
#define USR_REST_PORT	GPIOB
#define USR_REST_UP		GPIO_SetBits(USR_REST_PORT, USR_REST_Pin)
#define USR_REST_DN		GPIO_ResetBits(USR_REST_PORT, USR_REST_Pin)

#define USR_CFG_Pin		GPIO_Pin_1
#define USR_CFG_PORT	GPIOB
#define USR_CFG_UP		GPIO_SetBits(USR_CFG_PORT, USR_CFG_Pin)
#define USR_CFG_DN		GPIO_ResetBits(USR_CFG_PORT, USR_CFG_Pin)

#endif

////按钮开盖引脚定义
#define BTN1_Pin			GPIO_Pin_4		///有害
#define BTN1_Port			GPIOA

#define BTN2_Pin			GPIO_Pin_5		///kitchen
#define BTN2_Port			GPIOA

#define BTN3_Pin			GPIO_Pin_6		///recycle
#define BTN3_Port			GPIOA

#define BTN4_Pin			GPIO_Pin_7		////other
#define BTN4_Port			GPIOA



void Receive_Net(uint8_t * dat);
void Motor_Operation(door_motor_e motor_x, motor_dir dir_x);
uint8_t MakeChk(uint8_t * dat, uint8_t len);
uint16_t CRC16_MODBUS(char *buffer, int len);
void SendTest(void);
void SendInfo(uint8_t mode, device_para_state_t dps);
void WriteIni(const device_para_state_t *dev);
void ReadIni(device_para_state_t *dev);
void mp3_Play(voice_pormpt_e tips);


#endif
