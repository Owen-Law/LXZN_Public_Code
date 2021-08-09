#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x_conf.h"
#include "stm32f10x.h"

#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include <stdarg.h>

//#define USART_TEST	///���ڵ���ģʽ��
#define MOTOR_TEST	////�������
#define PRINTF_DEBUG//
#define CWS			///��������վ
#define USE_TCP232		//ʹ��tcpת����ģ��
#define USE_NEWMOTOR	///ʹ���µ��

//#define USE_TEST_BKSTG		///ʹ�ò��Ժ�̨
//#define USE_BTN_OPEN	////ʹ�ð�������


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


#define MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(b))  | ((uint16_t)((uint8_t)(a))) << 8))//a=��λ
#define MAKELONG(a, b)  ((uint32_t)(((uint16_t)(b)) | ((uint32_t)((uint16_t)(a))) << 16))

#define LOWORD(l)       ((uint16_t)(l))
#define HIWORD(l)       ((uint16_t)(((uint32_t)(l) >> 16) & 0xFFFF))

#define LOBYTE(x)       ((uint8_t)(x & 0x00FF))
#define HIBYTE(x)       ((uint8_t)((x & 0xFF00) >>8))

#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))

#define HTONS(n) 		(uint16_t)((((uint16_t) (n)) << 8) | (((uint16_t) (n)) >> 8))//HTONS�ǽ���8λ���8λ����

#define NOP()	__nop()
#define TIMERFUNC_0S5		50
#define MOTOR_NUM_MAX		4
#define MOTOR_WORK_TIME_MAX	200	//�������ʱ��,��ʱ=5s
#define DOOR_KEEP_OPEN_TIME	400
#define USART1__TIMEOUT		5			///50ms��ʱ
#define USART2__TIMEOUT		5			///50ms��ʱ
#define USART3__TIMEOUT		5			///50ms��ʱ

#define TIMERFUNC_1S		100
#define REPORT_TIME_MAX		30			///30s
#define HEAD0	0x23
#define HEAD1	0x5F
#define TCP_RECHK_TIME		1000		///1s
#define TCP_SEND_LOGINFO_DELY	1000	///1s

#define FLASH_ADDR		0x08009000 //��ŵ�ַ�õ�ַ����С��0x08004000����ԭ��δ֪
#define INITLEN			64

#define HARMFUL_OVER_LINE	1600//2000
#define OTHER_OVER_LINE		800//1000


#define DEV_TYPE	0x0A
#define DEV_REPORT	0x01
#define DEV_ACK		0x0E
#define APP_ACK		0x0F

#define DEV_CODE	0xAB	//ϵͳ����0xAB ��ʾ������������վ��0xBC ��ʾ������������
#define SEV_CODE	0xBC	//ϵͳ����������

#define AD_OVER_HARMFUL		ADC_Channel_10	//�к�
#define AD_OVER_KITCHEN		ADC_Channel_11	//�ͳ�
#define AD_OVER_RECYCLE		ADC_Channel_12	//�ɻ���
#define AD_OVER_OTHER		ADC_Channel_13	//����


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
	V_Scan_Qrcode_fail,///ɨ��ʧ��ר����ά���Ƿ񼤻�
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
	//dev_logon = 0x00,////�豸ע��
	//dev_report = 0x03,////������������Ϣ�ϱ�
	dev_opendoor = 0x04,////��Ͷ����
	dev_hwinfo = 0x05,///����վӲ��������Ϣ����ָ��
	dev_keepopendoor = 0x06,////��Ͱ�������أ�ҵ������1λ��1���� 2����
	dev_new_connect = 0xff,///������
}cmd_type_e;

typedef enum
{
	send_logon = 0x00,////�豸ע��
	send_report = 0x03,////Ӳ��״ֵ̬�������ϱ�
	//send_qr_code = 0x04,////���Ͷ�ά������
	no_data = 0xff,///������
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
	uint16_t OVERFLOW_VAL[MOTOR_NUM_MAX];//����ֵ
	uint8_t PWRDN_DOOR_STATE;
	uint16_t NETPORT;
	uint8_t  IPADDR[4];	
	uint8_t DEV_SN[11];
	uint8_t LONGITUDE[13];///���ȱ���11400.0518,E(E or W)
	uint8_t LATITUDE[12];///γ�ȱ���2235.9058,N(N or S)
	
	
}device_para_state_t;//�豸������״̬
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
	uint16_t Operation_time;//����ʱ��
	uint16_t Operation_count;///����ʱ�������
	motor_dir Direction;///����
	uint16_t Door_Keep_Open_Time;///������ʱ��
	bool bDoorKeepOpen;
}motor_operation_t;


typedef struct {
	bool bKeyScan;
	bool bTimerFunc;
	uint16_t timer_function_count;
	uint8_t test1,test2;
	volatile uint16_t ADCConvertedValue[10][4];//�������ADCת�������Ҳ��DMA��Ŀ���ַ,4ͨ����ÿͨ���ɼ�10�κ���ȡƽ����
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
	bool bTcpConnectOK;///tcp����״̬
	uint16_t tcp_chk_connect_delay_cnt;
	uint16_t tcp_send_logoninfo_cnt;
	//bool bSendLogonInfo;
	uint8_t SendInfoMode;
	uint8_t Undefined_Waste_cnt;
	uint8_t Undefined_Waste_Time_cnt;

	uint8_t rx2_buffer[64];///���չ�����Ϣָ��
	uint8_t rx2_have_data;
	uint16_t rx2_count; 
	uint8_t rx2_timeout;
	uint8_t SendManageMode;

	uint8_t rx3_buffer[64];///���չ�����Ϣָ��
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
	uint8_t SYS_NUM;		///2  	0x0A ��ʾ������������վ��0x0B ��ʾ�����������ˣ�0x0C ��ʾ�ۻ�����
	uint8_t DEV_ID[4];		///3-6	�豸ID����λ��ǰ
	cmd_type_e CMD_TYPE;	///7  	=0x10���豸��������
							//=0x07����������վ�������ƣ���������������Ͷ��������
	uint8_t DATA[23];		///8....	
}device_type_t;

typedef struct{
	uint8_t OPENDOOR_TIME;		///8		����Ͷ���Ŵ�״̬����ʱ�䣬��λ=�룬���ʱ��4min
	uint8_t PWRDN_DOOR_STATE;	///9		�ϵ�״̬Ͷ����״̬��0�ϵ糣����1�ϵ糣�أ�
	uint8_t OVERFLOW_WRNING;	///10	����Ԥ��ֵ�����磺=80������80%ΪԤ��ֵ
}device_setpara_t;

typedef struct{
	uint8_t COUNTER_DOOR;	///8  	�����������ÿ2��bitλ����һ��������=00 �����Ӧ�Źرգ�=01�����Ӧ�Ŵ򿪣�=11�����Ӧ��״̬���ı䣬
	uint8_t CLEAR_DOOR;		///9		����������
							//Bit0-bit1���к�����
							//Bit2-bit3���ͳ�����
							//Bit4-bit5���ɻ�������
							//Bit6-bit7����������
							//���ݺ���ͬ��
	uint8_t DROP_DOOR;		//Ͷ��������
							//Bit0-bit1���к�����
							//Bit2-bit3���ͳ�����
							//Bit4-bit5���ɻ�������
							//Bit6-bit7����������
							//���ݺ���ͬ��
}device_door_ctrl_t;


#else
typedef struct{
	uint8_t SYS_NUM;		///0  	ϵͳ��� 0xAB ��ʾ������������վ��0xBC ��ʾ������������
	cmd_type_e CMD_TYPE;	///1		ָ���ʶ=0 �豸ע��,=3������������Ϣ�ϱ�,=4��Ͷ����,=5 ����վӲ��������Ϣ����ָ��
	uint8_t D_LEN[4];			///2-5     ҵ�����ݳ���
	uint8_t DATA[64];
}device_type_t;

typedef struct{
	uint8_t OPENDOOR_TIME;		///6		����Ͷ���Ŵ�״̬����ʱ�䣬��λ=�룬���ʱ��4min
	uint8_t PWRDN_DOOR_STATE;	///7		�ϵ�״̬Ͷ����״̬��0�ϵ糣����1�ϵ糣�أ�
	uint8_t OVERFLOW_WRNING;	///8		����Ԥ��ֵ�����磺=80������80%ΪԤ��ֵ
	uint8_t crc_val[4];			///9-12    crcУ��
}device_setpara_t;

typedef struct{
	uint8_t DOOR_CTRL;			///6		��Ͷ���ţ�ҵ������1λ��1���������� 2�ɻ��������� 3�к������� 4��������        
	uint8_t crc_val[4];			///7-10    crcУ��
}device_door_ctrl_t;

typedef struct{
	uint8_t NEW_CONNECT;			///6		=0xff �½������ӣ���̨����Ӳ����Ҫ����ע��,����������Ч
	uint8_t crc_val[4];			///7-10    crcУ��
}device_new_connect_t;

typedef struct{
	uint8_t DOOR_STATE;			///6		1���� 2����
	uint8_t crc_val[4];			///7-10    crcУ��
}device_keep_open_t;


typedef struct{
	uint8_t SN[10];
	uint8_t FLG;
	uint8_t KITCHEN_STATE;	///6		����������ֵ�ٷֱȣ����ݺ���ͬ��
	uint8_t RECYCLE_STATE;	///7		�ɻ���������ֵ�ٷֱȣ����ݺ���ͬ��
	uint8_t HARMFUL_STATE;	///8		�к�������ֵ�ٷֱȣ�����=80�����ǵ�ǰ����80%
	uint8_t OTHER_STATE;	///9		����������ֵ�ٷֱȣ����ݺ���ͬ��
	//uint8_t crc_val[4];		///10-13    crcУ��
}device_report_t;


#endif








typedef struct{
	uint8_t SYS_NUM;		///2  	0x0A ��ʾ������������վ��0x0B ��ʾ�����������ˣ�0x0C ��ʾ�ۻ�����
	uint8_t DEV_ID[4];		///3-6	�豸ID����λ��ǰ
	uint8_t DEV_CMD;		///7		=0x10���豸��������
								  //=0x07����������վ�������ƣ���������������Ͷ��������
								  //=0x01���豸״̬�ϱ�
	uint8_t ACK;			///8 	=1��Ӳ���յ�����
}device_send_ack_t;




////���Ŷ���////

///���ԽŶ���////
#define TP1_Pin		GPIO_Pin_4
#define TP1_Port	GPIOC

#define TP2_Pin		GPIO_Pin_5
#define TP2_Port	GPIOC

#define TP3_Pin		GPIO_Pin_6
#define TP3_Port	GPIOC

#define TP4_Pin		GPIO_Pin_7
#define TP4_Port	GPIOC

///����������Ŷ���////
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


////led�����Ŷ���////
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

///�������Ŷ���////
#define KEY_Pin		GPIO_Pin_12
#define KEY_Port	GPIOB

#define RDKEY()		GPIO_ReadInputDataBit(KEY_Port,KEY_Pin)


///���������Ŷ���////
#define DM1_Pin		GPIO_Pin_0		///adc10
#define DM1_Port	GPIOC

#define DM2_Pin		GPIO_Pin_1		//adc11
#define DM2_Port	GPIOC

#define DM3_Pin		GPIO_Pin_2		//adc12
#define DM3_Port	GPIOC

#define DM4_Pin		GPIO_Pin_3		//adc13
#define DM4_Port	GPIOC


///��λ���ض���////
#define M1_LIMIT_A_Pin	GPIO_Pin_8		//������λ
#define M1_LIMIT_A_Port	GPIOC

#define M1_LIMIT_B_Pin	GPIO_Pin_9		//�ظ���λ
#define M1_LIMIT_B_Port	GPIOC

#define M2_LIMIT_A_Pin	GPIO_Pin_10		//������λ
#define M2_LIMIT_A_Port	GPIOC

#define M2_LIMIT_B_Pin	GPIO_Pin_11		//�ظ���λ
#define M2_LIMIT_B_Port	GPIOC

#define M3_LIMIT_A_Pin	GPIO_Pin_12		//������λ
#define M3_LIMIT_A_Port	GPIOC

#define M3_LIMIT_B_Pin	GPIO_Pin_13		//�ظ���λ
#define M3_LIMIT_B_Port	GPIOC

#define M4_LIMIT_A_Pin	GPIO_Pin_14		//������λ
#define M4_LIMIT_A_Port	GPIOC

#define M4_LIMIT_B_Pin	GPIO_Pin_15		//�ظ���λ
#define M4_LIMIT_B_Port	GPIOC

#define Get_M1_Open_Limit()		GPIO_ReadInputDataBit(M1_LIMIT_A_Port,M1_LIMIT_A_Pin)//��ȡm1������λ
#define Get_M1_Close_Limit()	GPIO_ReadInputDataBit(M1_LIMIT_B_Port,M1_LIMIT_B_Pin)//��ȡm1�ظ���λ

#define Get_M2_Open_Limit()		GPIO_ReadInputDataBit(M2_LIMIT_A_Port,M2_LIMIT_A_Pin)//��ȡm1������λ
#define Get_M2_Close_Limit()	GPIO_ReadInputDataBit(M2_LIMIT_B_Port,M2_LIMIT_B_Pin)//��ȡm1�ظ���λ

#define Get_M3_Open_Limit()		GPIO_ReadInputDataBit(M3_LIMIT_A_Port,M3_LIMIT_A_Pin)//��ȡm1������λ
#define Get_M3_Close_Limit()	GPIO_ReadInputDataBit(M3_LIMIT_B_Port,M3_LIMIT_B_Pin)//��ȡm1�ظ���λ

#define Get_M4_Open_Limit()		GPIO_ReadInputDataBit(M4_LIMIT_A_Port,M4_LIMIT_A_Pin)//��ȡm1������λ
#define Get_M4_Close_Limit()	GPIO_ReadInputDataBit(M4_LIMIT_B_Port,M4_LIMIT_B_Pin)//��ȡm1�ظ���λ


///USR_TCP232_T2ģ�����Ŷ���
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

////��ť�������Ŷ���
#define BTN1_Pin			GPIO_Pin_4		///�к�
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
