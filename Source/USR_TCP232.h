#include "main.h"


typedef enum {
    USR_GPIO = 0,   // GPIO init

    USR_DELAY,      // delay

    // [[ hard reset
    USR_HW_RESET1,
    // hard reset ]]

    USR_SOFT_RESET, // soft reset

    USR_CIPMODE_EXIT1,// ÍË³öÍ¸´«
    USR_CIPMODE_EXIT2,
    USR_CIPMODE_EXIT3,
	USR_CIPMODE_EXIT4,

    // [[ config
    USR_CONFIG01,
    USR_CONFIG02,
    USR_CONFIG03,
    USR_CONFIG04,
    USR_CONFIG05,
	USR_CONFIG06,
	USR_CONFIG07,

    // config ]]

	USR_OVERTIME,
    USR_ERROR,
} usr_init_t;
extern usr_init_t usr_curr_status;
extern usr_init_t usr_chk_curr_status;


#define USARTy_DMA 0
#define USARTy_DMA_RECV 0



#define TX1BUF_SIZE 256//256
#define RX1BUF_SIZE 256//256
#define USR_DELAY_COMMAND       300

extern uint8_t  Tx1_Buf[TX1BUF_SIZE];
extern uint8_t  Tx1_flag;
extern uint8_t  Rx1_Buf[RX1BUF_SIZE];
extern uint16_t Rx1_Count;
extern uint8_t  Rx1_flag;
extern uint16_t Rx1_to;
extern uint16_t Rx1_to_len;

extern usr_init_t usr_curr_status;
extern usr_init_t usr_delay_status; // delay
extern usr_init_t usr_cipmode_status; //

extern uint16_t usr_tick_delay; // 1ms


#define BUFF_ESP_RECV   Rx1_Buf//Rx3_Buf
#define POINT_ESP_RECV  Rx1_Count//Rx3_Count
#define SIZE_ESP_RECV   RX1BUF_SIZE//RX3BUF_SIZE
#define FLAG_ESP_RECV   Rx1_flag//Rx3_flag
#define TIMEOVER_RECV   Rx1_to_len//Rx3_to_len

#define BUFF_ESP_SEND   Tx1_Buf//Tx3_Buf




void USART1_printf( char* fmt, ... );
void USART1_Send_Array( uint8_t* array, uint16_t num );
void recv_process( uint8_t dt );
void USR_INIT( void );
void Check_Tcp(void);
void Start_Check_Tcp(void);



#define USR_SendString  USART1_printf
#define USR_SendArray   USART1_Send_Array
#define BUFF_ESP_RECV   Rx1_Buf//Rx3_Buf





