#include "main.h"
#include "USR_TCP232.h"
#include "led.h"
#include "manage.h"

global_variable_t gb_var;
motor_operation_t motor[4];
filter_data_t overflow_value[4];

device_para_state_t device_para_state;

					//      0      1      2       3         4       5      6      7      8       9      
/**/uint8_t TESTBUF[32] = {0x23,0x5f,0x0A,0x00,0x00,0x00,0x00,0x07,0xff,0xff,
					   0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
					   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
					   0x00,0x0E};///0x030E


GPIO_TypeDef* motor_stp_port_tab[] = {M1_STP_Port, M2_STP_Port, M3_STP_Port, M4_STP_Port};
uint16_t 	 motor_stp_pin_tab[] =  {M1_STP_Pin , M2_STP_Pin , M3_STP_Pin , M4_STP_Pin };

GPIO_TypeDef* motor_dir_port_tab[] = {M1_DIR_Port, M2_DIR_Port, M3_DIR_Port, M4_DIR_Port};
uint16_t 	 motor_dir_pin_tab[] =  {M1_DIR_Pin , M2_DIR_Pin , M3_DIR_Pin , M4_DIR_Pin };

GPIO_TypeDef* motor_en_port_tab[] = {M1_EN_Port, M2_EN_Port, M3_EN_Port, M4_EN_Port};
uint16_t 	 motor_en_pin_tab[] =  {M1_EN_Pin , M2_EN_Pin , M3_EN_Pin , M4_EN_Pin };



GPIO_TypeDef* open_limit_port_tab[] = {M1_LIMIT_A_Port, M2_LIMIT_A_Port, M3_LIMIT_A_Port, M4_LIMIT_A_Port};
uint16_t 	  open_limit_pin_tab[] =  {M1_LIMIT_A_Pin , M2_LIMIT_A_Pin , M3_LIMIT_A_Pin , M4_LIMIT_A_Pin };

GPIO_TypeDef* close_limit_port_tab[] = {M1_LIMIT_B_Port, M2_LIMIT_B_Port, M3_LIMIT_B_Port, M4_LIMIT_B_Port};
uint16_t 	  close_limit_pin_tab[] =  {M1_LIMIT_B_Pin , M2_LIMIT_B_Pin , M3_LIMIT_B_Pin , M4_LIMIT_B_Pin };


voice_pormpt_e tips_voice_tab[] = {V_Harmful_waste, V_Kitchen_waste, V_Recycle_waste, V_Other_waste};


void DelayUs(uint32_t us)////=10,å®žæµ‹18us
{
	uint32_t i = 0;

	while( us-- ) {
		i = 2 * ( SystemFrequency / 16000000 ); //16M=2
		while( i-- ) ;
		//WDR();
	}
}

void Delay_ms( uint32_t nTime )
{
    unsigned short i = 0;

    while( nTime-- ) {
        i = 2666 * ( SystemFrequency / 16000000 ); //16M=2666
        while( i-- ) ;
		//WDR();
    }
}

union union_temp16
{
    unsigned int un_temp16;
    unsigned char  un_temp8[2];		// example 16: 0x0102  8:[0]2 [1]1
}my_unTemp16;


/******************************************************
flash ×Ö·û´®Ð´Èë
Ã¿´Î´æÈëÁ½¸ö×Ö½Ú
*******************************************************/
void FlashWriteStr( u32 flash_add, u16 len, u8* data )
{
	//char cp[12];
	//u8 s = 0;
	u16 byteN = 0;


	FLASH_Unlock();
	FLASH_ErasePage(flash_add);
	//sprintf( cp, "len:%d", len);
//	Print("S");
	while( len )
	{
		my_unTemp16.un_temp8[0] = *(data+byteN);
		my_unTemp16.un_temp8[1] = *(data+byteN+1);		
		FLASH_ProgramHalfWord( flash_add+byteN , my_unTemp16.un_temp16 );

		//sprintf( cp, "bye:%d\r\n", s);
		//USART1_Puts(cp);
		if( 1==len )
		{
			//Èç¹ûÊý¾Ý³¤¶ÈÊÇÆæÊý,Îª1µÄÊ±ºòÌø³ö
			break;													   
		}
		else
		{
			byteN += 2;
			len -= 2;
		}	
	}
	FLASH_Lock();
}

/******************************************************
flash ×Ö·û´®¶Á³öµ½Ö¸¶¨dataÖÐ  
µØÖ·ÓëÐ´ÈëdataµØÖ·Í¬ ¶Á³öµÄ±£´æÀàÐÍÒ²±ØÐëÒ»ÖÂ
*******************************************************/
void FlashReadStr( u32 flash_add, u16 len, u8* data )
{
	u16 byteN = 0;
	while( len )
	{
		my_unTemp16.un_temp16 = *(vu16*)(flash_add+byteN);
		if( 1==len )
		{
			*(data+byteN) = my_unTemp16.un_temp8[0];
			break;			   
		}
		else
		{		
			*(data+byteN) = my_unTemp16.un_temp8[0];
			*(data+byteN+1) = my_unTemp16.un_temp8[1];
			byteN += 2;
			len -= 2;
		}
	}
}



void TIMER_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//72Mæ—¶é—´çš„åˆå§‹åŒ–
	TIM_TimeBaseStructure.TIM_Period =1000-1;//10MS
	TIM_TimeBaseStructure.TIM_Prescaler =(SystemFrequency/100000)-1;//10ms//åˆ†é¢‘
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ä¸»æ—¶é’Ÿåˆ†é¢‘
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_UpdateRequestConfig( TIM2, TIM_UpdateSource_Regular);
	TIM_Cmd(TIM2,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

	/* Enable the TIM2 global Interrupt  0æœ€é«˜*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void USART1_Configuration(uint32_t baud)//¹ã¸æ»úÍ¨Ñ¶»òÕßtcp×ª232½Ó¿ÚÍ¨Ñ¶
{
	GPIO_InitTypeDef gpioInitStruct;
	USART_InitTypeDef usartInitStruct;
	NVIC_InitTypeDef nvicInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);				//æ‰“å¼€GPIOAæ—¶é’Ÿ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);				//æ‰“å¼€USART1çš„æ—¶é’Ÿ

/**/
	//PA9	TXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 				//è®¾ç½®ä¸ºå¤ç”¨æ¨¡å¼	
	gpioInitStruct.GPIO_Pin = GPIO_Pin_9;						//åˆå§‹åŒ–Pin9	
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;					//æ‰¿è½½çš„æœ€å¤§é¢‘çŽ‡	
	GPIO_Init(GPIOA, &gpioInitStruct);
	

	//PA10	RXD	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;				//è®¾ç½®ä¸ºæµ®ç©ºæ¨¡å¼	
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10;							//åˆå§‹åŒ–Pin10	
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;					//æ‰¿è½½çš„æœ€å¤§é¢‘çŽ‡	
	GPIO_Init(GPIOA, &gpioInitStruct);

	usartInitStruct.USART_BaudRate = baud;   //è®¾ç½®ä¸²å£æ³¢ç‰¹çŽ‡	
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//æ— ç¡¬ä»¶æµæŽ§åˆ¶	
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//æŽ¥æ”¶å’Œå‘é€	
	usartInitStruct.USART_Parity = USART_Parity_No;					//æ— æ ¡éªŒ	
	usartInitStruct.USART_StopBits = USART_StopBits_1;				//1ä½åœæ­¢ä½	
	usartInitStruct.USART_WordLength = USART_WordLength_8b;				//8ä½æ•°æ®ä½	
	USART_Init(USART1, &usartInitStruct);


	nvicInitStruct.NVIC_IRQChannel = USART1_IRQn;					//usart1ä¸­æ–­å·	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);					//must!!!å¦åˆ™ä¼šé€ æˆå¤§é‡æ•°æ®ä¸¢å¤±ï¼Œé€ æˆå¤šåŒ…ä¸‹è½½å®¹æ˜“å¤±è´¥20190817
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE; 				//ä¸­æ–­é€šé“ä½¿èƒ½	
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;				//æŠ¢å ä¸­æ–­ä¼˜å…ˆçº§ï¼ˆå€¼è¶Šå°ä¼˜å…ˆçº§è¶Šé«˜ï¼‰	
	nvicInitStruct.NVIC_IRQChannelSubPriority = 1;					//å­ä¸­æ–­ä¼˜å…ˆçº§ï¼ˆå€¼è¶Šå°ä¼˜å…ˆçº§è¶Šé«˜ï¼‰	
	NVIC_Init(&nvicInitStruct);



	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);					//ä½¿èƒ½æŽ¥æ”¶ä¸­æ–­

	USART_Cmd(USART1, ENABLE);

}

void USART2_Configuration(uint32_t baud)//MP3²¥·ÅÆ÷Ê¹ÓÃ
{
	GPIO_InitTypeDef gpioInitStruct;
	USART_InitTypeDef usartInitStruct;
	NVIC_InitTypeDef nvicInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);				
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);				

/**/
	//PA2	TXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 				
	gpioInitStruct.GPIO_Pin = GPIO_Pin_2;						
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;					
	GPIO_Init(GPIOA, &gpioInitStruct);
	

	//PA3	RXD	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;				
	gpioInitStruct.GPIO_Pin = GPIO_Pin_3;							
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;					
	GPIO_Init(GPIOA, &gpioInitStruct);

	usartInitStruct.USART_BaudRate = baud;   
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			
	usartInitStruct.USART_Parity = USART_Parity_No;					
	usartInitStruct.USART_StopBits = USART_StopBits_1;				
	usartInitStruct.USART_WordLength = USART_WordLength_8b;				
	USART_Init(USART2, &usartInitStruct);


	nvicInitStruct.NVIC_IRQChannel = USART2_IRQn;					
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);					//must!!!å¦åˆ™ä¼šé€ æˆå¤§é‡æ•°æ®ä¸¢å¤±ï¼Œé€ æˆå¤šåŒ…ä¸‹è½½å®¹æ˜“å¤±è´¥20190817
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE; 				//ä¸­æ–­é€šé“ä½¿èƒ½	
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;				//æŠ¢å ä¸­æ–­ä¼˜å…ˆçº§ï¼ˆå€¼è¶Šå°ä¼˜å…ˆçº§è¶Šé«˜ï¼‰	
	nvicInitStruct.NVIC_IRQChannelSubPriority = 2;					//å­ä¸­æ–­ä¼˜å…ˆçº§ï¼ˆå€¼è¶Šå°ä¼˜å…ˆçº§è¶Šé«˜ï¼‰	
	NVIC_Init(&nvicInitStruct);



	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);					//ä½¿èƒ½æŽ¥æ”¶ä¸­æ–­

	USART_Cmd(USART2, ENABLE);

}

void USART3_Configuration(uint32_t baud)//¹ã¸æ»ú±¸ÓÃ´®¿Ú
{
	GPIO_InitTypeDef gpioInitStruct;
	USART_InitTypeDef usartInitStruct;
	NVIC_InitTypeDef nvicInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);				
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);				

/**/
	//PB10	TXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 				
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10;						
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;					
	GPIO_Init(GPIOB, &gpioInitStruct);
	

	//PB11	RXD	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;				
	gpioInitStruct.GPIO_Pin = GPIO_Pin_11;							
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;					
	GPIO_Init(GPIOB, &gpioInitStruct);

	usartInitStruct.USART_BaudRate = baud;   
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			
	usartInitStruct.USART_Parity = USART_Parity_No;					
	usartInitStruct.USART_StopBits = USART_StopBits_1;				
	usartInitStruct.USART_WordLength = USART_WordLength_8b;				
	USART_Init(USART3, &usartInitStruct);


	nvicInitStruct.NVIC_IRQChannel = USART3_IRQn;					
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);					//must!!!å¦åˆ™ä¼šé€ æˆå¤§é‡æ•°æ®ä¸¢å¤±ï¼Œé€ æˆå¤šåŒ…ä¸‹è½½å®¹æ˜“å¤±è´¥20190817
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE; 				//ä¸­æ–­é€šé“ä½¿èƒ½	
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;				//æŠ¢å ä¸­æ–­ä¼˜å…ˆçº§ï¼ˆå€¼è¶Šå°ä¼˜å…ˆçº§è¶Šé«˜ï¼‰	
	nvicInitStruct.NVIC_IRQChannelSubPriority = 3;					//å­ä¸­æ–­ä¼˜å…ˆçº§ï¼ˆå€¼è¶Šå°ä¼˜å…ˆçº§è¶Šé«˜ï¼‰	
	NVIC_Init(&nvicInitStruct);



	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);					//ä½¿èƒ½æŽ¥æ”¶ä¸­æ–­

	USART_Cmd(USART3, ENABLE);

}


void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
						   RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
						   RCC_APB2Periph_GPIOE| RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);////ÓÃPB3£¬PB4£¬PA15×öÆÕÍ¨IO£¬PA13&14ÓÃÓÚSWDµ÷ÊÔ

	////²âÊÔ½Å³õÊ¼»¯
	GPIO_InitStructure.GPIO_Pin = TP1_Pin | TP2_Pin | TP3_Pin | TP3_Pin;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TP1_Port, &GPIO_InitStructure);		

	////LEDÒý½Å³õÊ¼»¯
	#ifndef CWS
	GPIO_InitStructure.GPIO_Pin = L_SYS_Pin | L_TCP_Pin;	
	#else
	GPIO_InitStructure.GPIO_Pin = L_TCP_Pin;	
	#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(L_TCP_Port, &GPIO_InitStructure);

	////°´¼üÒý½Å³õÊ¼»¯
	GPIO_InitStructure.GPIO_Pin = KEY_Pin;//	Ñ¡ÔñÒª¿ØÖÆµÄKEYÒý½Å
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  ///ÉÏÀ­ÊäÈëÄ£Ê½
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  ///ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃÍÆÍìÊä³ö 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //ÉèÖÃÒý½ÅËÙÂÊÎª50MHz 
	GPIO_Init(KEY_Port, &GPIO_InitStructure);//µ÷ÓÃ¿âº¯Êý£¬³õÊ¼»¯ÏàÓ¦GPIO

	///ÏÞÎ»¿ª¹ØÒý½Å³õÊ¼»¯///
	GPIO_InitStructure.GPIO_Pin = 	M1_LIMIT_A_Pin | M1_LIMIT_B_Pin |\
									M2_LIMIT_A_Pin | M2_LIMIT_B_Pin |\
									M3_LIMIT_A_Pin | M3_LIMIT_B_Pin |\
									M4_LIMIT_A_Pin | M4_LIMIT_B_Pin ;//	Ñ¡ÔñÒª¿ØÖÆµÄKEYÒý½Å
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  ///ÉÏÀ­ÊäÈëÄ£Ê½
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  ///ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃÍÆÍìÊä³ö 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //ÉèÖÃÒý½ÅËÙÂÊÎª50MHz 
	GPIO_Init(M1_LIMIT_A_Port, &GPIO_InitStructure);//µ÷ÓÃ¿âº¯Êý£¬³õÊ¼»¯ÏàÓ¦GPIO	

	//ºìÍâ²â¾à½Å³õÊ¼»¯///
	GPIO_InitStructure.GPIO_Pin = 	DM1_Pin | DM2_Pin | DM3_Pin | DM4_Pin;//	Ñ¡ÔñÒª¿ØÖÆµÄKEYÒý½Å
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  ///ÉÏÀ­ÊäÈëÄ£Ê½
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  ///ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃÍÆÍìÊä³ö 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //ÉèÖÃÒý½ÅËÙÂÊÎª50MHz 
	GPIO_Init(DM1_Port, &GPIO_InitStructure);//µ÷ÓÃ¿âº¯Êý£¬³õÊ¼»¯ÏàÓ¦GPIO		

	//µç»ú¿ØÖÆ½Å///
	GPIO_InitStructure.GPIO_Pin = 	M1_DIR_Pin | M1_STP_Pin | M1_EN_Pin | M2_DIR_Pin | M2_STP_Pin;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(M1_DIR_Port, &GPIO_InitStructure);//µ÷ÓÃ¿âº¯Êý£¬³õÊ¼»¯ÏàÓ¦GPIO		

	GPIO_InitStructure.GPIO_Pin =	M2_EN_Pin;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(M2_EN_Port, &GPIO_InitStructure);//µ÷ÓÃ¿âº¯Êý£¬³õÊ¼»¯ÏàÓ¦GPIO		

	GPIO_InitStructure.GPIO_Pin = 	M3_DIR_Pin | M3_STP_Pin | M3_EN_Pin | M4_DIR_Pin | M4_STP_Pin | M4_EN_Pin;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(M3_DIR_Port, &GPIO_InitStructure);//µ÷ÓÃ¿âº¯Êý£¬³õÊ¼»¯ÏàÓ¦GPIO	

	//usr_TCP232Òý½Å³õÊ¼»¯//
	GPIO_InitStructure.GPIO_Pin = USR_CFG_Pin | USR_REST_Pin ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USR_CFG_PORT, &GPIO_InitStructure);

	

	
}

void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);//ADC1_DR_Address;//ADCµØÖ·	
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&gb_var.ADCConvertedValue; //ÄÚ´æµØÖ·	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //·½Ïò(´ÓÍâÉèµ½ÄÚ´æ) 
	DMA_InitStructure.DMA_BufferSize = 4*10; //´«ÊäÄÚÈÝµÄ´óÐ¡
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //ÍâÉèµØÖ·¹Ì¶¨ 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //ÓÃÀ´Éè¶¨ÄÚ´æµØÖ·¼Ä´æÆ÷µÝÔöÓë·ñ,´Ë´¦ÉèÎªµÝÔö£¬Enable
	DMA_InitStructure.DMA_PeripheralDataSize =	DMA_PeripheralDataSize_HalfWord ; //ÍâÉèÊý¾Ýµ¥Î»	
	DMA_InitStructure.DMA_MemoryDataSize =	DMA_MemoryDataSize_HalfWord ;	 //ÄÚ´æÊý¾Ýµ¥Î» 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular	; //DMAÄ£Ê½£ºÑ­»·´«Êä	
	DMA_InitStructure.DMA_Priority = DMA_Priority_High ; //ÓÅÏÈ¼¶£º¸ß	
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //½ûÖ¹ÄÚ´æµ½ÄÚ´æµÄ´«Êä		
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //ÅäÖÃDMA1µÄ4Í¨µÀ 
	DMA_Cmd(DMA1_Channel1,ENABLE);	


	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //¶ÀÁ¢ADCÄ£Ê½	
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;  //½ûÖ¹É¨Ãè·½Ê½	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//¿ªÆôÁ¬Ðø×ª»»Ä£Ê½	
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //²»Ê¹ÓÃÍâ²¿´¥·¢×ª»»
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //²É¼¯Êý¾ÝÓÒ¶ÔÆë 
	ADC_InitStructure.ADC_NbrOfChannel = 4; //Òª×ª»»µÄÍ¨µÀÊýÄ¿	
	ADC_Init(ADC1, &ADC_InitStructure); 	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);//ÅäÖÃADCÊ±ÖÓ£¬ÎªPCLK2µÄ8·ÖÆµ£¬¼´9Hz	

	ADC_RegularChannelConfig(ADC1, AD_OVER_HARMFUL, 1, ADC_SampleTime_55Cycles5);//ÅäÖÃADC1Í¨µÀ11Îª55.5¸ö²ÉÑùÖÜÆÚ		
	ADC_RegularChannelConfig(ADC1, AD_OVER_KITCHEN, 2, ADC_SampleTime_55Cycles5);//ÅäÖÃADC1Í¨µÀ11Îª55.5¸ö²ÉÑùÖÜÆÚ	
	ADC_RegularChannelConfig(ADC1, AD_OVER_RECYCLE, 3, ADC_SampleTime_55Cycles5);//ÅäÖÃADC1Í¨µÀ11Îª55.5¸ö²ÉÑùÖÜÆÚ	
	ADC_RegularChannelConfig(ADC1, AD_OVER_OTHER  , 4, ADC_SampleTime_55Cycles5);//ÅäÖÃADC1Í¨µÀ11Îª55.5¸ö²ÉÑùÖÜÆÚ	

	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1,ENABLE);
	ADC_ResetCalibration(ADC1);//¸´Î»Ð£×¼¼Ä´æÆ÷ 
	while(ADC_GetResetCalibrationStatus(ADC1));//µÈ´ýÐ£×¼¼Ä´æÆ÷¸´Î»Íê³É 	
	ADC_StartCalibration(ADC1);//ADCÐ£×¼	
	while(ADC_GetCalibrationStatus(ADC1));//µÈ´ýÐ£×¼Íê³É	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ÓÉÓÚÃ»ÓÐ²ÉÓÃÍâ²¿´¥·¢£¬ËùÒÔÊ¹ÓÃÈí¼þ´¥·¢ADC×ª»»


}


void Timer_10ms_ISR(void)////10ms¶¨Ê±Æ÷
{
	//GPIO_ToggleBits(TP2_Port , TP2_Pin);
	gb_var.bKeyScan = TRUE;
	gb_var.bBtnScan = TRUE;

	if(--gb_var.timer_function_count==0)
		{
		gb_var.bTimerFunc=TRUE;
		gb_var.timer_function_count=TIMERFUNC_0S5;
		}

	if(--gb_var.device_report_count == 0)
		{
		gb_var.device_report_count = TIMERFUNC_1S * REPORT_TIME_MAX;
		//gb_var.bReportData = TRUE;
		gb_var.SendInfoMode = send_report;
		}

	if( gb_var.rx1_timeout > 0) {
		gb_var.rx1_timeout --;
		if((gb_var.rx1_timeout==0)&&(gb_var.rx1_have_data==1))
			{
			gb_var.rx1_have_data = 3;
			}
	}

	if( gb_var.rx2_timeout > 0) {
		gb_var.rx2_timeout --;
		if((gb_var.rx2_timeout==0)&&(gb_var.rx2_have_data==1))
			{
			gb_var.rx2_have_data = 3;
			}
	}	
	
	if( gb_var.rx3_timeout > 0) {
		gb_var.rx3_timeout --;
		if((gb_var.rx3_timeout==0)&&(gb_var.rx3_have_data==1))
			{
			gb_var.rx3_have_data = 3;
			}
	}	


	//	if(gb_var.test1 == 1)
	//	GPIO_ToggleBits(M1_STP_Port, M1_STP_Pin);
/**/
	for(uint8_t i =0 ;i < MOTOR_NUM_MAX ; i++)
		{
		//if((--motor[i].Operation_count == 0)||((!(GPIO_ReadInputDataBit(open_limit_port_tab[i],open_limit_pin_tab[i])))&&(OPEN == motor[i].Direction))\
		//	|| ((!(GPIO_ReadInputDataBit(close_limit_port_tab[i],close_limit_pin_tab[i])))&&(CLOSE== motor[i].Direction)))
		if((RUN == motor[i].bMotor_operation)&&(((!(GPIO_ReadInputDataBit(open_limit_port_tab[i],open_limit_pin_tab[i])))&&(OPEN == motor[i].Direction))\
			|| ((!(GPIO_ReadInputDataBit(close_limit_port_tab[i],close_limit_pin_tab[i])))&&(CLOSE== motor[i].Direction))))
			{
			//d_debug("is limited!!!");
			if(OPEN == motor[i].Direction)
				{
				if(0 == device_para_state.OPENDOOR_TIME)
					motor[i].Door_Keep_Open_Time = DOOR_KEEP_OPEN_TIME;//Ä¬ÈÏ´ò¿ªÊ±¼ä
				else
					motor[i].Door_Keep_Open_Time = device_para_state.OPENDOOR_TIME * TIMERFUNC_1S;
				}
				
			
			motor[i].bMotor_operation = STOP;
/*			
			switch(i)
				{
				case Motor_harmful:
					M1_EN(STOP);
					break;
				case Motor_kitchen:
					M2_EN(STOP);
					break;
				case Motor_recycle:
					M3_EN(STOP);
					break;
				case Motor_other:
					M4_EN(STOP);
					break;
				}*/
			}

		if((motor[i].Door_Keep_Open_Time > 0) && (motor[i].bDoorKeepOpen == FALSE))
			{
			if(--motor[i].Door_Keep_Open_Time == 0)
				{
				Motor_Operation((door_motor_e)i, CLOSE);
				mp3_Play(V_Closeing_Be_careful);
				}
			}
		}
	
/*	if(((!(GPIO_ReadInputDataBit(open_limit_port_tab[Motor_harmful],open_limit_pin_tab[Motor_harmful])))&&(OPEN == motor[Motor_harmful].Direction))
		|| ((!(GPIO_ReadInputDataBit(close_limit_port_tab[Motor_harmful],close_limit_pin_tab[Motor_harmful])))&&(CLOSE== motor[Motor_harmful].Direction)))
		{
		motor[Motor_harmful].bMotor_operation = STOP;
		motor[Motor_harmful].Operation_count = 0;
		M1_EN(STOP);

		}*/
		
}


void SysTick_ISR(void)/////1mså®šæ—¶å™¨
{


	if( Rx1_to > 0 ) {
		Rx1_to--;
		if( 0 == Rx1_to ) {
			Rx1_flag = 1;
		}
	}

	if( usr_tick_delay > 0 ) {
		usr_tick_delay--;
		
	}

	if( gb_var.tcp_chk_connect_delay_cnt > 0 )
		{
			gb_var.tcp_chk_connect_delay_cnt --;
			if(0 == gb_var.tcp_chk_connect_delay_cnt)
				{
				//Start_Check_Tcp();
				}
		}
/*	
	if( gb_var.tcp_send_logoninfo_cnt > 0 )
		{
			gb_var.tcp_send_logoninfo_cnt --;
			if(0 == gb_var.tcp_send_logoninfo_cnt)
				{
				//gb_var.bSendLogonInfo = TRUE;
				gb_var.SendInfoMode = send_report;
				}
		}
*/
	for( uint8_t i = 0; i < MAX_LLED_COUNT; i++ ) {
		if( led_overtime[i] > 0 ) {
			led_overtime[i]--;
			if( 0 == led_overtime[i] ) {
				hw_led_off( ( show_led_t )i );
			}
		}
	}

	for( uint8_t i = 0; i < MAX_LLED_COUNT; i++ ) {
		if( led_oncnt[i] > 0 ) {
			led_oncnt[i]--;
			if( 0 == led_oncnt[i] ) {
				hw_led_off( ( show_led_t )i );
				
			}
		}
		else if(led_offcnt[i] > 0) {
			led_offcnt[i]--;
			if( 0 == led_offcnt[i] ) {
				
				if( led_flashcnt[i] > 0 )
					{
					if(led_flashcnt[i]!=AlWAYS_FLASH)
						led_flashcnt[i]--;
					led_offcnt[i] = led_offtime[i];
					led_oncnt[i] = led_ontime[i];
					hw_led_on( (show_led_t)i , 0 );
					}
			}
		}
	}

/*	
	for(uint8_t i =0 ;i < MOTOR_NUM_MAX ; i++)
		{
		if(RUN == motor[i].bMotor_operation)
			GPIO_ToggleBits(motor_stp_port_tab[i], motor_stp_pin_tab[i]);///µç»úÇý¶¯Âö³å
		}
*/
/*
	if(RUN == motor[Motor_harmful].bMotor_operation)
		GPIO_ToggleBits(M1_STP_Port, M1_STP_Pin);

	if(RUN == motor[Motor_kitchen].bMotor_operation)
		GPIO_ToggleBits(M2_STP_Port, M2_STP_Pin);

	if(RUN == motor[Motor_recycle].bMotor_operation)
		GPIO_ToggleBits(M3_STP_Port, M3_STP_Pin);
	
	if(RUN == motor[Motor_other].bMotor_operation)
		GPIO_ToggleBits(M4_STP_Port, M4_STP_Pin);
*/





}

int fputc(int ch, FILE *f)
{ 
	while((USART3->SR&0X40)==0);  //Ñ­»··¢ËÍ,Ö±µ½·¢ËÍÍê±Ï  
    USART3->DR = (u8) ch;     
   return ch;
}

void USART1_Send_Array( uint8_t* array, uint16_t num )
{

    for( uint16_t i = 0; i < num; i++ ) {
        USART_SendData( USART1, array[i] );
        while( USART_GetFlagStatus( USART1, USART_FLAG_TXE ) == RESET );
    }

}

void USART2_Send_Array( uint8_t* array, uint16_t num )
{

    for( uint16_t i = 0; i < num; i++ ) {
        USART_SendData( USART2, array[i] );
        while( USART_GetFlagStatus( USART2, USART_FLAG_TXE ) == RESET );
    }

}


void Start_Send_PC(uint8_t *array, uint8_t len)
{
	uint8_t buffer[32];
	buffer[0] = HEAD0;
	buffer[1] = HEAD1;
	memcpy(buffer+2,array,len);
	buffer[31] = MakeChk(array,len);
	//L_SYS_ON;
	USART2_Send_Array( buffer, sizeof(buffer) );

}


void BtnOpenDoor(void)
{
	switch(gb_var.BtnNo)
		{
		case Harmful_Waste :
			if(motor[Motor_harmful].Direction == OPEN)break;
			motor[Motor_harmful].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_harmful , OPEN);
			mp3_Play(tips_voice_tab[Motor_harmful]);
			break;
		case Kitchen_Waste:
			if(motor[Motor_kitchen].Direction == OPEN)break;
			motor[Motor_kitchen].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_kitchen, OPEN);	
			mp3_Play(tips_voice_tab[Motor_kitchen]);
			break;
		case Recycle_Waste:
			if(motor[Motor_recycle].Direction == OPEN)break;
			motor[Motor_recycle].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_recycle, OPEN);	
			mp3_Play(tips_voice_tab[Motor_recycle]);
			break;
		case Other_Waste:
			if(motor[Motor_other].Direction == OPEN)break;
			motor[Motor_other].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_other, OPEN);	
			mp3_Play(tips_voice_tab[Motor_other]);
			break;
		}
	gb_var.bBtnOk = FALSE;
}


#ifdef USE_BTN_OPEN
void BtnScan(void)
{
	uint8_t j, i;
	static uint8_t BtnPress = 0xff;
	static uint16_t BtnTime = 0;
	
	gb_var.bBtnScan = FALSE;
	j = 0xff;
	for(i = 0; i < 4; i++)
		{
		GPIO_ResetBits(BTN1_Port, BTN1_Pin);
		GPIO_ResetBits(BTN2_Port, BTN2_Pin);
		GPIO_ResetBits(BTN3_Port, BTN3_Pin);
		GPIO_ResetBits(BTN4_Port, BTN4_Pin);

		switch(i)
			{
			case 0:
				if(GPIO_ReadInputDataBit(BTN1_Port,BTN1_Pin)){j=Harmful_Waste;goto RDKEY;}
				break;
			case 1:
				if(GPIO_ReadInputDataBit(BTN2_Port,BTN2_Pin)){j=Kitchen_Waste;goto RDKEY;}
				break;
			case 2:
				if(GPIO_ReadInputDataBit(BTN3_Port,BTN3_Pin)){j=Recycle_Waste;goto RDKEY;}
				break;
			case 3:
				if(GPIO_ReadInputDataBit(BTN4_Port,BTN4_Pin)){j=Other_Waste;goto RDKEY;}
				break;
			}
		}

RDKEY:
	
	if(j == 0xff)
		{
		BtnPress = 0xff;
		BtnTime = 0;
		return ;
		}

	if(j == BtnPress)
		{
			if(BtnTime < 0xffff) BtnTime ++;

			if((j == Harmful_Waste) && (BtnTime > 200))
				{
				///2s³¤°´°´¼üÓÃ»§´úÂë
				
				
				BtnTime = 0;
				return;
				}

			if((j == Kitchen_Waste) && (BtnTime > 200))
				{
				///2s³¤°´°´¼üÓÃ»§´úÂë
				
				
				BtnTime = 0;
				return;
				}

			if((j == Recycle_Waste) && (BtnTime > 200))
				{
				///2s³¤°´°´¼üÓÃ»§´úÂë
				
				
				BtnTime = 0;
				return;
				}
			
			if((j == Other_Waste) && (BtnTime > 200))
				{
				///2s³¤°´°´¼üÓÃ»§´úÂë
				
				
				BtnTime = 0;
				return;
				}


		}

	BtnPress = j;
	BtnTime = 0;

	
	gb_var.BtnNo = j;
	gb_var.bBtnOk = TRUE;

	
}

#else
void BtnScan(void)
{
	uint8_t j, i;
	static uint8_t BtnPress = 0xff;
	static uint16_t BtnTime = 0;
	
	gb_var.bBtnScan = FALSE;
	j = 0xff;
	for(i = 0; i < 4; i++)
		{
		GPIO_SetBits(BTN1_Port, BTN1_Pin);
		GPIO_SetBits(BTN2_Port, BTN2_Pin);
		GPIO_SetBits(BTN3_Port, BTN3_Pin);
		GPIO_SetBits(BTN4_Port, BTN4_Pin);

		switch(i)
			{
			case 0:
				if(!(GPIO_ReadInputDataBit(BTN1_Port,BTN1_Pin))){j=Harmful_Waste;goto RDKEY;}
				break;
			case 1:
				if(!(GPIO_ReadInputDataBit(BTN2_Port,BTN2_Pin))){j=Kitchen_Waste;goto RDKEY;}
				break;
			case 2:
				if(!(GPIO_ReadInputDataBit(BTN3_Port,BTN3_Pin))){j=Recycle_Waste;goto RDKEY;}
				break;
			case 3:
				if(!(GPIO_ReadInputDataBit(BTN4_Port,BTN4_Pin))){j=Other_Waste;goto RDKEY;}
				break;
			}
		}

RDKEY:
	
	if(j == 0xff)
		{
		BtnPress = 0xff;
		BtnTime = 0;
		return ;
		}

	if(j == BtnPress)
		{
			if(BtnTime < 0xffff) BtnTime ++;

			if((j == Harmful_Waste) && (BtnTime > 200))
				{
				///2s³¤°´°´¼üÓÃ»§´úÂë
				
				
				BtnTime = 0;
				return;
				}

			if((j == Kitchen_Waste) && (BtnTime > 200))
				{
				///2s³¤°´°´¼üÓÃ»§´úÂë
				
				
				BtnTime = 0;
				return;
				}

			if((j == Recycle_Waste) && (BtnTime > 200))
				{
				///2s³¤°´°´¼üÓÃ»§´úÂë
				
				
				BtnTime = 0;
				return;
				}
			
			if((j == Other_Waste) && (BtnTime > 200))
				{
				///2s³¤°´°´¼üÓÃ»§´úÂë
				
				
				BtnTime = 0;
				return;
				}


		}

	BtnPress = j;
	BtnTime = 0;

	
	gb_var.BtnNo = j;
	gb_var.bBtnOk = TRUE;

}
#endif



void KeyScan(void)
{

	static bool bkeypress=FALSE;
	static uint16_t keytime=0;
    int i = 0x45; 
    char s[10];


	gb_var.bKeyScan = FALSE;

	if(RDKEY())////Î´°´
		{
		bkeypress=FALSE;
		keytime=0;
		return;
		}
	else
		{
		keytime++;
		if(keytime==200)//³¤°´2s
			{
			keytime=0;
			/////³¤°´¶¯×÷´úÂë
			NOP();
			//gb_var.test1 = 0;
			//M1_DIR_DN;
			//M1_EN_UP;
			//motor[Motor_harmful].bMotor_operation = STOP;
			//Motor_Operation(Motor_harmful,CLOSE);
			//Start_Check_Tcp();

			return;
			}
		if(bkeypress==TRUE)return;
		bkeypress=TRUE;

		///////¶Ì°´¶¯×÷´úÂë
		NOP();
		//gb_var.test1 = 1;
		//GPIO_ToggleBits(M1_STP_Port, M1_STP_Pin);
		//M1_STP_DN;
		//M1_DIR_DN;
		//M1_EN_DN;
		
		//Motor_Operation(Motor_harmful,OPEN);.
/*	
		gb_var.test1++;
		if(gb_var.test1%2)
			{
			motor[Motor_harmful].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_harmful , CLOSE);
			
			motor[Motor_kitchen].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_kitchen , CLOSE);
			
			motor[Motor_recycle].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_recycle , CLOSE);
			
			motor[Motor_other].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_other , CLOSE);

			}
		else
			{
			motor[Motor_harmful].bDoorKeepOpen = TRUE;
			Motor_Operation(Motor_harmful , OPEN);
			
			motor[Motor_kitchen].bDoorKeepOpen = TRUE;
			Motor_Operation(Motor_kitchen , OPEN);
			
			motor[Motor_recycle].bDoorKeepOpen = TRUE;
			Motor_Operation(Motor_recycle , OPEN);
			
			motor[Motor_other].bDoorKeepOpen = TRUE;
			Motor_Operation(Motor_other , OPEN);
			}
			*/
		//if(gb_var.test1++)
		//motor[Motor_harmful].bMotor_operation = STOP;
/**/		
		gb_var.test_motor_dir = OPEN;
		Motor_Operation(Motor_harmful,gb_var.test_motor_dir);//Motor_harmful
		Motor_Operation(Motor_kitchen,gb_var.test_motor_dir);
		Motor_Operation(Motor_recycle,gb_var.test_motor_dir);
		Motor_Operation(Motor_other,gb_var.test_motor_dir);
		mp3_Play(V_Closeing_Be_careful);
		/*
			motor[Motor_harmful].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_harmful , CLOSE);
			
			motor[Motor_kitchen].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_kitchen , CLOSE);
			
			motor[Motor_recycle].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_recycle , CLOSE);
			
			motor[Motor_other].bDoorKeepOpen = FALSE;
			Motor_Operation(Motor_other , CLOSE);
		*/


		
		//USR_SendString("AT+ENTM\r\n");
		//uint8_t *p = "nihao";
		//esp_send_data(0,p,5);
		
		//Start_Check_Tcp();
		//SendTest();
		//SendInfo(1, device_para_state);
		//hw_led_fls( LED_SYS, 50 , 2000 , AlWAYS_FLASH);
		SendInfo(send_report, device_para_state);

		}

}

void Timer_500ms(void)////0.5s¶¨Ê±Æ÷
{
	gb_var.bTimerFunc = FALSE;
	//L_TCP_OFF;
	//L_SYS_OFF;
	//GPIO_ToggleBits(L_SYS_Port , L_SYS_Pin);
	//gb_var.test1 ++;
	//gb_var.test2 ++;
	//d_debug("Harmful:%d, Kitchen:%d, Recycle:%d, Other:%d.\n",gb_var.ADC_Value[0],gb_var.ADC_Value[1],gb_var.ADC_Value[2],gb_var.ADC_Value[3]);
/*
	d_debug("Harmful:%d, Kitchen:%d, Recycle:%d, Other:%d.\n",device_para_state.OVERFLOW_VAL[Motor_harmful],\
															  device_para_state.OVERFLOW_VAL[Motor_kitchen],\
															  device_para_state.OVERFLOW_VAL[Motor_recycle],\
															  device_para_state.OVERFLOW_VAL[Motor_other]);
*/
	//d_debug("M0T:%d, M1T:%d, M2T:%d, M3T:%d.  ",motor[0].Door_Keep_Open_Time,motor[1].Door_Keep_Open_Time,motor[2].Door_Keep_Open_Time,motor[3].Door_Keep_Open_Time);
	//d_debug("M0K:%d, M1K:%d, M2K:%d, M3K:%d.\n",motor[0].bDoorKeepOpen,motor[1].bDoorKeepOpen,motor[2].bDoorKeepOpen,motor[3].bDoorKeepOpen);

	//Receive_Net(TESTBUF);
//	GPIO_SetBits(M1_DIR_Port, M1_DIR_Pin);	
//	GPIO_ResetBits(M1_STP_Port, M1_STP_Pin);
/*
	GPIO_ToggleBits(M1_DIR_Port, M1_DIR_Pin);	
	GPIO_ToggleBits(M1_STP_Port, M1_STP_Pin);
	GPIO_ToggleBits(M1_EN_Port, M1_EN_Pin);

	GPIO_ToggleBits(M2_DIR_Port, M2_DIR_Pin);	
	GPIO_ToggleBits(M2_STP_Port, M2_STP_Pin);
	GPIO_ToggleBits(M2_EN_Port, M2_EN_Pin);

	GPIO_ToggleBits(M3_DIR_Port, M3_DIR_Pin);	
	GPIO_ToggleBits(M3_STP_Port, M3_STP_Pin);
	GPIO_ToggleBits(M3_EN_Port, M3_EN_Pin);

	GPIO_ToggleBits(M4_DIR_Port, M4_DIR_Pin);	
	GPIO_ToggleBits(M4_STP_Port, M4_STP_Pin);
	GPIO_ToggleBits(M4_EN_Port, M4_EN_Pin);	
*/	

	if(gb_var.wellcom_voice_delay_cnt > 0)
		{
		gb_var.wellcom_voice_delay_cnt --;
		if(gb_var.wellcom_voice_delay_cnt == 0)
			mp3_Play(V_Welcome);
		}



	if(gb_var.Undefined_Waste_Time_cnt > 0)
		{
		gb_var.Undefined_Waste_Time_cnt--;
		if(gb_var.Undefined_Waste_Time_cnt == 0)
			gb_var.Undefined_Waste_cnt = 0;
		}

}

void global_variable_init(void)
{

	ReadIni(&device_para_state);


	gb_var.timer_function_count=TIMERFUNC_0S5;
	gb_var.test_motor_dir = OPEN;
	gb_var.device_report_count = TIMERFUNC_1S * REPORT_TIME_MAX;
	gb_var.bReportData = FALSE;
	device_para_state.OPENDOOR_TIME = 0;
	gb_var.test1 = 0;
	//gb_var.test2 = 2;

	for(uint8_t i = 0; i < 4; i++)
		{
		overflow_value[i].data_sum = 0;
		overflow_value[i].filter_cnt = 0;
		overflow_value[i].data_out = 0;
		overflow_value[i].data_in = 0;

		motor[i].Direction = CLOSE;

		
		}

	gb_var.bUsrInitOK = FALSE;
	gb_var.SendInfoMode = send_report;
	//device_para_state.DEV_SN = 123456789;
	strcpy(device_para_state.LONGITUDE,"11400.0518,E");
	strcpy(device_para_state.LATITUDE,"2235.9058,N");
	//strcpy(device_para_state.DEV_SN,"2021051234");
	//device_para_state.LONGITUDE[] = "11400.0518,E";
	//device_para_state.LATITUDE[] = "2235.9058,N";
	
}

void mp3_Play(voice_pormpt_e tips)
{
	uint8_t sm=0;
	if(gb_var.bMp3_Busy == TRUE) return;
	//gb_var.bMp3_Busy = TRUE;
	//gb_var.Mp3_delay_cnt = 2000;
	/*
	USART_SendData(USART2, 0xAA);
	USART_SendData(USART2, 0x07);
	USART_SendData(USART2, 0x02);
	USART_SendData(USART2, 0x00);
	USART_SendData(USART2, tips);
	sm = 0xAA+0x07+0x02+0x00+tips;	////ºÍ¼ìÑé £ºÎªÖ®Ç°ËùÓÐ×Ö½ÚÖ®ºÍµÄµÍ 8 Î»,¼´ÆðÊ¼Âëµ½Êý¾ÝÏà¼ÓºóÈ¡µÍ 8 Î»¡£
	USART_SendData(USART2, sm);*/

	uint8_t buffer[6];
	buffer[0] = 0xAA;
	buffer[1] = 0x07;
	buffer[2] = 0x02;
	buffer[3] = 0x00;
	buffer[4] = tips;
	buffer[5] = 0xAA+0x07+0x02+0x00+tips;
	USART2_Send_Array( buffer, sizeof(buffer) );
}


void Motor_Operation(door_motor_e motor_x, motor_dir dir_x)
{
	//if(RUN == motor[motor_x].bMotor_operation)	return;///µç»úÕýÔÚÔËÐÐ
	
	motor[motor_x].bMotor_operation = RUN;
	motor[motor_x].Direction = dir_x;
	//motor[motor_x].Operation_count = MOTOR_WORK_TIME_MAX;
#ifdef USE_NEWMOTOR
	dir_x == OPEN ? GPIO_SetBits(motor_dir_port_tab[motor_x],motor_dir_pin_tab[motor_x]) : \
					GPIO_ResetBits(motor_dir_port_tab[motor_x],motor_dir_pin_tab[motor_x]);
	
	GPIO_ResetBits(motor_en_port_tab[motor_x],motor_en_pin_tab[motor_x]);
#else
	switch(motor_x)
		{
		case Motor_harmful:
			dir_x == OPEN ? M1_DIR_DN : M1_DIR_UP;///¿ª¸ÇÊÇ·´×ª
			M1_EN(RUN);
			break;
		case Motor_kitchen:
			dir_x == OPEN ? M2_DIR_DN : M2_DIR_UP;
			M2_EN(RUN);
			break;
		case Motor_recycle:
			dir_x == OPEN ? M3_DIR_DN : M3_DIR_UP;
			M3_EN(RUN);
			break;
		case Motor_other:
			dir_x == OPEN ? M4_DIR_DN : M4_DIR_UP;
			M4_EN(RUN);
			break;
		}

#endif

	

}

uint8_t MakeChk(uint8_t * dat, uint8_t len)
{
	uint16_t sum;
	sum = 0;

	for(uint8_t i = 0; i < len;i++)
		{
		sum += *dat;
		dat ++;
		}
	return ((uint8_t)sum);
}
void Func_SetParam(const uint8_t * dat, device_para_state_t para)
{
	device_setpara_t * dstlp = (device_setpara_t *)dat;
	para.OPENDOOR_TIME = dstlp->OPENDOOR_TIME;
	para.OVERFLOW_WRNING = dstlp->OVERFLOW_WRNING;
	para.PWRDN_DOOR_STATE = dstlp->PWRDN_DOOR_STATE;
}
#ifndef USE_TCP232
void Func_DoorCtrl(uint8_t * dat)
{

	device_door_ctrl_t * ddctlp = (device_door_ctrl_t *)dat;
	d_debug("Recive:%x",ddctlp->DROP_DOOR);

	for(uint8_t i = 0;i < MOTOR_NUM_MAX ;i++)
		{
		switch((ddctlp->DROP_DOOR >> (i * 2)) & 0x03)
			{
			case C_door_close:
				motor[i].bDoorKeepOpen = FALSE;
				Motor_Operation((door_motor_e)i , CLOSE);
				break;
			case C_door_open:
				motor[i].bDoorKeepOpen = FALSE;
				Motor_Operation((door_motor_e)i , OPEN);
				break;
		
			case C_door_keep_open:
				motor[i].bDoorKeepOpen = TRUE;
				Motor_Operation((door_motor_e)i , OPEN);
				break;
		
			case C_door_keep_state:
				motor[i].bDoorKeepOpen = FALSE;
				break;
			}

		}
}
#endif

void Func_SendACK(uint8_t * dat)
{
	uint8_t buffer[29];

	device_type_t *dtlp = (device_type_t *)(dat);
	device_send_ack_t * dsatlp = (device_send_ack_t *)buffer;
	
	memset(buffer,0,sizeof(buffer));
	dsatlp->SYS_NUM = DEV_ACK;
	memset(dsatlp->DEV_ID,0,sizeof(dsatlp->DEV_ID));
	dsatlp->DEV_CMD = dtlp->CMD_TYPE;
	
	dsatlp->ACK = 1;
	
	Start_Send_PC(buffer,sizeof(buffer));
}

void Func_SendState(device_para_state_t dps)
{
/*	uint8_t buffer[29];
	device_type_t * dtlp = (device_type_t *)buffer;
	device_report_t * drlp = (device_report_t *)dtlp->DATA;

	memset(buffer,0,sizeof(buffer));

	dtlp->SYS_NUM = DEV_ACK;
	memcpy(dtlp->DEV_ID,dps.DEV_ID,sizeof(dps.DEV_ID));
	dtlp->CMD_TYPE = DEV_REPORT;
	
	drlp->HARMFUL_STATE = device_para_state.OVERFLOW_VAL[Motor_harmful];
	drlp->KITCHEN_STATE = device_para_state.OVERFLOW_VAL[Motor_kitchen];
	drlp->RECYCLE_STATE = device_para_state.OVERFLOW_VAL[Motor_recycle];
	drlp->OTHER_STATE = device_para_state.OVERFLOW_VAL[Motor_other];
	drlp->VOLT = 0;

	Start_Send_PC(buffer,sizeof(buffer));*/
	gb_var.bReportData = FALSE;
}

void SendTest(void)
{
	uint8_t buffer[13];
	uint16_t i;
	

	buffer[0] = 0xAB;
	buffer[1] = 0x00;
	buffer[2] = 0x08;//³¤¶È
	buffer[3] = '1';
	buffer[4] = '2';
	buffer[5] = '3';
	buffer[6] = '4';
	buffer[7] = '|';
	buffer[8] = '0';
	buffer[9] = '|';
	buffer[10] = '0';
	i = CRC16_MODBUS(buffer+3,buffer[0]);
	buffer[11] = HIWORD(i);
	buffer[12] = LOWORD(i);
	esp_send_data(0,buffer,sizeof(buffer));
}

//23 5F 0A 00 00 00 00 07 00 00 AA 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 BB
//23 5F 0A 00 00 00 00 07 00 00 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 CC
void Receive_UART3(uint8_t * dat)
{
	typedef struct{
		uint8_t HEAD_0;		///0
		uint8_t HEAD_1;		///1
		uint8_t SYS_NUM;	///2
		uint8_t ID[4];		///3-6
		uint8_t CMD_TYPE;	///7
		uint8_t COUNTER_DOOR;//8
		uint8_t CLEAR_DOOR;	///9
		uint8_t DROP_DOOR;	///10
		uint8_t NoUse[20];
		uint8_t CHKSUM;
	}cmd_t;
	uint8_t chkbyte;
	cmd_t * ct = (cmd_t *)dat;

	if((HEAD0 != ct->HEAD_0) || (HEAD1 != ct->HEAD_1) ||(DEV_TYPE != ct->SYS_NUM) ||(7 != ct->CMD_TYPE))goto RTRECEIVE3;
	chkbyte = MakeChk(dat + 2,29);
	if(chkbyte != ct->CHKSUM) goto RTRECEIVE3;

	if(ct->DROP_DOOR == 0xAA)
		{
		motor[Motor_harmful].bDoorKeepOpen = TRUE;
		Motor_Operation(Motor_harmful , OPEN);
		
		motor[Motor_kitchen].bDoorKeepOpen = TRUE;
		Motor_Operation(Motor_kitchen , OPEN);
		
		motor[Motor_recycle].bDoorKeepOpen = TRUE;
		Motor_Operation(Motor_recycle , OPEN);
		
		motor[Motor_other].bDoorKeepOpen = TRUE;
		Motor_Operation(Motor_other , OPEN);

		}
	else
		{
		motor[Motor_harmful].bDoorKeepOpen = FALSE;
		Motor_Operation(Motor_harmful , CLOSE);
		
		motor[Motor_kitchen].bDoorKeepOpen = FALSE;
		Motor_Operation(Motor_kitchen , CLOSE);
		
		motor[Motor_recycle].bDoorKeepOpen = FALSE;
		Motor_Operation(Motor_recycle , CLOSE);
		
		motor[Motor_other].bDoorKeepOpen = FALSE;
		Motor_Operation(Motor_other , CLOSE);
		}
	
RTRECEIVE3:
	gb_var.rx3_have_data = 0;
}

void Receive_Net(uint8_t * dat)
{
	device_type_t *dtlp = (device_type_t *)(dat);
	uint16_t crc;
	if(SEV_CODE != dtlp->SYS_NUM) goto RTRECEIVE;

	switch(dtlp->CMD_TYPE)
		{
		case dev_hwinfo:
			{
			device_setpara_t *dst = (device_setpara_t *)dtlp->DATA;
			i2b_def len ;
			len.b[0] = dtlp->D_LEN[3];
			len.b[1] = dtlp->D_LEN[2];
			len.b[2] = dtlp->D_LEN[1];
			len.b[3] = dtlp->D_LEN[0];
			//uint32_t len = dtlp->D_LEN.x;
			crc = CRC16_MODBUS( dtlp->DATA , len.x);
			
			if( MAKEWORD(dst->crc_val[2],dst->crc_val[3]) != crc)	goto RTRECEIVE;

			device_para_state.OPENDOOR_TIME = dst->OPENDOOR_TIME;
			device_para_state.PWRDN_DOOR_STATE = dst->PWRDN_DOOR_STATE;
			device_para_state.OVERFLOW_WRNING = dst->OVERFLOW_WRNING;

			gb_var.bTcpConnectOK = TRUE;
			led_tcp_connected();
			mp3_Play(V_TipsA);
			}
			break;
		case dev_opendoor:
			{
			device_door_ctrl_t *ddct = (device_door_ctrl_t *)dtlp->DATA;
			//static uint8_t Undefined_Waste_cnt = 0;
			
			i2b_def len ;
			len.b[0] = dtlp->D_LEN[3];
			len.b[1] = dtlp->D_LEN[2];
			len.b[2] = dtlp->D_LEN[1];
			len.b[3] = dtlp->D_LEN[0];
			crc = CRC16_MODBUS( dtlp->DATA , len.x);
			
			if( MAKEWORD(ddct->crc_val[2],ddct->crc_val[3]) != crc)	goto RTRECEIVE;

			switch(ddct->DOOR_CTRL)
				{
				case Harmful_Waste:
					motor[Motor_harmful].bDoorKeepOpen = FALSE;
					Motor_Operation(Motor_harmful , OPEN);
					gb_var.Undefined_Waste_cnt = 0;
					break;
				case Kitchen_Waste:
					motor[Motor_kitchen].bDoorKeepOpen = FALSE;
					Motor_Operation(Motor_kitchen , OPEN);
					gb_var.Undefined_Waste_cnt = 0;
					break;
				case Recycle_Waste:
					motor[Motor_recycle].bDoorKeepOpen = FALSE;
					Motor_Operation(Motor_recycle , OPEN);
					gb_var.Undefined_Waste_cnt = 0;

					break;
				case Other_Waste:
					motor[Motor_other].bDoorKeepOpen = FALSE;
					Motor_Operation(Motor_other , OPEN);
					gb_var.Undefined_Waste_cnt = 0;

					break;
				case Undefined_Waste:
					gb_var.Undefined_Waste_cnt ++;
					gb_var.test1++;
					gb_var.Undefined_Waste_Time_cnt = 10;
					if(4 == gb_var.Undefined_Waste_cnt)
						{
						motor[Motor_other].bDoorKeepOpen = FALSE;
						Motor_Operation(Motor_other , OPEN);
						gb_var.Undefined_Waste_cnt = 0;
						gb_var.Undefined_Waste_Time_cnt = 0;
						}
					break;
				}
			}
			break;
		case dev_keepopendoor:
			{
			device_keep_open_t *dnct = (device_keep_open_t *)dtlp->DATA;
			i2b_def len ;
			len.b[0] = dtlp->D_LEN[3];
			len.b[1] = dtlp->D_LEN[2];
			len.b[2] = dtlp->D_LEN[1];
			len.b[3] = dtlp->D_LEN[0];
			crc = CRC16_MODBUS( dtlp->DATA , len.x);
			
			if( MAKEWORD(dnct->crc_val[2],dnct->crc_val[3]) != crc)	goto RTRECEIVE;		

			switch(dnct->DOOR_STATE)
				{
				case C_door_close:
					motor[Motor_harmful].bDoorKeepOpen = FALSE;
					Motor_Operation(Motor_harmful , CLOSE);

					motor[Motor_kitchen].bDoorKeepOpen = FALSE;
					Motor_Operation(Motor_kitchen , CLOSE);

					motor[Motor_recycle].bDoorKeepOpen = FALSE;
					Motor_Operation(Motor_recycle , CLOSE);

					motor[Motor_other].bDoorKeepOpen = FALSE;
					Motor_Operation(Motor_other , CLOSE);					
					break;

				case C_door_keep_open:
					motor[Motor_harmful].bDoorKeepOpen = TRUE;
					Motor_Operation(Motor_harmful , OPEN);

					motor[Motor_kitchen].bDoorKeepOpen = TRUE;
					Motor_Operation(Motor_kitchen , OPEN);

					motor[Motor_recycle].bDoorKeepOpen = TRUE;
					Motor_Operation(Motor_recycle , OPEN);

					motor[Motor_other].bDoorKeepOpen = TRUE;
					Motor_Operation(Motor_other , OPEN);

					break;
				}
			
			}
			break;
		case dev_new_connect:
			{
			device_new_connect_t *dnct = (device_new_connect_t *)dtlp->DATA;
			i2b_def len ;
			len.b[0] = dtlp->D_LEN[3];
			len.b[1] = dtlp->D_LEN[2];
			len.b[2] = dtlp->D_LEN[1];
			len.b[3] = dtlp->D_LEN[0];
			crc = CRC16_MODBUS( dtlp->DATA , len.x);
			
			if( MAKEWORD(dnct->crc_val[2],dnct->crc_val[3]) != crc)	goto RTRECEIVE;

			if(0xff == dnct->NEW_CONNECT)
				{
				//gb_var.bSendLogonInfo = TRUE;
				gb_var.SendInfoMode = send_logon;
				}
			}
			break;
		}


RTRECEIVE:
	gb_var.rx1_have_data = 0;
}

uint16_t filter(filter_data_t * lp, uint16_t val)
{
	lp->data_in = val;
	lp->data_sum += lp->data_in;
	lp->filter_cnt ++;
	if(lp->filter_cnt < FILTER_N)
		{
		lp->data_out = lp->data_sum / lp->filter_cnt;

		}
	else
		{
		lp->filter_cnt = FILTER_N;
		lp->data_sum -= lp->data_out;
		lp->data_out = lp->data_sum / lp->filter_cnt;
		}

	return lp->data_out;
}

uint16_t CRC16_MODBUS(char *buffer, int len) 
	{    
	int wCRCin = 0xffff;    
	int POLYNOMIAL = 0xa001;  
	int i;
	for (i = 0; i < len; i++) {        
		//wCRCin ^= ((int) b & 0x00ff);        
		wCRCin ^= ((unsigned int)buffer[i]); 
		for (int j = 0; j < 8; j++) 
			{            
			if ((wCRCin & 0x0001) != 0) 
				{                
				wCRCin >>= 1;                
				wCRCin ^= POLYNOMIAL;            
				} else 
				{                
				wCRCin >>= 1;            
				}        
			}    
		}    
	return wCRCin ^= 0x0000;
	}


//mode=·¢Êý¾ÝÀàÐÍ£¬=1µÇÂ¼ÐÅÏ¢
void SendInfo(uint8_t mode, device_para_state_t dps)
{
	uint8_t buffer[128];
	i2b_def len;
	memset(buffer,0,sizeof(buffer));
	buffer[0] = DEV_CODE;
	switch( mode )
		{
		case send_logon:////Éè±¸×¢²á£¬ÓÉÉè±¸±àºÅ¡¢¾­¶È¡¢Î¬¶È¹¹³ÉµÄ×Ö·û´®£¬¸ñÊ½£ºSN|¾­¶È|Î³¶È
			{
			uint8_t buf[64];
			uint16_t crc;
			
			memset(buf,0,sizeof(buf));
			
			buffer[1] = send_logon;//Éè±¸×¢²áÐÅÏ¢

			sprintf(( char* )buf,"%s|%s|%s",dps.DEV_SN, dps.LONGITUDE, dps.LATITUDE);
			len.x = strlen(buf);
			
			//buffer[2] = HIWORD(len);
			//buffer[3] = LOWORD(len);

			buffer[2] = len.b[3];
			buffer[3] = len.b[2];
			buffer[4] = len.b[1];
			buffer[5] = len.b[0];

			strlcpy(buffer+6,buf,len.x +1);

			crc = CRC16_MODBUS(buf,len.x);

			buffer[len.x + 6] = 0;
			buffer[len.x + 7] = 0;
			buffer[len.x + 8] = HIBYTE(crc);
			buffer[len.x + 9] = LOBYTE(crc);

			}

			break;
		case send_report:
			uint8_t buf[64];
			uint8_t data[16];
			uint16_t crc;
			
			memset(buf,0,sizeof(buf));
			memset(buf,0,sizeof(data));
			
			buffer[1] = send_report;//Éè±¸×¢²áÐÅÏ¢
			
			//sprintf(( char* )buf,"%s|%s|%s",dps.DEV_SN, dps.LONGITUDE, dps.LATITUDE);
			device_report_t * drt = (device_report_t *)buf;
			//drt->SN = 
			sprintf(( char* )drt->SN,"%s",dps.DEV_SN);
			drt->FLG = '|';
			drt->KITCHEN_STATE = (dps.OVERFLOW_VAL[Motor_kitchen] < OTHER_OVER_LINE) ? '0' : '1';
			drt->RECYCLE_STATE = (dps.OVERFLOW_VAL[Motor_recycle] < OTHER_OVER_LINE) ? '0' : '1';
#ifdef USE_BTN_OPEN
			drt->HARMFUL_STATE = (dps.OVERFLOW_VAL[Motor_harmful] < OTHER_OVER_LINE) ? '0' : '1';
#else
			drt->HARMFUL_STATE = (dps.OVERFLOW_VAL[Motor_harmful] < HARMFUL_OVER_LINE) ? '0' : '1';
#endif
			drt->OTHER_STATE =	 (dps.OVERFLOW_VAL[Motor_other] < OTHER_OVER_LINE) ? '0' : '1';

			//sprintf(( char* )buf,"%s|%x",dps.DEV_SN, (uint8_t *)data);
			
			len.x = sizeof(device_report_t);
			
			//buffer[2] = HIWORD(len);
			//buffer[3] = LOWORD(len);

			buffer[2] = len.b[3];
			buffer[3] = len.b[2];
			buffer[4] = len.b[1];
			buffer[5] = len.b[0];

			strlcpy(buffer+6,buf,len.x +1);

			crc = CRC16_MODBUS(buf,len.x);

			buffer[len.x + 6] = 0;
			buffer[len.x + 7] = 0;
			buffer[len.x + 8] = HIBYTE(crc);
			buffer[len.x + 9] = LOBYTE(crc);			
			break;
		}
	esp_send_data(0,buffer,len.x + 10);
	gb_var.SendInfoMode = no_data;
}

void WriteIni(const device_para_state_t *dev)//±£´æÖ÷»úµØÖ·µÈ²ÎÊý
{ 
	uint8_t Buf[64];
	FlashReadStr(FLASH_ADDR,INITLEN,Buf);

	Buf[0]=0xaa;Buf[1]=0x55;
	memcpy(Buf+2, dev->DEV_SN, sizeof(dev->DEV_SN));
	//Buf[6]=0;Buf[7]=0;

	FlashWriteStr(FLASH_ADDR,INITLEN,Buf);
   
}

void ReadIni(device_para_state_t *dev)
{
	uint8_t Buf[64],i;
 	FlashReadStr(FLASH_ADDR,INITLEN,Buf);

	if ((Buf[0]==0xAA)&&(Buf[1]==0x55))
	{
		memcpy(dev->DEV_SN ,Buf+2, sizeof(dev->DEV_SN));
		
   }
   else
   {
		strcpy(dev->DEV_SN,"2021050001");
	 //WriteIni();//2011.10.31
   }
 
}


int main(void)
{
	SystemInit();

	GPIO_Configuration();
	
	USART1_Configuration(115200);
	USART2_Configuration(9600);
	USART3_Configuration(115200);
	TIMER_Configuration();
	ADC_Configuration();
	
	SysTick_Config(SystemFrequency / 1000);	
	global_variable_init();
	led_tcp_disconnected();
	gb_var.wellcom_voice_delay_cnt = 4;

	while(1)
	{
	USR_INIT();
	//if(TRUE == gb_var.bChkTcp) Check_Tcp();
	if(TRUE == gb_var.bKeyScan) KeyScan();
	if(TRUE == gb_var.bBtnScan) BtnScan();
	if(TRUE == gb_var.bBtnOk) BtnOpenDoor();

	if(TRUE == gb_var.bTimerFunc) Timer_500ms();
	
	if(3 == gb_var.rx1_have_data) Receive_Net(gb_var.rx_buffer);
	if(3 == gb_var.rx2_have_data) Receive_Manage_Info(gb_var.rx2_buffer);
	if(3 == gb_var.rx3_have_data) Receive_UART3(gb_var.rx3_buffer);

	if(no_data != gb_var.SendInfoMode) SendInfo(gb_var.SendInfoMode, device_para_state);
	if(no_data != gb_var.SendManageMode) Manage_Info_Ack(gb_var.SendManageMode, device_para_state);

	/**/	
	for(uint8_t i =0 ;i < MOTOR_NUM_MAX ; i++)
		{
		if(RUN == motor[i].bMotor_operation)
			GPIO_ToggleBits(motor_stp_port_tab[i], motor_stp_pin_tab[i]);///µç»úÇý¶¯Âö³å
		}

	//GPIO_ToggleBits(L_TCP_Port , L_TCP_Pin);	
	uint8_t i,j;
	uint32_t sum;

	for(i=0;i<4;i++)
		{			
		uint16_t value[MOTOR_NUM_MAX];
		sum=0;			
		for(j=0;j<10;j++)			
			{				
			sum+=gb_var.ADCConvertedValue[j][i];			
			}			
		value[i]=(uint16_t)((float)sum/(10*4096)*3300);//ÇóÆ½¾ùÖµ²¢×ª»»³ÉµçÑ¹Öµ	
		gb_var.ADC_Value[i] = filter(&overflow_value[i],value[i]);

		device_para_state.OVERFLOW_VAL[i] = (gb_var.ADC_Value[i] );
		
		}

	}

}
