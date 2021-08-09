/*******************************************************************************
* Copyright (C), 2000-2018,  Sunsky Electronic Technology Co., Ltd.
* 文件名: led.c
* 作  者: 杨学军
* 版  本: V1.0
* 日  期: 2018-12-10
* 说  明:
*
*
* 修订历史: STM32 3.5
*
*    1. 时间: 2018-12-10
*       修订者: 杨学军(728920175@qq.com)
*       修订内容: 创建
*    2.
* 其它:
*******************************************************************************/
//#include "base.h"
#include "main.h"
#include "led.h"
/**/
#ifndef LED_APB2Periph_CLK
#define LED_APB2Periph_CLK RCC_APB2Periph_GPIOB
#endif
/*
#ifndef L_NET_Pin
#define L_NET_Pin	GPIO_Pin_8
#define L_NET_Port	GPIOB

#endif

#ifndef L_RF_Pin
#define L_RF_Pin	GPIO_Pin_9
#define L_RF_Port	GPIOB

#endif
*/
#ifndef L_SYS_Pin
#define L_SYS_Pin	GPIO_Pin_13
#define L_SYS_Port	GPIOB

#endif



const uint16_t lled_pin[] = {
    //L_NET_Pin,  
	//L_RF_Pin,  
	L_TCP_Pin
};

GPIO_TypeDef*  lled_port[] = {
    //L_NET_Port, 
	//L_RF_Port, 
	L_TCP_Port
};

uint32_t led_overtime[MAX_LLED_COUNT]; // 0值不控制熄灯

uint32_t led_ontime[MAX_LLED_COUNT];//亮灯时间
uint32_t led_offtime[MAX_LLED_COUNT];//灭灯时间
uint32_t led_oncnt[MAX_LLED_COUNT];//亮灯计数
uint32_t led_offcnt[MAX_LLED_COUNT];//灭灯计数

uint32_t led_flashcnt[MAX_LLED_COUNT];//=0不闪,  =n亮次数
void hw_led_init( void )
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( LED_APB2Periph_CLK, ENABLE );

	////L_NET_Pin,L_RF_Pin,L_SYS_Pin
	//GPIO_InitStructure.GPIO_Pin = L_NET_Pin|L_RF_Pin|L_SYS_Pin;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(L_NET_Port, &GPIO_InitStructure);

    for( uint8_t i = 0; i < MAX_LLED_COUNT; i++ ) {
        GPIO_InitStructure.GPIO_Pin     = lled_pin[i];
        GPIO_Init( lled_port[i], &GPIO_InitStructure );

        hw_led_off( ( show_led_t )i );
        //hw_led_on( ( show_led_t )i, 15);
    }
}

inline
void hw_led_cpl( show_led_t id, uint16_t len)//
{
    if( id < MAX_LLED_COUNT ) {
        lled_port[id]->ODR ^= lled_pin[id];
        led_overtime[id] = len;
    }
}

inline
void hw_led_fls( show_led_t id, uint16_t onT, uint16_t offT, uint16_t fls)//fls=255一直闪烁
{
    if( id < MAX_LLED_COUNT ) {
        //lled_port[id]->ODR ^= lled_pin[id];
        hw_led_on( id , 0 );
		led_flashcnt[id] = fls;
		if( led_flashcnt[id] > 0)
			{
			led_ontime[id] = onT;
			led_oncnt[id] = onT;
			led_offtime[id] = offT;
			led_offcnt[id] = offT;
			if(led_flashcnt[id]!=AlWAYS_FLASH)
			led_flashcnt[id]--;
			}
    }
}

void hw_led_fls_off( show_led_t id)//
{
    if( id < MAX_LLED_COUNT ) {
        //lled_port[id]->ODR ^= lled_pin[id];
        hw_led_off( id );
		led_ontime[id] = 0;
		led_oncnt[id] = 0;
		led_offtime[id] = 0;
		led_offcnt[id] = 0;
		led_flashcnt[id] = 0;


    }
}


inline
void hw_led_on( show_led_t id, uint16_t len )////len=0常亮
{
    if( id < MAX_LLED_COUNT ) {
        lled_port[id]->BRR = lled_pin[id];
        led_overtime[id] = len;
    }
}

inline
void hw_led_off( show_led_t id )
{
    if( id < MAX_LLED_COUNT ) {
        lled_port[id]->BSRR = lled_pin[id];
    }
}

void hw_leds_off( void )
{
    for( uint8_t i=0; i < MAX_LLED_COUNT; i++ ) {
        lled_port[i]->BSRR = lled_pin[i];
    }
}
