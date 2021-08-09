/*******************************************************************************
* Copyright (C), 2000-2018,  Sunsky Electronic Technology Co., Ltd.
* �ļ���: led.h
* ��  ��: ��ѧ��
* ��  ��: V1.0
* ��  ��: 2018-12-10
* ˵  ��:
*
*
* �޶���ʷ: STM32 3.5
*
*    1. ʱ��: 2018-12-10
*       �޶���: ��ѧ��(728920175@qq.com)
*       �޶�����: ����
*    2.
* ����:
*******************************************************************************/
#ifndef __LED_H__
#define __LED_H__
#include "stdint.h"

#define MAX_LLED_COUNT 1
#define AlWAYS_FLASH		255///������˸

typedef enum {
    //LED_NET = 0,
    //LED_RF,
    LED_SYS,
} show_led_t;

extern uint32_t led_overtime[];
extern uint32_t led_flashcnt[];
extern uint32_t led_ontime[];//����ʱ��
extern uint32_t led_offtime[];//���ʱ��
extern uint32_t led_oncnt[];//���Ƽ���
extern uint32_t led_offcnt[];//��Ƽ���

void hw_led_init( void ); //��ʼ��
void hw_led_cpl( show_led_t id, uint16_t len );
void hw_led_on( show_led_t id, uint16_t len );
void hw_led_off( show_led_t id );
void hw_leds_off( void );
void hw_led_fls( show_led_t id, uint16_t onT, uint16_t offT, uint16_t fls);
void hw_led_fls_off( show_led_t id);
/*
#define led_net_no_sever()			hw_led_fls( LED_NET, 50 , 2000 , AlWAYS_FLASH)
#define led_net_sever_ok()			hw_led_fls( LED_NET, 500, 500  , AlWAYS_FLASH)//hw_led_fls_off(LED_NET)
#define led_net_client_connect()	hw_led_fls( LED_NET, 50 , 1950 , AlWAYS_FLASH)
#define led_net_receive_data()		hw_led_on(LED_NET,50)

#define led_rf_client_connect()		hw_led_fls( LED_RF, 500 , 1500 , AlWAYS_FLASH)
#define led_rf_receive_data()		hw_led_on(LED_RF,20)
#define led_rf_no_client_connect()	hw_led_fls_off(LED_RF)
*/
#define led_tcp_disconnected()	hw_led_fls( LED_SYS, 50 , 2000 , AlWAYS_FLASH)
#define led_tcp_connecting()	hw_led_fls( LED_SYS, 100 , 100 , AlWAYS_FLASH)
#define led_tcp_connected()		hw_led_fls( LED_SYS, 1000 , 1000 , AlWAYS_FLASH)

#endif
