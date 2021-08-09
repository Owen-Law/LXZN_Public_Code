#include "USR_TCP232.h"
#include "led.h"

uint8_t  Tx1_Buf[TX1BUF_SIZE];
uint8_t  Tx1_flag;
uint8_t  Rx1_Buf[RX1BUF_SIZE];
uint16_t Rx1_Count;
uint8_t  Rx1_flag;
uint16_t Rx1_to;
uint16_t Rx1_to_len;

usr_init_t usr_curr_status = USR_GPIO;
usr_init_t usr_delay_status; // delay
usr_init_t usr_cipmode_status; //

uint16_t usr_tick_delay; // 1ms

usr_init_t usr_chk_curr_status = USR_CIPMODE_EXIT1;


void buff_clear( void )
{
    FLAG_ESP_RECV = 0;
    POINT_ESP_RECV = 0;
    memset( BUFF_ESP_RECV, 0, SIZE_ESP_RECV );
}



void USART1_printf( char* fmt, ... )
{
#if USARTy_DMA==1
    while( 1 == Tx1_flag );
    va_list arg_ptr;
    va_start( arg_ptr, fmt );
    vsnprintf( ( char* )Tx1_Buf, TX1BUF_SIZE + 1, fmt, arg_ptr );
    va_end( arg_ptr );

    DMA_USARTy_Tx_Data( strlen( ( char* )Tx1_Buf ) );

#else
    uint16_t i = 0;
    va_list arg_ptr;
    va_start( arg_ptr, fmt );
    vsnprintf( ( char* )Tx1_Buf, TX1BUF_SIZE + 1, fmt, arg_ptr );
    va_end( arg_ptr );
//	if(esp_curr_status==ESP_CONFIG07)
//		NOP();
    while( ( i < TX1BUF_SIZE ) && Tx1_Buf[i] ) {
        USART_SendData( USR_USART, ( uint8_t ) Tx1_Buf[i++] );
        while( USART_GetFlagStatus( USR_USART, USART_FLAG_TXE ) == RESET );
    }
#endif
}

void hw_usr_reset(void)
{
	USR_CFG_UP;////正常工作时请将此引脚悬空或接高电平，可使用串口进行模块参
				//数配置。配置时先上电，再拉低 Reload 引脚，以进入串口配置状
				//态。配置完成后拉高或者悬空即可。

	USR_REST_UP;
	
	USR_REST_DN;
	Delay_ms(300);
	USR_REST_UP;
}

void hw_usr_init(void)
{


}


void recv_process( uint8_t dt )
{
	//if(FALSE == gb_var.bTcpConnectOK)	return;
	if(FALSE == gb_var.bUsrInitOK)	return;
	if(gb_var.rx1_have_data == 0)
		{
		gb_var.rx1_count=0;
		memset(gb_var.rx_buffer, 0, sizeof(gb_var.rx_buffer));
		}
	
	gb_var.rx_buffer[gb_var.rx1_count]=dt;
	//USART_ClearITPendingBit( USART1, USART_IT_RXNE );
	gb_var.rx1_timeout = USART1__TIMEOUT;
	gb_var.rx1_count ++;
	gb_var.rx1_have_data = 1;


}

void Start_Check_Tcp(void)
{
	usr_chk_curr_status = USR_CIPMODE_EXIT1;
	gb_var.bChkTcp = TRUE;
	//gb_var.bTcpConnectOK = FALSE;///断开连接
}

void Check_Tcp(void)///检查tcp连接,执行时间1.5s
{
	//GPIO_SetBits(TP2_Port , TP2_Pin);
	switch( usr_chk_curr_status )
		{
		case USR_DELAY:
            if( usr_tick_delay == 0 ) {
                usr_chk_curr_status = usr_delay_status;
                FLAG_ESP_RECV = 0;
            }
			break;		
		case USR_CIPMODE_EXIT1:///退出透传模式
			buff_clear();

			USR_SendString( "+++" );
			usr_chk_curr_status = USR_CIPMODE_EXIT2;
			usr_tick_delay = USR_DELAY_COMMAND;
			break;
		case USR_CIPMODE_EXIT2:///接收"a"，确认消息
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_chk_curr_status = USR_CIPMODE_EXIT1;
				//usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "a" ) ) {
						d_debug("Send(+++) is OK!\r\n");
						
						usr_delay_status = USR_CIPMODE_EXIT3;
						usr_chk_curr_status = USR_DELAY;
						usr_tick_delay = USR_DELAY_COMMAND;
					}
				}
			}
		
			break;
		case USR_CIPMODE_EXIT3:///发送"a"，确认退出透传(当设备接收‘a’后，必须在 3 秒内给模块发送一个‘a’。)
			buff_clear();
		
			USR_SendString( "a" );
			usr_chk_curr_status = USR_CIPMODE_EXIT4;
			usr_tick_delay = USR_DELAY_COMMAND;
		
			break;
		case USR_CIPMODE_EXIT4:///接收"+OK"指令，退出透传并进入“AT 指令模式
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_chk_curr_status = USR_CIPMODE_EXIT1;
				//usr_curr_status = USR_DELAY;
				usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "+ok" ) ) {
						d_debug("Send(a) is OK!\r\n");
						
						usr_delay_status = USR_CONFIG01;
						usr_chk_curr_status = USR_DELAY;
						usr_tick_delay = USR_DELAY_COMMAND;
					}
				}
			}
		
			break;
		case USR_CONFIG01:
			buff_clear();
		
			USR_SendString("AT+SOCKLK\r\n");
		
			usr_chk_curr_status = USR_CONFIG02;
			usr_tick_delay = USR_DELAY_COMMAND; 		
			break;
		case USR_CONFIG02:
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_chk_curr_status = USR_CONFIG01;
				//usr_curr_status = USR_DELAY;
				usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "+OK" ) ) {
						d_debug("AT+SOCKLK is OK!\r\n");

						if( strstr( ( char* )BUFF_ESP_RECV, "disconnect" ) )///tcp连接断开
							{
							gb_var.bTcpConnectOK = FALSE;
							led_tcp_disconnected();
							gb_var.tcp_chk_connect_delay_cnt = TCP_RECHK_TIME;
							}
						else
							{
							if(FALSE == gb_var.bTcpConnectOK)
								gb_var.tcp_send_logoninfo_cnt = TCP_SEND_LOGINFO_DELY;
							gb_var.bTcpConnectOK = TRUE;
							led_tcp_connected();
							}
						
						usr_delay_status = USR_CONFIG03;
						usr_chk_curr_status = USR_DELAY;
						usr_tick_delay = USR_DELAY_COMMAND;

					}
				}
			}
			break;

		case USR_CONFIG03:
			buff_clear();

			USR_SendString("AT+ENTM\r\n");
			usr_chk_curr_status = USR_CONFIG04;
			//usr_curr_status = USR_DELAY;
			usr_tick_delay = USR_DELAY_COMMAND;		
			break;
		case USR_CONFIG04:
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_delay_status = USR_CONFIG01;
				usr_chk_curr_status = USR_DELAY;
				usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "+OK" ) ) {
						d_debug("AT+ENTM is OK!\r\n");
						
						buff_clear();
						//gb_var.bUsrInitOK = TRUE;
						//Start_Check_Tcp();
						//led_tcp_connected();
						gb_var.bChkTcp = FALSE;
						//GPIO_ResetBits(TP2_Port , TP2_Pin);

					}
				}
			}

			
		}
	
}



void USR_INIT( void )
{
    char* s1;
    static uint8_t login_cnt = 0;
    static usr_init_t usr_prev_status;

	if(TRUE == gb_var.bUsrInitOK) return;

    if( usr_prev_status != usr_curr_status ) {
        usr_prev_status = usr_curr_status;
        //show_connect_proc();
    }


	switch( usr_curr_status )
		{
		case USR_GPIO:
			
			USR_CFG_UP;	//CFG(Reload)正常工作时请将此引脚悬空或接高电平
			USR_REST_DN;///引脚收到 200ms 低电平后复位模块
			
            usr_delay_status = USR_HW_RESET1;
            usr_curr_status = USR_DELAY;
            usr_tick_delay = 200; //500;   
            #if 1
			device_para_state.IPADDR[0]=47;
			device_para_state.IPADDR[1]=115;
			device_para_state.IPADDR[2]=159;
			device_para_state.IPADDR[3]=137;
#ifdef USE_TEST_BKSTG
			device_para_state.NETPORT = 19999;

#else
			device_para_state.NETPORT = 9999;
#endif
			#else
			device_para_state.IPADDR[0]=192;
			device_para_state.IPADDR[1]=168;
			device_para_state.IPADDR[2]=0;
			device_para_state.IPADDR[3]=110;
			device_para_state.NETPORT = 9999;
			#endif

			led_tcp_connecting();
			break;
		case USR_DELAY:
            if( usr_tick_delay == 0 ) {
                usr_curr_status = usr_delay_status;
                FLAG_ESP_RECV = 0;
            }

			break;
		case USR_HW_RESET1:
			USR_REST_UP;
            usr_curr_status = USR_CIPMODE_EXIT1;
            TIMEOVER_RECV = 150;
            usr_tick_delay = 5000;
		
			break;
		case USR_SOFT_RESET:
			
			break;
		case USR_CIPMODE_EXIT1:///退出透传模式
			buff_clear();

			USR_SendString( "+++" );
			usr_curr_status = USR_CIPMODE_EXIT2;
			usr_tick_delay = USR_DELAY_COMMAND;

			break;
		case USR_CIPMODE_EXIT2:///接收"a"，确认消息
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_curr_status = USR_CIPMODE_EXIT1;
				//usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "a" ) ) {
						d_debug("Send(+++) is OK!\r\n");
						
						usr_delay_status = USR_CIPMODE_EXIT3;
						usr_curr_status = USR_DELAY;
						usr_tick_delay = USR_DELAY_COMMAND;
					}
				}
			}

			break;
		case USR_CIPMODE_EXIT3:///发送"a"，确认退出透传(当设备接收‘a’后，必须在 3 秒内给模块发送一个‘a’。)
			buff_clear();

			USR_SendString( "a" );
			usr_curr_status = USR_CIPMODE_EXIT4;
			usr_tick_delay = USR_DELAY_COMMAND;

			break;
		case USR_CIPMODE_EXIT4:///接收"+OK"指令，退出透传并进入“AT 指令模式
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_curr_status = USR_CIPMODE_EXIT1;
				//usr_curr_status = USR_DELAY;
				usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "+ok" ) ) {
						d_debug("Send(a) is OK!\r\n");
						
						usr_delay_status = USR_CONFIG01;
						usr_curr_status = USR_DELAY;
						usr_tick_delay = USR_DELAY_COMMAND;
					}
				}
			}

			break;
		case USR_CONFIG01:
			buff_clear();

			USR_SendString("AT+SOCK=TCPC,%d.%d.%d.%d,%d\r\n",
			device_para_state.IPADDR[0],
			device_para_state.IPADDR[1],
			device_para_state.IPADDR[2],
			device_para_state.IPADDR[3],
			device_para_state.NETPORT);

			usr_curr_status = USR_CONFIG02;
			usr_tick_delay = USR_DELAY_COMMAND;			
			break;

		case USR_CONFIG02:
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_curr_status = USR_CONFIG01;
				//usr_curr_status = USR_DELAY;
				usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "+OK" ) ) {
						d_debug("IPADDR is OK!\r\n");
						
						usr_delay_status = USR_CONFIG03;
						usr_curr_status = USR_DELAY;
						usr_tick_delay = USR_DELAY_COMMAND;
					}
				}
			}

			break;
		case USR_CONFIG03:
			buff_clear();

			USR_SendString("AT+WANN=DHCP,192.168.5.7,255.255.255.0,192.168.5.1\r\n");//设置dhcp模式
			usr_curr_status = USR_CONFIG04;
			//usr_curr_status = USR_DELAY;
			usr_tick_delay = USR_DELAY_COMMAND;		
			break;
		case USR_CONFIG04:
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_delay_status = USR_CONFIG01;
				usr_curr_status = USR_DELAY;
				usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "+OK" ) ) {
						d_debug("set dhcp is OK!\r\n");

						/*
						buff_clear();
						gb_var.bUsrInitOK = TRUE;
						//Start_Check_Tcp();
						//led_tcp_connected();
						led_tcp_disconnected();
						gb_var.tcp_chk_connect_delay_cnt = 2000;*/
						usr_delay_status = USR_CONFIG05;
						usr_curr_status = USR_DELAY;
						usr_tick_delay = USR_DELAY_COMMAND;
					}
				}
			}


			break;
		case USR_CONFIG05:
			buff_clear();
		
			USR_SendString("AT+Z\r\n");//复位
			usr_curr_status = USR_CONFIG06;
			//usr_curr_status = USR_DELAY;
			usr_tick_delay = USR_DELAY_COMMAND; 	
			break;
		case USR_CONFIG06:
			if( usr_tick_delay == 0 ) {
				// wait 5sec done
				usr_delay_status = USR_CONFIG01;
				usr_curr_status = USR_DELAY;
				usr_tick_delay = USR_DELAY_COMMAND;
			} else {
				// check "OK"
				if( FLAG_ESP_RECV == 1 ) {
					FLAG_ESP_RECV = 0;
					if( strstr( ( char* )BUFF_ESP_RECV, "+OK" ) ) {
						d_debug("Reset is OK!\r\n");
		
						/**/
						buff_clear();
						gb_var.bUsrInitOK = TRUE;
						
						//Start_Check_Tcp();
						//led_tcp_connected();
						led_tcp_disconnected();
						//gb_var.tcp_chk_connect_delay_cnt = TCP_RECHK_TIME;
						//gb_var.bTcpConnectOK = TRUE;
						//usr_delay_status = USR_CONFIG05;
						//usr_curr_status = USR_DELAY;
						//usr_tick_delay = USR_DELAY_COMMAND;
					}
				}
			}
		
		
			break;

		}

}


uint8_t data_send( uint8_t* src, uint16_t size )
{
	//if(ESP12_RTS_CHK)return 0;
	#ifdef __USERTS__
	while(ESP12_RTS_CHK);
	#endif
	
    //if( esp_curr_status == ESP_CONFIG21 || esp_curr_status == ESP_CONFIG23 ) {
    if(gb_var.bUsrInitOK==TRUE){
#ifdef __DEBUG_INF__
        printf( "data_send(%d)...\n", size );
#endif
        //esp_heart_interval = RTC->CNTL; // by owen
        USR_SendArray( src, size );
        buff_clear();
        return 0;
    }
    return 1;
}


bool esp_send_data(uint8_t chl, uint8_t * src,uint16_t size)
{

	uint8_t esp_send_step=0;
	//uint8_t i;
	bool iret ;
	if(gb_var.bUsrInitOK==FALSE)return FALSE;


	//ESP_SendString( "AT+CIPSEND=%d,%d\r\n",chl,size );
#if 0	
	
KPWAIT:
	buff_clear();
	bs_var.send_delay_cnt=500;
	//while(!FLAG_ESP_RECV);//FLAG_ESP_RECV=0则一直等
	while((!FLAG_ESP_RECV)&&(bs_var.send_delay_cnt>0));//FLAG_ESP_RECV=0则一直等

	if(bs_var.send_delay_cnt==0) return FALSE;




	if( (strstr( ( char* )BUFF_ESP_RECV, "OK" ))&&(strstr( ( char* )BUFF_ESP_RECV, "AT+CIPSEND=" )) )
		{
		ESP_SendArray( src, size );
		buff_clear();
		bs_var.esp_send_data_busy=TRUE;
		iret=TRUE;

		while(!FLAG_ESP_RECV);////一直等到接收到SEND OK

		//if( strstr( ( char* )BUFF_ESP_RECV, "SEND OK" ) )
			{
			bs_var.esp_send_data_busy=FALSE;
			buff_clear();
			}
		
		//else
		//	buff_clear();
		//	goto KPWAIT;
		
		
		}	
	else
		//iret=FALSE;
		goto KPWAIT;

	
		
	
#else
	buff_clear();
	Delay_ms(20);
	USR_SendArray( src, size );
	buff_clear();	
	iret=TRUE;
#endif
	return iret;

}


