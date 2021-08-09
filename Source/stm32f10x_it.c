/**
  ******************************************************************************
  * @file    USART/Printf/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "USR_TCP232.h"
//#include "esp_user.h"
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern void SysTick_ISR(void);
void SysTick_Handler(void)
{
	SysTick_ISR();
}

void EXTI3_IRQHandler(void)
{
  // if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  // {
	// Rf_GDO0_Task();
  //   /* Clear the  EXTI line 0 pending bit */
  //   EXTI_ClearITPendingBit(EXTI_Line3);
  // }
}

extern void Timer_10ms_ISR(void);
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)== SET)	
	{	
		//Tim_10msIrq();
		Timer_10ms_ISR();
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update); 
	} 
}

#ifndef USART_TEST

#ifdef USE_TCP232
void USART1_IRQHandler( void )
{
#ifdef OS_TICKS_PER_SEC     //如果时钟节拍数定义了,说明要使用ucosII了.
    OSIntEnter();
#endif

    // 空闲中断
    if( USART_GetITStatus( USART1, USART_IT_IDLE ) != RESET ) {
#if USARTy_DMA_RECV == 1
        USART_ReceiveData( USART1 ); //读取数据 注意：这句必须要，否则不能够清除中断标志位。
        Rx1_Count = USARTy_Rx_BufSize - DMA_GetCurrDataCounter( USARTy_Rx_DMA_Channel ); // 获得接收到的字节数
        Rx1_flag = 1;              // 标记接收成功

        USART_ClearITPendingBit( USART1, USART_IT_IDLE );       //清除中断标志

        //恢复DMA指针，等待下一次的接收
        DMA_Cmd( USARTy_Rx_DMA_Channel, DISABLE ); //关闭USART1 TX DMA1 所指示的通道
        DMA_SetCurrDataCounter( USARTy_Rx_DMA_Channel, USARTy_Rx_BufSize ); //DMA通道的DMA缓存的大小
        DMA_Cmd( USARTy_Rx_DMA_Channel, ENABLE ); //使能USART1 TX DMA1 所指示的通道
#endif
        uint8_t Clear;//不能删除
        Clear = USART1->SR;
        Clear = USART1->DR;
    }

    //// 检查 ORE 标志,防止开关总中断死机，放在接收中断前面
    //if( USART_GetFlagStatus( USART1, USART_FLAG_ORE ) != RESET ) {
    //    USART_ClearFlag( USART1, USART_FLAG_ORE );
    //    USART_ReceiveData( USART1 );
    //}

#if USARTy_DMA == 0 || USARTy_DMA_RECV == 0
    // 接收中断(接收到的数据必须是\n结尾)
    if( USART_GetITStatus( USART1, USART_IT_RXNE ) != RESET ) {
        uint8_t dt = USART1->DR;
        Rx1_Buf[Rx1_Count % RX1BUF_SIZE] = dt;
        USART_ClearITPendingBit( USART1, USART_IT_RXNE );

        //USART3_Send_Array( &dt, 1 ); // 输出返回信息=>调试接口
        //RTT_LOG(dt);
		//__disable_irq();

        recv_process( dt );
/*	
		if(dt == 0x18)
			{
			Test2++;
			}
		*/
		//__enable_irq();

        Rx1_to = Rx1_to_len;//TIME_RECV_IDLE;
        Rx1_Count++;
    }
#endif

#ifdef OS_TICKS_PER_SEC     //如果时钟节拍数定义了,说明要使用ucosII了.
    OSIntExit();
#endif
}

#else
void USART1_IRQHandler( void )
{
	u8 data; //判断是不是接受中断
	if( USART_GetITStatus( USART1, USART_IT_IDLE ) != RESET ) {
	
        uint8_t Clear;//不能删除
        Clear = USART1->SR;
        Clear = USART1->DR;	
	}


	
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		data=USART_ReceiveData(USART1);
		//Delay_ms(256);
		//USART_SendData(USART2,data);
		//esp_send_data(0,&data,1);
		//Receive_Etc(data);
		
		uint8_t dt = USART1->DR;
		//Rx1_Buf[Rx1_Count % RX1BUF_SIZE] = dt;
		if(gb_var.rx1_have_data == 0)gb_var.rx1_count=0;
		
		gb_var.rx_buffer[gb_var.rx1_count]=dt;
		USART_ClearITPendingBit( USART1, USART_IT_RXNE );
		gb_var.rx1_timeout = USART1__TIMEOUT;
		gb_var.rx1_count ++;
		gb_var.rx1_have_data = 1;
	}

}
#endif

//锟叫断凤拷锟斤拷锟斤拷
void USART2_IRQHandler(void)
{
	u8 data; //判断是不是接受中断
	if( USART_GetITStatus( USART2, USART_IT_IDLE ) != RESET ) {
	
		uint8_t Clear;//不能删除
		Clear = USART2->SR;
		Clear = USART2->DR; 
	}


	
	if(USART_GetITStatus(USART2,USART_IT_RXNE))
	{
	data=USART_ReceiveData(USART2);
	uint8_t dt = USART2->DR;
	//Rx1_Buf[Rx1_Count % RX1BUF_SIZE] = dt;
	if(gb_var.rx2_have_data == 0)gb_var.rx2_count=0;
	
	gb_var.rx2_buffer[gb_var.rx2_count]=dt;
	USART_ClearITPendingBit( USART2, USART_IT_RXNE );
	gb_var.rx2_timeout = USART2__TIMEOUT;
	gb_var.rx2_count ++;
	gb_var.rx2_have_data = 1;

	}

}


//锟叫断凤拷锟斤拷锟斤拷
void USART3_IRQHandler(void)
{
	u8 data; //判断是不是接受中断
	if( USART_GetITStatus( USART3, USART_IT_IDLE ) != RESET ) {
	
		uint8_t Clear;//不能删除
		Clear = USART3->SR;
		Clear = USART3->DR; 
	}


	
	if(USART_GetITStatus(USART3,USART_IT_RXNE))
	{
	data=USART_ReceiveData(USART3);
	uint8_t dt = USART3->DR;
	//Rx1_Buf[Rx1_Count % RX1BUF_SIZE] = dt;
	if(gb_var.rx3_have_data == 0)gb_var.rx3_count=0;
	
	gb_var.rx3_buffer[gb_var.rx3_count]=dt;
	USART_ClearITPendingBit( USART3, USART_IT_RXNE );
	gb_var.rx3_timeout = USART3__TIMEOUT;
	gb_var.rx3_count ++;
	gb_var.rx3_have_data = 1;

	}

}




#else
//锟叫断凤拷锟斤拷锟斤拷
void USART1_IRQHandler(void)
{
	u8 data; //锟叫讹拷锟角诧拷锟角斤拷锟斤拷锟叫讹拷
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
	data=USART_ReceiveData(USART1);
	//Delay_ms(256);
	USART_SendData(USART1,data);
	}
}


//锟叫断凤拷锟斤拷锟斤拷
void USART2_IRQHandler(void)
{
	u8 data; //锟叫讹拷锟角诧拷锟角斤拷锟斤拷锟叫讹拷
	if(USART_GetITStatus(USART2,USART_IT_RXNE))
	{
	data=USART_ReceiveData(USART2);
	//Delay_ms(256);
	USART_SendData(USART2,data);
	}
}

//锟叫断凤拷锟斤拷锟斤拷
void USART3_IRQHandler(void)
{
	u8 data; //锟叫讹拷锟角诧拷锟角斤拷锟斤拷锟叫讹拷
	if(USART_GetITStatus(USART3,USART_IT_RXNE))
	{
	data=USART_ReceiveData(USART3);
	//Delay_ms(256);
	USART_SendData(USART3,data);
	}
}

#endif
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
