/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "my_datadef.h"       // 一些数据类型定义 , 如: ID号类型,时间类型
#include <RTL.h>
#include "all.h"
/** @addtogroup STM32F10x_StdPeriph_Template
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
/*void SVC_Handler(void)
{
}
*/
/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*void PendSV_Handler(void)
{
}
*/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*void SysTick_Handler(void)
{
}*/

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/


/******************************************************************************
 下面是STM32的外设中断处理函数，注意函数名格式：PPP_IRQFandler
 PPP表示外设名，如：ADC，TIM等。
*/


/**************串口接收全局********************/
extern FlagStatus        packet_head;    //帧头即开始接收标志
extern FlagStatus        packet_head2;    //帧头即开始接收标志
//extern FlagStatus        packet_R_OK;    //接收完毕标志   SET有效
extern u8                packet_rct;     //接收字节计数
extern u8                packet_len;     //包长字节
extern u8                packet_xor;
extern u8                packet_rbuf[64]; //接收的数据缓存

extern FlagStatus        packet_T_OK;    //发送完成标志 0就绪 1发送中
extern u8                packet_sendsize;//发送包大小
extern u8                packet_tct;
extern u8                packet_tbuf[32]; //发送的数据缓存

extern u8                packet_rct2;     //接收字节计数
extern u8                packet_len2;     //包长字节
extern u8                packet_xor2;
extern u8                packet_rbuf2[32]; //接收的数据缓存

extern FlagStatus        packet_T_OK2;    //发送完成标志 0就绪 1发送中
extern u8                packet_sendsize2;//发送包大小
extern u8                packet_tct2;
extern u8                packet_tbuf2[2]; //发送的数据缓存

void USART1_IRQHandler(void)//牛角座口
{
	u8 SBUF;
   	//LCD19264_DisOFF();
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)// 接收标志判断
	{
		SBUF = USART_ReceiveData(USART1);// 读入一个字节至缓存


		if(packet_head == SET)
		{
			packet_rct++;
			if(packet_rct==2)
			{
				if(5 < SBUF && SBUF < 64)//必须大于6字节，小于64字节
				{
					packet_len = SBUF;
					packet_xor = SBUF;//从包长开始异或	
				}
				else
				{
					packet_head = RESET;
				}
			}
			else
			{
				if(packet_rct < packet_len+2)//继续接收数据
				{
					packet_rbuf[packet_rct-3] = SBUF;
					packet_xor ^= SBUF;
				}
				else//接收完毕
				{
					if(packet_xor == SBUF)//校验判断
					{
						packet_R_OK = SET;	
					}
					packet_head = RESET;//需重新开始接收帧头
				}
			}
		}
		else
		{
			if(0xa1 == SBUF)//1.接收到帧头
			{
				packet_head = SET;//收到帧头
				packet_rct = 1;
			}
		}
	}
		
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)// 发送标志判断
	{   
		if(packet_tct<packet_sendsize)
		{
			USART_SendData(USART1, packet_tbuf[packet_tct++]);// 发送
		}
		else
		{
			packet_tct = 0;
			packet_T_OK = SET;//发送完毕
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}

	}

//LCD19264_DisON();

}




extern OS_TID tid_ComDebug;

void USART2_IRQHandler(void)//6针口
{
	u8 SBUF;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)// 接收标志判断
	{
		SBUF = USART_ReceiveData(USART2);// 读入一个字节至缓存


		if(packet_head2 == SET)
		{
			packet_rct2++;
			if(packet_rct2==2)
			{
				if(5 < SBUF && SBUF < 64)//必须大于6字节，小于64字节
				{
					packet_len2 = SBUF;
					//Rxbyte_ct=packet_len;
					packet_xor2 = SBUF;//从包长开始异或	
				}
				else
				{
					packet_head2 = RESET;
				}
			}
			else
			{
				if(packet_rct2 <= (packet_len2+2))//继续接收数据
				{
					packet_rbuf2[packet_rct2-3] = SBUF;
					//packet_xor ^= SBUF;
				}
				else//接收完毕
				{
					//if(1)//packet_xor == SBUF校验判断
					//{
						packet_R_OK2 = SET;
						packet_rct2=0;
					//}
					packet_head2 = RESET;//需重新开始接收帧头
				}
			}
		}
		else
		{
			if(0xa3 == SBUF)//1.接收到帧头
			{
				packet_head2 = SET;//收到帧头
				packet_rct2 = 1;
			}
		}
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)// 发送标志判断
	{   
		if(packet_tct2<packet_sendsize2)
		{
			USART_SendData(USART2, packet_tbuf2[packet_tct2++]);// 发送
		}
		else
		{
			packet_tct2 = 0;
			packet_T_OK2 = SET;//发送完毕
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}

	}

}


void RTC_IRQHandler(void)
{
	extern FlagStatus      SystemTime_SecUpdate_Flag;// 更新标志

	if(RTC_GetITStatus(RTC_IT_SEC) != RESET)/*  判断RTC是否发生了秒中断（也有可能是溢出或者闹钟中断) */
	{
		RTC_ClearITPendingBit(RTC_IT_SEC);/* 清除秒中断标志 */

		
		RTC_WaitForLastTask();/* 等待上一次对RTC寄存器的写操作完成 */
		
		if(24*60*60 <= RTC_GetCounter())/* 如果时间达到23:59:59则下一刻时间为00:00:00 */
		{
			RTC_SetCounter(0x0);
			RTC_WaitForLastTask();/* 等待上一次对RTC寄存器的写操作完成 */
		}
		SystemTime_SecUpdate_Flag = SET;

		Disconnect_pc++; //
		if(Disconnect_pc>(60*2))
		{
		 Disconnect_pc=0;
		 PCRDS_Blinking_flag=OFF;
		 pc_area_group_terminal_set_flag=0;
		 pc_gbset=FALSE;
		 }

	}
	if(RTC_GetITStatus(RTC_IT_ALR) != RESET)// 发生闹钟中断
	{
		RTC_ClearITPendingBit(RTC_IT_ALR);// 清除标志
		;// 要增加的代码
	}
	if(RTC_GetITStatus(RTC_IT_OW) != RESET)
	{
		RTC_ClearITPendingBit(RTC_IT_OW);// 清除标志
		;// 要增加的代码
	}
}

#include "anjian.h"
void TIM3_IRQHandler(void)// TIM3的中断服务函数       10ms溢出
{
	extern uint32_t sendstate_Time_Count;// 
	extern uint32_t Menu_Timeout_Count;// 菜单操作超时计数变量
	extern uint16_t CursorBlink_Count;// 光标翻转倒计时
	extern uint16_t CollectUpdate_Interval;
	extern uint32_t sound_ch_switch_Timeout_Count;// 

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);// 清 标志

	sendstate_Time_Count++;
	if(Menu_Timeout_Count)
	{
		Menu_Timeout_Count--;
	}
	if(CursorBlink_Count)
	{
		CursorBlink_Count--;
	}
	if(CollectUpdate_Interval)
	{
		CollectUpdate_Interval--;
	}

     if(sound_ch_switch_Timeout_Count>0)
	  sound_ch_switch_Timeout_Count--;

	KEY_UP_delayct();// 长按持续计数
	KEY_DOWN_delayct();
	KEY_LEFT_delayct();
	KEY_RIGHT_delayct();

}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
