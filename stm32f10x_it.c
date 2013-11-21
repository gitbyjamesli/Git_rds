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
#include "my_datadef.h"       // һЩ�������Ͷ��� , ��: ID������,ʱ������
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
 ������STM32�������жϴ�������ע�⺯������ʽ��PPP_IRQFandler
 PPP��ʾ���������磺ADC��TIM�ȡ�
*/


/**************���ڽ���ȫ��********************/
extern FlagStatus        packet_head;    //֡ͷ����ʼ���ձ�־
extern FlagStatus        packet_head2;    //֡ͷ����ʼ���ձ�־
//extern FlagStatus        packet_R_OK;    //������ϱ�־   SET��Ч
extern u8                packet_rct;     //�����ֽڼ���
extern u8                packet_len;     //�����ֽ�
extern u8                packet_xor;
extern u8                packet_rbuf[64]; //���յ����ݻ���

extern FlagStatus        packet_T_OK;    //������ɱ�־ 0���� 1������
extern u8                packet_sendsize;//���Ͱ���С
extern u8                packet_tct;
extern u8                packet_tbuf[32]; //���͵����ݻ���

extern u8                packet_rct2;     //�����ֽڼ���
extern u8                packet_len2;     //�����ֽ�
extern u8                packet_xor2;
extern u8                packet_rbuf2[32]; //���յ����ݻ���

extern FlagStatus        packet_T_OK2;    //������ɱ�־ 0���� 1������
extern u8                packet_sendsize2;//���Ͱ���С
extern u8                packet_tct2;
extern u8                packet_tbuf2[2]; //���͵����ݻ���

void USART1_IRQHandler(void)//ţ������
{
	u8 SBUF;
   	//LCD19264_DisOFF();
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)// ���ձ�־�ж�
	{
		SBUF = USART_ReceiveData(USART1);// ����һ���ֽ�������


		if(packet_head == SET)
		{
			packet_rct++;
			if(packet_rct==2)
			{
				if(5 < SBUF && SBUF < 64)//�������6�ֽڣ�С��64�ֽ�
				{
					packet_len = SBUF;
					packet_xor = SBUF;//�Ӱ�����ʼ���	
				}
				else
				{
					packet_head = RESET;
				}
			}
			else
			{
				if(packet_rct < packet_len+2)//������������
				{
					packet_rbuf[packet_rct-3] = SBUF;
					packet_xor ^= SBUF;
				}
				else//�������
				{
					if(packet_xor == SBUF)//У���ж�
					{
						packet_R_OK = SET;	
					}
					packet_head = RESET;//�����¿�ʼ����֡ͷ
				}
			}
		}
		else
		{
			if(0xa1 == SBUF)//1.���յ�֡ͷ
			{
				packet_head = SET;//�յ�֡ͷ
				packet_rct = 1;
			}
		}
	}
		
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)// ���ͱ�־�ж�
	{   
		if(packet_tct<packet_sendsize)
		{
			USART_SendData(USART1, packet_tbuf[packet_tct++]);// ����
		}
		else
		{
			packet_tct = 0;
			packet_T_OK = SET;//�������
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}

	}

//LCD19264_DisON();

}




extern OS_TID tid_ComDebug;

void USART2_IRQHandler(void)//6���
{
	u8 SBUF;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)// ���ձ�־�ж�
	{
		SBUF = USART_ReceiveData(USART2);// ����һ���ֽ�������


		if(packet_head2 == SET)
		{
			packet_rct2++;
			if(packet_rct2==2)
			{
				if(5 < SBUF && SBUF < 64)//�������6�ֽڣ�С��64�ֽ�
				{
					packet_len2 = SBUF;
					//Rxbyte_ct=packet_len;
					packet_xor2 = SBUF;//�Ӱ�����ʼ���	
				}
				else
				{
					packet_head2 = RESET;
				}
			}
			else
			{
				if(packet_rct2 <= (packet_len2+2))//������������
				{
					packet_rbuf2[packet_rct2-3] = SBUF;
					//packet_xor ^= SBUF;
				}
				else//�������
				{
					//if(1)//packet_xor == SBUFУ���ж�
					//{
						packet_R_OK2 = SET;
						packet_rct2=0;
					//}
					packet_head2 = RESET;//�����¿�ʼ����֡ͷ
				}
			}
		}
		else
		{
			if(0xa3 == SBUF)//1.���յ�֡ͷ
			{
				packet_head2 = SET;//�յ�֡ͷ
				packet_rct2 = 1;
			}
		}
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)// ���ͱ�־�ж�
	{   
		if(packet_tct2<packet_sendsize2)
		{
			USART_SendData(USART2, packet_tbuf2[packet_tct2++]);// ����
		}
		else
		{
			packet_tct2 = 0;
			packet_T_OK2 = SET;//�������
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}

	}

}


void RTC_IRQHandler(void)
{
	extern FlagStatus      SystemTime_SecUpdate_Flag;// ���±�־

	if(RTC_GetITStatus(RTC_IT_SEC) != RESET)/*  �ж�RTC�Ƿ��������жϣ�Ҳ�п�����������������ж�) */
	{
		RTC_ClearITPendingBit(RTC_IT_SEC);/* ������жϱ�־ */

		
		RTC_WaitForLastTask();/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
		
		if(24*60*60 <= RTC_GetCounter())/* ���ʱ��ﵽ23:59:59����һ��ʱ��Ϊ00:00:00 */
		{
			RTC_SetCounter(0x0);
			RTC_WaitForLastTask();/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
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
	if(RTC_GetITStatus(RTC_IT_ALR) != RESET)// ���������ж�
	{
		RTC_ClearITPendingBit(RTC_IT_ALR);// �����־
		;// Ҫ���ӵĴ���
	}
	if(RTC_GetITStatus(RTC_IT_OW) != RESET)
	{
		RTC_ClearITPendingBit(RTC_IT_OW);// �����־
		;// Ҫ���ӵĴ���
	}
}

#include "anjian.h"
void TIM3_IRQHandler(void)// TIM3���жϷ�����       10ms���
{
	extern uint32_t sendstate_Time_Count;// 
	extern uint32_t Menu_Timeout_Count;// �˵�������ʱ��������
	extern uint16_t CursorBlink_Count;// ��귭ת����ʱ
	extern uint16_t CollectUpdate_Interval;
	extern uint32_t sound_ch_switch_Timeout_Count;// 

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);// �� ��־

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

	KEY_UP_delayct();// ������������
	KEY_DOWN_delayct();
	KEY_LEFT_delayct();
	KEY_RIGHT_delayct();

}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
