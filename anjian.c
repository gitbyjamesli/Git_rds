/*********************************************************************





*/

#include "stm32f10x.h"
#include "anjian.h"

// 按键功能的实现，如长按可递增可递减
FlagStatus UP_LockFlag=RESET;
FlagStatus UP_DelayEnd=RESET;
uint16_t   UP_StartDelayct=0;
uint16_t   UP_Delayct=0;

FlagStatus DOWN_DelayEnd=RESET;
FlagStatus DOWN_LockFlag=RESET;
uint16_t   DOWN_StartDelayct=0;
uint16_t   DOWN_Delayct=0;

FlagStatus LEFT_LockFlag=RESET;
FlagStatus LEFT_DelayEnd=RESET;
uint16_t   LEFT_StartDelayct=0;
uint16_t   LEFT_Delayct=0;

FlagStatus RIGHT_DelayEnd=RESET;
FlagStatus RIGHT_LockFlag=RESET;
uint16_t   RIGHT_StartDelayct=0;
uint16_t   RIGHT_Delayct=0;

//#define KEY_UP_STATUS()              EXTI_GetFlagStatus(EXTI_Line2)


FlagStatus KEY_UP_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line2) || UP_DelayEnd)
	{
		EXTI_ClearFlag(EXTI_Line2);
		UP_DelayEnd = RESET;
		return SET;
	}
	else
	{
		return RESET;
	}
}
#define KEY_UP_CLEAR()               EXTI_ClearFlag(EXTI_Line2);

FlagStatus KEY_DOWN_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line0) || DOWN_DelayEnd)
	{
		EXTI_ClearFlag(EXTI_Line0);
		DOWN_DelayEnd = RESET;
		return SET;
	}
	else
	{
		return RESET;
	}
}
#define KEY_DOWN_CLEAR()               EXTI_ClearFlag(EXTI_Line0);

FlagStatus KEY_LEFT_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line12) || LEFT_DelayEnd)
	{
		EXTI_ClearFlag(EXTI_Line12);
		LEFT_DelayEnd = RESET;
		return SET;
	}
	else
	{
		return RESET;
	}
	
}
#define KEY_LEFT_CLEAR()               EXTI_ClearFlag(EXTI_Line12);

FlagStatus KEY_RIGHT_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line5) || RIGHT_DelayEnd)
	{
		EXTI_ClearFlag(EXTI_Line5);
		RIGHT_DelayEnd = RESET;
		return SET;
	}
	else
	{
		return RESET;
	}
	
}

#define KEY_RIGHT_CLEAR()               EXTI_ClearFlag(EXTI_Line5);

FlagStatus KEY_OK_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line1))
	{
		EXTI_ClearFlag(EXTI_Line1);
		return SET;
	}
	else
	{
		return RESET;
	}
	
}
#define KEY_OK_CLEAR()               EXTI_ClearFlag(EXTI_Line1);


FlagStatus KEY_MENU_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line4))
	{
		EXTI_ClearFlag(EXTI_Line4);
		return SET;
	}
	else
	{
		return RESET;
	}
}
#define KEY_MENU_STATUS()       EXTI_ClearFlag(EXTI_Line4);

FlagStatus KEY_RDS_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line7))
	{
		EXTI_ClearFlag(EXTI_Line7);
		return SET;
	}
	else
	{
		return RESET;
	}
}
#define KEY_RDS_CLEAR()       EXTI_ClearFlag(EXTI_Line7);



FlagStatus AUX_INPUT_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line10))
	{
		EXTI_ClearFlag(EXTI_Line10);
		return SET;
	}
	else
	{
		return RESET;
	}
}

FlagStatus MP3_INPUT_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line11))
	{
		EXTI_ClearFlag(EXTI_Line11);
		return SET;
	}
	else
	{
		return RESET;
	}
}

FlagStatus PTT_INPUT_STATUS(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line12))
	{
		EXTI_ClearFlag(EXTI_Line12);
		return SET;
	}
	else
	{
		return RESET;
	}
}

void KEY_UP_delayct(void)// 长按持续计数
{
	if(UP_LockFlag==SET)
	{
		if(Bit_RESET==GPIO_ReadInputDataBit(KEY_UP_PORT,KEY_UP_PIN))// 按下
		{
			if(UP_StartDelayct<80)
				UP_StartDelayct++;
			if(UP_StartDelayct==80)
			{
				UP_Delayct++;
				if(UP_Delayct==20)
				{
					UP_Delayct=0;
					UP_DelayEnd = SET;
				}
			}
		}
		else
		{
			UP_StartDelayct=0;
			UP_Delayct=0;
			UP_LockFlag=RESET;
		}
	}
}
void KEY_DOWN_delayct(void)// 长按持续计数
{
	if(DOWN_LockFlag==SET)
	{
		if(Bit_RESET==GPIO_ReadInputDataBit(KEY_DOWN_PORT,KEY_DOWN_PIN))// 按下
		{
			if(DOWN_StartDelayct<80)
				DOWN_StartDelayct++;
			if(DOWN_StartDelayct==80)
			{
				DOWN_Delayct++;
				if(DOWN_Delayct==20)
				{
					DOWN_Delayct=0;
					DOWN_DelayEnd = SET;
				}
			}
		}
		else
		{
			DOWN_StartDelayct=0;
			DOWN_Delayct=0;
			DOWN_LockFlag=RESET;
		}
	}
}
void KEY_LEFT_delayct(void)// 长按持续计数
{
	if(LEFT_LockFlag==SET)
	{
		if(Bit_RESET==GPIO_ReadInputDataBit(KEY_LEFT_PORT,KEY_LEFT_PIN))// 按下
		{
			if(LEFT_StartDelayct<80)
				LEFT_StartDelayct++;
			if(LEFT_StartDelayct==80)
			{
				LEFT_Delayct++;
				if(LEFT_Delayct==20)
				{
					LEFT_Delayct=0;
					LEFT_DelayEnd = SET;
				}
			}
		}
		else
		{
			LEFT_StartDelayct=0;
			LEFT_Delayct=0;
			LEFT_LockFlag=RESET;
		}
	}
}
void KEY_RIGHT_delayct(void)// 长按持续计数
{
	if(RIGHT_LockFlag==SET)
	{
		if(Bit_RESET==GPIO_ReadInputDataBit(KEY_RIGHT_PORT,KEY_RIGHT_PIN))// 按下
		{
			if(RIGHT_StartDelayct<80)
				RIGHT_StartDelayct++;
			if(RIGHT_StartDelayct==80)
			{
				RIGHT_Delayct++;
				if(RIGHT_Delayct==20)
				{
					RIGHT_Delayct=0;
					RIGHT_DelayEnd = SET;
				}
			}
		}
		else
		{
			RIGHT_StartDelayct=0;
			RIGHT_Delayct=0;
			RIGHT_LockFlag=RESET;
		}
	}
}
	
