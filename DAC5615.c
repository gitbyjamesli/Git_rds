#include "stm32f10x.h"

// 宏定义-----------------------------------------------------------
// 硬件IO定义
#define SDA_PIN             GPIO_Pin_1
#define SDA_GPIO_PORT		GPIOC
#define SDA_GPIO_CLK 		RCC_APB2Periph_GPIOC

#define CLK_PIN             GPIO_Pin_2
#define CLK_GPIO_PORT		GPIOC
#define CLK_GPIO_CLK 		RCC_APB2Periph_GPIOC

#define CS_PIN              GPIO_Pin_3
#define CS_GPIO_PORT		GPIOC
#define CS_GPIO_CLK 		RCC_APB2Periph_GPIOC

#define SDA_LOW()           GPIO_ResetBits(SDA_GPIO_PORT, SDA_PIN);
#define SDA_HIGH()          GPIO_SetBits(SDA_GPIO_PORT, SDA_PIN);
#define CLK_LOW()           GPIO_ResetBits(CLK_GPIO_PORT, CLK_PIN);
#define CLK_HIGH()          GPIO_SetBits(CLK_GPIO_PORT, CLK_PIN);
#define CS_LOW()            GPIO_ResetBits(CS_GPIO_PORT, CS_PIN);
#define CS_HIGH()           GPIO_SetBits(CS_GPIO_PORT, CS_PIN);

void DAC5615delay(void)
{
	uint32_t dz=10;
	while(dz--);
}
void DAC5615_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);// 启动设置

	CLK_LOW();// 时钟常态为低电平
}

void DAC5615Out(uint16_t dac_value)
{
	uint8_t n;

	CS_LOW();
	dac_value<<=6;
	for(n=0;n<10;n++)// 有效数据位
	{
		if(dac_value&0x8000)
		{
			SDA_HIGH();
		}
		else
		{
			SDA_LOW();
		}
		CLK_HIGH();
		DAC5615delay();
		dac_value<<=1;
		CLK_LOW();
		DAC5615delay();
	}
	for(n=0;n<2;n++)// 最后填充2bit
	{
		CLK_HIGH();
		DAC5615delay();
		CLK_LOW();
		DAC5615delay();
	}

	CS_HIGH();
}



// END-------------------------------------------------------------
