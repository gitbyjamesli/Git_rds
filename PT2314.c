/****************************************************************

      PT2314是一款基于I2C总线控制的四输入音质处理IC，本程序利用软件模拟I2c通
 讯实现对PT2314的操作，下面是一些功能函数。

 各命令字节定义参考PT2314数据手册

 日期：2012-11-23/文友

*/


// 包含文件---------------------------------------------------------
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// 宏定义-----------------------------------------------------------
// 硬件IO定义
#define SDA_PIN             GPIO_Pin_9
#define SDA_GPIO_PORT		GPIOB
#define SDA_GPIO_CLK 		RCC_APB2Periph_GPIOB

#define CLK_PIN             GPIO_Pin_8
#define CLK_GPIO_PORT		GPIOB
#define CLK_GPIO_CLK 		RCC_APB2Periph_GPIOB

#define SDA_LOW()           GPIO_ResetBits(SDA_GPIO_PORT, SDA_PIN)
#define SDA_HIGH()          GPIO_SetBits(SDA_GPIO_PORT, SDA_PIN)

#define CLK_LOW()           GPIO_ResetBits(CLK_GPIO_PORT, CLK_PIN)
#define CLK_HIGH()          GPIO_SetBits(CLK_GPIO_PORT, CLK_PIN)

#define SDA_STATE()         GPIO_ReadInputDataBit(SDA_GPIO_PORT,SDA_PIN)

#include "i2c.h"        // 含模拟的I2C总线操作函数，如启动、主机发1字节、接收从机应答位、校验停止等


// 其它宏
#define GAIN_11dB25 1
#define GAIN_7dB5   2
#define GAIN_3dB75  3
#define GAIN_OFF    0

// 变量定义---------------------------------------------------------

u8 pt2314_error_flag=0;


// 函数原型---------------------------------------------------------

u8 PT2314_SetReg(u8 dat);
u8 PT2314_SetNRegs(u8 *pRegVal, u8 n);
u8 PT2314_Init(void);

u8 PT2314_Init(void)
{
//  0x5c   0101 1100     响度增益0dB/响度关/选择声道1
//  0xc0   1100 0000	 左声道衰减     0dB
//  0xe0   1110 0000	 右声道衰减	    0dB
//	0x00   0000 0000	 总音量衰减控制 -0dB
//  0x6F   0110 1111	 低音控制       +0dB
//	0x7F   0111 1111	 高音控制       +0dB
	u8 PT2314_InitData[6] = {0x5c,0xc0,0xe0,0x00,0x6F,0x7F};
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(SDA_GPIO_CLK |
	                       CLK_GPIO_CLK , 
						   ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);// 启动设置

	GPIO_SetBits(GPIOB, GPIO_Pin_8|GPIO_Pin_9);

	pt2314_error_flag = PT2314_SetNRegs(PT2314_InitData,6);
	return pt2314_error_flag;
}

u8 PT2314_Setup_CVD(u8 channel,u8 gain_volume)// 通道/响度/增益设置   成功返回0
{
	u8 byte=0x40;
	u8 ack=0;

	switch (gain_volume)
	{
		case GAIN_11dB25 : byte |= 0x00;
			break;
		case GAIN_7dB5   : byte |= 0x08;
			break;
		case GAIN_3dB75  : byte |= 0x10;
			break;
		case GAIN_OFF    : byte |= 0x1c;
			break;
		default:           return 1;
	}		  
	if(channel>4)return 1;
	byte |= (channel-1);
	pt2314_error_flag = ack = PT2314_SetReg(byte);

	return ack;
}

u8 PT2314_Setup_Volume(u8 attenuation)// 总音量控制设置  成功返回0
{
	u8 ack=0;

	if(attenuation>32)return 1;
	pt2314_error_flag = ack = PT2314_SetReg(32-attenuation);

	return ack;
}
u8 PT2314_Setup_Bass(u8 Bass_dB)// 低音设置
{
	u8 ack=0;
	u8 byte=0x60|(Bass_dB&0x0F);// 0110 ????

	pt2314_error_flag = ack = PT2314_SetReg(byte);

	return ack;
}
u8 PT2314_Setup_Treble(u8 Treble_dB)// 高音设置
{
	u8 ack=0;
	u8 byte=0x70|(Treble_dB&0x0F);// 0111 ????

	pt2314_error_flag = ack = PT2314_SetReg(byte);

	return ack;
}
u8 PT2314_SetReg(u8 dat)// 写配置字节   成功返回0
{
	u8 ack = 0;

	I2CStart();
    I2CWrite8Bit(0x88);
	ack = I2CChkAck();
	if(ack)goto stop;
	I2CWrite8Bit(dat);
	ack = I2CChkAck();
	if(ack)goto stop;
stop:
	I2CStop();
	return ack;
}

u8 PT2314_SetNRegs(u8 *pRegVal,	u8 n)// 连续写n个配置字节  成功返回0
{
	u8	ack = 0;
	u8 	i;
	
	I2CStart();
    I2CWrite8Bit(0x88);
	ack = I2CChkAck();
	if(ack)goto stop;
	for(i = 0; i < n; i++)
	{
		I2CWrite8Bit(pRegVal[i]);
		ack = I2CChkAck();
		if(ack)goto stop;
	}
stop:
	I2CStop();
	return ack;
}



//--------------文件完-----------------------------------------------
