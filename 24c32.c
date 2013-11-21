/*************************************************************






*/

// 头文件-----------------------------------------------------------
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// 宏定义-----------------------------------------------------------

// 先声明模拟的SDA，SCL所在端口
#define SDA_PIN             GPIO_Pin_7
#define SDA_GPIO_PORT		GPIOB
#define SDA_GPIO_CLK 		RCC_APB2Periph_GPIOB

#define CLK_PIN             GPIO_Pin_6
#define CLK_GPIO_PORT		GPIOB
#define CLK_GPIO_CLK 		RCC_APB2Periph_GPIOB

#define SDA_LOW()           GPIO_ResetBits(SDA_GPIO_PORT, SDA_PIN)
#define SDA_HIGH()          GPIO_SetBits(SDA_GPIO_PORT, SDA_PIN)

#define CLK_LOW()           GPIO_ResetBits(CLK_GPIO_PORT, CLK_PIN)
#define CLK_HIGH()          GPIO_SetBits(CLK_GPIO_PORT, CLK_PIN)

#define SDA_STATE()         GPIO_ReadInputDataBit(SDA_GPIO_PORT,SDA_PIN)
#include "i2c.h"// 加入模拟的i2c驱动程序

// 变量定义------------------------------------------------

u8 eeprom_error_flag;
//u8 QN8027_error_flag;


u8 EEPROM_Read1Byte(u16 addr)
{
	u8 addr_H = addr>>8;
	u8 addr_L = addr;
	u8 outdata=0;

	I2CStart();

    I2CWrite8Bit(0xA0);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(addr_H);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(addr_L);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CStart();
	I2CWrite8Bit(0xA1);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	outdata = I2CRead8Bit();
	I2CSlave_NOACK();
stop:
	I2CStop();

	return (outdata);
}

void EEPROM_Write1Byte(u16 addr,u8 indata)
{
	u8 addr_H = addr>>8;
	u8 addr_L = addr;

	I2CStart();

    I2CWrite8Bit(0xA0);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(addr_H);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(addr_L);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(indata);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;
stop:
	I2CStop();
}


u8 EEPROM_ReadBytes(u16 addr,u8 *buf,u16 len)
{
	u16 i;
	u8 addr_H = addr>>8;
	u8 addr_L = addr;

	I2CStart();

    I2CWrite8Bit(0xA0);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(addr_H);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(addr_L);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CStart();
	I2CWrite8Bit(0xA1);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	for(i=0;i<(len-1);i++)
	{
		buf[i] = I2CRead8Bit();
		I2CSlave_ACK();
	}
	buf[i] = I2CRead8Bit();
	I2CSlave_NOACK();

stop:
	I2CStop();

	return eeprom_error_flag;
}

u8 EEPROM_WriteBytes(u16 addr,u8 *buf,u16 len)
{
	u16 i;
	u8 addr_H = addr>>8;
	u8 addr_L = addr;

	I2CStart();

    I2CWrite8Bit(0xA0);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(addr_H);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	I2CWrite8Bit(addr_L);
	eeprom_error_flag = I2CChkAck();
	if(eeprom_error_flag)goto stop;

	for(i=0;i<len;i++)
	{
		I2CWrite8Bit(buf[i]);
		eeprom_error_flag = I2CChkAck();
		if(eeprom_error_flag)goto stop;
	}

stop:
	I2CStop();

	return eeprom_error_flag;
}


