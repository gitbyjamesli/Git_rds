/*************************************************************

 说明：
        下面的部分是用来调试QN8027调频发射芯片的，这里使用模拟的i2c总线
	PB10和PB11引脚。

	2012-11-23/文友




*/

// 头文件-----------------------------------------------------------
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "all.h"
// 宏定义-----------------------------------------------------------

// 先声明模拟的SDA，SCL所在端口
#define SDA_PIN             GPIO_Pin_11
#define SDA_GPIO_PORT		GPIOB
#define SDA_GPIO_CLK 		RCC_APB2Periph_GPIOB

#define CLK_PIN             GPIO_Pin_10
#define CLK_GPIO_PORT		GPIOB
#define CLK_GPIO_CLK 		RCC_APB2Periph_GPIOB

#define SDA_LOW()           GPIO_ResetBits(SDA_GPIO_PORT, SDA_PIN)
#define SDA_HIGH()          GPIO_SetBits(SDA_GPIO_PORT, SDA_PIN)

#define CLK_LOW()           GPIO_ResetBits(CLK_GPIO_PORT, CLK_PIN)
#define CLK_HIGH()          GPIO_SetBits(CLK_GPIO_PORT, CLK_PIN)

#define SDA_STATE()         GPIO_ReadInputDataBit(SDA_GPIO_PORT,SDA_PIN)
#include "i2c.h"// 加入模拟的i2c驱动程序

#define Chipid_QN8027     1



// 变量定义------------------------------------------------

u8 QN8027_error_flag;


static
void PortInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能外设时钟 (APB2)*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, 
	                                    ENABLE);

	// 两个LED灯的控制IO模式设置为强驱，TX_LED(RDS发射) -> PB13,  ER_LED(报警) -> PB14
	//  I2C_SDA-> PB7,   I2C_SCL-> PB6

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_7|GPIO_Pin_6);// 输出高电平
}
void QN8027_WriteReg(u8 RegisAddr,u8 data)
{
	I2CStart();

    I2CWrite8Bit(0x58);//0x20
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

	I2CWrite8Bit(RegisAddr);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

	I2CWrite8Bit(data);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

stop:
	I2CStop();
}

u8 QN8027_ReadReg(u8 RegisAddr)
{
	u8 readout_data=0;

	I2CStart();
// 2.写器件地址 
    I2CWrite8Bit(0x58);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;
// 3.写寄存器地址 
	I2CWrite8Bit(RegisAddr);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

// 4.重写器件地址，以便开始读 
	I2CStart();
 
    I2CWrite8Bit(0x59);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

	readout_data = I2CRead8Bit();
	I2CSlave_NOACK();
stop:
	I2CStop();

	return (readout_data);
}

void QN8027_MultidataRead(u8 RegisAddr,u8 *buf,u16 len)
{
	u16 read_ct;

	I2CStart();
// 2.写器件地址 
    I2CWrite8Bit(0x58);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;
// 3.写寄存器地址 
	I2CWrite8Bit(RegisAddr);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

// 4.重写器件地址 ,以便开始读
	I2CStart();
 
    I2CWrite8Bit(0x59);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

	for(read_ct=0;   read_ct<(len-1);    read_ct++)
	{
		buf[read_ct] = I2CRead8Bit();
		I2CSlave_ACK();
	}
	buf[read_ct] = I2CRead8Bit();
	I2CSlave_NOACK();
stop:
	I2CStop();
}

void QN8027_MultidataWrite(u8 RegisAddr,u8 *buf,u16 len)
{
	u16 count;

	I2CStart();

    I2CWrite8Bit(0x58);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

	I2CWrite8Bit(RegisAddr);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

 
    for(count=0;    count<len;   count++)
	{
		I2CWrite8Bit(buf[count]);
		QN8027_error_flag = I2CChkAck();
		if(QN8027_error_flag)goto stop;
	}
stop:
	I2CStop();
}

void QN8027_Delay(u16 ms) 
{
    u16 i,k;
    for(i=0; i<60000;i++) 
    {    
        for(k=0; k<ms; k++); 
    }
}

void QN8027_SetFMChannel(u32 FMChannel)
{
	u32 temp = (FMChannel-60000)/50;
	u8 byte_high, byte_low;

	byte_low = (u8)temp;					// XXXX XXXX
	byte_high =  (temp>>8)&0x03;			// 0000 00XX
	QN8027_WriteReg(0x07,byte_low);      
	QN8027_WriteReg(0x0a,byte_high);   
}



//功能:修改QN80XX寄存器值
//输入1:RegAddr参数:寄存器地址
//输入2:DataByte参数:写入寄存器的数据
//输入3:BitMask参数:写入寄存器的数据掩码
//输出:无
//编写日期:2011-02-03
//修改日期:无
u8  TempRegData2;
void	QndSetReg(u8 RegAddr,u8 DataByte,u8 BitMask)
{
    u8 TempRegData;
    TempRegData2=QN8027_ReadReg(RegAddr);	//读取寄存器值
    TempRegData2&=(u8)(~BitMask);		//清除寄存器旧值
    TempRegData2|=DataByte&BitMask;		//写入新值到寄存器数据
    QN8027_WriteReg(RegAddr,TempRegData2);	//将数据写回寄存器
}



//功能:QN80XX RDS开关
//输入:Mode参数:1打开RDS功能,0关闭RDS功能
//输出:无
//编写日期:2011-02-03
//修改日期:无
#define	QndRds		0x12
#define	RdsFdev		32	//RDS调制度0-127
#define	QndGplt		0x02
#define	On	        0x01
#define	OFF	        0x00
#define	QndPaOffDisable	0x30
#define	QndSetT1mSel(Value)	QndSetReg(QndGplt,Value,0x30)
#define	QndSystem1	0x00	//系统标志寄存器1
#define	QndCh1		0x01

void	QndRdsMode(u8	Mode)
{
#if	Chipid_QN8027
		if(Mode)
		{
			QN8027_WriteReg(QndRds,0x80|RdsFdev);	//开启RDS
			//QndSetReg(QndGplt,0x40,0x40);	//开启RDS加密
		}
		else
		{
			QndSetReg(QndRds,0x00,(1<<7));	//禁止RDS
//			QndSetReg(QndGplt,0,(1<<6));	//禁止RDS加密
		}
	
#endif
	
#if	Chipid_QN8035
		if(Mode)
		{
			QndWriteReg(QndIntCtrl,0xa8);
			//QndSetReg(QndIntCtrl,0x08,0x08);			
			QndSetReg(QndSystem1,(1<<3),(1<<3));	//开启RDS

		}
		else
		{
			QndSetReg(QndSystem1,0x00,(1<<3));	//禁止RDS
			QndSetReg(QndIntCtrl,0x00,0x08);
		}

#endif

}

//功能:修改QN80XX频道
//输入:ChannelByte参数:10位频道数据
//输出:无
//编写日期:2011-02-03
//修改日期:无
u8	TempChannel;
void	QNd8027_SetChannel(u16	ChannelByte)
{
	//u8	TempChannel;	
#if	Chipid_QN8027	//QN8027基频为64M，加上频道设置的步进频率为实际发射频率	

	TempChannel=(u8)(ChannelByte>>8);	//取频道高2位
	Reg0_save=TempChannel;
	QndSetReg(QndSystem1,TempChannel,0x03);	//写入寄存器
	TempChannel=(u8)ChannelByte;			//取频道低8位
	QN8027_WriteReg(QndCh1,TempChannel);		//写入寄存器
	//Reg0_save=QN8027_ReadReg(0x0);
	/*
	TempChannel=QN8027_ReadReg(QndSystem1);
	TempChannel = (TempChannel & 0xFC) | ((ChannelByte&0X300)>>8);
	TempChannel &= 0xdf;//*set standby mode
	QN8027_WriteReg(QndSystem1,TempChannel);		//写入寄存器

	TempChannel=(u8)ChannelByte;			//取频道低8位
	QN8027_WriteReg(QndCh1,TempChannel);		//写入寄存器

	TempChannel=QN8027_ReadReg(QndSystem1);
	TempChannel |= 0x20;//*set transmit mode
	QN8027_WriteReg(QndSystem1,TempChannel);		//写入寄存器
	*/
#endif
	
}

//功能:QN80XX静音设置
//输入:Mode参数:1开启静音,0关闭静音
//输出:无
//编写日期:2011-02-03
//修改日期:无
void	QndMute(u8	Mode)
{
#if	Chipid_QN8027
	if(Mode)
	{
		QndSetReg(QndSystem1,(1<<4),(1<<4));
	}
	else
	{
		QndSetReg(QndSystem1,0x00,(1<<4));
	}

#endif
}


//功能:QN80XX天线初始化
void	QndCapBankTuning(void)
{
	u8	TempRegValue;
	QN8027_WriteReg(0x4f,0x80);
	TempRegValue=QN8027_ReadReg(0x4f);
	QN8027_WriteReg(0x4f,(TempRegValue/2));
}

void QN8027_Init(void)
{
    PortInit();// 模拟的端口先初始化
    QN8027_WriteReg(QndSystem1,0x81); //复位芯片寄存器
	QN8027_Delay(100); 
	//QndFreqModify=0;	//频率修正值,发射频率要比实际频率低0K	

	QN8027_WriteReg(0x03,0x3f);	//3f 设置QN8027为外部正旋波时钟输入( 与硬件设计相关)
	QN8027_WriteReg(0x04,0x51); //51 设置12MHz时钟频率( 与硬件设计相关)
	QndSetReg(0x00,0x40,0x40); //QN8027有限状态机校验
	QndSetReg(0x00,0x00,0x40);	//QN8027有限状态机校验

	QN8027_Delay(100);         //增加20ms的延迟来等待QN8027有限状态机校验的完成
	QN8027_WriteReg(0x18,0xe4); //改善信噪比SNR
	QN8027_WriteReg(0x1b,0xf0);	//使QN8027发射功率最大
	QN8027_WriteReg(0x11,200);	//使QN8027音量最大	
	//QN8027_WriteReg(0x10,0x1E);
	QN8027_WriteReg(0x1f,0x40);

	QndRdsMode(On);
	QN8027_WriteReg(0x02,0xb9);//0xa7 
	QN8027_WriteReg(0x00,0x22);// 0xEF
	QndMute(OFF);	
	QN8027_Delay(100); 	//延时10MS

	//QndSetReg(0x02,0x30,0x30);
	//QndCapBankTuning();
}



// -------------------END File----------------------------------------
