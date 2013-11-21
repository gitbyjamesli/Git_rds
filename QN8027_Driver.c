/*************************************************************

 ˵����
        ����Ĳ�������������QN8027��Ƶ����оƬ�ģ�����ʹ��ģ���i2c����
	PB10��PB11���š�

	2012-11-23/����




*/

// ͷ�ļ�-----------------------------------------------------------
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "all.h"
// �궨��-----------------------------------------------------------

// ������ģ���SDA��SCL���ڶ˿�
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
#include "i2c.h"// ����ģ���i2c��������

#define Chipid_QN8027     1



// ��������------------------------------------------------

u8 QN8027_error_flag;


static
void PortInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ������ʱ�� (APB2)*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, 
	                                    ENABLE);

	// ����LED�ƵĿ���IOģʽ����Ϊǿ����TX_LED(RDS����) -> PB13,  ER_LED(����) -> PB14
	//  I2C_SDA-> PB7,   I2C_SCL-> PB6

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_7|GPIO_Pin_6);// ����ߵ�ƽ
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
// 2.д������ַ 
    I2CWrite8Bit(0x58);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;
// 3.д�Ĵ�����ַ 
	I2CWrite8Bit(RegisAddr);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

// 4.��д������ַ���Ա㿪ʼ�� 
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
// 2.д������ַ 
    I2CWrite8Bit(0x58);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;
// 3.д�Ĵ�����ַ 
	I2CWrite8Bit(RegisAddr);
	QN8027_error_flag = I2CChkAck();
	if(QN8027_error_flag)goto stop;

// 4.��д������ַ ,�Ա㿪ʼ��
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



//����:�޸�QN80XX�Ĵ���ֵ
//����1:RegAddr����:�Ĵ�����ַ
//����2:DataByte����:д��Ĵ���������
//����3:BitMask����:д��Ĵ�������������
//���:��
//��д����:2011-02-03
//�޸�����:��
u8  TempRegData2;
void	QndSetReg(u8 RegAddr,u8 DataByte,u8 BitMask)
{
    u8 TempRegData;
    TempRegData2=QN8027_ReadReg(RegAddr);	//��ȡ�Ĵ���ֵ
    TempRegData2&=(u8)(~BitMask);		//����Ĵ�����ֵ
    TempRegData2|=DataByte&BitMask;		//д����ֵ���Ĵ�������
    QN8027_WriteReg(RegAddr,TempRegData2);	//������д�ؼĴ���
}



//����:QN80XX RDS����
//����:Mode����:1��RDS����,0�ر�RDS����
//���:��
//��д����:2011-02-03
//�޸�����:��
#define	QndRds		0x12
#define	RdsFdev		32	//RDS���ƶ�0-127
#define	QndGplt		0x02
#define	On	        0x01
#define	OFF	        0x00
#define	QndPaOffDisable	0x30
#define	QndSetT1mSel(Value)	QndSetReg(QndGplt,Value,0x30)
#define	QndSystem1	0x00	//ϵͳ��־�Ĵ���1
#define	QndCh1		0x01

void	QndRdsMode(u8	Mode)
{
#if	Chipid_QN8027
		if(Mode)
		{
			QN8027_WriteReg(QndRds,0x80|RdsFdev);	//����RDS
			//QndSetReg(QndGplt,0x40,0x40);	//����RDS����
		}
		else
		{
			QndSetReg(QndRds,0x00,(1<<7));	//��ֹRDS
//			QndSetReg(QndGplt,0,(1<<6));	//��ֹRDS����
		}
	
#endif
	
#if	Chipid_QN8035
		if(Mode)
		{
			QndWriteReg(QndIntCtrl,0xa8);
			//QndSetReg(QndIntCtrl,0x08,0x08);			
			QndSetReg(QndSystem1,(1<<3),(1<<3));	//����RDS

		}
		else
		{
			QndSetReg(QndSystem1,0x00,(1<<3));	//��ֹRDS
			QndSetReg(QndIntCtrl,0x00,0x08);
		}

#endif

}

//����:�޸�QN80XXƵ��
//����:ChannelByte����:10λƵ������
//���:��
//��д����:2011-02-03
//�޸�����:��
u8	TempChannel;
void	QNd8027_SetChannel(u16	ChannelByte)
{
	//u8	TempChannel;	
#if	Chipid_QN8027	//QN8027��ƵΪ64M������Ƶ�����õĲ���Ƶ��Ϊʵ�ʷ���Ƶ��	

	TempChannel=(u8)(ChannelByte>>8);	//ȡƵ����2λ
	Reg0_save=TempChannel;
	QndSetReg(QndSystem1,TempChannel,0x03);	//д��Ĵ���
	TempChannel=(u8)ChannelByte;			//ȡƵ����8λ
	QN8027_WriteReg(QndCh1,TempChannel);		//д��Ĵ���
	//Reg0_save=QN8027_ReadReg(0x0);
	/*
	TempChannel=QN8027_ReadReg(QndSystem1);
	TempChannel = (TempChannel & 0xFC) | ((ChannelByte&0X300)>>8);
	TempChannel &= 0xdf;//*set standby mode
	QN8027_WriteReg(QndSystem1,TempChannel);		//д��Ĵ���

	TempChannel=(u8)ChannelByte;			//ȡƵ����8λ
	QN8027_WriteReg(QndCh1,TempChannel);		//д��Ĵ���

	TempChannel=QN8027_ReadReg(QndSystem1);
	TempChannel |= 0x20;//*set transmit mode
	QN8027_WriteReg(QndSystem1,TempChannel);		//д��Ĵ���
	*/
#endif
	
}

//����:QN80XX��������
//����:Mode����:1��������,0�رվ���
//���:��
//��д����:2011-02-03
//�޸�����:��
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


//����:QN80XX���߳�ʼ��
void	QndCapBankTuning(void)
{
	u8	TempRegValue;
	QN8027_WriteReg(0x4f,0x80);
	TempRegValue=QN8027_ReadReg(0x4f);
	QN8027_WriteReg(0x4f,(TempRegValue/2));
}

void QN8027_Init(void)
{
    PortInit();// ģ��Ķ˿��ȳ�ʼ��
    QN8027_WriteReg(QndSystem1,0x81); //��λоƬ�Ĵ���
	QN8027_Delay(100); 
	//QndFreqModify=0;	//Ƶ������ֵ,����Ƶ��Ҫ��ʵ��Ƶ�ʵ�0K	

	QN8027_WriteReg(0x03,0x3f);	//3f ����QN8027Ϊ�ⲿ������ʱ������( ��Ӳ��������)
	QN8027_WriteReg(0x04,0x51); //51 ����12MHzʱ��Ƶ��( ��Ӳ��������)
	QndSetReg(0x00,0x40,0x40); //QN8027����״̬��У��
	QndSetReg(0x00,0x00,0x40);	//QN8027����״̬��У��

	QN8027_Delay(100);         //����20ms���ӳ����ȴ�QN8027����״̬��У������
	QN8027_WriteReg(0x18,0xe4); //���������SNR
	QN8027_WriteReg(0x1b,0xf0);	//ʹQN8027���书�����
	QN8027_WriteReg(0x11,200);	//ʹQN8027�������	
	//QN8027_WriteReg(0x10,0x1E);
	QN8027_WriteReg(0x1f,0x40);

	QndRdsMode(On);
	QN8027_WriteReg(0x02,0xb9);//0xa7 
	QN8027_WriteReg(0x00,0x22);// 0xEF
	QndMute(OFF);	
	QN8027_Delay(100); 	//��ʱ10MS

	//QndSetReg(0x02,0x30,0x30);
	//QndCapBankTuning();
}



// -------------------END File----------------------------------------
