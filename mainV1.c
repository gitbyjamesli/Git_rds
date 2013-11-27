/************************************************************************

 2012-11-21/����

*/
// �����ļ�----------------------------------------------------------------------

#include "stm32f10x.h"
#include "LCD19264_DriverV20.h"			   // 19264�ײ�
#include "DrawPicture.h"				   // ����19264�Ļ�ͼ
#include "PT2314.h"						   // ����������IC
#include "DAC5615.h"					   // DACоƬ����
#include "QN8027_Driver.h"				   // FM/RDS����оƬ����
#include "anjian.h"
#include <RTL.h>
#include <string.h>
#include <stdlib.h>


#include "show.h"
#include "my_datadef.h"                    // һЩ�������Ͷ��� , ��: ID������,ʱ������
#include "all.h"

// �궨��-------------------------------------------------------------------------
#define TBVAL_ADD 	1
#define TBVAL_SUB 	0

#define ER_LED_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_14)		   // ER_LED  ������
#define ER_LED_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_14)	   // ER_LED  �����
#define TX_LED_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_13)		   // Tx_LED  ������
#define TX_LED_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_13)	   // Tx_LED  �����
#define	BP_RING()      GPIO_ResetBits(GPIOD,GPIO_Pin_2)        // ������  �����
#define BP_MUTE()      GPIO_SetBits(GPIOD,GPIO_Pin_2)          // ������  ����

#define Pin_FAN_OPEN() GPIO_SetBits(GPIOC,GPIO_Pin_13)		   // ���ȿ��ƹܽ������	��������
#define Pin_FAN_CLOSE()  GPIO_ResetBits(GPIOC,GPIO_Pin_13)	   // ���ȿ��ƹܽ������    �ر�

#define Pin_PTT_OPEN()  GPIO_ResetBits(GPIOC,GPIO_Pin_0)	       // PTT���ƹܽ������		  ����PTT 
#define Pin_PTT_CLOSE() GPIO_SetBits(GPIOC,GPIO_Pin_0)		   // PTT���ƹܽ������		  �ر�PTT
#define PTT_EN		 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)//�ⲿʹ�ܶ�EN�ߵ�ƽ


#define ADC_VALUE_Vp   ADCConvertedValue[0]	// V+
#define ADC_VALUE_Vn   ADCConvertedValue[1]	// V-
#define ADC_VALUE_Vt   ADCConvertedValue[2]
#define ADC_VALUE_Vin  ADCConvertedValue[3]	//��Ƶ���뼤����ѹ
#define ADC_VALUE_Vi   ADCConvertedValue[4]				
uint32_t Vp_mv = 0,Vn_mv = 0;//V+��V-����λ����

uint16_t const  jiaozheng[]={160,150,140,130,125,120,110,100};
//ʹ��RTXϵͳ��һЩ�궨��
#define ALARM_START_EVENT    0x0001		   //      ��������ȴ��Ŀ�ʼ�ź�
#define ALARM_END_EVENT      0x0002		   //      ��������ȴ��Ľ����ź�

#define PC_START_EVENT    0x0001		   //      ��������ȴ��Ŀ�ʼ�ź�
#define PC_END_EVENT      0x0002		   //      ��������ȴ��Ľ����ź�

#define RDS_START_EVENT    0x0001		   //      ��������ȴ��Ŀ�ʼ�ź�
#define RDS_END_EVENT      0x0002		   //      ��������ȴ��Ľ����ź�

#define FALSE   0		   //      ��������ȴ��Ŀ�ʼ�ź�
#define OK      1		   //      ��������ȴ��Ľ����ź�

uint16_t dav_save=0;//�¶ȹ��ϻָ�ʱ��
uint16_t Power_set_save=0;//�¶ȹ��ϻָ�ʱ��
//#define RDS_DEVICES_BIG_SPK 3
//#define RDS_DEVICES_MIDDLE_SPK 2
//#define RDS_DEVICES_SMALL_SPK 1
//#define RDS_DEVICES_OFF_SPK 0

 //������
/*
unsigned char const  Tiaoyin[]={0x00,0x01,0x02,0x03,
                      0x04,0x05,0x06,0x07,
                      0x0f,0x0e,0x0d,0x0c,
                      0x0b,0x0a,0x09,0x08};
*/
// �ҵ��������Ͷ���
typedef struct
{
	uint8_t x;
	uint8_t y;
}PositDef;// ����ṹ���Ͷ���


// ȫ�ֱ���-----------------------------------------------------------------------

static U64         main_task_stack[800/8];// ��������Ķ�ջ
OS_TID             tid_main_task;
OS_TID             tid_alarm_task;
OS_TID             tid_ComDebug;
OS_MUT             LCD_mutex;// Һ���������Ļ�����
OS_TID             tid_AutoFanControl;// �Զ����ȿ��������ID
OS_TID             tid_AutoAdjustPower;// �Զ����ʵ��������ID
OS_TID             tid_AutoChoiceAudio;// �Զ����ʵ��������ID
OS_TID             tid_PCRDS_task;// �Զ����ʵ��������ID
OS_TID             tid_comdata_task;// �����ID
// ��λ�����ڵı���
USART_InitTypeDef   USART1_InitStructure; // ���   ����1�����ò���

FlagStatus            packet_head = RESET;    //֡ͷ����ʼ���ձ�־
FlagStatus            packet_head2 = RESET;    //֡ͷ����ʼ���ձ�־
FlagStatus            packet_R_OK = RESET;    //������ϱ�־   SET��Ч
FlagStatus            packet_R_OK2 = RESET;    //������ϱ�־   SET��Ч
uint8_t               packet_rct;             //�����ֽڼ���
uint8_t               packet_rct2;             //�����ֽڼ���
uint8_t               packet_len;             //�����ֽ�
uint8_t               packet_len2;             //�����ֽ�
uint8_t               packet_rbuf[64];        //���յ����ݻ���
uint8_t               packet_rbuf2[32];        //���յ����ݻ���
uint8_t               packet_xor;
uint8_t               packet_xor2;

FlagStatus            packet_T_OK = SET;    //������ɱ�־ 0���� 1������
uint8_t               packet_sendsize;        //���Ͱ���С
uint8_t               packet_tct;
uint8_t               packet_tbuf[32];        //���͵����ݻ���

FlagStatus            packet_T_OK2 = RESET;    //������ɱ�־ 0���� 1������
uint8_t               packet_sendsize2;        //���Ͱ���С
uint8_t               packet_tct2;
uint8_t               packet_tbuf2[2];        //���͵����ݻ���
// ���ջ����ڵı��� ���������ر𴮿ڵ����õ���4��������
USART_InitTypeDef    USART2_InitStructure; // ����2�����ò���
uint8_t              Rxbyte_ct;
uint8_t              Rxbuf[3];
uint8_t              Txbyte;
FlagStatus           S2_receive_flag;// ���ڽ��ձ�־


// ����ȫ�ֶ���
struct TimeTypeDef   SystemTime={0,0,0}; // ʱ��ṹ    ʱ/��/��
FlagStatus           SystemTime_SecUpdate_Flag;       // ����±�־
uint8_t              Volume_value=32;    // ������ֵ		ֵ��	0~31
uint8_t              Bass_value=7;       // ����ֵ			ֵ��	0~14
uint8_t              Treble_value=7;     // ����ֵ			ֵ��	0~14
uint8_t              PE_value=50;        // PEֵ            ֵ��    50��75
uint8_t              TP_set=99;        // �¶ȱ���ֵ		ֵ��    60~80
uint8_t              SWRP_set=25;      // SWR����ֵ       ֵ��    20~30
uint8_t              IP_value=3;         // ��������ֵ		ֵ��    ???(�ݲ��� !)
uint8_t              PS_Name[10][8]={"ABCDEFGH"};	 // 10��PS��
uint8_t              Area_Name[20][7]={"BCDEFGH"};   // 20��������
uint8_t              RDS_DevicesGroup_Mode[24][16];  // ����������豸״̬��ʱ���档ע�⣬ÿ��15���նˣ����һ���ֽ�������

uint32_t             FM_value=101700;      // ��ǰFMƵ��ֵ
uint16_t             Power_set=20;	   // ���õĹ���ֵ
uint16_t             Power_value=0;	       // ��ǰ������ֵ
uint16_t             RP_value=100;         // ��ǰ������ֵ
uint8_t              SWR_value=0;		   // ��ǰSWR
uint16_t              I_value=0;		       // ��ǰ����
uint8_t              T_value=25;		   // ��ǰ�¶�
uint16_t             Excitation_V=0;     // ��ǰ������ѹֵ

uint32_t           	 sendstate_Time_Count;
uint32_t             Menu_Timeout_Count;    // �˵�������ʱ��������  ��TIM3�� -1 ����
uint16_t             CursorBlink_Count;     // ��귭ת����ʱ		 ��TIM3�� -1 ����
uint16_t             CollectUpdate_Interval;// �¶�ֵ���¼������
uint32_t             sound_ch_switch_Timeout_Count=0;    // ����ͨ���л���ʱ����  ��TIM3�� -1 ����

FlagStatus           PTT_status;           // SET״̬Ϊ�����У�RESETΪ�ر�
FlagStatus           TX_status=RESET;           // SET״̬Ϊ�����У�RESETΪ�ر�
FlagStatus           setsys_para=RESET;           // SET״̬Ϊ�����У�RESETΪ�ر�
FlagStatus           menu_set=RESET; 

FlagStatus           HighTempWarning_Flag; // ���±���״̬��SET��ʾ�Ѿ�������RESET��ʾδ���б���
FlagStatus           HighCURRENTWarning_Flag; // ��������״̬��SET��ʾ�Ѿ�������RESET��ʾδ���б���
FlagStatus           HighSWRWarning_Flag; // ��������״̬��SET��ʾ�Ѿ�������RESET��ʾδ���б���
FlagStatus           HighPRWarning_Flag;
FlagStatus           HighLPWarning_Flag;
FlagStatus           VinWarning_Flag; //SET��ʾ����0.2V

FlagStatus           VIN_AND_EN=FALSE; // ��������״̬��SET��ʾ�Ѿ�OK��RESET��ʾδ�ã�
FlagStatus           fm_rf_set = RESET; //����״̬ 
FlagStatus           tp_adjust_p = RESET; // 
uint32_t fmval_bak = 0;//FM_value;
uint16_t powerval_bak = 0;//Power_set;


const uint8_t ZF_Table[4][22]=
{
	{' ','!','"','#','$','%','&',0x27,'(',')','*','+',',','-','.','/','0','1','2','3','4','5'},
	{'6','7','8','9',':',';','<','=','>','?','@','A','B','C','D','E','F','G','H','I','J','K'},
	{'L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','[',0x5c,']','a','b','c','d'},
	{'e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'}
};


uint8_t pty_num=0;                // 
uint8_t area_num=0;                // �����ֵ
uint8_t rds_state=0;                // rds״̬��0-ȫ�أ�1ȫ����2-����

uint8_t PCRDS_Blinking_flag=OFF;
uint16_t Disconnect_pc=0;






// ����ԭ��------------------------------------------------------------------------
__task void InitTask(void);// ��һ����ʼ���е�����
__task void main_task(void);// ������
__task void Alarm_task(void);// ������������ʾ����
__task void ComDebug_task(void);
__task void RDS_task(void);// С���RDS����������

__task void AutoFanControl_task(void);// �Զ�������ͣ��������
__task void AutoAdjustPower_task(void);
__task void AutoChoiceAudio_task(void);//�Զ���Ƶѡ��
__task void PCRDS_Blinking_task(void);//�Զ���Ƶѡ��
__task void comdata_task(void);//����������

void Delay_ms(u32 nCount)// �ղ�����ʱ����
{
	u32 z;
	while(nCount--)
	{
		for(z=1000;z>0;z--)
		{
			;
		}
	}
}
void KEY_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* ʹ������ʱ�� (APB2)*/
	RCC_APB2PeriphClockCmd(   RCC_APB2Periph_GPIOA
	                         |RCC_APB2Periph_GPIOB
	                         |RCC_APB2Periph_GPIOC
	                         |RCC_APB2Periph_GPIOC
	                         |RCC_APB2Periph_GPIOD
						     |RCC_APB2Periph_AFIO		// ���õ� AFIO   �����ܸ���IO��
						   , ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;// ��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0
	                             |GPIO_Pin_1
								 |GPIO_Pin_2
								 |GPIO_Pin_5
								 |GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;// ��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);// 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);// �ⲿ�ж�0   ѡ��GPIOB   -> PB.0
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);// 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);//
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);//
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);//

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     // 
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // �½���  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // ʹ��
    EXTI_InitStructure.EXTI_Line = EXTI_Line0
	                              |EXTI_Line1
	                              |EXTI_Line2
	                              |EXTI_Line7
	                              |EXTI_Line12
	                              |EXTI_Line4
								  |EXTI_Line5;
	                              //|EXTI_Line10
								  //|EXTI_Line11;
    EXTI_Init(&EXTI_InitStructure);
	/*
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);//
    //GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     // 
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // ������  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // ʹ��
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_Init(&EXTI_InitStructure);
	*/
// ����Ϊ����io������


	// TX_LED -> PB13  ,   ER_LED -> PB14   ������Ϊǿ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14);// ����ָʾ�Ƴ�ʼ״̬Ϊ��	 ��ʵ���ϴ�ʱMCU�Ѿ����Ϊ0
	// PC13  ǿ��   PTT���ƹܽ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	Pin_FAN_CLOSE();
	Pin_PTT_CLOSE();


	// PD2  ��©���   ���������ƹܽ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	BP_MUTE();                                  // �Ⱦ���

	
	// PA8����Ϊʱ��������������ŵĸ��ù��ܣ�
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	RCC_MCOConfig(RCC_MCO_HSE); 


}

void RTC_Configuration(void)
{
	/* Enable PWR and BKP clocks */
	/* PWRʱ�ӣ���Դ���ƣ���BKPʱ�ӣ�RTC�󱸼Ĵ�����ʹ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	
	/* Allow access to BKP Domain */
	/*ʹ��RTC�ͺ󱸼Ĵ������� */
	PWR_BackupAccessCmd(ENABLE);
	
	/* Reset Backup Domain */
	/* ������BKP��ȫ���Ĵ�������Ϊȱʡֵ */
	BKP_DeInit();
	
	/* Enable LSE */
	/* ʹ��LSE���ⲿ32.768KHz���پ���*/
	RCC_LSEConfig(RCC_LSE_ON);
	
	/* Wait till LSE is ready */
	/* �ȴ��ⲿ�������ȶ���� */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
	
	/* Select LSE as RTC Clock Source */
	/*ʹ���ⲿ32.768KHz������ΪRTCʱ�� */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	
	/* Enable RTC Clock */
	/* ʹ�� RTC ��ʱ�ӹ��� */
	RCC_RTCCLKCmd(ENABLE);
	
	/* Wait for RTC registers synchronization */
	/*�ȴ�RTC�Ĵ���ͬ�� */
	RTC_WaitForSynchro();
	
	/* Wait until last write operation on RTC registers has finished */
	/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
	RTC_WaitForLastTask();
	
	/* Enable the RTC Second */
	/* ʹ��RTC�����ж� */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	
	/* Wait until last write operation on RTC registers has finished */
	/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
	RTC_WaitForLastTask();
	
	/* Set RTC prescaler: set RTC period to 1sec */
	/* 32.768KHz����Ԥ��Ƶֵ��32767,����Ծ���Ҫ��ܸ߿����޸Ĵ˷�Ƶֵ��У׼���� */
	RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
	
	/* Wait until last write operation on RTC registers has finished */
	/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
	RTC_WaitForLastTask();
}

void delayms(void)
{
	unsigned int n;
	for(n=3000000;n>0;n--);
}
void COM1_2Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;      //DMA��ʼ���ṹ������

	// 1. ����1/2  ��Ҫʹ��GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ����1/2  ��Ҫʹ��GPIOA


	// 2. ����2����ĳ�ʼ������
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);// ����2����ʱ��   ��
	
	// USART2�� Rx  (PA3)->����   Tx  (PA2)->���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART2_InitStructure.USART_BaudRate = 9600;
	USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART2_InitStructure.USART_StopBits = USART_StopBits_1;
	USART2_InitStructure.USART_Parity = USART_Parity_No;
	USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART2_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);// ʹ�ܽ����ж�
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// ʹ�ܷ����ж�
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);// �����ж�  ��ֹ
	//USART_ITConfig(USART2, USART_IT_TXE, DISABLE); // �����ж�  ��ֹ
	USART_Cmd(USART2, ENABLE);                     // ��󴮿�2ʹ��

	/*
	// DMA1 channel1 configuration ----------------------------------------------
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel7);                  // ������
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40004404;//USART2->DR;    //DMA��Ӧ���������ַ	   //0x40004404;//
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)packet_tbuf;          //�ڴ�洢����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	                   //DMA��ת��ģʽΪDSTģʽ�����ڴ���Ƶ�����
	DMA_InitStructure.DMA_BufferSize = 64;		                           //DMA�����С��1��,��λΪDMA_MemoryDataSize
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	   //����һ�����ݺ��豸��ַ��ֹ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                //����һ�����ݺ�Ŀ���ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�����������ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        //DMA�����ݳߴ磬8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                        //ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                   //DMA���ȼ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		                   //M2Mģʽ����
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);// �ٽ�����д��          
	

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); // ����2�ķ���DMAʹ��
	DMA_Cmd(DMA1_Channel7, ENABLE);
	*/
//	while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);

///*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);// ����1����ʱ��   ��

	// USART1�� Rx(PA10)->��������   Tx(PA9)->���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;   //
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);



	USART1_InitStructure.USART_BaudRate = 115200;//9600;
	USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART1_InitStructure.USART_StopBits = USART_StopBits_1;
	USART1_InitStructure.USART_Parity = USART_Parity_No;
	USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART1_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);// ʹ�ܽ����ж�
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);// ʹ�ܷ����ж�
	USART_Cmd(USART1, ENABLE);// ���ʹ�ܴ���1 

}

/***************************************************************
 ���ܣ�ͨ�ö�ʱ��3����Ϊ�жϷ�ʽ/10ms
 ���룺��
 �������
 ˵�����˺�����ǰ������ѧϰ�����ã�ʱ����������
*/

void TIM3Init(void)// ͨ�ö�ʱ��3���ã�ʹ���жϷ�ʽ, 10ms��ʱ
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);// ��ʱ��3 ʱ��ʹ��
	
	TIM_DeInit(TIM3);// �Ȼָ�Ĭ��״̬

	// �������� 
	
	TIM_TimeBaseStructure.TIM_Period = (10*1000 - 1);// �Զ�װ�ؼĴ���ֵ�� ��λus
	
	TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);// Ԥ��Ƶϵ���� 
	
	// �߼�Ӧ�ñ��β��漰�������ڶ�ʱ��ʱ��(CK_INT)Ƶ���������˲���(ETR,TIx)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;// ��Ƶ��Ϊ1
	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// ���ϼ���
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);// д������ֵ
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);// ���TIM3����жϱ�־
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);// ��������ж�
	
	TIM_Cmd(TIM3, ENABLE);  // ������ʹ�ܣ���ʼ����
}
/******************************************************************
  Ƕ���жϿ�����NVIC�����ú���������ϵͳ����ʹ�õ��жϰ�һ�������ȼ���
  �ý������ã���ϸ���ü���������
  
  ע�⣺���������ж��������ﶨ�����ȼ���
*******************************************************************/
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	// Configure the NVIC Preemption Priority Bits 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);// ����0���ʽ    ��4λȫ���������ȼ������ȼ�����Ч��
	
	// Enable the USART2 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;// ͨ����
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// ���ȼ�0      ����ʱ����Ч�ģ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;// �����ȼ�4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);// ��������

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
/*	
	// Enable the TIM3 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// ���ȼ� 0  ����ʱ����Ч�ģ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;// �����ȼ� 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);// ��������
*/	
	// Enable the RTC gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// ���ȼ� 0  ����ʱ����Ч�ģ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;// �����ȼ� 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);// ��������

	// Enable the TIM3 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// ���ȼ� 0  ����ʱ����Ч�ģ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;// �����ȼ� 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);// ��������

}

void KEY_Status_AllClear(void)
{
	KEY_UP_CLEAR();
	KEY_DOWN_CLEAR();
	KEY_LEFT_CLEAR();
	KEY_RIGHT_CLEAR();
	KEY_OK_CLEAR();
	KEY_MENU_CLEAR();
	KEY_RDS_CLEAR();
}
void UpdateSystemTime(void)// ����ϵͳʱ��
{
	uint32_t rtc_value = RTC_GetCounter();

	SystemTime.hour = rtc_value/3600%24;		
	SystemTime.mins = rtc_value%3600/60;
	SystemTime.sec = rtc_value%60;
}
void SetupSystemTime(struct TimeTypeDef *time)// ����RTCʱ��
{
	uint32_t value;

	value = time->hour*3600 + time->mins*60 + time->sec;

	RTC_WaitForLastTask();/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */

	RTC_SetCounter(value);

	RTC_WaitForLastTask();/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
}
/*******************�ָ���*****************************************
 ��Ϊϵͳ�������ý���ĺ�����FM�����ʡ�������ʱ�䡢�¶ȵ�����
***/
void SET_FM_And_RF_Power(void)
{
    u8 fm_change=0;  
	u8 set_target=1;           // ����Ŀ��
	FlagStatus gb_flag=RESET;  // ���״̬  SET����RESET��

	KEY_Status_AllClear();// ���ȫ���������±�־
	Draw_Fill(1,39,190,63,0);// ��һ�հ�����

	Draw_String6X8(1,39,"SET FM AND RF POWER");
	Draw_String6X8(20,47,"FM:     MHz AutoSet: ");
	Draw_String6X8(20,55,"RF POWER:    W");

showpara:
	Menu_Timeout_Count=1000; // �˵���ʱ��ι����10s
	CursorBlink_Count=50;    // �����˸��ʱ  ����500ms
// FMƵ��ֵ
	SetingWindos_ShowFMchValue(FM_value);
// ����ֵ
	SetingWindos_ShowPowerValue(Power_set);
// �Զ�ƥ��
	if(set_target==2)
	{
		Draw_Ascii6X8(20+6*20,47,'>');
	}
	else
	{
		Draw_Ascii6X8(20+6*20,47,' ');
	}
// �˳�ͼ��
	if(set_target==4)
	{
		Draw_String6X8(160,55,">Exit");
	}
	else
	{
		Draw_String6X8(160,55," Exit");
	}

	while(1)
	{
		if(SystemTime_SecUpdate_Flag == SET)// ϵͳʱ�������
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// ��ʾʱ��
		}

		if(KEY_LEFT_STATUS() == SET)
		{
			if(--set_target < 1)
				set_target=4;
			goto showpara;
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			if(++set_target > 4)
				set_target=1;
			goto showpara;
		}
		if(KEY_UP_STATUS() == SET)
		{
			UP_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			switch (set_target)
			{
				case 1:
					FM_value+=100;// 100KHz ����
					if(FM_value>108000)
					{
						FM_value = 76000;
					}
					psys_data->ui_FM_value=FM_value;
					fm_change=1;
					//QNd8027_SetChannel((FM_value-76000)/50);
					break;
				case 2:break;
				case 3:
					if(Power_set<150)
					{
						Power_set++;
						psys_data->ui_RF_POWER=Power_set;
					}
					break;
			}
			goto showpara;
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			DOWN_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			switch (set_target)
			{
				case 1:
					FM_value-=100;// 100KHz ����
					if(FM_value<76000)
					{
						FM_value = 108000;
					}
				   psys_data->ui_FM_value=FM_value;
				   fm_change=1;
				   //QNd8027_SetChannel((FM_value-76000)/50);
					break;
				case 2:break;
				case 3:
					if(Power_set>0)
					{
						Power_set--;
						psys_data->ui_RF_POWER=Power_set;
					}
					break;
			}
			goto showpara;
		}
		if(KEY_MENU_STATUS() == SET)
		{
		    SYS_PARAMETER_Save();
			break;
		}
		if(KEY_OK_STATUS() == SET)
		{
		    SYS_PARAMETER_Save();
			if(set_target==4)
			 {
			    while(KEY_OK_STATUS() == SET);
				break;
				}
		}
		if(Menu_Timeout_Count==0)// �˵�������ʱ
		{
		   SYS_PARAMETER_Save(); //��ʱ�˳�Ҳ����
			break;
		}
		if(CursorBlink_Count==0)// �����ʱ�䵽
		{
			CursorBlink_Count=50;// ����500ms
			if(gb_flag)
			{
				gb_flag = RESET;
				switch (set_target)
				{
					case 1:SetingWindos_DestroyFMchValue();break;
						
					case 3:SetingWindos_DestroyPowerValue();break;
						
					default :break;
				}
			}
			else
			{
				gb_flag = SET;
				switch (set_target)
				{
					case 1:SetingWindos_ShowFMchValue(FM_value);break;
						
					case 3:SetingWindos_ShowPowerValue(Power_set);break;
						
					default :break;
				}
			}
		}

	 if(fm_change==1&&DOWN_LockFlag==RESET)//�ɿ�����ʱ������
	  {
	   fm_change=0;
	   QNd8027_SetChannel((FM_value-76000)/50);
	   }
	}

}

void TBval_Change(u8 *TBval,u8 addsub)// ������ĸ���(����ֵ)   �ӻ��1
{
									// �㷨��ο�PT2314��Bass/Trebleȡֵ����
	if( (*TBval) & 0x08)
	{
		if(addsub)// ��������
		{
			if(0 != ((*TBval)&0x07))    (*TBval)--;
				
		}
		else// ��������
		{
			if(0x07 == ((*TBval)&0x07))
			{
				(*TBval) = 0x07;
			}
			else
			{
				(*TBval)++;
			}
		}
	}
	else
	{
		if(addsub)// �Ӳ���
		{
			if(0x07 == ((*TBval)&0x07))
			{
				(*TBval) = 0x0f;
			}
			else
			{
				(*TBval)++;
			}
		}
		else // ������
		{
			if( 0 != ((*TBval)&0x07))  (*TBval)--;
		}
	}
}

void SET_Audio_And_Time(void)
{
	uint8_t set_target=1;
	uint8_t petmp=0;						// PEֵ   0 ->50us   1->75us
	uint8_t disnum;
	FlagStatus gb_flag = RESET;
	struct TimeTypeDef  temp_time;			// ��ʱ���ʱ���޸�ֵ	

	UpdateSystemTime();
	temp_time.sec = SystemTime.sec;
	temp_time.mins = SystemTime.mins;
	temp_time.hour = SystemTime.hour;

	KEY_Status_AllClear();// ���ȫ���������±�־
	Draw_Fill(1,39,190,63,0);// ��һ�հ�����

	Draw_String6X8(1,39,"SET Audio AND Time");
	Draw_String6X8(20,47,"VOL:--PE:--us H:--");
	Draw_String6X8(20,55,"Time --:--:-- B:--");

show_para:
	Menu_Timeout_Count=1000;// �˵���ʱ��ι����10s
	CursorBlink_Count=50;   // �����˸��ʱ 500ms
// ��������ʾ
	Draw_Ascii6X8(20+6*4,47,Volume_value/10+'0');
	Draw_Ascii6X8(20+6*5,47,Volume_value%10+'0'); 
// PEֵ����ʾ
	if(PE_value==50)
	{
		Draw_Ascii6X8(20+6*9,47,'5');	 //50 us
		Draw_Ascii6X8(20+6*10,47,'0'); 
	}
	else
	{
		Draw_Ascii6X8(20+6*9,47,'7');  //75 us
		Draw_Ascii6X8(20+6*10,47,'5');
	}
// ����ֵ����ʾ
    if(Treble_value>7)
         disnum = 23-Treble_value;// dBֵ
	else disnum=Treble_value;
	Draw_Ascii6X8(20+6*16,47,disnum/10+'0');
	Draw_Ascii6X8(20+6*17,47,disnum%10+'0');
// ����ֵ����ʾ
    if(Bass_value>7)
         disnum = 23-Bass_value;// dBֵ
	else disnum=Bass_value;
	Draw_Ascii6X8(20+6*16,55,disnum/10+'0');
	Draw_Ascii6X8(20+6*17,55,disnum%10+'0');
// Сʱֵ����ʾ
	Draw_Ascii6X8(20+6*5,55,temp_time.hour/10+'0');
	Draw_Ascii6X8(20+6*6,55,temp_time.hour%10+'0');
// ����ֵ����ʾ
	Draw_Ascii6X8(20+6*8,55,temp_time.mins/10+'0');
	Draw_Ascii6X8(20+6*9,55,temp_time.mins%10+'0');
// ����ֵ����ʾ
	Draw_Ascii6X8(20+6*11,55,temp_time.sec/10+'0');
	Draw_Ascii6X8(20+6*12,55,temp_time.sec%10+'0');
// ��ʾ�˳���ť״̬
	if(set_target==8)
	{
		Draw_String6X8(160,55,">Exit");
	}
	else
	{
		Draw_String6X8(160,55," Exit");
	}
	while(1)
	{
		if(SystemTime_SecUpdate_Flag == SET)// ϵͳʱ�������
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// ��ʾʱ��
		}

		if(KEY_LEFT_STATUS() == SET)
		{
			if(--set_target < 1)
			{
				set_target=8;
			}
			goto show_para;
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			if(++set_target>8)
			{
				set_target=1;
			}
			goto show_para;
		}
		if(KEY_UP_STATUS() == SET)
		{
			UP_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			switch (set_target)
			{
				case 1:// ������
					if(Volume_value<32)Volume_value++;
					PT2314_Setup_Volume(Volume_value);
					psys_data->ui_Audio_VOL=Volume_value;
					break;
				case 2:
					if(PE_value==50)
					{
						PE_value=75;
						psys_data->ui_pe_value=PE_value;
					}
					else
					{
						PE_value=50;
						psys_data->ui_pe_value=PE_value;
					}
					break;
				case 3:
					//if(Treble_value<14)Treble_value++;
					TBval_Change(&Treble_value,TBVAL_ADD);// ϵͳ����ֵ��1
					PT2314_Setup_Treble(Treble_value);
					psys_data->ui_Audio_H=Treble_value;
					break;
				case 4:
					if(++temp_time.hour>23)temp_time.hour=0;
					break;
				case 5:
					if(++temp_time.mins>59)temp_time.mins=0;
					break;
				case 6:
					if(++temp_time.sec>59)temp_time.sec=0;
					break;
				case 7:
					//if(Bass_value<14)Bass_value++;
					TBval_Change(&Bass_value,TBVAL_ADD);// ϵͳ������ֵ��1
					PT2314_Setup_Bass(Bass_value);
					psys_data->ui_Audio_B=Bass_value;
					break;
				default :break;
			}
			goto show_para;
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			DOWN_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			switch (set_target)
			{
				case 1:// ������
					if(Volume_value>0)Volume_value--;
					PT2314_Setup_Volume(Volume_value);
					psys_data->ui_Audio_VOL=Volume_value;
					break;
				case 2:
					if(PE_value==50)
					{
						PE_value=75;
						psys_data->ui_pe_value=PE_value;
					}
					else
					{
						PE_value=50;
						psys_data->ui_pe_value=PE_value;
					}
					break;
				case 3:
					//if(Treble_value>0)Treble_value--;
					TBval_Change(&Treble_value,TBVAL_SUB);// ϵͳ����ֵ��1
					PT2314_Setup_Treble(Treble_value);
					psys_data->ui_Audio_H=Treble_value;
					break;
				case 4:
					if(--temp_time.hour>24)temp_time.hour=23;
					break;
				case 5:
					if(--temp_time.mins>60)temp_time.mins=59;
					break;
				case 6:
					if(--temp_time.sec>60)temp_time.sec=59;
					break;
				case 7:
					//if(Bass_value>0)Bass_value--;
					TBval_Change(&Bass_value,TBVAL_SUB);// ϵͳ����ֵ��1
					PT2314_Setup_Bass(Bass_value);
					psys_data->ui_Audio_B=Bass_value;
					break;
				default :break;
			}
			goto show_para;
		}
		if(KEY_OK_STATUS() == SET)
		{
		   SYS_PARAMETER_Save();
			if(set_target==8)
			{
				SetupSystemTime(&temp_time);// д��RTC��
				while(KEY_OK_STATUS() == SET);
				break;
			}
		}
		if(KEY_MENU_STATUS() == SET)
		{
		    SYS_PARAMETER_Save();
			break;
		}
		if(CursorBlink_Count==0)
		{
	      CursorBlink_Count=50;
			if(gb_flag)
			{
				gb_flag = RESET;
				switch (set_target)
				{
					case 1:// ������  
						Draw_Fill(44,    47,    55,      54,      0);// ���հ�
						break;
					case 2:// peֵ
						Draw_Fill(20+6*9,  47,   74+11,    54,   0);// ���հ�
						break;
					case 3:// ����
						Draw_Fill(116,    47,    116+11,  54,    0);// ���հ�
						break;
					case 4:// Сʱ
						Draw_Fill(50,        55,    61,     63,     0);// ���հ�
						break;
					case 5:// ����
						Draw_Fill(68 ,       55,    79,    63,     0);// ���հ�
						break;
					case 6:// ��
						Draw_Fill(86,        55,    97,    63,      0);// ���հ�
						break;
					case 7:// ����
						Draw_Fill(116,   55,    116+11,   63,   0);// ���հ�
						break;
					default :break;
				}
				
			}
			else
			{
				gb_flag = SET;
				//goto show_para;
				switch (set_target)
				{
					case 1:
					// ��������ʾ
						Draw_Ascii6X8(20+6*4,47,Volume_value/10+'0');
						Draw_Ascii6X8(20+6*5,47,Volume_value%10+'0'); 
					 break;
					case 2:
					// PEֵ����ʾ
						if(PE_value==50)
						{
							Draw_Ascii6X8(20+6*9,47,'5');	 //50 us
							Draw_Ascii6X8(20+6*10,47,'0'); 
						}
						else
						{
							Draw_Ascii6X8(20+6*9,47,'7');  //75 us
							Draw_Ascii6X8(20+6*10,47,'5');
						}
					  break;
					  case 3:
					// ����ֵ����ʾ
					    if(Treble_value>7)
					         disnum = 23-Treble_value;// dBֵ
						else disnum=Treble_value;
						Draw_Ascii6X8(20+6*16,47,disnum/10+'0');
						Draw_Ascii6X8(20+6*17,47,disnum%10+'0');
					  break;
					  case 7:
					// ����ֵ����ʾ
					    if(Bass_value>7)
					         disnum = 23-Bass_value;// dBֵ
						else disnum=Bass_value;
						Draw_Ascii6X8(20+6*16,55,disnum/10+'0');
						Draw_Ascii6X8(20+6*17,55,disnum%10+'0');
					  break;
					case 4:
					// Сʱֵ����ʾ
						Draw_Ascii6X8(20+6*5,55,temp_time.hour/10+'0');
						Draw_Ascii6X8(20+6*6,55,temp_time.hour%10+'0');
					  break;
					case 5:
					// ����ֵ����ʾ
						Draw_Ascii6X8(20+6*8,55,temp_time.mins/10+'0');
						Draw_Ascii6X8(20+6*9,55,temp_time.mins%10+'0');
					  break;
					case 6:
					// ����ֵ����ʾ
						Draw_Ascii6X8(20+6*11,55,temp_time.sec/10+'0');
						Draw_Ascii6X8(20+6*12,55,temp_time.sec%10+'0');
					break;	
					default :break;
				}
			}
		}
		if(Menu_Timeout_Count==0)// �˵�������ʱ
		{	
		    SYS_PARAMETER_Save();
			break;
		}
	}

}
void SET_Temp_And_SWR(void)
{
	uint8_t set_target=1;
	FlagStatus gb_flag=RESET;
	//uint8_t swr_value=20;// SWR����ֵ      ע�⣺������С��ת��������

	KEY_Status_AllClear();// ���ȫ���������±�־
	Draw_Fill(1,39,190,63,0);// ��һ�հ�����

	Draw_String6X8(1,39,"SET Temp AND SWR");
	Draw_String6X8(20,47,"HIGH TEMP:");Show_Picture(20+6*12,47,FH_du8X8,8,8);// ��ʾ���� ��C
	Draw_String6X8(20,55,"HIGH SWR:");
showpara:
	Menu_Timeout_Count=1000;// �˵���ʱ��ι����10s
	CursorBlink_Count=50;   // �����˸��ʱ 500ms

// ���� HIGH TEMP����ֵ	
	Draw_Ascii6X8(20+6*10,47,TP_set/10+'0');
	Draw_Ascii6X8(20+6*11,47,TP_set%10+'0');
// SER����ֵ
	Draw_Ascii6X8(20+6*9,55,SWRP_set/10+'0');
	Draw_Ascii6X8(20+6*10,55,'.');
	Draw_Ascii6X8(20+6*11,55,SWRP_set%10+'0');
// ��ʾ�˳���ť״̬
	if(set_target==3)
	{
		Draw_String6X8(160,55,">Exit");
	}
	else
	{
		Draw_String6X8(160,55," Exit");
	}
	while(1)
	{
		if(SystemTime_SecUpdate_Flag == SET)// ϵͳʱ�������
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// ��ʾʱ��
		}

		if(KEY_LEFT_STATUS() == SET)
		{
			if(--set_target==0)
			{
				set_target=3;
			}
			goto showpara;
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			if(++set_target>3)
			{
				set_target=1;
			}
			goto showpara;
		}
		if(KEY_UP_STATUS() == SET)
		{
			UP_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			switch (set_target)
			{
				case 1:
					if(++TP_set>80)
						TP_set=60;
					psys_data->ui_TEMP=TP_set;
					break;
				case 2:
					if(++SWRP_set>30)
						SWRP_set=20;
					psys_data->ui_SWR=SWRP_set;
					break;
				default :break;
			}
			goto showpara;
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			DOWN_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			switch (set_target)
			{
				case 1:
					if(--TP_set<60)
						TP_set=80;
					psys_data->ui_TEMP=TP_set;
					break;
				case 2:
					if(--SWRP_set<20)
						SWRP_set=30;
					psys_data->ui_SWR=SWRP_set;
					break;
				default :break;
			}
			goto showpara;
		}
		if(KEY_OK_STATUS() == SET)
		{	
		    SYS_PARAMETER_Save();
			if(set_target==3)
			 {
			    while(KEY_OK_STATUS() == SET);
				break;
				}
		}
		if(KEY_MENU_STATUS() == SET)// �˵���==���ؼ�
		{
		    SYS_PARAMETER_Save();
			break;
		}
		if(CursorBlink_Count==0)
		{
		  CursorBlink_Count = 50;
			if(gb_flag==SET)
			{
				gb_flag = RESET;
				
				switch (set_target)
				{
					case 1:Draw_Fill(80,   47,    91,   54,   0);;// ���հ�
						break;
					case 2:Draw_Fill(74,   55,    91,   63,   0);;// ���հ�
						break;
					default :break;
				}
			}
			else
			{
				gb_flag = SET;
				//goto showpara;
				switch (set_target)
				{
					case 1:
					// ���� HIGH TEMP����ֵ	
						Draw_Ascii6X8(20+6*10,47,TP_set/10+'0');
						Draw_Ascii6X8(20+6*11,47,TP_set%10+'0');
					 break;
					case 2:
					// SER����ֵ
						Draw_Ascii6X8(20+6*9,55,SWRP_set/10+'0');
						Draw_Ascii6X8(20+6*10,55,'.');
						Draw_Ascii6X8(20+6*11,55,SWRP_set%10+'0');
					  break;	
					default :break;
				}
			}
		}
		if(Menu_Timeout_Count==0)// �˵�������ʱ
		{
		    SYS_PARAMETER_Save();
			break;
		}
	}
}



/*******************�ָ���*****************************************
 ��ΪRDS�������ý���ĺ���
***/
void Print_ZF_Table(uint8_t x,uint8_t y)// ��Ļ��ӡ���ַ���
{
	uint8_t i,h;
	for(h=0;h<4;h++)
	{
		for(i=0;i<22;i++)
		{
			Draw_Ascii6X8(x+i*6,y+12*h,  ZF_Table[h][i]);
		}
	}

}
void Show_PS_Name(uint8_t num)
{
	uint8_t n;
	Draw_Ascii6X8(86,39,num+'0');
	Draw_Ascii6X8(92,39,'-');
	for(n=0;n<8;n++)
	{
		Draw_Ascii6X8(98+6*n,39,PS_Name[num][n]);
	}
}
void Show_Area_Name(uint8_t num)
{
	uint8_t n;
	Draw_Ascii6X8(86,47,num/10 + '0');
	Draw_Ascii6X8(92,47,num%10 + '0');
	Draw_Ascii6X8(98,47,'-');
	for(n=0;n<7;n++)
	{
		Draw_Ascii6X8(104+6*n,47,Area_Name[num][n]);
	}
}
void Show_RDS_SetMode(uint8_t num)
{
	switch (num)
	{
		case 0:
			Draw_String6X8(86,55,"0-ALL OFF ");
			break;
		case 1:
			Draw_String6X8(86,55,"1-ALL ON  ");
			break;
		case 2:
			Draw_String6X8(86,55,"2-ALL SET ");
			break;
		default :break;
	}
}
void Setup_PS_Name(uint8_t num)
{
	uint8_t n;
	uint8_t Name_pos=0;// ����λ�û��������
	struct
	{
		uint8_t x; // 0~21
		uint8_t y; // 0~3
	}zf_pos={0,0};// �ַ�λ��
	uint8_t edit_flag=0;// ��0 =�ַ�ѡ��״̬    0=����ѡ��״̬
	uint8_t name_string[8]={0};

	memcpy(name_string,&PS_Name[num][0],8);
	Draw_Clear();
	KEY_MENU_CLEAR();
	Draw_Rect(0,0,        191,63,  1); // ��Ļ�󷽿�
	Draw_Rect(2,12,        150,61,  1);// �ַ�����
	Print_ZF_Table(8,14);              // ��ʾ�ַ���
	Draw_Ascii6X8(1,2,'[');
	Draw_Ascii6X8(7,2,num+'0');
	Draw_Ascii6X8(13,2,']');
	Menu_Timeout_Count=1000;// �˵� ��ι����  10s	
name_pos_update:
	for(n=0;n<8;n++)
	{
		if(n == Name_pos)
		{
			Draw_Ascii6X8_Downline(19+6*n,2,name_string[n]);
		}
		else
		{
			Draw_Ascii6X8(19+6*n,2,name_string[n]);
		}
	}
	if(Name_pos==8)
	{
		Draw_String6X8(80,2,">SAVE");
	}
	else
	{
		Draw_String6X8(80,2," SAVE");
	}
	if(Name_pos==9)
	{
		Draw_String6X8(80+6*6,2,">EXIT");
	}
	else
	{
		Draw_String6X8(80+6*6,2," EXIT");
	}

	while(1)
	{
		if(KEY_OK_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)// �ַ�ѡ��״̬
			{
				if(Name_pos<8)
				{
					Draw_XLine(8 + 6*zf_pos.x,   14+8+12*zf_pos.y,  8 + 6*zf_pos.x+6,   0);// ���ַ������
					edit_flag=0;// �ص�����ѡ��״̬
					name_string[Name_pos] = ZF_Table[zf_pos.y][zf_pos.x];// �ɵ�ǰ���ַ�������������ַ�ֵ
					++Name_pos;
					goto name_pos_update;// ���أ���ʾ���ƴ����
				}
			}
			else
			{
				if(Name_pos==8)// ���洦
				{
					memcpy(&PS_Name[num][0],name_string,8);
					PS_Name_Save();
					ps_int();
					return ;
				}
				else if(Name_pos==9)// �˳���
				{
					return ;
				}
				else
				{
					edit_flag=1;//�����ַ�ѡ��״̬
					zf_pos.x=0;
					zf_pos.y=0;
					Draw_XLine(8 + 6*zf_pos.x,   14+8+12*zf_pos.y,  8 + 6*zf_pos.x+6,  1);// ��ʾ�ַ������
				}
			}
		}
		if(KEY_UP_STATUS() == SET)
		{
			UP_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)
			{
				uint8_t x = 8 + 6*zf_pos.x;
				uint8_t y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    0);
				if(--zf_pos.y>3)
				{
					zf_pos.y=3;
				}
				y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    1);
			}
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			DOWN_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)
			{
				uint8_t x = 8 + 6*zf_pos.x;
				uint8_t y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    0);
				if(++zf_pos.y>3)
				{
					zf_pos.y=0;
				}
				y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    1);
			}
		}
		if(KEY_LEFT_STATUS() == SET)
		{
			LEFT_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)
			{
				uint8_t x = 8 + 6*zf_pos.x;
				uint8_t y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    0);
				if(--zf_pos.x>21)
				{
					zf_pos.x=21;
				}
				x = 8 + 6*zf_pos.x;
				Draw_XLine(x, y,    x+6,    1);
			}
			else
			{
				if(--Name_pos>9)
					Name_pos=9;
				goto name_pos_update;// ���أ���ʾ���ƴ����
			}
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			RIGHT_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)
			{
				uint8_t x = 8 + 6*zf_pos.x;
				uint8_t y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    0);
				if(++zf_pos.x>21)
				{
					zf_pos.x=0;
				}
				x = 8 + 6*zf_pos.x;
				Draw_XLine(x, y,    x+6,    1);
			}
			else
			{
				if(++Name_pos>9)
					Name_pos=0;
				goto name_pos_update;// ���أ���ʾ���ƴ����
			}
		}
		if(KEY_MENU_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			break;
		}
		if(Menu_Timeout_Count==0)
		{
			break;
		}

	}
	
}
void Setup_Area_Name(uint8_t num)
{
	uint8_t n;
	uint8_t Name_pos=0;// ����λ�û��������
	struct
	{
		uint8_t x; // 0~21
		uint8_t y; // 0~3
	}zf_pos={0,0};// �ַ�λ��
	uint8_t edit_flag=0;// ��0 =�ַ�ѡ��״̬    0=����ѡ��״̬
	uint8_t name_string[7]={0};

	memcpy(name_string,&Area_Name[num][0],7);
	Draw_Clear();
	KEY_MENU_CLEAR();
	Draw_Rect(0,0,        191,63,  1); // ��Ļ�󷽿�
	Draw_Rect(2,12,        150,61,  1);// �ַ�����
	Print_ZF_Table(8,14);              // ��ʾ�ַ���
	Draw_Ascii6X8(1,2,'[');
	Draw_Ascii6X8(7,2,num/10+'0');
	Draw_Ascii6X8(13,2,num%10+'0');
	Draw_Ascii6X8(19,2,']');
	Menu_Timeout_Count=1000;// �˵� ��ι����  10s	
name_pos_update:
	for(n=0;n<7;n++)
	{
		if(n == Name_pos)
		{
			Draw_Ascii6X8_Downline(25+6*n,2,name_string[n]);
		}
		else
		{
			Draw_Ascii6X8(25+6*n,2,name_string[n]);
		}
	}
	if(Name_pos==7)
	{
		Draw_String6X8(80,2,">SAVE");
	}
	else
	{
		Draw_String6X8(80,2," SAVE");
	}
	if(Name_pos==8)
	{
		Draw_String6X8(80+6*6,2,">EXIT");
	}
	else
	{
		Draw_String6X8(80+6*6,2," EXIT");
	}

	while(1)
	{
		if(KEY_OK_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)// �ַ�ѡ��״̬
			{
				if(Name_pos<7)
				{
					Draw_XLine(8 + 6*zf_pos.x,   14+8+12*zf_pos.y,  8 + 6*zf_pos.x+6,   0);// ���ַ������
					edit_flag=0;// �ص�����ѡ��״̬
					name_string[Name_pos] = ZF_Table[zf_pos.y][zf_pos.x];// �ɵ�ǰ���ַ�������������ַ�ֵ
					++Name_pos;
					goto name_pos_update;// ���أ���ʾ���ƴ����
				}
			}
			else
			{
				if(Name_pos==7)// ���洦
				{
					memcpy(&Area_Name[num][0],name_string,7);
					Area_Name_Save();
					return ;
				}
				else if(Name_pos==8)// �˳���
				{
					return ;
				}
				else
				{
					edit_flag=1;//�����ַ�ѡ��״̬
					zf_pos.x=0;
					zf_pos.y=0;
					Draw_XLine(8 + 6*zf_pos.x,   14+8+12*zf_pos.y,  8 + 6*zf_pos.x+6,  1);// ��ʾ�ַ������
				}
			}
		}
		if(KEY_UP_STATUS() == SET)
		{
			UP_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)
			{
				uint8_t x = 8 + 6*zf_pos.x;
				uint8_t y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    0);
				if(--zf_pos.y>3)
				{
					zf_pos.y=3;
				}
				y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    1);
			}
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			DOWN_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)
			{
				uint8_t x = 8 + 6*zf_pos.x;
				uint8_t y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    0);
				if(++zf_pos.y>3)
				{
					zf_pos.y=0;
				}
				y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    1);
			}
		}
		if(KEY_LEFT_STATUS() == SET)
		{
			LEFT_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)
			{
				uint8_t x = 8 + 6*zf_pos.x;
				uint8_t y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    0);
				if(--zf_pos.x>21)
				{
					zf_pos.x=21;
				}
				x = 8 + 6*zf_pos.x;
				Draw_XLine(x, y,    x+6,    1);
			}
			else
			{
				if(--Name_pos>8)
					Name_pos=8;
				goto name_pos_update;// ���أ���ʾ���ƴ����
			}
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			RIGHT_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			if(edit_flag)
			{
				uint8_t x = 8 + 6*zf_pos.x;
				uint8_t y = 14+8+12*zf_pos.y;
				Draw_XLine(x, y,    x+6,    0);
				if(++zf_pos.x>21)
				{
					zf_pos.x=0;
				}
				x = 8 + 6*zf_pos.x;
				Draw_XLine(x, y,    x+6,    1);
			}
			else
			{
				if(++Name_pos>8)
					Name_pos=0;
				goto name_pos_update;// ���أ���ʾ���ƴ����
			}
		}
		if(KEY_MENU_STATUS() == SET)
		{
			break;
		}
		if(Menu_Timeout_Count==0)// �˵�������ʱ
		{
			break;
		}
	}
	
}
void Show_DevStatus_One(uint8_t x,uint8_t y,uint8_t status)
{
	uint8_t x2 = 16+10*x;
	uint8_t y2 = 18+11*y;
	uint8_t const *pic = X_SPK;
	switch (status)
	{
		case 0:pic = X_SPK;break;
		case 1:pic = S_SPK;break;
		case 2:pic = M_SPK;break;
		case 3:pic = B_SPK;break;
		default :         return;
	}
	Show_Picture(x2,y2,pic,8,11);// ��ʾ������
	
}
void Show_ALLSET_devgb(uint8_t x,uint8_t y)// �豸��귴�ף�x(0~15)��,y(0~3)��
{
	uint8_t x2 = 16+10*x;
	uint8_t y2 = 18+11*y;
	Draw_Lump_fb(x2,y2,x2+7,y2+10);
}
void Show_DevStatus_Group(uint8_t line_num,uint8_t znum)// ��ʾ�ն��豸����״̬��line_num��0~3�У�znumΪ���
{
	uint8_t n;
	uint8_t lineval = 18 + line_num*11;
	char group_tab[]="00";
	group_tab[0] += znum/10;
	group_tab[1] += znum%10;
	Draw_String6X8(1,lineval+2,group_tab);
	for(n=0;n<16;n++)
	{
		Show_DevStatus_One( n,   line_num,     RDS_DevicesGroup_Mode[znum][n]);
	}
}
void Show_Device_hangnum(void)// ��ʾ�����
{
	uint8_t n;
	for(n=0;n<10;n++)
	{
		Draw_Ascii6X8(17+10*n,10,n+'0');
	}
	for(;n<15;n++)
	{
		Draw_Ascii6X8(17+10*n,10,n-10+'A');
	}
	Show_Picture(17+10*15,10,SW_8X8,8,8);
}
void Show_Strip(void)// ��ʾ������
{
}
void Show_ALLSET_Area_Name(uint8_t num,uint8_t select_f)// ��ʾ������
{
	uint8_t i;

	Draw_Ascii6X8(1,1,'[');
	if(select_f)
	{
		Draw_Ascii6X8_Downline(7,1,num/10+'0');
		Draw_Ascii6X8_Downline(13,1,num%10+'0');
	}
	else
	{
		Draw_Ascii6X8(7,1,num/10+'0');
		Draw_Ascii6X8(13,1,num%10+'0');
	}
	Draw_Ascii6X8(19,1,']');
	for(i=0;i<7;i++)
	{
		Draw_Ascii6X8(25+6*i,1,Area_Name[num][i]);
	}
	Draw_Ascii6X8(25+6*7,1,':');
	
}
void Show_ALLSET_Area_Fullsetval(uint8_t setval,uint8_t select_f)
{
	uint8_t const *pic = X_SPK8X8;
	switch (setval)
	{
		case 0:pic = X_SPK8X8;break;
		case 1:pic = S_SPK8X8;break;
		case 2:pic = M_SPK8X8;break;
		case 3:pic = B_SPK8X8;break;
	}
	Show_Picture(73,1,pic,8,8);// ��ʾ������
	if(select_f)
	{
		Draw_Lump_fb(73,1,73+7,1+7);
	}
}
void GetArea_DevStatus(uint8_t num)// ��24C64�ж�ȡ����������豸״̬����RDS_DevicesGroup_Mode[24][15]��
{

   RDS_DevicesGroup_Mode_Update(num);

}
void Setup_RDSMode(uint8_t num)
{
	if(num==2)// RDS�ն�ȫ����
	{

/*
 
 set_target    ��4��ֵ���ֱ��ʾ4��״̬��
        1      ��������				����-�Ҽ���-+set_target�����¼��޸�ֵ��	 
		2      ������ͳһ�޸Ĵ�		����-�Ҽ���-+set_target�����¼��޸�ֵ��
		3	   ��SAVE��				����-�Ҽ���-+set_target�����¼��޸�ֵ��
		4	   ��EXIT��				����-�Ҽ���-+set_target�����¼��޸�ֵ��	 

 partset_flag  
		����0ʱ��ʾ���������á�״̬    ��ʱ��-��-��-�Ҽ��ɱ༭dev_select_pos�������ﶥ���ϼ�������0�α�־
         	
*/
		uint8_t set_target=1;	
		uint8_t partset_flag=0;            // ���ֱ༭״̬��־  �ȴ��ڷǲ��ֱ༭״̬
		//uint8_t area_num=0;                // �����ֵ
		uint8_t area_fullsetval=0;         // ����ͳһ����ֵ��0-1-2-3��ֵ
//		uint8_t group_fluusetval[24];      // 24�����ͳһ����ֵ��
		PositDef dev_select_pos={0,0};     // �����޸�״̬�µ�����(����)
		uint8_t y_val=0;                   // ��Ļy����0~3
		uint8_t x_val=0;                   // ��Ļx����0~14

		PCRDS_Blinking_flag=OFF;
		Draw_Clear();
		Draw_Rect(0,0,        191,63,  1);       // ��Ļ�󷽿�
		Draw_Line(0,9,        191, 9,  1);       // һ����
		Show_Device_hangnum();                   // ��ʾ�����
        gb_area_num=&area_num;
        gb_area_setval=&area_fullsetval;
		gb_Group_num=&(dev_select_pos.y);
		gb_terminal_num=&(dev_select_pos.x);
area_update:
		//GetArea_DevStatus(area_num);             // ��ȡ���������µ��ն�״̬��Ϣ
		Show_DevStatus_Group(0,0);               // 0������ʾ0���ն�״̬
		Show_DevStatus_Group(1,1);               // 1��	  1��
		Show_DevStatus_Group(2,2);               // 2��	  2��
		Show_DevStatus_Group(3,3);               // 3��	  3��
		dev_select_pos.x = dev_select_pos.y = x_val = y_val= 0;	 // ���긴λ
gb_update:
		if(set_target==1)
		{
			Show_ALLSET_Area_Name(area_num,1);       // ��ʾ��������ѡ��״̬
		}
		else
		{
			Show_ALLSET_Area_Name(area_num,0);       // ��ʾ����������ѡ״̬
		}

		if(set_target==2)
		{
			Show_ALLSET_Area_Fullsetval(area_fullsetval,1);		 // ��ʾ����ͳһ�޸�ֵ��ѡ��״̬
		}
		else
		{
			Show_ALLSET_Area_Fullsetval(area_fullsetval,0);		 // ��ʾ����ͳһ�޸�ֵ����ѡ״̬
		}

		if(set_target==3)
		{
			Draw_String6X8(128,1,">SAVE");
		}
		else
		{
			Draw_String6X8(128,1," SAVE");
		}

		if(set_target==4)
		{
			Draw_String6X8(160,1,">EXIT");
		}
		else
		{
			Draw_String6X8(160,1," EXIT");
		}

	    area_allset_flag=1;
		while(1)
		{
			if(Menu_Timeout_Count==0)// �˵�������ʱ
			{
				area_allset_flag=0;
				break;
			}
			
			if(KEY_MENU_STATUS() == SET)
			{
				area_allset_flag=0;
				break;
			}
			if(KEY_OK_STATUS() == SET)
			{
				Menu_Timeout_Count=1000;// �˵� ��ι����  10s
				if(partset_flag)
				{
					uint8_t *temp = &RDS_DevicesGroup_Mode[dev_select_pos.y][dev_select_pos.x];
					if(++(*temp)>3)// ֵ��Χ0~3  �ֱ���� �ء�С���С���
					{
						(*temp)=0;
					}
					if(dev_select_pos.x==15)
					{
						memset(&RDS_DevicesGroup_Mode[dev_select_pos.y],(*temp),15);// ���Ƹ�����
						Show_DevStatus_Group(y_val,  dev_select_pos.y);// ������Ļ�ϵ�����
						Show_ALLSET_devgb(x_val,  y_val);// ����
						gb_Group_setval=&(*temp);
						area_group_terminal_set_flag=2;;//����ͳһ������������
					}
					else
					{
						Show_DevStatus_One(x_val,y_val,*temp);
						Show_ALLSET_devgb(x_val,y_val);// ����
						gb_terminal_setval=&(*temp);
						area_group_terminal_set_flag=3;;//�����ն�������������
					}
				}
				else
				{
					if(set_target==2)
					{
						if(++area_fullsetval>3)// ֵ��Χ0~3  �ֱ���� �ء�С���С���
							area_fullsetval=0;
						Show_ALLSET_Area_Fullsetval(area_fullsetval,1);// ��ʾͳһ�޸ĵ�ֵ��ѡ��״̬
						memset(RDS_DevicesGroup_Mode,area_fullsetval,24*16);// ���Ƹ�����
						Show_DevStatus_Group(3,3);// 3��
						Show_DevStatus_Group(2,2);// 2��
						Show_DevStatus_Group(1,1);// 1��	
						Show_DevStatus_Group(0,0);// 0�� 

						area_group_terminal_set_flag=1;;// ����ͳһ��������������
					}
					else if(set_target==3)
					{
					// �����
					RDS_DevicesGroup_Mode_Save(area_num);
					}
					else if(set_target==4)
					{// �˳���
						area_group_terminal_set_flag=0;
						area_allset_flag=0;
						return ;
					}
				}
			}
			if(KEY_UP_STATUS() == SET)
			{
				UP_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
				Menu_Timeout_Count=1000;// �˵� ��ι����  10s
				if(partset_flag)
				{
					if(dev_select_pos.y==0)
					{
						Show_ALLSET_devgb(x_val,0);// ��ԭ
						partset_flag=0;
						goto gb_update;
					}
					else
					{
						dev_select_pos.y--;
						if(y_val>0)
						{
							Show_ALLSET_devgb(x_val,y_val);// ��ԭ 	
							y_val--;
							Show_ALLSET_devgb(x_val,y_val);// ���� 	
						}
						else
						{
							Show_DevStatus_Group(3,dev_select_pos.y+3);// 3��
							Show_DevStatus_Group(2,dev_select_pos.y+2);// 2��
							Show_DevStatus_Group(1,dev_select_pos.y+1);// 1��	
							Show_DevStatus_Group(0,dev_select_pos.y+0);// 0�� 
							Show_ALLSET_devgb(x_val,0); 	
						}
					}
				}
				else
				{
					switch (set_target)
					{
						case 1:
							if(++area_num>9)area_num=0;
							GetArea_DevStatus(area_num);
							goto area_update;

						case 2:
//							if(++area_fullsetval>3)area_fullsetval=0;
//							Show_ALLSET_Area_Fullsetval(area_fullsetval,1);// ��ʾͳһ�޸ĵ�ֵ��ѡ��״̬
							break;
						default :break;
					}
				}
			}
			if(KEY_DOWN_STATUS() == SET)
			{
				DOWN_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
				Menu_Timeout_Count=1000;// �˵� ��ι����  10s
				if(partset_flag)
				{
					if(dev_select_pos.y<23)
					{
						dev_select_pos.y++;
						if(y_val<3)
						{
							Show_ALLSET_devgb(x_val,y_val);// ��ԭ 	
							y_val++;
							Show_ALLSET_devgb(x_val,y_val);// ���� 	
						}
						else
						{
							Show_DevStatus_Group(0,dev_select_pos.y-3);// 0��
							Show_DevStatus_Group(1,dev_select_pos.y-2);// 1��
							Show_DevStatus_Group(2,dev_select_pos.y-1);// 2��	
							Show_DevStatus_Group(3,dev_select_pos.y-0);// 3��
							Show_ALLSET_devgb(x_val,3); 	
						}
					}
				}
				else
				{
					switch (set_target)
					{
						case 1:
							if(--area_num>9)area_num=9;
							GetArea_DevStatus(area_num);
							goto area_update;

						case 2:
//							if(--area_fullsetval>3)area_fullsetval=3;
//							Show_ALLSET_Area_Fullsetval(area_fullsetval,1);// ��ʾͳһ�޸ĵ�ֵ��ѡ��״̬
							break;
						default :break;
					}
				}
			}
			if(KEY_LEFT_STATUS() == SET)
			{
				LEFT_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
				Menu_Timeout_Count=1000;// �˵� ��ι����  10s
				if(partset_flag)
				{
					Show_ALLSET_devgb(x_val,y_val);// ԭ����λ�ø�ԭ

					if(--dev_select_pos.x>15)dev_select_pos.x=15;
					x_val = dev_select_pos.x;

					Show_ALLSET_devgb(x_val,y_val);// ���ڵ�λ�÷���
				}
				else
				{
					if(--set_target==0)set_target=4;
					if(set_target==2)
					{
						Draw_String6X8(128,1," SAVE");// ����
						partset_flag=1;
						Show_ALLSET_devgb(x_val,y_val);// ���ڵ�λ�÷���
					}
					else
					{
						goto gb_update;
					}
				}
			}
			if(KEY_RIGHT_STATUS() == SET)
			{
				RIGHT_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
				Menu_Timeout_Count=1000;// �˵� ��ι����  10s
				if(partset_flag)// ��������״̬
				{
					Show_ALLSET_devgb(x_val,y_val);// ԭ����λ�ø�ԭ

					if(++dev_select_pos.x>15)dev_select_pos.x=0;
					x_val = dev_select_pos.x;

					Show_ALLSET_devgb(x_val,y_val);// ���ڵ�λ�÷���
				}
				else
				{
					if(++set_target>4)set_target=1;
					if(set_target==3)
					{
						Show_ALLSET_Area_Fullsetval(area_fullsetval,0);// ��ʾ����ͳһ�޸�ֵ����ѡ״̬
						partset_flag=1;
						Show_ALLSET_devgb(x_val,y_val);// ���ڵ�λ�÷���
					}
					else
					{
						goto gb_update;
					}
				}
			}
		}// whlie(1)	
	}
	else if(num==1)// RDS�ն�ȫ����
	{
		area_group_terminal_set_flag=4;// ������������
	}
	else // RDS�ն�ȫ���ر�
	{
		area_group_terminal_set_flag=5;// ������������
	}
}
void Setup_RDS_Window(void)
{
	uint8_t gb_pos=1;
	uint8_t PSName_pos=0,AreaName_pos=0,RDSMode_pos=rds_state;

	Draw_Fill(1,39,190,63,0);// ��һ�հ�����
winup:
    PSName_pos=pty_num;
    AreaName_pos=area_num;
    //PCRDS_Blinking_flag=ON;
	KEY_MENU_CLEAR();
	Draw_String6X8(1,39," PS Name  :");
	Draw_String6X8(1,47," Area Name:");
	Draw_String6X8(1,55," RDS Mode :");
	Show_PS_Name(PSName_pos);
	Show_Area_Name(AreaName_pos);
	Show_RDS_SetMode(RDSMode_pos);
gb_update:
	Draw_Ascii6X8(1,39+(gb_pos-1)*8,'>');
	Draw_Ascii6X8(80,39+(gb_pos-1)*8,'<');
	Draw_Ascii6X8(80+6*11,39+(gb_pos-1)*8,'>');
	Menu_Timeout_Count=1000;// �˵� ��ι����  10s
	while(1)
	{
		if(SystemTime_SecUpdate_Flag == SET)// ϵͳʱ�������
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// ��ʾʱ��
		}
		if(KEY_UP_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			Draw_Ascii6X8(1,39+(gb_pos-1)*8,' ');
			Draw_Ascii6X8(80,39+(gb_pos-1)*8,' ');
			Draw_Ascii6X8(80+6*11,39+(gb_pos-1)*8,' ');
			if(--gb_pos < 1)
				gb_pos=3;
			goto gb_update;
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			Draw_Ascii6X8(1,39+(gb_pos-1)*8,' ');
			Draw_Ascii6X8(80,39+(gb_pos-1)*8,' ');
			Draw_Ascii6X8(80+6*11,39+(gb_pos-1)*8,' ');
			if(++gb_pos > 3)
				gb_pos=1;
			goto gb_update;
		}
		if(KEY_LEFT_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			switch (gb_pos)
			{
				case 1:
					if(PSName_pos)
					{
						PSName_pos--;
						pty_num=PSName_pos;
						psys_data->ui_psnum=PSName_pos;
						SYS_PARAMETER_Save();//
						Show_PS_Name(PSName_pos);
						ps_int();
					}
					break;
				case 2:
					if(AreaName_pos)
					{
						AreaName_pos--;
						area_num=AreaName_pos;
						psys_data->ui_areanum=AreaName_pos;
						SYS_PARAMETER_Save();//���������
						GetArea_DevStatus(area_num);
						Show_Area_Name(AreaName_pos);
					}
					break;
				case 3:
					if(RDSMode_pos)
					{
						RDSMode_pos--;
						rds_state=RDSMode_pos;
						psys_data->ui_rds_state=rds_state;
						SYS_PARAMETER_Save();
						Show_RDS_SetMode(RDSMode_pos);
					}
					break;
			}
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			switch (gb_pos)
			{
				case 1:
					if(PSName_pos<9)
					{
						PSName_pos++;
						pty_num=PSName_pos;
						psys_data->ui_psnum=PSName_pos;
						SYS_PARAMETER_Save();//����
						Show_PS_Name(PSName_pos);
						ps_int();
					}
					break;
				case 2:
					if(AreaName_pos<19)
					{
						AreaName_pos++;
						area_num=AreaName_pos;
						psys_data->ui_areanum=AreaName_pos;
						SYS_PARAMETER_Save();//���������
						GetArea_DevStatus(area_num);
						Show_Area_Name(AreaName_pos);
					}
					break;
				case 3:
					if(RDSMode_pos<2)
					{
						RDSMode_pos++;
						rds_state=RDSMode_pos;
						psys_data->ui_rds_state=rds_state;
						SYS_PARAMETER_Save();
						Show_RDS_SetMode(RDSMode_pos);
					}
					break;
			}
		}
		if(KEY_MENU_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			os_dly_wait(10);
			while((KEY_MENU_STATUS() == SET)) ;
			area_group_terminal_set_flag=0;
			break;
		}
		if(KEY_RDS_STATUS() == SET)	//��ֹ��������ý����°�RDS��ʱ�˳��ֻص�������
		{
         KEY_RDS_CLEAR();
		}
		if(Menu_Timeout_Count==0)// �˵�������ʱ
		{
			break;
		}

		if(KEY_OK_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s
			switch (gb_pos)
			{
				case 1:
				    PCRDS_Blinking_flag=OFF;
					menu_set=SET;
					os_dly_wait(50);
					Setup_PS_Name(PSName_pos);
					break;
				case 2:
				    PCRDS_Blinking_flag=OFF;
					menu_set=SET;
					os_dly_wait(50);
					Setup_Area_Name(AreaName_pos);
					break;
				case 3:
				    PCRDS_Blinking_flag=OFF;
					menu_set=SET;
					os_dly_wait(50);
					Setup_RDSMode(RDSMode_pos);
					break;
				
				default:break;
			}
			menu_set=RESET;

			if(Menu_Timeout_Count==0)return;// �˵�������ʱ
				
			Draw_Clear();	
			Draw_Rect(0,0,        191,63,  1);// ��Ļ�󷽿�
			Draw_Rect(0,0,         65,19,  1);// ���ϽǷ���
			Draw_Line(0,19,       179,19,  1);// ��Ļ�м�һ����
			Draw_Line(0,37,       191,37,  1);// ��Ļ�м�һ����
			Draw_Rect(179,0,      191,20,  1);// ����
			Draw_Line(179,10,     191,10,  1);// ��
			Show_RDS_logox();//	СRDSͼ��
			Show_Picture(181,1,FH_RD8X8,8,8);
			Show_Picture(181,10,FH_PC8X8,8,8);
		
			Show_RDS(rds_state);// ��ʾRDS״̬
			Show_TX(PTT_status);// ��ʾ����״̬
			(PTT_status==SET) ? TX_LED_ON():TX_LED_OFF();
			Show_Time(&SystemTime);// ��ʾʱ��
			Show_FM(FM_value);// ��ʾFMƵ��ֵ
			Show_I(I_value);// ��ʾ����ֵ
			Show_T(T_value);// ��ʾ�¶�ֵ
			Show_P(Power_value);// ��ʾ����ֵ
			Show_RP(RP_value);// ��ʾ��������ֵ
			Show_SWR(SWR_value);// ��ʾSWRֵ
			goto winup;
		}
	}

}


//===uint16_t=======================
float ADValue_T[15],ADValue_P[15],ADValue_RP[15],ADValue_I[15],ADValue_vin[15];
uint16_t ADValue_swr[15];

static u8 count_t=0,count_p=0,count_rp=0,count_i=0,count_swr=0,count_vin=0;

float filter_t(float tmp_t)
{
    float Average=0,temp;
	float SumMiddle;
	uint8_t n,i,j,min;

    ADValue_T[count_t++]=tmp_t;
	if(count_t>9)count_t=0;

    n=9;  //���� ,ȥ��ֵ��ֵ�˲�
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_T[min]>ADValue_T[j])min=j;
        temp=ADValue_T[i];
        ADValue_T[i]=ADValue_T[min];
        ADValue_T[min]=temp;      
     }

    SumMiddle=0;     //���м�3��ֵ���
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_T[i];

    Average= SumMiddle/3;     //���м�3��ֵ��ƽ��
	return Average;

}

float filter_p(float tmp_p)//uint16_t
{
    float Average=0,temp;
	float SumMiddle;
	uint8_t n,i,j,min;

    ADValue_P[count_rp++]=tmp_p;
	if(count_p>9)count_p=0;

    n=9;  //���� ,ȥ��ֵ��ֵ�˲�
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_P[min]>ADValue_P[j])min=j;
        temp=ADValue_P[i];
        ADValue_P[i]=ADValue_P[min];
        ADValue_P[min]=temp;      
     }

    SumMiddle=0;     //���м�3��ֵ���
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_P[i];

    Average= SumMiddle/3;     //���м�3��ֵ��ƽ��
	return Average;

}


float filter_rp(float tmp_rp)
{
    float Average=0,temp;
	float SumMiddle;
	uint8_t n,i,j,min;

    ADValue_RP[count_rp++]=tmp_rp;
	if(count_rp>9)count_rp=0;

    n=9;  //���� ,ȥ��ֵ��ֵ�˲�
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_RP[min]>ADValue_RP[j])min=j;
        temp=ADValue_RP[i];
        ADValue_RP[i]=ADValue_RP[min];
        ADValue_RP[min]=temp;      
     }

    SumMiddle=0;     //���м�3��ֵ���
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_RP[i];

    Average= SumMiddle/3;     //���м�3��ֵ��ƽ��
	return Average;

}

float filter_I(float tmp_i)
{
    float Average=0,temp;
	float SumMiddle5;
	uint8_t n,i,j,min;

    ADValue_I[count_i++]=tmp_i;
	if(count_i>9)count_i=0;

    n=9;  //���� ,ȥ��ֵ��ֵ�˲�
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_I[min]>ADValue_I[j])min=j;
        temp=ADValue_I[i];
        ADValue_I[i]=ADValue_I[min];
        ADValue_I[min]=temp;      
     }

    SumMiddle5=0;     //���м�3��ֵ���
    for (i=4;i<7;i++) 
	  SumMiddle5=SumMiddle5+ADValue_I[i];

    Average= SumMiddle5/3;     //���м�3��ֵ��ƽ��
	return Average;

}

uint16_t filter_swr(uint16_t tmp_swr)
{
    uint16_t Average=0,temp;
	uint32_t SumMiddle;
	uint8_t n,i,j,min;

    ADValue_swr[count_swr++]=tmp_swr;
	if(count_swr>9)count_swr=0;

    n=9;  //���� ,ȥ��ֵ��ֵ�˲�
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_swr[min]>ADValue_swr[j])min=j;
        temp=ADValue_swr[i];
        ADValue_swr[i]=ADValue_swr[min];
        ADValue_swr[min]=temp;      
     }

    SumMiddle=0;     //���м�3��ֵ���
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_swr[i];

    Average= SumMiddle/3;     //���м�3��ֵ��ƽ��
	return Average;

}


float filter_vin(float tmp_vin)
{
    float Average=0,temp;
	float SumMiddle;
	uint8_t n,i,j,min;

    ADValue_vin[count_vin++]=tmp_vin;
	if(count_vin>9)count_vin=0;

    n=9;  //���� ,ȥ��ֵ��ֵ�˲�
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_vin[min]>ADValue_vin[j])min=j;
        temp=ADValue_vin[i];
        ADValue_vin[i]=ADValue_vin[min];
        ADValue_vin[min]=temp;      
     }

    SumMiddle=0;     //���м�3��ֵ���
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_vin[i];

    Average= SumMiddle/3;     //���м�3��ֵ��ƽ��
	return Average;

}


//==========================

uint8_t Get_CollectTempval(void)// �����¶�ֵ����λ��C
{
	uint32_t voltage_uV;//��ѹֵ����λuV
	uint32_t voltage_mV;//��ѹֵ����λmV
	uint8_t tempval;// 0~100 ��C

	voltage_uV = ADC_VALUE_Vt*805;
	voltage_mV = voltage_uV/1000;
	if(voltage_mV>=1500)
	{
	 return 99;
	 }
	if(voltage_mV>=500)
	{
		tempval = (voltage_mV-500)/10;// ת��Ϊ��C      10mV/��C
		if(tempval<100)
		{
			return tempval;
		}
	}

	return 0;
}
uint16_t Get_CollectCurrentVal(void)// ���ص���ֵ����λA
{
	uint32_t val2=0;

	val2 = (ADC_VALUE_Vi*805)/30000;

	return val2;
}
uint16_t Get_PowerValue(void)
{
	uint32_t val_mV;
	float val_pW;
	
	val_mV = (ADC_VALUE_Vp*805)/1000;
	//val_pW = ((val_mV/4)*val_mV*80)/1000/1000;

	val_mV =filter_p(val_mV);
	Vp_mv=val_mV;
	val_pW = (val_mV*val_mV)/1000.0/1000.0;
	val_pW =(float)val_pW*(jiaozheng[((FM_value/100)/50-14)]/10)*4;
	return (uint16_t)val_pW;
}

uint16_t Get_RPowerValue(void)
{
	uint32_t val_mV;
	float val_pW;
	
	val_mV = (ADC_VALUE_Vn*805)/1000;
	val_mV =filter_rp(val_mV);
	Vn_mv=val_mV;
    val_pW = (val_mV*val_mV)/1000.0/1000.0;
	val_pW =(float) val_pW*(jiaozheng[((FM_value/100)/50-14)]/10)*4;
	return (uint16_t)val_pW;
}



void CollectShowTempval(void)
{
	//T_value = Get_CollectTempval();
	T_value =filter_t( Get_CollectTempval() );
	Show_T(T_value);
}

void CollectShowPower(void)
{
	//Power_value = Get_PowerValue();
	Power_value =Get_PowerValue();
	Show_P(Power_value);
}

void CollectShowRPower(void)
{
	//RP_value = Get_RPowerValue();
	RP_value = Get_RPowerValue();
	Show_RP(RP_value);
}

void CollectShowCurrent(void)
{

	//I_value = Get_CollectCurrentVal();
	I_value =filter_I(Get_CollectCurrentVal());
	Show_I(I_value);
}

void CollectShowSWR(void)
{
	//SWR_value=abs((ADC_VALUE_Vp+ADC_VALUE_Vn)/(ADC_VALUE_Vp-ADC_VALUE_Vn));
	SWR_value=filter_swr( abs((ADC_VALUE_Vp+ADC_VALUE_Vn)/(ADC_VALUE_Vp-ADC_VALUE_Vn)) );
	Show_SWR(SWR_value);
}

void CollectVIN(void)//excitation voltage  ������ѹ
{
	uint32_t val_mV;

	val_mV = (ADC_VALUE_Vin*805)/1000;
    //Excitation_V=(uint16_t)val_mV;	 //��λ����
	Excitation_V=filter_vin( (uint16_t)val_mV );
}

int main(void)
{
//	I_value = floor(123.62);
//	T_value = ceil(124.2);
	KEY_config();
	DAC5615_Init();
	if(BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
	{
		RTC_Configuration();
		
		RTC_WaitForLastTask();/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
		
		RTC_SetCounter(0);/* ��RTC����ֵд��RTC�Ĵ��� */
		
		RTC_WaitForLastTask();/* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
		
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);/* �޸ĺ󱸼Ĵ���1��ֵΪ0XA5A5 */
	}
	else
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

		PWR_BackupAccessCmd(ENABLE);

		RTC_WaitForSynchro();/* Wait for RTC registers synchronization */
		
		RTC_ITConfig(RTC_IT_SEC, ENABLE);/* Enable the RTC Second */
		
		RTC_WaitForLastTask();/* Wait until last write operation on RTC registers has finished */
	}

	COM1_2Init();


	TIM3Init();
	NVIC_Configuration();// ϵͳ�жϿ�����
	
	os_sys_init(InitTask);// RTX����ϵͳ��ʼ������������һ������ InitTask
	for(;;);
}

__task
void InitTask(void)
{
	tid_main_task = os_tsk_create_user(main_task, 1, &main_task_stack, sizeof(main_task_stack));
	//tid_alarm_task = os_tsk_create(Alarm_task, 1);// ������������
	//tid_PCRDS_task = os_tsk_create(PCRDS_Blinking_task, 1);// ������������

	os_tsk_delete_self();
}

#include "RDS_setup_debug(bak).h"  // ��ʱ���Ժ��� DebugMode();
__task
void main_task(void)
{
	unsigned int temp1=0;
	unsigned char ledflash=0;
	unsigned char sample_count=0;

	DAC5615Out(0);// ��ʼ�����ϵ��һʱ������5615���0V��
	Delay_ms(5000);

	LCD19264_Init();
	LCD19264_BG_Open();
	LCD19264_Display_String(0,0," System Init...");

PT2314init:
	PT2314_Init();// ��ʼ���ɹ��� ��Ĭ��  ͨ��1��������0dB������ƽ�⣬����/����0dB
	if(pt2314_error_flag)
	{
		LCD19264_Display_String(0,2,"PT2314 Fail");
		goto PT2314init;
	}
	LCD19264_Display_String(0,2,"PT2314 OK  ");
	LCD19264_Clear();


// 1.��ӭ����
	Draw_Clear();	
	Draw_Picture_logoRDS();                // ��ʾlogo
	Draw_Rect(0,0,        191,63,  1);     // ��Ļ�󷽿�
	Draw_Line(0,32,       191,32,  1);     // ��Ļ�м�һ����
	Draw_String6X8(64,  34+ 0,     "Welcome!");
	Draw_String6X8(16,  34+10,     "Set parameters? 8 YES");
	Draw_String6X8(20,  34+20,     "www.funsongrf.com");
	for(temp1=8;temp1>0;temp1--)
	{
		Draw_Ascii6X8( 16+16*6, 34+10,(u8)temp1 + '0');
		os_dly_wait(100);
		if(KEY_OK_STATUS() == SET)
		{
		    KEY_OK_CLEAR();
//			PT2314_Setup_CVD(3,GAIN_OFF);				// FM/DRS   3ͨ��
//			QN8027_Init();
//			QNd8027_SetChannel((103000-76000)/50);
//			DebugMode();
        	KEY_Status_AllClear();// �����а�����־
            setsys_para=SET;
			os_dly_wait(200);
			Draw_Clear();
			break;
		}
	}
	PCRDS_Blinking_flag=OFF;
// ȷ����������
	Power_set=20;
	//FM_value = 101700;
	PT2314_Setup_CVD(3,GAIN_OFF);				       // ��ͨ���ջ�   3ͨ��
	QN8027_Init();
	SYS_PARAMETER_Update();
	QNd8027_SetChannel((FM_value-76000)/50);
	PT2314_Setup_Volume(Volume_value);
	PT2314_Setup_Treble(Treble_value);
	PT2314_Setup_Bass(Bass_value);
	/*
	if(rds_state==0)//ȫ��
      area_group_terminal_set_flag=5;
	if(rds_state==1)//ȫ��
      area_group_terminal_set_flag=4;
	if(rds_state==2)//����
      area_group_terminal_set_flag=0;
	 */
	//while(1);
// 2.������������
	os_tsk_create(RDS_task,1);                         // ����С���RDS����
	//RDS_task_int();
	os_mut_init(LCD_mutex);                            // ��ʼ�� LCD�����Ļ������
//	tid_ComDebug = os_tsk_create(ComDebug_task,3);	   // ���ڵ�������
//	tid_AutoFanControl = os_tsk_create(AutoFanControl_task,1);
	tid_AutoAdjustPower = os_tsk_create(AutoAdjustPower_task,1);
	tid_AutoChoiceAudio = os_tsk_create(AutoChoiceAudio_task,1);

	PTT_status = SET;// ������ʱĬ���ϵ翪��PTT
	TX_status=SET;

	tid_alarm_task = os_tsk_create(Alarm_task, 1);// ������������
	tid_PCRDS_task = os_tsk_create(PCRDS_Blinking_task, 1);// ��������
	tid_comdata_task = os_tsk_create(comdata_task, 1);// ��������
// 2.����������
main_window_entry:
    //PCRDS_Blinking_flag=ON;
	os_mut_wait(LCD_mutex,0xffff);
	Draw_Clear();	
	Draw_Rect(0,0,        191,63,  1);// ��Ļ�󷽿�
	Draw_Rect(0,0,         65,19,  1);// ���ϽǷ���
	Draw_Line(0,19,       179,19,  1);// ��Ļ�м�һ����
	Draw_Line(0,37,       191,37,  1);// ��Ļ�м�һ����
	Draw_Rect(179,0,      191,19,  1);// ����
	Draw_Line(179,9,     191,9,  1);// ��
	Show_RDS_logox();//	СRDSͼ��
	Show_FSRF_Logo();// ��ʾFS-RFͼ��
	Show_Picture(181,1,FH_RD8X8,8,8);
	Show_Picture(181,10,FH_PC8X8,8,8);


	Show_RDS(rds_state);// ��ʾRDS״̬
	Show_TX(PTT_status);// ��ʾ�����״̬
	(PTT_status==SET) ? TX_LED_ON():TX_LED_OFF();
	Show_Time(&SystemTime);// ��ʾʱ��
	Show_FM(FM_value);// ��ʾFMƵ��ֵ
	Show_I(I_value);// ��ʾ����ֵ
	Show_T(T_value);// ��ʾ�¶�ֵ
	Show_P(Power_value);// ��ʾ����ֵ
	Show_RP(RP_value);// ��ʾ��������ֵ
	Show_SWR(SWR_value);// ��ʾSWRֵ

update_v1:
	Show_Volume(Volume_value);
	Show_PEVal(PE_value);
	Show_VOLB(Bass_value);// ��ʾ����ֵ
	Show_VOLH(Treble_value);// ��ʾ����ֵ
	Show_FSRF_Logo();// ��ʾFS-RFͼ��
	os_mut_release(LCD_mutex);

	while(1)
	{
tab1:
		if(SystemTime_SecUpdate_Flag == SET)
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// ��ʾʱ��
		}
		if(CollectUpdate_Interval==0)
		{
			CollectUpdate_Interval = 10;// ���100ms

			CollectShowCurrent();  // �ɼ���������ʾ
			CollectShowTempval();  // �ɼ��¶Ȳ���ʾ
			CollectShowPower();	   // �ɼ����ʲ���ʾ
			CollectShowRPower();   // �ɼ������ʲ���ʾ
			CollectVIN();   // ������ѹ
			CollectShowSWR();   // ����SWR
			sample_count++;
			if(sample_count>=10)
			 sample_count=10;

		}
	   if(sample_count>=10)		//����10���Ժ�
       { 
		 if(Excitation_V<=200||(PTT_EN==Bit_RESET))
		{
		   	//if(VinWarning_Flag!=SET)
			{
				Pin_PTT_CLOSE();
				Show_TX(PTT_status=RESET);// ��ʾ�����״̬
				TX_LED_OFF();
				VIN_AND_EN=FALSE;
					
				DAC5615Out(0);// 
				powerval_bak = 0; //ʹ�´���vinʱ�����ٵ���
				fmval_bak = 0;
				dav_save=0;
			    VinWarning_Flag=SET;
			}
		}
		else if(Excitation_V>200&&(PTT_EN==Bit_SET))//����0.2V
		{
		   //if(VinWarning_Flag==SET)
		   {
		     if(HighTempWarning_Flag==RESET	&&
			    HighCURRENTWarning_Flag==RESET &&
				HighSWRWarning_Flag==RESET&&
		        HighPRWarning_Flag == RESET && 
		        HighLPWarning_Flag == RESET 
				)
			 {
			  Pin_PTT_OPEN(); 
			  TX_LED_ON();
			  Show_TX(PTT_status=SET);// ��ʾ�����״̬
			  VIN_AND_EN=OK;
			 }
		   }
		}


		if(T_value>38)
		{
			Pin_FAN_OPEN();
		}
		if(T_value<35)
		{
			Pin_FAN_CLOSE();
		}

		if(setsys_para==RESET)
		{
 			if(T_value >= 65 && TP_set>65) //
			{	
			    tp_adjust_p=SET;
				DAC5615Out(0);// 
				powerval_bak = 0; 
				fmval_bak = 0;
				dav_save=dav_save*0.707;//�����½�
			 	Power_set_save=Power_set/2;

			 }

 			if(T_value <= 45 && tp_adjust_p==SET) //
			{
                tp_adjust_p=RESET;
				DAC5615Out(0);// 
				powerval_bak = 0; //
				fmval_bak = 0;
				dav_save=dav_save;//�Ӱ�ֵ����������
				Power_set_save=Power_set;
			 }

			if(T_value > TP_set) //���±���
			{
				if(HighTempWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// ��ʾ�����״̬
					
					DAC5615Out(0);//
					powerval_bak = 0; //���µ���
					fmval_bak = 0;
					dav_save=0;
										 					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// ����һ�������źŸ���������
					Draw_Fill(1,39,190,63,0);// ��һ�հ�����
					Show_Picture(88,40,Alarm_Pic16X16,16,13);
					Draw_String6X8(40,55,"HIGH TEMP WARNING !");
					HighTempWarning_Flag = SET;
				}
			}
			else
			{
				if(HighTempWarning_Flag == SET)
				{
				   if(VIN_AND_EN==OK)
					 Show_TX(PTT_status=SET);// ��ʾ�����״̬

					TX_LED_ON();
					TX_status=SET;
					os_evt_set(ALARM_END_EVENT,tid_alarm_task);// ����һ�������źŸ���������
					Draw_Fill(1,39,190,63,0);// ��һ�հ�����
					HighTempWarning_Flag = RESET;
					KEY_Status_AllClear();
					goto update_v1;
				}
			}

 			if(I_value > (8*12)) //����������������8A,����120%
			{
				if(HighCURRENTWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// ��ʾ�����״̬
					
					DAC5615Out(0);//
					powerval_bak = 0; //���µ���
					fmval_bak = 0;
					dav_save=0;
					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// ����һ�������źŸ���������
					Draw_Fill(1,39,190,63,0);// ��һ�հ�����
					Show_Picture(88,40,Alarm_Pic16X16,16,13);
					Draw_String6X8(40,55,"HIGH CURRENT WARNING !");
					HighCURRENTWarning_Flag = SET;
				}
			}
			else  //���ɻָ���������
			{
			   ;
			}
			//=======================
 			if(SWR_value > SWRP_set&& RP_value>5) //פ���ȱ���
			{
				if(HighSWRWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// ��ʾ�����״̬
					
					DAC5615Out(0);//
					powerval_bak = 0; //���µ���
					fmval_bak = 0;
					dav_save=0;
					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// ����һ�������źŸ���������
					Draw_Fill(1,39,190,63,0);// ��һ�հ�����
					Show_Picture(88,40,Alarm_Pic16X16,16,13);
					Draw_String6X8(40,55,"HIGH VSWR WARNING !");
					HighSWRWarning_Flag = SET;
				}
			}
			else  //���ɻָ���������
			{
			;
			}

 			if(Vn_mv > (Vp_mv*5/4) && abs(RP_value)>5) //�����书�ʱ���  ,�����ʴ��������ʵ�1/4
			{
				if(HighPRWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// ��ʾ�����״̬
					
					DAC5615Out(0);//
					powerval_bak = 0; //���µ���
					fmval_bak = 0;
					dav_save=0;
					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// ����һ�������źŸ���������
					Draw_Fill(1,39,190,63,0);// ��һ�հ�����
					Show_Picture(88,40,Alarm_Pic16X16,16,13);
					Draw_String6X8(40,55,"HIGH PR WARNING !");
					HighPRWarning_Flag = SET;
				}
			}

 			if(Vp_mv < 2000&& I_value>(64)) //С���ʴ��������
			{
				if(HighLPWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// ��ʾ�����״̬
					
					DAC5615Out(0);//
					powerval_bak = 0; //���µ���
					fmval_bak = 0;
					dav_save=0;
					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// ����һ�������źŸ���������
					Draw_Fill(1,39,190,63,0);// ��һ�հ�����
					Show_Picture(88,40,Alarm_Pic16X16,16,13);
					Draw_String6X8(40,55," LP-HI  WARNING !");
					HighLPWarning_Flag = SET;
				}
			}

	   }

	  }

		if(HighTempWarning_Flag == SET	||
		   HighCURRENTWarning_Flag==  SET   ||
		   HighSWRWarning_Flag==  SET ||
		   HighPRWarning_Flag == SET  ||
		   HighLPWarning_Flag == SET 
		   )
		{
			goto tab1;
		} 


// ����������--------------------------------------------------------------




		if(KEY_DOWN_STATUS() == SET)
		{
		    KEY_DOWN_CLEAR() ;
			DOWN_LockFlag=SET;// ��������ʹ��          ��TIM3�жϷ�������
			if(Volume_value > 0 )
			{
				os_mut_wait(LCD_mutex,0xffff);
				Show_Volume(--Volume_value);
				PT2314_Setup_Volume(Volume_value);
				psys_data->ui_Audio_VOL=Volume_value;
				os_mut_release(LCD_mutex);
			}
		}
		if(KEY_UP_STATUS() == SET)
		{
		   KEY_UP_CLEAR();
			UP_LockFlag=SET;// ��������ʹ��         ��TIM3�жϷ�������
			if(Volume_value < 32)
			{
				os_mut_wait(LCD_mutex,0xffff);
				Show_Volume(++Volume_value);
				PT2314_Setup_Volume(Volume_value);
				psys_data->ui_Audio_VOL=Volume_value;
				os_mut_release(LCD_mutex);
			}
		}
		if(KEY_OK_STATUS() == SET)
		{
		   KEY_OK_CLEAR();
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
		   KEY_RIGHT_CLEAR();
		}
		if(KEY_LEFT_STATUS() == SET)
		{
		   KEY_LEFT_CLEAR();
		}
/**************************************************************************************
 MENU�����½��� ϵͳ�����޸Ľ���
 ��FMƵ�������ʱ���ֵ��ϵͳ������ϵͳʱ�䡢�¶ȱ���ֵ��SWRֵ��
*/

		if(KEY_MENU_STATUS() == SET ||setsys_para==SET)
		{
			uint8_t set_target=1;// �˵�ѡ��� 1��2��3
			//setsys_para=RESET;
main_menu_entry:
			Menu_Timeout_Count=1000;// �˵� ��ι����  10s 
			Draw_Fill(1,39,190,63,0);// ��һ�հ�����
			Draw_String6X8(20,39,"SET FM AND RF POWER");
			Draw_String6X8(20,47,"SET Audio AND Time");
			Draw_String6X8(20,55,"SET TEMP AND SWR");
			Draw_Ascii6X8(10,31+set_target*8,'>');
			os_dly_wait(10);
			while(KEY_MENU_STATUS() == SET);//�ȴ��ɿ�
			while(1)
			{
				if(SystemTime_SecUpdate_Flag == SET)
				{
					SystemTime_SecUpdate_Flag = RESET;
					UpdateSystemTime();
					Show_Time(&SystemTime);// ��ʾʱ��
				}
				if(KEY_UP_STATUS() == SET)
				{
					Menu_Timeout_Count=1000;// �˵� ��ι����  10s 
					Draw_Ascii6X8(10,31+set_target*8,' ');
					if(--set_target < 1)
					{
						set_target=3;
					}
					Draw_Ascii6X8(10,31+set_target*8,'>');
				}
				if(KEY_DOWN_STATUS() == SET)
				{
					Menu_Timeout_Count=1000;// �˵� ��ι����  10s 
					Draw_Ascii6X8(10,31+set_target*8,' ');
					if(++set_target > 3)
					{
						set_target=1;
					}
					Draw_Ascii6X8(10,31+set_target*8,'>');
				}
				if(KEY_OK_STATUS() == SET)
				{
					Menu_Timeout_Count=1000;// �˵� ��ι����  10s 
					//os_dly_wait(10);
					switch (set_target)
					{
						case 1:
						  fm_rf_set=SET;
						  SET_FM_And_RF_Power();
						  os_dly_wait(20);
						  fm_rf_set=RESET;
						  tp_adjust_p=RESET;//����޸��˹��ʣ�Ƶ�ʣ��¶ȵ�����־��λ
						  break;
							
						case 2:SET_Audio_And_Time();os_dly_wait(20);break;
						
						case 3:SET_Temp_And_SWR();os_dly_wait(20);break;
						
						default: break;
					}

					KEY_Status_AllClear();// ���ȫ���������±�־

					if(Menu_Timeout_Count==0)// �ȿ����ǲ��ǳ�ʱ�ķ���
					{
						break;
					}
					else
					{
						goto main_menu_entry;// ���������˵���ڣ����ѡ������
					}
				}
				if(Menu_Timeout_Count==0)
				{
					break;
				}
				if(KEY_RDS_STATUS() == SET)// RDS������Ϊ�˳��˵��Ĺ���
				{
				 os_dly_wait(10);
				 //if(KEY_MENU_STATUS() == SET)
				  //{
				   while(KEY_RDS_STATUS() == SET);//�ȴ��ɿ�
					break; 
				  //}
				 }

				if(KEY_MENU_STATUS() == SET)	//��ֹ��������ý����°�MENU��ʱ�˳��ֻص�������
				{
		         KEY_MENU_STATUS();
				}
			}//while(1)
			setsys_para=RESET;
			goto main_window_entry;

		}
		//}
/**************************************************************************************
 RDS�����½��� RDS�����޸Ľ���
*/
		if(KEY_RDS_STATUS() == SET)
		{	
		    KEY_RDS_CLEAR();
		    //PS_Name_Update();
			//Area_Name_Update();
			Setup_RDS_Window();
		    KEY_Status_AllClear();// ���ȫ���������±�־
			goto main_window_entry;
		}

	}// ��ѭ��	
}

__task
void AutoFanControl_task(void)
{
	while(1)
	{
		if(T_value>38)
		{
//			Pin_FAN_OPEN();
			TX_LED_ON();
//			while(T_value>34);
//			Pin_FAN_CLOSE();
//			TX_LED_OFF();
		}
		if(T_value<35)
		{
			TX_LED_OFF();
		}
	}
}
__task 
void ComDebug_task(void)
{
	while(1)
	{
		os_evt_wait_and(0x0001,0xffff);// �ȴ�����2�жϳ�����¼��ź�
		if(S2_receive_flag==SET)
		{
			S2_receive_flag = RESET;
			switch (Rxbuf[0])
			{
				case 0x00:
//					QND_WriteReg(Rxbuf[1], Rxbuf[2]);
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// ����һ�������źŸ���������
					break;
				case 0x01:
//					Txbyte = QND_ReadReg(Rxbuf[1]);
//					USART_SendData(USART1, Txbyte);// ����
					os_evt_set(ALARM_END_EVENT,tid_alarm_task);// ����һ�������źŸ���������
					break;
				case 0x10:
					QN8027_WriteReg(Rxbuf[1], Rxbuf[2]);
					break;
				case 0x11:
					Txbyte = QN8027_ReadReg(Rxbuf[1]);
					USART_SendData(USART2, Txbyte);// ����
					break;
				default:
					break;
			}
		}
	}
}
__task
void AutoAdjustPower_task(void)
{
    //uint32_t fmval_bak = 0;//FM_value;
	//uint16_t powerval_bak = 0;//Power_set;
	uint16_t dav=0;
	uint8_t adjct;
	while(1)
	{
		if(   ((powerval_bak != Power_set) || 		// ���ʱ仯
		      (fmval_bak != FM_value)   )  &&	   // Ƶ�α仯
			   fm_rf_set==RESET	  &&//������״̬
		       HighTempWarning_Flag==RESET	&&             
			   HighCURRENTWarning_Flag==RESET && 
			   HighSWRWarning_Flag==RESET && 
		       HighPRWarning_Flag == RESET  &&
		       HighLPWarning_Flag == RESET &&
			   VIN_AND_EN==OK &&
			   setsys_para==RESET //�˳����ò���ʱ�ٿ�ʼ����         
						  )		
		{
			//ER_LED_ON();
			dav=0;
			adjct=0;
			DAC5615Out(dav_save);//��������ù��ʡ�Ƶ���з��أ������ϴε�ֵ
			if(tp_adjust_p==SET)
			  dav=dav_save;
			do
			{	if(tp_adjust_p==SET)
				 {
					if(Power_value > (Power_set_save))	//Power_set/2 �¶ȴﵽ65��ʱ������һ��
					{
						if(dav>10)
						{
							DAC5615Out(dav-=10);
						}
					}
					else
					{
	 					if(dav<890)// <3.5V
						{
							DAC5615Out(dav+=10);
						}
					}
				 }
				else
				  {
					if(Power_value > (Power_set))
					{
						if(dav>10)
						{
							DAC5615Out(dav-=10);
						}
					}
					else
					{
	 					if(dav<890)// <3.5V
						{
							DAC5615Out(dav+=10);
						}
					}
				  }

				os_dly_wait(10);
				if(++adjct>150)break;//����15��
				
			}while(abs(Power_set-Power_value)>2 && 
			       fm_rf_set==RESET  && //���������У������������Ƶ�ʡ�����ʱ�˳�
			       HighTempWarning_Flag==RESET	&&             
				   HighCURRENTWarning_Flag==RESET && 
				   HighSWRWarning_Flag==RESET && 
		           HighPRWarning_Flag == RESET  &&
		           HighLPWarning_Flag == RESET &&
				   VIN_AND_EN==OK  &&
				   setsys_para==RESET
				       );
					    
			if(fm_rf_set==SET)
			   {
			    powerval_bak = 0;
			    fmval_bak = 0;
			    dav_save=dav;
			   }
			else
			{
			 powerval_bak = Power_set;
			 fmval_bak = FM_value;
			 tp_adjust_p=RESET;
			 dav_save=dav;
			}

		if(HighTempWarning_Flag==SET	||             
		   HighCURRENTWarning_Flag==SET || 
		   HighSWRWarning_Flag==SET   || 
		   HighPRWarning_Flag == SET  ||
		   HighLPWarning_Flag == SET ||
		   VIN_AND_EN==FALSE 	||
		   setsys_para==SET
				  )	
			{
			DAC5615Out(0);//ȷ���й���ʱ���0V
			}

			//ER_LED_OFF();
		}
		os_dly_wait(5);
	}

}


/*
��Ƶ���ȼ������ջ���DTU��AUX��MP3��//MP3���
���ȼ��ߵ��л������ȼ��͵ģ�Ӧ�������л���
�����ȼ��͵��л������ȼ��ߵ�ʱ��Ӧ��ʱ5S���л���
*/
__task
void AutoChoiceAudio_task(void)
{
	uint8_t status_tmp=0,
	        status_new=0,
	        status_save=0;
    uint16_t sound_ch_input_count=0;	        

	while(1)
	{

	    if(transfer_flag)
		{	
			status_tmp=4;	
		 }
	    else if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
		{	
			status_tmp=3;	
		 }	
		else if( !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10))	// AUX_INPUT_STATUS()== SET     PC10
		{
			status_tmp=2; 
		 }
		else if( !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) )// MP3_INPUT_STATUS()== SET
		{
			status_tmp=1; 
		 }

		if(status_save!=status_tmp)	//������ʱ
		{

		 if(!sound_ch_switch_Timeout_Count)
		 sound_ch_input_count++;

		 if(sound_ch_input_count>20) 
		  {
			  if(status_save<status_tmp)//�����ȼ��͵��л�����,�����л�
			   {
			   	switch (status_tmp)
				{
					case 1:
					  PT2314_Setup_CVD(2,GAIN_OFF);	//MP3ͨ��
						break;
					case 2:
					  PT2314_Setup_CVD(4,GAIN_OFF);	//AUXͨ��
						break;
					case 3:
					  PT2314_Setup_CVD(1,GAIN_OFF);	//DTUͨ��
						break;
					case 4:
					  PT2314_Setup_CVD(3,GAIN_OFF);	//��תͨ��
						break;
					default:
						break;
			      } 
				  status_save=status_tmp;
				  status_new=0;
				  sound_ch_input_count=0;
			     }
	
			  if(status_save>status_tmp)//�����ȼ��ߵ��л�����,��ʱ30S���л�
			   {
			    if(status_new!=status_tmp)
				{
			      status_new=status_tmp;
			      sound_ch_switch_Timeout_Count=3000;//30S
				  }
			    if(sound_ch_switch_Timeout_Count==0)
				 {
				   	switch (status_tmp)
					{
						case 1:
						  PT2314_Setup_CVD(2,GAIN_OFF);	//MP3ͨ��
							break;
						case 2:
						  PT2314_Setup_CVD(4,GAIN_OFF);	//AUXͨ��
							break;
						case 3:
						  PT2314_Setup_CVD(1,GAIN_OFF);	//DTUͨ��
							break;
						case 4:
						  PT2314_Setup_CVD(3,GAIN_OFF);	//��תͨ��
							break;
						default:
							break;
				      } 
			      status_save=status_tmp;
				  sound_ch_input_count=0;
				  }
			   }

		  }

		 }
		else
		  {
		   sound_ch_input_count=0;
		   sound_ch_switch_Timeout_Count=0;
		  }

		os_dly_wait(5);
	}

}

__task 
void BEEP_task(void)// ��������������
{
	OS_RESULT result;

	for(;;)
	{
		BP_RING();// ��������
		os_dly_wait(10);
		BP_MUTE();// ����
		os_dly_wait(10);
		BP_RING();// ��������
		os_dly_wait(10);
		BP_MUTE();// ����
		result = os_evt_wait_and(0x0001,60);
		if(result == OS_R_EVT)
		{
			os_tsk_delete_self();
		}
	}
}
__task
void LED_Blinking_task(void)// LED������˸����
{
	OS_RESULT result;

	for(;;)
	{
		ER_LED_ON();// ERR����
		os_dly_wait(10);// ��100ms
		ER_LED_OFF();// ERR����
		result = os_evt_wait_and(0x0001,10);// ��100ms
		if(result == OS_R_EVT)
		{
			os_tsk_delete_self();
		}
	}
}
__task
void Alarm_task(void)
{
	OS_TID beeptask;   //��ŷ�������������ID
	OS_TID ledtask;    //���LED��˸����ID

	while(1)
	{
		os_evt_clr(ALARM_START_EVENT,tid_alarm_task);
		os_evt_wait_and(ALARM_START_EVENT,0xffff);       // һֱ�ȴ�һ�������ź�
		ledtask = os_tsk_create(LED_Blinking_task,10);   // ����LED��˸����
		beeptask = os_tsk_create(BEEP_task,11);          // ������������������

		os_evt_clr(ALARM_END_EVENT,tid_alarm_task);
		os_evt_wait_and(ALARM_END_EVENT,0xffff);         // һֱ�ȴ�һ�������ź�
		os_evt_set(0x0001,beeptask);                     // ����һ���źţ�������������������
		os_evt_set(0x0001,ledtask);                      // ����һ���źţ�����LED��˸����

		os_dly_wait(50);                                 // ��ʱ500msʹ ��������LED�����Լ�ɾ�����
	}
}

__task
void PCRDS_Blinking_task(void)// PC����RDSͼ����˸����
{
	unsigned char const KB[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};// ���Է���8X8.BMP
	unsigned char disonece=1;
	for(;;)
	{
		//os_evt_clr(PC_START_EVENT,tid_PCRDS_task);
		//os_evt_wait_and(PC_START_EVENT,0xffff);       // һֱ�ȴ�һ�������ź�
		if(menu_set==RESET)	//�����ò˵�ʱ
		{
			if(PCRDS_Blinking_flag)
			{
				disonece=1;
			    os_mut_wait(LCD_mutex,0xffff);
				Draw_UpdateToLCD();
			    if(transfer_flag)
				  Show_Picture(181,1,FH_RD8X8,8,8);
				else
				  Show_Picture(181,10,FH_PC8X8,8,8);
				os_mut_release(LCD_mutex);
				os_dly_wait(50);// ��500ms
		
				os_mut_wait(LCD_mutex,0xffff);
				Draw_UpdateToLCD();
				if(transfer_flag)
				  {
				    Show_Picture(181,1,KB,8,8);//KB--�հ�
				    Show_Picture(181,10,FH_PC8X8,8,8);
				   }
				else
				  {
				   Show_Picture(181,11,KB,8,8);// ��
				   Show_Picture(181,1,FH_RD8X8,8,8);
				   }
				os_mut_release(LCD_mutex);
				os_dly_wait(50);// ��500ms
		   }
		   else
		   {
		   	 
		     if(disonece)
			 {
			 os_mut_wait(LCD_mutex,0xffff);
			 disonece=0;
		     Show_Picture(181,10,FH_PC8X8,8,8);
			 Show_Picture(181,1,FH_RD8X8,8,8);
			 os_mut_release(LCD_mutex);
			 }
		   }

	    }
	}
}

__task
void comdata_task(void)// PC����RDSͼ����˸����
{

	for(;;)
	{
	   os_mut_wait(LCD_mutex,0xffff);
	    if(packet_R_OK)	 //pc
		  {
		   protocol(packet_rbuf);
		   /*
		   if(packet_T_OK==RESET)
           {
			if(1)//USART_GetITStatus(USART1, USART_IT_TXE) != RESET ���ͱ�־�ж�
			{   
				while(packet_tct<packet_sendsize)
				{
					USART_SendData(USART1, packet_tbuf[packet_tct++]);// ����
					//while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
				    os_dly_wait(1);
				}
				//else
				{
					packet_tct = 0;
					packet_T_OK = SET;//�������
				}
		
			}
		   }
		   */

		   }
	    if(packet_R_OK2)//��ת
		  {
		   //packet_R_OK2=RESET;
		   //protocol(packet_rbuf2);
		   ;
		   }
	   os_mut_release(LCD_mutex);
	   os_dly_wait(1);// ��500ms
	}
}



// ���������õģ�����ʱ�ɲ����

#ifdef  USE_FULL_ASSERT	         // �����жϣ��Ƿ��������Ĺ���

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)// ������ʾ����
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

// �ļ��� ----------------------------------------------------
