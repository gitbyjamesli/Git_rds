/************************************************************************

 2012-11-21/文友

*/
// 包含文件----------------------------------------------------------------------

#include "stm32f10x.h"
#include "LCD19264_DriverV20.h"			   // 19264底层
#include "DrawPicture.h"				   // 基于19264的绘图
#include "PT2314.h"						   // 音量、音调IC
#include "DAC5615.h"					   // DAC芯片驱动
#include "QN8027_Driver.h"				   // FM/RDS发射芯片驱动
#include "anjian.h"
#include <RTL.h>
#include <string.h>
#include <stdlib.h>


#include "show.h"
#include "my_datadef.h"                    // 一些数据类型定义 , 如: ID号类型,时间类型
#include "all.h"

// 宏定义-------------------------------------------------------------------------
#define TBVAL_ADD 	1
#define TBVAL_SUB 	0

#define ER_LED_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_14)		   // ER_LED  亮操作
#define ER_LED_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_14)	   // ER_LED  灭操作
#define TX_LED_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_13)		   // Tx_LED  亮操作
#define TX_LED_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_13)	   // Tx_LED  灭操作
#define	BP_RING()      GPIO_ResetBits(GPIOD,GPIO_Pin_2)        // 蜂鸣器  响操作
#define BP_MUTE()      GPIO_SetBits(GPIOD,GPIO_Pin_2)          // 蜂鸣器  静音

#define Pin_FAN_OPEN() GPIO_SetBits(GPIOC,GPIO_Pin_13)		   // 风扇控制管脚输出高	开启风扇
#define Pin_FAN_CLOSE()  GPIO_ResetBits(GPIOC,GPIO_Pin_13)	   // 风扇控制管脚输出低    关闭

#define Pin_PTT_OPEN()  GPIO_ResetBits(GPIOC,GPIO_Pin_0)	       // PTT控制管脚输出低		  开启PTT 
#define Pin_PTT_CLOSE() GPIO_SetBits(GPIOC,GPIO_Pin_0)		   // PTT控制管脚输出高		  关闭PTT
#define PTT_EN		 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)//外部使能端EN高电平


#define ADC_VALUE_Vp   ADCConvertedValue[0]	// V+
#define ADC_VALUE_Vn   ADCConvertedValue[1]	// V-
#define ADC_VALUE_Vt   ADCConvertedValue[2]
#define ADC_VALUE_Vin  ADCConvertedValue[3]	//射频输入激励电压
#define ADC_VALUE_Vi   ADCConvertedValue[4]				
uint32_t Vp_mv = 0,Vn_mv = 0;//V+，V-，单位毫伏

uint16_t const  jiaozheng[]={160,150,140,130,125,120,110,100};
//使用RTX系统的一些宏定义
#define ALARM_START_EVENT    0x0001		   //      报警任务等待的开始信号
#define ALARM_END_EVENT      0x0002		   //      报警任务等待的结束信号

#define PC_START_EVENT    0x0001		   //      报警任务等待的开始信号
#define PC_END_EVENT      0x0002		   //      报警任务等待的结束信号

#define RDS_START_EVENT    0x0001		   //      报警任务等待的开始信号
#define RDS_END_EVENT      0x0002		   //      报警任务等待的结束信号

#define FALSE   0		   //      报警任务等待的开始信号
#define OK      1		   //      报警任务等待的结束信号

uint16_t dav_save=0;//温度故障恢复时用
uint16_t Power_set_save=0;//温度故障恢复时用
//#define RDS_DEVICES_BIG_SPK 3
//#define RDS_DEVICES_MIDDLE_SPK 2
//#define RDS_DEVICES_SMALL_SPK 1
//#define RDS_DEVICES_OFF_SPK 0

 //调音表
/*
unsigned char const  Tiaoyin[]={0x00,0x01,0x02,0x03,
                      0x04,0x05,0x06,0x07,
                      0x0f,0x0e,0x0d,0x0c,
                      0x0b,0x0a,0x09,0x08};
*/
// 我的数据类型定义
typedef struct
{
	uint8_t x;
	uint8_t y;
}PositDef;// 坐标结构类型定义


// 全局变量-----------------------------------------------------------------------

static U64         main_task_stack[800/8];// 给主任务的堆栈
OS_TID             tid_main_task;
OS_TID             tid_alarm_task;
OS_TID             tid_ComDebug;
OS_MUT             LCD_mutex;// 液晶屏操作的互斥量
OS_TID             tid_AutoFanControl;// 自动风扇控制任务的ID
OS_TID             tid_AutoAdjustPower;// 自动功率调整任务的ID
OS_TID             tid_AutoChoiceAudio;// 自动功率调整任务的ID
OS_TID             tid_PCRDS_task;// 自动功率调整任务的ID
OS_TID             tid_comdata_task;// 任务的ID
// 上位机串口的变量
USART_InitTypeDef   USART1_InitStructure; // 存放   串口1的配置参数

FlagStatus            packet_head = RESET;    //帧头即开始接收标志
FlagStatus            packet_head2 = RESET;    //帧头即开始接收标志
FlagStatus            packet_R_OK = RESET;    //接收完毕标志   SET有效
FlagStatus            packet_R_OK2 = RESET;    //接收完毕标志   SET有效
uint8_t               packet_rct;             //接收字节计数
uint8_t               packet_rct2;             //接收字节计数
uint8_t               packet_len;             //包长字节
uint8_t               packet_len2;             //包长字节
uint8_t               packet_rbuf[64];        //接收的数据缓存
uint8_t               packet_rbuf2[32];        //接收的数据缓存
uint8_t               packet_xor;
uint8_t               packet_xor2;

FlagStatus            packet_T_OK = SET;    //发送完成标志 0就绪 1发送中
uint8_t               packet_sendsize;        //发送包大小
uint8_t               packet_tct;
uint8_t               packet_tbuf[32];        //发送的数据缓存

FlagStatus            packet_T_OK2 = RESET;    //发送完成标志 0就绪 1发送中
uint8_t               packet_sendsize2;        //发送包大小
uint8_t               packet_tct2;
uint8_t               packet_tbuf2[2];        //发送的数据缓存
// 接收机串口的变量 （这里是特别串口调试用到的4个变量）
USART_InitTypeDef    USART2_InitStructure; // 串口2的配置参数
uint8_t              Rxbyte_ct;
uint8_t              Rxbuf[3];
uint8_t              Txbyte;
FlagStatus           S2_receive_flag;// 串口接收标志


// 其它全局定义
struct TimeTypeDef   SystemTime={0,0,0}; // 时间结构    时/分/秒
FlagStatus           SystemTime_SecUpdate_Flag;       // 秒更新标志
uint8_t              Volume_value=32;    // 总音量值		值域	0~31
uint8_t              Bass_value=7;       // 低音值			值域	0~14
uint8_t              Treble_value=7;     // 高音值			值域	0~14
uint8_t              PE_value=50;        // PE值            值域    50或75
uint8_t              TP_set=99;        // 温度保护值		值域    60~80
uint8_t              SWRP_set=25;      // SWR保护值       值域    20~30
uint8_t              IP_value=3;         // 电流保护值		值域    ???(暂不明 !)
uint8_t              PS_Name[10][8]={"ABCDEFGH"};	 // 10个PS名
uint8_t              Area_Name[20][7]={"BCDEFGH"};   // 20个区域名
uint8_t              RDS_DevicesGroup_Mode[24][16];  // 整个区域的设备状态临时缓存。注意，每组15个终端，最后一个字节其它用

uint32_t             FM_value=101700;      // 当前FM频率值
uint16_t             Power_set=20;	   // 设置的功率值
uint16_t             Power_value=0;	       // 当前正向功率值
uint16_t             RP_value=100;         // 当前反向功率值
uint8_t              SWR_value=0;		   // 当前SWR
uint16_t              I_value=0;		       // 当前电流
uint8_t              T_value=25;		   // 当前温度
uint16_t             Excitation_V=0;     // 当前激励电压值

uint32_t           	 sendstate_Time_Count;
uint32_t             Menu_Timeout_Count;    // 菜单操作超时计数变量  在TIM3中 -1 计数
uint16_t             CursorBlink_Count;     // 光标翻转倒计时		 在TIM3中 -1 计数
uint16_t             CollectUpdate_Interval;// 温度值更新间隔计数
uint32_t             sound_ch_switch_Timeout_Count=0;    // 声音通道切换超时计数  在TIM3中 -1 计数

FlagStatus           PTT_status;           // SET状态为发射中，RESET为关闭
FlagStatus           TX_status=RESET;           // SET状态为发射中，RESET为关闭
FlagStatus           setsys_para=RESET;           // SET状态为发射中，RESET为关闭
FlagStatus           menu_set=RESET; 

FlagStatus           HighTempWarning_Flag; // 高温报警状态，SET表示已经报警，RESET表示未进行报警
FlagStatus           HighCURRENTWarning_Flag; // 过流报警状态，SET表示已经报警，RESET表示未进行报警
FlagStatus           HighSWRWarning_Flag; // 过流报警状态，SET表示已经报警，RESET表示未进行报警
FlagStatus           HighPRWarning_Flag;
FlagStatus           HighLPWarning_Flag;
FlagStatus           VinWarning_Flag; //SET表示低于0.2V

FlagStatus           VIN_AND_EN=FALSE; // 条件满足状态，SET表示已经OK，RESET表示未好，
FlagStatus           fm_rf_set = RESET; //设置状态 
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
uint8_t area_num=0;                // 区域号值
uint8_t rds_state=0;                // rds状态，0-全关，1全开，2-设置

uint8_t PCRDS_Blinking_flag=OFF;
uint16_t Disconnect_pc=0;






// 函数原型------------------------------------------------------------------------
__task void InitTask(void);// 第一个开始运行的任务
__task void main_task(void);// 主任务
__task void Alarm_task(void);// 蜂鸣器报警提示任务
__task void ComDebug_task(void);
__task void RDS_task(void);// 小李的RDS任务函数声明

__task void AutoFanControl_task(void);// 自动风扇启停控制任务
__task void AutoAdjustPower_task(void);
__task void AutoChoiceAudio_task(void);//自动音频选择
__task void PCRDS_Blinking_task(void);//自动音频选择
__task void comdata_task(void);//处理串口数据

void Delay_ms(u32 nCount)// 空操作延时函数
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

	/* 使能外设时钟 (APB2)*/
	RCC_APB2PeriphClockCmd(   RCC_APB2Periph_GPIOA
	                         |RCC_APB2Periph_GPIOB
	                         |RCC_APB2Periph_GPIOC
	                         |RCC_APB2Periph_GPIOC
	                         |RCC_APB2Periph_GPIOD
						     |RCC_APB2Periph_AFIO		// 有用到 AFIO   （功能复用IO）
						   , ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;// 上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0
	                             |GPIO_Pin_1
								 |GPIO_Pin_2
								 |GPIO_Pin_5
								 |GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;// 上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);// 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);// 外部中断0   选择GPIOB   -> PB.0
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);// 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);//
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);//
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);//

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     // 
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // 使能
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
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // 上升沿  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // 使能
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_Init(&EXTI_InitStructure);
	*/
// 下面为其它io的配置


	// TX_LED -> PB13  ,   ER_LED -> PB14   都设置为强驱
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14);// 两个指示灯初始状态为灭	 ，实际上此时MCU已经输出为0
	// PC13  强驱   PTT控制管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	Pin_FAN_CLOSE();
	Pin_PTT_CLOSE();


	// PD2  开漏输出   蜂鸣器控制管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	BP_MUTE();                                  // 先静音

	
	// PA8设置为时钟输出（属该引脚的复用功能）
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	RCC_MCOConfig(RCC_MCO_HSE); 


}

void RTC_Configuration(void)
{
	/* Enable PWR and BKP clocks */
	/* PWR时钟（电源控制）与BKP时钟（RTC后备寄存器）使能 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	
	/* Allow access to BKP Domain */
	/*使能RTC和后备寄存器访问 */
	PWR_BackupAccessCmd(ENABLE);
	
	/* Reset Backup Domain */
	/* 将外设BKP的全部寄存器重设为缺省值 */
	BKP_DeInit();
	
	/* Enable LSE */
	/* 使能LSE（外部32.768KHz低速晶振）*/
	RCC_LSEConfig(RCC_LSE_ON);
	
	/* Wait till LSE is ready */
	/* 等待外部晶振震荡稳定输出 */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
	
	/* Select LSE as RTC Clock Source */
	/*使用外部32.768KHz晶振作为RTC时钟 */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	
	/* Enable RTC Clock */
	/* 使能 RTC 的时钟供给 */
	RCC_RTCCLKCmd(ENABLE);
	
	/* Wait for RTC registers synchronization */
	/*等待RTC寄存器同步 */
	RTC_WaitForSynchro();
	
	/* Wait until last write operation on RTC registers has finished */
	/* 等待上一次对RTC寄存器的写操作完成 */
	RTC_WaitForLastTask();
	
	/* Enable the RTC Second */
	/* 使能RTC的秒中断 */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	
	/* Wait until last write operation on RTC registers has finished */
	/* 等待上一次对RTC寄存器的写操作完成 */
	RTC_WaitForLastTask();
	
	/* Set RTC prescaler: set RTC period to 1sec */
	/* 32.768KHz晶振预分频值是32767,如果对精度要求很高可以修改此分频值来校准晶振 */
	RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
	
	/* Wait until last write operation on RTC registers has finished */
	/* 等待上一次对RTC寄存器的写操作完成 */
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
	DMA_InitTypeDef DMA_InitStructure;      //DMA初始化结构体声明

	// 1. 串口1/2  需要使用GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 串口1/2  需要使用GPIOA


	// 2. 串口2外设的初始化配置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);// 串口2外设时钟   打开
	
	// USART2的 Rx  (PA3)->浮空   Tx  (PA2)->输出
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

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);// 使能接收中断
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// 使能发送中断
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);// 接收中断  禁止
	//USART_ITConfig(USART2, USART_IT_TXE, DISABLE); // 发送中断  禁止
	USART_Cmd(USART2, ENABLE);                     // 最后串口2使能

	/*
	// DMA1 channel1 configuration ----------------------------------------------
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel7);                  // 先重置
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40004404;//USART2->DR;    //DMA对应的外设基地址	   //0x40004404;//
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)packet_tbuf;          //内存存储基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	                   //DMA的转换模式为DST模式，由内存搬移到外设
	DMA_InitStructure.DMA_BufferSize = 64;		                           //DMA缓存大小，1个,单位为DMA_MemoryDataSize
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	   //接收一次数据后，设备地址禁止后移
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                //接收一次数据后，目标内存地址后移
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//定义外设数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        //DMA搬数据尺寸，8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                        //循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                   //DMA优先级高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		                   //M2M模式禁用
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);// 再将配置写入          
	

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); // 串口2的发送DMA使能
	DMA_Cmd(DMA1_Channel7, ENABLE);
	*/
//	while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);

///*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);// 串口1外设时钟   打开

	// USART1的 Rx(PA10)->上拉输入   Tx(PA9)->输出
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

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);// 使能接收中断
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);// 使能发送中断
	USART_Cmd(USART1, ENABLE);// 最后使能串口1 

}

/***************************************************************
 功能：通用定时器3配置为中断方式/10ms
 输入：无
 输出：无
 说明：此函数当前仅当做学习调试用，时间参数随意改
*/

void TIM3Init(void)// 通用定时器3配置，使能中断方式, 10ms定时
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);// 定时器3 时钟使能
	
	TIM_DeInit(TIM3);// 先恢复默认状态

	// 参数配置 
	
	TIM_TimeBaseStructure.TIM_Period = (10*1000 - 1);// 自动装载寄存器值， 单位us
	
	TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);// 预分频系数， 
	
	// 高级应用本次不涉及。定义在定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 分频比为1
	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上计数
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);// 写入设置值
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);// 清除TIM3溢出中断标志
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);// 允许溢出中断
	
	TIM_Cmd(TIM3, ENABLE);  // 计数器使能，开始工作
}
/******************************************************************
  嵌套中断控制器NVIC的配置函数，即将系统中有使用的中断按一定的优先级配
  置进行配置，详细配置见函数体中
  
  注意：当有新增中断需在这里定义优先级。
*******************************************************************/
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	// Configure the NVIC Preemption Priority Bits 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);// 采用0组格式    即4位全部是亚优先级，抢先级被无效掉
	
	// Enable the USART2 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;// 通道号
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// 抢先级0      （此时是无效的）
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;// 副优先级4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 启动设置

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
/*	
	// Enable the TIM3 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// 抢先级 0  （此时是无效的）
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;// 副优先级 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 启动设置
*/	
	// Enable the RTC gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// 抢先级 0  （此时是无效的）
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;// 副优先级 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 启动设置

	// Enable the TIM3 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// 抢先级 0  （此时是无效的）
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;// 副优先级 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);// 启动设置

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
void UpdateSystemTime(void)// 更新系统时间
{
	uint32_t rtc_value = RTC_GetCounter();

	SystemTime.hour = rtc_value/3600%24;		
	SystemTime.mins = rtc_value%3600/60;
	SystemTime.sec = rtc_value%60;
}
void SetupSystemTime(struct TimeTypeDef *time)// 设置RTC时间
{
	uint32_t value;

	value = time->hour*3600 + time->mins*60 + time->sec;

	RTC_WaitForLastTask();/* 等待上一次对RTC寄存器的写操作完成 */

	RTC_SetCounter(value);

	RTC_WaitForLastTask();/* 等待上一次对RTC寄存器的写操作完成 */
}
/*******************分割线*****************************************
 下为系统参数设置界面的函数，FM，功率、声音、时间、温度等设置
***/
void SET_FM_And_RF_Power(void)
{
    u8 fm_change=0;  
	u8 set_target=1;           // 设置目标
	FlagStatus gb_flag=RESET;  // 光标状态  SET亮，RESET灭

	KEY_Status_AllClear();// 清除全部按键按下标志
	Draw_Fill(1,39,190,63,0);// 画一空白区域

	Draw_String6X8(1,39,"SET FM AND RF POWER");
	Draw_String6X8(20,47,"FM:     MHz AutoSet: ");
	Draw_String6X8(20,55,"RF POWER:    W");

showpara:
	Menu_Timeout_Count=1000; // 菜单超时“喂狗”10s
	CursorBlink_Count=50;    // 光标闪烁计时  重置500ms
// FM频段值
	SetingWindos_ShowFMchValue(FM_value);
// 功率值
	SetingWindos_ShowPowerValue(Power_set);
// 自动匹配
	if(set_target==2)
	{
		Draw_Ascii6X8(20+6*20,47,'>');
	}
	else
	{
		Draw_Ascii6X8(20+6*20,47,' ');
	}
// 退出图标
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
		if(SystemTime_SecUpdate_Flag == SET)// 系统时间秒更新
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// 显示时间
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
			UP_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			switch (set_target)
			{
				case 1:
					FM_value+=100;// 100KHz 步进
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
			DOWN_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			switch (set_target)
			{
				case 1:
					FM_value-=100;// 100KHz 步进
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
		if(Menu_Timeout_Count==0)// 菜单操作超时
		{
		   SYS_PARAMETER_Save(); //超时退出也保存
			break;
		}
		if(CursorBlink_Count==0)// 光标灭时间到
		{
			CursorBlink_Count=50;// 重置500ms
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

	 if(fm_change==1&&DOWN_LockFlag==RESET)//松开按键时再设置
	  {
	   fm_change=0;
	   QNd8027_SetChannel((FM_value-76000)/50);
	   }
	}

}

void TBval_Change(u8 *TBval,u8 addsub)// 将输入的高音(低音值)   加或减1
{
									// 算法需参考PT2314的Bass/Treble取值方法
	if( (*TBval) & 0x08)
	{
		if(addsub)// 加音操作
		{
			if(0 != ((*TBval)&0x07))    (*TBval)--;
				
		}
		else// 减音操作
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
		if(addsub)// 加操作
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
		else // 减操作
		{
			if( 0 != ((*TBval)&0x07))  (*TBval)--;
		}
	}
}

void SET_Audio_And_Time(void)
{
	uint8_t set_target=1;
	uint8_t petmp=0;						// PE值   0 ->50us   1->75us
	uint8_t disnum;
	FlagStatus gb_flag = RESET;
	struct TimeTypeDef  temp_time;			// 临时存放时间修改值	

	UpdateSystemTime();
	temp_time.sec = SystemTime.sec;
	temp_time.mins = SystemTime.mins;
	temp_time.hour = SystemTime.hour;

	KEY_Status_AllClear();// 清除全部按键按下标志
	Draw_Fill(1,39,190,63,0);// 画一空白区域

	Draw_String6X8(1,39,"SET Audio AND Time");
	Draw_String6X8(20,47,"VOL:--PE:--us H:--");
	Draw_String6X8(20,55,"Time --:--:-- B:--");

show_para:
	Menu_Timeout_Count=1000;// 菜单超时“喂狗”10s
	CursorBlink_Count=50;   // 光标闪烁计时 500ms
// 音量的显示
	Draw_Ascii6X8(20+6*4,47,Volume_value/10+'0');
	Draw_Ascii6X8(20+6*5,47,Volume_value%10+'0'); 
// PE值的显示
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
// 高音值的显示
    if(Treble_value>7)
         disnum = 23-Treble_value;// dB值
	else disnum=Treble_value;
	Draw_Ascii6X8(20+6*16,47,disnum/10+'0');
	Draw_Ascii6X8(20+6*17,47,disnum%10+'0');
// 低音值的显示
    if(Bass_value>7)
         disnum = 23-Bass_value;// dB值
	else disnum=Bass_value;
	Draw_Ascii6X8(20+6*16,55,disnum/10+'0');
	Draw_Ascii6X8(20+6*17,55,disnum%10+'0');
// 小时值的显示
	Draw_Ascii6X8(20+6*5,55,temp_time.hour/10+'0');
	Draw_Ascii6X8(20+6*6,55,temp_time.hour%10+'0');
// 分钟值的显示
	Draw_Ascii6X8(20+6*8,55,temp_time.mins/10+'0');
	Draw_Ascii6X8(20+6*9,55,temp_time.mins%10+'0');
// 秒钟值的显示
	Draw_Ascii6X8(20+6*11,55,temp_time.sec/10+'0');
	Draw_Ascii6X8(20+6*12,55,temp_time.sec%10+'0');
// 显示退出按钮状态
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
		if(SystemTime_SecUpdate_Flag == SET)// 系统时间秒更新
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// 显示时间
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
			UP_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			switch (set_target)
			{
				case 1:// 总音量
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
					TBval_Change(&Treble_value,TBVAL_ADD);// 系统高音值加1
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
					TBval_Change(&Bass_value,TBVAL_ADD);// 系统低音音值加1
					PT2314_Setup_Bass(Bass_value);
					psys_data->ui_Audio_B=Bass_value;
					break;
				default :break;
			}
			goto show_para;
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			DOWN_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			switch (set_target)
			{
				case 1:// 总音量
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
					TBval_Change(&Treble_value,TBVAL_SUB);// 系统高音值减1
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
					TBval_Change(&Bass_value,TBVAL_SUB);// 系统低音值减1
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
				SetupSystemTime(&temp_time);// 写入RTC中
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
					case 1:// 总音量  
						Draw_Fill(44,    47,    55,      54,      0);// 填充空白
						break;
					case 2:// pe值
						Draw_Fill(20+6*9,  47,   74+11,    54,   0);// 填充空白
						break;
					case 3:// 高音
						Draw_Fill(116,    47,    116+11,  54,    0);// 填充空白
						break;
					case 4:// 小时
						Draw_Fill(50,        55,    61,     63,     0);// 填充空白
						break;
					case 5:// 分钟
						Draw_Fill(68 ,       55,    79,    63,     0);// 填充空白
						break;
					case 6:// 秒
						Draw_Fill(86,        55,    97,    63,      0);// 填充空白
						break;
					case 7:// 低音
						Draw_Fill(116,   55,    116+11,   63,   0);// 填充空白
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
					// 音量的显示
						Draw_Ascii6X8(20+6*4,47,Volume_value/10+'0');
						Draw_Ascii6X8(20+6*5,47,Volume_value%10+'0'); 
					 break;
					case 2:
					// PE值的显示
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
					// 高音值的显示
					    if(Treble_value>7)
					         disnum = 23-Treble_value;// dB值
						else disnum=Treble_value;
						Draw_Ascii6X8(20+6*16,47,disnum/10+'0');
						Draw_Ascii6X8(20+6*17,47,disnum%10+'0');
					  break;
					  case 7:
					// 低音值的显示
					    if(Bass_value>7)
					         disnum = 23-Bass_value;// dB值
						else disnum=Bass_value;
						Draw_Ascii6X8(20+6*16,55,disnum/10+'0');
						Draw_Ascii6X8(20+6*17,55,disnum%10+'0');
					  break;
					case 4:
					// 小时值的显示
						Draw_Ascii6X8(20+6*5,55,temp_time.hour/10+'0');
						Draw_Ascii6X8(20+6*6,55,temp_time.hour%10+'0');
					  break;
					case 5:
					// 分钟值的显示
						Draw_Ascii6X8(20+6*8,55,temp_time.mins/10+'0');
						Draw_Ascii6X8(20+6*9,55,temp_time.mins%10+'0');
					  break;
					case 6:
					// 秒钟值的显示
						Draw_Ascii6X8(20+6*11,55,temp_time.sec/10+'0');
						Draw_Ascii6X8(20+6*12,55,temp_time.sec%10+'0');
					break;	
					default :break;
				}
			}
		}
		if(Menu_Timeout_Count==0)// 菜单操作超时
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
	//uint8_t swr_value=20;// SWR保护值      注意：这里是小数转整数处理

	KEY_Status_AllClear();// 清除全部按键按下标志
	Draw_Fill(1,39,190,63,0);// 画一空白区域

	Draw_String6X8(1,39,"SET Temp AND SWR");
	Draw_String6X8(20,47,"HIGH TEMP:");Show_Picture(20+6*12,47,FH_du8X8,8,8);// 显示符号 °C
	Draw_String6X8(20,55,"HIGH SWR:");
showpara:
	Menu_Timeout_Count=1000;// 菜单超时“喂狗”10s
	CursorBlink_Count=50;   // 光标闪烁计时 500ms

// 高温 HIGH TEMP保护值	
	Draw_Ascii6X8(20+6*10,47,TP_set/10+'0');
	Draw_Ascii6X8(20+6*11,47,TP_set%10+'0');
// SER保护值
	Draw_Ascii6X8(20+6*9,55,SWRP_set/10+'0');
	Draw_Ascii6X8(20+6*10,55,'.');
	Draw_Ascii6X8(20+6*11,55,SWRP_set%10+'0');
// 显示退出按钮状态
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
		if(SystemTime_SecUpdate_Flag == SET)// 系统时间秒更新
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// 显示时间
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
			UP_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
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
			DOWN_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
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
		if(KEY_MENU_STATUS() == SET)// 菜单键==返回键
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
					case 1:Draw_Fill(80,   47,    91,   54,   0);;// 填充空白
						break;
					case 2:Draw_Fill(74,   55,    91,   63,   0);;// 填充空白
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
					// 高温 HIGH TEMP保护值	
						Draw_Ascii6X8(20+6*10,47,TP_set/10+'0');
						Draw_Ascii6X8(20+6*11,47,TP_set%10+'0');
					 break;
					case 2:
					// SER保护值
						Draw_Ascii6X8(20+6*9,55,SWRP_set/10+'0');
						Draw_Ascii6X8(20+6*10,55,'.');
						Draw_Ascii6X8(20+6*11,55,SWRP_set%10+'0');
					  break;	
					default :break;
				}
			}
		}
		if(Menu_Timeout_Count==0)// 菜单操作超时
		{
		    SYS_PARAMETER_Save();
			break;
		}
	}
}



/*******************分割线*****************************************
 下为RDS参数设置界面的函数
***/
void Print_ZF_Table(uint8_t x,uint8_t y)// 屏幕打印出字符表
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
	uint8_t Name_pos=0;// 名称位置或操作类型
	struct
	{
		uint8_t x; // 0~21
		uint8_t y; // 0~3
	}zf_pos={0,0};// 字符位置
	uint8_t edit_flag=0;// 非0 =字符选择状态    0=操作选择状态
	uint8_t name_string[8]={0};

	memcpy(name_string,&PS_Name[num][0],8);
	Draw_Clear();
	KEY_MENU_CLEAR();
	Draw_Rect(0,0,        191,63,  1); // 屏幕大方框
	Draw_Rect(2,12,        150,61,  1);// 字符表方框
	Print_ZF_Table(8,14);              // 显示字符表
	Draw_Ascii6X8(1,2,'[');
	Draw_Ascii6X8(7,2,num+'0');
	Draw_Ascii6X8(13,2,']');
	Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s	
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
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
			if(edit_flag)// 字符选择状态
			{
				if(Name_pos<8)
				{
					Draw_XLine(8 + 6*zf_pos.x,   14+8+12*zf_pos.y,  8 + 6*zf_pos.x+6,   0);// 清字符区光标
					edit_flag=0;// 回到操作选择状态
					name_string[Name_pos] = ZF_Table[zf_pos.y][zf_pos.x];// 由当前的字符坐标来查表获得字符值
					++Name_pos;
					goto name_pos_update;// 跳回，显示名称处光标
				}
			}
			else
			{
				if(Name_pos==8)// 保存处
				{
					memcpy(&PS_Name[num][0],name_string,8);
					PS_Name_Save();
					ps_int();
					return ;
				}
				else if(Name_pos==9)// 退出处
				{
					return ;
				}
				else
				{
					edit_flag=1;//进入字符选择状态
					zf_pos.x=0;
					zf_pos.y=0;
					Draw_XLine(8 + 6*zf_pos.x,   14+8+12*zf_pos.y,  8 + 6*zf_pos.x+6,  1);// 显示字符区光标
				}
			}
		}
		if(KEY_UP_STATUS() == SET)
		{
			UP_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
			DOWN_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
			LEFT_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
				goto name_pos_update;// 跳回，显示名称处光标
			}
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			RIGHT_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
				goto name_pos_update;// 跳回，显示名称处光标
			}
		}
		if(KEY_MENU_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
	uint8_t Name_pos=0;// 名称位置或操作类型
	struct
	{
		uint8_t x; // 0~21
		uint8_t y; // 0~3
	}zf_pos={0,0};// 字符位置
	uint8_t edit_flag=0;// 非0 =字符选择状态    0=操作选择状态
	uint8_t name_string[7]={0};

	memcpy(name_string,&Area_Name[num][0],7);
	Draw_Clear();
	KEY_MENU_CLEAR();
	Draw_Rect(0,0,        191,63,  1); // 屏幕大方框
	Draw_Rect(2,12,        150,61,  1);// 字符表方框
	Print_ZF_Table(8,14);              // 显示字符表
	Draw_Ascii6X8(1,2,'[');
	Draw_Ascii6X8(7,2,num/10+'0');
	Draw_Ascii6X8(13,2,num%10+'0');
	Draw_Ascii6X8(19,2,']');
	Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s	
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
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
			if(edit_flag)// 字符选择状态
			{
				if(Name_pos<7)
				{
					Draw_XLine(8 + 6*zf_pos.x,   14+8+12*zf_pos.y,  8 + 6*zf_pos.x+6,   0);// 清字符区光标
					edit_flag=0;// 回到操作选择状态
					name_string[Name_pos] = ZF_Table[zf_pos.y][zf_pos.x];// 由当前的字符坐标来查表获得字符值
					++Name_pos;
					goto name_pos_update;// 跳回，显示名称处光标
				}
			}
			else
			{
				if(Name_pos==7)// 保存处
				{
					memcpy(&Area_Name[num][0],name_string,7);
					Area_Name_Save();
					return ;
				}
				else if(Name_pos==8)// 退出处
				{
					return ;
				}
				else
				{
					edit_flag=1;//进入字符选择状态
					zf_pos.x=0;
					zf_pos.y=0;
					Draw_XLine(8 + 6*zf_pos.x,   14+8+12*zf_pos.y,  8 + 6*zf_pos.x+6,  1);// 显示字符区光标
				}
			}
		}
		if(KEY_UP_STATUS() == SET)
		{
			UP_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
			DOWN_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
			LEFT_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
				goto name_pos_update;// 跳回，显示名称处光标
			}
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			RIGHT_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
				goto name_pos_update;// 跳回，显示名称处光标
			}
		}
		if(KEY_MENU_STATUS() == SET)
		{
			break;
		}
		if(Menu_Timeout_Count==0)// 菜单操作超时
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
	Show_Picture(x2,y2,pic,8,11);// 显示大喇叭
	
}
void Show_ALLSET_devgb(uint8_t x,uint8_t y)// 设备光标反白，x(0~15)列,y(0~3)行
{
	uint8_t x2 = 16+10*x;
	uint8_t y2 = 18+11*y;
	Draw_Lump_fb(x2,y2,x2+7,y2+10);
}
void Show_DevStatus_Group(uint8_t line_num,uint8_t znum)// 显示终端设备整组状态，line_num在0~3行，znum为组号
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
void Show_Device_hangnum(void)// 显示行序号
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
void Show_Strip(void)// 显示滚动条
{
}
void Show_ALLSET_Area_Name(uint8_t num,uint8_t select_f)// 显示区域名
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
	Show_Picture(73,1,pic,8,8);// 显示大喇叭
	if(select_f)
	{
		Draw_Lump_fb(73,1,73+7,1+7);
	}
}
void GetArea_DevStatus(uint8_t num)// 从24C64中读取整个区域的设备状态放入RDS_DevicesGroup_Mode[24][15]中
{

   RDS_DevicesGroup_Mode_Update(num);

}
void Setup_RDSMode(uint8_t num)
{
	if(num==2)// RDS终端全设置
	{

/*
 
 set_target    有4种值，分别表示4种状态：
        1      区域名处				（左-右键可-+set_target，上下键修改值）	 
		2      区域名统一修改处		（左-右键可-+set_target，上下键修改值）
		3	   在SAVE处				（左-右键可-+set_target，上下键修改值）
		4	   在EXIT处				（左-右键可-+set_target，上下键修改值）	 

 partset_flag  
		当非0时表示“部分设置”状态    此时上-下-左-右键可编辑dev_select_pos，当到达顶部上键可退清0次标志
         	
*/
		uint8_t set_target=1;	
		uint8_t partset_flag=0;            // 部分编辑状态标志  先处于非部分编辑状态
		//uint8_t area_num=0;                // 区域号值
		uint8_t area_fullsetval=0;         // 区域统一设置值，0-1-2-3种值
//		uint8_t group_fluusetval[24];      // 24个组的统一设置值，
		PositDef dev_select_pos={0,0};     // 部分修改状态下的坐标(或光标)
		uint8_t y_val=0;                   // 屏幕y坐标0~3
		uint8_t x_val=0;                   // 屏幕x坐标0~14

		PCRDS_Blinking_flag=OFF;
		Draw_Clear();
		Draw_Rect(0,0,        191,63,  1);       // 屏幕大方框
		Draw_Line(0,9,        191, 9,  1);       // 一横线
		Show_Device_hangnum();                   // 显示行序号
        gb_area_num=&area_num;
        gb_area_setval=&area_fullsetval;
		gb_Group_num=&(dev_select_pos.y);
		gb_terminal_num=&(dev_select_pos.x);
area_update:
		//GetArea_DevStatus(area_num);             // 获取整个区域下的终端状态信息
		Show_DevStatus_Group(0,0);               // 0行上显示0组终端状态
		Show_DevStatus_Group(1,1);               // 1行	  1组
		Show_DevStatus_Group(2,2);               // 2行	  2组
		Show_DevStatus_Group(3,3);               // 3行	  3组
		dev_select_pos.x = dev_select_pos.y = x_val = y_val= 0;	 // 坐标复位
gb_update:
		if(set_target==1)
		{
			Show_ALLSET_Area_Name(area_num,1);       // 显示区域名，选择状态
		}
		else
		{
			Show_ALLSET_Area_Name(area_num,0);       // 显示区域名，非选状态
		}

		if(set_target==2)
		{
			Show_ALLSET_Area_Fullsetval(area_fullsetval,1);		 // 显示区域统一修改值，选择状态
		}
		else
		{
			Show_ALLSET_Area_Fullsetval(area_fullsetval,0);		 // 显示区域统一修改值，非选状态
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
			if(Menu_Timeout_Count==0)// 菜单操作超时
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
				Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
				if(partset_flag)
				{
					uint8_t *temp = &RDS_DevicesGroup_Mode[dev_select_pos.y][dev_select_pos.x];
					if(++(*temp)>3)// 值范围0~3  分别代表 关、小、中、大
					{
						(*temp)=0;
					}
					if(dev_select_pos.x==15)
					{
						memset(&RDS_DevicesGroup_Mode[dev_select_pos.y],(*temp),15);// 复制给其它
						Show_DevStatus_Group(y_val,  dev_select_pos.y);// 更新屏幕上的整行
						Show_ALLSET_devgb(x_val,  y_val);// 反白
						gb_Group_setval=&(*temp);
						area_group_terminal_set_flag=2;;//整组统一设置命令，待添加
					}
					else
					{
						Show_DevStatus_One(x_val,y_val,*temp);
						Show_ALLSET_devgb(x_val,y_val);// 反白
						gb_terminal_setval=&(*temp);
						area_group_terminal_set_flag=3;;//单个终端设置命令，待添加
					}
				}
				else
				{
					if(set_target==2)
					{
						if(++area_fullsetval>3)// 值范围0~3  分别代表 关、小、中、大
							area_fullsetval=0;
						Show_ALLSET_Area_Fullsetval(area_fullsetval,1);// 显示统一修改的值，选择状态
						memset(RDS_DevicesGroup_Mode,area_fullsetval,24*16);// 复制给其它
						Show_DevStatus_Group(3,3);// 3行
						Show_DevStatus_Group(2,2);// 2行
						Show_DevStatus_Group(1,1);// 1行	
						Show_DevStatus_Group(0,0);// 0行 

						area_group_terminal_set_flag=1;;// 区域统一设置命令待添加中
					}
					else if(set_target==3)
					{
					// 保存键
					RDS_DevicesGroup_Mode_Save(area_num);
					}
					else if(set_target==4)
					{// 退出键
						area_group_terminal_set_flag=0;
						area_allset_flag=0;
						return ;
					}
				}
			}
			if(KEY_UP_STATUS() == SET)
			{
				UP_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
				Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
				if(partset_flag)
				{
					if(dev_select_pos.y==0)
					{
						Show_ALLSET_devgb(x_val,0);// 复原
						partset_flag=0;
						goto gb_update;
					}
					else
					{
						dev_select_pos.y--;
						if(y_val>0)
						{
							Show_ALLSET_devgb(x_val,y_val);// 复原 	
							y_val--;
							Show_ALLSET_devgb(x_val,y_val);// 反白 	
						}
						else
						{
							Show_DevStatus_Group(3,dev_select_pos.y+3);// 3行
							Show_DevStatus_Group(2,dev_select_pos.y+2);// 2行
							Show_DevStatus_Group(1,dev_select_pos.y+1);// 1行	
							Show_DevStatus_Group(0,dev_select_pos.y+0);// 0行 
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
//							Show_ALLSET_Area_Fullsetval(area_fullsetval,1);// 显示统一修改的值，选择状态
							break;
						default :break;
					}
				}
			}
			if(KEY_DOWN_STATUS() == SET)
			{
				DOWN_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
				Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
				if(partset_flag)
				{
					if(dev_select_pos.y<23)
					{
						dev_select_pos.y++;
						if(y_val<3)
						{
							Show_ALLSET_devgb(x_val,y_val);// 复原 	
							y_val++;
							Show_ALLSET_devgb(x_val,y_val);// 反白 	
						}
						else
						{
							Show_DevStatus_Group(0,dev_select_pos.y-3);// 0行
							Show_DevStatus_Group(1,dev_select_pos.y-2);// 1行
							Show_DevStatus_Group(2,dev_select_pos.y-1);// 2行	
							Show_DevStatus_Group(3,dev_select_pos.y-0);// 3行
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
//							Show_ALLSET_Area_Fullsetval(area_fullsetval,1);// 显示统一修改的值，选择状态
							break;
						default :break;
					}
				}
			}
			if(KEY_LEFT_STATUS() == SET)
			{
				LEFT_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
				Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
				if(partset_flag)
				{
					Show_ALLSET_devgb(x_val,y_val);// 原来的位置复原

					if(--dev_select_pos.x>15)dev_select_pos.x=15;
					x_val = dev_select_pos.x;

					Show_ALLSET_devgb(x_val,y_val);// 现在的位置反白
				}
				else
				{
					if(--set_target==0)set_target=4;
					if(set_target==2)
					{
						Draw_String6X8(128,1," SAVE");// 清光标
						partset_flag=1;
						Show_ALLSET_devgb(x_val,y_val);// 现在的位置反白
					}
					else
					{
						goto gb_update;
					}
				}
			}
			if(KEY_RIGHT_STATUS() == SET)
			{
				RIGHT_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
				Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
				if(partset_flag)// 部分设置状态
				{
					Show_ALLSET_devgb(x_val,y_val);// 原来的位置复原

					if(++dev_select_pos.x>15)dev_select_pos.x=0;
					x_val = dev_select_pos.x;

					Show_ALLSET_devgb(x_val,y_val);// 现在的位置反白
				}
				else
				{
					if(++set_target>4)set_target=1;
					if(set_target==3)
					{
						Show_ALLSET_Area_Fullsetval(area_fullsetval,0);// 显示区域统一修改值，非选状态
						partset_flag=1;
						Show_ALLSET_devgb(x_val,y_val);// 现在的位置反白
					}
					else
					{
						goto gb_update;
					}
				}
			}
		}// whlie(1)	
	}
	else if(num==1)// RDS终端全开启
	{
		area_group_terminal_set_flag=4;// 这里待添加命令
	}
	else // RDS终端全部关闭
	{
		area_group_terminal_set_flag=5;// 这里待添加命令
	}
}
void Setup_RDS_Window(void)
{
	uint8_t gb_pos=1;
	uint8_t PSName_pos=0,AreaName_pos=0,RDSMode_pos=rds_state;

	Draw_Fill(1,39,190,63,0);// 画一空白区域
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
	Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
	while(1)
	{
		if(SystemTime_SecUpdate_Flag == SET)// 系统时间秒更新
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// 显示时间
		}
		if(KEY_UP_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
			Draw_Ascii6X8(1,39+(gb_pos-1)*8,' ');
			Draw_Ascii6X8(80,39+(gb_pos-1)*8,' ');
			Draw_Ascii6X8(80+6*11,39+(gb_pos-1)*8,' ');
			if(--gb_pos < 1)
				gb_pos=3;
			goto gb_update;
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
			Draw_Ascii6X8(1,39+(gb_pos-1)*8,' ');
			Draw_Ascii6X8(80,39+(gb_pos-1)*8,' ');
			Draw_Ascii6X8(80+6*11,39+(gb_pos-1)*8,' ');
			if(++gb_pos > 3)
				gb_pos=1;
			goto gb_update;
		}
		if(KEY_LEFT_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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
						SYS_PARAMETER_Save();//保存区域号
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
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
			switch (gb_pos)
			{
				case 1:
					if(PSName_pos<9)
					{
						PSName_pos++;
						pty_num=PSName_pos;
						psys_data->ui_psnum=PSName_pos;
						SYS_PARAMETER_Save();//保存
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
						SYS_PARAMETER_Save();//保存区域号
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
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
			os_dly_wait(10);
			while((KEY_MENU_STATUS() == SET)) ;
			area_group_terminal_set_flag=0;
			break;
		}
		if(KEY_RDS_STATUS() == SET)	//防止在这个设置界面下按RDS后超时退出又回到本界面
		{
         KEY_RDS_CLEAR();
		}
		if(Menu_Timeout_Count==0)// 菜单操作超时
		{
			break;
		}

		if(KEY_OK_STATUS() == SET)
		{
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s
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

			if(Menu_Timeout_Count==0)return;// 菜单操作超时
				
			Draw_Clear();	
			Draw_Rect(0,0,        191,63,  1);// 屏幕大方框
			Draw_Rect(0,0,         65,19,  1);// 左上角方框
			Draw_Line(0,19,       179,19,  1);// 屏幕中间一横线
			Draw_Line(0,37,       191,37,  1);// 屏幕中间一横线
			Draw_Rect(179,0,      191,20,  1);// 方框
			Draw_Line(179,10,     191,10,  1);// 线
			Show_RDS_logox();//	小RDS图标
			Show_Picture(181,1,FH_RD8X8,8,8);
			Show_Picture(181,10,FH_PC8X8,8,8);
		
			Show_RDS(rds_state);// 显示RDS状态
			Show_TX(PTT_status);// 显示发射状态
			(PTT_status==SET) ? TX_LED_ON():TX_LED_OFF();
			Show_Time(&SystemTime);// 显示时间
			Show_FM(FM_value);// 显示FM频率值
			Show_I(I_value);// 显示电流值
			Show_T(T_value);// 显示温度值
			Show_P(Power_value);// 显示功率值
			Show_RP(RP_value);// 显示保护功率值
			Show_SWR(SWR_value);// 显示SWR值
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

    n=9;  //排序 ,去极值均值滤波
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_T[min]>ADValue_T[j])min=j;
        temp=ADValue_T[i];
        ADValue_T[i]=ADValue_T[min];
        ADValue_T[min]=temp;      
     }

    SumMiddle=0;     //对中间3个值求和
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_T[i];

    Average= SumMiddle/3;     //对中间3个值求平均
	return Average;

}

float filter_p(float tmp_p)//uint16_t
{
    float Average=0,temp;
	float SumMiddle;
	uint8_t n,i,j,min;

    ADValue_P[count_rp++]=tmp_p;
	if(count_p>9)count_p=0;

    n=9;  //排序 ,去极值均值滤波
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_P[min]>ADValue_P[j])min=j;
        temp=ADValue_P[i];
        ADValue_P[i]=ADValue_P[min];
        ADValue_P[min]=temp;      
     }

    SumMiddle=0;     //对中间3个值求和
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_P[i];

    Average= SumMiddle/3;     //对中间3个值求平均
	return Average;

}


float filter_rp(float tmp_rp)
{
    float Average=0,temp;
	float SumMiddle;
	uint8_t n,i,j,min;

    ADValue_RP[count_rp++]=tmp_rp;
	if(count_rp>9)count_rp=0;

    n=9;  //排序 ,去极值均值滤波
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_RP[min]>ADValue_RP[j])min=j;
        temp=ADValue_RP[i];
        ADValue_RP[i]=ADValue_RP[min];
        ADValue_RP[min]=temp;      
     }

    SumMiddle=0;     //对中间3个值求和
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_RP[i];

    Average= SumMiddle/3;     //对中间3个值求平均
	return Average;

}

float filter_I(float tmp_i)
{
    float Average=0,temp;
	float SumMiddle5;
	uint8_t n,i,j,min;

    ADValue_I[count_i++]=tmp_i;
	if(count_i>9)count_i=0;

    n=9;  //排序 ,去极值均值滤波
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_I[min]>ADValue_I[j])min=j;
        temp=ADValue_I[i];
        ADValue_I[i]=ADValue_I[min];
        ADValue_I[min]=temp;      
     }

    SumMiddle5=0;     //对中间3个值求和
    for (i=4;i<7;i++) 
	  SumMiddle5=SumMiddle5+ADValue_I[i];

    Average= SumMiddle5/3;     //对中间3个值求平均
	return Average;

}

uint16_t filter_swr(uint16_t tmp_swr)
{
    uint16_t Average=0,temp;
	uint32_t SumMiddle;
	uint8_t n,i,j,min;

    ADValue_swr[count_swr++]=tmp_swr;
	if(count_swr>9)count_swr=0;

    n=9;  //排序 ,去极值均值滤波
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_swr[min]>ADValue_swr[j])min=j;
        temp=ADValue_swr[i];
        ADValue_swr[i]=ADValue_swr[min];
        ADValue_swr[min]=temp;      
     }

    SumMiddle=0;     //对中间3个值求和
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_swr[i];

    Average= SumMiddle/3;     //对中间3个值求平均
	return Average;

}


float filter_vin(float tmp_vin)
{
    float Average=0,temp;
	float SumMiddle;
	uint8_t n,i,j,min;

    ADValue_vin[count_vin++]=tmp_vin;
	if(count_vin>9)count_vin=0;

    n=9;  //排序 ,去极值均值滤波
    for (i=0;i<n-1;i++)      
     { 
	    min=i;
        for (j=i+1;j<n;j++)
		  if(ADValue_vin[min]>ADValue_vin[j])min=j;
        temp=ADValue_vin[i];
        ADValue_vin[i]=ADValue_vin[min];
        ADValue_vin[min]=temp;      
     }

    SumMiddle=0;     //对中间3个值求和
    for (i=4;i<7;i++) 
	  SumMiddle=SumMiddle+ADValue_vin[i];

    Average= SumMiddle/3;     //对中间3个值求平均
	return Average;

}


//==========================

uint8_t Get_CollectTempval(void)// 返回温度值，单位°C
{
	uint32_t voltage_uV;//电压值，单位uV
	uint32_t voltage_mV;//电压值，单位mV
	uint8_t tempval;// 0~100 °C

	voltage_uV = ADC_VALUE_Vt*805;
	voltage_mV = voltage_uV/1000;
	if(voltage_mV>=1500)
	{
	 return 99;
	 }
	if(voltage_mV>=500)
	{
		tempval = (voltage_mV-500)/10;// 转换为°C      10mV/°C
		if(tempval<100)
		{
			return tempval;
		}
	}

	return 0;
}
uint16_t Get_CollectCurrentVal(void)// 返回电流值，单位A
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

void CollectVIN(void)//excitation voltage  激励电压
{
	uint32_t val_mV;

	val_mV = (ADC_VALUE_Vin*805)/1000;
    //Excitation_V=(uint16_t)val_mV;	 //单位毫伏
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
		
		RTC_WaitForLastTask();/* 等待上一次对RTC寄存器的写操作完成 */
		
		RTC_SetCounter(0);/* 把RTC计数值写入RTC寄存器 */
		
		RTC_WaitForLastTask();/* 等待上一次对RTC寄存器的写操作完成 */
		
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);/* 修改后备寄存器1的值为0XA5A5 */
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
	NVIC_Configuration();// 系统中断控制器
	
	os_sys_init(InitTask);// RTX操作系统初始化，并创建第一个任务 InitTask
	for(;;);
}

__task
void InitTask(void)
{
	tid_main_task = os_tsk_create_user(main_task, 1, &main_task_stack, sizeof(main_task_stack));
	//tid_alarm_task = os_tsk_create(Alarm_task, 1);// 创建报警任务
	//tid_PCRDS_task = os_tsk_create(PCRDS_Blinking_task, 1);// 创建报警任务

	os_tsk_delete_self();
}

#include "RDS_setup_debug(bak).h"  // 临时调试函数 DebugMode();
__task
void main_task(void)
{
	unsigned int temp1=0;
	unsigned char ledflash=0;
	unsigned char sample_count=0;

	DAC5615Out(0);// 初始化，上电第一时间先让5615输出0V。
	Delay_ms(5000);

	LCD19264_Init();
	LCD19264_BG_Open();
	LCD19264_Display_String(0,0," System Init...");

PT2314init:
	PT2314_Init();// 初始化成功后 ，默认  通道1，总音量0dB，左右平衡，高音/低音0dB
	if(pt2314_error_flag)
	{
		LCD19264_Display_String(0,2,"PT2314 Fail");
		goto PT2314init;
	}
	LCD19264_Display_String(0,2,"PT2314 OK  ");
	LCD19264_Clear();


// 1.欢迎界面
	Draw_Clear();	
	Draw_Picture_logoRDS();                // 显示logo
	Draw_Rect(0,0,        191,63,  1);     // 屏幕大方框
	Draw_Line(0,32,       191,32,  1);     // 屏幕中间一横线
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
//			PT2314_Setup_CVD(3,GAIN_OFF);				// FM/DRS   3通道
//			QN8027_Init();
//			QNd8027_SetChannel((103000-76000)/50);
//			DebugMode();
        	KEY_Status_AllClear();// 清所有按键标志
            setsys_para=SET;
			os_dly_wait(200);
			Draw_Clear();
			break;
		}
	}
	PCRDS_Blinking_flag=OFF;
// 确定几个参数
	Power_set=20;
	//FM_value = 101700;
	PT2314_Setup_CVD(3,GAIN_OFF);				       // 接通道收机   3通道
	QN8027_Init();
	SYS_PARAMETER_Update();
	QNd8027_SetChannel((FM_value-76000)/50);
	PT2314_Setup_Volume(Volume_value);
	PT2314_Setup_Treble(Treble_value);
	PT2314_Setup_Bass(Bass_value);
	/*
	if(rds_state==0)//全关
      area_group_terminal_set_flag=5;
	if(rds_state==1)//全开
      area_group_terminal_set_flag=4;
	if(rds_state==2)//设置
      area_group_terminal_set_flag=0;
	 */
	//while(1);
// 2.创建几个任务
	os_tsk_create(RDS_task,1);                         // 建立小李的RDS任务
	//RDS_task_int();
	os_mut_init(LCD_mutex);                            // 初始化 LCD操作的互斥变量
//	tid_ComDebug = os_tsk_create(ComDebug_task,3);	   // 串口调试任务
//	tid_AutoFanControl = os_tsk_create(AutoFanControl_task,1);
	tid_AutoAdjustPower = os_tsk_create(AutoAdjustPower_task,1);
	tid_AutoChoiceAudio = os_tsk_create(AutoChoiceAudio_task,1);

	PTT_status = SET;// 这里暂时默认上电开启PTT
	TX_status=SET;

	tid_alarm_task = os_tsk_create(Alarm_task, 1);// 创建报警任务
	tid_PCRDS_task = os_tsk_create(PCRDS_Blinking_task, 1);// 创建任务
	tid_comdata_task = os_tsk_create(comdata_task, 1);// 创建任务
// 2.进入主界面
main_window_entry:
    //PCRDS_Blinking_flag=ON;
	os_mut_wait(LCD_mutex,0xffff);
	Draw_Clear();	
	Draw_Rect(0,0,        191,63,  1);// 屏幕大方框
	Draw_Rect(0,0,         65,19,  1);// 左上角方框
	Draw_Line(0,19,       179,19,  1);// 屏幕中间一横线
	Draw_Line(0,37,       191,37,  1);// 屏幕中间一横线
	Draw_Rect(179,0,      191,19,  1);// 方框
	Draw_Line(179,9,     191,9,  1);// 线
	Show_RDS_logox();//	小RDS图标
	Show_FSRF_Logo();// 显示FS-RF图标
	Show_Picture(181,1,FH_RD8X8,8,8);
	Show_Picture(181,10,FH_PC8X8,8,8);


	Show_RDS(rds_state);// 显示RDS状态
	Show_TX(PTT_status);// 显示发射机状态
	(PTT_status==SET) ? TX_LED_ON():TX_LED_OFF();
	Show_Time(&SystemTime);// 显示时间
	Show_FM(FM_value);// 显示FM频率值
	Show_I(I_value);// 显示电流值
	Show_T(T_value);// 显示温度值
	Show_P(Power_value);// 显示功率值
	Show_RP(RP_value);// 显示保护功率值
	Show_SWR(SWR_value);// 显示SWR值

update_v1:
	Show_Volume(Volume_value);
	Show_PEVal(PE_value);
	Show_VOLB(Bass_value);// 显示低音值
	Show_VOLH(Treble_value);// 显示高音值
	Show_FSRF_Logo();// 显示FS-RF图标
	os_mut_release(LCD_mutex);

	while(1)
	{
tab1:
		if(SystemTime_SecUpdate_Flag == SET)
		{
			SystemTime_SecUpdate_Flag = RESET;
			UpdateSystemTime();
			Show_Time(&SystemTime);// 显示时间
		}
		if(CollectUpdate_Interval==0)
		{
			CollectUpdate_Interval = 10;// 间隔100ms

			CollectShowCurrent();  // 采集电流并显示
			CollectShowTempval();  // 采集温度并显示
			CollectShowPower();	   // 采集功率并显示
			CollectShowRPower();   // 采集反向功率并显示
			CollectVIN();   // 激励电压
			CollectShowSWR();   // 计算SWR
			sample_count++;
			if(sample_count>=10)
			 sample_count=10;

		}
	   if(sample_count>=10)		//采样10次以后
       { 
		 if(Excitation_V<=200||(PTT_EN==Bit_RESET))
		{
		   	//if(VinWarning_Flag!=SET)
			{
				Pin_PTT_CLOSE();
				Show_TX(PTT_status=RESET);// 显示发射机状态
				TX_LED_OFF();
				VIN_AND_EN=FALSE;
					
				DAC5615Out(0);// 
				powerval_bak = 0; //使下次有vin时功率再调整
				fmval_bak = 0;
				dav_save=0;
			    VinWarning_Flag=SET;
			}
		}
		else if(Excitation_V>200&&(PTT_EN==Bit_SET))//大于0.2V
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
			  Show_TX(PTT_status=SET);// 显示发射机状态
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
				dav_save=dav_save*0.707;//慢慢下降
			 	Power_set_save=Power_set/2;

			 }

 			if(T_value <= 45 && tp_adjust_p==SET) //
			{
                tp_adjust_p=RESET;
				DAC5615Out(0);// 
				powerval_bak = 0; //
				fmval_bak = 0;
				dav_save=dav_save;//从半值处慢慢上升
				Power_set_save=Power_set;
			 }

			if(T_value > TP_set) //高温报警
			{
				if(HighTempWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// 显示发射机状态
					
					DAC5615Out(0);//
					powerval_bak = 0; //重新调整
					fmval_bak = 0;
					dav_save=0;
										 					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// 发送一个启动信号给报警任务
					Draw_Fill(1,39,190,63,0);// 画一空白区域
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
					 Show_TX(PTT_status=SET);// 显示发射机状态

					TX_LED_ON();
					TX_status=SET;
					os_evt_set(ALARM_END_EVENT,tid_alarm_task);// 发送一个结束信号给报警任务
					Draw_Fill(1,39,190,63,0);// 画一空白区域
					HighTempWarning_Flag = RESET;
					KEY_Status_AllClear();
					goto update_v1;
				}
			}

 			if(I_value > (8*12)) //过流报警，最大电流8A,过流120%
			{
				if(HighCURRENTWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// 显示发射机状态
					
					DAC5615Out(0);//
					powerval_bak = 0; //重新调整
					fmval_bak = 0;
					dav_save=0;
					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// 发送一个启动信号给报警任务
					Draw_Fill(1,39,190,63,0);// 画一空白区域
					Show_Picture(88,40,Alarm_Pic16X16,16,13);
					Draw_String6X8(40,55,"HIGH CURRENT WARNING !");
					HighCURRENTWarning_Flag = SET;
				}
			}
			else  //不可恢复，需重启
			{
			   ;
			}
			//=======================
 			if(SWR_value > SWRP_set&& RP_value>5) //驻波比报警
			{
				if(HighSWRWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// 显示发射机状态
					
					DAC5615Out(0);//
					powerval_bak = 0; //重新调整
					fmval_bak = 0;
					dav_save=0;
					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// 发送一个启动信号给报警任务
					Draw_Fill(1,39,190,63,0);// 画一空白区域
					Show_Picture(88,40,Alarm_Pic16X16,16,13);
					Draw_String6X8(40,55,"HIGH VSWR WARNING !");
					HighSWRWarning_Flag = SET;
				}
			}
			else  //不可恢复，需重启
			{
			;
			}

 			if(Vn_mv > (Vp_mv*5/4) && abs(RP_value)>5) //过反射功率保护  ,反向功率大于正向功率的1/4
			{
				if(HighPRWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// 显示发射机状态
					
					DAC5615Out(0);//
					powerval_bak = 0; //重新调整
					fmval_bak = 0;
					dav_save=0;
					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// 发送一个启动信号给报警任务
					Draw_Fill(1,39,190,63,0);// 画一空白区域
					Show_Picture(88,40,Alarm_Pic16X16,16,13);
					Draw_String6X8(40,55,"HIGH PR WARNING !");
					HighPRWarning_Flag = SET;
				}
			}

 			if(Vp_mv < 2000&& I_value>(64)) //小功率大电流保护
			{
				if(HighLPWarning_Flag != SET)
				{
					Show_TX(PTT_status=RESET);// 显示发射机状态
					
					DAC5615Out(0);//
					powerval_bak = 0; //重新调整
					fmval_bak = 0;
					dav_save=0;
					
					TX_LED_OFF();
					TX_status=RESET;
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// 发送一个启动信号给报警任务
					Draw_Fill(1,39,190,63,0);// 画一空白区域
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


// 按键处理部分--------------------------------------------------------------




		if(KEY_DOWN_STATUS() == SET)
		{
		    KEY_DOWN_CLEAR() ;
			DOWN_LockFlag=SET;// 长按功能使能          在TIM3中断服务函数中
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
			UP_LockFlag=SET;// 长按功能使能         在TIM3中断服务函数中
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
 MENU键按下进入 系统参数修改界面
 如FM频道、功率保护值、系统声音、系统时间、温度保护值、SWR值等
*/

		if(KEY_MENU_STATUS() == SET ||setsys_para==SET)
		{
			uint8_t set_target=1;// 菜单选项号 1、2、3
			//setsys_para=RESET;
main_menu_entry:
			Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s 
			Draw_Fill(1,39,190,63,0);// 画一空白区域
			Draw_String6X8(20,39,"SET FM AND RF POWER");
			Draw_String6X8(20,47,"SET Audio AND Time");
			Draw_String6X8(20,55,"SET TEMP AND SWR");
			Draw_Ascii6X8(10,31+set_target*8,'>');
			os_dly_wait(10);
			while(KEY_MENU_STATUS() == SET);//等待松开
			while(1)
			{
				if(SystemTime_SecUpdate_Flag == SET)
				{
					SystemTime_SecUpdate_Flag = RESET;
					UpdateSystemTime();
					Show_Time(&SystemTime);// 显示时间
				}
				if(KEY_UP_STATUS() == SET)
				{
					Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s 
					Draw_Ascii6X8(10,31+set_target*8,' ');
					if(--set_target < 1)
					{
						set_target=3;
					}
					Draw_Ascii6X8(10,31+set_target*8,'>');
				}
				if(KEY_DOWN_STATUS() == SET)
				{
					Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s 
					Draw_Ascii6X8(10,31+set_target*8,' ');
					if(++set_target > 3)
					{
						set_target=1;
					}
					Draw_Ascii6X8(10,31+set_target*8,'>');
				}
				if(KEY_OK_STATUS() == SET)
				{
					Menu_Timeout_Count=1000;// 菜单 “喂狗”  10s 
					//os_dly_wait(10);
					switch (set_target)
					{
						case 1:
						  fm_rf_set=SET;
						  SET_FM_And_RF_Power();
						  os_dly_wait(20);
						  fm_rf_set=RESET;
						  tp_adjust_p=RESET;//如果修改了功率，频率，温度调整标志复位
						  break;
							
						case 2:SET_Audio_And_Time();os_dly_wait(20);break;
						
						case 3:SET_Temp_And_SWR();os_dly_wait(20);break;
						
						default: break;
					}

					KEY_Status_AllClear();// 清除全部按键按下标志

					if(Menu_Timeout_Count==0)// 先看看是不是超时的返回
					{
						break;
					}
					else
					{
						goto main_menu_entry;// 重新跳至菜单入口，光标选项重显
					}
				}
				if(Menu_Timeout_Count==0)
				{
					break;
				}
				if(KEY_RDS_STATUS() == SET)// RDS键按下为退出菜单的功能
				{
				 os_dly_wait(10);
				 //if(KEY_MENU_STATUS() == SET)
				  //{
				   while(KEY_RDS_STATUS() == SET);//等待松开
					break; 
				  //}
				 }

				if(KEY_MENU_STATUS() == SET)	//防止在这个设置界面下按MENU后超时退出又回到本界面
				{
		         KEY_MENU_STATUS();
				}
			}//while(1)
			setsys_para=RESET;
			goto main_window_entry;

		}
		//}
/**************************************************************************************
 RDS键按下进入 RDS参数修改界面
*/
		if(KEY_RDS_STATUS() == SET)
		{	
		    KEY_RDS_CLEAR();
		    //PS_Name_Update();
			//Area_Name_Update();
			Setup_RDS_Window();
		    KEY_Status_AllClear();// 清除全部按键按下标志
			goto main_window_entry;
		}

	}// 主循环	
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
		os_evt_wait_and(0x0001,0xffff);// 等待串口2中断程序给事件信号
		if(S2_receive_flag==SET)
		{
			S2_receive_flag = RESET;
			switch (Rxbuf[0])
			{
				case 0x00:
//					QND_WriteReg(Rxbuf[1], Rxbuf[2]);
					os_evt_set(ALARM_START_EVENT,tid_alarm_task);// 发送一个启动信号给报警任务
					break;
				case 0x01:
//					Txbyte = QND_ReadReg(Rxbuf[1]);
//					USART_SendData(USART1, Txbyte);// 发送
					os_evt_set(ALARM_END_EVENT,tid_alarm_task);// 发送一个结束信号给报警任务
					break;
				case 0x10:
					QN8027_WriteReg(Rxbuf[1], Rxbuf[2]);
					break;
				case 0x11:
					Txbyte = QN8027_ReadReg(Rxbuf[1]);
					USART_SendData(USART2, Txbyte);// 发送
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
		if(   ((powerval_bak != Power_set) || 		// 功率变化
		      (fmval_bak != FM_value)   )  &&	   // 频段变化
			   fm_rf_set==RESET	  &&//非设置状态
		       HighTempWarning_Flag==RESET	&&             
			   HighCURRENTWarning_Flag==RESET && 
			   HighSWRWarning_Flag==RESET && 
		       HighPRWarning_Flag == RESET  &&
		       HighLPWarning_Flag == RESET &&
			   VIN_AND_EN==OK &&
			   setsys_para==RESET //退出设置参数时再开始调整         
						  )		
		{
			//ER_LED_ON();
			dav=0;
			adjct=0;
			DAC5615Out(dav_save);//如果从设置功率、频率中返回，继续上次的值
			if(tp_adjust_p==SET)
			  dav=dav_save;
			do
			{	if(tp_adjust_p==SET)
				 {
					if(Power_value > (Power_set_save))	//Power_set/2 温度达到65度时，调至一半
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
				if(++adjct>150)break;//超过15秒
				
			}while(abs(Power_set-Power_value)>2 && 
			       fm_rf_set==RESET  && //调整过程中，如果进入设置频率、功率时退出
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
			DAC5615Out(0);//确保有故障时输出0V
			}

			//ER_LED_OFF();
		}
		os_dly_wait(5);
	}

}


/*
音频优先级：接收机→DTU→AUX→MP3。//MP3最低
优先级高的切换到优先级低的，应该立即切换，
但优先级低的切换到优先级高的时，应延时5S再切换。
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

		if(status_save!=status_tmp)	//防抖延时
		{

		 if(!sound_ch_switch_Timeout_Count)
		 sound_ch_input_count++;

		 if(sound_ch_input_count>20) 
		  {
			  if(status_save<status_tmp)//从优先级低的切换到高,立即切换
			   {
			   	switch (status_tmp)
				{
					case 1:
					  PT2314_Setup_CVD(2,GAIN_OFF);	//MP3通道
						break;
					case 2:
					  PT2314_Setup_CVD(4,GAIN_OFF);	//AUX通道
						break;
					case 3:
					  PT2314_Setup_CVD(1,GAIN_OFF);	//DTU通道
						break;
					case 4:
					  PT2314_Setup_CVD(3,GAIN_OFF);	//中转通道
						break;
					default:
						break;
			      } 
				  status_save=status_tmp;
				  status_new=0;
				  sound_ch_input_count=0;
			     }
	
			  if(status_save>status_tmp)//从优先级高的切换到低,延时30S再切换
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
						  PT2314_Setup_CVD(2,GAIN_OFF);	//MP3通道
							break;
						case 2:
						  PT2314_Setup_CVD(4,GAIN_OFF);	//AUX通道
							break;
						case 3:
						  PT2314_Setup_CVD(1,GAIN_OFF);	//DTU通道
							break;
						case 4:
						  PT2314_Setup_CVD(3,GAIN_OFF);	//中转通道
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
void BEEP_task(void)// 蜂鸣器报警任务
{
	OS_RESULT result;

	for(;;)
	{
		BP_RING();// 蜂鸣器响
		os_dly_wait(10);
		BP_MUTE();// 静音
		os_dly_wait(10);
		BP_RING();// 蜂鸣器响
		os_dly_wait(10);
		BP_MUTE();// 静音
		result = os_evt_wait_and(0x0001,60);
		if(result == OS_R_EVT)
		{
			os_tsk_delete_self();
		}
	}
}
__task
void LED_Blinking_task(void)// LED报警闪烁任务
{
	OS_RESULT result;

	for(;;)
	{
		ER_LED_ON();// ERR灯亮
		os_dly_wait(10);// 亮100ms
		ER_LED_OFF();// ERR灯灭
		result = os_evt_wait_and(0x0001,10);// 灭100ms
		if(result == OS_R_EVT)
		{
			os_tsk_delete_self();
		}
	}
}
__task
void Alarm_task(void)
{
	OS_TID beeptask;   //存放蜂鸣器报警任务ID
	OS_TID ledtask;    //存放LED闪烁任务ID

	while(1)
	{
		os_evt_clr(ALARM_START_EVENT,tid_alarm_task);
		os_evt_wait_and(ALARM_START_EVENT,0xffff);       // 一直等待一个启动信号
		ledtask = os_tsk_create(LED_Blinking_task,10);   // 创建LED闪烁任务
		beeptask = os_tsk_create(BEEP_task,11);          // 创建蜂鸣器报警任务

		os_evt_clr(ALARM_END_EVENT,tid_alarm_task);
		os_evt_wait_and(ALARM_END_EVENT,0xffff);         // 一直等待一个结束信号
		os_evt_set(0x0001,beeptask);                     // 发送一个信号，结束蜂鸣器报警任务
		os_evt_set(0x0001,ledtask);                      // 发送一个信号，结束LED闪烁任务

		os_dly_wait(50);                                 // 延时500ms使 蜂鸣器与LED任务自己删除完毕
	}
}

__task
void PCRDS_Blinking_task(void)// PC或者RDS图标闪烁任务
{
	unsigned char const KB[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};// 电脑符号8X8.BMP
	unsigned char disonece=1;
	for(;;)
	{
		//os_evt_clr(PC_START_EVENT,tid_PCRDS_task);
		//os_evt_wait_and(PC_START_EVENT,0xffff);       // 一直等待一个启动信号
		if(menu_set==RESET)	//非设置菜单时
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
				os_dly_wait(50);// 亮500ms
		
				os_mut_wait(LCD_mutex,0xffff);
				Draw_UpdateToLCD();
				if(transfer_flag)
				  {
				    Show_Picture(181,1,KB,8,8);//KB--空白
				    Show_Picture(181,10,FH_PC8X8,8,8);
				   }
				else
				  {
				   Show_Picture(181,11,KB,8,8);// 灭
				   Show_Picture(181,1,FH_RD8X8,8,8);
				   }
				os_mut_release(LCD_mutex);
				os_dly_wait(50);// 灭500ms
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
void comdata_task(void)// PC或者RDS图标闪烁任务
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
			if(1)//USART_GetITStatus(USART1, USART_IT_TXE) != RESET 发送标志判断
			{   
				while(packet_tct<packet_sendsize)
				{
					USART_SendData(USART1, packet_tbuf[packet_tct++]);// 发送
					//while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
				    os_dly_wait(1);
				}
				//else
				{
					packet_tct = 0;
					packet_T_OK = SET;//发送完毕
				}
		
			}
		   }
		   */

		   }
	    if(packet_R_OK2)//中转
		  {
		   //packet_R_OK2=RESET;
		   //protocol(packet_rbuf2);
		   ;
		   }
	   os_mut_release(LCD_mutex);
	   os_dly_wait(1);// 灭500ms
	}
}



// 下面库调试用的，不用时可不理会

#ifdef  USE_FULL_ASSERT	         // 条件判断：是否检验参数的功能

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)// 出错提示函数
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

// 文件完 ----------------------------------------------------
