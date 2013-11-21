#include "stm32f10x.h"
#include <stdio.h>
#include "all.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"		   // 0x40012400+4C
#define ADC1_DR_Address    ((uint32_t)0x4001244C)  //ADC1 DR寄存器基地址

USART_InitTypeDef USART_InitStructure;  //串口初始化结构体声明
ADC_InitTypeDef ADC_InitStructure;      //ADC初始化结构体声明
DMA_InitTypeDef DMA_InitStructure;      //DMA初始化结构体声明
// 注：ADC为12位模数转换器，只有ADCConvertedValue的低12位有效
__IO uint16_t ADCConvertedValue[5];
//static u16 ADCConvertedValueLocal, Precent = 0, Vp = 0,Vn = 0,VT = 0,Vin = 0,Vi = 0,num=0;

void ADC_GPIO_Configuration(void);


/*static void Delay_ARMJISHU(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}
*/
void adc_int(void)
{
   
  
  ADC_GPIO_Configuration();
  
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 
  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;    //DMA对应的外设基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCConvertedValue;   //内存存储基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//DMA的转换模式为SRC模式，由外设搬移到内存
  DMA_InitStructure.DMA_BufferSize = 5;		   //DMA缓存大小，1个,单位为DMA_MemoryDataSize
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//接收一次数据后，设备地址禁止后移
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//接收一次数据后，目标内存地址后移
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //定义外设数据宽度为16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA搬数据尺寸，HalfWord就是为16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //转换模式，循环缓存模式。
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//DMA优先级高
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		  //M2M模式禁用
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);          
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //独立的转换模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;		  //开启扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   //开启连续转换模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ADC外部开关，关闭状态
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //对齐方式,ADC为12位中，右对齐方式
  ADC_InitStructure.ADC_NbrOfChannel = 5;	 //开启通道数，1个
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel10 configuration ADC通道组， 第10个通道 采样顺序1，转换时间 */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_55Cycles5);
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);	  //ADC命令，使能
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);  //开启ADC1
  
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);	  //重新校准
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));  //等待重新校准完成
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);		//开始校准
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));	   //等待校准完成
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//连续转换开始，ADC通过DMA方式不断的更新RAM区。

}




void ADC_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;                       
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*
void adc_get_value(void)
{

    //ADCConvertedValueLocal = ADCConvertedValue[0];
     Vp = ADCConvertedValue[0];
     Vn = ADCConvertedValue[1];	
     VT = ADCConvertedValue[2];
    Vin = ADCConvertedValue[3];	
     Vi = ADCConvertedValue[4];				
	//Precent = (ADCConvertedValueLocal*100/4096);	//算出百分比
    //Voltage = Precent*33;						  // 3.3V的电平，计算等效电平
	
	switch (num)
	{
		case 0:
			  sprintf(packet_tbuf2, "V+:%5.0d", Vp);
			  packet_sendsize2 = 8;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// 使能发送中断  
			break;
		case 1:
			  sprintf(packet_tbuf2, "V-:%5.0d", Vn);
			  packet_sendsize2 = 8;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// 使能发送中
			break;
		case 2:
			  sprintf(packet_tbuf2, "VT:%5.0d", VT);
			  packet_sendsize2 = 8;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// 使能发送中
			break;
		case 3:
			  sprintf(packet_tbuf2, "Vin:%5.0d", Vin);
			  packet_sendsize2 = 10;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// 使能发送中
			break;
		case 4:
			  sprintf(packet_tbuf2, "VI:%5.0d", Vi);
			  packet_sendsize2 = 8;
			  packet_tct2 = 0;
			  packet_T_OK2 = RESET;
			  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);// 使能发送中
			break;
		default:
			break;
	}
   if(num<5)num++;
   else num=0;
   

}
 */