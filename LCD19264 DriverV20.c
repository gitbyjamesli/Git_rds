/*******************************************************************************
 �ļ�����LCD19264 DriverV20
 ��  ;��





*/


// ͷ�ļ�---------------------------------------------------------------------------------------
#include "stm32f10x.h"
#include "FONT8X16_ascii.h"    // ���������ֿ�
#include "FONT6X8_ascii.h"




// �궨��---------------------------------------------------------------------------------------

/* Һ������ CSL  ->  PC.9 */
#define LCD19264_CSL_PIN                  GPIO_Pin_9
#define LCD19264_CSL_GPIO_PORT            GPIOC
#define LCD19264_CSL_GPIO_CLK             RCC_APB2Periph_GPIOC
/* Һ������ CSM  ->  PA.8 */
#define LCD19264_CSM_PIN                  GPIO_Pin_8
#define LCD19264_CSM_GPIO_PORT            GPIOA
#define LCD19264_CSM_GPIO_CLK             RCC_APB2Periph_GPIOA
/* Һ������ CSR  ->  PA.11 */
#define LCD19264_CSR_PIN                  GPIO_Pin_11
#define LCD19264_CSR_GPIO_PORT            GPIOA
#define LCD19264_CSR_GPIO_CLK             RCC_APB2Periph_GPIOA
/* Һ������ DI  ->  PB.15 */
#define LCD19264_DI_PIN                  GPIO_Pin_15
#define LCD19264_DI_GPIO_PORT            GPIOB
#define LCD19264_DI_GPIO_CLK             RCC_APB2Periph_GPIOB
/* Һ������ EN  ->  PC.6 */
#define LCD19264_EN_PIN                  GPIO_Pin_6
#define LCD19264_EN_GPIO_PORT            GPIOC
#define LCD19264_EN_GPIO_CLK             RCC_APB2Periph_GPIOC
/* ��ת��оƬ���� HC164_CLK  ->  PC.7 */
#define HC164_CLK_PIN                  GPIO_Pin_7
#define HC164_CLK_GPIO_PORT            GPIOC
#define HC164_CLK_GPIO_CLK             RCC_APB2Periph_GPIOC
/* ��ת��оƬ���� HC164_DAT  ->  PC.8 */
#define HC164_DAT_PIN                  GPIO_Pin_8
#define HC164_DAT_GPIO_PORT            GPIOC
#define HC164_DAT_GPIO_CLK             RCC_APB2Periph_GPIOC
/* Һ�����������  ->  PA.12 */
#define LCD19264_BG_PIN                  GPIO_Pin_12
#define LCD19264_BG_GPIO_PORT            GPIOA
#define LCD19264_BG_GPIO_CLK             RCC_APB2Periph_GPIOA

#define CS1_LOW()        GPIO_ResetBits(LCD19264_CSL_GPIO_PORT, LCD19264_CSL_PIN)
#define CS1_HIGH()       GPIO_SetBits(LCD19264_CSL_GPIO_PORT, LCD19264_CSL_PIN)

#define CS2_LOW()        GPIO_ResetBits(LCD19264_CSM_GPIO_PORT, LCD19264_CSM_PIN)
#define CS2_HIGH()       GPIO_SetBits(LCD19264_CSM_GPIO_PORT, LCD19264_CSM_PIN)

#define CS3_LOW()        GPIO_ResetBits(LCD19264_CSR_GPIO_PORT, LCD19264_CSR_PIN)
#define CS3_HIGH()       GPIO_SetBits(LCD19264_CSR_GPIO_PORT, LCD19264_CSR_PIN)

#define DI_LOW()        GPIO_ResetBits(LCD19264_DI_GPIO_PORT, LCD19264_DI_PIN)
#define DI_HIGH()       GPIO_SetBits(LCD19264_DI_GPIO_PORT, LCD19264_DI_PIN)

#define EN_LOW()        GPIO_ResetBits(LCD19264_EN_GPIO_PORT, LCD19264_EN_PIN)
#define EN_HIGH()       GPIO_SetBits(LCD19264_EN_GPIO_PORT, LCD19264_EN_PIN)

#define HC164_CLK_LOW()        GPIO_ResetBits(HC164_CLK_GPIO_PORT, HC164_CLK_PIN)
#define HC164_CLK_HIGH()       GPIO_SetBits(HC164_CLK_GPIO_PORT, HC164_CLK_PIN)

#define HC164_DAT_LOW()        GPIO_ResetBits(HC164_DAT_GPIO_PORT, HC164_DAT_PIN)
#define HC164_DAT_HIGH()       GPIO_SetBits(HC164_DAT_GPIO_PORT, HC164_DAT_PIN)

#define LCD19264_BG_LOW()        GPIO_ResetBits(LCD19264_BG_GPIO_PORT, LCD19264_BG_PIN)
#define LCD19264_BG_HIGH()       GPIO_SetBits(LCD19264_BG_GPIO_PORT, LCD19264_BG_PIN)



/*****************����������********************/
static
void delay(void)
{
	unsigned int n;
	for(n=60;n>0;n--);
}
static
void delaylcd(unsigned int tmp)
{
	unsigned int n;
	for(n=tmp;n>0;n--);
}
static
void LCD_RST(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; // ���ò���


	/* ʹ������ʱ�� (APB2)*/
	RCC_APB2PeriphClockCmd(   RCC_APB2Periph_GPIOA
	                         |RCC_APB2Periph_GPIOB
	                         |RCC_APB2Periph_GPIOC
						   , ENABLE);

	// PC.6/7/8/9����Ϊ���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// PA.8/11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// PC.15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// PA.12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static
void SendByte(unsigned char byte)
{
	u8 n;
	for(n=0;n<8;n++)
	{
		if(byte&0x80)
		{
			HC164_DAT_HIGH();
		}
		else
		{
			HC164_DAT_LOW();
		}
		//delaylcd(10); //3.0
		HC164_CLK_HIGH();
		byte <<= 1;
		//delaylcd(10); //3.0
		HC164_CLK_LOW();
		//delaylcd(10); //3.0
	}
}

static
void LCD_WriteCMD(unsigned char byte)
{
//	DI_LOW();
//	EN_HIGH();//E = 1;
//    delaylcd(50);//3.0 
//	SendByte(byte);
//	EN_LOW();//E = 0;
//	delay();
	__disable_irq();
	DI_LOW();
	SendByte(byte);
	EN_HIGH();//E = 1;
	EN_LOW();//E = 0;
	__enable_irq();
	delay();

}
static
void LCD_WriteDAT(unsigned char byte)
{
    __disable_irq();
	DI_HIGH();
	SendByte(byte);
	EN_HIGH();//E = 1;
	EN_LOW();//E = 0;
	__enable_irq();
	delay();
}
static
void WriteCommandL(unsigned char CommandByte)//����ָ��
{   
    CS1_LOW();// ѡ 
	//delaylcd(120);//3.0  
	LCD_WriteCMD(CommandByte);
    CS1_HIGH();// ��ѡ 
	//delaylcd(50);//3.0   
}
static
void WriteCommandM(unsigned char CommandByte)//����ָ��
{   
    CS2_LOW();// ѡ 
	//delaylcd(120);//3.0
	LCD_WriteCMD(CommandByte);
    CS2_HIGH();// ��ѡ 
//	delaylcd(50);//3.0  
}
static
void WriteCommandR(unsigned char CommandByte)//����ָ��
{   
    CS3_LOW();// ѡ 
	//delaylcd(120);//3.0  
	LCD_WriteCMD(CommandByte);
    CS3_HIGH();// ��ѡ 
	//delaylcd(50);//3.0   
}



/************************************************************************************/

void LCD19264_BG_Open(void)
{
	LCD19264_BG_LOW();
}
void LCD19264_BG_Close(void)
{
	LCD19264_BG_HIGH();
}
void LCD19264_StartLine(unsigned char num)
{
	if(num<64)
	{
		CS1_HIGH();// ��ѡ   
	    CS2_HIGH();// ��ѡ   
	    CS3_HIGH();// ��ѡ   
		WriteCommandL(0xc0+num);
		WriteCommandM(0xc0+num);
		WriteCommandR(0xc0+num);
	}
}
void LCD19264_DisON(void)
{
    CS1_HIGH();// ��ѡ   
    CS2_HIGH();// ��ѡ   
    CS3_HIGH();// ��ѡ   
	WriteCommandL(0x3f);
	WriteCommandM(0x3f);
	WriteCommandR(0x3f);
}
void LCD19264_DisOFF(void)
{
    CS1_HIGH();// ��ѡ   
    CS2_HIGH();// ��ѡ   
    CS3_HIGH();// ��ѡ 
	  
	WriteCommandL(0x3e);
	WriteCommandM(0x3e);
	WriteCommandR(0x3e);
}
void LCD19264_Posbyte(unsigned char x,unsigned char y,unsigned char byte)
{
	if(y>7)return;
	switch (x/64)
	{													// L M R
		case 0:CS1_LOW(); CS2_HIGH();CS3_HIGH();  break;// 0 1 1  
		case 1:CS1_HIGH();CS2_LOW(); CS3_HIGH();  break;// 1 0 1
		case 2:CS1_HIGH();CS2_HIGH();CS3_LOW();   break;// 1 1 0
		default: return;
	}

	x %= 64;
	LCD_WriteCMD(0x40+x);// ������
	LCD_WriteCMD(0xB8+y);// ������y = 0~7
	//delaylcd(50);//3.0  ---
	LCD_WriteDAT(byte);// д������
}
void LCD19264_Clear(void)							
{
	unsigned char x,y;
	for(y=0;y<8;y++)
	{
		for(x=0;x<192;x++)
			LCD19264_Posbyte(x,y,0);
	}
}
void LCD19264_Init(void)
{
	LCD_RST();
	LCD19264_DisOFF();
	LCD19264_Clear();
	LCD19264_DisON();
	LCD19264_StartLine(0);// ������ʼ��Ϊ0	Ҳ��Ĭ�ϵ�
}
void LCD19264_DisChar(unsigned char x,unsigned char y,char asc)
{
	unsigned char i;
	unsigned char a = x;
	unsigned char const *cp;

	cp = &EZK[asc-0x20][0];
	for(i=0;i<8;i++)
	{
		LCD19264_Posbyte(a,y,*cp++);
		a++;
	}
	y++;
	a = x;// ������һ��
	for(i=0;i<8;i++)
	{
		LCD19264_Posbyte(a,y,*cp++);
		a++;
	}
}
void LCD19264_Display_String(unsigned char x,unsigned char y,char *str)
{
	while(*str)
	{
		LCD19264_DisChar(x,y,*str++);
		x+=8;
	}
}

void LCD19264_DisChar6X8(unsigned char x,unsigned char y,char asc)
{
	unsigned char i;
	unsigned char const *cp;

	cp = &FONT6x8[asc-0x20][0];
	for(i=0;i<6;i++)
	{
		LCD19264_Posbyte(x+i,y,*cp++);
	}
}
void LCD19264_Display_String6X8(unsigned char x,unsigned char y,char *str)
{
	while(*str)
	{
		LCD19264_DisChar6X8(x,y,*str++);
		x+=6;
	}
}


void LCD19264_DisChar_mask(unsigned char x,unsigned char y,char asc)
{
	unsigned char i;
	unsigned char a = x;
	unsigned char const *cp;

	cp = &EZK[asc-0x20][0];
	for(i=0;i<8;i++)
	{
		LCD19264_Posbyte(a,y,~(*cp++));
		a++;
	}
	y++;
	a = x;// ������һ��
	for(i=0;i<8;i++)
	{
		LCD19264_Posbyte(a,y,~(*cp++));
		a++;
	}
}

void LCD19264_Display_String_mask(unsigned char x,unsigned char y,char *str)
{
	while(*str)
	{
		LCD19264_DisChar_mask(x,y,*str++);
		x+=8;
	}
}

// �ļ���-------------------------------------------------------------------------
