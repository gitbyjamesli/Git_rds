/**********************************************************************
     Ϊ��ʹmain.c��д��һ�㣬�����˴����ҳ�ṹ�ķ�ʽ��ʹ���������д���׶������޸�
 ����ļ��ṩ��һЩ���������ʾ�������磺

�������µĲ�����ʾ�����У�
	void Show_RDS(u8 state);// ��ʾRDS״̬
	void Show_TX(u8 state);// ��ʾ����״̬
	void Show_Time(struct TimeTypeDef *timeval);// ��ʾʱ��
	void Show_FM(u32 val);// ��ʾFMƵ��ֵ
	void Show_I(u8 val);// ��ʾ����ֵ
	void Show_T(u8 val);// ��ʾ�¶�ֵ
	void Show_P(u16 val);// ��ʾ����ֵ
	void Show_RP(u8 val);// ��ʾ��������ֵ
	void Show_SWR(u8 val);// ��ʾSWRֵ
	void Show_VOLB(u8 val);// ��ʾ����ֵ   0~15
	void Show_VOLH(u8 val);// ��ʾ����ֵ   0~15
	void Show_Volume(u8 val);// ��ʾ������ֵ   0 ~ 32
	void Show_PEVal(u8 val);// ��ʾPEֵ  ����ֵ����  50 �� 75
	void Show_RDS_logox(void);// ��ʾСRDSͼ��
	
	void Show_Picture(u8 sx,u8 sy,u8 const *Pic,u8 size_x,u8 size_y);// ��ʾͼƬ������Pic����ָ�����ָ��   *�ض���ʽͼƬ


*/


#include "stm32f10x.h"
#include "LCD19264_DriverV20.h"
#include "DrawPicture.h"
//#include <tgmath.h>
#include <stdlib.h>
#include "my_datadef.h"

/***********************************************************************
ȡģ���: PCtoLCD2002������.exe
ͼƬλ�ã�"D:\My Documents\RDS_logo_x.BMP"
ȡģ��ʽ=����
ȡģ����=����
ͼƬ��С=64��19��
*/
unsigned char const RDS_logo_x[8*19]=
{
0xC0,0x03,0xC0,0x03,0x00,0x00,0x00,0x00,0xE0,0x0F,0xF0,0x07,0x00,0x00,0x00,0x00,0x38,0x38,0x18,0x1C,0x00,0x00,0x00,0x00,0x0C,0x60,0x06,0x30,0xF8,0xE0,0x01,0x0F,
0x86,0xC3,0xC3,0x61,0x08,0xE1,0x83,0x08,0xE3,0x8F,0xF1,0xC7,0x08,0x21,0x86,0x00,0xF1,0xDF,0xF8,0x8F,0x08,0x21,0x84,0x00,0x71,0x3C,0x3C,0x8E,0x88,0x20,0x84,0x01,
0x39,0x18,0x1F,0x9C,0x78,0x20,0x04,0x03,0x2F,0x88,0x1F,0xF4,0x18,0x20,0x04,0x06,0x47,0xC4,0x27,0xE2,0x28,0x2C,0x34,0x6C,0x8F,0xE3,0xC7,0xF1,0x48,0x2C,0x34,0x68,
0x1E,0xF0,0x0F,0xF0,0x88,0x2C,0x36,0x68,0x3E,0xF8,0x1F,0x7C,0x08,0xED,0xB3,0x68,0xFC,0x7F,0xFE,0x3F,0x08,0xED,0xB1,0x67,0xF8,0x3F,0xFC,0x1F,0x00,0x00,0x00,0x00,
0xE0,0x1F,0xF0,0x0F,0x00,0x00,0x00,0x00,0xC0,0x07,0xC0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
unsigned char const FSRF_Logo[]=
{
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x83,0x4F,0x00,0xF0,0x1F,0xFF,0x03,0x06,0xC3,0x70,0x00,0x60,0x30,0x06,0x03,0x06,0x64,0x40,0x00,0x60,0x60,0x06,0x04,
0x06,0x64,0x40,0x00,0x60,0x60,0x06,0x04,0x06,0x60,0x00,0x00,0x60,0x60,0x06,0x00,0x86,0xE0,0x00,0x00,0x60,0x60,0x86,0x00,0x86,0xC0,0x03,0x00,0x60,0x30,0x86,0x00,
0xFE,0x00,0x0F,0xFE,0xE7,0x0F,0xFE,0x00,0x86,0x00,0x3C,0x00,0x60,0x06,0x86,0x00,0x86,0x00,0x30,0x00,0x60,0x0C,0x86,0x00,0x06,0x00,0x60,0x00,0x60,0x0C,0x06,0x00,
0x06,0x20,0x60,0x00,0x60,0x18,0x06,0x00,0x06,0x20,0x60,0x00,0x60,0x18,0x06,0x00,0x06,0x60,0x60,0x00,0x60,0x30,0x06,0x00,0x06,0xE0,0x30,0x00,0x60,0x30,0x06,0x00,
0x0F,0x20,0x1F,0x00,0xF0,0xE0,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"F:\�ҵĵ���С��Ŀ\19-С��RDS��Ŀ\RDS����ģ��\��Ŀ����\RDS��ĿԴ�����V2.0\BMPȡģ�ļ�\fs-rf.BMP",0*/
};
unsigned char const FH_du8X8[8]=    {0x00,0x3A,0x44,0x04,0x04,0x44,0x38,0x00};// ��C
unsigned char const FH_duigou8X8[8]={0x80,0x40,0x40,0x20,0x13,0x14,0x08,0x00};// �Թ�
unsigned char const FH_chacha8X8[8]={0x82,0x44,0x28,0x10,0x28,0x44,0x82,0x00};// ���
unsigned char const FH_PC8X8[8]=    {0x00,0xFE,0x82,0x82,0x82,0xFE,0x10,0xFE};// ���Է���8X8.BMP
unsigned char const FH_RD8X8[8]=    {0x60,0x18,0x04,0xFE,0x82,0x82,0x82,0xFE};// ���ջ�����8X8.BMP
unsigned char const B_SPK8X8[8]=    {0x06,0x0E,0xFE,0xFE,0xFE,0x0E,0x06,0x00};/*"D:\My Documents\����ͼ\������_8X8.BMP",0*/
unsigned char const M_SPK8X8[8]= 	{0x04,0x0C,0x1C,0x7C,0x1C,0x0C,0x04,0x00};/*"D:\My Documents\����ͼ\������_8X8.BMP",0*/
unsigned char const S_SPK8X8[8]= 	{0x00,0x08,0x18,0x78,0x18,0x08,0x00,0x00};/*"D:\My Documents\����ͼ\С����_8X8.BMP",0*/
unsigned char const X_SPK8X8[8]= 	{0x82,0x44,0x28,0x10,0x28,0x44,0x82,0x00};/*"D:\My Documents\����ͼ\������_8X8.BMP",0*/
unsigned char const SW_8X8[8]  =    {0x10,0x10,0x54,0x92,0x92,0x54,0x38,0x00};/*"D:\My Documents\����ͼ\switch8X8.BMP",0*/

unsigned char const B_SPK[11] = {0x00,0x02,0x06,0x0A,0x72,0x42,0x72,0x0A,0x06,0x02,0x00};/*"D:\My Documents\����ͼ\������.BMP",0*/
unsigned char const M_SPK[11] = {0x00,0x00,0x04,0x0C,0x74,0x44,0x74,0x0C,0x04,0x00,0x00};/*"D:\My Documents\����ͼ\������.BMP",0*/
unsigned char const S_SPK[11] = {0x00,0x00,0x00,0x08,0x18,0x68,0x18,0x08,0x00,0x00,0x00};/*"D:\My Documents\����ͼ\С����.BMP",0*/
unsigned char const X_SPK[11] = {0x00,0x00,0x82,0x44,0x28,0x10,0x28,0x44,0x82,0x00,0x00};/*"D:\My Documents\����ͼ\������.BMP",0*/
unsigned char const Alarm_Pic16X16[] = {0x00,0x0F,0x80,0x88,0x40,0x48,0x20,0x28,0x1E,0x08,0x02,0x08,0x02,0xE8,0x02,0x08,0x1E,0x08,0x20,0x28,0x40,0x48,0x80,0x88,0x00,0x0F};/*������־*/
/**************************************************************************
 �˺���ΪһͼƬ��ʾ����
 �������˵����
 sx       ��ʾͼƬ��ʾλ�õ���ʼx����
 sy       ��ʾy����
 Pic      ��ʾͼƬ������Դ
 size_x   ��ʾͼƬ�����С
 size_y   ��ʾͼƬ�����С

 ע�⣺
 ͼƬ���ݵĸ�ʽ�����ǣ���  ȡģ��ʽ=����  ȡģ����=����   ��

*/
void Show_Picture(u8 sx,u8 sy,u8 const *Pic,u8 size_x,u8 size_y)// ��ʾͼƬ������Pic����ָ�����ָ��   *�ض���ʽͼƬ
{
	u8 i,h,n,tp;
	for(h=sy;h<(sy+size_y);h++)
	{
		for(i=sx;i<(sx+size_x);)
		{
			tp = *Pic++;
			for(n=0;n<8;n++)
			{
				if(tp&1)
				{
					Draw_Point(i++,h,1);
				}
				else
				{
					Draw_Point(i++,h,0);
				}
				tp >>= 1;
			}
				
		}
	}
}

// RDS��TX��FM�Ȳ���       ��Ϣ��ʼ��ð�ź��棩����λ�û����궨��
#define xp_RDS     90 
#define yp_RDS     2 

#define xp_TX     90 
#define yp_TX     11 

#define xp_TIME   112 
#define yp_TIME   11 

#define xp_FM     19 
#define yp_FM     21 

#define xp_I     96 
#define yp_I     21 

#define xp_T     160 
#define yp_T     21 

#define xp_P     19 
#define yp_P     21+8 

#define xp_RP     96 
#define yp_RP     21+8 

#define xp_SWR    160 
#define yp_SWR    21+8 

void Show_RDS(u8 state)// ��ʾRDS״̬
{
	char str[]="RDS:";
	Draw_String6X8( xp_RDS-4*6,  yp_RDS,     str);

	if(state==0) //��
	{
		Show_Picture(xp_RDS,yp_RDS,FH_chacha8X8,8,8);// ��ʾ���� ���
	
	}
	else if(state==1) //��
	{
		Show_Picture(xp_RDS,yp_RDS,FH_duigou8X8,8,8);// ��ʾ���� �Թ�

	}
	else if(state==2) //����
	{
		Draw_String6X8( xp_RDS-4*6,  yp_RDS,  "RDS:E");
	}
}
void Show_TX(u8 state)// ��ʾ����״̬
{
	char str[]="TX :";
	Draw_String6X8( xp_TX-4*6,  yp_TX,     str);
	if(state)
	{
		Show_Picture(xp_TX,yp_TX,FH_duigou8X8,8,8);// ��ʾ���� �Թ�
	}
	else
	{
		Show_Picture(xp_TX,yp_TX,FH_chacha8X8,8,8);// ��ʾ���� ���
	}
}
void Show_Time(struct TimeTypeDef *timeval)// ��ʾʱ��
{
	char str[]="00:00:00";

	Draw_String6X8( xp_TIME+16,    yp_TIME-9,    "Time");
	str[0] += timeval->hour/10;
	str[1] += timeval->hour%10;
	str[3] += timeval->mins/10;
	str[4] += timeval->mins%10;
	str[6] += timeval->sec/10;
	str[7] += timeval->sec%10;
	Draw_String6X8( xp_TIME,    yp_TIME,     str);
}

void Show_FM(u32 val)// ��ʾFMƵ��ֵ
{
	char str[]="FM:000.0MHz";

	str[3] += val%1000000/100000;
	str[4] += val%100000/10000;
	str[5] += val%10000/1000;
	str[7] += val%1000/100;
	Draw_String6X8( xp_FM-3*6,    yp_FM,     str);
}
void Show_I(u16 val)// ��ʾ����ֵ
{
	char str[]="I:00.0A";

//	str[2] += val/10;
//	str[3] += val%10;
//	Draw_String6X8(xp_I-2*6,yp_I,str);

	str[2] += val%1000/100;
	str[3] += val%100/10;
	str[5] += val%10;
	Draw_String6X8(xp_I-2*6,yp_I,str);
}
void Show_T(u8 val)// ��ʾ�¶�ֵ
{
	char str[]="T:00";
	str[2] += val/10;
	str[3] += val%10;
	Draw_String6X8(xp_T-2*6,yp_T,str);
	Show_Picture(xp_T+2*6,yp_T,FH_du8X8,8,8);// ��ʾ���� ��C
}
void Show_P(u16 val)// ��ʾ����ֵ
{
	char str[]="P:0000W";
	str[2] += val%10000/1000;
	str[3] += val%1000/100;
	str[4] += val%100/10;
	str[5] += val%10;
	Draw_String6X8(xp_P-2*6,yp_P,str);
}
void Show_RP(u8 val)// ��ʾ��������ֵ
{
	char str[]="RP:000W";
	str[3] += val%1000/100;
	str[4] += val%100/10;
	str[5] += val%10;
	Draw_String6X8(xp_RP-3*6,yp_RP,str);
}
void Show_SWR(u8 val)// ��ʾSWRֵ
{
	char str[]="SWR:0.0";
	str[4] += val/10;
	str[6] += val%10;
	Draw_String6X8(xp_SWR-4*6,yp_SWR,str);
}
void Show_VOLH(u8 val)// ��ʾ����ֵ   -7 ~ 7
{
	char str[]="H: 00dB";
	u8 len = val*3;
	u8 tp;
	if(val<7)
	{
		str[2]='-';
		tp = (7-val)*2;
	}
	else if(val==7)
	{
		str[2]='+';
		tp = (7-val)*2;
	}
	else 
	{
		str[2]='+';
		tp = (16-val)*2;//(val-7)*2;
	}

	str[3] += tp/10;
	str[4] += tp%10;
	Draw_String6X8(1,47,str);

	Draw_String6X8(43+5-2   ,47,"-");
	Draw_Rect(49+5,47,      49+45+5,47+6,1);
	Draw_Line(50+5,49,      50+len+5,49,1);Draw_Line(50+len+5,49,  50+42+5,49,0);
	Draw_Line(50+5,50,      50+len+5,50,1);Draw_Line(50+len+5,50,  50+42+5,50,0);
	Draw_Line(50+5,51,      50+len+5,51,1);Draw_Line(50+len+5,51,  50+42+5,51,0);
	Draw_String6X8(49+51,47,"+");
}
void Show_VOLB(u8 val)// ��ʾ����ֵ   -7 ~ 7
{
	char str[]="B: 00dB";
	u8 len = val*3;
	u8 tp;
	if(val<7)
	{
		str[2]='-';
		tp = (7-val)*2;
	}
	else if(val==7)
	{
		str[2]='+';
		tp = (7-val)*2;
	}
	else
	{
		str[2]='+';
		tp = (16-val)*2;//(val-7)*2;
	}
	str[3] += tp/10;
	str[4] += tp%10;
	Draw_String6X8(1,55,str);

	Draw_String6X8(43-2+5   ,55,"-");
	Draw_Rect(49+5,55,      49+45+5,55+6,1);
	Draw_Line(50+5,57,      50+len+5,57,1);Draw_Line(50+len+5,57,  50+42+5,57,0);
	Draw_Line(50+5,58,      50+len+5,58,1);Draw_Line(50+len+5,58,  50+42+5,58,0);
	Draw_Line(50+5,59,      50+len+5,59,1);Draw_Line(50+len+5,59,  50+42+5,59,0);
	Draw_String6X8(49+46+5,55,"+");
}
void Show_Volume(u8 val)// ��ʾ������ֵ   0 ~ 32
{
	char str[]="VOL:00";

	str[4] += val/10;
	str[5] += val%10;
	Draw_String6X8(4,39,str);
}
void Show_PEVal(u8 val)// ��ʾPEֵ  ����ֵ����  50 �� 75
{
    Draw_String6X8(56,39,"        ");
	if(val==50)
	{
		Draw_String6X8(56,39,"PE:50us");
	}
	else if(val==75)
	{
		Draw_String6X8(56,39,"PE:75us");
	}
	else if(val==100)
	{
		Draw_String6X8(56,39,"PE:100us");
	}
	else
      Draw_String6X8(56,39,"PE:50us");
}






void Show_RDS_logox(void)// ��ʾСRDSͼ��
{
	Show_Picture(1,1,RDS_logo_x,64,18);
}

void Show_FSRF_Logo(void)
{
	Show_Picture(120,39,FSRF_Logo,64,24);
}


//-------------�����ָ���-------------------------------------------------------------------
void SetingWindos_ShowFMchValue(uint32_t value)
{
	char strout[5];
	strout[0] = value%1000000/100000 + '0';
	strout[1] = value%100000/10000   + '0';
	strout[2] = value%10000/1000     + '0';
	strout[3] = '.';
	strout[4] = value%1000/100       + '0';
	Draw_Ascii6X8(20+6*3,47,strout[0]);
	Draw_Ascii6X8(20+6*4,47,strout[1]);
	Draw_Ascii6X8(20+6*5,47,strout[2]);
	Draw_Ascii6X8(20+6*6,47,'.');
	Draw_Ascii6X8(20+6*7,47,strout[4]);
}
void SetingWindos_DestroyFMchValue(void)
{
	Draw_Fill(20+6*3,47,20+6*8-1,54,0);// ���հ�
}
void SetingWindos_ShowPowerValue(uint32_t val)
{
	char strout[5];
	strout[0] = val%10000/1000 + '0';
	strout[1] = val%1000/100   + '0';
	strout[2] = val%100/10     + '0';
	strout[3] = val%10         + '0';
	Draw_Ascii6X8(20+6*9,55,strout[0]);
	Draw_Ascii6X8(20+6*10,55,strout[1]);
	Draw_Ascii6X8(20+6*11,55,strout[2]);
	Draw_Ascii6X8(20+6*12,55,strout[3]);
}
void SetingWindos_DestroyPowerValue(void)
{
	Draw_Fill(20+6*9,55,20+6*13-1,63,0);// ���հ�
}

// �ļ���-------------------------------------------------------------
