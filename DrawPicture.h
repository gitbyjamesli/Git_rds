/************************************************************

 ��LCD19264Һ������ʵ�ֻ��㡢���ߡ������εĹ��ܺ���


 �������ڣ�2012-11-23/����

 �����ϵ��
	 (x,y)
	 0,0------------------------------191,0
	 |									|
	 |			����ϵ					|
	 |									|
	 |									|
	 0,64----------------------------191,64

 �汾��V10

*/


#ifndef __Draw_Picture_H__
#define __Draw_Picture_H__



//void Draw_Clear_DDRAM(void);// ��DDRAM
//void Draw_Point_DDRAM(u16 x,u16 y,u16 color);// DDRAM��ָ���������ã�1������0����
//u8   Draw_GetPointColor(u16 x,u16 y);// ��ȡDDRAM��ָ������ĵ�״̬������ 1������0����










/**********************************�ָ���***************************************/

//����------���㺯��-----------------------
void Draw_Point(u16 X,u16 Y,u16 point);

/*****************************************************************
 ��һ������
*/
void Draw_XLine(u16 x0,u16 y0,u16 x1,u16 color);
/*****************************************************************
 ��һ������
*/
void Draw_YLine(u16 x0,u16 y0,u16 y1,u16 color);

//����------��������仭�ߺ���-------------
void Draw_Line (u16 x0,//���x
				u16 y0,//���y
				u16 x1,//����x
				u16 y1,//����y
				u16 color);//������ɫ
//����------��Բ����  ----
void Draw_Circle(u16 x0,
				 u16 y0,
				 u16 r,
				 u16 color);
//����------�����κ���-------
void Draw_Rect( u16 xs,//���x
				u16 ys,//���y
				u16 xe,//����x
				u16 ye,//����y
				u16 color);//������ɫ
//����------��������亯��-------
void Draw_Fill( u16 xs,//���x
				u16 ys,//���y
				u16 xe,//����x
				u16 ye,//����y
				u16 color);//�����ɫ

void Draw_Clear(void);
void Draw_Ascii6X8(unsigned char x,unsigned char y,char asc);// ��6X8���ַ�
void Draw_Ascii6X8_Downline(unsigned char x,unsigned char y,char asc);// ��6X8���»����ַ���
void Draw_String6X8(unsigned char x,unsigned char y,char *str);// ��6X8���ַ���
void Draw_Picture_logoRDS(void);
void Draw_Lump_fb(uint8_t xs,uint8_t ys,uint8_t xe,uint8_t ye);// ָ�����С�ķ���


#endif

