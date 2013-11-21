/************************************************************

 在LCD19264液晶屏上实现画点、画线、画矩形的功能函数


 建立日期：2012-11-23/文友

 坐标关系：
	 (x,y)
	 0,0------------------------------191,0
	 |									|
	 |			坐标系					|
	 |									|
	 |									|
	 0,64----------------------------191,64

 版本：V10

*/


#ifndef __Draw_Picture_H__
#define __Draw_Picture_H__



//void Draw_Clear_DDRAM(void);// 清DDRAM
//void Draw_Point_DDRAM(u16 x,u16 y,u16 color);// DDRAM中指定坐标设置，1点亮，0点灭
//u8   Draw_GetPointColor(u16 x,u16 y);// 获取DDRAM中指定坐标的点状态，返回 1点亮，0点灭










/**********************************分割线***************************************/

//定义------画点函数-----------------------
void Draw_Point(u16 X,u16 Y,u16 point);

/*****************************************************************
 画一条横线
*/
void Draw_XLine(u16 x0,u16 y0,u16 x1,u16 color);
/*****************************************************************
 画一条竖线
*/
void Draw_YLine(u16 x0,u16 y0,u16 y1,u16 color);

//定义------任意两点间画线函数-------------
void Draw_Line (u16 x0,//起点x
				u16 y0,//起点y
				u16 x1,//结束x
				u16 y1,//结束y
				u16 color);//线条颜色
//定义------画圆函数  ----
void Draw_Circle(u16 x0,
				 u16 y0,
				 u16 r,
				 u16 color);
//定义------画矩形函数-------
void Draw_Rect( u16 xs,//起点x
				u16 ys,//起点y
				u16 xe,//结束x
				u16 ye,//结束y
				u16 color);//线条颜色
//定义------画矩形填充函数-------
void Draw_Fill( u16 xs,//起点x
				u16 ys,//起点y
				u16 xe,//结束x
				u16 ye,//结束y
				u16 color);//填充颜色

void Draw_Clear(void);
void Draw_Ascii6X8(unsigned char x,unsigned char y,char asc);// 画6X8的字符
void Draw_Ascii6X8_Downline(unsigned char x,unsigned char y,char asc);// 画6X8的下划线字符，
void Draw_String6X8(unsigned char x,unsigned char y,char *str);// 画6X8的字符串
void Draw_Picture_logoRDS(void);
void Draw_Lump_fb(uint8_t xs,uint8_t ys,uint8_t xe,uint8_t ye);// 指定块大小的反白


#endif

