/***********************************************************************

 19264液晶屏底层驱动程序，驱动函数声明文件

 驱动方式：串行/只读


 硬件接口：
	液晶引脚 CSL                  ->  PC.9 
	液晶引脚 CSM                  ->  PA.8 
	液晶引脚 CSR                  ->  PA.11 
	液晶引脚 DI                   ->  PB.15 
	液晶引脚 EN                   ->  PC.6 
	串转并芯片引脚 HC164_CLK      ->  PC.7 
	串转并芯片引脚 HC164_DAT      ->  PC.8


 建立时间：2012-11-23/文友
 版本：V20

 备注：
     坐标关系
    x,y(0,0)|___________________________________|x,y(191,0)
	        |___________________________________|
	        |___________________________________|
	        |___________________________________|
 	        |___________________________________|
 	        |___________________________________|
 	        |___________________________________|
    x,y(0,7)|___________________________________|x,y(191,7)

     
	 函数中的xy坐标取值范围
	 x       表示列     0~191
	 y       表示行     0~7
*/


void LCD19264_Init(void);       // 液晶屏初始化
void LCD19264_Posbyte(unsigned char x,unsigned char y,unsigned char byte);	 // 指定坐标处写一字节
void LCD19264_Clear(void);     // 清整屏							
void LCD19264_DisON(void);     // 打开显示
void LCD19264_DisOFF(void);    // 关闭显示

void LCD19264_BG_Open(void);
void LCD19264_BG_Close(void);
void LCD19264_DisChar(unsigned char x,unsigned char y,char asc);             //打印一字符
void LCD19264_Display_String(unsigned char x,unsigned char y,char *str);     //打印一字符串
void LCD19264_DisChar6X8(unsigned char x,unsigned char y,char asc);
void LCD19264_Display_String6X8(unsigned char x,unsigned char y,char *str);

void LCD19264_DisChar_mask(unsigned char x,unsigned char y,char asc);        // 打印一字符     反白显示
void LCD19264_Display_String_mask(unsigned char x,unsigned char y,char *str);// 打印一字符串   反白显示




// 文件完-------------------------------------------------------------------
