/***********************************************************************

 19264Һ�����ײ����������������������ļ�

 ������ʽ������/ֻ��


 Ӳ���ӿڣ�
	Һ������ CSL                  ->  PC.9 
	Һ������ CSM                  ->  PA.8 
	Һ������ CSR                  ->  PA.11 
	Һ������ DI                   ->  PB.15 
	Һ������ EN                   ->  PC.6 
	��ת��оƬ���� HC164_CLK      ->  PC.7 
	��ת��оƬ���� HC164_DAT      ->  PC.8


 ����ʱ�䣺2012-11-23/����
 �汾��V20

 ��ע��
     �����ϵ
    x,y(0,0)|___________________________________|x,y(191,0)
	        |___________________________________|
	        |___________________________________|
	        |___________________________________|
 	        |___________________________________|
 	        |___________________________________|
 	        |___________________________________|
    x,y(0,7)|___________________________________|x,y(191,7)

     
	 �����е�xy����ȡֵ��Χ
	 x       ��ʾ��     0~191
	 y       ��ʾ��     0~7
*/


void LCD19264_Init(void);       // Һ������ʼ��
void LCD19264_Posbyte(unsigned char x,unsigned char y,unsigned char byte);	 // ָ�����괦дһ�ֽ�
void LCD19264_Clear(void);     // ������							
void LCD19264_DisON(void);     // ����ʾ
void LCD19264_DisOFF(void);    // �ر���ʾ

void LCD19264_BG_Open(void);
void LCD19264_BG_Close(void);
void LCD19264_DisChar(unsigned char x,unsigned char y,char asc);             //��ӡһ�ַ�
void LCD19264_Display_String(unsigned char x,unsigned char y,char *str);     //��ӡһ�ַ���
void LCD19264_DisChar6X8(unsigned char x,unsigned char y,char asc);
void LCD19264_Display_String6X8(unsigned char x,unsigned char y,char *str);

void LCD19264_DisChar_mask(unsigned char x,unsigned char y,char asc);        // ��ӡһ�ַ�     ������ʾ
void LCD19264_Display_String_mask(unsigned char x,unsigned char y,char *str);// ��ӡһ�ַ���   ������ʾ




// �ļ���-------------------------------------------------------------------
