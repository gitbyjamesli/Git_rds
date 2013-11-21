/**********************************************************************
*/
#include "my_datadef.h"

extern unsigned char const FH_du8X8[];// ��C
extern unsigned char const FH_duigou8X8[8];// �Թ�
extern unsigned char const FH_chacha8X8[8];// ���
extern unsigned char const FH_PC8X8[8];// ��������   ����
extern unsigned char const FH_RD8X8[8];// ��������   ���ջ�
extern unsigned char const B_SPK8X8[8];
extern unsigned char const M_SPK8X8[8];
extern unsigned char const S_SPK8X8[8];
extern unsigned char const X_SPK8X8[8];
extern unsigned char const SW_8X8[8];
extern unsigned char const B_SPK[11];
extern unsigned char const M_SPK[11];
extern unsigned char const S_SPK[11];
extern unsigned char const X_SPK[11];
extern unsigned char const Alarm_Pic16X16[]; 


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

void Show_Picture(u8 sx,u8 sy,u8 const *Pic,u8 size_x,u8 size_y);// ��ʾͼƬ������Pic����ָ�����ָ��   *�ض���ʽͼƬ
void Show_RDS_logox(void);// ��ʾСRDSͼ��
void Show_FSRF_Logo(void);// ��ʾFS-RFͼ��




void SetingWindos_ShowFMchValue(uint32_t FM_value);	    // ���ý���  FMƵ��ֵ��ʾ
void SetingWindos_DestroyFMchValue(void);				// ���ý���  FMƵ��ֵ����ʾ
void SetingWindos_ShowPowerValue(uint32_t val);			// ���ý���  Powerֵ��ʾ
void SetingWindos_DestroyPowerValue(void);				// ���ý���  Powerֵ����ʾ

