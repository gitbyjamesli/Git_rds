/**********************************************************************
*/
#include "my_datadef.h"

extern unsigned char const FH_du8X8[];// °C
extern unsigned char const FH_duigou8X8[8];// 对勾
extern unsigned char const FH_chacha8X8[8];// 叉叉
extern unsigned char const FH_PC8X8[8];// 联机符号   电脑
extern unsigned char const FH_RD8X8[8];// 联机符号   接收机
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


void Show_RDS(u8 state);// 显示RDS状态
void Show_TX(u8 state);// 显示发射状态
void Show_Time(struct TimeTypeDef *timeval);// 显示时间
void Show_FM(u32 val);// 显示FM频率值
void Show_I(u8 val);// 显示电流值
void Show_T(u8 val);// 显示温度值
void Show_P(u16 val);// 显示功率值
void Show_RP(u8 val);// 显示保护功率值
void Show_SWR(u8 val);// 显示SWR值
void Show_VOLB(u8 val);// 显示低音值   0~15
void Show_VOLH(u8 val);// 显示高音值   0~15
void Show_Volume(u8 val);// 显示总音量值   0 ~ 32
void Show_PEVal(u8 val);// 显示PE值  输入值必须  50 与 75

void Show_Picture(u8 sx,u8 sy,u8 const *Pic,u8 size_x,u8 size_y);// 显示图片函数，Pic数据指针必须指向   *特定格式图片
void Show_RDS_logox(void);// 显示小RDS图标
void Show_FSRF_Logo(void);// 显示FS-RF图标




void SetingWindos_ShowFMchValue(uint32_t FM_value);	    // 设置界面  FM频段值显示
void SetingWindos_DestroyFMchValue(void);				// 设置界面  FM频段值不显示
void SetingWindos_ShowPowerValue(uint32_t val);			// 设置界面  Power值显示
void SetingWindos_DestroyPowerValue(void);				// 设置界面  Power值不显示

