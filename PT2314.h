/*****************************************************************
 多通道音频，音质控制芯片驱动函数
*/
#ifndef __PT2314_H__
#define __PT2314_H__


// 响度参数的宏定义
#define GAIN_11dB25 1
#define GAIN_7dB5   2
#define GAIN_3dB75  3
#define GAIN_OFF    0


extern u8 pt2314_error_flag;
/* 应用层函数 */
u8 PT2314_Init(void);
u8 PT2314_Setup_Volume(u8 attenuation);               // 总音量控制设置  成功返回0
u8 PT2314_Setup_CVD(u8 channel,u8 gain_volume);       // 通道/响度设置  成功返回0
u8 PT2314_Setup_Bass(u8 Bass_dB);                     // 低音增益控制  成功返回0
u8 PT2314_Setup_Treble(u8 Treble_dB);                 // 高音增益控制  成功返回0








/* 底层操作函数 */
u8 PT2314_SetReg(u8 dat);
u8 PT2314_SetNRegs(u8 *pRegVal,	u8 n);







#endif
//----------结尾--------------------------------------------------
