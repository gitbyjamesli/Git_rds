/*****************************************************************
 ��ͨ����Ƶ�����ʿ���оƬ��������
*/
#ifndef __PT2314_H__
#define __PT2314_H__


// ��Ȳ����ĺ궨��
#define GAIN_11dB25 1
#define GAIN_7dB5   2
#define GAIN_3dB75  3
#define GAIN_OFF    0


extern u8 pt2314_error_flag;
/* Ӧ�ò㺯�� */
u8 PT2314_Init(void);
u8 PT2314_Setup_Volume(u8 attenuation);               // ��������������  �ɹ�����0
u8 PT2314_Setup_CVD(u8 channel,u8 gain_volume);       // ͨ��/�������  �ɹ�����0
u8 PT2314_Setup_Bass(u8 Bass_dB);                     // �����������  �ɹ�����0
u8 PT2314_Setup_Treble(u8 Treble_dB);                 // �����������  �ɹ�����0








/* �ײ�������� */
u8 PT2314_SetReg(u8 dat);
u8 PT2314_SetNRegs(u8 *pRegVal,	u8 n);







#endif
//----------��β--------------------------------------------------
