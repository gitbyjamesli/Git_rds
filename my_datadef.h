/***********************************************************************





*/

#ifndef __MY_DATADEF_H__
#define __MY_DATADEF_H__


// ���ݽṹ�������Զ���
struct DeviceID_TypeDef// RDS-ID�ű����Ľṹ����
{
	uint8_t Rnum;// ����
	uint8_t Znum;// ���
	uint8_t Dnum;// �ն˺�
};



struct TimeTypeDef// ϵͳʱ������Ľṹ����
{
	uint8_t hour;
	uint8_t mins;
	uint8_t sec;
};


#endif
// END
