/***********************************************************************





*/

#ifndef __MY_DATADEF_H__
#define __MY_DATADEF_H__


// 数据结构、类型自定义
struct DeviceID_TypeDef// RDS-ID号变量的结构定义
{
	uint8_t Rnum;// 区号
	uint8_t Znum;// 组号
	uint8_t Dnum;// 终端号
};



struct TimeTypeDef// 系统时间变量的结构定义
{
	uint8_t hour;
	uint8_t mins;
	uint8_t sec;
};


#endif
// END
