#ifndef __ALL_H__
#define __ALL_H__
#include "stm32f10x.h"

 #define OFF      0x0
#define ON       0x1

typedef struct  
{
 u8 Rnum;
 u8 Znum;
 u8 Dnum;
}Dveid_TypeDef;

typedef struct  
{
 u8 strbuf[2*16];
 u8 send_count;	//包序号
 u8 length;
 u8 package_length;
 u8 package_tmp;
}SendRDStxt_TypeDef;

typedef  struct STRUCT_PARAMETER
    {
    u32  ui_FM_value;     //4
	u8  ui_RF_POWER;      //2
	u8 	ui_Audio_VOL;      //1 
	u8 	ui_Audio_PE;       //1
  	u8 	ui_Audio_H;       //1
  	u8 	ui_Audio_B;  	   //1
  	u8 	ui_TEMP;          //1	
  	u8 	ui_SWR;           //1
  	u8 	ui_psnum;         //1  //保存ps名称号
  	u8 	ui_areanum;       //1  //保存区域号
  	u8 	ui_pe_value;	   //1
    //总共14个数据	       
    }STRUCT_PARAMETER_DATA , *PT_STRUCT_PARAMETER_DATA;

typedef enum {FALSE = 0, TRUE = !FALSE}MYSTATE;
 
//typedef 

//Dveid_TypeDef *p_ID;
extern Dveid_TypeDef Dveid,NEW_Dveid;
extern void FMToString(u32 val,char *strout);
extern void Show_FM(u32 val);
extern void Receivetxt_int(void);
extern void	QndRdsMode(u8	Mode);
extern void	QndMute(u8	Mode);
extern void RDSsend_IDNum(void);
extern void Dveid_Int(void);
extern void RDSsend_txt(void);
extern u8   packet_rbuf[64]; //接收的数据缓存
extern u8   packet_len;     //包长字节
extern u8   packet_rbuf2[32]; //接收的数据缓存
extern u8   packet_tbuf2[2]; //接收的数据缓存
extern uint8_t   packet_tct2;
extern FlagStatus            packet_T_OK2;
extern uint8_t               packet_sendsize2;    
extern u8   packet_len2;     //包长字节
extern void RDSsend_Volume(void);
extern void RDSsend_PS(void);
extern void ps_int(void);
extern u8 transfer_flag;
extern uint8_t pty_num;
extern uint8_t area_num;
//extern uint8_t area_fullsetval;
extern uint8_t *gb_area_num;
extern uint8_t *gb_area_setval;
extern uint8_t *gb_Group_num;
extern uint8_t *gb_Group_setval;
extern uint8_t *gb_terminal_num;
extern uint8_t *gb_terminal_setval;
extern  u8 area_group_terminal_set_flag;
extern PT_STRUCT_PARAMETER_DATA psys_data;//
extern uint32_t             FM_value;
void RDSsend_area_allset(u8 AGT_flag);
void adc_get_value(void);
extern void protocol(u8 *data);
extern void adc_int(void);
extern uint8_t               packet_sendsize;        //发送包大小
extern uint8_t               packet_tbuf[32];        //发送的数据缓存
extern FlagStatus            packet_T_OK;    //发送完成标志 0就绪 1发送中
extern uint8_t               packet_tct;
extern void txt_int(void);
extern uint8_t              Volume_value;
extern uint8_t             Treble_value;
extern uint8_t             Bass_value;
extern uint16_t            Power_set;
extern uint8_t            TP_set;
extern uint8_t            SWRP_set;
extern uint8_t RDS_DevicesGroup_Mode[24][16];
extern uint8_t PS_Name[10][8];
extern uint8_t Area_Name[20][7];
extern FlagStatus        packet_R_OK;    //接收完毕标志   SET有效
extern FlagStatus        packet_R_OK2;    //接收完毕标志   SET有效
extern __IO uint16_t ADCConvertedValue[];
extern u8 area_allset_flag;
extern   uint8_t PCRDS_Blinking_flag;
extern   uint16_t Disconnect_pc;
extern FlagStatus           setsys_para; 
extern FlagStatus           menu_set; 
extern FlagStatus           TX_status;
extern u8 Reg0_save;
extern u8 pc_area_group_terminal_set_flag;
extern MYSTATE pc_gbset;
#endif
