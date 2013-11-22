#include "stm32f10x.h"
#include "QN8027_Driver.h"				   // FM/RDS发射芯片驱动
#include <RTL.h>
#include <string.h>
#include <stdio.h>


#include "all.h"
#define	OFF	        0x00
#define	On	        0x01

#define	FALSE	    0x00
#define	OK	        0x01

#define	set_fm_ok	  0x03
#define	set_gl_ok	  0x05
#define	send_wb_ok	  0x07
#define	set_a_ok	  0x0F
#define	set_z_ok	  0x11
#define	set_zd_ok	  0x13

#define	send_pe_ok	  0x1F
#define	send_pty_ok	  0x21
#define	send_ps_ok	  0x23
#define	send_startwb_ok	  0x25
#define	send_stopwb_ok	  0x27
#define	send_area_ok	  0x29
#define	send_gbset_ok	  0x2B
#define	send_notgbset_ok  0x2D

OS_TID tid_RDSsend_txt_task;
OS_TID tid_RDSsend_IDNum_task;
OS_TID tid_RDSsend_Volume_task;
//OS_TID tid_RDSsend_PTY_task;//节目类型 （信息，新闻。。。等）
//OS_TID tid_RDSsend_PS_task;//节目名称(台标) 

OS_MUT QN8027_mutex;// QN8027操作的互斥量
__task void RDSsend_IDNum_task(void);
__task void RDSsend_txt_task(void);
__task void RDSsend_Volume_task(void);
//__task void RDSsend_PTY_task(void);
//__task void RDSsend_PS_task(void);


extern u8 EEPROM_ReadBytes(u16 addr,u8 *buf,u16 len);
extern u8 EEPROM_WriteBytes(u16 addr,u8 *buf,u16 len);
extern uint32_t sendstate_Time_Count;// 
extern  uint16_t             Power_value ;
extern  uint16_t             RP_value;
extern  uint8_t              PE_value;
extern  uint8_t              Bass_value;
extern  uint8_t              Treble_value;
extern  uint8_t              T_value;
extern  uint8_t              SWR_value;		   // 当前SWR
extern  uint8_t              I_value;		       // 当前电流
extern unsigned char const  Tiaoyin[] ;
u8 Volume_set=50;

Dveid_TypeDef Dveid,NEW_Dveid;
u8 receive_Flag=0;
u8 Send_receive=0;
SendRDStxt_TypeDef PTY=
{
{0}
};
SendRDStxt_TypeDef *pPTY=&PTY;
u8 *ptystr=&PTY.strbuf[0];

u8 strbuf[8],strbuf2[8],strbuf3[2*16],PTY_strbuf[10];
u8 strbuf5[2*8*2];
u8 *str=&strbuf3[0];

u8 send_count=0;	//包序号
u8 length,PTY_length;
u8 package_length,package_tmp,PTY_package_length,PTY_package_tmp;
u8 Byte_count=0,taltly_count=0;
u8 *string={"广州天河城"};


uint8_t *gb_area_num;
uint8_t *gb_area_setval;
uint8_t *gb_Group_num;
uint8_t *gb_Group_setval;
uint8_t *gb_terminal_num;
uint8_t *gb_terminal_setval;

uint8_t pc_area_num; 	//保存上位机发来的值
uint8_t pc_area_setval;
uint8_t pc_Group_num;
uint8_t pc_Group_setval;
uint8_t pc_terminal_num;
uint8_t pc_terminal_setval;

//uint8_t *gb_area_setval;

u8 area_group_terminal_set_flag=0;
u8 pc_area_group_terminal_set_flag=0;
u8 area_allset_flag=0; //设置状态
u8 transfer_flag=0;
char show_string[10];
u32 FM_val2=89000;
MYSTATE mybool;

MYSTATE sendwb_en=FALSE;
MYSTATE pc_gbset=FALSE;

u8 Reg0_save=0;
u8 count_state=0;
static uint8_t	state_tmp[375]; 

union
{
	u8 sz[2];
	u16 zs;
}convert;

union 
    {
      u8                     uchar_part[15];
	  STRUCT_PARAMETER_DATA  SYS_PARAMETER;
    }UNION_DATA;//*PT_UNION_DATA;

PT_STRUCT_PARAMETER_DATA psys_data;  
void Dveid_Int(void)
{
	Dveid.Rnum=1;
	Dveid.Znum=1;
	Dveid.Dnum=1;
} 

u8 RDSData_Verify2(u8 *buf)
{  
	u8 i=0,xor_out=0;

	for(i=0;i<7;i++) 
	{ 
		xor_out^=buf[i]; 
	} 
	return xor_out;
}

u8 ComData_Verify(u8 *buf,u8 len)
{  
	u8 i=0,xor_out=0;

	for(i=0;i<len;i++) 
	{ 
		xor_out^=buf[i]; 
	} 
	return xor_out;
}

void ps_int(void)
{
	//sscanf("NM6L8969","%s",pPTY->strbuf); // 
	//pPTY->length=sizeof("NM6L8969")-1;	//
	memcpy(pPTY->strbuf,&PS_Name[pty_num][0],8);
    pPTY->length=8;	
	pPTY->package_length=pPTY->length/4+( (pPTY->length%4) ? 1:0 );
	pPTY->package_tmp=pPTY->package_length;
	pPTY->package_length=(u8)pPTY->package_length<<4;
	pPTY->send_count=0;
}

void Receiveps_int(void)
{
    memset(pPTY->strbuf, 0, sizeof(pPTY->strbuf));
	if(packet_len>15)
	 packet_len=15;
	memcpy(pPTY->strbuf, packet_rbuf+5, packet_len-7);//
    pPTY->length=8;	
	pPTY->package_length=pPTY->length/4+( (pPTY->length%4) ? 1:0 );
	pPTY->package_tmp=pPTY->package_length;
	pPTY->package_length=(u8)pPTY->package_length<<4;
	pPTY->send_count=0;
}

void Receivetxt_int(void)
{	 
	memset(strbuf5, 0, sizeof(strbuf5));
	memcpy(strbuf5, packet_rbuf+5, packet_len-7); 
	length=packet_len-7;//
	
	packet_len=0;
	package_length=length/4+( (length%4) ? 1:0 );
	package_tmp=package_length;
	package_length=(u8)package_length<<4;
	Send_receive=1;
	//send_count=0;
} 
 
void Receive_areastate_int(void)
{	 
    //packet_rbuf+5
	uint8_t z,d;
	uint16_t k=0;

	count_state=packet_rbuf[6];//
	memcpy( state_tmp+count_state*25 , packet_rbuf+7, 25);//每次收到25个字节

    if(count_state==14)//接收结束
	 {
		for(z=0;z<24;z++)
		{
			for(d=0;d<15;d++)
			{
				RDS_DevicesGroup_Mode[z][d] = state_tmp[k++]; // 喇叭状态
			}
	
		}
	 }


} 

//static u8 tmp2=0;
void toggle(void)
{
   u8 tmp,tmp2=0;
	tmp=QN8027_ReadReg(0x0);//&0x04
	tmp2=tmp&0x04;

	if(tmp2)
		QN8027_WriteReg(0x00,tmp&0xfb);
		//QndSetReg(0x00,0x00,0x04);
	else
		QN8027_WriteReg(0x00,tmp|0x04);
		//QndSetReg(0x00,0x04,0x04);
 }

void turn(u8 flag)
{
   u8 tmp;
	tmp=QN8027_ReadReg(0x0);
	if(flag)
		QN8027_WriteReg(0x00,tmp|0x20); //打开发射
	else
		QN8027_WriteReg(0x00,tmp&0xdf);//关闭发射
 }

void txt_int(void)
{	
	sscanf("测试:123456ab","%s",strbuf3); // 
    length=sizeof("测试:123456ab")-1;	//
    package_length=length/4+( (length%4) ? 1:0 );
	package_tmp=package_length;
	package_length=(u8)package_length<<4;
}

void RDSsend_IDNum(void)
{	 
    u8 i,tmp;

	strbuf2[0]=0x49;//'I'
	strbuf2[1]=Dveid.Rnum;
	strbuf2[2]=Dveid.Znum;
	strbuf2[3]=Dveid.Dnum;
	strbuf2[4]=NEW_Dveid.Rnum;
	strbuf2[5]=NEW_Dveid.Znum;
	strbuf2[6]=NEW_Dveid.Dnum;
	strbuf2[7]=RDSData_Verify2(strbuf2);	  
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,strbuf2[i]); // *p++
	}

   toggle();
}

static 	u8 zuhao=0;
uint8_t	tmpbuff[4];
void RDSsend_state(void)
{	 
    u8 i,tmp;

	uint8_t z=0,d;
	uint8_t	stmp[33];
	 
   if(zuhao>10)	zuhao=0;
   //memset(tmpbuff, 0, sizeof(tmpbuff));
	tmpbuff[0]=0;
	tmpbuff[1]=0;
	tmpbuff[2]=0;
	tmpbuff[3]=0;

	for(d=0;d<16;d++)
	{
		 stmp[d+(z%2)*16]=RDS_DevicesGroup_Mode[zuhao][d] ; // 喇叭状态
		 //以下将一组（16个）终端状态值 组合到	tmpbuff[] 中  ,
		 //每个终端值（范围0~3）占2bit ，这样一次可以发射16个终端机的状态
		 if(d<=3)
		   tmpbuff[0]|=stmp[d+(z%2)*16]<<(6-d*2);
		 if(d>=4&&d<=7)
		   tmpbuff[1]|=stmp[d+(z%2)*16]<<(6-(d-4)*2);
		 if(d>=8&&d<=11)
		   tmpbuff[2]|=stmp[d+(z%2)*16]<<(6-(d-8)*2);
		 if(d>=12&&d<=15)
		   tmpbuff[3]|=stmp[d+(z%2)*16]<<(6-(d-12)*2);
	}


	strbuf2[0]='S';//状态
	strbuf2[1]=area_num; //区域号
	strbuf2[2]=zuhao;	 //组号号
	strbuf2[3]=tmpbuff[0];
	strbuf2[4]=tmpbuff[1];
	strbuf2[5]=tmpbuff[2];
	strbuf2[6]=tmpbuff[3];
	strbuf2[7]=RDSData_Verify2(strbuf2);
	
		  
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,strbuf2[i]); // *p++
	}

   toggle();

	if(1)//sendstate_Time_Count>100 3S
	{
	 sendstate_Time_Count=0;
 	 zuhao++;
	}
}

void RDSsend_NULL(void)
{	 
    u8 i,tmp;

	strbuf2[0]=0x00;//'I'
	strbuf2[1]=0x00;
	strbuf2[2]=0x00;
	strbuf2[3]=0x00;
	strbuf2[4]=0x00;
	strbuf2[5]=0x00;
	strbuf2[6]=0x00;
	strbuf2[7]=RDSData_Verify2(strbuf2);	  
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,strbuf2[i]); // *p++
	}
    toggle();
}

MYSTATE Check_QN8027_tx(void)
{
    static u8 CurValue,PreValue,tmp;
	tmp=QN8027_ReadReg(0x07);
	CurValue=tmp&0x08;	// 检查bit3 是否改变
	if(CurValue!=PreValue) // 接收到新的数据
	{
	 PreValue=CurValue;
	 return 1;	
	 }	  
	else return 0;
}

void RDSsend_area_allset(u8 AGT_flag)
{	 
    u8 i,tmp;

    if(Check_QN8027_tx()==FALSE)
	 return;

	switch (AGT_flag)
	{
		case 1:
			strbuf2[0]=0x61;//'a' //区域
			strbuf2[1]=0x73;//'s'
			strbuf2[2]=*gb_area_num;
			strbuf2[3]=*gb_area_setval;
			strbuf2[4]=0;
	        strbuf2[5]=0;
			break;
		case 2:
			strbuf2[0]=0x67;//'g'	//组
			strbuf2[1]=0x73;//'s'
			strbuf2[2]=*gb_area_num;
			strbuf2[3]=*gb_Group_num;
			strbuf2[4]=*gb_Group_setval;
	        strbuf2[5]=0;
			break;
		case 3:
			strbuf2[0]=0x74;//'t'  //单个终端
			strbuf2[1]=0x73;//'s'
			strbuf2[2]=*gb_area_num;
			strbuf2[3]=*gb_Group_num;
			strbuf2[4]=*gb_terminal_num;
	        strbuf2[5]=*gb_terminal_setval;
			break;
		case 4:	  //全开
			strbuf2[0]=0x61;//'a'
			strbuf2[1]=0x6f;//'o'
			strbuf2[2]=0;
			strbuf2[3]=0;
			strbuf2[4]=0;
	        strbuf2[5]=0;
			break;
		case 5:	   //全关
			strbuf2[0]=0x61;//'a'
			strbuf2[1]=0x63;//'c'
			strbuf2[2]=0;
			strbuf2[3]=0;
			strbuf2[4]=0;
	        strbuf2[5]=0;
			break;
		default:
			break;
	}

	strbuf2[6]=0;
	strbuf2[7]=RDSData_Verify2(strbuf2);	  
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,strbuf2[i]); // *p++
	}
    toggle();
}

void PCRDS_set(u8 AGT_flag)	  //上位机设置 RDS
{	 
    u8 i,tmp;

	switch (AGT_flag)
	{
		case 1:
			strbuf2[0]=0x61;//'a' //区域
			strbuf2[1]=0x73;//'s'
			strbuf2[2]=*gb_area_num;
			strbuf2[3]=*gb_area_setval;
			strbuf2[4]=0;
	        strbuf2[5]=0;
			break;
		case 2:
			strbuf2[0]=0x67;//'g'	//组
			strbuf2[1]=0x73;//'s'
			strbuf2[2]=pc_area_num;
			strbuf2[3]=pc_Group_num;
			strbuf2[4]=pc_Group_setval;
	        strbuf2[5]=0;
			break;
		case 3:
			strbuf2[0]=0x74;//'t'  //单个终端
			strbuf2[1]=0x73;//'s'
			strbuf2[2]=pc_area_num;
			strbuf2[3]=pc_Group_num;
			strbuf2[4]=pc_terminal_num;
	        strbuf2[5]=pc_terminal_setval;
			break;
		case 4:	  //全开
			strbuf2[0]=0x61;//'a'
			strbuf2[1]=0x6f;//'o'
			strbuf2[2]=0;
			strbuf2[3]=0;
			strbuf2[4]=0;
	        strbuf2[5]=0;
			break;
		case 5:	   //全关
			strbuf2[0]=0x61;//'a'
			strbuf2[1]=0x63;//'c'
			strbuf2[2]=0;
			strbuf2[3]=0;
			strbuf2[4]=0;
	        strbuf2[5]=0;
			break;
		default:
			break;
	}

	strbuf2[6]=0;
	strbuf2[7]=RDSData_Verify2(strbuf2);	  
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,strbuf2[i]); // *p++
	}
    toggle();
}



void RDSsend_pty(void)
{	 
	u8 i,tmp,strbuf[8];
	memset(strbuf, 0, sizeof(strbuf));
	strbuf[0]=0x70;//'p'
	strbuf[1]=pty_num;//节目代号
	strbuf[2]=0;
	strbuf[3]=0;
	strbuf[4]=0;
	strbuf[5]=0;
	strbuf[6]=0;	 
	strbuf[7]=RDSData_Verify2(strbuf);
	
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,strbuf[i]); // *p++
	}
	
    toggle();

}


void RDSsend_PS(void)
{	 
	u8 i,tmp,strbuf6[8];
	static u8 regtmp=0;
	memset(strbuf6, 0, sizeof(strbuf6));
	strbuf6[0]=0x74;//'t'
	strbuf6[1]=0X01;//信息编号
	strbuf6[2]=PTY.package_length+PTY.send_count;

	if(*str)strbuf6[3]=*ptystr++;
	if(*str)strbuf6[4]=*ptystr++;
	if(*str)strbuf6[5]=*ptystr++;
	if(*str)strbuf6[6]=*ptystr++;  
 
	strbuf6[7]=RDSData_Verify2(strbuf6);
	

	 //regtmp=Reg0_save+0x20;
	 //QN8027_WriteReg(0x00,regtmp);
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,strbuf6[i]); // *p++
		//QN8027_WriteReg(0x00,regtmp);
	}

	 //regtmp=regtmp|0x04;
	 //QN8027_WriteReg(0x00,regtmp);

    toggle();

	PTY.send_count++;	
	if(PTY.send_count==PTY.package_tmp)//2	package_length
	{
		ptystr=&PTY.strbuf[0];
		PTY.send_count=0;
	}
}
void RDSsend_Volume(void)
{	 
    u8 i,tmp;
	
	strbuf2[0]=0x79;
	strbuf2[1]=Dveid.Rnum;
	strbuf2[2]=Dveid.Znum;
	strbuf2[3]=Dveid.Dnum;
	strbuf2[4]=Volume_set;
	strbuf2[5]=0;
	strbuf2[6]=0;
    strbuf2[7]=RDSData_Verify2(strbuf2);
	for(i=0;i<8;i++)
    {
	 QN8027_WriteReg(0x08+i,strbuf2[i]); // *p++
     }

    toggle();
}


void RDSsend_txt(void)
{	 
	u8 i,tmp,strbuf4[8];

	memset(strbuf4, 0, sizeof(strbuf4));
	strbuf4[0]=0x77;
	strbuf4[1]=0X01;//信息编号
	strbuf4[2]=package_length+send_count;	//信息长度+当前发送的包号
	if(*str)strbuf4[3]=*str++;
	if(*str)strbuf4[4]=*str++;
	if(*str)strbuf4[5]=*str++;
	if(*str)strbuf4[6]=*str++;  
	strbuf4[7]=RDSData_Verify2(strbuf4);
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,strbuf4[i]); // *p++
	}

    toggle();

	send_count++;	
	if(send_count==package_tmp)//2	package_length
	{
		if(Send_receive)
			str=&strbuf5[0];
		else 
			str=&strbuf3[0];
		send_count=0;
	}
}

void Answer_handshake(void)
{ 
    u8 buff[9]={0xA2, 0x07, 0x00, 0xff, 0x01, 0x05, 0x00, 0x00, 0x00};
	u8 i;

	for(i=0;i<9;i++)
	  packet_tbuf[i]=buff[i];
	
	packet_sendsize =9;
	packet_tct = 0;
	packet_T_OK = RESET;
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);// 使能发送中断
	
 }

void send_pdata(void)
{ 
    u8 buff[22];
	u8 i,len=0;
	u16 fmtmp;
	if(!mybool)
	{
    mybool=1;
    buff[0]=0xA2;
    buff[1]=0x33;
    buff[2]=0x0; //PLL
    buff[3]=TX_status; //tx
    buff[4]=T_value;//rds temp
	fmtmp =FM_value/100;
    buff[5]=fmtmp/256;//fm-H //高8位	 //缩小100倍;
    buff[6]=fmtmp%256;
    buff[7]=Power_value/256;//Power_value; //fwd
    buff[8]=Power_value%256;
    buff[9]=I_value/256;//电流值
    buff[10]=I_value%256;
    buff[11]=SWR_value;	 //驻波比
    buff[12]=ComData_Verify(buff,12);
    len=13;
	}
	else
	{
    mybool=0;
    buff[0]=0xA2;
    buff[1]=0x33;
    buff[2]=RP_value/256; 
    buff[3]=RP_value%256; 
    buff[4]=Volume_value;
    buff[5]=PE_value;
    buff[6]=0x0; //Rin
    buff[7]=0x0; //Lin
    buff[8]=Treble_value; //H
    buff[9]=Bass_value; //低音英文bourdon
    buff[10]=TP_set;
    buff[11]=SWRP_set;
    buff[12]=Power_set/256;//Power_value; //fwd
    buff[13]=Power_set%256;
	buff[14]=ComData_Verify(buff,14);
	len=15;
	 }
	/*
    buff[0]=0xA2;
    buff[1]=22;
    buff[2]=0x0;
    buff[3]=0x01;
    buff[4]=0x01;
    buff[5]=0x33;
    buff[6]=0x0;

    buff[7]=0x0; //PLL
    buff[8]=0x0; //tx
    buff[9]=T_value; //rds temp
	fmtmp =FM_value/100;
    buff[10]=FM_value/256; //fm-H //高8位	 //缩小100倍
    buff[11]=FM_value%256; //fm-L//低8为
    buff[12]=Power_value; //fwd
    buff[13]=RP_value; //ref
    buff[14]=Volume_value; //stereo vol
    buff[15]=PE_value; //pe
    buff[16]=0x0; //Rin
    buff[17]=0x0; //Lin
    buff[18]=Treble_value; //H
    buff[19]=Bass_value; //低音英文bourdon
	buff[20]=ComData_Verify(buff,20); 
 	buff[21]=0xAA; 
	*/
	for(i=0;i<len;i++)
	  packet_tbuf[i]=buff[i];
	
	packet_sendsize =len;
	packet_tct = 0;
	packet_T_OK = RESET;
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);// 使能发送中断
	
 }


void answer(u8 type)
{ 
    u8 buff[6];
	u8 i,len=0;

    buff[0]=0xA2;
    buff[1]=type;
    buff[2]=0x0; //PLL
    buff[3]=0xAA; //tx
    buff[4]=0xAA; //tx
    len=5;

	for(i=0;i<len;i++)
	  packet_tbuf[i]=buff[i];
	packet_sendsize =len;
	packet_tct = 0;
	packet_T_OK = RESET;
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);// 使能发送中断
	
 }

void protocol(u8 *data)
{ 
 u8 tmp;
  tmp=*(data+3);

 switch (tmp)
 {
 	case 1:	   //设置频率
		convert.sz[1]=*(data+5); //高位
		convert.sz[0]=*(data+6); //低位

		FM_val2=(u32)convert.zs*100;// 
		if(FM_val2>108000)
		{
			FM_val2 = 108000;
		}
		if(FM_val2<76000)
		{
			FM_val2 = 76000;
		}
		//QNd8027_SetChannel((FM_val2-76000)/50);
		Show_FM(FM_val2);
		FM_value=FM_val2;
		QNd8027_SetChannel((FM_value-76000)/50);
		answer(set_fm_ok);	     
		break;
 	case 2:	   //设置功率

		Power_set=*(data+5);	 
		answer(set_gl_ok);		    
		break;
	case 0x04:
	    if(packet_R_OK2)
		{
		  transfer_flag=On;
		  }
		else if(packet_R_OK)
		  {
		  packet_R_OK=RESET;
		  Answer_handshake();
		  }
		break;
	case 6:	//接收到文本
		Receivetxt_int();
		answer(send_wb_ok);
		break;

	case 8:
		Dveid.Rnum=*(data+5);
	    Dveid.Znum=*(data+6);
	    Dveid.Dnum=*(data+7);
		NEW_Dveid.Rnum=*(data+8);
	    NEW_Dveid.Znum=*(data+9);
	    NEW_Dveid.Dnum=*(data+10);
		break;
	case 0X0A:
//		if(*(data+5)=='o'&& 
//		   *(data+6)=='p'&&	
//		   *(data+7)=='e'&&	
//		   *(data+8)=='n'	 
//		    )
			{
			 turn(On);    
			}
		break;

	case 0X0C:
//		 if(*(data+5)=='c'&& 
//		   *(data+6)=='l'&&	
//		   *(data+7)=='o'&&	
//		   *(data+8)=='s'&&	 
//		   *(data+9)=='e'
//		    )
			{
			//QN8027_WriteReg(0x00,0x02);
			//QndRdsMode(OFF);//On
			//QndMute(OFF);
			turn(OFF); 
			}
		break;
	case 0X0E: //区域统一设定
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  *gb_area_num=*(data+5);
			  *gb_area_setval=*(data+6);

			  RDSsend_area_allset(1);
			  answer(set_a_ok);		  
		  }
		break;

	case 0X10: //组统一设定
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  pc_area_num=*(data+5);
			  if(pc_area_num==area_num)//只接收区域号相同的数据
			  {
			  pc_Group_num=*(data+6);
              pc_Group_setval=*(data+7);
			  //PCRDS_set(2);
			  pc_area_group_terminal_set_flag=2;
			  answer(set_z_ok);
			  }  		  
		  }
		break;
	case 0X12: //单个终端设定
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  pc_area_num=*(data+5);
			  if(pc_area_num==area_num)//只接收区域号相同的数据
			  {
				  pc_Group_num=*(data+6);
				  pc_terminal_num=*(data+7);
				  pc_terminal_setval=*(data+8);
				  //PCRDS_set(3);
				  pc_area_group_terminal_set_flag=3;
				  answer(set_zd_ok);
			  }		  
		  }
		break;
	case 0X14: //全开
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  //PCRDS_set(4);	
			  pc_area_group_terminal_set_flag=4;	  
		  }
		break;	
	case 0X16: //全关
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  //PCRDS_set(5);	
			  pc_area_group_terminal_set_flag=5;	  
		  }
		break;	
	case 0x1E: //设置pe
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  PE_value=*(data+5);
			  Show_PEVal(PE_value);
			  answer(send_pe_ok);
		  }
		break;
	case 0x20: //设置节目类型
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  pty_num=*(data+5);
			  answer(send_pty_ok);
		  }
		break;
	case 0x22: //设置台标
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
              Receiveps_int();
			  answer(send_ps_ok);
		  }
		break;

	case 0x24: //开始发送文本
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
              sendwb_en=TRUE;
			  answer(send_startwb_ok);
		  }
		break;
	case 0x26: //停止发送文本
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
              sendwb_en=FALSE;
			  answer(send_stopwb_ok);
		  }
		break;

	case 0x28: //设置整个区的每个喇叭状态
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  //area_num=*(data+5);//区号
			  //count_state=*(data+6);//序号
			  Receive_areastate_int();
			  answer(send_area_ok);
		  }
		break;

	case 0x2A: //PC进入设置广播状态
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  answer(send_gbset_ok);
			  pc_area_group_terminal_set_flag=0;
			  pc_gbset=TRUE;
		  }
		break;

	case 0x2C: //PC离开设置广播状态
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
			  answer(send_notgbset_ok);
			  pc_area_group_terminal_set_flag=0;
			  pc_gbset=FALSE;
		  }
		break;

	case 0X32: //查询参数
         if(packet_R_OK)
		  {
			  packet_R_OK=RESET;
              send_pdata();	
			  Disconnect_pc=0;
			  PCRDS_Blinking_flag=ON;	  
		  }
		break;							
	default:
		break;
  }
 packet_R_OK=RESET;
 packet_R_OK2=RESET;
}

void SYS_PARAMETER_Update(void)
{
    u8 i=0;

	EEPROM_ReadBytes(0x0200,&UNION_DATA.uchar_part[0],15);

	FM_value=UNION_DATA.SYS_PARAMETER.ui_FM_value;
    if(FM_value<76000||FM_value>108000)
	   FM_value=88000;

	Volume_value=UNION_DATA.SYS_PARAMETER.ui_Audio_VOL;
    if(Volume_value>32)//不在范围内
	  Volume_value=32;

    Treble_value=UNION_DATA.SYS_PARAMETER.ui_Audio_H;
    if(Treble_value>15)//不在范围内
	  Treble_value=7;

    Bass_value=UNION_DATA.SYS_PARAMETER.ui_Audio_B;
    if(Bass_value>15)//不在范围内
	  Bass_value=7;

	Power_set=UNION_DATA.SYS_PARAMETER.ui_RF_POWER;
	if(Power_set>150)
	  Power_set=150;

    TP_set=UNION_DATA.SYS_PARAMETER.ui_TEMP;
	if(TP_set<60||TP_set>80)
	TP_set=60;
    //TP_set=99;		  
	SWRP_set=UNION_DATA.SYS_PARAMETER.ui_SWR;
	if(SWRP_set>30)
	  SWRP_set=30;
	 //SWRP_set=90;
    PE_value=UNION_DATA.SYS_PARAMETER.ui_pe_value;
	if(PE_value!=50||PE_value!=75)
	  PE_value=50;

	pty_num=UNION_DATA.SYS_PARAMETER.ui_psnum; //读PS号
    if(pty_num>9)pty_num=9;

	area_num=UNION_DATA.SYS_PARAMETER.ui_areanum; //读取区域号
	if(area_num>19)
	  area_num=19;

	rds_state=UNION_DATA.SYS_PARAMETER.ui_rds_state; //读取区域号
	if(rds_state>2)
	  rds_state=2;

}

void SYS_PARAMETER_Save(void)
{
	EEPROM_WriteBytes(0x0200,&UNION_DATA.uchar_part[0],15);
}

void PS_Name_Update(void)
{
	uint8_t z,d;
	uint8_t	tmp[33]; 

	for(z=0;z<10;z++)
	{
        if(!(z%2))
		 {
		 EEPROM_ReadBytes(32*(z/2),tmp,32);
		 os_dly_wait(5);
		  }
		for(d=0;d<8;d++)
		{
			PS_Name[z][d] = tmp[d+(z%2)*16]; //
		}

	}
}

void PS_Name_Save(void)
{
	uint8_t z,d;
	uint8_t	tmp[33]; 
	for(z=0;z<10;z++)
	{

		for(d=0;d<8;d++)
		{
			 tmp[d+(z%2)*16]=PS_Name[z][d] ; // 
		}
        if(z%2)
		 {
		 EEPROM_WriteBytes(32*(z/2),tmp,32);
		 os_dly_wait(5);
		  }

	}
}

void Area_Name_Update(void)
{
	uint8_t z,d;
	uint8_t	tmp[33]; 

	for(z=0;z<20;z++)
	{
        if(!(z%2))
		 {
		 EEPROM_ReadBytes(32*(z/2+6),tmp,32);
		 os_dly_wait(5);
		  }
		for(d=0;d<7;d++)
		{
			Area_Name[z][d] = tmp[d+(z%2)*16]; //
		}

	}
}

void Area_Name_Save(void)
{
	uint8_t z,d;
	uint8_t	tmp[33]; 
	for(z=0;z<20;z++)
	{

		for(d=0;d<7;d++)
		{
			 tmp[d+(z%2)*16]=Area_Name[z][d] ; // 
		}
        if(z%2)
		 {
		 EEPROM_WriteBytes(32*(z/2+6),tmp,32);
		 os_dly_wait(5);
		  }

	}
}

void RDS_DevicesGroup_Mode_Update(uint8_t num)
{
	uint8_t z,d;
	uint8_t	tmp[33]; 

	for(z=0;z<24;z++)
	{
        if(!(z%2))//每次读出32字节
		 {
		 EEPROM_ReadBytes(0x0200+32*(z/2+1)+(24/2)*32*num,tmp,32);
		 os_dly_wait(5);
		  }
		for(d=0;d<16;d++)
		{
			RDS_DevicesGroup_Mode[z][d] = tmp[d+(z%2)*16]; // 喇叭状态
		}

	}
}

void RDS_DevicesGroup_Mode_Save(uint8_t num)
{
	uint8_t z,d;
	uint8_t	tmp[33]; 
	for(z=0;z<24;z++)
	{

		for(d=0;d<16;d++)
		{
			 tmp[d+(z%2)*16]=RDS_DevicesGroup_Mode[z][d] ; // 喇叭状态
		}
        if(z%2)//每次存32字节
		 {
		 EEPROM_WriteBytes(0x0200+32*(z/2+1)+(24/2)*32*num,tmp,32);
		 os_dly_wait(5);
		  }

	}
}


__task
void RDS_task(void)	 
{
	Dveid_Int();
	txt_int();
	adc_int();
	//SYS_PARAMETER_Update();
	PS_Name_Update();
	Area_Name_Update();
	ps_int();
	//TP_set=99;
	RDS_DevicesGroup_Mode_Update(area_num);//读取各个终端号值，以便发射机初始化终端机


	psys_data=&UNION_DATA.SYS_PARAMETER;

	os_mut_init(QN8027_mutex); // 初始化 8027操作的互斥变量

    tid_RDSsend_IDNum_task = os_tsk_create(RDSsend_IDNum_task,1); 
    tid_RDSsend_Volume_task = os_tsk_create(RDSsend_Volume_task,1);
	tid_RDSsend_txt_task = os_tsk_create(RDSsend_txt_task,1);


	os_tsk_delete_self();
}
__task
void RDSsend_IDNum_task(void)
{
   u8 ledflash=0;
   sendstate_Time_Count=0;

	while(1)
	{
	    ledflash=~ledflash;
		//os_mut_wait(QN8027_mutex,0xffff);
		if(pc_gbset==FALSE)
		{
			switch (area_group_terminal_set_flag)
			{
				case 0:
				   if(Check_QN8027_tx()==OK) //可以发新数据
				   {
				    if(!area_allset_flag)
				     //RDSsend_IDNum();
					  RDSsend_state();
					else 
					  RDSsend_NULL();  //发送空信号，保持接收机在RDS模式
					}
					break;
				case 1:
				    RDSsend_area_allset(1);
					break;
				case 2:
				    RDSsend_area_allset(2);
					break;
				case 3:
				    RDSsend_area_allset(3);
					break;
				case 4:
				    RDSsend_area_allset(4);
					break;
				case 5:
				    RDSsend_area_allset(5);
					break;
				default:
					break;
			}
		}

   	 else if(pc_gbset==TRUE)
	 {
	   PCRDS_set(pc_area_group_terminal_set_flag);
	  }

//		if(ledflash)
//        GPIO_SetBits(GPIOB,GPIO_Pin_14);		   // ER_LED  亮操作
//		else
//        GPIO_ResetBits(GPIOB,GPIO_Pin_14);	   // ER_LED  灭操作

		//os_dly_wait(25); //     os_dly_wait(50)大约为1秒
	   os_dly_wait(2);
	}

}

__task
void RDSsend_txt_task(void)
{
    u8 sendwho=0;
	u8 ledflash=0;
	OS_RESULT wait_result;
	while(1)
	{
	    ledflash=~ledflash;
		//wait_result = os_mut_wait(QN8027_mutex,0xffff);//100
		//os_mut_wait(QN8027_mutex,100);//100
	    if(packet_R_OK)	 //pc
		  {
		   //packet_R_OK=RESET;
		   //protocol(packet_rbuf);
		   ;
		   }
	    if(packet_R_OK2)//中转
		  {
		   //packet_R_OK2=RESET;
		   //protocol(packet_rbuf2);
		   }
	   if(!area_allset_flag )//&& sendwb_en==TRUE
		{
		   if(Check_QN8027_tx()==OK)
		    RDSsend_txt();//发射文本信息
		}

//		if(ledflash)
//        GPIO_SetBits(GPIOB,GPIO_Pin_14);		   // ER_LED  亮操作
//		else
//        GPIO_ResetBits(GPIOB,GPIO_Pin_14);	   // ER_LED  灭操作

		os_dly_wait(2);
	}

}

__task
void RDSsend_Volume_task(void)
{
   u8 turn_flag=0;
   u8 ledflash=0;
	while(1)
	{
	    
		//os_mut_wait(QN8027_mutex,100);//0xffff
		ledflash=~ledflash;
		if(!area_allset_flag)
		{
		    turn_flag++;

//			if(turn_flag==1)
//			{
//			   if(Check_QN8027_tx()==OK)
//				//RDSsend_Volume();
//				;
//			}
			 if(turn_flag==1)
			{
			   if(Check_QN8027_tx()==OK)
				RDSsend_pty();
			}
			else if(turn_flag==2)
			{
			   if(Check_QN8027_tx()==OK)
			    RDSsend_PS();

//			if(ledflash)
//	        GPIO_SetBits(GPIOB,GPIO_Pin_14);		   // ER_LED  亮操作
//			else
//	        GPIO_ResetBits(GPIOB,GPIO_Pin_14);	   // ER_LED  灭操作
			}
			else 
			 turn_flag=0;

	        //os_dly_wait(200);
			os_dly_wait(2);
		}
		//os_mut_release(QN8027_mutex);
	}

}

