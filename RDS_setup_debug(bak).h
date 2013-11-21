u8 RDSData_Verify(u8 *buf,u8 len)
{  
	u8 i,xor_out=0;
	for(i=0;i<len;i++) 
	{ 
		xor_out^=buf[i]; 
	} 
	return xor_out;
}
void RDS_SendID(struct DeviceID_TypeDef *ID1,struct DeviceID_TypeDef *ID2)
{
	u8 buf[8];
	u8 i;
	buf[0] = 0x49;
	buf[1] = ID1->Rnum;
	buf[2] = ID1->Znum;
	buf[3] = ID1->Dnum;
	buf[4] = ID2->Rnum;
	buf[5] = ID2->Znum;
	buf[6] = ID2->Dnum;
	buf[7] = RDSData_Verify(buf,7);
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,buf[i]);
	}
	if(QN8027_ReadReg(0x0)&0x04)
	{
		QN8027_WriteReg(0x00,0x22);
	}
	else
	{
		QN8027_WriteReg(0x00,0x26);
	}
}
void RDS_SendVolume(struct DeviceID_TypeDef *ID1,u8 value)
{
	u8 buf[8];
	u8 i;
	buf[0] = 0x79;
	buf[1] = ID1->Rnum;
	buf[2] = ID1->Znum;
	buf[3] = ID1->Dnum;
	buf[4] = value;
	buf[5] = 0;
	buf[6] = 0;
	buf[7] = RDSData_Verify(buf,7);
	for(i=0;i<8;i++)
	{
		QN8027_WriteReg(0x08+i,buf[i]);
	}
	if(QN8027_ReadReg(0x0)&0x04)
	{
		QN8027_WriteReg(0x00,0x22);
	}
	else
	{
		QN8027_WriteReg(0x00,0x26);
	}
	
}
void DebugMode(void)
{
	u8 set_target=1;// 光标位置，初始在1
	u8 debug2_vol=0;
	struct DeviceID_TypeDef ID1 = {0,0,0};
	struct DeviceID_TypeDef ID2 = {0,0,0};
//	char str_info[10];

	Draw_Clear();
	LCD19264_Display_String(20,0,"--<Debug Mode>--");
	LCD19264_Display_String(0,2,"ID1:");
	LCD19264_Display_String(0,4,"Volume:    ID2:");	
	KEY_Status_AllClear();// 清所有按键标志
showip:
// 目标ID
	if(set_target==1)
	{// 反白显示
		LCD19264_DisChar_mask(8*4,2,ID1.Rnum/10+'0');
		LCD19264_DisChar_mask(8*5,2,ID1.Rnum%10+'0');
	}
	else
	{// 正常显示
		LCD19264_DisChar(8*4,2,ID1.Rnum/10+'0');
		LCD19264_DisChar(8*5,2,ID1.Rnum%10+'0');
	}
	LCD19264_DisChar(8*6,2,'-');
	if(set_target==2)
	{
		LCD19264_DisChar_mask(8*7,2,ID1.Znum/10+'0');
		LCD19264_DisChar_mask(8*8,2,ID1.Znum%10+'0');
	}
	else
	{
		LCD19264_DisChar(8*7,2,ID1.Znum/10+'0');
		LCD19264_DisChar(8*8,2,ID1.Znum%10+'0');
	}
	LCD19264_DisChar(8*9,2,'-');
	if(set_target==3)
	{
		LCD19264_DisChar_mask(8*10,2,ID1.Dnum/10+'0');
		LCD19264_DisChar_mask(8*11,2,ID1.Dnum%10+'0');
	}
	else
	{
		LCD19264_DisChar(8*10,2,ID1.Dnum/10+'0');
		LCD19264_DisChar(8*11,2,ID1.Dnum%10+'0');
	}
// 音量值
	if(set_target==4)
	{
		LCD19264_DisChar_mask(8*7,4,debug2_vol/10+'0');
		LCD19264_DisChar_mask(8*8,4,debug2_vol%10+'0');
	}
	else
	{
		LCD19264_DisChar(8*7,4,debug2_vol/10+'0');
		LCD19264_DisChar(8*8,4,debug2_vol%10+'0');
	}
// ID值
	if(set_target==5)
	{
		LCD19264_DisChar_mask(8*15,4,ID2.Rnum/10+'0');
		LCD19264_DisChar_mask(8*16,4,ID2.Rnum%10+'0');
	}
	else
	{
		LCD19264_DisChar(8*15,4,ID2.Rnum/10+'0');
		LCD19264_DisChar(8*16,4,ID2.Rnum%10+'0');
	}
	LCD19264_DisChar(8*17,4,'-');
	if(set_target==6)
	{
		LCD19264_DisChar_mask(8*18,4,ID2.Znum/10+'0');
		LCD19264_DisChar_mask(8*19,4,ID2.Znum%10+'0');
	}
	else
	{
		LCD19264_DisChar(8*18,4,ID2.Znum/10+'0');
		LCD19264_DisChar(8*19,4,ID2.Znum%10+'0');
	}
	LCD19264_DisChar(8*20,4,'-');
	if(set_target==7)
	{
		LCD19264_DisChar_mask(8*21,4,ID2.Dnum/10+'0');
		LCD19264_DisChar_mask(8*22,4,ID2.Dnum%10+'0');
	}
	else
	{
		LCD19264_DisChar(8*21,4,ID2.Dnum/10+'0');
		LCD19264_DisChar(8*22,4,ID2.Dnum%10+'0');
	}

//	str_info[0] = ID1.Rnum/10+'0';
//	str_info[1] = ID1.Rnum%10+'0';
//	str_info[2] = '-';
//	str_info[3] = ID1.Znum/10+'0';
//	str_info[4] = ID1.Znum%10+'0';
//	str_info[5] = '-';
//	str_info[6] = ID1.Dnum/10+'0';
//	str_info[7] = ID1.Dnum%10+'0';
//	str_info[8] = '\0';
//	LCD19264_Display_String(8*4,2,str_info);
//	str_info[0] = debug2_vol/10+'0';
//	str_info[1] = debug2_vol%10+'0';
//	str_info[2] = '\0';
//	LCD19264_Display_String(8*7,4,str_info);
//	str_info[0] = ID2.Rnum/10+'0';
//	str_info[1] = ID2.Rnum%10+'0';
//	str_info[2] = '-';
//	str_info[3] = ID2.Znum/10+'0';
//	str_info[4] = ID2.Znum%10+'0';
//	str_info[5] = '-';
//	str_info[6] = ID2.Dnum/10+'0';
//	str_info[7] = ID2.Dnum%10+'0';
//	str_info[8] = '\0';
//	LCD19264_Display_String(8*15,4,str_info);
	while(1)
	{
		if(KEY_UP_STATUS() == SET)
		{
			KEY_UP_CLEAR();
			switch (set_target)
			{
				case 1:ID1.Rnum++;break;
				case 2:ID1.Znum++;break;
				case 3:ID1.Dnum++;break;
				case 4:
					if(debug2_vol<63)
					{
						debug2_vol++;
					}
					break;
				case 5:ID2.Rnum++;break;
				case 6:ID2.Znum++;break;
				case 7:ID2.Dnum++;break;
				default:break;
			}
			goto showip;
		}
		if(KEY_DOWN_STATUS() == SET)
		{
			KEY_DOWN_CLEAR();
			switch (set_target)
			{
				case 1:ID1.Rnum--;break;
				case 2:ID1.Znum--;break;
				case 3:ID1.Dnum--;break;
				case 4:
					if(debug2_vol>0)
					{
						debug2_vol--;
					}
					break;
				case 5:ID2.Rnum--;break;
				case 6:ID2.Znum--;break;
				case 7:ID2.Dnum--;break;
				default:break;
			}
			goto showip;
		}
		if(KEY_LEFT_STATUS() == SET)
		{
			KEY_LEFT_CLEAR();
			if(--set_target<1)
			{
				set_target=7;
			}
			goto showip;
		}
		if(KEY_RIGHT_STATUS() == SET)
		{
			KEY_RIGHT_CLEAR();
			if(++set_target>7)
			{
				set_target=1;
			}
			goto showip;
		}
		if(KEY_OK_STATUS() == SET)
		{
			KEY_OK_CLEAR();
			if(set_target==4)
			{
				LCD19264_Display_String(0,6,"Send...");
				RDS_SendVolume(&ID1,debug2_vol);
				Delay_ms(5000);// 空延时，
				LCD19264_Display_String(0,6,"OK     ");
				Delay_ms(5000);// 空延时，
				LCD19264_Display_String(0,6,"  ");
			}
			else if(set_target>4)
			{
				LCD19264_Display_String(0,6,"Send...");
				RDS_SendID(&ID1,&ID2);//
				Delay_ms(5000);// 空延时，
				LCD19264_Display_String(0,6,"OK     ");
				Delay_ms(5000);// 空延时，
				LCD19264_Display_String(0,6,"  ");
			}
		}
		if(KEY_MENU_STATUS() == SET)
		{
			KEY_MENU_CLEAR();
			return;
		}
	}	


}

