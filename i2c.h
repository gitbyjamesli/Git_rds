/************************************************************************

 模拟的I2C总线驱动函数：

 void WaitUs(void)         // 软件延时
 void I2CStart(void)       // 总线启动
 void I2CStop(void)		   // 总线停止
 u8 I2CChkAck(void)	 	   // 应答检查，无应答返回1，有应答返回0
 void I2CWrite8Bit(u8 byte)// 总线发送8个bit数据给从机
 void I2CSlave_ACK(void)   // 主机给    应答信号
 void I2CSlave_NOACK(void) // 主机给  非应答信号
 u8 I2CRead8Bit(void)

 说明：
    由外部定义好引脚
	SDA    模拟的数据线
	CLK      模拟的始终闲

 2012-11-2/文友
*/


// 变量定义---------------------------------------------------------









// 函数原型---------------------------------------------------------

static
void WaitUs(void)
{
	u32 i;
	for(i=60;i>0;i--);
}

static 
void I2CStart(void)
{
	 SDA_HIGH();	
     CLK_HIGH();
	 WaitUs();
	 SDA_LOW();	
	 WaitUs();
     CLK_LOW();
	 WaitUs();
}

static 
void I2CStop(void)
{
     SDA_LOW();
	 CLK_HIGH();
	 WaitUs();
	 SDA_HIGH();	
	 WaitUs();
}

static
u8 I2CChkAck(void)
{
	u8 errflag=1;
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(SDA_GPIO_PORT,&GPIO_InitStructure);// 启动设置为上拉输入

    CLK_HIGH();
	WaitUs();
	
	if(Bit_RESET==SDA_STATE())errflag = 0;

	CLK_LOW();
	WaitUs();
	GPIO_InitStructure.GPIO_Pin = SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SDA_GPIO_PORT,&GPIO_InitStructure);// 启动设置为输出

	return errflag;
}

static 
void I2CWrite8Bit(u8 byte)
{
	u8 temp;
	
	for(temp=8; temp>0; temp--)
	{
		if(byte & 0x80)	// 高位在前
		{
			SDA_HIGH();
		}
		else
		{
			SDA_LOW();
		}
		WaitUs();
		CLK_HIGH();
		WaitUs();
		byte <<= 1;
		CLK_LOW();
		WaitUs();
	}
}

static
void I2CSlave_ACK(void)
{
	SDA_LOW();

	CLK_HIGH();
	WaitUs();
	CLK_LOW();
	WaitUs();
}

static
void I2CSlave_NOACK(void)
{
	SDA_HIGH();

	CLK_HIGH();
	WaitUs();
	CLK_LOW();
	WaitUs();
}

static
u8 I2CRead8Bit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	u8 i;
	u8 byte=0;

	GPIO_InitStructure.GPIO_Pin = SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(SDA_GPIO_PORT,&GPIO_InitStructure);// 启动设置为上拉输入

	for(i=0;i<8;i++)
	{
	    byte<<=1;
		CLK_HIGH();
		WaitUs();
		if(Bit_SET==SDA_STATE())
		{
			byte|=0x01;
		}
	    CLK_LOW();
		WaitUs();
		
	}
	GPIO_InitStructure.GPIO_Pin = SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SDA_GPIO_PORT,&GPIO_InitStructure);// 启动设置为输出

	return (byte);
}

//-----------------------------------结尾----------------------------------------
