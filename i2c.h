/************************************************************************

 ģ���I2C��������������

 void WaitUs(void)         // �����ʱ
 void I2CStart(void)       // ��������
 void I2CStop(void)		   // ����ֹͣ
 u8 I2CChkAck(void)	 	   // Ӧ���飬��Ӧ�𷵻�1����Ӧ�𷵻�0
 void I2CWrite8Bit(u8 byte)// ���߷���8��bit���ݸ��ӻ�
 void I2CSlave_ACK(void)   // ������    Ӧ���ź�
 void I2CSlave_NOACK(void) // ������  ��Ӧ���ź�
 u8 I2CRead8Bit(void)

 ˵����
    ���ⲿ���������
	SDA    ģ���������
	CLK      ģ���ʼ����

 2012-11-2/����
*/


// ��������---------------------------------------------------------









// ����ԭ��---------------------------------------------------------

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
	GPIO_Init(SDA_GPIO_PORT,&GPIO_InitStructure);// ��������Ϊ��������

    CLK_HIGH();
	WaitUs();
	
	if(Bit_RESET==SDA_STATE())errflag = 0;

	CLK_LOW();
	WaitUs();
	GPIO_InitStructure.GPIO_Pin = SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SDA_GPIO_PORT,&GPIO_InitStructure);// ��������Ϊ���

	return errflag;
}

static 
void I2CWrite8Bit(u8 byte)
{
	u8 temp;
	
	for(temp=8; temp>0; temp--)
	{
		if(byte & 0x80)	// ��λ��ǰ
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
	GPIO_Init(SDA_GPIO_PORT,&GPIO_InitStructure);// ��������Ϊ��������

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
	GPIO_Init(SDA_GPIO_PORT,&GPIO_InitStructure);// ��������Ϊ���

	return (byte);
}

//-----------------------------------��β----------------------------------------
