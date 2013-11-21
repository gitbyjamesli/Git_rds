/*************************************************************************************************
programming name: cx702a driver
building date:    2010-10-17
writer:          jiangqiao
versions:        v01
function:     FM  transmit
frequency:    76MHz~108MHz
*****************************************************************************************************/


#include "STC15.h"
/****************************define ******************************/

typedef unsigned char  UINT8;             //typedef char           INT8;              
typedef unsigned short UINT16;            //typedef short          INT16;




#define SYSTEM1		0x00
#define CH			0x01
#define CH_STEP		0x00
#define CH_CH      0x03
#define on  0x20
#define off 0x00

#define FREQ2CHREG(freq)   ((freq-7600)/5)



#define I2C_DEV0_ADDRESS     0x58
#define I2C_TIMEOUT_TIME    10
#define I2C_TIMEOUT_COUNT    8
#define MS_DELAY_CONST    40






/****************I/O control  pin defin***********************/
#define SDA    P3_2
#define SCL    P3_3


/***************function define*****************************/

extern void Msdelay(UINT16 dly);
//extern UINT8 ChipReset(UINT8 Slave) ;
extern void chip_init();


void cxF_SetCh(UINT16 freq);

extern void cxF_SetCh(UINT16 freq);

extern UINT8 cxD_ReadReg(UINT8 adr);
extern UINT8 cxD_WriteReg(UINT8 adr, UINT8 value);



/** the following functions is for other I2C devices rather than cx80xx ***/
extern UINT8 cxD_I2C_WRITE(UINT8 Regis_Addr,UINT8 Data);
extern UINT8 cxD_I2C_READ(UINT8 Regis_Addr);
//extern UINT8 cxD_I2C_NREAD(UINT8 Regis_Addr, UINT8 *buf, UINT8 n);
//extern UINT8 cxD_I2C_NWRITE(UINT8 Regis_Addr, UINT8 *buf, UINT8 n);

void cxD_TXSetPower( UINT8 gain);
void cx_txmode(UINT8 switch);




void init_io(void);
void key_scan(void);

/****************************driver function*************************/

void cxF_SetRegBit(UINT8 reg, UINT8 bitMask, UINT8 data_val) 
{
	UINT8 temp;
	temp = cxD_ReadReg(reg);
	temp &= (UINT8)(~bitMask);
	temp |= data_val;
	cxD_WriteReg(reg, temp);
}

void chip_init(void)
{
  //change the crystal setting

   cxF_SetRegBit(0x03,0xC0,0x00);	// Crystal source
   cxF_SetRegBit(0x04,0x80,0x00);	// Crystal:12MHz

  //..................................................
  //chip reset

  	cxF_SetRegBit(0x00,0x40,0x40);
	 Msdelay(200);				//delay 20 ms
	cxF_SetRegBit(0x00,0x40,0x20);
    Msdelay(200);
   // cxF_SetRegBit(0x00,0x20,0x20);

	//set( pre and de time)、(pa off time)、（TX pilot frequency deviation)

	cxD_WriteReg(0x02,0xb9);


	//set TX frequency deviation

    cxD_WriteReg(0x11,0x81);



	//***********************************************************
    cxD_WriteReg(0x18,0xe4);	// Improve SNR
	cxD_WriteReg(0x1b,0xf0);    // Increase PA max output power
    
    cxD_TXSetPower(0x00);       //change TX power
}

void cxF_SetCh(UINT16 freq)     //发射频率的函数
{
    // calculate ch para
    UINT8 tStep;
	UINT8 tCh;
    UINT16 f; 
		f = FREQ2CHREG(freq); 
		// set to reg: CH
		tCh = (UINT8) f;
		cxD_WriteReg(CH, tCh);
		// set to reg: CH_STEP
		tStep = cxD_ReadReg(CH_STEP);
		tStep &= ~CH_CH;
		tStep |= ((UINT8) (f >> 8) & CH_CH);
		cxD_WriteReg(CH_STEP, tStep);
 
}


void cxD_TXSetPower( UINT8 gain)
{
    UINT8 value = 0;
    value |= 0x40;  
    value |= gain;
    cxD_WriteReg(0x1f, value);
}

/**************************io port  driver functions************************/

//UINT8 cxd_i2c;
UINT8 cxd_i2c_timeout;
//UINT8 i2cMux = 0; //add this gloab temp to solve conflick of call timerRssicheck() and call 2-wire() at the same time

/*****************************************************************************************************
** Name:      Msdelay()
** Function:  time delay
*****************************************************************************************************/



void Msdelay(UINT16 dly)
{ 
    UINT8 i;
    for( ; dly>0; dly--)
    for(i=0; i<MS_DELAY_CONST; i++);
}


/*************************************************************************************************
** Name:      I2C_subfunction
** Function:  Software i2c code  
*****************************************************************************************************/
void Set_SCL(UINT8 i)      
{
    if(i) 
    {
        SCL=1;
    }
    else  
    { 
        SCL=0;
    }
}

void Set_SDA(UINT8 i)      
{
    if(i) {
             
              SDA=1;        // pin_in;
          } 
    else 
         {    
           
              SDA=0;       //   pin_out;
         }
}

void Start(void)               
{ 
    Msdelay(1);
    Set_SCL(1);
        Msdelay(1);
    Set_SDA(1);
    Msdelay(1);
    Set_SDA(0);
    Msdelay(2);
    Set_SCL(0);
}

void Stop(void)              
{
    Msdelay(1);
    Set_SDA(0);
    Msdelay(1);
    Set_SCL(1);
    Msdelay(2);
    Set_SDA(1);
}

void Send_ACK(UINT8 i)               //i=0 for ACK and i=1 for Nack
{
    Set_SDA(i);
    Msdelay(1);
    Set_SCL(1);
    Msdelay(1);
    Set_SCL(0);
    Msdelay(1);
    Set_SDA(1);
}
 
void Check_ACK(void)
{
   // pin_in;
    Msdelay(1);
    Set_SCL(1);
	cxd_i2c_timeout = (UINT8)SDA;
    Msdelay(1);
    Set_SCL(0);
  //  pin_out;
}

void Write_Bit(UINT8 i)
{
    Msdelay(1);
    Set_SDA(i);
    Msdelay(1);
    Set_SCL(1);
    Msdelay(2);
    Set_SCL(0);
}

void Write_Byte(UINT8 Data)
{
    UINT8 i;
    for(i=0;i<8;i++)
    {
        Write_Bit(Data>>7);
        Data<<=1;
    }
}

UINT8 Read_Byte(void)    //      
{
    UINT8 Data1=0x00;
    UINT8 j;
//    pin_in;               //设置口为输入
    for(j=0;j<8;j++)
    {
        Msdelay(1);
        Set_SCL(1);
        Data1 = (Data1<<1) | (SDA & 0x01);
        Msdelay(1);
            Set_SCL(0);
        }
//    pin_out;             //设置口为输出
    return Data1;
}
 
/************************************************************************************************************
** Name: I2C_Write_1byte                         
** Funcation:write a data to a desired            
**           register through i2c bus 
** Description: Slave---------device address
**              Regis_Addr----register address
*************************************************************************************************************/
UINT8 I2C_Write_1byte(UINT8 Slave,UINT8 Regis_Addr,UINT8 Data)
{
    UINT8 temp;
    temp=Slave;

    Start();
    Write_Byte(temp);
    Check_ACK();
    if(!cxd_i2c_timeout) {
        Write_Byte(Regis_Addr);
        Check_ACK();
        if(!cxd_i2c_timeout) {
            Write_Byte(Data);
            Check_ACK();
        }
    }
    Stop();
    return !cxd_i2c_timeout;
}
 
/***********************************************************************************************************
** Name: I2C_Read_1byte                          
** Function: Read a data from a desired register 
**           through i2c bus 
** Description: Slave---------device address
**              Regis_Addr----register address
************************************************************************************************************/
UINT8 I2C_Read_1byte(UINT8 Slave,UINT8 Regis_Addr)
{ 
    UINT8 Data=0x00;
    UINT8 temp;
    temp =Slave | 0x01;

    Start();
    Write_Byte(Slave);
    Check_ACK();
    if(!cxd_i2c_timeout) {
        Write_Byte(Regis_Addr);
        Check_ACK();
        if(!cxd_i2c_timeout) {
			//stop();
			Stop();
            Start();
            Write_Byte(temp);
            Check_ACK();
            if(!cxd_i2c_timeout) {
                Data = Read_Byte();
                Send_ACK(1);
            }
        }
    }
    Stop();
    return Data;
}

  

UINT8 cxD_WriteReg(UINT8 Regis_Addr,UINT8 Data)
{


	return cxD_I2C_WRITE(Regis_Addr,Data);

    return 1;
}

UINT8 cxD_ReadReg(UINT8 Regis_Addr)
{
    UINT8 Data;
    Data = cxD_I2C_READ(Regis_Addr);
    return Data;
}

UINT8 cxD_I2C_WRITE(UINT8 Regis_Addr,UINT8 Data)      
{
    UINT8 ret;
    UINT8 tryCount = I2C_TIMEOUT_COUNT;
    while(--tryCount) 
      {
        ret = I2C_Write_1byte(I2C_DEV0_ADDRESS, Regis_Addr, Data);
        if(ret) break;
      }
	
    return 1;
}

UINT8 cxD_I2C_READ(UINT8 Regis_Addr)      
{
    UINT8 ret;
    UINT8 tryCount = I2C_TIMEOUT_COUNT;
    while(--tryCount) 
      {
	ret = I2C_Read_1byte(I2C_DEV0_ADDRESS, Regis_Addr);
        if(!cxd_i2c_timeout) break;
      }
	return ret;
}

/***********************Transmit mode  or    IDLE mode******************************/
void cx_txmode(UINT8 switch)
{
  cxF_SetRegBit(0x00,0x20,switch);
}



/************************************mcu port init**********************************/

void init_io(void)
{
   P3M1=0;
   P3M0=0;
   CLK_DIV=CLK_DIV|0x07;
}


/*************************main*************************/









void main()
{
init_io();
chip_init();

 while(1)
{
if(P3_5==0)
cxF_SetCh(7680);   //发射频率是76.8MHz
else
cxF_SetCh(10790);  //发射频率是107.9MHz
  }
   
}






