/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief
 *           Show a Master how to access Slave.
 *           This sample code needs to work with I2C_Slave.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M0564.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


//typedef   signed char  int8_t; 		// signed 8-bit number    (-128 to +127)
//typedef unsigned char  uint8_t; 	// unsigned 8-bit number  (+0 to +255)
//typedef   signed short int16_t; 	// signed 16-bt number    (-32768 to +32767)
//typedef unsigned short uint16_t; 	// unsigned 16-bit number (+0 to +65535)
//typedef   signed int   int32_t; 	// signed 32-bt number    (-2,147,483,648 to +2,147,483,647)
//typedef unsigned int   uint32_t; 	// unsigned 32-bit number (+0 to +4,294,967,295)

#define MXC4XX5XC_ADDRESS			0x15

#define MXC4XX5XC_REG_DATA			0x03
#define MXC4XX5XC_REG_CTRL			0x0D
#define MXC4XX5XC_REG_DEVICE_ID		0x0E


#define MXC4XX5XC_REG_INT_SRC0		0x00
#define MXC4XX5XC_REG_INT_SRC1		0x01
#define MXC4XX5XC_REG_INT_CLR0		0x00
#define MXC4XX5XC_REG_INT_CLR1		0x01
#define MXC4XX5XC_REG_INT_MASK0		0x0A
#define MXC4XX5XC_REG_INT_MASK1		0x0B
#define MXC4XX5XC_REG_DETECTION		0x0C

#define MXC4XX5XC_CMD_8G_POWER_ON	0x40
#define MXC4XX5XC_CMD_4G_POWER_ON	0x20
#define MXC4XX5XC_CMD_2G_POWER_ON	0x00
#define MXC4XX5XC_CMD_POWER_DOWN	0x01


#define MXC4XX5XC_DEVICE_ID_1		0x02
#define MXC4XX5XC_DEVICE_ID_2		0x03

#define MXC4XX5XC_2G_SENSITIVITY	1024
#define MXC4XX5XC_4G_SENSITIVITY	512
#define MXC4XX5XC_8G_SENSITIVITY	256
#define MXC4XX5XC_T_ZERO			25
#define MXC4XX5XC_T_SENSITIVITY		0.586

float *acc_out;
float *acc_out1;
float *acc_out2;
float *t_out;

uint8_t status=0;
 uint8_t condition=0;
 uint8_t recd;

 uint8_t count=0;

 int16_t data_acc[3] = {0};
double  data_temp = 0;

 uint8_t buffer_temperature[2]={0x09};
 uint8_t buffer_[2]={0x09};


 uint8_t sendcommand( uint8_t* );
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[6];
volatile uint8_t g_u8MstRxData[10];
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

  u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    CLK_EnableModuleClock(TMR0_MODULE);
          CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

    /* Set PD multi-function pins for I2C0 SDA and SCL */
    SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE12MFP_Msk | SYS_GPE_MFPH_PE13MFP_Msk);
    SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE12MFP_I2C0_SCL | SYS_GPE_MFPH_PE13MFP_I2C0_SDA);
}

void I2C0_Init(void)
{

	 /* Enable I2C0 module clock */
	    CLK_EnableModuleClock(I2C0_MODULE);

    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 400000);

    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */


    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
      //  I2C_START(I2C0);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
//        if(g_u8MstDataLen != 1)
//        {
//            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
//            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
//        }
//        else
//        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
//        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8MstRxData[count++] =  I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
      //  I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
//        if(g_u8MstDataLen != 1)
//        {
//            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
//            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
//        }
//        else
        {
           I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}


int32_t I2C0_Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;


    g_u8DeviceAddr = slvaddr;

        g_u8MstDataLen = 0;
       g_u8MstEndFlag = 0;


        /* I2C function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

//        /* Wait I2C Tx Finish */
        while(g_u8MstEndFlag == 0);
        g_u8MstEndFlag = 0;




        /* I2C function to read data from slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

        g_u8MstDataLen = 0;
        g_u8DeviceAddr = slvaddr;

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);
//}

        /* Wait I2C Rx Finish */
        while(g_u8MstEndFlag == 0);
//

    return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
  //  UART0_Init();

    /* Lock protected regist  ers */
    SYS_LockReg();


    /* Init I2C0 */
   // I2C0_Init();


 //   I2C_WriteByte(I2C0,0X0D,0X40);



     // recd = I2C_ReadByteOneReg(I2C0,0X15,0X09);
//
// //   I2C_WriteByteOneReg(I2C0,0X15,0X0D,0X00);

    while(1)
    {

    	 I2C0_Init();

    sendcommand(0x0F);

 //   TIMER_Delay(TIMER0, 10000);

    sendcommand(0x03);

//    TIMER_Delay(TIMER0, 10000);

    sendcommand(0x04);

 //   TIMER_Delay(TIMER0, 10000);

    sendcommand(0x05);

 //   TIMER_Delay(TIMER0, 10000);

    sendcommand(0x06);

 //   TIMER_Delay(TIMER0, 10000);

    sendcommand(0x07);

 //   TIMER_Delay(TIMER0, 10000);

    sendcommand(0x08);

 //   TIMER_Delay(TIMER0, 10000);

    sendcommand(0x09);

//    receivedata();

    /* Close I2C0 */
    I2C0_Close();

//    memset(g_u8MstRxData,0,10);
//    count=0;

       receivedata();
//    }
    }
}
void receivedata()

{

/* the output raw data unit is "count or LSB" */
data_acc[0]=(int16_t)(g_u8MstRxData[1]<<8|g_u8MstRxData[2])>>4;
data_acc[1]=(int16_t)(g_u8MstRxData[3]<<8|g_u8MstRxData[4])>>4;
data_acc[2]=(int16_t)(g_u8MstRxData[5]<<8|g_u8MstRxData[6])>>4;

data_temp = (int8_t)g_u8MstRxData[7];


/* convert to unit g */
acc_out = (data_acc[0] / MXC4XX5XC_8G_SENSITIVITY);
acc_out1 = data_acc[1] / MXC4XX5XC_8G_SENSITIVITY;
acc_out2 = data_acc[2] / MXC4XX5XC_8G_SENSITIVITY;

/* convert to unit is degree Celsius */
//*t_out = /*(float)data_temp*/MXC4XX5XC_T_SENSITIVITY + MXC4XX5XC_T_ZERO;

//memset(g_u8MstRxData,0,10);
    count=0;
    TIMER_Delay(TIMER0, 100000);
}

uint8_t sendcommand( uint8_t *data)
{
	g_au8MstTxData[0]=data;
//	g_au8MstTxData[1]=0X04;
//	g_au8MstTxData[2]=&data[2];
	I2C0_Read_Write_SLAVE(0x15);
}

/*********************************************************************************
* decription: data ready interrupt enable
*********************************************************************************/
void MXC4XX5XC_DataReady_INT_Enable(void)
{
	/* Write reg 0x0B, set DRDYE bit high, enable data ready interrupt */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_MASK1, 0x01);
}
/*********************************************************************************
* decription: data ready interrupt disable
*********************************************************************************/
void MXC4XX5XC_DataReady_INT_Disable(void)
{
	/* Write reg 0x0B, set DRDYE bit low, disable data ready interrupt */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_MASK1, 0x00);
}
/*********************************************************************************
* decription: XY axis shake interrupt enable
*********************************************************************************/
void MXC4XX5XC_XY_Shake_INT_Enable(void)
{
	/* Write reg 0x0A, set SHYME,SHYPE,SHXME,SHXPE bit high, enable XY shake interrupt */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_MASK0, 0x0F);

	/* Write reg 0x0C */
	/* Set SHM high, easy to triger the interrupt */
	/* set SHTH<2:0> 000-0.25g to 111-2.0g */
	/* Set SHC<1:0> 00-8 readings, 01-16 readings, 10-32 readings, 11-64 readings. */
	/* Set ORC<1:0>, 16,32,64,128 consecutive valid new orientation readings */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_DETECTION, 0x80);
}
/*********************************************************************************
* decription: XY axis shake interrupt disable
*********************************************************************************/
void MXC4XX5XC_XY_Shake_INT_Disable(void)
{
	/* Write reg 0x0A, set all bits low, disable XY shake interrupt */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_MASK0, 0x00);
}
/*********************************************************************************
* decription: XY axis orientation interrupt enable
*********************************************************************************/
void MXC4XX5XC_XY_Orientaion_INT_Enable(void)
{
	/* Write reg 0x0A, set ORXYE bit high, enable XY orientaion interrupt */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_MASK0, 0x40);

	/* Write reg 0x0C */
	/* Set ORC<1:0>, 16,32,64,128 consecutive valid new orientation readings */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_DETECTION, 0x00);
}
/*********************************************************************************
* decription: XY axis orientation interrupt disable
*********************************************************************************/
void MXC4XX5XC_XY_Orientaion_INT_Disable(void)
{
	/* Write reg 0x0A, set all bits low, disable XY orientaion interrupt */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_MASK0, 0x00);
}

/*********************************************************************************
* decription: Z axis orientation(tilt) interrupt enable
*********************************************************************************/
void MXC4XX5XC_Z_Orientaion_INT_Enable(void)
{
	/* Write reg 0x0A, set ORZE bit high, enable Z orientaion interrupt */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_MASK0, 0x80);

	/* Write reg 0x0C */
	/* Set ORC<1:0>, 16,32,64,128 consecutive valid new orientation readings */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_DETECTION, 0x00);
}
/*********************************************************************************
* decription: Z axis orientation(tilt) interrupt disable
*********************************************************************************/
void MXC4XX5XC_Z_Orientaion_INT_Disable(void)
{
	/* Write reg 0x0A, set all bits low, disable Z orientaion interrupt */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_MASK0, 0x00);
}

/*********************************************************************************
* decription: Clear data ready interrupt
*********************************************************************************/
void MXC4XX5XC_Clear_DataReady_INT(void)
{
	/* Write reg 0x01, clear the data ready interrupt flags */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_CLR1, 0x01);
}
/*********************************************************************************
* decription: Clear XY shake interrupt
*********************************************************************************/
void MXC4XX5XC_Clear_XY_Shake_INT(void)
{
	/* Write reg 0x00, clear all the shake interrupt flags */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_CLR0, 0x0F);
}
/*********************************************************************************
* decription: Clear XY orientation interrupt
*********************************************************************************/
void MXC4XX5XC_Clear_XY_Ori_INT(void)
{
	/* Write reg 0x00, clear XY orientation interrupt flags */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_CLR0, 0x40);
}
/*********************************************************************************
* decription: Clear Z orientation interrupt
*********************************************************************************/
void MXC4XX5XC_Clear_Z_Ori_INT(void)
{
	/* Write reg 0x00, clear Z orientation interrupt flags */
	I2C_Write_Reg(MXC4XX5XC_ADDRESS, MXC4XX5XC_REG_INT_CLR0, 0x80);
}

