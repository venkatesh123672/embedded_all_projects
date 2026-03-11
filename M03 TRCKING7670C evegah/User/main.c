/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "config.h"
#include "gpio.h"
#include "ADC.h"
#include <string.h>
#include "fmc.h"



#define PLL_CLOCK   FREQ_72MHZ



#define UNLOCK 0xA5
#define LOCK 0x5A

uint32_t vbn[10];
uint8_t count = 0;

uint8_t reset_mc=1;

uint8_t timeOut;

uint8_t loopCount;

uint8_t u32Prescale = 32;

/*---------------UART0------------------*/
uint8_t UART0_RX_BUF[BUF_LEN];
uint8_t UART0_TX_BUF[BUF_LEN];
uint8_t count_0;
uint8_t newWord_0 =0;


/*---------------UART1------------------*/
uint8_t UART1_RX_BUF[BUF_LEN];
uint8_t UART1_TX_BUF[BUF_LEN];
uint8_t count_1;
uint8_t newWord_1 =0;


/*---------------UART2------------------*/
uint8_t UART2_RX_BUF[BUF_LEN];
uint8_t UART2_TX_BUF[BUF_LEN];
uint8_t count_2;
uint8_t newWord_2 =0;

/*--------------------URL--------------------*/
 uint8_t APN_URL[URL_LENGTH];
uint8_t webURL[API_BUFF_LEN];
uint8_t bpon[BUF_LEN_SMALL];

uint8_t configMode = 0;
uint8_t readGNSS = 0;
uint8_t dataUpdateReady=0;
uint8_t gpsDebugData = 0;
uint8_t urlsize;

uint8_t swDevProd = 0;
uint8_t configReadBack = 1;

uint8_t readIMEI=0;
uint8_t IMEI[BUF_LEN_SMALL];
uint8_t testURL[BUF_LEN];
uint8_t testURl;
uint8_t testURLsize[5];
uint8_t devID[BUF_LEN_SMALL];

/***********************GPS DATA**************************/
uint8_t *gpsData;
uint8_t *endofStr;
uint8_t *rtc;
uint8_t *gpsStatus;
uint8_t	*latitude;
uint8_t lat_deg[2];
uint8_t lat_min[2];
uint8_t LAT_min[10];
uint8_t LAT_min_cov[10];
uint8_t lat_sec[5];
uint8_t *hem;
uint8_t	*longitude;
uint8_t long_deg[3];
uint8_t long_min[3];
uint8_t LONG_min[10];
uint8_t LONG_min_cov[10];
uint8_t long_sec[5];
uint8_t	*ew;
uint8_t	*Speed;
uint8_t	Sped[3];
uint8_t	*cog;
uint8_t	*date;
uint8_t *mv;
float speeed=1.852;
uint8_t knotspeed;
uint8_t fixed[4];
uint32_t knot;
uint32_t gpsSpeedDat;
uint32_t long_dege;
uint32_t lat_dege;
uint32_t gpsLatitudeD;
uint32_t gpsLongitudeD;

uint8_t timegpsh[3];
uint8_t timegpsm[3];
uint8_t timegpss[3];
uint8_t timegps[11];

uint8_t dategpsy[5];
uint8_t dategpsm[3];
uint8_t dategpsd[3];
uint8_t dategps[12];

uint8_t gpsLatitudeData[API_BUFF_LEN];
uint8_t gpsLongitudeData[API_BUFF_LEN];
uint8_t gpsAltitudeData[API_BUFF_LEN];
uint8_t gpsSpeedData[API_BUFF_LEN];
uint8_t gpsTimeData[API_BUFF_LEN];
uint8_t gpsDateData[API_BUFF_LEN];

uint8_t extBatt_Data[API_BUFF_LEN];
uint8_t intBatt_Data[API_BUFF_LEN];

uint8_t testURL[BUF_LEN];


uint16_t loopCounter;
uint32_t delayCounter = 200000;
uint32_t delayTempCount;

uint8_t *tGRMC;
uint8_t *tEndofRMC;
uint8_t GPS_data[100];
uint8_t newGpsData = 0;

uint8_t loopCount = 0;
uint8_t tloopCount=0;
uint8_t loopDelay=0;
uint8_t timeOut = 0 ;

/**************battery calculations***********************/

#define maxcharge100  13.5
#define mincharge20 10.5
uint32_t sum = 0;
uint8_t str[3];
float mess_battv;
float charge;
float differance;
float peakbatt;
float lowbatt;
uint8_t percentage;
uint8_t percentag;
uint8_t percent;
uint8_t battVExt[]="&ebv=";
uint8_t pbattVExt[]="&pebv=";
uint8_t battVInt[]="&ibv=";
uint16_t pextBatt_Data;
uint8_t BATTERY[BUF_LEN_SMALL];
uint32_t battery[10];

uint8_t batery;
uint8_t i;
uint8_t BEEPON;
uint8_t relay_satus=0;
uint8_t srver_test=0;

uint8_t uart_test=0;

float average;
float battADC_LC = 0.067;//0.023;//0.0222;//0.022;
float intBattV_conv;
float extBattV_conv;
float battint=0.0085;

extern uint32_t intBatt_V;
extern uint32_t extBatt_V;
extern uint32_t cmdReadBk[BUF_LEN_LONG/4];

/**************************beep on and off********************************/
#define BEEPER_ON  PA7=1
#define BEEPER_OFF  PA7=0
/***************************RELAY1 on and off*****************************/
#define RELAY1_ON  PA2=1
#define RELAY1_OFF  PA2=0
/***************************POWERRELAY on and off**************************/
#define POWERRELAY_ON PB6=1
#define POWERRELAY_OFF  PB6=0

#define  GSM_RESET_OFF  PA6=0
#define  GSM_RESET_ON   PA6=1

/************************web url********************************************/
uint8_t webURL[API_BUFF_LEN];
uint8_t api_update[API_BUFF_LEN] = "uDIM?";
uint8_t gpsLatitude[API_BUFF_LEN]="&lat=";
uint8_t gpsLongitude[API_BUFF_LEN]="&long=";
uint8_t gpsSpeed[API_BUFF_LEN]="&sp=";
/*****************************light_on_and_off*********************/
//#define LIGHT_ON
//#define LIGHT_OFF

/************************* special char ****************************/
char DBQUOTE[1] ={0x22};
/***************************DEVICE LOCKSTATUS***********************/
uint8_t deviceStatus;
uint8_t lockHist;
uint8_t LStat = 0;
uint8_t lockTest=0;
uint8_t devLock = 1; // config should set this to true for lock
uint8_t configMod;

uint8_t error_count=0;

//void UART2_TEST_HANDLE();
//void UART1_TEST_HANDLE();
void delay_loop();

void loop_delay(uint8_t loopDel);
void gsm_init();
void gsm_get_snapshot();
void gsm_datacall();
void gsm_connect();
void gsm_query();
void gsm_http_get();
void gsm_idle();
void gsm_http_report_data();
void gsm_http_send_registartion();
void get_gps_data();
void gsm_http_report_data();
void gsm_http_get_lock_status();

void gsm_http_send_lock_status();
void gsm_http_send_unlock_status();
void gsm_http_send_registartion();
void gsm_http_send_lightON_status();
void gsm_http_sending_lightOff_status();
void gsm_http_send_beepOn_status();
void gsm_http_send_beepOff_status();
void gsm_gsm_idle();
void gsm_beep_on();

void beep_delay();
void alram();

void power_rest_beep();
void wdt_rest_beep();

uint32_t cause;

int intmyatoi(char *);
void pextBat_Data(float );

void reverse(char* str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char* res, int afterpoint);

uint32_t UART_Write_main(UART_T* uart, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);

enum
{
	GSM_INIT=1,
	GSM_SNAPSHOT,
	GSM_DATACALL,
	GSM_CONNECT,
	GSM_QUERY,
	GSM_HTTP_REPORT_DATA,
	GSM_HTTP_GET_LOCK_STATUS,
	GSM_HTTP_SEND_LOCK,
	GSM_HTTP_SEND_UNLOCK,
	GSM_HTTP_SEND_LIGHTON,
	GSM_HTTP_SEND_LIGHTOFF,
	GSM_HTTP_SEND_BEEPOff,
	GSM_HTTP_SEND_BEEPON,
	GSM_HTTP_SEND_REG,
	GSM_BEEP_ON,
	GSM_UNLOCK,
	GSM_SLEEP,

}GSM_SM_T;

uint8_t gsmState = GSM_INIT;



extern uint32_t cmdReadBk[BUF_LEN_LONG/4];


int seconds;
uint32_t secLatU;
uint32_t secLatL;

int minutes;

uint8_t secLatUStr[2];
uint8_t secLatLStr[5];

uint8_t minLat[2];
uint8_t minLong[2];

int secondsLong;
uint32_t secLongU;
uint32_t secLongL;

uint32_t LAT_min_l;
uint32_t LONG_min_l;

uint8_t secLongUStr[2];
uint8_t secLongLStr[5];

void SYS_Init(void)
 {

	/* Unlock protected registers */
	SYS_UnlockReg();

//	  CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//
//	    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

	    /* Waiting for HIRC clock ready */
	//    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LIRCSTB_Msk);

	/* Enable HIRC clock (Internal RC 48MHz) */
	CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk);

	/* Wait for HIRC clock ready */
	while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

	while (!(CLK->STATUS & CLK_STATUS_LIRCSTB_Msk));

//	while (!(CLK->STATUS & CLK_STATUS_LXTSTB_Msk));

	/* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
	CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk))| CLK_CLKSEL0_HCLKSEL_HIRC;

	CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk))| CLK_CLKDIV0_HCLK(1);

	/* Enable UART module clock */
	CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk| CLK_APBCLK0_UART2CKEN_Msk);

	/* Update System Core Clock */
	SystemCoreClockUpdate();

	/********************************timer init**************************/
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

	/******************************** Init UART0 *************************/
	/* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
	SYS->GPB_MFPH = (SYS->GPB_MFPH& ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))|(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

	/******************************** Init UART1 *************************/

	/* Set PB multi-function pins for UART1 RXD(PB.2) and TXD(PB.3) */
	SYS->GPB_MFPL = (SYS->GPB_MFPL& ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk))|(SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

	/******************************** Init UART2 *************************/
	/* Set PB multi-function pins for UART2 RXD(PF.5) and TXD(PF.4) */
	SYS->GPF_MFPL = (SYS->GPF_MFPL& ~(SYS_GPF_MFPL_PF4MFP_Msk | SYS_GPF_MFPL_PF5MFP_Msk))|(SYS_GPF_MFPL_PF4MFP_UART2_TXD | SYS_GPF_MFPL_PF5MFP_UART2_RXD);

	/******************************ADC pinINTIALIZATION **********************/

	/* Enable ADC module clock */
	CLK_EnableModuleClock(ADC_MODULE);

	/*     /* ADC clock source is PCLK1, set divider to 1 */
	CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_HIRC, CLK_CLKDIV0_ADC(10));

	/* Set PB.0 to input mode */
	GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);

	/* Configure the PB.0 ADC analog input pins.  */
	SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk))| (SYS_GPB_MFPL_PB0MFP_ADC0_CH0);

	/* Disable the PB.0 digital input path to avoid the leakage current. */
	GPIO_DISABLE_DIGITAL_PATH(PB, BIT0);

	/* Configure the PB.1 ADC analog input pins.  */
	GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT);

	/* Configure the PB.1 ADC analog input pins.  */
	SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB1MFP_Msk))| (SYS_GPB_MFPL_PB1MFP_ADC0_CH1);

	/* Disable the PB.2 digital input path to avoid the leakage current. */
	GPIO_DISABLE_DIGITAL_PATH(PB, BIT1);
	/********************************************BUZZER**************************/
	GPIO_SetMode(PA, BIT7, GPIO_MODE_OUTPUT);

	SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA7MFP_Msk))| (SYS_GPA_MFPL_PA7MFP_GPIO);

	/********************************************RELAY_1**************************/
	GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);

	SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk))| (SYS_GPA_MFPL_PA2MFP_GPIO);

	/********************************************RELAY_2**************************/
	GPIO_SetMode(PB, BIT6, GPIO_MODE_OUTPUT);

	SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB6MFP_Msk))| (SYS_GPB_MFPL_PB6MFP_GPIO);

	/********************************************GSM_REST**************************/
		GPIO_SetMode(PA, BIT6, GPIO_MODE_OUTPUT);

		SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk))| (SYS_GPA_MFPL_PA6MFP_GPIO);


   /**************************Switch WDT clock source to LIRC *******************/

	 CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

   /**************************Enable WDT clock **********************************/

     CLK_EnableModuleClock(WDT_MODULE);

   /*******************wdt_time_setting*********************************************/

     WDT_Open(WDT_TIMEOUT_2POW18, WDT_RESET_DELAY_1026CLK, TRUE, FALSE);

	/************************** Lock protected registers ***********************/
     SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void) {
	/* Reset UART0 */
	SYS->IPRST1 |= SYS_IPRST1_UART0RST_Msk;
	SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

	/* Configure UART0 and set UART0 baud rate */
	UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 9600);
	UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init() {
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init UART 1                                                                                              */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Reset UART1 */
	SYS->IPRST1 |= SYS_IPRST1_UART1RST_Msk;
	SYS->IPRST1 &= ~SYS_IPRST1_UART1RST_Msk;

	/* Configure UART1 and set UART1 baud rate */
	UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 9600);
	UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}
void UART2_Init() {
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init UART 2                                                                                              */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Reset UART2 */
	SYS->IPRST1 |= SYS_IPRST1_UART2RST_Msk;
	SYS->IPRST1 &= ~SYS_IPRST1_UART2RST_Msk;

	/* Configure UART2 and set UART2 baud rate */
	UART2->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 9600);
	UART2->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}
void UART02_IRQHandler(void)
{
    UART02_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART2->INTSTS;

    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {

        /* Get all the input characters */
        while(UART_IS_RX_READY(UART2))
        {
          UART_Read(UART2, &UART2_RX_BUF[count_2], 1);

        if(UART2_RX_BUF[count_2] == '\r')
        	newWord_2 = 1;

        if((configMode == 1) && (count_2 > 2))
        {
        	if((UART2_RX_BUF[count_2] == '\r') && (UART2_RX_BUF[count_2 -1] == '-') && (UART2_RX_BUF[count_2 -2] == '-') && (UART2_RX_BUF[count_2 -3] == '-'))
			{
				configMode = 0;
				UART_Write(UART2, "Exit Config Mode\r\n", 18);
				configReadBack = 1;
			}

        }

        if((count_2 > 2) && (configMode == 0))
        {
        	if((UART2_RX_BUF[count_2] == '\r') && (UART2_RX_BUF[count_2 -1] == '+') && (UART2_RX_BUF[count_2 -2] == '+') && (UART2_RX_BUF[count_2 -3] == '+'))
        	{
        		configMode = 1;
        		UART_Write(UART2, "Entered Config Mode\r\n", 21);
        	}

        }

        count_2++;
        if(count_2 > BUF_LEN_LONG)
            count_2 = 0;

        }

    }
    UART2->INTSTS =0;



     u32IntSts = UART0->INTSTS;

         if(u32IntSts & UART_INTSTS_RDAINT_Msk)
         {

             /* Get all the input characters */
             while(UART_IS_RX_READY(UART0))
             {
               UART_Read(UART0, &UART0_RX_BUF[count_0], 1);
               if(gpsDebugData)
            	   UART_Write_main(UART2, &UART0_RX_BUF[count_0], 1);

             if(UART0_RX_BUF[count_0] == '\r')
             	newWord_0 = 1;

             count_0++;
             if(count_0 > BUF_LEN)
             {
            	 count_0 = 0;
            	 // extract $GPRMC
            	 tGRMC = strstr(UART0_RX_BUF,"GPRMC");
            	 if(tGRMC)
            	 {
            		 tEndofRMC = strstr(tGRMC,"$G");
            		 if((tEndofRMC > tGRMC) && (tEndofRMC < UART0_RX_BUF + BUF_LEN))
            		 {
            			 strncpy(GPS_data, tGRMC, tEndofRMC - tGRMC);
            			 newGpsData++;
            		 }
            	 }
            	 else
            	 {
            		 tGRMC = strstr(UART0_RX_BUF,"GNRMC");
					 if(tGRMC)
					 {
						 tEndofRMC = strstr(tGRMC,"$G");
						 if((tEndofRMC > tGRMC) && (tEndofRMC < UART0_RX_BUF + BUF_LEN))
						 {
							 strncpy(GPS_data, tGRMC, tEndofRMC - tGRMC);
							 newGpsData++;
						 }
					 }

            	 }


             }

             }

         }
         UART0->INTSTS =0;


}

/*---------------------------------------------------------------------------------------------------------*/
void UART13_IRQHandler(void)
{
	TIMER_DisableInt(TIMER0);
    UART1_TEST_HANDLE();
   TIMER_EnableInt(TIMER0);
}
uint8_t noByt;
/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART1->INTSTS;
    char * srcStr;


    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {

        /* Get all the input characters */


        while(UART_IS_RX_READY(UART1))
        {
        	UART_Read(UART1, &UART1_RX_BUF[count_1], 1);
       		UART_Write_main(UART2, &UART1_RX_BUF[count_1], 1);

       		if(readIMEI)
       		{
       			readIMEI = 0;
       			srcStr = strstr(&UART1_RX_BUF[count_1 - 35], "CGSN");
       			memcpy(&IMEI[0],srcStr+7,15);
       			UART_Write_main(UART2,"IMEI is :", 9);
       			UART_Write_main(UART2, IMEI, strlen(IMEI));
       			UART_Write_main(UART2, "\r\n",2);


       		}


        if(UART1_RX_BUF[count_1] == '\r')
        	newWord_1 = 1;

        count_1++;
        if(count_1 >= BUF_LEN)
        	count_1 = 0;

        }

    }

    UART1->INTSTS =0;

}
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
        loopCount++;
        timeOut = 1;
        batery++;

    }
}


int main()
  {
	SYS_Init();

//	GSM_RESET_ON;

//	BEEPER_ON;

	RELAY1_ON;

//	POWERRELAY_ON;

//	GSM_RESET_OFF;

    ADC_init();

	UART0_Init();

	UART1_Init();

	UART2_Init();

	NVIC_EnableIRQ(UART02_IRQn);

	NVIC_EnableIRQ(UART13_IRQn);

	UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk));

	UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk));

	UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk));

	// TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);

	NVIC_EnableIRQ(TMR0_IRQn);


	TIMER0->CTL = (TIMER0->CTL & ~(TIMER_CTL_OPMODE_Msk | TIMER_CTL_PSC_Msk))
			| (TIMER_PERIODIC_MODE | (u32Prescale - 1) | TIMER_CTL_INTEN_Msk);
	TIMER0->CMP = __HIRC / u32Prescale / 6;

	TIMER_EnableInt(TIMER0);

	UART_Write(UART2, "edel-iot\n", 10);

	BEEPER_OFF;

	RELAY1_OFF;

	POWERRELAY_OFF;

//	UART_Write(UART1, "AT\r\n",strlen("AT\r\n"));

//	GSM_RESET_OFF;
//
//	delayCounter=900000;
//
//	  while(delayCounter)
//	    {
//	    	delayCounter--;
//
//	    }
//	  GSM_RESET_ON;
//	  WDT_RESET_COUNTER();

	BEEPER_ON;

	TIMER_Start(TIMER0);

	while (1) {

		if(reset_mc)
		{
			reset_mc=0;

			 relay_satus=relay_fmc_read_status();

			 if(relay_satus==0XBB)
			 {
				 RELAY1_ON;
			 }
			 else if(relay_satus==0XAA)
			 {
				 RELAY1_OFF;
			 }
		}

		 cause = SYS->RSTSTS;
//
			if (cause & SYS_RSTSTS_PORF_Msk)
			{
			//	GSM_RESET_OFF;
			//	UART_Write(UART2,"+----------------------------------------+\r\n",44);
				UART_Write(UART2,"Power-On Reset\n",strlen("Power-On Reset\n"));
			//	UART_Write(UART2,"+----------------------------------------+\r\n",44);
				SYS->RSTSTS = cause;
	            power_rest_beep();
//
			}
			if (cause & SYS_RSTSTS_WDTRF_Msk)
			{
			//	GSM_RESET_OFF;
		//		UART_Write(UART2,"+----------------------------------------+\r\n",44);
				UART_Write(UART2,"wdt-On Reset\n",strlen("wdt-On Reset\n"));
		//		UART_Write(UART2,"+----------------------------------------+\r\n",44);
				SYS->RSTSTS = cause;
				srver_test=1;
				wdt_rest_beep();
			}


//		if(uart_test)
//		{
//			if (strstr((char*)UART1_RX_BUF, "ERROR") != NULL)
//			{
//
//			    error_count++;
//			    if(error_count>7)
//			    {
//			    	UART_Write(UART2,"+----------------------------------------+\r\n",44);
//			    	error_count=0;
//			    }
//			}
//		}

		if (configMod) {
			configMod = 0;

			if (strstr(BATTERY, "52")) {
				peakbatt = 58;

				lowbatt = (mincharge20) * 4;

			} else if (strstr(BATTERY, "12")) {
				peakbatt = maxcharge100;

				lowbatt = mincharge20;
			}
			if ((strstr(bpon, "on")) || (strstr(bpon, "ON"))) {
				BEEPON = 1;

			} else if ((strstr(bpon, "off")) || (strstr(bpon, "OFF"))) {
				BEEPON = 0;

			}
		}
		while (configReadBack) {
			read_config_data();
			configReadBack = 0;
			UART_Write(UART2, "read back from main\r\n", 22);
			UART_Write(UART2, cmdReadBk, strlen(cmdReadBk));
			UART_Write(UART2, "\r\n", 2);
			parse_config_data();
			configMod = 1;

		}
		if (configMode) {

			configMode_app();

		}

		if (batery > 30) {

			batery = 0;

			ADC_measure();

			battery[i] = extBatt_V;

			++i;
		}
		if (i >= 10) {
			i = 0;

			for (int i = 0; i < 10; i++) {
				sum = sum + battery[i];
			}
			average = sum / 10;

			average = average * battADC_LC;

			memset(battery, 0, sizeof(battery));

			pextBat_Data(average);
		}

		while (lockTest == 1) {

			if (LStat == 9) {

				LStat = 0;
				if (devLock)
				{
				//	 WDT_RESET_COUNTER();
					TIMER_DisableInt(TIMER0);

					RELAY1_ON; //temp

					relay_fmc_write_status(0XBB);
					 TIMER_EnableInt(TIMER0);
				     alram();
				}
				else {
					BEEPER_ON;
					delayCounter = 2000000;
					delay_loop();
					BEEPER_OFF;
					delayCounter = 2000000;
					delay_loop();
					BEEPER_ON;
					delayCounter = 2000000;
					delay_loop();
					BEEPER_OFF;

				}
				lockTest = 0; // to be 0
			} else if (LStat == 5) {

				LStat = 0;
				if (devLock)
				{
				//	 WDT_RESET_COUNTER();
			//		TIMER_DisableInt(TIMER0);

					RELAY1_OFF; //temp lock_bike

					 relay_fmc_write_status(0XAA);

			//		 TIMER_EnableInt(TIMER0);
					 alram();
				}
				else {
					BEEPER_ON;
					delayCounter = 400000;
					delay_loop();
					BEEPER_OFF;
				}
				lockTest = 0;
			}
		}

		if (timeOut) {
			timeOut = 0;

			if (newGpsData > 10)
			{

				UART_Write_main(UART2, "\r\n", 2);
				UART_Write_main(UART2, "NEW GPS DATA\r\n", 14);
				UART_Write_main(UART2, GPS_data, strlen(GPS_data));
				UART_Write_main(UART2, "\r\n\r\n", 4);
				get_gps_data();
				memset(GPS_data, 0, 100);
				newGpsData = 0;

				ADC_measure();
				intBattV_conv = intBatt_V * battint;
				extBattV_conv = extBatt_V *battADC_LC;

				memset(intBatt_Data, 0, API_BUFF_LEN);
				memset(extBatt_Data, 0, API_BUFF_LEN);

				if(!srver_test)
				ftoa(intBattV_conv, intBatt_Data, 3);

				ftoa(extBattV_conv, extBatt_Data, 3);

				UART_Write_main(UART2, "INT BATT:", 10);
				UART_Write_main(UART2, intBatt_Data, strlen(intBatt_Data));
				UART_Write_main(UART2, " V\r\n", 4);
				UART_Write_main(UART2, "EXT BATT:", 10);
				UART_Write_main(UART2, extBatt_Data, strlen(extBatt_Data));
				UART_Write_main(UART2, " V\r\n", 4);

			}
			switch (gsmState) {

			case GSM_INIT:
				gsm_init();
				  WDT_RESET_COUNTER();
				break;

			case GSM_SNAPSHOT:
				gsm_get_snapshot();
				  WDT_RESET_COUNTER();
				break;

			case GSM_DATACALL:
				gsm_datacall();
				  WDT_RESET_COUNTER();
				break;

			case GSM_CONNECT:
				gsm_connect();
				  WDT_RESET_COUNTER();
				break;

			case GSM_QUERY:
				gsm_query();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_REPORT_DATA:
				gsm_http_report_data();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_GET_LOCK_STATUS:
				gsm_http_get_lock_status();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_SEND_LOCK:
				gsm_http_send_lock_status();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_SEND_UNLOCK:
				gsm_http_send_unlock_status();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_SEND_LIGHTON:
				gsm_http_send_lightON_status();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_SEND_LIGHTOFF:
				gsm_http_sending_lightOff_status();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_SEND_BEEPON:
				gsm_http_send_beepOn_status();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_SEND_BEEPOff:
				gsm_http_send_beepOff_status();
				  WDT_RESET_COUNTER();
				break;

			case GSM_HTTP_SEND_REG:
				gsm_http_send_registartion();
				  WDT_RESET_COUNTER();
				break;

			case GSM_BEEP_ON:
				gsm_beep_on();
			   WDT_RESET_COUNTER();
			   break;

			case GSM_UNLOCK:
				gsm_gsm_idle();
				WDT_RESET_COUNTER();

			case GSM_SLEEP:
				gsm_idle();
				  WDT_RESET_COUNTER();
				break;

			default:
			//	gsmState=GSM_HTTP_GET_LOCK_STATUS;
				loopCount=0;
				  WDT_RESET_COUNTER();
				break;

			}
		}
	}
}



void gsm_init()
{
	switch (loopCount)
	{

	case 1:
		UART_Write(UART1, "AT\r\n", sizeof("AT\r\n"));

		break;
	case 2:
		UART_Write(UART1, "ATE1V1\r\n", sizeof("ATE1V1\r\n"));
		break;
	case 3:
		UART_Write(UART1, "AT+CGMM\r\n", sizeof("AT+CGMM\r\n"));
		break;
	case 4:
		UART_Write(UART1, "AT+CGMI\r\n", sizeof("AT+CGMI\r\n"));
		break;
	case 5:
		UART_Write(UART1, "AT+CMGD=4\r\n", sizeof("AT+CMGD=4\r\n"));
		break;
	case 6:
		UART_Write(UART1, "AT+CIMI\r\n",9 );
		GSM_RESET_ON;
		break;
	case 7:
		break;
	case 8:
		gsmState = GSM_SNAPSHOT;
		loopCount = 0;
		break;

	default:
		if (loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}
}
/**
 * @brief       gsm_get_snapshot
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Get a snapshot of GSm module, network etc.
 */
void gsm_get_snapshot()
{
	switch (loopCount)
	{

	case 1:
		UART_Write(UART1, "AT+CGMR\r\n", sizeof("AT+CGMR\r\n"));
		break;
	case 2:
		UART_Write(UART1, "AT+CGSN\r\n", 9);
		loop_delay(1);
		break;
	case 3:
		readIMEI = 1;
		UART_Write(UART1,"\r\n",2);
		break;
	case 4:
		UART_Write(UART1, "AT+CCLK?\r\n", 10);
		break;
	case 5:
		UART_Write(UART1, "AT+CPIN?\r\n", 10);
		loop_delay(1);
		break;
	case 6:
		UART_Write(UART1, "AT+CGDCONT?\r\n", 13);
		break;
	case 7:
		UART_Write(UART1, "AT+CSQ\r\n", sizeof("AT+CSQ\r\n") );
		break;
	case 8:
		UART_Write(UART1, "AT+IPREX=9600\r\n", sizeof("AT+IPREX=9600\r\n"));
		break;
	case 9:
		UART_Write(UART1, "AT+COPS?\r\n", sizeof("AT+COPS?\r\n"));
		loop_delay(5);

		break;
	case 10:
		UART_Write(UART1, "AT+CREG?\r\n",sizeof("AT+CREG?\r\n" ));
//		POWERRELAY_ON;
		break;
	case 11:
		UART_Write(UART1, "AT+CGACT?\r\n",sizeof("AT+CGACT?\r\n" ));
		break;
	case 12:
		UART_Write(UART1, "AT+CFUN?\r\n", sizeof("AT+CFUN?\r\n" ));
		break;
	case 13:
		UART_Write(UART1, "AT+CGCLASS?\r\n", sizeof("AT+CGCLASS?\r\n"));
		break;
	case 14:
		UART_Write(UART1, "AT+CSCA?\r\n", sizeof("AT+CSCA?\r\n"));
		break;
	case 15:
		UART_Write(UART1, "AT+CMGF?\r\n",sizeof("AT+CMGF?\r\n" ));
		BEEPER_OFF;
		break;
	case 16:
		gsmState = GSM_DATACALL;
		loopCount = 0;
		break;
	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}

}
/**
 * @brief       gsm_datacall
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Set APN, connect, get device ip address
 */
void gsm_datacall()
{
	switch (loopCount)
	{

	/*Disconnect and re connect to be safe - follow datacall bearer connect
	 * file or query, check response and connect or disconnect*/
	case 1:
		UART_Write(UART1, "AT+CREG?\r\n", 10);
		break;
	case 2:
		UART_Write(UART1, "AT+CGREG?\r\n", 11);
		break;
	case 3:
		UART_Write(UART1, "AT+CPSI?\r\n", 10);
		break;
	case 4:
		UART_Write(UART1, "AT+CNSMOD?\r\n", 12);
		break;
	case 5:
		UART_Write(UART1, "AT+CGACT?\r\n", 11);
		break;
	case 6:
		UART_Write(UART1, "AT+CMEE=1\r\n", 11);
		break;
	case 7:
		UART_Write(UART1, "AT+CGATT=1\r\n", 12);
		break;
	case 8:
		UART_Write(UART1, "AT+CGACT=1,1\r\n", 14);
		//loop_delay(2);
		break;
	case 9:
		UART_Write(UART1, "AT+CGPADDR=1\r\n", 14);
		break;
	case 10:
		break;
	case 11:
		//UART_Write(UART1, "AT+CGACT=1,1\r\n",14 );
		//loop_delay(2);
		break;
	case 12:
		gsmState = GSM_QUERY;
		loopCount = 0;
		break;
	default:
		if (loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}

}
/**
 * @brief       gsm_connect
 *
 * @param       None
 *
 * @return      None
 *
 * @details     List bearer, open get IP.
 */
void gsm_connect()
{
	switch (loopCount)
	{

	case 1:
		UART_Write(UART1, "AT+CGACT=1,1\r\n",14 );
		loop_delay(2);
		break;
	case 2:
		UART_Write(UART1, "AT+NETOPEN?\r\n",14 );
		break;
	case 3:
		UART_Write(UART1, "AT+IPADDR\r\n", 11);
		break;
	case 4:
		UART_Write(UART1,"AT+NETOPEN\r\n",12);
		break;
	case 5:
		UART_Write(UART1, "AT+IPADDR\r\n", 11);
		break;
	case 6:
		UART_Write(UART1, "AT+IPADDR\r\n", 11);
		break;
	case 7:
		break;
	case 8:
		gsmState = GSM_QUERY;
		loopCount = 0;
		break;

	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}

}
/**
 * @brief       gsm_query
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Query if connected to apn, check IP.
 */
void gsm_query()
{
	switch (loopCount)
	{

	case 1:
		UART_Write(UART1, "AT+NETOPEN\r\n", 11);// close
	//	loop_delay(1);
		break;
	case 2:
		UART_Write(UART1, "AT+IPADDR\r\n", 12);// query check result
	//	loop_delay(1);
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
			break;
	case 6:
		//	query_conn = 0;
			gsmState = GSM_HTTP_REPORT_DATA;
			loopCount = 0;

		break;

	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}

}
/**
 * @brief       gsm_http_get
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Initialise HTTP, use GET  method.
 */

void gsm_http_report_data()
{
	switch (loopCount)
	{
	case 1:
		uart_test=1;
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		break;
	case 3:
		loop_delay(5);
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "URL",3);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		//if(dataUpdateReady)
		{
			dataUpdateReady = 0;
			UART_Write(UART1,webURL,strlen(webURL));
			UART_Write(UART1,testURL,strlen(testURL));
			UART_Write(UART1, DBQUOTE,1);

		}

		UART_Write(UART1, "\r\n",2);

		break;

	case 4:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
    	loop_delay(5);
    	break;
	case 5:
		UART_Write(UART1, "AT+HTTPHEAD=?\r\n",14);
		break;
	case 6:
	//	UART_Write(UART1, "AT+HTTPHEAD?\r\n",13);
		break;
 	case 7:
 	//	UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
		break;
	case 8:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",14 );
		break;

	case 9:
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 10:
		gsmState = GSM_HTTP_GET_LOCK_STATUS;
		loopCount = 0;
//		uart_test=0;
//		error_count=0;
		break;
	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}

}

void gsm_http_get_lock_status()
{
	char *srcString;
	char lockStat[25];
	char lock[3];

	switch (loopCount)
	{
	case 1:
		count_1 = 0;
		UART_Write_main(UART2, "\nGet Lock Status from Web\r\n",28 );
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		break;
	case 3:
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "URL",3);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"getDeviceInstructions?",strlen("getDeviceInstructions?"));
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "\r\n",2);
		loop_delay(5);
		break;
	case 4:
		break;
	case 5:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
		loop_delay(3);
		//BEEPER_ON;
		beep_delay();
		delayCounter = 1500000;
		delay_loop();
		BEEPER_OFF;
		//BEEPER_ON;
		beep_delay();
		delayCounter = 1500000;
		delay_loop();
		BEEPER_OFF;
		break;
	case 6:
		UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);
	//	loop_delay(5);
		break;
	case 7:
		UART_Write(UART1, "AT+HTTPHEAD\r\n",13 );
	//	loop_delay(5);
		break;
	case 8:
		UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
	//	loop_delay(5);
		break;
	case 9:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",13 );
	//	loop_delay(5);
		break;
	case 10:
		UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18);
	//	loop_delay(5);
		break;
	case 11:
		UART_Write(UART1, "AT+HTTPTERM\r\n",14 );
	//	loop_delay(2);
		break;
	case 12:
		if(newWord_1)
		{
			newWord_1 =0;
			srcString = 0;
			srcString = strstr(UART1_RX_BUF,"instructionsName");
			loop_delay(5);

			if(srcString)
			{
				UART_Write_main(UART2, "\r\n/*** LOCK STATUS ***/\r\n",25);
				UART_Write_main(UART2, srcString + 17, 26);
				UART_Write_main(UART2,"\r\n",2);
				memset(lockStat,0,strlen(lockStat));
				memcpy(lockStat,srcString + 17, 42);

			}
			srcString = 0;

			if((strstr(lockStat,"deviceUnlock")))/*||(strstr(lockStat,"devceUnlok"))||(strstr(lockStat,"eviceUnock"))||(strstr(lockStat,"devceUnloc"))||(strstr(lockStat,"deiceUnlok"))||(strstr(lockStat,"dviceUnlck"))||(strstr(lockStat,"dviceUnock"))||(strstr(lockStat,"deiceUnlck"))||(strstr(lockStat,"deviceUnlck"))||(strstr(lockStat,"dviceUnlck"))||(strstr(lockStat,"deviceU"))||(strstr(lockStat,"devicL"))|| (strstr(lockStat,"devicen")) || (strstr(lockStat,"deviceUnock"))||(strstr(lockStat,"devicU")))*/
			{
				if(lockHist != UNLOCK)
				{
					deviceStatus = UNLOCK;
					lockHist = UNLOCK;
					lockTest = 1;
					LStat = 9;
					loopCount = 0;
					loop_delay(1);
				}
				gsmState =  GSM_UNLOCK;
				loopCount = 0;
				loop_delay(1);

			}
			else if((strstr(lockStat,"deviceLock")))/*||(strstr(lockStat,"devceLock"))||(strstr(lockStat,"eviceLok"))||(strstr(lockStat,"dviceLok"))||(strstr(lockStat,"deiceLoc"))||(strstr(lockStat,"deviceLoc"))||(strstr(lockStat,"dviceLoc"))||(strstr(lockStat,"deviceLck"))||(strstr(lockStat,"deviceL")) ||(strstr(lockStat,"deviceo")) ||(strstr(lockStat,"deviceLok"))|| (strstr(lockStat,"devicL")))*/

			{
				if(lockHist != LOCK)
				{
					deviceStatus = LOCK;
					lockHist = LOCK;
					lockTest = 1;
				     LStat = 5;
					loopCount = 0;
					loop_delay(1);
				}
				gsmState = GSM_HTTP_SEND_LOCK;
				loopCount = 0;
				loop_delay(1);

			}
			else if((strstr(lockStat,"deviceR")))/*||(strstr(lockStat,"deiceRegitration")))*/

			{
				gsmState = GSM_HTTP_SEND_REG;
				loopCount = 0;
				loop_delay(5);
			}
//		    else if((strstr(lockStat,"LightOn")))/*||(strstr(lockStat,"LigtOn"))||(strstr(lockStat,"LihtOn"))||(strstr(lockStat,"Lightn"))||(strstr(lockStat,"LghtOn"))||(strstr(lockStat,"LighOn"))||(strstr(lockStat,"ightOn")))*/
//		   {
//			//	 LIGHT_ON;
//				 loop_delay(1);
//				 gsmState=GSM_HTTP_SEND_LIGHTON;
//				 loopCount = 0;
//				 loop_delay(1);
//			}
//			else if((strstr(lockStat,"LightOff")))/*||(strstr(lockStat,"LihtOff"))||(strstr(lockStat,"LigtOff"))||(strstr(lockStat,"Lightff"))||(strstr(lockStat,"LightOf"))||(strstr(lockStat,"LghtOff"))||(strstr(lockStat,"LighOff"))||(strstr(lockStat,"ightOff")))*/
//			{
//			//	 LIGHT_OFF;
//				 loop_delay(1);
//				 gsmState=GSM_HTTP_SEND_LIGHTOFF;
//				 loopCount = 0;
//				 loop_delay(1);
//
//			}
			else if((strstr(lockStat,"BeepOn")))/*||(strstr(lockStat,"Beepn"))||(strstr(lockStat,"eepOn"))||(strstr(lockStat,"BeepOn"))||(strstr(lockStat,"BepOn"))||(strstr(lockStat,"BeeOn")))*/
			{

				 gsmState=GSM_BEEP_ON;//GSM_HTTP_SEND_BEEPON;
				 alram();
				 loopCount = 0;
				 loop_delay(1);
			}
		else if((strstr(lockStat,"BeepOff")))/*||(strstr(lockStat,"Beepff"))||(strstr(lockStat,"BeeOff"))||(strstr(lockStat,"eepOff"))||(strstr(lockStat,"BepOff"))||(strstr(lockStat,"BeepOf")))*/
			{
				BEEPER_OFF;
				//alram();
			 gsmState=GSM_HTTP_SEND_BEEPOff;
			 loopCount = 0;
			 loop_delay(1);

			}

			else
			{
				deviceStatus = 0;
				gsmState = GSM_SLEEP;
				loopCount = 0;
				loop_delay(5);
			}
		}
		break;
	case 13:
		gsmState = GSM_SLEEP;
		loopCount = 0;
		break;
	case 14:
		break;
	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}

}
void gsm_http_send_lock_status()
{

	switch (loopCount)
	{
	case 1:
		UART_Write(UART1, "Send Lock Status to Web\r\n",25 );
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		break;
	case 3:
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "URL",3);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"lockDevice?",strlen("lockDevice?"));
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1,"&lockStatus=2",strlen("&lockStatus=2"));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "\r\n",2);
		break;
	case 4:
		break;
	case 5:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
		break;
	case 6:
		UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);
		break;
	case 7:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",13 );
		break;
	case 8:
		UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );
		break;
	case 9:
		UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
		break;
	case 10:
		gsmState = GSM_SLEEP;
		loopCount = 0;
		break;

	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}
}

void gsm_http_send_unlock_status()
{
	switch (loopCount)
		{
		case 1:
			UART_Write(UART1, "Send Lock Status to Web\r\n",25 );
			UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
			break;
		case 2:
			UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
			break;
		case 3:
			UART_Write(UART1, "AT+HTTPPARA=",12);
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, "URL",3);
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, ",",1);
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1,webURL,strlen(webURL));
			UART_Write(UART1,"unlockDevice?",strlen("unlockDevice?"));
			UART_Write(UART1,devID,strlen(devID));
			UART_Write(UART1,"&lockStatus=1",strlen("&lockStatus=1"));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, "\r\n",2);
			break;
		case 4:
			break;
		case 5:
			UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
			break;
		case 6:
			UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);

			break;
		case 7:
			UART_Write(UART1, "AT+HTTPREAD?\r\n",13 );
			break;
		case 8:
			UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );
			break;
		case 9:
			UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
			break;
		case 10:
			gsmState = GSM_SLEEP;
			loopCount = 0;
			break;

		default:
			loopCount = 0;
//			if(loopDelay)
//				loopDelay--;
//			else
//				loopCount = tloopCount;
			break;
	}
}
void gsm_http_send_registartion()
{

//	char lockStat[12];

	switch (loopCount)
	{
	case 1:
		UART_Write_main(UART2, "Send Registration Status to Web\r\n",25 );
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		break;
	case 3:
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "URL",3);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"dR?",strlen("dR?"));
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1,"&imeiN=",strlen("&imeiN="));
		UART_Write(UART1,IMEI,strlen(IMEI));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "\r\n",2);
		break;
	case 4:

		break;
	case 5:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
//		loop_delay(3);
		break;
	case 6:
		UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);
//		loop_delay(5);
		break;
	case 7:
	    UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
//	    loop_delay(5);
	   break;
	case 8:
		UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
//		loop_delay(5);
		break;
	case 9:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",14 );
//		loop_delay(5);
		break;
    case 10:
		UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );
//		loop_delay(5);
		break;
	case 11:
		UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
		break;
	case 12:
		gsmState = GSM_SLEEP;
		loopCount = 0;
		break;

	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}

}


void gsm_http_send_lightON_status()
{


	switch (loopCount)
	{
	case 1:
		UART_Write(UART1, "Send light Status to Web\r\n",25 );
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		break;
	case 3:
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "URL",3);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"DeviceLightOn?",strlen("DeviceLightOn?"));
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "\r\n",2);
		break;
	case 4:
		break;
	case 5:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
//		loop_delay(3);
		break;
	case 6:
		UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);
//		loop_delay(5);
		break;
	case 7:
	//	UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
//		loop_delay(5);
		break;
	case 8:
	//	UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
//		loop_delay(5);
		break;
	case 9:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",14 );
//		loop_delay(5);
		break;
	case 10:
	   UART_Write(UART1, "AT+HTTPREAD=0,500?\r\n",18);
//	   loop_delay(5);
		break;
	case 11:
		UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
		break;
	case 12:
		gsmState = GSM_SLEEP;
		loopCount = 0;
		break;

	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}
}
void gsm_http_sending_lightOff_status()
{


	switch (loopCount)
	{
	case 1:
		UART_Write(UART1, "Send light Status to Web\r\n",25 );
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		break;
	case 3:
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "URL",3);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"DeviceLightOff?",strlen("DeviceLightOff?"));
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "\r\n",2);
		break;
	case 4:

		break;
	case 5:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
	//	loop_delay(3);
		break;
	case 6:
		UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);
	//	loop_delay(5);
		break;
	case 7:
	 //  UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
	 //  loop_delay(5);
	   break;
	case 8:
	//	UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
	//	loop_delay(5);
		break;
	case 9:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",14 );
	//	loop_delay(5);
		break;
	case 10:
	   UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );
	//   loop_delay(5);
		break;
	case 11:
		UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
		break;
	case 12:
		gsmState = GSM_SLEEP;
		loopCount = 0;
		break;

	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}
}

void gsm_http_send_beepOn_status()
{


	switch (loopCount)
	{
	case 1:
		UART_Write(UART1, "Send beep Status to Web\r\n",25 );
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		break;
	case 3:
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "URL",3);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"setBeepOn?",strlen("setBeepOn?"));
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "\r\n",2);
		break;
	case 4:

		break;
	case 5:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
	//	loop_delay(3);
		break;
	case 6:
		UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);
//		loop_delay(5);
		break;
	case 7:
	//	UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
//	    loop_delay(5);
	    break;
	case 8:
	//	UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
//		loop_delay(5);
		break;
	case 9:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",14 );
//		loop_delay(5);
		break;
	case 10:
		UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );
//		loop_delay(5);
		break;
	case 11:
		UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
		break;
	case 12:
		gsmState = GSM_SLEEP;
		loopCount = 0;
		break;

	default:
		loopCount = 0;
//		if(loopDelay)
//			loopDelay--;
//		else
//			loopCount = tloopCount;
		break;
	}
}
void gsm_beep_on()
{
	switch (loopCount)
		{
		case 1:
			UART_Write(UART1, "Send beep Status to Web\r\n",25 );
			UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
			break;
		case 2:
			UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
			break;
		case 3:
			UART_Write(UART1, "AT+HTTPPARA=",12);
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, "URL",3);
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, ",",1);
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1,webURL,strlen(webURL));
			UART_Write(UART1,"setBeepOn?",strlen("setBeepOn?"));
			UART_Write(UART1,devID,strlen(devID));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, "\r\n",2);
			break;
		case 4:

			break;
		case 5:
			UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
		//	loop_delay(3);
			break;
		case 6:
			UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);
	//		loop_delay(5);
			break;
		case 7:
		//	UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
	//	    loop_delay(5);
		    break;
		case 8:
		//	UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
	//		loop_delay(5);
			break;
		case 9:
			UART_Write(UART1, "AT+HTTPREAD?\r\n",14 );
	//		loop_delay(5);
			break;
		case 10:
			UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );
	//		loop_delay(5);
			break;
		case 11:
			UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
			break;
		case 12:
			gsmState = GSM_SLEEP;
			loopCount = 0;
			break;

		default:
			loopCount = 0;
	//		if(loopDelay)
	//			loopDelay--;
	//		else
	//			loopCount = tloopCount;
			break;
		}
}

void gsm_http_send_beepOff_status()
{


	switch (loopCount)
	{
	case 1:
		UART_Write(UART1, "Send beep Status to Web\r\n",25 );
		UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
		break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		break;
	case 3:
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "URL",3);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"setBeepOff?",strlen("setBeepOff?"));
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "\r\n",2);
		break;
	case 4:

		break;
	case 5:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
	//	loop_delay(3);
		break;
	case 6:
     	UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);
	//	loop_delay(5);
		break;
	case 7:
	//	UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
//		 loop_delay(5);
	   break;
	case 8:
	//	UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
	//	loop_delay(5);
		break;

	case 9:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",14 );
	//	loop_delay(5);
			break;
    case 10:
		UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );
	//	loop_delay(5);
			break;
	case 11:
		  UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
		  break;
	case 12:
		gsmState = GSM_SLEEP;
		loopCount = 0;
		break;

	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}
}


/**
 * @brief       gsm_idle
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Idle.
 */
void gsm_gsm_idle()
{
	switch (loopCount)
	{
	case 1:
				UART_Write(UART1, "Send Lock Status to Web\r\n",25 );
				UART_Write(UART1, "AT+HTTPTERM\r\n", 13);
				break;
			case 2:
				UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
				break;
			case 3:
				UART_Write(UART1, "AT+HTTPPARA=",12);
				UART_Write(UART1, DBQUOTE, 1);
				UART_Write(UART1, "URL",3);
				UART_Write(UART1, DBQUOTE, 1);
				UART_Write(UART1, ",",1);
				UART_Write(UART1, DBQUOTE,1);
				UART_Write(UART1,webURL,strlen(webURL));
				UART_Write(UART1,"unlockDevice?",strlen("unlockDevice?"));
				UART_Write(UART1,devID,strlen(devID));
				UART_Write(UART1,"&lockStatus=1",strlen("&lockStatus=1"));
				UART_Write(UART1, DBQUOTE,1);
				UART_Write(UART1, "\r\n",2);
				break;
			case 4:
				break;
			case 5:
				UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
				break;
			case 6:
				UART_Write(UART1, "AT+HTTPHEAD=?\r\n",15);

				break;
			case 7:
				UART_Write(UART1, "AT+HTTPREAD?\r\n",13 );
				break;
	case 8:
		break;
	case 9:
		break;
	case 10:
		// check periodically for status changes
	//	UART_Write_main(UART2, "\nCheck Connectivity periodically\r\n",34 );
		gsmState = GSM_SLEEP;
		loopCount = 0;
//		query_conn = 1;
//		query_conn = 0;  // should check edel each time
		break;
	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;

	}

}
void gsm_idle()
{
	switch (loopCount)
		{
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
			break;
		case 9:
			break;
		case 10:
			// check periodically for status changes
			UART_Write_main(UART2, "\nCheck Connectivity periodically\r\n",34 );
			gsmState = GSM_QUERY;
			loopCount = 0;
	//		query_conn = 1;
	//		query_conn = 0;  // should check edel each time
			break;
		default:
			if(loopDelay)
				loopDelay--;
			else
				loopCount = tloopCount;
			break;

		}

}

void loop_delay(uint8_t loopDel)
{
	tloopCount = loopCount;
	loopCount = 0x5f;
	loopDelay = loopDel;
}

void get_gps_data()
{


UART_Write_main(UART2,"\r\n", 2);


memset(lat_deg,0,2);
memset(lat_min,0,2);

memset(LAT_min,0,10);

memset(minLat,0,2);
memset(minLong,0,2);

memset(dategpsy,0,5);
memset(dategpsm,0,3);
memset(dategpsd,0,3);

memset(timegpsh,0,3);
memset(timegpsm,0,3);
memset(timegpss,0,3);


memset(lat_sec,0,7);
memset(long_deg,0,3);
memset(long_min,0,3);

memset(LONG_min,0,10);

memset(long_sec,0,7);
memset(secLatUStr,0,2);
memset(secLatLStr,0,5);
memset(secLongUStr,0,2);
memset(secLongLStr,0,5);


rtc = strstr(GPS_data,",");
gpsStatus = strstr(rtc +1,",");

//if gpsSatus is invalid, dont use the data, skip

if(strncmp(gpsStatus + 1, "A",1) == 0) // valid gps data continue
{
	latitude = strstr(gpsStatus +1,",");
	strncpy(lat_deg, latitude+1, 2);
	strncpy(lat_min, latitude+3, 2);
	strncpy(lat_sec, latitude+6, 5);

	strncpy(LAT_min,lat_min,2);
	strcat(LAT_min,lat_sec);

	LAT_min_l = atoi(LAT_min);
	LAT_min_l = LAT_min_l * 100;
	LAT_min_l = LAT_min_l/60;
	itoa(LAT_min_l,LAT_min_cov,10);


	minutes = atoi(lat_min);
	minutes = (minutes*100)/60;
	itoa(minutes,minLat,10);

	seconds = atoi(lat_sec);
	seconds = seconds *60;
	secLatU = seconds/100000;
	secLatL= seconds%100000;
	itoa(secLatU,secLatUStr,10);
	itoa(secLatL,secLatLStr, 10);
	hem = strstr(latitude+1,",");

	longitude = strstr(hem+1,",");
	strncpy(long_deg, longitude+1, 3);
	strncpy(long_min, longitude+4, 2);
	strncpy(long_sec, longitude+7, 5);

	strncpy(LONG_min,long_min,3);
	strcat(LONG_min,long_sec);

	LONG_min_l = atoi(LONG_min);
	LONG_min_l = LONG_min_l * 100;
	LONG_min_l = LONG_min_l/60;
	itoa(LONG_min_l,LONG_min_cov,10);



	minutes = atoi(long_min);
	minutes = (minutes * 100)/60;
	itoa(minutes,minLong,10);

	secondsLong = atoi(long_sec);
	secondsLong = secondsLong *60;
	secLongU = secondsLong/100000;
	secLongL= secondsLong%100000;
	itoa(secLongU,secLongUStr,10);
	itoa(secLongL,secLongLStr, 10);

	ew = strstr(longitude+1,",");
	Speed = strstr(ew +1,",");
	cog = strstr(Speed+1,",");
	date = strstr(cog+1,",");
	mv = strstr(date+1,",");

	UART_Write_main(UART2,"\r\n\r\n:", 4);
	UART_Write_main(UART2,"---------------\r\n", 17);
	UART_Write_main(UART2,"    GPD DATA   \r\n", 17);
	UART_Write_main(UART2,"---------------\r\n", 17);

	UART_Write_main(UART2,"TIME:", 5);

	if(gpsStatus > rtc)
	{
		UART_Write_main(UART2, rtc+1, gpsStatus - rtc - 1);
		memset(gpsTimeData,0,API_BUFF_LEN);
		strncpy(gpsTimeData,rtc+1, gpsStatus - rtc - 1);

	}
	UART_Write_main(UART2,"\r\n", 2);

	UART_Write_main(UART2,"Status:", 7);

	if(latitude > gpsStatus)
	{
		UART_Write_main(UART2, gpsStatus+1, latitude - gpsStatus - 1);
	}
	UART_Write_main(UART2,"\r\n", 2);

	UART_Write_main(UART2,"Latitude:", 9);

	if(hem > latitude)
	{

		UART_Write_main(UART2, latitude+1, hem - latitude - 1);
		memset(gpsLatitudeData,0,API_BUFF_LEN);
		strncpy(gpsLatitudeData,latitude+1, hem - latitude - 1);
	}
	UART_Write_main(UART2,"\r\n", 2);

	UART_Write_main(UART2,"Hemisphere:", 11);

	if(longitude > hem)
	{
		UART_Write_main(UART2, hem+1, longitude - hem - 1);
	}
	UART_Write_main(UART2,"\r\n", 2);

	UART_Write_main(UART2,"Longitude:", 10);
	if(ew > longitude)
	{


		UART_Write_main(UART2, longitude+1, ew - longitude - 1);
		memset(gpsLongitudeData,0,API_BUFF_LEN);
		strncpy(gpsLongitudeData,longitude+1, ew - longitude - 1);

	}
	UART_Write_main(UART2,"\r\n", 2);

	UART_Write_main(UART2,"EW:", 3);

	if(Speed > ew)
	{
		UART_Write_main(UART2, ew+1, Speed - ew - 1);
	}
	UART_Write_main(UART2,"\r\n", 2);

	UART_Write_main(UART2,"Speed:", 6);

	if( cog > Speed)
	{

	 strncpy(fixed,Speed+1,4);

	 knot=atoi(fixed);

	 knotspeed=(knot)*(speeed);

	itoa(knotspeed,Sped,10);

	UART_Write_main(UART2, Speed+1, cog - Speed - 1);

	memset(gpsSpeedData,0,API_BUFF_LEN);

	strncpy(gpsSpeedData,Sped, cog - Speed - 1);


	}
	UART_Write_main(UART2,"\r\n", 2);

	UART_Write_main(UART2,"COG:", 4);
	if( date > cog )
	{
		UART_Write_main(UART2, cog+1, date - cog - 1);
	}
	UART_Write_main(UART2,"\r\n", 2);

	UART_Write_main(UART2,"Date:", 5);
	if( mv > date)
	{
		UART_Write_main(UART2, date+1, mv - date - 1);
		memset(gpsDateData,0,API_BUFF_LEN);
		strncpy(gpsDateData,date+1, mv - date - 1);

	}

	memset(testURL,0,BUF_LEN);

//		strncpy(testURL,webURL,strlen(webURL));

		strcat(testURL,api_update);

		strcat(testURL,gpsLatitude);

	lat_dege=atoi(lat_deg);

	 if((lat_dege>7) && (lat_dege<38))
	 {
	       strcat(testURL,lat_deg);

	        strcat(testURL,".");

	         strcat(testURL,LAT_min_cov);

	           lat_dege=0;
	 }

	strcat(testURL,gpsLongitude);

	long_dege=atoi(long_deg);

	 if((long_dege>60) && (long_dege<97))
	 {

         	strcat(testURL,long_deg);

               	strcat(testURL,".");

              	strcat(testURL,LONG_min_cov);

		         long_dege=0;
	    }

	 strcat(testURL,gpsSpeed);

	         if(knotspeed<40)
	           {
			       strcat(testURL,gpsSpeedData);

	             }
    strcat(testURL,devID);

    strcat(testURL,battVExt);
    strcat(testURL,extBatt_Data);
   	strcat(testURL,battVInt);
   	if(srver_test) /*ASCII value of '1' is 0x31*/
   	 intBatt_Data[0]=0x31;

  	strcat(testURL,intBatt_Data);
   	strcat(testURL,pbattVExt);
    strcat(testURL,str);

	dataUpdateReady = 1;

	UART_Write_main(UART2,"\r\n\r\n", 4);
	UART_Write_main(UART2,"/******************/\r\n", 22);
	UART_Write_main(UART2,testURL,strlen(testURL));
	UART_Write_main(UART2,"\r\n", 2);
	UART_Write_main(UART2,"/******************/\r\n", 22);
	UART_Write_main(UART2,"\r\n\r\n:", 4);

	srver_test=0;
//    BEEPER_ON;
//	delayCounter = 2000000;
//	delay_loop();
//	BEEPER_OFF;
}
else
{
	UART_Write_main(UART2,"/******************/\r\n", 22);
	UART_Write_main(UART2,"GPS - is looking for Satellites",32);
	UART_Write_main(UART2,"\r\n", 2);
	UART_Write_main(UART2,"/******************/\r\n", 22);
	dataUpdateReady = 0;

//		UART_Write_main(UART2,"\r\n\r\n", 4);
//		UART_Write_main(UART2,"/******************/\r\n", 22);
//		UART_Write_main(UART2,testURL,strlen(testURL));
//		UART_Write_main(UART2,"\r\n", 2);
//		UART_Write_main(UART2,"/******************/\r\n", 22);
//		UART_Write_main(UART2,"\r\n\r\n:", 4);
}

}
void pextBat_Data(float extBattV)
{
	mess_battv=(extBattV)-(lowbatt);

    charge=peakbatt-lowbatt;

    differance=(mess_battv/charge);

    percentage=differance*80;

     percentag=percentage+20;

         if((percentag>20)&&(percentag<=100))
           {

                  itoa(percentag,str,10);

            }

            else if(percentag<20)
             {
            	//set the total array is zero
                 	memset(str,48,3);
               //ascii zero value is 48 and two zeros send the server
            	    memset(str,48,2);

             }
}
// Reverses a string 'str' of length 'len'
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

/* check if config mode is on, if configMode is ON, do not write*/
uint32_t UART_Write_main(UART_T* uart, uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
	uint32_t u32Count = 0;

	if(!configMode)
		u32Count = UART_Write(uart, pu8TxBuf, u32WriteBytes);

	return u32Count;

}
void delay_loop()
{
    while(delayCounter)
    {
    	delayCounter--;
    }

}
void beep_delay()
{
	if (BEEPON)
	{
		BEEPER_ON;
	}
}
void alram()
{
	uint8_t k=0;

for(k=0;k<=10;k++)
{
	BEEPER_ON;
	delayCounter = 150000;
	delay_loop();
	BEEPER_OFF;
	delayCounter = 150000;
    delay_loop();
}

}
 void wdt_rest_beep()
 {
	 uint8_t k=0;

	 for( k=0;k<=20;k++)
	 {
		BEEPER_ON;
		delayCounter = 100000;
		delay_loop();
		BEEPER_OFF;
		delayCounter = 100000;
	    delay_loop();
		BEEPER_ON;
		delayCounter = 100000;
		delay_loop();
		BEEPER_OFF;
		delayCounter = 100000;
	    delay_loop();
	 }
 }
 void power_rest_beep()
 {
	 BEEPER_ON;
	 delayCounter = 11000000;
	 delay_loop();
	 BEEPER_OFF;
 }
