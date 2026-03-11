/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/10/17 3:06p $
 * @brief    Software Development Template.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M0564.h"
#include <string.h>
#include "ADC.h"
#include "gpio.h"
#include "config.h"


//#define M0564_DEV_BOARD 1
//#define EDEL_TRACKER_BRD_1 1
#define EDEL_TRACKER_BRD_2 1 // SIMCOM AND UBLOX WITH M0564

/***************************************************/
//// TO SWITCH BETWEEN TEST AND PRODUCTION VERSION
//
/***************************************************/
//#define KRITIN_DEV_VER 1
//#define KRITIN_PROD_V01 1
#define EVEGAH_BETA 1

#define  LIGHT  1

#ifdef LIGHT
uint16_t lockCounte;
uint16_t count;
uint16_t turn;
uint8_t  comand[1];
uint8_t  light;
uint8_t beepn;
void alram();
void Alaram(uint32_t ,uint32_t ,uint32_t );
//uint32_t ONTIME;
//uint32_t OFFTIME;
//uint32_t TIMES;

#endif

#define solenoid_E  PE10= 1
#define solenoid_D  PE10= 0

#define  RELAYPOWER_ON    PE11= 1
#define  RELAYPOWER_OFF   PE11= 0

#define NEW_STEPPER_MOTOR 1

#ifdef NEW_STEPPER_MOTOR

#define ENABLE_A 	PD4 = 1
#define DISABLE_A	PD4 = 0

#define ENABLE_B	PD5 = 1
#define DISABLE_B	PD5 = 0

#define SET_1A	PD7 = 1
#define CLR_1A	PD7 = 0

#define SET_1B	PC0 = 1
#define CLR_1B	PC0 = 0

#define SET_2A	PC1 = 1
#define CLR_2A	PC1 = 0

#define SET_2B	PC4 = 1
#define CLR_2B	PC4 = 0

#define CLR_ALL (CLR_1A,CLR_1B,CLR_2A,CLR_2B)



#define ON_WIND_1_CW 	(SET_1A, CLR_1B)
#define ON_WIND_1_CCW	(CLR_1A, SET_1B)
#define OFF_WIND_1 		(CLR_1A, CLR_1B)

#define ON_WIND_2_CW 	(SET_2A, CLR_2B)
#define ON_WIND_2_CCW	(CLR_2A, SET_2B)
#define OFF_WIND_2		(CLR_2A, CLR_2B )

void stepper_motor_drv(uint8_t coil);

uint8_t stepper_wave_pattern[10];
uint8_t stepperNoStep = 3;
uint8_t stepCW[4] = {17, 18, 34, 33};
uint8_t stepCCW[4] = {17, 33, 34, 18};

uint8_t stepCount =0;
uint8_t countloop=0;

#endif
/***************************************************/

#define GPS_UART_ENABLE 1

#define PLL_CLOCK       72000000

#define GPS_BUF_LEN 250
#define UNLOCK 0xA5
#define LOCK 0x5A
#define LOCK_BOOT_UP 0x0F
#define TURN_COUNT 290 //250

uint8_t UART2_RX_BUF[BUF_LEN_LONG];
uint8_t UART2_TX_BUF[BUF_LEN];
uint16_t count_2;
uint8_t newWord_2 =0;
uint8_t deviceStatus;
uint8_t lockHist;
uint16_t turnCount=1500;

uint8_t configMode = 0;

uint8_t UART1_RX_BUF[BUF_LEN];
uint8_t UART1_TX_BUF[BUF_LEN];
uint8_t count_1;
uint8_t newWord_1 =0;

uint8_t UART0_RX_BUF[BUF_LEN];
uint8_t UART0_TX_BUF[BUF_LEN];
uint8_t count_0;
uint8_t newWord_0 =0;

uint8_t gpsDebugData = 0;
uint8_t dataUpdateReady=0;

uint8_t swDevProd = 0;
uint8_t configReadBack = 1;


#define LATITUDE "latitude="

#ifdef EVEGAH_BETA
uint8_t webURL[API_BUFF_LEN];
#endif

#ifdef KRITIN_DEV_VER
uint8_t webURL[API_BUFF_LEN] = "http://evegah-admin.kritin.in/api/";
//uint8_t webURL[API_BUFF_LEN];
#endif

#ifdef KRITIN_PROD_V01
uint8_t webURL[API_BUFF_LEN] = "http://admin.evegah.com/api/";
#endif

uint8_t APN_URL[URL_LENGTH] = "airtelgprs.com";


uint8_t api_update[API_BUFF_LEN] = "uDIM?";
uint8_t api_getDevInst[API_BUFF_LEN] = "getDeviceInstructions?";
uint8_t api_lockDev[API_BUFF_LEN] = "lockDevice?";
uint8_t api_unlockDev[API_BUFF_LEN] = "unlockDevice?";

uint8_t gpsLocation[API_BUFF_LEN]="loc=INDIA";
uint8_t gpsLatitude[API_BUFF_LEN]="&lat=";
uint8_t gpsLongitude[API_BUFF_LEN]="&long=";
uint8_t gpsAltitude[API_BUFF_LEN]="&alt=";
uint8_t gpsSpeed[API_BUFF_LEN]="&sp=";
uint8_t gpsTime[API_BUFF_LEN]="";
uint8_t gpsDate[API_BUFF_LEN]="";

#define BATT_VOLTAGE_MEAS 1
#ifdef BATT_VOLTAGE_MEAS
uint8_t battVolatge[]="&battery=36V";
uint8_t battVInt[]="&ibv=";
uint8_t battVExt[]="&ebv=";
uint8_t pbattVExt[]="&pebv=";
#endif
#define maxcharge100  13.5
#define mincharge20 10.5
uint8_t str[3];
float mess_battv;
float charge;
float differance;
float peakbatt;
float lowbatt;
uint8_t percentage;
uint8_t percentag;
uint8_t percent;
uint8_t BATTERY[BUF_LEN_SMALL];
uint8_t configMod;
uint8_t BPON[BUF_LEN_SMALL];
uint8_t bpon[BUF_LEN_SMALL];

uint8_t BEEPON;

float average;

uint8_t readIMEI=0;
uint8_t IMEI[BUF_LEN_SMALL];


uint8_t devID[BUF_LEN_SMALL];

/*GPS DATA*/
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

uint8_t pwmTest = 0x04;
uint16_t freq = 2731;
uint8_t dCycle = 50;
uint8_t beepOn =0;
uint32_t beepDelay;

uint16_t beepOnTime;
uint16_t beepOffTime;

uint16_t noOfBeeps;
uint16_t beepCounter;
uint8_t beepTest, beepON, beepOFF, beepCOUNT;
uint8_t devLock = 1; // config should set this to true for lock

#define BEEP_ON 2
#define BEEP_OFF 5

uint8_t gpsLatitudeData[API_BUFF_LEN];
uint8_t gpsLongitudeData[API_BUFF_LEN];
uint8_t gpsAltitudeData[API_BUFF_LEN];
uint8_t gpsSpeedData[API_BUFF_LEN];
uint8_t gpsTimeData[API_BUFF_LEN];
uint8_t gpsDateData[API_BUFF_LEN];
uint8_t extBatt_Data[API_BUFF_LEN];
uint8_t intBatt_Data[API_BUFF_LEN];

uint16_t pextBatt_Data;

uint32_t battery[10];

uint8_t batery;
uint8_t i;

extern uint32_t intBatt_V;
extern uint32_t extBatt_V;
extern uint32_t cmdReadBk[BUF_LEN_LONG/4];

// batt voltage measurement potential divider is 18K and 1K
// one ADC count corresponds to 0.023565 Volts
float battADC_LC = 0.023;//0.0222;//0.022;
float intBattV_conv;
float extBattV_conv;
float battint=0.00177;


uint8_t testURL[BUF_LEN];
//uint8_t testURL[255];
char SMState[10];
uint8_t loopStr[10];
uint16_t loopCounter;
uint32_t delayCounter = 200000;
uint32_t delayTempCount;

#ifdef EDEL_TRACKER_BRD_2

#define IGNITION_ON 	PD2 = 1
#define IGNITION_OFF    PD2 = 0

#define LIGHT_ON	PD3 = 1
#define LIGHT_OFF	PD3 = 0

#define BEEPER_ON PE2 = 1
#define BEEPER_OFF PE2 = 0

#endif

uint8_t length = 0;
volatile uint32_t g_au32TMRINTCount[4] = {0};
uint8_t loopCount = 0;
uint8_t tloopCount=0;
uint8_t loopDelay=0;
uint8_t timeOut = 0 ;
uint8_t query_conn = 0;

uint8_t *tGRMC;
uint8_t *tEndofRMC;
uint8_t GPS_data[100];
uint8_t newGpsData = 0;

uint32_t counter;
uint8_t gpsIntInit = 0;
uint8_t ubx_reset[12]= {0xB5, 0x62, 0x06, 0x04, 0x04, 0, 0xff, 0xff, 0x02, 0x00, 0x0e, 0x61};
uint8_t LStat = 0;
uint8_t lockTest=0;


/* special char */
char DBQUOTE[1] ={0x22};

void UART2_TEST_HANDLE();
void UART1_TEST_HANDLE();


void loop_delay(uint8_t loopDel);
void gsm_init();
void gsm_get_snapshot();
void gsm_datacall();
void gsm_connect();
void gsm_query();
void gsm_http_get();
void gsm_idle();

void gsm_http_report_data();
void gsm_http_get_lock_status();

void gsm_http_send_lock_status();
void gsm_http_send_unlock_status();
void gsm_http_send_registartion();

void gsm_http_send_lightON_status();
void gsm_http_send_lightOff_status();
void gsm_http_send_beepOn_status();
void gsm_http_send_beepOff_status();

void get_gps_data();


void rotate_unlock();
void rotate_lock();

void pextBat_Data(float);

int intmyatoi(char *);

void reverse(char* str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char* res, int afterpoint);

void beep_on();
void beep_off();
void beep_control();
void stepper_test();

void delay_loop();
void delay_loop_2();
void delay_loop_4();

void set_A();
void set_B();
void set_C();
void set_D();

void set_AB();
void set_BC();
void set_CD();
void set_DA();

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
	GSM_HTTP_SEND_REG,
	GSM_HTTP_SEND_LIGHTON,
	GSM_HTTP_SEND_LIGHTOFF,
	GSM_HTTP_SEND_BEEPOff,
	GSM_HTTP_SEND_BEEPON,
	GSM_SLEEP,

}GSM_SM_T;

uint8_t gsmState = GSM_INIT;


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void)
{
    UART2_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_TEST_HANDLE()
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
        	if((UART2_RX_BUF[count_2] == '\r') && (UART2_RX_BUF[count_2 -1] == '1') && (UART2_RX_BUF[count_2 -2] == '$') && (UART2_RX_BUF[count_2 -3] == '$'))
			{
					UART_Write(UART2, "Forced Lock\r\n", 13);
					lockTest = 1;
					LStat = 5;
			}
        	else if((UART2_RX_BUF[count_2] == '\r') && (UART2_RX_BUF[count_2 -1] == '2') && (UART2_RX_BUF[count_2 -2] == '$') && (UART2_RX_BUF[count_2 -3] == '$'))
        	{
        			UART_Write(UART2, "Forced Unlock\r\n", 15);
        			lockTest = 1;
        			LStat = 9;
        	}

        	else if((UART2_RX_BUF[count_2] == '\r') && (UART2_RX_BUF[count_2 -1] == '3') && (UART2_RX_BUF[count_2 -2] == '$') && (UART2_RX_BUF[count_2 -3] == '$'))
        	           {
        	        	 UART_Write(UART2, "1_light on  2_light off  \r\n", 25);
                            light=1;
        	        	}

        	else if((UART2_RX_BUF[count_2] == '\r') && (UART2_RX_BUF[count_2 -1] == '4') && (UART2_RX_BUF[count_2 -2] == '$') && (UART2_RX_BUF[count_2 -3] == '$'))
        	        	           {
        	        	        //	 UART_Write(UART2, "beepON\r\n", 10);
        	        	        //	 beepn = 1;
        		                    percent=0;
        	        	        	 UART_Write(UART2, "_12v \r\n", 5);
        	        	        	// percent=0;

        	        	        	 percent=1;
        	        	        	}
        	else if((UART2_RX_BUF[count_2] == '\r') && (UART2_RX_BUF[count_2 -1] == '5') && (UART2_RX_BUF[count_2 -2] == '$') && (UART2_RX_BUF[count_2 -3] == '$'))
        	        	        	           {
        		                                 percent=0;
        	        	        	        	 UART_Write(UART2, "_36v \r\n", 5);
        	        	        	        //	 percent=0;

        	        	        	        	 percent=2;
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
void UART1_IRQHandler(void)
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
       			srcStr = strstr(&UART1_RX_BUF[count_1 - 35], "CIMI");
       			memcpy(&IMEI[0],srcStr+7,15);
       			UART_Write_main(UART2,"IMEI is :", 9);
       			UART_Write_main(UART2, IMEI, strlen(IMEI));

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


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_M0564.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        g_au32TMRINTCount[0]++;
        loopCount++;
        timeOut = 1;
        batery++;
        countloop++;


    }
}



#ifdef EDEL_TRACKER_BRD_2
void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC,9600);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRST1 |=  SYS_IPRST1_UART1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART1RST_Msk;

    /* Configure UART1 and set UART1 baud rate */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC,9600);
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

//	  SYS_ResetModule(UART1_RST);
//
//	    /* Configure UART0 and set UART0 Baudrate */
//	    UART_Open(UART1, 9600);
}

void UART2_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRST1 |=  SYS_IPRST1_UART2RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART2RST_Msk;

    /* Configure UART1 and set UART1 baud rate */
    UART2->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC,9600);
    UART2->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

//	  SYS_ResetModule(UART2_RST);
//
//		    /* Configure UART0 and set UART0 Baudrate */
//		    UART_Open(UART2, 9600);
}
#endif

void SYS_Init(void)
{
#ifdef EDEL_TRACKER_BRD_2
        /* Set PF multi-function pins for X32_OUT(PF.0) and X32_IN(PF.1) */
        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_X32_OUT;
        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk)) | SYS_GPF_MFPL_PF1MFP_X32_IN;

        /*---------------------------------------------------------------------------------------------------------*/
        /* Init System Clock                                                                                       */
        /*---------------------------------------------------------------------------------------------------------*/


        /* Enable HIRC and LXT clock */
      CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LXTEN_Msk);

        /* Wait for HIRC and LXT clock ready */
        while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));
        while(!(CLK->STATUS & CLK_STATUS_LXTSTB_Msk));

        /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
        CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
        CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

        /* Update System Core Clock */
        SystemCoreClockUpdate();

        /* Enable UART module clock */
        CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk | CLK_APBCLK0_UART2CKEN_Msk);

        /* Select UART module clock source as HIRC and UART module clock divider as 1 */
        CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC;
        CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk)) | CLK_CLKDIV0_UART(1);

        CLK_EnableModuleClock(TMR0_MODULE);
        CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

        CLK_EnableModuleClock(TMR2_MODULE);
        CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HIRC, 0);
       /*----------------------------------------------------------------*/
#endif

#ifdef EDEL_TRACKER_BRD_2
/*******************************uart0******************************************/
        SYS->GPA_MFPL &= ~( SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
        SYS->GPA_MFPL |= ( SYS_GPA_MFPL_PA2MFP_UART0_TXD | SYS_GPA_MFPL_PA3MFP_UART0_RXD);
/*******************************uart1*******************************************/
        SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk );
         SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_UART1_TXD | SYS_GPA_MFPL_PA1MFP_UART1_RXD );


//       SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE12MFP_Msk | SYS_GPE_MFPH_PE13MFP_Msk);
//       SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE12MFP_UART1_TXD | SYS_GPE_MFPH_PE13MFP_UART1_RXD);

/**********************************uart2****************************************/
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC2MFP_Msk |SYS_GPC_MFPL_PC3MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC2MFP_UART2_TXD | SYS_GPC_MFPL_PC3MFP_UART2_RXD);
/***********************************gpio*****************************************/
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk |SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk |SYS_GPB_MFPL_PB7MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB4MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO | SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB7MFP_GPIO );

    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk );
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD2MFP_GPIO | SYS_GPD_MFPL_PD3MFP_GPIO );

    SYS->GPE_MFPL &= (SYS_GPE_MFPL_PE0MFP_Msk);
    SYS->GPE_MFPL |= SYS_GPE_MFPL_PE0MFP_GPIO;

    SYS->GPE_MFPH &= (SYS_GPE_MFPH_PE11MFP_Msk);
    SYS->GPE_MFPH |= SYS_GPE_MFPH_PE11MFP_GPIO;

    GPIO_SetMode(PE, BIT11, GPIO_MODE_OUTPUT);

#ifdef NEW_STEPPER_MOTOR
    /********************************/
    /**** TEMP STEPPER MOTOR CODE ***/
    /********************************/

    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk |SYS_GPD_MFPL_PD7MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD4MFP_GPIO | SYS_GPD_MFPL_PD5MFP_GPIO | SYS_GPD_MFPL_PD7MFP_GPIO );

    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk |SYS_GPC_MFPL_PC1MFP_Msk |SYS_GPC_MFPL_PC4MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_GPIO | SYS_GPC_MFPL_PC1MFP_GPIO | SYS_GPC_MFPL_PC4MFP_GPIO);

    GPIO_SetMode(PD, BIT4, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, BIT5, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, BIT7, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, BIT0, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, BIT1, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, BIT4, GPIO_MODE_QUASI);
#endif


    //************************switch int***************************************//

            SYS->GPD_MFPL &= ~(SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk);
            SYS->GPD_MFPL |= SYS_GPC_MFPL_PC6MFP_GPIO | SYS_GPC_MFPL_PC7MFP_GPIO;

            GPIO_SetMode(PC, BIT6, GPIO_MODE_INPUT);
            GPIO_EnableInt(PC, 6, GPIO_INT_BOTH_EDGE  );

                	GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
                	GPIO_ENABLE_DEBOUNCE(PC,  BIT6 );

                	GPIO_SetMode(PC, BIT7, GPIO_MODE_INPUT);
                	    	GPIO_EnableInt(PC, 7, GPIO_INT_BOTH_EDGE  );

                	    	GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);


    /*********************************************
      * WIRING FOR ON BOARD 2003 STEPPER MOTOR
      *
      * *******************************************/
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO | SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB7MFP_GPIO;


      GPIO_SetMode(PB, BIT6, GPIO_MODE_QUASI);
      GPIO_SetMode(PB, BIT7, GPIO_MODE_QUASI);
      GPIO_SetMode(PB, BIT4, GPIO_MODE_QUASI);
      GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI);

      GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);
      GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);

	/*--------------------------ADC PIN INTIALIZATION--------------------------*/
	/* Enable ADC module clock */
	CLK_EnableModuleClock(ADC_MODULE);

	/* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
	CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADCSEL_HIRC, CLK_CLKDIV0_ADC(7));

	/* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
	GPIO_DISABLE_DIGITAL_PATH(PB, 0xF);

	/* Configure the GPB0 - GPB3 ADC analog input pins (IFB_ADC, VFB_ADC, Volt_Read, Current_Read) */
	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
	SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1 | SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
	/* Enable PWM0 and PWM1 module clock */
	CLK_EnableModuleClock(PWM0_MODULE);

	/* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_PCLK0, NULL);
//    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL1_PWM1SEL_PCLK1, NULL);

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
 //   CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_PLL, NULL);

    /*---------------------------------------------------------------------------------------------------------*/
	//PWM PWM0_CH2 PE.2

    IGNITION_OFF;
     LIGHT_OFF;

    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE2MFP_Msk);
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE2MFP_GPIO);

    GPIO_SetMode(PE, BIT2, GPIO_MODE_OUTPUT);

    PE2 = 1;
    PE2 = 0;
    PE2 = 1;
    PE2 = 0;

    SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE10MFP_Msk);
    SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE10MFP_GPIO);

       GPIO_SetMode(PE, BIT10, GPIO_MODE_OUTPUT);

       solenoid_D;
       solenoid_E;
       solenoid_D;
       solenoid_E;
       solenoid_D;

       solenoid_D;
       solenoid_E;
       solenoid_D;
       solenoid_E;
       solenoid_D;



#endif
}

void GPCDEF_IRQHandler(void)
    {

    //LOCK
    	if(GPIO_GET_INT_FLAG(PC, BIT6))
    	{
    		GPIO_CLR_INT_FLAG(PC, BIT6);
    		lockTest = 1;
    	    LStat = 5;
    	}
    	//UNLOCK
    	if(GPIO_GET_INT_FLAG(PC, BIT7))
    		{
    			GPIO_CLR_INT_FLAG(PC, BIT7);
    			lockTest = 1;
    		    LStat = 9;
    		}

    }




int main()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            {
    int8_t ch;
    int8_t array[4];
    char *tString;

    /* Unlock protected registers */
    SYS_UnlockReg();


    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    DISABLE_A;
    DISABLE_B;

    PE0 = 1;  //RST gsm pin

    RELAYPOWER_ON;

    IGNITION_ON;
    IGNITION_OFF;
    IGNITION_ON;
    IGNITION_OFF;
    LIGHT_OFF;

    lockHist = LOCK_BOOT_UP;

    LIGHT_ON;

    LIGHT_OFF;
    LIGHT_ON;
    LIGHT_OFF;

    IGNITION_ON;
    LIGHT_ON;
    IGNITION_OFF;
    LIGHT_OFF;

    RELAYPOWER_OFF;

    PE0 = 0;  //RST gsm pin

    IGNITION_ON;
    LIGHT_ON;
    LIGHT_OFF;
    IGNITION_OFF;

    LIGHT_ON;
    IGNITION_ON;
    IGNITION_OFF;
    LIGHT_OFF;

    BEEPER_ON;
    BEEPER_OFF;
    BEEPER_ON;
    BEEPER_OFF;

    while(delayCounter)
    {
    	delayCounter--;
    	RELAYPOWER_ON;
    }

    /* Init UART2 to 9600-8n1 for print message */

    UART2_Init();
    UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk ));


    UART_Write_main(UART2, "EDEL SMART DEVICES - 001\r\n",26 );


    /* Init UART1 to 9600-8n1 for gsm message */

    UART1_Init();

    UART_Write(UART1, "AT\r\n",4 );
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk ));

#ifdef GPS_UART_ENABLE
    /* Init UART0 to 9600-8n1 for GPS */

    UART0_Init();

    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk ));
#endif

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 10);
    TIMER_EnableInt(TIMER0);

      /* Enable Timer0 ~ Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /*GPIO ENABLE */
         NVIC_EnableIRQ(GPCDEF_IRQn);

    /* Start Timer0 ~ Timer3 counting */
    TIMER_Start(TIMER0);

    ADC_init();
    UART_Write_main(UART2, "EDEL SMART DEVICES - 002\r\n",26 );


    loopCount = 0;



#ifdef NEW_STEPPER_MOTOR

    delayTempCount = 7000;

#endif


    while(1)
    {


    	if(configMod)
    	{
    		configMod=0;

    	      if(strstr(BATTERY,"36"))
             	{

               //   peakbatt=(maxcharge100)*3;

                  peakbatt=42;

                  lowbatt=(mincharge20)*3;


               	}
    	      else if(strstr(BATTERY,"12"))
    	    		  {
    	    	          peakbatt=maxcharge100;

    	    	          lowbatt=mincharge20;
    	    		  }
    	       if((strstr(bpon,"on"))||(strstr(bpon,"ON")))
    	      {
    	    	  BEEPON=1;

    	      }
    	      else if((strstr(bpon,"off"))||(strstr(bpon,"OFF")))
    	         	      {
    	         	    	  BEEPON=0;

    	         	      }
    	}

         	while(beepn==1)
         	{
    		// alram();
    		 beepn=0;
    	    }
    	while(light)
    	{

    	  strcpy(comand, UART2_RX_BUF);
    		memset(UART2_RX_BUF,0,BUF_LEN_LONG);
    	   count_2=0;


    	                                if(strstr(comand,"1"))
    	        	        			{
    	        	        				UART_Write(UART2, "light ON", 8);
    	        	        				 LIGHT_ON;
    	        	        				 light=0;

    	        	        			}
    	        	        			else if(strstr(comand,"2"))
    	        	        			{
    	        	        			UART_Write(UART2, "light off", 9);
    	        	        			 LIGHT_OFF;
    	        	        			 light=0;
    	        	        			}
    	}

    	while(configReadBack)
    	{
    		read_config_data();
    		configReadBack = 0;
    		UART_Write(UART2,"read back from main\r\n",22);
    		UART_Write(UART2,cmdReadBk,strlen(cmdReadBk));
    		UART_Write(UART2,"\r\n",2);
    		parse_config_data();
    		configMod=1;

    	}
    	 while(lockTest == 1)
    	 {
    		 solenoid_E;
    	    if(LStat == 9)
    	    {
    	    	ENABLE_A;
    	    	ENABLE_B;
    	    	LStat = 0;
    	    	if(devLock)
    	    		lock_bike(); //temp
    	    	else
    	    	{
    	    	//	BEEPER_ON;
    	    		beep_on();
    	    		delayCounter = 2000000;
    	    		delay_loop();
    	    		BEEPER_OFF;
    	    		delayCounter = 2000000;
    	    		delay_loop();
    	    	//	BEEPER_ON;
    	    		beep_on();
					delayCounter = 2000000;
					delay_loop();
					BEEPER_OFF;

    	    	}


    	    //	IGNITION_OFF;
    	    	IGNITION_ON;
    	    //	LIGHT_ON;
 //   	    	LStat = 9; // to be removed
    	    	lockTest = 0;// to be 0
    	    	solenoid_D;
    	    	DISABLE_A;
    	    	DISABLE_B;
    	    }
    	    else if(LStat == 5)
    	    {
    	    	ENABLE_A;
    	    	ENABLE_B;
    	    	LStat = 0;
    	    	if(devLock)
    	    		unlock_bike(); //temp lock_bike
    	    	else
				{
				//	BEEPER_ON;
    	    		beep_on();
					delayCounter = 400000;
					delay_loop();
					BEEPER_OFF;
				}

    	    	//lock_bike();
    	    //	IGNITION_ON;
    	    	IGNITION_OFF;
    	    	LIGHT_OFF;
 //   	    	LStat = 5;
    	    	lockTest = 0;
    	    	solenoid_D;
       	    	DISABLE_A;
        	    DISABLE_B;
    	    }
    	 }

    	 loopCounter++;

    	 itoa(loopCounter, loopStr, 10);

    	 memset(loopStr,0,10);

    	 if(configMode)
    	 {
    		 configMode_app();


    	 }

    	 if(batery>30)
    	    	{

    	         	batery=0;

    	         	ADC_measure();

    	         	 battery[i]=extBatt_V;

    	                ++i;
    	         	}
    	         	 if(i>=10)
    	         	 {
    	         		 i=0;
    	         	       uint32_t sum=0;
    	         	         		for(int i=0;i<10;i++)
    	         	         		{
    	         	         			 sum=sum+battery[i];
    	         	         		}
    	         	         		         average=sum/10;

    	         	              	 average=average*battADC_LC;


    	         	         		memset(battery,0,sizeof(battery));

    	         	         		pextBat_Data(average);
    	         	 }



        if(timeOut)
         {
        	timeOut = 0;



#ifdef GPS_UART_ENABLE
        	if(newGpsData > 10)
        	{
        		UART_Write_main(UART2,"\r\n",2);
        		UART_Write_main(UART2,"NEW GPS DATA\r\n",14);
        		UART_Write_main(UART2,GPS_data, strlen(GPS_data));
        		UART_Write_main(UART2,"\r\n\r\n",4);
        		get_gps_data();
        		memset(GPS_data,0,100);
        		newGpsData = 0;

        		// also measure battery voltage
          		ADC_measure();
        		intBattV_conv = intBatt_V *battint;     // battADC_LC;
        		extBattV_conv = extBatt_V * battADC_LC;

        		memset(intBatt_Data,0,API_BUFF_LEN);
        		memset(extBatt_Data,0,API_BUFF_LEN);

        		ftoa(intBattV_conv, intBatt_Data, 3);
        		ftoa(extBattV_conv, extBatt_Data, 3);

        		UART_Write_main(UART2,"INT BATT:",10);
        		UART_Write_main(UART2,intBatt_Data, strlen(intBatt_Data));
        		UART_Write_main(UART2," V\r\n",4);
        		UART_Write_main(UART2,"EXT BATT:",10);
        		UART_Write_main(UART2,extBatt_Data, strlen(extBatt_Data));
        		UART_Write_main(UART2," V\r\n",4);
        	}


#endif

        	if(countloop >50)
        	{
        		delayCounter=10000;

        		 PE0 = 1;  //RST gsm pin

        		 while(delayCounter)
        		    {
        		    	delayCounter--;

        		    }
        		 PE0 = 0;  //RST gsm pin

//        		 delayCounter=20000;
//
//        		 while(delayCounter)
//        		        		    {
//        		        		    	delayCounter--;
//
//        		        		    }
//
//        		 gsmState = GSM_INIT;
//
//        		 loopCount=0;

        	}

			switch(gsmState)
			{

				case GSM_INIT:
					gsm_init();
					countloop=0;
					break;

				case GSM_SNAPSHOT:
					gsm_get_snapshot();
					countloop=0;
					break;

				case GSM_DATACALL:
					gsm_datacall();
					countloop=0;
					break;

				case GSM_CONNECT:
					gsm_connect();
					countloop=0;
					break;

				case GSM_QUERY:
					gsm_query();
					countloop=0;
					break;

				case GSM_HTTP_REPORT_DATA:
					gsm_http_report_data();
					countloop=0;
					break;

				case GSM_HTTP_GET_LOCK_STATUS:
					gsm_http_get_lock_status();
					countloop=0;
					break;

				case GSM_HTTP_SEND_LOCK:
					gsm_http_send_lock_status();
					countloop=0;
					break;

				case GSM_HTTP_SEND_UNLOCK:
					gsm_http_send_unlock_status();
					countloop=0;
					break;

				case GSM_HTTP_SEND_REG:
					gsm_http_send_registartion();
					countloop=0;
					break;

				case GSM_HTTP_SEND_LIGHTON:
					 gsm_http_send_lightON_status();
					 countloop=0;
					 break;

				case GSM_HTTP_SEND_LIGHTOFF:
				     gsm_http_send_lightOff_status();
				     countloop=0;
				     break;

				case GSM_HTTP_SEND_BEEPON:
					gsm_http_send_beepOn_status();
					countloop=0;
					break;

				case GSM_HTTP_SEND_BEEPOff:
					gsm_http_send_beepOff_status();
					countloop=0;
					break;

				case GSM_SLEEP:
					gsm_idle();
					countloop=0;
					break;

				default:
					break;
			}
         }
    }

}


/**
 * @brief       gsm_init
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Initialise GSM.
 */
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
		break;
	case 6:
		gsmState = GSM_SNAPSHOT;
		loopCount = 0;
		break;

	default:
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
		break;
	case 3:
		UART_Write(UART1, "AT+CIMI\r\n",9 );
		loop_delay(1);
		break;
	case 4:
		readIMEI = 1;
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
		UART_Write(UART1, "AT+CGPADDR= 1\r\n", 14);
		break;
	case 10:
		break;
	case 11:
		break;
	case 12:
		gsmState = GSM_CONNECT;
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
	//	loop_delay(2);
		break;
	case 2:
		UART_Write(UART1, "AT+NETOPEN?\r\n",15);
		break;

	case 3:
		UART_Write(UART1, "AT+IPADDR\r\n", 12);
		break;
	case 4:
		UART_Write(UART1,"AT+NETOPEN\r\n",15);
		break;
	case 5:
		UART_Write(UART1, "AT+IPADDR\r\n", 12);
		break;
	case 6:

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
		UART_Write(UART1, "AT+IPADDR\r\n", 12);// close
	//	loop_delay(1);
		break;
	case 2:
		UART_Write(UART1, "AT+NETOPEN\r\n", 15);// query check result
	//	loop_delay(1);
		break;
	case 3:
		 UART_Write(UART1, "AT+IPADDR\r\n", 12);
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
		if(dataUpdateReady)
		{
			dataUpdateReady = 0;
			UART_Write(UART1,testURL,strlen(testURL));
			UART_Write(UART1, DBQUOTE,1);
			TIMER_Delay(TIMER1, 100);
		}

		UART_Write(UART1, "\r\n",2);
		break;
	case 4:

		break;
	case 5:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 18);

	//	loop_delay(1);
		break;
	case 6:
		UART_Write(UART1, "AT+HTTPHEAD=?\r\n",16);
    	//loop_delay(1);
    	break;
	case 7:
		UART_Write(UART1, "AT+HTTPHEAD\r\n",15);
	//	loop_delay(5);
		break;
	case 8:
		UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
	//	loop_delay(5);
		break;
 	case 9:
		UART_Write(UART1, "AT+HTTPREAD?\r\n",14 );
    //	loop_delay(5);
		break;
	case 10:
		UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );
	//	loop_delay(1);
		break;
	case 11:
		UART_Write(UART1, "AT+HTTPTERM\r\n",14 );
//		loop_delay(5);
		break;
	case 12:
		break;
	case 13:
		gsmState = GSM_HTTP_GET_LOCK_STATUS;
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
void gsm_http_get_lock_status()
{
	char *srcString;
	char lockStat[25];
	char lock[3];

	switch (loopCount)
	{
	case 1:
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
#ifdef EVEGAH_BETA
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"getDeviceInstructions?",strlen("getDeviceInstructions?"));
#endif
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
	//	BEEPER_ON;
		beep_on();
		delayCounter = 1500000;
		delay_loop();
		BEEPER_OFF;
	//	BEEPER_ON;
		beep_on();
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
				memset(lockStat,0,23);
				memcpy(lockStat,srcString + 17, 42);

			}
			srcString = 0;


			if((strstr(lockStat,"devceUnlok"))||(strstr(lockStat,"eviceUnock"))||(strstr(lockStat,"devceUnloc"))||(strstr(lockStat,"deiceUnlok"))||(strstr(lockStat,"dviceUnlck"))||(strstr(lockStat,"dviceUnock"))||(strstr(lockStat,"deiceUnlck"))||(strstr(lockStat,"deviceUnlck"))||(strstr(lockStat,"dviceUnlck"))||(strstr(lockStat,"deviceU"))||(strstr(lockStat,"devicL"))|| (strstr(lockStat,"devicen")) || (strstr(lockStat,"deviceUnock"))||(strstr(lockStat,"devicU")))
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
				gsmState =  GSM_HTTP_SEND_UNLOCK;
				loopCount = 0;
				loop_delay(1);

			}
			else if((strstr(lockStat,"devceLock"))||(strstr(lockStat,"eviceLok"))||(strstr(lockStat,"dviceLok"))||(strstr(lockStat,"deiceLoc"))||(strstr(lockStat,"deviceLoc"))||(strstr(lockStat,"dviceLoc"))||(strstr(lockStat,"deviceLck"))||(strstr(lockStat,"deviceL")) ||(strstr(lockStat,"deviceo")) ||(strstr(lockStat,"deviceLok"))|| (strstr(lockStat,"devicL")))

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
			else if((strstr(lockStat,"deviceR"))||(strstr(lockStat,"deiceRegitration")))

			{
				gsmState = GSM_HTTP_SEND_REG;
				loopCount = 0;
				loop_delay(5);
			}
#ifdef LIGHT


			           else if((strstr(lockStat,"LighOn"))||(strstr(lockStat,"LigtOn"))||(strstr(lockStat,"LihtOn"))||(strstr(lockStat,"Lightn"))||(strstr(lockStat,"LghtOn"))||(strstr(lockStat,"LightOn"))||(strstr(lockStat,"ightOn")))
			           {
							 LIGHT_ON;
							 gsmState=GSM_HTTP_SEND_LIGHTON;
							 loopCount = 0;
				             loop_delay(1);
						}
						else if((strstr(lockStat,"LighOff"))||(strstr(lockStat,"LihtOff"))||(strstr(lockStat,"LigtOff"))||(strstr(lockStat,"Lightff"))||(strstr(lockStat,"LightOf"))||(strstr(lockStat,"LghtOff"))||(strstr(lockStat,"LightOff"))||(strstr(lockStat,"ightOff")))
						{
							 LIGHT_OFF;
							 gsmState=GSM_HTTP_SEND_LIGHTOFF;
							 loopCount = 0;
					         loop_delay(1);

						}
						else if((strstr(lockStat,"BeepOn"))||(strstr(lockStat,"Beepn"))||(strstr(lockStat,"eepOn"))||(strstr(lockStat,"BeepOn"))||(strstr(lockStat,"BepOn"))||(strstr(lockStat,"BeeOn")))
						{
							alram();
							 gsmState=GSM_HTTP_SEND_BEEPON;
							 loopCount = 0;
			                 loop_delay(1);
						}
						else if((strstr(lockStat,"Beepff"))||(strstr(lockStat,"BeepOff"))||(strstr(lockStat,"BeeOff"))||(strstr(lockStat,"eepOff"))||(strstr(lockStat,"BepOff"))||(strstr(lockStat,"BeepOf")))
						{
							alram();
							 gsmState=GSM_HTTP_SEND_BEEPOff;
							 loopCount = 0;
		                     loop_delay(1);

						 }
#endif


			else
			{
				deviceStatus = 0;
				gsmState = GSM_SLEEP;
				loopCount = 0;
//				loop_delay(10);
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

	char lockStat[12];

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
#ifdef EVEGAH_BETA
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"lockDevice?",strlen("lockDevice?"));
#endif
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1,"&lockStatus=2",strlen("&lockStatus=2"));
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
		UART_Write(UART1, "AT+HTTPREAD\r\n",13 );
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

void gsm_http_send_unlock_status()
{

	char lockStat[12];

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
#ifdef EVEGAH_BETA
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"unlockDevice?",strlen("unlockDevice?"));
#endif
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1,"&lockStatus=1",strlen("&lockStatus=1"));
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
    	UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
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
#ifdef EVEGAH_BETA
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"DeviceLightOn?",strlen("DeviceLightOn?"));
#endif
		UART_Write(UART1,devID,strlen(devID));
	//	UART_Write(UART1,"&lightStatus=4",strlen("&lightkStatus=4"));
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
//		loop_delay(5);
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
void gsm_http_send_lightOff_status()
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
#ifdef EVEGAH_BETA
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"DeviceLightOff?",strlen("DeviceLightOff?"));
#endif
		UART_Write(UART1,devID,strlen(devID));
		//UART_Write(UART1,"&DeviceLight=5",strlen("&DeviceLight=5"));
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
	   UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
	 //  loop_delay(5);
	   break;
	case 8:
		UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
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
#ifdef EVEGAH_BETA
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"setBeepOn?",strlen("setBeepOn?"));
#endif

		UART_Write(UART1,devID,strlen(devID));
	//	UART_Write(UART1,"&beepStatus=7",strlen("&beepkStatus=7"));
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
#ifdef EVEGAH_BETA
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"setBeepOff?",strlen("setBeepOff?"));
#endif
		UART_Write(UART1,devID,strlen(devID));
		//UART_Write(UART1,"&beepStatus=6",strlen("&beepkStatus=6"));
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
		UART_Write(UART1, "AT+HTTPHEAD\r\n",13);
//		 loop_delay(5);
	   break;
	case 8:
		UART_Write(UART1, "AT+HTTPREAD=?\r\n",15);
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


void gsm_http_send_registartion()
{

	char lockStat[12];

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
#ifdef EVEGAH_BETA
		UART_Write(UART1,webURL,strlen(webURL));
		UART_Write(UART1,"dR?",strlen("dR?"));
		UART_Write(UART1,devID,strlen(devID));
		UART_Write(UART1,"&imeiN=",strlen("&imeiN="));
		UART_Write(UART1,IMEI,strlen(IMEI));
//		UART_Write(UART1,"&rN=MP09PW1209&sN=00099009091&cN=767787&dOM=07/08/2022",71);
#endif
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
/**
 * @brief       gsm_idle
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Idle.
 */
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

void rotate_unlock()
{
	stepper_motor_drv(stepCW[stepCount]);
	if(stepCount >= stepperNoStep )
		stepCount = 0;
	else
		stepCount++;
}

void rotate_lock()
{
	stepper_motor_drv(stepCCW[stepCount]);
	if(stepCount >= stepperNoStep )
		stepCount = 0;
	else
		stepCount++;

}

void unlock_bike()
{
	uint16_t lockCounter = 0;


	beep_on_off(100,70,10);

	turnCount =TURN_COUNT;

	for(lockCounter = 0; lockCounter < turnCount; lockCounter++)
	{
		while(counter <=  turnCount)
		{
			counter++;
			beep_control();
		}
		if(counter > turnCount)
		{
//			rotate_unlock();
			rotate_lock();
			counter = 0;
		}
	}
	loopCount = 0;
	lockTest = 0;
}


void lock_bike()
{
	uint16_t lockCounter = 0;


	beep_on_off(40,50,10);

	turnCount = TURN_COUNT;

	for(lockCounter = 0; lockCounter < turnCount; lockCounter++)
	{
		while(counter <=  turnCount)
		{
			counter++;
			beep_control();
		}
		if(counter > turnCount)
		{
//			rotate_lock();
			rotate_unlock();
			counter = 0;
		}
	}
	loopCount = 0;
	lockTest = 0;
}


	void alram()
	{
	uint16_t lockCounter = 0;


		beep_on_off(100,100,10);

		turnCount = TURN_COUNT;

		for(lockCounter = 0; lockCounter < turnCount; lockCounter++)
		{
			while(counter <=  turnCount)
			{
				counter++;
				beep_control();
			}
			counter=0;
		}
		loopCount = 0;
		beepn=0;

}
void Alaram(uint32_t ONTIME,uint32_t OFFTIME,uint32_t TIMES)   // Beep Customize Function
	{
	 for(int i=1;i<=TIMES;i++)
	 {
		 beep_on();
		// TIMER_Delay(TIMER2,ONTIME);
		 delayCounter = 150000;
		 		 		delay_loop();
		 beep_off();
		 TIMER_Delay(TIMER2,OFFTIME);
		  }



	}






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
void get_gps_data()
{


	UART_Write_main(UART2,"\r\n", 2);


	memset(lat_deg,0,2);
	memset(lat_min,0,2);

	memset(LAT_min,0,10);

	memset(minLat,0,2);
	memset(minLong,0,2);


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
		//	UART_Write_main(UART2, Speed, 3);
		memset(gpsSpeedData,0,API_BUFF_LEN);

		strncpy(gpsSpeedData,Sped, cog - Speed - 1);
	//	strncpy(gpsSpeedData,Speed+1, cog - Speed - 1);

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

		strncpy(testURL,webURL,strlen(webURL));

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

         if(knotspeed<26)
           {
		       strcat(testURL,gpsSpeedData);

             }

	    strcat(testURL,devID);

#ifdef BATT_VOLTAGE_MEAS
		strcat(testURL,battVExt);
		strcat(testURL,extBatt_Data);
		strcat(testURL,battVInt);
		strcat(testURL,intBatt_Data);
		strcat(testURL,pbattVExt);
		strcat(testURL,str);
#endif
		dataUpdateReady = 1;

		UART_Write_main(UART2,"\r\n\r\n", 4);
		UART_Write_main(UART2,"/******************/\r\n", 22);
		UART_Write_main(UART2,testURL,strlen(testURL));
		UART_Write_main(UART2,"\r\n", 2);
		UART_Write_main(UART2,"/******************/\r\n", 22);
		UART_Write_main(UART2,"\r\n\r\n:", 4);
   	//	BEEPER_ON;
		beep_on();
    	delayCounter = 20000;
    	delay_loop();
    	BEEPER_OFF;
	}
	else
	{
		UART_Write_main(UART2,"/******************/\r\n", 22);
		UART_Write_main(UART2,"GPS - is looking for Satellites",32);
		UART_Write_main(UART2,"\r\n", 2);
		UART_Write_main(UART2,"/******************/\r\n", 22);
		dataUpdateReady = 0;
//		memset(testURL,0,BUF_LEN);
//		strncpy(testURL,webURL,strlen(webURL));
//		strcat(testURL,api_update);
//		strcat(testURL,gpsLatitude);
//	//	strcat(testURL,"0.0");
//		strcat(testURL,gpsLongitude);
//	//	strcat(testURL,"0.0");
//		strcat(testURL,gpsSpeed);
//	//	strcat(testURL,"0.0");
//		strcat(testURL,devID);
#ifdef BATT_VOLTAGE_MEAS
//		strcat(testURL,battVExt);
//		strcat(testURL,extBatt_Data);
//		strcat(testURL,battVInt);
//		strcat(testURL,intBatt_Data);
//		strcat(testURL,pbattVExt);
//		strcat(testURL,str);
#endif
	//	dataUpdateReady = 1;


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
                 	memset(str,48,2);
               //ascii zero value is 48 and two zeros send the server
            	//    memset(str,48,2);

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


void beep_on()
{
	if(BEEPON)
	{

	    PE2 = 1;
//		PWM_EnableOutput(PWM0, 0x04);
//		PWM_Start(PWM0, 0x04);
//		beepDelay = 0;
	}
}

void beep_off()
{
	PE2 = 0;
//		PWM_DisableOutput(PWM0, 0x04);
 //		beepDelay = 0;
}

void beep_on_off(uint8_t onTime, uint8_t offTime, uint16_t beepCount)
{
	beepOn = TRUE;
	beepOnTime = onTime * 100;
	beepOffTime = offTime * 100;
	beepCounter = beepCount;

}

void beep_control()
{

	if(beepOn)
	{
		if(beepDelay == 0)
		{
			//beep on
			beep_on();
			beepDelay++;
		}
		else if((beepDelay >= beepOnTime) && (beepDelay < (beepOnTime +beepOffTime )))
		{
			//beepoff
			beep_off();
			beepDelay++;
		}
		else if(beepDelay >= (beepOnTime + beepOffTime))
		{
			//beepCount inc
			beepDelay = 0;
			noOfBeeps++;
//    			beepDelay++;
			if(noOfBeeps > beepCounter)
			{
				beepOn = FALSE;
				noOfBeeps = 0;
				//beepOff
				beep_off();
				beepDelay = 0;
			}
		}
		else
		{
			beepDelay++;
		}

	}
}



void delay_loop()
{
    while(delayCounter)
    {
    	delayCounter--;
    }

}

void delay_loop_2()
{
	delayCounter = delayTempCount;
	delay_loop();
}

uint8_t delayDenom = 4;
void delay_loop_4()
{
	delayCounter = delayTempCount/delayDenom;
	delay_loop();
}

/* check if config mode is on, if configMode is ON, do not write*/
uint32_t UART_Write_main(UART_T* uart, uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
	uint32_t u32Count = 0;

	if(!configMode)
		u32Count = UART_Write(uart, pu8TxBuf, u32WriteBytes);

	return u32Count;

}

#ifdef NEW_STEPPER_MOTOR

uint8_t fullStop =0;
uint8_t coil_1;
uint8_t coil_2;

void stepper_motor_drv(uint8_t coil )
{
	coil_1 = (coil & 0xF0) >> 4;
	coil_2 = (coil & 0x0F);

	switch(coil_1)
	{
	case 0:
		OFF_WIND_1;
		break;
	case 1:
		ON_WIND_1_CW;
		break;
	case 2:
		ON_WIND_1_CCW;
		break;
	default:
		OFF_WIND_1;
		break;
	}

	switch(coil_2)
	{
	case 0:
		OFF_WIND_2;
		break;
	case 1:
		ON_WIND_2_CW;
		break;
	case 2:
		ON_WIND_2_CCW;
		break;
	default:
		OFF_WIND_2;
		break;
	}

	delay_loop_2();

	if(fullStop)
	{
		OFF_WIND_1;
		OFF_WIND_2;
		delay_loop_4();
	}

}
#endif
