/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/10/17 3:06p $

 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "M0564.h"
#include <string.h>
#include "ADC.h"
#include "gpio.h"
#include "config.h"

#define EDEL_TRACKER_BRD_2 1

uint8_t configMode = 0;

/*---------------UART0------------------*/
uint8_t UART0_RX_BUF[BUF_LEN];
uint8_t UART0_TX_BUF[BUF_LEN];
uint8_t count_0;
uint8_t newGpsData;
uint8_t GPS_data;
/*--------------UART1-------------------*/
uint8_t UART1_RX_BUF[BUF_LEN];
uint8_t UART1_TX_BUF[BUF_LEN];
uint8_t count_1;
uint8_t newWord_1 = 0;

/*--------------UART2-------------------*/
uint8_t UART2_RX_BUF[BUF_LEN_LONG];
uint8_t UART2_TX_BUF[BUF_LEN];
uint16_t count_2;

/*--------------------URL--------------------*/

extern uint8_t WLTURL[100]; //= "http://data.gpsiot.net:8937/post?";
uint8_t URLLENGTH[BUF_LEN_SMALL] = "33,80";
uint8_t WLTIME[BUF_LEN_SMALL] = "imei=";
uint8_t WLTLAT[BUF_LEN_SMALL] = "&lat=";
uint8_t WLTLON[BUF_LEN_SMALL] = "&lon=";
uint8_t WLTHEADING[BUF_LEN_SMALL] = "&heading=";
uint8_t WLTSPEED[BUF_LEN_SMALL] = "&speed=";
uint8_t WLTODOMETER[BUF_LEN_SMALL] = "&odometer=";
uint8_t WLTIGNITION[BUF_LEN_SMALL] = "&ignition=";
uint8_t WLTGPSDATE[BUF_LEN_SMALL] = "&gpsdate=";
uint8_t WLTEVENTID[BUF_LEN_SMALL] = "&eventid=";
uint8_t WLTALTITUDE[BUF_LEN_SMALL] = "&altitude=";

uint8_t WURL[100]="http://evegah.kritin.in/api/getDeviceInstructions?&dId=EMI2309";

uint8_t readIMEI = 0;
uint8_t readGNSS=0;

/*GPS DATA*/
/*uint8_t *gpsData;
uint8_t *endofStr;*/
uint8_t *rtc;

uint8_t gnssutc[10];
uint8_t gnsslat[8];
uint8_t gnsslon[8];
uint8_t gnsshdop[3];
uint8_t gnssalt[5];
uint8_t gnssfix[1];
uint8_t gnsscog[6];
uint8_t gnssspkm[3];
uint8_t gnssspkn[3];
uint8_t gnssdate[6];
uint8_t gnssnsat[2];

uint8_t gnssutctime[3];
uint8_t gnssutcmin[3];
uint8_t gnssutcsec[3];

uint8_t gnssyear[3];
uint8_t gnssmon[3];
uint8_t gnssday[3];

uint8_t DITION=0;
uint8_t GNSS=0;
uint32_t urlsize;
uint8_t wlturlsize[5];


uint8_t gnss[BUF_LEN_SMALL];

uint8_t IME[BUF_LEN_SMALL];

uint8_t devID[BUF_LEN_SMALL];

uint8_t GNSS_data[100];

uint8_t URL[150];


uint32_t delayCounter = 200000;

void GET_GNSS_data();
void WLT_SEND_DATA();

volatile uint32_t g_au32TMRINTCount[4] = { 0 };
uint8_t loopCount = 0;
uint8_t tloopCount = 0;
uint8_t loopDelay = 0;
uint8_t timeOut = 0;

/* special char */
char DBQUOTE[1] = { 0x22 };

void UART2_TEST_HANDLE();
void UART1_TEST_HANDLE();

void loop_delay(uint8_t loopDel);
void gsm_init();
void gsm_get_snapshot();
void gsm_datacall();
void gsm_connect();
void gps_query();
void gsm_http_get();
void gsm_idle();

void get_gps_data();

int intmyatoi(char*);

void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);

uint32_t UART_Write_main(UART_T *uart, uint8_t *pu8TxBuf,
		uint32_t u32WriteBytes);

enum {
	GSM_INIT = 1,
	GSM_SNAPSHOT,
	GSM_DATACALL,
	GSM_CONNECT,
	GNSS_QUERY,
	GSM_SLEEP,

} GSM_SM_T;

uint8_t gsmState = GSM_INIT;

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void) {
	UART2_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_TEST_HANDLE() {
	uint8_t u8InChar = 0xFF;
	uint32_t u32IntSts = UART2->INTSTS;

	if (u32IntSts & UART_INTSTS_RDAINT_Msk) {

		/* Get all the input characters */
		while (UART_IS_RX_READY(UART2)) {
			UART_Read(UART2, &UART2_RX_BUF[count_2], 1);

			if ((count_2 > 2) && (configMode == 0)) {
				if ((UART2_RX_BUF[count_2] == '\r')
						&& (UART2_RX_BUF[count_2 - 1] == '+')
						&& (UART2_RX_BUF[count_2 - 2] == '+')
						&& (UART2_RX_BUF[count_2 - 3] == '+')) {
					configMode = 1;
					UART_Write(UART2, "Entered Config Mode\r\n", 21);
				}

			}

			count_2++;
			if (count_2 > BUF_LEN_LONG)
				count_2 = 0;

		}

	}
	UART2->INTSTS = 0;

	u32IntSts = UART0->INTSTS;

	if (u32IntSts & UART_INTSTS_RDAINT_Msk) {

		/* Get all the input characters */
		while (UART_IS_RX_READY(UART0)) {
			UART_Read(UART0, &UART0_RX_BUF[count_0], 1);

			UART_Write_main(UART2, &UART0_RX_BUF[count_0], 1);

			count_0++;
			//            if(count_0 > BUF_LEN)
//             {
//            	 count_0 = 0;
//            	 // extract $GPRMC
//            	 tGRMC = strstr(UART0_RX_BUF,"GPRMC");
//            	 if(tGRMC)
//            	 {
//            		 tEndofRMC = strstr(tGRMC,"$G");
//            		 if((tEndofRMC > tGRMC) && (tEndofRMC < UART0_RX_BUF + BUF_LEN))
//            		 {
//            			 strncpy(GPS_data, tGRMC, tEndofRMC - tGRMC);
//            			 newGpsData++;
//            		 }
//            	 }
//            	 else
////            	 {
////            		 tGRMC = strstr(UART0_RX_BUF,"GNRMC");
////					 if(tGRMC)
////					 {
////						 tEndofRMC = strstr(tGRMC,"$G");
////						 if((tEndofRMC > tGRMC) && (tEndofRMC < UART0_RX_BUF + BUF_LEN))
////						 {
////							 strncpy(GPS_data, tGRMC, tEndofRMC - tGRMC);
////							 newGpsData++;
////						 }
////					 }
////
////            	 }
//
//
//             }

		}

	}
	UART0->INTSTS = 0;

}

/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void) {
	TIMER_DisableInt(TIMER0);
	UART1_TEST_HANDLE();
	TIMER_EnableInt(TIMER0);
}
uint8_t noByt;
/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE() {
	uint8_t u8InChar = 0xFF;
	uint32_t u32IntSts = UART1->INTSTS;
	char *srcStr;
	char *cr;

	if (u32IntSts & UART_INTSTS_RDAINT_Msk) {

		/* Get all the input characters */

		while (UART_IS_RX_READY(UART1))
		{
			UART_Read(UART1, &UART1_RX_BUF[count_1], 1);

			UART_Write_main(UART2, &UART1_RX_BUF[count_1], 1);

//			if(readGNSS)
//			{
//				memset(GNSS_data,0,75);
//				memset(URL,0,BUF_LEN);
//				readGNSS=0;
//				cr=strstr(&UART1_RX_BUF,"+QGPSLOC:");
//				strncpy(GNSS_data,cr,75);
//				GNSS=1;
////				GET_GNSS_data();
//			}

			if (readIMEI)
			{
				readIMEI = 0;
				UART_Write_main(UART2, "\r\n", 2);
				srcStr = strstr(&UART1_RX_BUF[count_1 - 35], "CGSN");
				memcpy(&IME[0], srcStr+7 , 15);
				UART_Write_main(UART2, "IMEI is :", 9);
				UART_Write_main(UART2, IME, strlen(IME));
				UART_Write_main(UART2, "\r\n", 2);

			}

			if (UART1_RX_BUF[count_1] == '\r')
				newWord_1 = 1;

			count_1++;
			if (count_1 >= BUF_LEN)
				count_1 = 0;

		}

	}
	UART1->INTSTS = 0;

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
void TMR0_IRQHandler(void) {
	if (TIMER_GetIntFlag(TIMER0) == 1) {
		/* Clear Timer0 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER0);

		g_au32TMRINTCount[0]++;
		loopCount++;
		timeOut = 1;

	}
}

#ifdef EDEL_TRACKER_BRD_2
void UART0_Init() {
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init UART                                                                                               */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Reset UART0 */
	SYS->IPRST1 |= SYS_IPRST1_UART0RST_Msk;
	SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

	/* Configure UART0 and set UART0 baud rate */
	UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 9600);
	UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init() {
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init UART                                                                                               */
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
	/* Init UART                                                                                               */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Reset UART1 */
	SYS->IPRST1 |= SYS_IPRST1_UART2RST_Msk;
	SYS->IPRST1 &= ~SYS_IPRST1_UART2RST_Msk;

	/* Configure UART1 and set UART1 baud rate */
	UART2->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 9600);
	UART2->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}
#endif

void SYS_Init(void) {

#ifdef EDEL_TRACKER_BRD_2
//	/* Set PF multi-function pins for X32_OUT(PF.0) and X32_IN(PF.1) */
//	SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk))
//			| SYS_GPF_MFPL_PF0MFP_X32_OUT;
//	SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF1MFP_Msk))
//			| SYS_GPF_MFPL_PF1MFP_X32_IN;

	/*---------------------------------------------------------------------------------------------------------*/
	/* Init System Clock                                                                                       */
	/*---------------------------------------------------------------------------------------------------------*/

	/* Enable HIRC and LXT clock */
	CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LXTEN_Msk);
//
	/* Wait for HIRC and LXT clock ready */
	while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
		;
	while (!(CLK->STATUS & CLK_STATUS_LXTSTB_Msk))
		;

	/* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
	CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk))
			| CLK_CLKSEL0_HCLKSEL_HIRC;
	CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk))
			| CLK_CLKDIV0_HCLK(3);

	/* Update System Core Clock */
	SystemCoreClockUpdate();

	/* Enable UART module clock */
	CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk
			| CLK_APBCLK0_UART2CKEN_Msk);

	/* Select UART module clock source as HIRC and UART module clock divider as 1 */
	CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk))
			| CLK_CLKSEL1_UARTSEL_HIRC;
	CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk))
			| CLK_CLKDIV0_UART(1);

	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

//	CLK_EnableModuleClock(TMR2_MODULE);
//	CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HIRC, 0);
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init I/O Multi-function                                                                                 */
	/*---------------------------------------------------------------------------------------------------------*/

#endif

#ifdef EDEL_TRACKER_BRD_2
//
//	SYS->GPA_MFPL &= ~( SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
//	SYS->GPA_MFPL |= ( SYS_GPA_MFPL_PA2MFP_UART0_TXD
//			| SYS_GPA_MFPL_PA3MFP_UART0_RXD);

	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk);
	SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_UART1_TXD
			| SYS_GPA_MFPL_PA1MFP_UART1_RXD);

//       SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE12MFP_Msk | SYS_GPE_MFPH_PE13MFP_Msk);
//       SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE12MFP_UART1_TXD | SYS_GPE_MFPH_PE13MFP_UART1_RXD);

	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk);
	SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC2MFP_UART2_TXD
			| SYS_GPC_MFPL_PC3MFP_UART2_RXD);

//	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk
//			| SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
//	SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB4MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO
//			| SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB7MFP_GPIO);
//
//	SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
//	SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD2MFP_GPIO | SYS_GPD_MFPL_PD3MFP_GPIO);
//
//	SYS->GPE_MFPL &= (SYS_GPE_MFPL_PE0MFP_Msk);
//	SYS->GPE_MFPL |= SYS_GPE_MFPL_PE0MFP_GPIO;

//    SYS->GPE_MFPH &= (SYS_GPE_MFPH_PE11MFP_Msk);
//    SYS->GPE_MFPH |= SYS_GPE_MFPH_PE11MFP_GPIO;

//	/********************************/
//	/**** TEMP STEPPER MOTOR CODE ***/
//	/********************************/
//
//	SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk
//			| SYS_GPD_MFPL_PD7MFP_Msk);
//	SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD4MFP_GPIO | SYS_GPD_MFPL_PD5MFP_GPIO
//			| SYS_GPD_MFPL_PD7MFP_GPIO);
//
//	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk
//			| SYS_GPC_MFPL_PC4MFP_Msk);
//	SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_GPIO | SYS_GPC_MFPL_PC1MFP_GPIO
//			| SYS_GPC_MFPL_PC4MFP_GPIO);
//
//	GPIO_SetMode(PD, BIT4, GPIO_MODE_QUASI);
//	GPIO_SetMode(PD, BIT5, GPIO_MODE_QUASI);
//	GPIO_SetMode(PD, BIT7, GPIO_MODE_QUASI);
//	GPIO_SetMode(PC, BIT0, GPIO_MODE_QUASI);
//	GPIO_SetMode(PC, BIT1, GPIO_MODE_QUASI);

	//************************switch int***************************************//

//	SYS->GPD_MFPL &= ~(SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk);
//	SYS->GPD_MFPL |= SYS_GPC_MFPL_PC6MFP_GPIO | SYS_GPC_MFPL_PC7MFP_GPIO;
//
//	GPIO_SetMode(PC, BIT6, GPIO_MODE_INPUT);
//	GPIO_EnableInt(PC, 6, GPIO_INT_BOTH_EDGE);
//
//	GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
//	GPIO_ENABLE_DEBOUNCE(PC, BIT6);
//
//	GPIO_SetMode(PC, BIT7, GPIO_MODE_INPUT);
//	GPIO_EnableInt(PC, 7, GPIO_INT_BOTH_EDGE);
//
//	GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
//
//	/*********************************************
//	 * WIRING FOR ON BOARD 2003 STEPPER MOTOR
//	 *
//	 * *******************************************/
//	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk
//			| SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
//	SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO
//			| SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB7MFP_GPIO;
//
//	GPIO_SetMode(PB, BIT6, GPIO_MODE_QUASI);
//	GPIO_SetMode(PB, BIT7, GPIO_MODE_QUASI);
//	GPIO_SetMode(PB, BIT4, GPIO_MODE_QUASI);
//	GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI);
//
//	GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);
//	GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);

	/*


	 /*--------------------------ADC PIN INTIALIZATION--------------------------*/
	/* Enable ADC module clock */
//	CLK_EnableModuleClock(ADC_MODULE);

	/* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
//	CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADCSEL_HIRC, CLK_CLKDIV0_ADC(7));

	/* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
//	GPIO_DISABLE_DIGITAL_PATH(PB, 0xF);

	/* Configure the GPB0 - GPB3 ADC analog input pins (IFB_ADC, VFB_ADC, Volt_Read, Current_Read) */
//	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk
//			| SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
//	SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1
//			| SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3;

	/*---------------------------------------------------------------------------------------------------------*/
	/* PWM clock frequency configuration                                                                       */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Enable PWM0 and PWM1 module clock */
//	CLK_EnableModuleClock(PWM0_MODULE);

	/* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
//	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

	/* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
	/* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
//	CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_PCLK0, NULL);
//    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL1_PWM1SEL_PCLK1, NULL);

	/* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
	//CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_PLL, NULL);
	/*---------------------------------------------------------------------------------------------------------*/
	//PWM PWM0_CH2 PE.2
//	SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE2MFP_Msk);
//	SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE2MFP_GPIO);
//
//	GPIO_SetMode(PE, BIT2, GPIO_MODE_OUTPUT);
//
//	PE2 = 1;
//	PE2 = 0;
//	PE2 = 1;
//	PE2 = 0;
//
//	SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE10MFP_Msk);
//	SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE10MFP_GPIO);
//
//	GPIO_SetMode(PE, BIT10, GPIO_MODE_OUTPUT);

#endif
}
//
//void GPCDEF_IRQHandler(void) {
//
//	//LOCK
//	if (GPIO_GET_INT_FLAG(PC, BIT6)) {
//		GPIO_CLR_INT_FLAG(PC, BIT6);
//
//	}
//	//UNLOCK
//	if (GPIO_GET_INT_FLAG(PC, BIT7)) {
//		GPIO_CLR_INT_FLAG(PC, BIT7);
//
//	}
//
//}

int main() {
	int8_t ch;
	int8_t array[4];
	char *tString;

	/* Unlock protected registers */
	SYS_UnlockReg();

	SYS_Init();

	/* Lock protected registers */
	SYS_LockReg();

//	while (delayCounter) {
//		delayCounter--;
//	}

	/* Init UART2 to 9600-8n1 for print message */

	UART2_Init();
	UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk));

	UART_Write_main(UART2, "EDEL SMART DEVICES - 001\r\n", 26);

	/* Init UART1 to 9600-8n1 for gsm message */

	UART1_Init();

	UART_Write(UART1, "AT\r\n", 4);
	UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk));

	/* Init UART0 to 9600-8n1 for GPS */

	//  UART0_Init();
	//  UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk ));
	/* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 5);
	TIMER_EnableInt(TIMER0);

	/* Enable Timer0 ~ Timer3 NVIC */
	NVIC_EnableIRQ(TMR0_IRQn);

	/*GPIO ENABLE */
	//  NVIC_EnableIRQ(GPCDEF_IRQn);
	/* Start Timer0 ~ Timer3 counting */
	TIMER_Start(TIMER0);

	//  ADC_init();
	UART_Write_main(UART2, "EDEL SMART DEVICES - 002\r\n", 26);

	loopCount = 0;

	while (1) {

//	    if(GNSS)
//	    {
//	    	GNSS=0;
//		GET_GNSS_data();
//	    }
		if (timeOut)
		{
			timeOut = 0;



			switch (gsmState) {

			case GSM_INIT:
				gsm_init();
				break;

			case GSM_SNAPSHOT:
				gsm_get_snapshot();
				break;

			case GSM_DATACALL:
				gsm_datacall();
				break;

			case GSM_CONNECT:
				gsm_connect();
				break;



			case GNSS_QUERY:
				gnss_query();
				break;

			case GSM_SLEEP:
				gsm_idle();
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
void gsm_init() {
	switch (loopCount) {

	case 1:
		UART_Write(UART1, "AT\r\n", sizeof("AT\r\n"));
		break;
	case 2:
		UART_Write(UART1, "ATI\r\n", sizeof("ATI\r\n"));
		break;
	case 3:
		UART_Write(UART1, "ATE1V1\r\n", sizeof("ATE1V1\r\n"));
		break;
	case 4:
		UART_Write(UART1, "AT+CGMM\r\n", sizeof("AT+CGMM\r\n"));
		break;
	case 5:
		UART_Write(UART1, "AT+CGMI\r\n", sizeof("AT+CGMI\r\n"));
		break;
	case 6:
		break;
	case 7:
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
		loop_delay(1);
		break;
	case 3:
		readIMEI = 1;
		UART_Write(UART1, "\r\n", 2);
		UART_Write(UART1, "AT+CIMI\r\n", 9);
//		loop_delay(1);
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
		UART_Write(UART1, "AT+CSQ\r\n", sizeof("AT+CSQ\r\n"));
		break;
	case 8:
		//UART_Write(UART1, "AT+IPR=115200\r\n", sizeof("AT+IPR=115200\r\n"));
		break;
	case 9:
		UART_Write(UART1, "AT+COPS?\r\n", sizeof("AT+COPS?\r\n"));
		loop_delay(5);
		break;
	case 10:
		UART_Write(UART1, "AT+CREG?\r\n", sizeof("AT+CREG?\r\n"));
		break;
	case 11:
		UART_Write(UART1, "AT+CGACT?\r\n", sizeof("AT+CGACT?\r\n"));
		break;
	case 12:
		UART_Write(UART1, "AT+CFUN?\r\n", sizeof("AT+CFUN?\r\n"));
		break;
	case 13:
		UART_Write(UART1, "AT+CGCLASS?\r\n", sizeof("AT+CGCLASS?\r\n"));
		break;
	case 14:
		UART_Write(UART1, "AT+CSCA?\r\n", sizeof("AT+CSCA?\r\n"));
		break;
	case 15:
		UART_Write(UART1, "AT+CMGF?\r\n", sizeof("AT+CMGF?\r\n"));
		break;
	case 16:
		gsmState = GSM_DATACALL;
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
		UART_Write(UART1, "AT+CGACT?\r\n", 11);
		break;
	case 4:
		UART_Write(UART1, "AT+CMEE=1\r\n", 11);
		break;
	case 5:
		UART_Write(UART1, "AT+CGATT=1\r\n", 12);
		break;
	case 6:
		UART_Write(UART1, "AT+CGACT=1,1\r\n", 14);
		break;
	case 7:
		UART_Write(UART1, "AT+CGPADDR= 1\r\n", 14);
		break;
	case 8:
		break;
	case 9:
		break;
	case 10:
		gsmState = GSM_CONNECT;
		loopCount = 0;
		break;
	default:
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
	switch (loopCount) {

	case 1:
		UART_Write(UART1, "AT+CGACT=1,1\r\n", 14);
		break;
	case 2:
		UART_Write(UART1, "AT+QHTTPCFG=", 12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "contextid", 9);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",", 1);
		UART_Write(UART1, "1", 1);
		UART_Write(UART1, "\r\n", 2);
		break;
	case 3:
		UART_Write(UART1, "AT+QIACT=?\r\n", 14);
		break;
	case 4:
		UART_Write(UART1, "AT+QIACT=1\r\n", 14);
		break;
	case 5:

		UART_Write(UART1, "AT+QIACT?\r\n", 15);
		UART_Write(UART1, "\r\n", 2);
		loop_delay(1);

		break;
	case 6:
		UART_Write(UART1, "AT+QHTTPURL=", 12);
		UART_Write(UART1, "62", 2);
		UART_Write(UART1, ",", 1);
		UART_Write(UART1, "80", 2);
		UART_Write(UART1, "\r\n", 2);
		break;
	case 7:
		break;
	case 8:
		loop_delay(2);
		UART_Write(UART1, WURL, strlen(WURL));
		UART_Write(UART2, WURL, strlen(WURL));
		UART_Write(UART1, "\r\n", 2);
		break;
	case 9:
//		if(DITION)
//		{
//			 WLT_SEND_DATA();
//			 DITION=0;
		loop_delay(5);
		UART_Write(UART1, "AT+QHTTPGET=",12);
	//	UART_Write(UART1, wlturlsize, strlen(wlturlsize));
	//	UART_Write(UART1, ",", 1);
		UART_Write(UART1, "80", 2);
	//	UART_Write(UART1, ",", 1);
	//	UART_Write(UART1, "80", 2);
		UART_Write(UART1, "\r\n", 2);

	//	}
		break;

	case 10:

		break;
	case 11:
		loop_delay(2);
//		UART_Write(UART1, "\r\n", 2);
//		UART_Write(UART2, "\r\n", 2);
	//	UART_Write(UART2, URL, strlen(URL));
	//	UART_Write(UART1, URL, strlen(URL));
	//	UART_Write(UART1, "\r\n", 2);
	//	UART_Write(UART2, "\r\n", 2);
		break;
	case 12:
//		loop_delay(3);
//		UART_Write(UART1, "\r\n", 4);
//		UART_Write(UART2, "\r\n", 4);
//		UART_Write(UART2, URL, strlen(URL));
//		UART_Write(UART1, URL, strlen(URL));
		break;
	case 13:
//		loop_delay(2);
//		UART_Write(UART1, "AT+QHTTPREAD=", 13);
//			//	UART_Write(UART1, DBQUOTE, 1);
//				UART_Write(UART1, "80",2);
//			//	UART_Write(UART1, DBQUOTE, 1);
//		//		UART_Write(UART1, ",", 1);
//		//		UART_Write(UART1, "1", 1);
//			UART_Write(UART1, "\r\n", 2);

			break;
	case 14:
//		if(DITION)
//		{
//			DITION=0;
//		        UART_Write(UART1, "AT+QHTTPREAD=80\r\n", 19);
//		        UART_Write(UART2, "\r\n", 2);
//		}
			break;
	case 15:
		loop_delay(2);
	UART_Write(UART1, "AT+QHTTPREAD=", 13);
		//	UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, "80",2);
		//	UART_Write(UART1, DBQUOTE, 1);
	//		UART_Write(UART1, ",", 1);
	//		UART_Write(UART1, "1", 1);
		UART_Write(UART1, "\r\n", 2);

			break;

	case 16:
		gsmState = GNSS_QUERY;
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
 * @brief       gnss_query
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Query if connected to apn, check IP.
 */

void gnss_query()
{
	switch (loopCount)
	{
	case 1:
//		UART_Write(UART1, "AT+QHTTPREAD=", 13);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "80",2);
//		UART_Write(UART1, DBQUOTE, 1);
////		UART_Write(UART1, ",", 1);
////		UART_Write(UART1, "1", 1);
//	UART_Write(UART1, "\r\n", 2);

		break;
	case 2:
//		loop_delay(2);
	//	UART_Write(UART1, "AT+QGPS=1\r\n", 13);
	//	UART_Write(UART1, "\r\n", 2);
		//loop_delay(3);
		break;
	case 3:

		break;
	case 4:
		loop_delay(5);
	//	UART_Write(UART1, "AT+QGPSLOC=2\r\n", 16);
		DITION=1;
		break;

	case 5:
//		loop_delay(2);
	//	readGNSS = 1;
	//	UART_Write(UART1, "\r\n", 2);
		break;
	case 6:
//		UART_Write(UART1, "AT+QGPSCFG=", 11);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "autogps", 7);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "1", 1);
//		UART_Write(UART1, "\r\n", 2);
		/*UART_Write(UART1, "AT+QGPSGNMEA=", 13);
		 UART_Write(UART1, DBQUOTE, 1);
		 UART_Write(UART1, "GGA", 3);
		 UART_Write(UART1, DBQUOTE, 1);
		 UART_Write(UART1, "\r\n", 4);*/
		break;

	case 7:
		break;
	case 8:
		gsmState = GSM_SLEEP;
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
 * @brief       gsm_http_get
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Initialise HTTP, use GET  method.
 */

/**
 * @brief       gsm_idle
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Idle.
 */
void gsm_idle() {
	switch (loopCount) {
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
		UART_Write_main(UART2, "Check Connectivity periodically\r\n", 34);
		gsmState = GSM_CONNECT;
		loopCount = 0;
//		query_conn = 1;
//		query_conn = 0;  // should check edel each time
		break;
	default:
		if (loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;

	}

}

void loop_delay(uint8_t loopDel) {
	tloopCount = loopCount;
	loopCount = 0x5f;
	loopDelay = loopDel;
}

void  GET_GNSS_data()
{


		rtc = strstr(GNSS_data,"+QGPSLOC:");

	    strncpy(gnssutc,rtc+10,12);
	    strncpy(gnsslat,rtc+21,8);
	    strncpy(gnsslon,rtc+30,8);
	    strncpy(gnsshdop,rtc+39,3);
	    strncpy(gnssalt,rtc+43,5);
	    strncpy(gnssfix,rtc+49,1);
	    strncpy(gnsscog,rtc+51,6);
	    strncpy(gnssspkm,rtc+58,3);
	    strncpy(gnssspkn,rtc+62,3);
	    strncpy(gnssdate,rtc+66,6);
	    strncpy(gnssnsat,rtc+73,2);

	    strncpy(gnssday,gnssdate,2);

	    strncpy(gnssmon,gnssdate+2,2);

	    strncpy(gnssyear,gnssdate+4,2);

	    strncpy(gnssutctime,gnssutc,2);

	    strncpy(gnssutcmin,gnssutc+2,2);

	    strncpy(gnssutcsec,gnssutc+4,2);

}

void WLT_SEND_DATA()
{


	//memset(URL,0,BUF_LEN);
	strcat(URL,WLTIME);
	strcat(URL,IME);
	strcat(URL,WLTLAT);
	strcat(URL,gnsslat);
    strncpy(URL+35,0,8);
	strcat(URL,WLTLON);
	strcat(URL,gnsslon);
	strcat(URL,WLTHEADING);
//	strcat(URL,"0");
	strcat(URL,gnsshdop);
//	strcat(URL,WLTEVENTID);
//	strcat(URL,"123");
	strcat(URL,WLTSPEED);
//	strcat(URL,"30");
	strcat(URL,gnssspkm);
	strcat(URL,WLTODOMETER);
	strcat(URL,"0");
	strcat(URL,WLTIGNITION);
	strcat(URL,"1");
	strcat(URL,WLTGPSDATE);
	strcat(URL,"20");
	strcat(URL,gnssyear);
	strcat(URL,"-");
	strcat(URL,gnssmon);
	strcat(URL,"-");
	strcat(URL,gnssday);
	strcat(URL,"T");
	strcat(URL,gnssutctime);
	strcat(URL,":");
	strcat(URL,gnssutcmin);
	strcat(URL,":");
	strcat(URL,gnssutcsec);
	strcat(URL,"Z");
	strcat(URL,WLTALTITUDE);
	strcat(URL,gnssalt);


	urlsize=strlen(URL);
	itoa(urlsize,wlturlsize,10);

//	UART_Write_main(UART2,"\r\n",2);
//	urlsize=sizeof(URL);
//	UART_Write_main(UART2,"******************WLTDATA**************",39);
//	UART_Write_main(UART2,"\r\n",2);
//	UART_Write_main(UART2,URL ,strlen(URL));
//	UART_Write_main(UART2,"\r\n",2);
//	UART_Write_main(UART2,"******************WLTDATA**************",39);
//	UART_Write_main(UART2,"\r\n",2);



}


// Reverses a string 'str' of length 'len'
void reverse(char *str, int len) {
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
int intToStr(int x, char str[], int d) {
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
void ftoa(float n, char *res, int afterpoint) {
	// Extract integer part
	int ipart = (int) n;

	// Extract floating part
	float fpart = n - (float) ipart;

	// convert integer part to string
	int i = intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0) {
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter
		// is needed to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		intToStr((int) fpart, res + i + 1, afterpoint);
	}
}

void delay_loop() {
	while (delayCounter) {
		delayCounter--;
	}

}

/* check if config mode is on, if configMode is ON, do not write*/
uint32_t UART_Write_main(UART_T *uart, uint8_t *pu8TxBuf,
		uint32_t u32WriteBytes) {
	uint32_t u32Count = 0;

	if (!configMode)
		u32Count = UART_Write(uart, pu8TxBuf, u32WriteBytes);

	return u32Count;

}

