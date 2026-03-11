/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 7 $
 * $Date: 17/05/04 1:36p $
 * @brief    Configure SPI0 as Master mode and demonstrate how to communicate
 *           with an off-chip SPI Slave device with FIFO mode. This sample
 *           code needs to work with SPI_SlaveFifoMode sample code.
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M0564.h"
#include "rfm69_function.h"
#include "rfm69_registor.h"
#include "wlc_stack.h"
#include "string.h"


void door_sensor_status();
void smart_switch_statua();
void panic_status();
void motion_status();

#define TEST_COUNT 16
#define Data_count 16

#define   BUFSIZE              30
#define   REC_BUF_SIZE         15
#define   SEND_BUF_SIZE        15
#define   SPI_REC_BUF_SIZE     15

extern uint8_t RFState;

#define BUF_LEN_SMALL 20
#define API_BUFF_LEN 50
#define URL_LENGTH 50
#define BUF_LEN 250
#define BUF_LEN_LONG 500
#define DATA_FLASH_START 0x10000

uint8_t UART1_RX_BUF[BUF_LEN];
uint8_t UART1_TX_BUF[BUF_LEN];
uint8_t count_1;
uint8_t newWord_1 = 0;

uint8_t UART2_RX_BUF[BUF_LEN_LONG];
uint8_t UART2_TX_BUF[BUF_LEN];
uint16_t count_2;
uint8_t newWord_2 = 0;

uint8_t readIMEI = 0;
uint8_t IMEI[BUF_LEN_SMALL];

uint8_t configMode = 0;

/* special char */
char DBQUOTE[1] = { 0x22 };

void UART2_TEST_HANDLE();
void UART1_TEST_HANDLE();

void loop_delay(uint8_t loopDel);
void gsm_init();
void gsm_get_snapshot();
void gsm_datacall();
void gsm_connect();
void gsm_query();
void switch_command();

void gsm_idle();
void gsm_http_report_data();

int intmyatoi(char*);

void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);

uint8_t z;
uint16_t COUNTER=0;
uint8_t COUNT[5];

uint32_t UART_Write_main(UART_T *uart, uint8_t *pu8TxBuf,
		uint32_t u32WriteBytes);

enum {
	GSM_INIT = 1,
	GSM_SNAPSHOT,
	GSM_DATACALL,
	GSM_CONNECT,
	GSM_QUERY,
	GSM_HTTP_REPORT_DATA,
	SWITCH_COMMND,
	GSM_SLEEP,
} GSM_SM_T;

uint8_t gsmState = GSM_INIT;

uint8_t *databuff;
uint8_t z;
uint8_t buffsize;
void RFMair();
uint8_t sizebuff[BUF_LEN_SMALL];
uint8_t testURL[BUF_LEN]="http://thingsboard.cloud/api/v1/WnWwwDh4D8Hi1H4HnJ95/telemetry";
uint8_t PM1[BUFSIZE]="PM1.0:";
uint8_t PM2[BUFSIZE]="PM2.5:";
uint8_t PM4[BUFSIZE]="PM4.0:";
uint8_t PM10[BUFSIZE]="PM10:";
uint8_t Humidity[BUFSIZE]="Humidity:";
uint8_t Temparature[BUFSIZE]="Temparature:";
uint8_t VOC[BUFSIZE]="VOC:";
uint8_t  NOx[BUFSIZE]="NOx:";
uint8_t  COUN[BUFSIZE]="COUNTER:";
uint8_t   URLSWITCH[BUF_LEN]="http://thingsboard.cloud/api/v1/WnWwwDh4D8Hi1H4HnJ95/rpc";

uint8_t size;
uint8_t buffvalue[50];
uint8_t   testurl[BUF_LEN];


uint8_t loopCount = 0;
uint8_t tloopCount = 0;
uint8_t loopDelay = 0;
uint8_t timeOut = 0;

uint8_t pm1[3];
uint8_t pm2[3];
uint8_t pm4[3];
uint8_t pm10[3];
uint8_t humidity[3];
uint8_t temparature[3];
uint8_t voc[3];
uint8_t nox[3];
uint8_t databbuf[BUFSIZE];

uint16_t batPower;
uint16_t cal_power;
float rec_bat_power = 0;
uint8_t hard_ware = 0;
uint8_t soft_ware = 0;
uint8_t bat_power=0;

uint8_t txFlag = 0;
uint8_t recFlag = 0;

uint16_t crc_rec = 0;
uint8_t sequence = 0x01;
uint8_t sendBuf[SEND_BUF_SIZE];

uint8_t RFbuffer[BUFSIZE];                // RF buffer
uint8_t Rec_cpy[BUFSIZE];
uint16_t RFM;
uint8_t recBuf[REC_BUF_SIZE];
uint8_t RFbufferSize;

uint8_t spi_rec_data[SPI_REC_BUF_SIZE];
uint8_t spi_rec_count = 0;

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

// ***CRC calculation *****//
uint16_t crc_calcul(uint8_t buf[], int len) {
	uint16_t crc = 0xFFFF;
	uint16_t crcLow, crcHigh;
	uint8_t pos = 0, i;
	for (pos = 0; pos < len; pos++) {
		crc ^= (uint16_t) buf[pos];      // XOR byte into least sig. byte of crc

		for (i = 8; i != 0; i--) {                         // Loop over each bit
			if ((crc & 0x0001) != 0) {                      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			} else
				// Else LSB is not set
				crc >>= 1;                      // Just shift right
		}
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
	crcLow = crc & 0xFF00;
	crcHigh = crc & 0x00FF;
	crcLow >>= 8;
	crcHigh <<= 8;
	crc = crcHigh | crcLow;
	return crc;
}
// ***send data through rfm69 *****//
void send(uint8_t source, uint8_t Dest, uint16_t sourId, uint16_t DestId,
		uint16_t Data, uint8_t *rec_value) {
	uint8_t ReturnCode = -1;
	uint16_t crc_send = 0;

	memset(RFbuffer, 0, BUFSIZE);
	WLC.SOF = '$';
	WLC.Source = source;
	WLC.Destination = Dest;

	RFM = sourId;
	sourId = sourId >> 8;
	sourId |= RFM << 8;
	WLC.sourceId = sourId;

	RFM = DestId;
	DestId = DestId >> 8;
	DestId |= RFM << 8;
	WLC.DestId = DestId;

	WLC.Sequence = sequence;
	WLC.ReadWrite = WRITE;
	WLC.Misc = 0x00;

	WLC1.Data = Data;
	WLC1.Value = rec_value;
	WLC1.Length = strlen(WLC1.Value);

	memcpy(RFbuffer, &WLC, sizeof(WLC_t));
	memcpy(RFbuffer + sizeof(WLC_t), &WLC1.Length, sizeof(WLC1.Length));
	memcpy(RFbuffer + sizeof(WLC_t) + 1, &WLC1.Data, sizeof(WLC1.Data));
	memcpy(RFbuffer + sizeof(WLC_t) + 1 + sizeof(WLC1.Data), WLC1.Value,(WLC1.Length));

	crc_send = crc_calcul(RFbuffer,sizeof(WLC_t) + 1 + sizeof(WLC1.Data) + WLC1.Length);
	RFM = crc_send;
	crc_send = crc_send >> 8;
	crc_send |= RFM << 8;

	WLC1.crc = crc_send;
	memcpy(RFbuffer + sizeof(WLC_t) + 1 + sizeof(WLC1.Data) + WLC1.Length,&WLC1.crc, sizeof(WLC1.crc));
	RFbufferSize = sizeof(WLC_t) + 1 + sizeof(WLC1.Data) + WLC1.Length + 2;
	SendRfFrame(RFbuffer, RFbufferSize, &ReturnCode); // Sends the frame to the RF chip
	sequence++;
	memset(&WLC, 0, sizeof(WLC_t));
	memset(RFbuffer, 0, BUFSIZE);
}
// ***Receive data from another rfm69 device *****//
void receive(void) {
	uint8_t ReturnCode = -1;

	memset(RFbuffer, 0, BUFSIZE);
	memset(recBuf, 0, REC_BUF_SIZE);


	RFbufferSize = sizeof(WLC_t) + 1 + sizeof(WLC1.Data) + WLC1.Length + 2;
	ReceiveRfFrame(&RFbuffer, &RFbufferSize, &ReturnCode); // Receives the frame from the RF chip

	memcpy(Rec_cpy, RFbuffer + 2, RFbufferSize);

	memcpy(&WLC, &Rec_cpy, sizeof(WLC_t));
	memcpy(&WLC1.Length, &Rec_cpy[sizeof(WLC_t)], sizeof(WLC1.Length));
	memcpy(&WLC1.Data, &Rec_cpy[sizeof(WLC_t) + 1], sizeof(WLC1.Data));
	memcpy(recBuf, &Rec_cpy[sizeof(WLC_t) + 1 + sizeof(WLC1.Data)],WLC1.Length);
	WLC1.Value = &recBuf;
	memcpy(&WLC1.crc,&Rec_cpy[sizeof(WLC_t) + 1 + sizeof(WLC1.Data) + WLC1.Length],sizeof(WLC1.crc));

//	memcpy(databbuf,recBuf,sizeof(recBuf));
}
// ***Reset RFM69 chip*****//
void reset_rfm69() {
	PD2 = 1;
	TIMER_Delay(TIMER1, 100);
	PD2 = 0;
	TIMER_Delay(TIMER1, 100);
}
void TMR0_IRQHandler(void) {
	if (TIMER_GetIntFlag(TIMER0) == 1) {
		/* Clear Timer0 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER0);
		loopCount++;
		timeOut = 1;

	}
}

void UART02_IRQHandler(void)
{
	UART2_TEST_HANDLE();
}
void UART2_TEST_HANDLE() {
	uint8_t u8InChar = 0xFF;
	uint32_t u32IntSts = UART2->INTSTS;

	if (u32IntSts & UART_INTSTS_RDAINT_Msk) {

		/* Get all the input characters */
		while (UART_IS_RX_READY(UART2)) {
			UART_Read(UART2, &UART2_RX_BUF[count_2], 1);

		//	databuff[z++] = (UART2_RX_BUF[count_2]);

//        if(UART2_RX_BUF[count_2] == '\r')
//        	newWord_2 = 1;

//        if((configMode == 1) && (count_2 > 2))
//        {
//        	if((UART2_RX_BUF[count_2] == '\r') && (UART2_RX_BUF[count_2 -1] == '-') && (UART2_RX_BUF[count_2 -2] == '-') && (UART2_RX_BUF[count_2 -3] == '-'))
//			{
//				configMode = 0;
//				UART_Write(UART2, "Exit Config Mode\r\n", 18);
//				configReadBack = 1;
//
//			}
//
//        }


			count_2++;
			if (count_2 > BUF_LEN_LONG)
				count_2 = 0;

		}

	}
	UART2->INTSTS = 0;



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
/* Init SPI0                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/

void SPI_Init(void) {
	/* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
	/* Set IP clock divider. SPI clock rate = 2MHz */
	SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 1600000);

	SPI_SetFIFO(SPI0, 0, 0);
	SPI_EnableInt(SPI0, SPI_FIFO_RXTH_INT_MASK);
	NVIC_EnableIRQ(SPI0_IRQn);

}
/*---------------------------------------------------------------------------------------------------------*/
/* Init SPI0 Interrupt Handler                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/

void SPI0_IRQHandler(void)
{
	/* Check RX EMPTY flag */


	if (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
	{
		/* Read RX FIFO */
		spi_rec_data[spi_rec_count++] = SPI_READ_RX(SPI0);
		if (spi_rec_count > SPI_REC_BUF_SIZE)
		{
			spi_rec_count = 0;
			memset(spi_rec_data, 0, SPI_REC_BUF_SIZE);

		}
	}

}

/* GPIO pin(DIO0) interrupt used for send and receive  */
void GPCDEF_IRQHandler(void) {

	if (GPIO_GET_INT_FLAG(PD, BIT3)) {
		GPIO_CLR_INT_FLAG(PD, BIT3);
		if (PD3) {
			RFState |= RF_RX_DONE;
			RFState &= ~RF_BUSY;
			if (txFlag) {
				txFlag = 0;
				recFlag = 0;
			} else {
				recFlag = 1;
			}
		} else {
			RFState |= RF_STOP;
			RFState &= ~RF_RX_DONE;
		}
	}
}
void SYS_Init(void) {

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



	/* Enable UART module clock */
	CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk| CLK_APBCLK0_UART2CKEN_Msk);

	/* Select UART module clock source as HIRC and UART module clock divider as 1 */
	CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk))| CLK_CLKSEL1_UARTSEL_HIRC;
	CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk))| CLK_CLKDIV0_UART(1);

	/* Select PCLK0 as the clock source of SPI0 */
	CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK0, MODULE_NoMsk);

	/* Enable UART peripheral clock */
//	CLK_EnableModuleClock(UART0_MODULE);

	/* Enable SPI0 peripheral clock */
	CLK_EnableModuleClock(SPI0_MODULE);

	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
	SystemCoreClockUpdate();

	/*---------------------------------------------------------------------------------------------------------*/
	/* Init uart initial-function                                                                                 */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Set PD multi-function pins for UART0 RXD and TXD */

//	SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
//	SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD| SYS_GPD_MFPL_PD1MFP_UART0_TXD);

	 SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk );
	  SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_UART1_TXD | SYS_GPA_MFPL_PA1MFP_UART1_RXD );

	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk);
	SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC2MFP_UART2_TXD| SYS_GPC_MFPL_PC3MFP_UART2_RXD);

	/* Setup SPI0 multi-function pins */

	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk| SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);

	SYS->GPB_MFPL = SYS_GPB_MFPL_PB4MFP_SPI0_SS | SYS_GPB_MFPL_PB5MFP_SPI0_MOSI| SYS_GPB_MFPL_PB6MFP_SPI0_MISO | SYS_GPB_MFPL_PB7MFP_SPI0_CLK;

	//timer0 initial
	 CLK_EnableModuleClock(TMR0_MODULE);
	 CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

	 //timer1  initial
	 CLK_EnableModuleClock(TMR1_MODULE);
		 CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);



	/* Gpio init for rfm69 trans receive */
	GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);
	PD2 = 0;

	/* Gpio init for rfm69 trans receive */
	GPIO_SetMode(PD, BIT3, GPIO_MODE_INPUT);
	GPIO_EnableInt(PD, 3, GPIO_INT_BOTH_EDGE);
	NVIC_EnableIRQ(GPCDEF_IRQn);
//	    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
//	    GPIO_ENABLE_DEBOUNCE(PD, BIT3);
}
/* ------------- */
/* Main function */
/* ------------- */
int main(void) {
	/* Unlock protected registers */
	SYS_UnlockReg();
	/* Init System, IP clock and multi-function I/O. */
	SYS_Init();
	/* Lock protected registers */
	SYS_LockReg();
	/* Init UART2 to 9600-8n1 for print message */
	UART2_Init();
	UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk));

	UART_Write_main(UART2, "EDEL SMART DEVICES - 001\r\n", 26);
	//UART_Write(UART2, "EDEL SMART DEVICES - 001\r\n", 26);

	UART1_Init();
//UART_Write(UART1, ubx_reset,12 );

	UART_Write(UART1, "AT\r\n", 4);
	UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk));

	/* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 3);
	TIMER_EnableInt(TIMER0);

	/* Enable Timer0 ~ Timer3 NVIC */
	NVIC_EnableIRQ(TMR0_IRQn);

	/* Start Timer0 ~ Timer3 counting */
	TIMER_Start(TIMER0);

	/* Init SPI */
	SPI_Init();

	/* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
	UART_Open(UART0, 9600);
	/* reset RFM69 */
	reset_rfm69();

	/* Init RFM69 */
	InitRFChip();

	while (1) {
		if (recFlag) {
			recFlag = 0;
			receive();
		} else {
			receiveInit();
		}

		if (WLC.SOF == '#') {
			crc_rec = crc_calcul(Rec_cpy,
					sizeof(WLC_t) + 1 + sizeof(WLC1.Data) + WLC1.Length);
			RFM = crc_rec;
			crc_rec = crc_rec >> 8;
			crc_rec |= RFM << 8;
			if (crc_rec == WLC1.crc) {
				if (WLC.Destination == CONTROL_PANEL) {
					memset(sendBuf, 0, SEND_BUF_SIZE);
					 memset(databbuf,0,sizeof(databbuf));
					memcpy(databbuf,recBuf,sizeof(recBuf));
					 // RFMair();
//				    itoa(recBuf[0],perbuf,10);
//				    UART_Write(UART2,PM1,14);
//					UART_Write(UART2,perbuf,3);
//					UART_Write(UART2,"\n",3);
//					switch (WLC.Source) {
//					case (DOOR_SENSOR):
//						door_sensor_status();recBuf
//						break;
//					case (SMART_SWITCH):
//						smart_switch_statua();uint8_t PM1="PM1.0 (ug/m3)=";

//						break;
//					case (PANIC_BUTTON):
//						panic_status();
//						break;
//					case (MOTION_SENSOR):
//						motion_status();
//						break;
//					default:
//						sendBuf[0] = NACK;
//						send(PANIC_BUTTON, CONTROL_PANEL, PANIC_BUTTON_ID,
//						CONTROL_PANEL_ID, UNSUCCESSFUL_ACKNOWLEDGEMENT,
//								&sendBuf);
//						break;
//					}
				}
			}
			memset(&WLC, 0, sizeof(WLC_t));
			memset(&WLC1, 0, sizeof(WLC_t1));
		}

		if (timeOut) {
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

			case GSM_QUERY:
				gsm_query();
				break;

			case GSM_HTTP_REPORT_DATA:
				gsm_http_report_data();
				break;

			case SWITCH_COMMND:
				switch_command();
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


 void RFMair()

 {

//	 UART_Write(UART2, "\n", 1);

//	 memset(databbuf,0,sizeof(databbuf));
//
//	 memcpy(databbuf,recBuf,sizeof(recBuf));

	 memset(testurl,0,strlen(testurl));

	 COUNTER++;

	 itoa(COUNTER,COUNT,10);

	itoa(databbuf[0], pm1, 10);
//	UART_Write(UART2, PM1, strlen(PM1));
//	UART_Write(UART2, pm1, strlen(pm1));
//	UART_Write(UART2, "\n", 1);

	itoa(databbuf[1], pm2, 10);
//	UART_Write(UART2, PM2, strlen(PM2));
//	UART_Write(UART2, pm2, strlen(pm2));
//	UART_Write(UART2, "\n", 1);

	itoa(databbuf[2], pm4, 10);
//	UART_Write(UART2, PM4, strlen(PM4));
//	UART_Write(UART2, pm4, strlen(pm4));
//	UART_Write(UART2, "\n", 1);

	itoa(databbuf[3], pm10, 10);
//	UART_Write(UART2, PM10, strlen(PM10));
//	UART_Write(UART2, pm10, strlen(pm10));
//	UART_Write(UART2, "\n", 1);

	itoa(databbuf[4], humidity, 10);
//	UART_Write(UART2, Humidity, strlen(Humidity));
//	UART_Write(UART2, humidity, strlen(humidity));
//	UART_Write(UART2, "\n", 1);

	itoa(databbuf[5], temparature, 10);
//	UART_Write(UART2, Temparature, strlen(Temparature));
//	UART_Write(UART2, temparature, strlen(temparature));
//	UART_Write(UART2, "\n", 1);

	itoa(databbuf[6], voc, 10);
//	UART_Write(UART2, VOC, strlen(VOC));
//	UART_Write(UART2, voc, strlen(voc));
//	UART_Write(UART2, "\n", 1);

	itoa(databbuf[7], nox, 10);
//	UART_Write(UART2, NOx, strlen(NOx));
//	UART_Write(UART2, nox, strlen(nox));
//	UART_Write(UART2, "\n", 1);
	strcat(testurl,"{");
	strcat(testurl,PM1);
	strcat(testurl,pm1);
	strcat(testurl,",");
	strcat(testurl,PM2);
	strcat(testurl,pm2);
	strcat(testurl,",");
	strcat(testurl,PM4);
	strcat(testurl,pm4);
	strcat(testurl,",");
	strcat(testurl,PM10);
	strcat(testurl,pm10);
	strcat(testurl,",");
	strcat(testurl,Humidity);
	strcat(testurl,humidity);
	strcat(testurl,",");
	strcat(testurl,Temparature);
	strcat(testurl,temparature);
	strcat(testurl,",");
	strcat(testurl,VOC);
	strcat(testurl,voc);
	strcat(testurl,",");
	strcat(testurl,NOx);
	strcat(testurl,nox);
	strcat(testurl,",");
	strcat(testurl,COUN);
	strcat(testurl,COUNT);
	strcat(testurl,"}");



	size=strlen(testurl);
//	UART_Write(UART2, testurl, strlen(testurl));
	itoa(size,buffvalue,10);

//	memset(databbuf,0,sizeof(databbuf));
//	memset(recBuf, 0, REC_BUF_SIZE);
 }


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
void door_sensor_status() {
	UART_Write(UART2,"Door_Sensor ",sizeof("Door_Sensor "));
//	printf("Door_Sensor  ");
	memset(sendBuf, 0, SEND_BUF_SIZE);
	switch (WLC1.Data) {
	case (DOOR_STATUS):
		if (*WLC1.Value == OPEN)
		//	printf("Door_Open\n");
	UART_Write(UART2,"Door_Open\n",sizeof("Door_Open\n"));
		if (*WLC1.Value == CLOSE)
			//printf("Door_closed\n");
			UART_Write(UART2,"Door_closed\n",sizeof("Door_closed\n"));
		break;
	case (BATTERY_LEVEL):
		rec_bat_power = *WLC1.Value << 8 | *(WLC1.Value + 1);
		rec_bat_power = (rec_bat_power / 2046) * 100;
		//printf("Battery level %f", rec_bat_power);
		UART_Write(UART2,"Battery level %f",sizeof("Battery level %f"));
		bat_power=0;
				  UART_Write(UART2,DBQUOTE,1);
				  bat_power=floor(rec_bat_power);
				  UART_Write(UART2,bat_power,sizeof(bat_power));
				  UART_Write(UART2,DBQUOTE,1);
		break;
	case (HARDWARE_VERSION):
		hard_ware = *(WLC1.Value + 0);
	//	printf("Hard_ware_vertion%x\n", hard_ware);
	 UART_Write(UART2,"Hard_ware_vertion%x\n",sizeof("Hard_ware_vertion%x\n"));
	 UART_Write(UART2,DBQUOTE,1);
	 UART_Write(UART2,hard_ware,sizeof(hard_ware));
	 UART_Write(UART2,DBQUOTE,1);
		break;
	case (SOFTWARE_VERSION):
		soft_ware = *WLC1.Value << 0;
	//	printf("Soft_ware_vertion%x\n", soft_ware);
		UART_Write(UART2,"Soft_ware_vertion%x\n",sizeof("Soft_ware_vertion%x\n"));
		  UART_Write(UART2,DBQUOTE,1);
		  UART_Write(UART2,soft_ware,sizeof(soft_ware));
		  UART_Write(UART2,DBQUOTE,1);

		break;
	case (TAMPER_STATUS):
		if (*WLC1.Value == OPEN)

			//printf("Tamper_Open\n");
	UART_Write(UART2,"Tamper_Open\n",sizeof("Tamper_Open\n"));

		if (*WLC1.Value == CLOSE)
			//printf("Tamper_closed\n");
		UART_Write(UART2,"Tamper_closed\n",sizeof("Tamper_closed\n"));
		break;
	case (LED_STATUS):
		if (*WLC1.Value == ON)
		//	printf("Led_ON\n");
	UART_Write(UART2,"Led_ON\n",sizeof("Led_ON\n"));
		if (*WLC1.Value == OFF)
		//	printf("Led_OFF\n");
			UART_Write(UART2,"Led_OFF\n",sizeof("Led_OFF\n"));
		break;
	case (WIRED_STATUS):
		if (*WLC1.Value == CONNECTED)
		//	printf("Wire_Connected\n");
	UART_Write(UART2,"Wire_Connected\n",sizeof("Wire_Connected\n"));
		if (*WLC1.Value == DISCONNECTED)
			//printf("Wire_Disconnected\n");
			UART_Write(UART2,"Wire_Disconnected\n",sizeof("Wire_Disconnected\n"));
		break;
	default:
		sendBuf[0] = UNSUCCESSFUL_ACKNOWLEDGEMENT;
		send(DOOR_SENSOR, CONTROL_PANEL, DOOR_SENSOR_ID, CONTROL_PANEL_ID, NACK,
				&sendBuf);
		break;
	}
}
void smart_switch_statua() {
	//printf("Key_Fob ");
	UART_Write(UART2,"Key_Fob",sizeof("Key_Fob"));
	memset(sendBuf, 0, SEND_BUF_SIZE);
	switch (WLC1.Data) {
	case (BATTERY_LEVEL):
		rec_bat_power = *WLC1.Value << 8 | *(WLC1.Value + 1);
		rec_bat_power = (rec_bat_power / 2046) * 100;
		//printf("Battery level %f", rec_bat_power);
		UART_Write(UART2,"Battery level %f",sizeof("Battery level %f"));
		bat_power=0;
		UART_Write(UART2,DBQUOTE,1);
		bat_power= floor(rec_bat_power);
		UART_Write(UART2,bat_power,sizeof(bat_power));
		UART_Write(UART2,DBQUOTE,1);
		break;
	case (HARDWARE_VERSION):
		//printf("Hard_ware_vertion%x\n", *WLC1.Value);
		UART_Write(UART2,"Hard_ware_vertion%x\n",sizeof("Hard_ware_vertion%x\n"));
	    UART_Write(UART2,DBQUOTE,1);
     	UART_Write(UART2,*WLC1.Value,sizeof(*WLC1.Value));
     	UART_Write(UART2,DBQUOTE,1);
		break;
	case (SOFTWARE_VERSION):
	//	printf("Soft_ware_vertion%x\n", *WLC1.Value);
		UART_Write(UART2,"Soft_ware_vertion%x\n",sizeof("Force_Home_ARM state\n"));
     	UART_Write(UART2,DBQUOTE,1);
	    UART_Write(UART2,*WLC1.Value,sizeof(*WLC1.Value));
	    UART_Write(UART2,DBQUOTE,1);

		break;
	case (ARM):
	//	printf("ARM state\n");
		UART_Write(UART2,"ARM state\n",sizeof("ARM state\n"));
		sendBuf[0] = ON;
		send(CONTROL_PANEL, DOOR_SENSOR, DOOR_SENSOR_ID, CONTROL_PANEL_ID,
				LED_STATUS, &sendBuf);
		break;
	case (FORCE_ARM):
		//printf("Forced ARM state\n");
		UART_Write(UART2,"Forced ARM state\n",sizeof("Forced ARM state\n"));
		break;
	case (DISARM):
		//printf("DISARM state\n");
		UART_Write(UART2,"DISARM state\n",sizeof("DISARM state\n"));
		sendBuf[0] = OFF;
		send(CONTROL_PANEL, DOOR_SENSOR, DOOR_SENSOR_ID, CONTROL_PANEL_ID,
				LED_STATUS, &sendBuf);
		break;
	case (HOME_ARM):
		UART_Write(UART2,"Home_ARM state\r\n",sizeof("Home_ARM state\r\n"));
	    databuff="Home_ARM state";
		break;
	case (FORCE_HOME_ARM):
	//	printf("Force_Home_ARM state\n");
	UART_Write(UART2,"Force_Home_ARM state\r\n",sizeof("Force_Home_ARM state\r\n"));
		break;
	default:
		sendBuf[0] = UNSUCCESSFUL_ACKNOWLEDGEMENT;
		send(SMART_SWITCH, CONTROL_PANEL, SMART_SWITCH_ID, CONTROL_PANEL_ID,
				NACK, &sendBuf);
		break;
	}
}
void panic_status() {
//	printf("Panic_Button  ");
	UART_Write(UART2,"Panic_Button ",sizeof("Panic_Button "));
	memset(sendBuf, 0, SEND_BUF_SIZE);
	switch (WLC1.Data) {
	case (PANIC_DETECTION):
		if (*WLC1.Value == ON)
			//printf("Panic Button Pressed\n");
			UART_Write(UART2,"Panic Button Pressed\n ",sizeof("Panic Button Pressed\n"));
			break;
	case (BATTERY_LEVEL):
		rec_bat_power = *WLC1.Value << 8 | *(WLC1.Value + 1);
		rec_bat_power = (rec_bat_power / 2046) * 100;
		//printf("Battery level %f", rec_bat_power);
		UART_Write(UART2,"Battery level %f ",sizeof("Battery level %f"));
		bat_power=0;
		 UART_Write(UART2,DBQUOTE,1);
		 bat_power= floor(rec_bat_power);
		 UART_Write(UART2,bat_power,sizeof(bat_power));
		 UART_Write(UART2,DBQUOTE,1);

		break;
	case (HARDWARE_VERSION):
	//	printf("Hard_ware_vertion%x\n", *WLC1.Value);
		UART_Write(UART2,"Hard_ware_vertion% x",sizeof("Hard_ware_vertion% x"));
	    UART_Write(UART2,DBQUOTE,1);
	    UART_Write(UART2,*WLC1.Value,sizeof(*WLC1.Value));
	    UART_Write(UART2,DBQUOTE,1);
		break;
	case (SOFTWARE_VERSION):
	//	printf("Soft_ware_vertion%x\n", *WLC1.Value);
		UART_Write(UART2,"Soft_ware_vertion%x\n",sizeof("Soft_ware_vertion%x\n"));
	  UART_Write(UART2,DBQUOTE,1);
	  UART_Write(UART2,*WLC1.Value,sizeof(*WLC1.Value));
	  UART_Write(UART2,DBQUOTE,1);
		break;

	case (TAMPER_STATUS):
		if (*WLC1.Value == OPEN)
		//	printf("Tamper_Open\n");
			 UART_Write(UART2,"Tamper_Open\n",sizeof("Tamper_Open\n"));
		if (*WLC1.Value == CLOSE)
			//printf("Tamper_closed\n");
			 UART_Write(UART2,"Tamper_closed\n",sizeof("Tamper_closed\n"));
		break;
	case (LED_STATUS):
		if (*WLC1.Value == ON)
			//printf("Led_ON\n");
			UART_Write(UART2,"Led_ON\n",sizeof("Led_ON\n"));
		if (*WLC1.Value == OFF)
		//	printf("Led_OFF\n");
			UART_Write(UART2,"Led_OFF\n",sizeof("Led_OFF\n"));
		break;
	case (WIRED_STATUS):
		if (*WLC1.Value == CONNECTED)
			//printf("Wire_Connected\n");
			UART_Write(UART2,"Wire_Connected\n",sizeof("Wire_Connected\n"));

		if (*WLC1.Value == DISCONNECTED)
			//printf("Wire_Disconnected\n");
			UART_Write(UART2,"Wire_Disconnected\n",sizeof("Wire_Disconnected\n"));
		break;
	default:
		sendBuf[0] = UNSUCCESSFUL_ACKNOWLEDGEMENT;
		send(SMART_SWITCH, CONTROL_PANEL, SMART_SWITCH_ID, CONTROL_PANEL_ID,
				NACK, &sendBuf);
		break;
	}
}
void motion_status() {
	//printf("Motion Sensor  ");
	UART_Write(UART2,"Motion Sensor\n",sizeof("Motion Sensor\n"));
	memset(sendBuf, 0, SEND_BUF_SIZE);
	switch (WLC1.Data) {
	case (MOTION_DETECTION):
		if (*WLC1.Value == DETECTED)
			//printf("Motion Detected\n");
			UART_Write(UART2,"Motion Detected\n",sizeof("Motion Detected\n"));
		else if (*WLC1.Value == UNDETECTED)
		//	printf("Motion Un_Detected\n");
			UART_Write(UART2,"Motion Un_Detected\n",sizeof("Motion Un_Detected\n"));
		break;
	case (BATTERY_LEVEL):
//	    rec_bat_power=recBuf[0]<<8 |recBuf[1];
		rec_bat_power = *WLC1.Value << 8 | *(WLC1.Value + 1);
		rec_bat_power = (rec_bat_power / 2046) * 100;
	//	printf("Battery level %f", rec_bat_power);
		UART_Write(UART2,"Battery level%f\n",sizeof("Battery level%f\n"));
		bat_power=0;
		UART_Write(UART2,DBQUOTE,1);
		bat_power= floor(rec_bat_power);
		UART_Write(UART2,bat_power,sizeof(bat_power));
		UART_Write(UART2,DBQUOTE,1);
		break;
	case (HARDWARE_VERSION):
		//printf("Hard_ware_vertion%x\n", *WLC1.Value);
		UART_Write(UART2,"Hard_ware_vertion%x\n",sizeof("Hard_ware_vertion%x\n"));
	    UART_Write(UART2,DBQUOTE,1);
	    UART_Write(UART2,*WLC1.Value,sizeof(*WLC1.Value));
	    UART_Write(UART2,DBQUOTE,1);
		break;
	case (SOFTWARE_VERSION):
		//printf("Soft_ware_vertion%x\n", *WLC1.Value);
		UART_Write(UART2,"Soft_ware_vertion%x\n",sizeof("Soft_ware_vertion%x\n"));
	    UART_Write(UART2,DBQUOTE,1);
	    UART_Write(UART2,*WLC1.Value,sizeof(*WLC1.Value));
	    UART_Write(UART2,DBQUOTE,1);
		break;

	case (TAMPER_STATUS):
		if (*WLC1.Value == OPEN)
			//printf("Tamper_Open\n");
			UART_Write(UART2,"Tamper_Open\n",sizeof("Tamper_Open\n"));
		if (*WLC1.Value == CLOSE)
			//printf("Tamper_closed\n");
			UART_Write(UART2,"Tamper_closed\n",sizeof("Tamper_closed\n"));
		break;

	case (LED_STATUS):
		if (*WLC1.Value == ON)
		//	printf("Led_ON\n");
			UART_Write(UART2,"Led_ON\n",sizeof("Led_ON\n"));
		if (*WLC1.Value == OFF)
			//printf("Led_OFF\n");
			UART_Write(UART2,"Led_OFF\n",sizeof("Led_OFF\n"));
		break;
	case (WIRED_STATUS):
		if (*WLC1.Value == CONNECTED)
			//printf("Wire_Connected\n");
			UART_Write(UART2,"Wire_Connected\n",sizeof("Wire_Connected\n"));
		if (*WLC1.Value == DISCONNECTED)
		//	printf("Wire_Disconnected\n");
			UART_Write(UART2,"Wire_Disconnected\n",sizeof("Wire_Disconnected\n"));
		break;
	default:
		sendBuf[0] = UNSUCCESSFUL_ACKNOWLEDGEMENT;
		send(SMART_SWITCH, CONTROL_PANEL, SMART_SWITCH_ID, CONTROL_PANEL_ID,
				NACK, &sendBuf);
		break;
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
void gsm_get_snapshot() {
	switch (loopCount) {

	case 1:
		UART_Write(UART1, "AT+CGMR\r\n", sizeof("AT+CGMR\r\n"));
		break;
	case 2:
		UART_Write(UART1, "AT+CGSN\r\n", 9);
		break;
	case 3:
		UART_Write(UART1, "AT+CIMI\r\n", 9);
		loop_delay(1);
		break;
	case 4:
	//	readIMEI = 1;
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
		UART_Write(UART1, "AT+IPREX=9600\r\n", sizeof("AT+IPREX=9600\r\n"));
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
void gsm_datacall() {
	switch (loopCount) {

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
void gsm_connect() {
	switch (loopCount) {

	case 1:
		UART_Write(UART1, "AT+CGACT=1,1\r\n", 14);
		break;
	case 2:
		UART_Write(UART1, "AT+NETOPEN\r\n", 12);
		break;
	case 3:
		break;
	case 4:
		UART_Write(UART1, "AT+IPADDR\r\n", 11);
		break;
	case 5:
		UART_Write(UART1, "AT+NETOPEN\r\n", 12);
		//UART_Write(UART1, "\r\n",2);
		break;
	case 6:
		UART_Write(UART1, "AT+IPADDR\r\n", 11);
		break;
	case 7:
		UART_Write(UART1, "AT+IPADDR\r\n", 11);
		break;
	case 8:
		break;
	case 9:
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
 * @brief       gsm_query
 *
 * @param       None
 *
 * @return      None
 *
 * @details     Query if connected to apn, check IP.
 */
void gsm_query() {
	switch (loopCount) {
	case 1:
		UART_Write(UART1, "\r\n", 2);
		UART_Write(UART1, "AT+NETOPEN=1\r\n", 14);
		break;
	case 2:
		UART_Write(UART1, "AT+IPADDR\r\n", 11); // close
		break;
	case 3:
		UART_Write(UART1, "AT+NETOPEN\r\n", 12); // query check result
		break;
	case 4:
				gsmState = GSM_HTTP_REPORT_DATA;
				loopCount = 0;
		break;
	default:
	//	loopCount=0;
		break;
	}

}
/**
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
          // 	loop_delay(10);
      	break;
	case 2:
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);
		loop_delay(2);
		 RFMair();
		 memset(databbuf,0,sizeof(recBuf));
		break;
	case 3:
		UART_Write(UART1, "AT+HTTPPARA=",12);
			UART_Write(UART1, DBQUOTE, 1);
		    UART_Write(UART1, "URL",3);
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, ",",1);
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1,testURL,strlen(testURL));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, "\r\n", 2);
		break;

	case 4:
		        UART_Write(UART1, "AT+HTTPDATA=", 12);
				UART_Write(UART1,buffvalue , strlen(buffvalue));
				UART_Write(UART1, ",", 3);
				UART_Write(UART1, "100", 3);
				UART_Write(UART1, "\r\n", 2);

			break;
	case 5:
	      //	loop_delay(2);
		  //   RFMair();
			break;
	case 6:

            break;

	case 7:
		    loop_delay(2);
	    	UART_Write(UART1, testurl,strlen(testurl));
	 //   	UART_Write(UART2, testurl,strlen(testurl));
		   UART_Write(UART1, "\r\n",2);

			break;
	case 8:
	        	UART_Write(UART2, testurl,strlen(testurl));
	     	  UART_Write(UART2, "\r\n",2);
	       	UART_Write(UART1, "AT+HTTPACTION=1\r\n", 17);

	    break;
	case 9:
			    break;
	case 10:
			    break;
	case 11:
		UART_Write(UART1, "AT+HTTPHEAD\r\n",15);
		    break;
	case 12:
		UART_Write(UART1, "AT+HTTPREAD=0,500\r\n",18 );

			break;
	case 13:
		    break;

	case 14:
		UART_Write(UART1, "AT+HTTPTERM\r\n",13 );
		        break;

	case 15:
           //  memset(testurl,0,strlen(testurl));
			   break;
	case 16:
		gsmState =GSM_SLEEP;// SWITCH_COMMND;

		loopCount = 0;
		break;
	case 17:
		break;
	default:
		if(loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
	}

}

void switch_command () {
	switch (loopCount) {
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
					UART_Write(UART1,URLSWITCH,strlen(URLSWITCH));
					UART_Write(UART1, DBQUOTE,1);
					UART_Write(UART1, "\r\n", 2);
		break;
	case 4:
		UART_Write(UART1, "AT+HTTPACTION=0\r\n", 17);
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		UART_Write(UART1, "AT+HTTPHEAD\r\n",15);
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
		if (loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
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
		UART_Write_main(UART2, "Check Connectivity periodically\r\n", 33);
		gsmState = GSM_QUERY;
		loopCount = 0;
		break;
	default:
		if (loopDelay)
			loopDelay--;
		else
			loopCount = tloopCount;
		break;
		break;
	}

}

void loop_delay(uint8_t loopDel) {
	tloopCount = loopCount;
	loopCount = 0x5f;
	loopDelay = loopDel;
}

//void Alaram(uint32_t ONTIME,uint32_t OFFTIME,uint32_t TIMES)   // Beep Customize Function
//	{
//	 for(int i=1;i<=TIMES;i++)
//	 {
//		 beep_on();
//		// TIMER_Delay(TIMER2,ONTIME);
//		 delayCounter = 150000;
//		 		 		delay_loop();
//		 beep_off();
//		 TIMER_Delay(TIMER2,OFFTIME);
//		  }
//
//
//	}

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

//void beep_on()
//{
//	PE2 = 1;
////		PWM_EnableOutput(PWM0, 0x04);
////		PWM_Start(PWM0, 0x04);
////		beepDelay = 0;
//}

//void beep_off()
//{
//	PE2 = 0;
////		PWM_DisableOutput(PWM0, 0x04);
// //		beepDelay = 0;
//}

//void beep_on_off(uint8_t onTime, uint8_t offTime, uint16_t beepCount)
//{
//	beepOn = TRUE;
//	beepOnTime = onTime * 100;
//	beepOffTime = offTime * 100;
//	beepCounter = beepCount;
//
//}

//void beep_control()
//{
//
//	if(beepOn)
//	{
//		if(beepDelay == 0)
//		{
//			//beep on
//			beep_on();
//			beepDelay++;
//		}
//		else if((beepDelay >= beepOnTime) && (beepDelay < (beepOnTime +beepOffTime ) ))
//		{
//			//beepoff
//			beep_off();
//			beepDelay++;
//		}
//		else if(beepDelay >= (beepOnTime + beepOffTime))
//		{
//			//beepCount inc
//			beepDelay = 0;
//			noOfBeeps++;
////    			beepDelay++;
//			if(noOfBeeps > beepCounter)
//			{
//				beepOn = FALSE;
//				noOfBeeps = 0;
//				//beepOff
//				beep_off();
//				beepDelay = 0;
//			}
//		}
//		else
//		{
//			beepDelay++;
//		}
//
//	}
//}

/* check if config mode is on, if configMode is ON, do not write*/
uint32_t UART_Write_main(UART_T *uart, uint8_t *pu8TxBuf,
		uint32_t u32WriteBytes) {
	uint32_t u32Count = 0;

	if (!configMode)
		u32Count = UART_Write(uart, pu8TxBuf, u32WriteBytes);

	return u32Count;

}

