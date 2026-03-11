/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 7 $
 * $Date: 17/05/04 1:36p $
 * @brief    Configure SPI0 as Master mode and demonstrate how to communicate
 *           with an off-chip SPI Slave device with FIFO mode. This sample
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
#include <string.h>
#include <ctype.h>


#define BUF_LEN_SMALL 20
#define API_BUFF_LEN 50
#define URL_LENGTH 50
#define BUF_LEN 250
#define BUF_LEN_LONG 500

#define data_flash_size  38

#define TEST_COUNT 16
#define Data_count 16




#define   BUFSIZE              50
#define   REC_BUF_SIZE         15
#define   SEND_BUF_SIZE        15
#define   SPI_REC_BUF_SIZE     15
#define   UART_REC_BUF_SiZE    50
#define   WIRE_REC_BUF_SIZE    50
#define   WAKE_UP_BUF_SIZE     50

#define   SET_RE               PD4=1
#define   SET_DE               PD5=1
#define   CLR_RE               PD4=0
#define   CLR_DE               PD5=0


#define data_flash_addres 0x8050
#define vechicle_data_flash_addres 0xA3F0

#define BEEP_ON PD7= 1
#define BEEP_OFF PD7=0


#define phone_numbe_is_avaliable  0x33
#define vechical_number_avalible  0x34
#define icmi_number_avalible      0x35




void door_sensor_status();
void smart_switch_statuas();
void panic_status();
void motion_status();
void key_status();
void indoor_siren_status();
void register_the_mobile_number();
void sim_calling_function();

void  UART1_TEST_HANDLE();
void  default_at_commands();
void  TMR1_IRQHandler(void);

void gsm_init();
void gsm_get_snapshot();
void gsm_datacall();
void gsm_connect();
void gsm_query();
void simcard_checking_command();
//void simcard_data_flash_icmi(uint8_t );

void gsm_idle();
void gsm_http_report_data();
void send_data_and_time();

void compare_old_new_data();

void data_write_flash_write(uint8_t [],uint16_t,uint8_t );
int32_t *data_flash_read(uint16_t ,uint8_t );
void sensor_data_sending_to_sms();

void loop_delay(uint8_t loopDel);
void delay_loop();

uint32_t delayCounter = 200000;

uint8_t UART1_RX_BUF[BUF_LEN];
uint8_t UART1_TX_BUF[BUF_LEN];
uint8_t count_1;
uint8_t newWord_1 = 0;

uint8_t UART0_RX_BUF[BUF_LEN];
uint8_t UART0_TX_BUF[BUF_LEN];
uint32_t count_0;
uint8_t newWord_0 =0;

uint8_t *tGRMC;
uint8_t *tEndofRMC;
uint8_t GPS_data[100];
uint8_t newGpsData = 0;



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

uint8_t mobile_number_arry[25];
uint8_t *mobile_number_pointer=NULL;
uint8_t mobile_number_counter=0;
uint8_t data_index;

uint8_t calling_mobile_number[data_flash_size];
int32_t read_data_flash_buffer[10];

uint8_t recived_lorry_number[13];

uint8_t sim_icmi_number[15];

uint8_t long_buffer[40];

int32_t data_form_falsh[10];

uint8_t phone_counter=40;
uint8_t uart_data_registrion=0;
uint8_t uart_data_data=0;
uint8_t uart_clcok_read=0;
uint8_t data_count_1=0X90;

uint8_t *sim_card_test_pointer=0;
uint8_t sim_card_checking=0;
uint8_t sim_card_test_buffer[15];

uint8_t sim_card_error_checkingh=0;
uint8_t regisrter_mobile_number=0;
uint8_t get_the_icmi_new_data=0;

int seconds;
uint32_t secLatU;
uint32_t secLatL;

uint8_t checking=0;

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


uint8_t gpsLatitudeData[API_BUFF_LEN];
uint8_t gpsLongitudeData[API_BUFF_LEN];
uint8_t gpsAltitudeData[API_BUFF_LEN];
uint8_t gpsSpeedData[API_BUFF_LEN];
uint8_t gpsTimeData[API_BUFF_LEN];
uint8_t gpsDateData[API_BUFF_LEN];
uint8_t webURL[API_BUFF_LEN];
uint8_t testURL[BUF_LEN];

uint32_t u32RData;
uint32_t tempAddr;

uint8_t api_update[API_BUFF_LEN] = "uDIM?";
uint8_t gpsLatitude[API_BUFF_LEN]="lat";
uint8_t gpsLongitude[API_BUFF_LEN]="lng";
uint8_t gpsAltitude[API_BUFF_LEN]="&alt=";
uint8_t gpsSpeed[API_BUFF_LEN]="&sp=";
uint8_t gpsTime[API_BUFF_LEN]="";
uint8_t gpsDate[API_BUFF_LEN]="";

uint8_t gpsDebugData = 0;
uint8_t dataUpdateReady=0;

uint8_t sizof_send_buffer[10];
uint16_t sizof_send_buffer_interger;

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


uint8_t configMode = 0;
char DBQUOTE[1] ={0x22, 0x00};
char steing_end[1]={0x26};
uint32_t UART_Write_main(UART_T *uart, uint8_t *pu8TxBuf,uint32_t u32WriteBytes);
uint8_t fuel_sensor_buff[BUF_LEN];
uint8_t lorry_number_buff[13];
uint8_t send_value=0;
uint8_t send_data=0;
uint8_t uart_data=0;
uint8_t count_12=0;
extern uint8_t RFState;

uint16_t  batPower;
uint16_t  cal_power;
float     rec_bat_power = 0;
uint32_t   hard_ware_version = 0;
uint32_t   soft_ware_version= 0;
uint8_t    device_version[10];
uint8_t   uart_clock_data=0;


uint8_t   txFlag = 0;
uint8_t   recFlag = 0;

uint16_t  crc_rec = 0;
uint8_t   sequence = 0x01;
uint8_t   sendBuf[SEND_BUF_SIZE];

uint8_t   RFbuffer[BUFSIZE];                // RF buffer
uint8_t   Rec_cpy[BUFSIZE];

uint16_t  RFM;
uint32_t  Rec_swap_byte=0;

uint8_t date_and_time[SIZEOFVALUE];
uint8_t dates[20];
uint8_t times[4];

uint8_t   recBuf[REC_BUF_SIZE];
uint8_t   RFbufferSize;
uint8_t   strating_only=0;

uint8_t   spi_rec_data[SPI_REC_BUF_SIZE];
uint8_t   spi_rec_count = 0;

uint8_t Key1_Flag=0;
uint8_t rec_done=0;

uint8_t uatr_rec_buf[UART_REC_BUF_SiZE];
uint8_t uart_rec_count=0;
uint8_t ask_flag=0;
uint16_t rec_rssi=0;


uint8_t    wire_rec_buf[WIRE_REC_BUF_SIZE];
uint8_t    wire_rec_count=0;
uint8_t    rec_from_wire_done=0;
uint8_t    rec_from_wireless=0;


uint16_t   keypressed_value=0;
uint8_t    key_pressed[4];

uint32_t control_panel=CONTROL_PANEL;
uint16_t conv_32to_16_cp =0;
uint16_t conv32_16_dest=0;

//signal strength measured
uint16_t  singnal_strength=0;

uint8_t mobile_number_arry_1[10];

uint8_t loopCount = 0;
uint8_t tloopCount=0;
uint8_t loopDelay=0;
uint8_t timeOut = 0 ;


uint8_t mahaan=0;
uint8_t value[10];
uint8_t phone_number_buffer[BUF_LEN];
uint8_t phone_number_buffer_1[REC_BUF_SIZE];

uint8_t data_send_buffer[BUF_LEN];

uint8_t recived_mobile_number[10];
int32_t data_flash_phone_vecicale_iemi_buffer[10];
int32_t *mobile_number_recived;
uint8_t recived_mobile_number_counter=0;
uint8_t read_data_form_data_flash=1;

uint8_t set_date_time_buffer[BUF_LEN];
uint8_t *date_and_time_pointer=0;
uint8_t  year_buffer[9];
uint8_t  time_buffer[5];
uint8_t vechical_registion_buffer[BUF_LEN];
uint8_t *vechical_registion_pointer=0;

uint8_t num1[10] = "9032888261"/*"8688043920"*/;
uint8_t num2[10] ;

uint8_t password[11]="DJR2RITHIKA";
uint8_t recived_password[11];

uint8_t google_map_url[]="https://WWW.google.com/maps?q=";
uint8_t sms_google_buffer[100];
//send wake up messege
uint8_t wake_up_msg[WAKE_UP_BUF_SIZE]={"Hello_Device_this_is_Wake_up_messege_from_EDEL\n"};
/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);
void get_gps_data();
void date_time_function();
void vehicale_registrion();

/*****************uatr_1****************************/
void UART1_TEST_HANDLE();

// ***CRC calculation *****//
uint16_t crc_calcul(uint8_t buf[], int len)
{
	uint16_t crc = 0xFFFF;
	uint16_t crcLow, crcHigh;
    uint8_t pos=0,i;
  for ( pos = 0; pos <len ; pos++)
	{
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc

    for ( i = 8; i != 0; i--)
		{                                   // Loop over each bit
      if ((crc & 0x0001) != 0)
				{                               // If the LSB is set
					crc >>= 1;                    // Shift right and XOR 0xA001
					crc ^= 0xA001;
				}
      else                              // Else LSB is not set
        crc >>= 1;                      // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  crcLow = crc & 0xFF00;
  crcHigh = crc & 0x00FF;
  crcLow >>=8;
  crcHigh <<=8;
  crc = crcHigh | crcLow;
  return crc;
}
/******Send Data to Receiver *******/
void send(uint8_t source,uint8_t Dest, uint16_t sourId,uint16_t DestId,uint8_t Data,uint8_t *value)
{
	  uint8_t ReturnCode = -1;
	  uint16_t crc_send=0;

	  memset(RFbuffer,0,BUFSIZE);
	  WLC.SOF='%';
	  WLC.Source=source;
	  WLC.Destination=Dest;
	  WLC.sourceId=sourId;
	  WLC.DestId=DestId;
      WLC.Data=Data;
      memcpy(WLC.Value,value,SIZEOFVALUE);


	  memcpy(RFbuffer,&WLC,(sizeof(WLC_t)-2));
	  crc_send=crc_calcul(RFbuffer,(sizeof(WLC_t)-2));
	  WLC.crc=crc_send;
	  memcpy(RFbuffer +(sizeof(WLC_t)-2) ,&WLC.crc,sizeof( WLC.crc));

	  RFbufferSize=sizeof(WLC_t);
      SendRfFrame(RFbuffer,RFbufferSize, &ReturnCode);      // Sends the frame to the RF chip

  	  memset(&WLC,0,sizeof(WLC_t));
	  memset(RFbuffer,0,BUFSIZE);
}
/******Receive  Data From Sender *******/
void receive(void)
{
	uint8_t ReturnCode = -1;

	memset(RFbuffer,0,BUFSIZE);

	RFbufferSize=sizeof(WLC_t);
	ReceiveRfFrame(RFbuffer,&RFbufferSize, &ReturnCode);             // Receives the frame from the RF chip
	memcpy(Rec_cpy,RFbuffer+2,sizeof(WLC_t));
	memcpy(&WLC,Rec_cpy,sizeof(WLC_t));
	check_receive_packet();

}
// ***Reset RFM69 chip*****//
void reset_rfm69()
{
PD2=1;
TIMER_Delay(TIMER0,100);
PD2=0;
TIMER_Delay(TIMER0,100);
}
// ***Uart0 and Uart2 test handler*****//
void UART02_TEST_HANDLE()
{
	uint32_t u32IntSts = UART2->INTSTS;
	if (u32IntSts & UART_INTSTS_RDAINT_Msk)
	{

		/* Get all the input characters */
		while (UART_IS_RX_READY(UART2))
		{
			UART_Read(UART2, &uatr_rec_buf[uart_rec_count], 1);

			if ((uatr_rec_buf[uart_rec_count] == '\n')
					&& (uatr_rec_buf[uart_rec_count - 1] == '*')
					&& (uatr_rec_buf[uart_rec_count - 2] == '*'))
				Key1_Flag = 1;

			if ((uatr_rec_buf[uart_rec_count] == '\n') && (Key1_Flag == 0)
					&& (uart_rec_count > 3))
				ask_flag = 1;

			if (uart_rec_count > UART_REC_BUF_SiZE)
				uart_rec_count = 0;
			uart_rec_count++;
		}
	}
	UART2->INTSTS = 0;

	  u32IntSts = UART0->INTSTS;

	         if(u32IntSts & UART_INTSTS_RDAINT_Msk)
	         {

	             /* Get all the input characters */
	             while(UART_IS_RX_READY(UART0))
	             {
	               UART_Read(UART0, &UART0_RX_BUF[count_0], 1);
//	               if(gpsDebugData)
//	            	   UART_Write_main(UART2, &UART0_RX_BUF[count_0], 1);

//	             if(UART0_RX_BUF[count_0] == '\r')
//	             	newWord_0 = 1;

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
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC,115200);
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
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
    UART2->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC,115200);
    UART2->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}
void UART1_IRQHandler(void)
{
//	TIMER_DisableInt(TIMER1);
    UART1_TEST_HANDLE();
 //   TIMER_EnableInt(TIMER1);
}
void UART1_TEST_HANDLE()
{
    uint32_t u32IntSts = UART1->INTSTS;

    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART1))
        {
         UART_Read(UART1, &UART1_RX_BUF[count_1], 1);
         count_1 = (count_1 + 1) % BUF_LEN;
               data_count_1++;
           }

        if((strstr(UART1_RX_BUF,"+CMT: "))&&(!uart_data_data))
        {
        	data_count_1=0;
           uart_data_data=1;
        }
       if((strstr(UART1_RX_BUF,"+CCLK:"))&&(!uart_clock_data))
//    	   if((strstr(UART1_RX_BUF,"AT+CCLK?"))&&(!uart_clock_data))
	   {
    	   data_count_1=0;
	       uart_clock_data=1;
	   }
//       if((data_count_1>78)&&(data_count_1<=85)&&(uart_data_data))
//       if((strstr(UART1_RX_BUF,"+CMT: "))&&(data_count_1>78))
//        {
//        	uart_data_data=0;
//        	uart_data_registrion=1;
//        	count_1=0;
//			data_count_1=0x90;
//			memcpy(phone_number_buffer,UART1_RX_BUF,sizeof(UART1_RX_BUF));
//			memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
//        }
//        if((data_count_1>16)&&(data_count_1<=17)&&(uart_clock_data))
////       if((strstr(UART1_RX_BUF,"+CCLK:"))&&(data_count_1>16))
////       if((strstr(UART1_RX_BUF,"AT+CCLK?"))&&(data_count_1>23))
//        {
//        	uart_clock_data=0;
//        	uart_clcok_read=1;
//        	count_1=0;
//			data_count_1=0x90;
//			memcpy(set_date_time_buffer,UART1_RX_BUF,sizeof(UART1_RX_BUF));
//			memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
//        }

    }
    uart_data=1;
    UART1->INTSTS =0;


}
/*---------------------------------------------------------------------------------------------------------*/
/* Init SPI0                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/

void SPI_Init(void)
{
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 1600000);

    SPI_SetFIFO(SPI0, 0, 0);
    SPI_EnableInt(SPI0, SPI_FIFO_RXTH_INT_MASK );
    NVIC_EnableIRQ(SPI0_IRQn);

}
/*---------------------------------------------------------------------------------------------------------*/
/* Init SPI0 Interrupt Handler                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/

void SPI0_IRQHandler(void)
{
    /* Check RX EMPTY flag */
    if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
    {
        /* Read RX FIFO */
    	spi_rec_data[spi_rec_count++] = SPI_READ_RX(SPI0);
        if(spi_rec_count > SPI_REC_BUF_SIZE)
        {
        	spi_rec_count=0;
        	memset(spi_rec_data,0,SPI_REC_BUF_SIZE);
        }
    }
}
/* GPIO pin(DIO0) interrupt used for send and receive  */
void GPCDEF_IRQHandler(void)
{

    if(GPIO_GET_INT_FLAG(PD, BIT3))
    {
       GPIO_CLR_INT_FLAG(PD, BIT3);
 	   if(PD3)
 		{
			RFState |= RF_RX_DONE;
			RFState &= ~RF_BUSY;
			if(txFlag)
			{
				txFlag=0;
				recFlag=0;
			}
			else
			{
				recFlag=1;
			}
 		}
 		else
 		{
			RFState |= RF_STOP;
			RFState &= ~RF_RX_DONE;
 		}
    }
  if(GPIO_GET_INT_FLAG(PC, BIT0))
  {
	  GPIO_CLR_INT_FLAG(PC, BIT0);
	  if(PC0)
		  BEEP_OFF;
  }
}
void UART02_IRQHandler(void)
{
    UART02_TEST_HANDLE();
}
uint32_t UART_Write_main(UART_T *uart, uint8_t *pu8TxBuf,uint32_t u32WriteBytes)
{
	uint32_t u32Count = 0;

	if (!configMode)
		u32Count = UART_Write(uart, pu8TxBuf, u32WriteBytes);

	return u32Count;

}
void TMR1_IRQHandler(void)
{
	if (TIMER_GetIntFlag(TIMER1) == 1)
	{
		/* Clear Timer0 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER1);

		send_value++;
		  loopCount++;
		  timeOut = 1;
	}
}
void SYS_Init(void)
{
	/* Enable UART module clock */
	CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk
			| CLK_APBCLK0_UART2CKEN_Msk);

	/* Select UART module clock source as HIRC and UART module clock divider as 1 */
	CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk))
			| CLK_CLKSEL1_UARTSEL_HIRC;
	CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk))
			| CLK_CLKDIV0_UART(1);

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_UART0_TXD | SYS_GPA_MFPL_PA3MFP_UART0_RXD);

    /* Set PD multi-function pins for UART1 RXD and TXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk );
   	  SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_UART1_TXD | SYS_GPA_MFPL_PA1MFP_UART1_RXD );

	/* Set PB multi-function pins for UART2 TXD(PC.2) and RXD(PC.3) */
	SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk)) | SYS_GPC_MFPL_PC2MFP_UART2_TXD;
	SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_UART2_RXD;

    /* Setup SPI0 multi-function pins */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk |SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB4MFP_SPI0_SS | SYS_GPB_MFPL_PB5MFP_SPI0_MOSI | SYS_GPB_MFPL_PB6MFP_SPI0_MISO | SYS_GPB_MFPL_PB7MFP_SPI0_CLK;


	//timer0 initial
	 CLK_EnableModuleClock(TMR0_MODULE);
	 CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);


	 CLK_EnableModuleClock(TMR1_MODULE);
	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);

    /* Gpio init for rfm69 trans receive */
	    GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);
	    PD2=0;

//	/* Gpio init for wire RE/DE receive */
//		GPIO_SetMode(PD, BIT4, GPIO_MODE_OUTPUT);
//		GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
//		PD4=0;
//		PD5=0;
	    /* Gpio init for rfm69 trans receive */
	    	    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
	    	    GPIO_EnableInt(PC, 0, GPIO_INT_RISING);

	/* Gpio init for rfm69 trans receive */
	    GPIO_SetMode(PD, BIT3, GPIO_MODE_INPUT);
	    GPIO_EnableInt(PD, 3, GPIO_INT_BOTH_EDGE);
	    NVIC_EnableIRQ(GPCDEF_IRQn);


//	    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
//	    GPIO_ENABLE_DEBOUNCE(PD, BIT3);
	    GPIO_SetMode(PD, BIT7, GPIO_MODE_OUTPUT);
//	    GPIO_EnableInt(PC, 1, GPIO_INT_BOTH_EDGE);
//	    NVIC_EnableIRQ(GPCDEF_IRQn);
}
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();


    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART0_Init();
    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk ));


    UART1_Init();
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk ));

  //  NVIC_EnableIRQ(UART1_IRQn);
  //  UART_Write(UART1, "AT\r\n", 4);

//    UART_Write(UART1, "AT\r\n", strlen("AT\r\n"));

    /* Init SPI */
       SPI_Init();

    /* Configure UART2: 115200, 8-bit word, no parity bit, 1 stop bit. */

    UART2_Init();
    UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk ));

	TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 8);
	TIMER_EnableInt(TIMER1);

	/* Enable Timer0 ~ Timer3 NVIC */
	NVIC_EnableIRQ(TMR1_IRQn);

	/* Start Timer0 ~ Timer3 counting */
	TIMER_Start(TIMER1);


    UART_Write(UART2, "EDEL SMART DEVICES - 001\r\n", 26);
 //  NVIC_EnableIRQ(UART02_IRQn);

    /* reset RFM69 */
//    reset_rfm69();
//    /* Init RFM69 */
//    InitRFChip();

//    sim_calling_function();

//    delayCounter = 100000;
//   	 delay_loop();


    default_at_commands();

   simcard_checking_command();



while(1)
{

    if(uart_data)
     {
   	  uart_data=0;
   	  if(count_12>count_1)
   	      count_12=count_1-1;

		while(count_12<count_1)
		{

			UART_Write(UART2, &UART1_RX_BUF[count_12++], 1);
			if(count_12>BUF_LEN)
				count_12=0;

		}
		if(memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "AT+CIMI",strlen("AT+CIMI"))&&(count_1>16))
		{
			sim_card_test_pointer=memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "AT+CIMI", strlen("AT+CIMI"));

//			sim_card_test_pointer=strstr(UART1_RX_BUF,"AT+CIMI");

			memcpy(sim_card_test_buffer,sim_card_test_pointer+10,(sizeof(sim_card_test_buffer)));

			sim_card_error_checkingh=0;

			sim_card_test_pointer=NULL;

			if(memmem(sim_card_test_buffer, sizeof(sim_card_test_buffer), "ERROR", 5) != NULL)
			{
				uart_data=0;
				UART_Write(UART1, "AT+CIMI\r\n",strlen("AT+CIMI\r\n"));
				sim_card_error_checkingh=1;
				memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
			}
		     if((regisrter_mobile_number)&&(!sim_card_error_checkingh))
			{
		    	 regisrter_mobile_number=0;

				// wirte the data flsh the icmi sim card unique number
			data_write_flash_write(sim_card_test_buffer,icmi_number_avalible,38);

			memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));

			}
		     if(!sim_card_error_checkingh)
			{

				get_the_icmi_new_data=1;
			}
		}
		if(memmem(sim_card_checking, sizeof(UART1_RX_BUF), "SIM REMOVED",strlen("SIM REMOVED")))
		{
			  sim_card_checking=1;

		}
//		 if((data_count_1>16)&&(data_count_1<=17)&&(uart_clock_data))
//       if((strstr(UART1_RX_BUF,"+CCLK:"))&&(data_count_1>16))
		  if((strstr(UART1_RX_BUF,"AT+CCLK?"))&&(data_count_1>23))
			{
				uart_clock_data=0;
				uart_clcok_read=1;
				count_1=0;
				data_count_1=0x90;
				memcpy(set_date_time_buffer,UART1_RX_BUF,sizeof(UART1_RX_BUF));
				memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
			}
//	      if((strstr(UART1_RX_BUF,"+CMT: "))&&(data_count_1>78))
//	    	if((strstr(UART1_RX_BUF,"+CMT: "))&&(data_count_1>66))
		  if(memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CMT: ",strlen("+CMT: ")))
	        {
//			  sim_card_test_pointer=strstr(&UART1_RX_BUF,"+CMT: ");
			  sim_card_test_pointer =memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CMT: ",strlen("+CMT: "));
	        	uart_data_data=0;
	        	uart_data_registrion=1;
	        	count_1=0;
				data_count_1=0x90;
				memcpy(phone_number_buffer,sim_card_test_pointer,sizeof(UART1_RX_BUF));
				memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
	        }

		uart_data=0;
	}
    if(uart_data_registrion)
    {
    	uart_data_registrion=0;

    	register_the_mobile_number();

    //	count_1=0;
    }
    if(uart_clcok_read)
    {
    	uart_clcok_read=0;

    	 date_time_function();

    	 memset(set_date_time_buffer,0,sizeof(set_date_time_buffer));
    	 //	memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));

    //	 count_1=0;
    }
 if( read_data_form_data_flash)
 {
	 read_data_form_data_flash=0;

	 memcpy(data_flash_phone_vecicale_iemi_buffer,data_flash_read(vechicle_data_flash_addres,data_flash_size),/*data_flash_size*/40);

	 memcpy(long_buffer,data_flash_phone_vecicale_iemi_buffer,sizeof(data_flash_phone_vecicale_iemi_buffer));

	 memcpy(recived_mobile_number,long_buffer,10);

	 memcpy(recived_lorry_number,long_buffer+11,13);

	 memcpy(sim_icmi_number,long_buffer+25,15);

 }
//  if((send_value>20)&&(send_data))
//  {
//	  send_value=0;
//	  send_data=0;
////	  sensor_data_sending_to_sms();
//
//  }
 if(recFlag)
 {
	recFlag=0;
	receive();
 }
 else if(rec_from_wire_done)
	  receive();
 else
 {
   receiveInit();
 }
 if(newGpsData > 10)
	{
		UART_Write_main(UART2,"\r\n",2);
		UART_Write_main(UART2,"NEW GPS DATA\r\n",14);
		UART_Write_main(UART2,GPS_data, strlen(GPS_data));
		UART_Write_main(UART2,"\r\n\r\n",4);
		get_gps_data();
		memset(GPS_data,0,100);
		newGpsData = 0;
	}
	if((get_the_icmi_new_data))
	{
		get_the_icmi_new_data=0;

		// to copmare the data after power supply reset icmi number is not same  blink the red led

		if(memcmp(sim_card_test_buffer, sim_icmi_number, 15)!= 0)
		{
			sim_card_checking=1;
		}
	//	compare_old_new_data();
	}
  if(sim_card_checking)
	{
//		UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk ));
//     memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
////		//or
//		while(1);
//		{
//			//blink the red led to indicate the sim is  chnanged
//		}

  	sim_card_checking=0;
//		UART_Write(UART1, "AT+CFUN=0\r\n",strlen("AT+CFUN=0\r\n"));


	}

	if (timeOut)
	{
		timeOut=0;
        switch (gsmState) {

  			case GSM_INIT:
  				gsm_init();
//  				  WDT_RESET_COUNTER();
  				break;

  			case GSM_SNAPSHOT:
  				gsm_get_snapshot();
//  				  WDT_RESET_COUNTER();
  				break;

  			case GSM_DATACALL:
  				gsm_datacall();
//  				  WDT_RESET_COUNTER();
  				break;

  			case GSM_CONNECT:
  				gsm_connect();
//  				  WDT_RESET_COUNTER();
  				break;

  			case GSM_QUERY:
  				gsm_query();
//  				  WDT_RESET_COUNTER();
  				break;

  			case GSM_HTTP_REPORT_DATA:
  				gsm_http_report_data();
//  				  WDT_RESET_COUNTER();
  				break;

//  			case GSM_HTTP_GET_LOCK_STATUS:
//  				gsm_http_get_lock_status();
////  				  WDT_RESET_COUNTER();
//  				break;

//  			case GSM_HTTP_SEND_LOCK:
//  				gsm_http_send_lock_status();
////  				  WDT_RESET_COUNTER();
//  				break;

//  			case GSM_HTTP_SEND_UNLOCK:
//  				gsm_http_send_unlock_status();
////  				  WDT_RESET_COUNTER();
//  				break;

//  			case GSM_HTTP_SEND_LIGHTON:
//  				gsm_http_send_lightON_status();
////  				  WDT_RESET_COUNTER();
//  				break;

//  			case GSM_HTTP_SEND_LIGHTOFF:
//  				gsm_http_sending_lightOff_status();
////  				  WDT_RESET_COUNTER();
//  				break;

//  			case GSM_HTTP_SEND_BEEPON:
//  				gsm_http_send_beepOn_status();
////  				  WDT_RESET_COUNTER();
//  				break;

//  			case GSM_HTTP_SEND_BEEPOff:
//  				gsm_http_send_beepOff_status();
////  				  WDT_RESET_COUNTER();
//  				break;
//
//  			case GSM_HTTP_SEND_REG:
//  				gsm_http_send_registartion();
////  				  WDT_RESET_COUNTER();
//  				break;

  			case GSM_SLEEP:
  				gsm_idle();
//  				  WDT_RESET_COUNTER();
  				break;

  			default:
//  				  WDT_RESET_COUNTER();
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
	//	readIMEI = 1;
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
//		UART_Write(UART1, "AT+IPREX=9600\r\n", sizeof("AT+IPREX=9600\r\n"));
		break;
	case 9:
		UART_Write(UART1, "AT+COPS?\r\n", sizeof("AT+COPS?\r\n"));
		loop_delay(5);
		break;
	case 10:
		UART_Write(UART1, "AT+CREG?\r\n",sizeof("AT+CREG?\r\n" ));
	//	POWERRELAY_ON;
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
		loopCount = 0;
//		if(loopDelay)
//			loopDelay--;
//		else
//			loopCount = tloopCount;
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
//		loop_delay(5);
		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);

		break;
	case 2:
//		UART_Write(UART1, "AT+HTTPPARA=",12);
////					UART_Write(UART1, DBQUOTE, 1);
////					UART_Write(UART1, "URL",3);
////					UART_Write(UART1, DBQUOTE, 1);
////					UART_Write(UART1, ",",1);
//					UART_Write(UART1, DBQUOTE,1);
//					UART_Write(UART1, "CID",strlen("CID"));
//					UART_Write(UART1, DBQUOTE,1);
//					UART_Write(UART1, ",",2);
//					UART_Write(UART1, "1",strlen("1"));
//					UART_Write(UART1, "\r\n",2);
//		loop_delay(5);

//		UART_Write(UART1, "AT+HTTPSSL=1",strlen("AT+HTTPSSL=1"));
////		UART_Write(UART1, DBQUOTE, 1);
////		UART_Write(UART1, "1",1);
////		UART_Write(UART1, DBQUOTE, 1);
/////		UART_Write(UART1, ",",1);
//		UART_Write(UART1, "\r\n",2);


		break;
	case 3:

//		loop_delay(5);
			UART_Write(UART1, "AT+HTTPPARA=",12);
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, "URL",3);
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, ",",1);
			UART_Write(UART1, DBQUOTE,1);
			//if(dataUpdateReady)
			{
			//	dataUpdateReady = 0;
//				UART_Write(UART1,webURL,sizeof(webURL));
//				UART_Write(UART1,"https://jsonplaceholder.typicode.com/posts",strlen("https://jsonplaceholder.typicode.com/posts"));
				UART_Write(UART1,"http://api.djr2innovationpvtltd.com/api/iot/events?deviceIMEI=123456789101234",strlen("https://api.djr2innovationpvtltd.com/api/iot/events?deviceIMEI=123456789101234"));
//				UART_Write(UART1,url_tset_buffer,strlen(url_tset_buffer));
//				UART_Write(UART1,"http://tracking-backend-5nzh.onrender.com/api/v1/sensors/EDL-DEV-7236E9/data",strlen("https://tracking-backend-5nzh.onrender.com/api/v1/sensors/EDL-DEV-7236E9/data"));
//				UART_Write(UART1,"sensor-data",strlen("sensor-data"));
				UART_Write(UART1, DBQUOTE,1);
				UART_Write(UART1, "\r\n",2);

			}
		break;
	case 4:
		UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "CONTENT",strlen("CONTENT"));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, ",",1);
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "application/json",strlen("application/json"));
//		UART_Write(UART1, " ;charset=UTF-8",strlen("; charset=UTF-8"));
		UART_Write(UART1, DBQUOTE,1);
		UART_Write(UART1, "\r\n",2);
	//	loop_delay(1);
		break;
	case 5:

		break;
	case 6:


		break;
	case 7:

		break;
	case 8:
	//	if(dataUpdateReady)
		{

		UART_Write(UART1, "AT+HTTPDATA=",strlen("AT+HTTPDATA="));
		UART_Write(UART1, sizof_send_buffer,strlen(sizof_send_buffer));
//		UART_Write(UART1, "38",strlen("59"));
		UART_Write(UART1, ",",1);
		UART_Write(UART1, "1000",strlen("1000"));
		UART_Write(UART1, "\r\n",2);

		}

    	break;
	case 9:


		break;

 	case 10:

	case 11:
	//	if(dataUpdateReady)
		{
		UART_Write(UART1,data_send_buffer,strlen(data_send_buffer));
		UART_Write(UART1, "\r\n",strlen("\r\n"));
//		dataUpdateReady=0;
		}

	//	loop_delay(1);
		break;
	case 12:


		break;

	case 13:
		UART_Write(UART1, "AT+HTTPACTION=1\r\n", strlen("AT+HTTPACTION=1\r\n"));


	    break;
	case 14:

		break;
	case 15:
		gsmState=GSM_SLEEP;
//		gsmState = GSM_HTTP_GET_LOCK_STATUS;
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
		UART_Write_main(UART2, "\nCheck Connectivity periodically\r\n",strlen("\nCheck Connectivity periodically\r\n") );
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

void check_receive_packet()
{
	if (WLC.SOF == '%') {
				crc_rec = crc_calcul(Rec_cpy, sizeof(WLC_t) - 2);
				RFM = crc_rec;
	//			crc_rec = crc_rec >> 8;
	//			crc_rec |= RFM << 8;

				if (crc_rec == WLC.crc){
					if (WLC.Destination == CONTROL_PANEL) {
				{
					 UART_Write(UART1, "AT+CCLK?\r\n", strlen("AT+CCLK?\r\n"));
					  memset(fuel_sensor_buff,0,sizeof(fuel_sensor_buff));
						switch (WLC.Data)
						{
						case (CAP_STATUS):
						if ((WLC.Value[0]) == OPEN_CAP)
						{
							 BEEP_ON;
//							strcat(fuel_sensor_buff,lorry_number_buff);
//							strncpy(fuel_sensor_buff,lorry_number_buff,13);
							memcpy(fuel_sensor_buff,recived_lorry_number,13);
							strcat(fuel_sensor_buff,"\n");
						//	strcat(fuel_sensor_buff,"Alert!vechicel Started:");
							strcat(fuel_sensor_buff,"Fuel Tank Cap Open");
							strcat(fuel_sensor_buff,"\n");
							strcat(fuel_sensor_buff,"Date:");
							year_buffer[sizeof(year_buffer)] ='\0';
							strcat(fuel_sensor_buff,year_buffer);
							strcat(fuel_sensor_buff,"Time:");
							strcat(fuel_sensor_buff,time_buffer);
							strcat(fuel_sensor_buff,"\n");
						    strcat(fuel_sensor_buff,sms_google_buffer);
						}
						if ((WLC.Value[0]) == CLOSE_CAP)
						{
							 BEEP_OFF;
//							strcat(fuel_sensor_buff,lorry_number_buff);
//							strncpy(fuel_sensor_buff,lorry_number_buff,13);
							memcpy(fuel_sensor_buff,recived_lorry_number,13);
							strcat(fuel_sensor_buff,"\n");
						//	strcat(fuel_sensor_buff,"Alert!vechicel Started:");
							strcat(fuel_sensor_buff,"Fuel Tank Cap Close");
							strcat(fuel_sensor_buff,"\n");
							strcat(fuel_sensor_buff,"Date:");
							year_buffer[sizeof(year_buffer)] ='\0';
							strcat(fuel_sensor_buff,year_buffer);
							strcat(fuel_sensor_buff,"Time:");
							strcat(fuel_sensor_buff,time_buffer);
							strcat(fuel_sensor_buff,"\n");
						    strcat(fuel_sensor_buff,sms_google_buffer);
//						    UART_Write(UART1, "AT+CCLK?\r\n", strlen("AT+CCLK?\r\n"));
						}
							break;
						case (BATTERY_VALUE):
//						itoa((WLC.Value[0]),value,10);
//						 strncpy(fuel_sensor_buff,lorry_number_buff,13);
//						strcat(fuel_sensor_buff,lorry_number_buff);
		                memcpy(fuel_sensor_buff,recived_lorry_number,13);
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Battery level=");
						strcat(fuel_sensor_buff,WLC.Value/*value*/);
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Date:");
						year_buffer[sizeof(year_buffer)] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
						strcat(fuel_sensor_buff,time_buffer);
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,sms_google_buffer);
//    					UART_Write(UART1, "AT+CCLK?\r\n", strlen("AT+CCLK?\r\n"));
							break;
						case (FLOW_SENSOR):
//						itoa((WLC.Value[0]),value,10);
//						strcat(fuel_sensor_buff,lorry_number_buff);
		                memcpy(fuel_sensor_buff,recived_lorry_number,13);
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Flow Sensor Count=");
						strcat(fuel_sensor_buff,WLC.Value/*value*/);
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Date:");
						year_buffer[sizeof(year_buffer)] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
						strcat(fuel_sensor_buff,time_buffer);
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,sms_google_buffer);
//						UART_Write(UART1, "AT+CCLK?\r\n", strlen("AT+CCLK?\r\n"));
							break;
						case (FUEL_GUAGE):
				//		itoa((WLC.Value[0]),value,10);
						memcpy(fuel_sensor_buff,recived_lorry_number,13);
		              //strncpy(fuel_sensor_buff,lorry_number_buff,13);
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Fuel level=");
						strcat(fuel_sensor_buff,WLC.Value/*value*/);
						strcat(fuel_sensor_buff,"L");
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Date:");
						year_buffer[sizeof(year_buffer)] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
						strcat(fuel_sensor_buff,time_buffer);
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,sms_google_buffer);
//						UART_Write(UART1, "AT+CCLK?\r\n", strlen("AT+CCLK?\r\n"));
							break;
						case (THAFT):
					   if(knotspeed<=5)
						{
		                BEEP_ON;
//		                strcat(fuel_sensor_buff,lorry_number_buff);
//		                strncpy(fuel_sensor_buff,lorry_number_buff,13);
		                memcpy(fuel_sensor_buff,recived_lorry_number,13);
						strcat(fuel_sensor_buff,"\n");
		                strcat(fuel_sensor_buff,"DROP:");
		                strcat(fuel_sensor_buff,WLC.Value/*value*/);
		                strcat(fuel_sensor_buff,"Phone:+");
		                strcat(fuel_sensor_buff,recived_mobile_number);
//		                memcpy(fuel_sensor_buff,recived_mobile_number,sizeof(recived_mobile_number));
		            	strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Date:");
						year_buffer[sizeof(year_buffer)] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
						strcat(fuel_sensor_buff,time_buffer);
		                strcat(fuel_sensor_buff,"\n");
		                strcat(fuel_sensor_buff,sms_google_buffer);
//		                UART_Write(UART1, "AT+CCLK?\r\n", strlen("AT+CCLK?\r\n"));
						}
						break;

						//					default:
					//						break;
						}
						sensor_data_sending_to_sms();
						memset(&WLC, 0, sizeof(WLC_t));
						 send_data=1;
					}

				}
			}
		}
}
void panic_alert()
{
//	 strcat(fuel_sensor_buff,lorry_number_buff);
//    strncpy(fuel_sensor_buff,lorry_number_buff,13);
	 memcpy(fuel_sensor_buff,recived_lorry_number,13);
	 strcat(fuel_sensor_buff,"\n");
	 strcat(fuel_sensor_buff,"Panic Alert!Help Needed:");
	 strcat(fuel_sensor_buff,"\n");
	 strcat(fuel_sensor_buff,"Immediately");
	 strcat(fuel_sensor_buff,"Phone:+");
	 memcpy(fuel_sensor_buff,recived_mobile_number,/*sizeof(recived_mobile_number)*/13);
//	 strcat(fuel_sensor_buff,"\n");
//	 strcat(fuel_sensor_buff,"Date:");
//	 strcat(fuel_sensor_buff,year_buffer);
//	 strcat(fuel_sensor_buff,"Time:");
//	 strcat(fuel_sensor_buff,time_buffer);
	 strcat(fuel_sensor_buff,"\n");
	 strcat(fuel_sensor_buff,sms_google_buffer);
}

void sensor_data_sending_to_sms()
{
	//if(send_value>30)
	{

//		if(strating_only)
//		{
//			strating_only=0;
//		  UART_Write(UART1, "AT+CMGD=1,", strlen("AT+CMGD=1,"));
//		//UART_Write(UART1, DBQUOTE, 1);
//		  UART_Write(UART1, "4", strlen("4"));
//		//UART_Write(UART1, DBQUOTE, 1);
////	      UART_Write(UART1, ",", sizeof(","));
////		  UART_Write(UART1, "2", sizeof("2"));
//		  UART_Write(UART1, "\r\n", 2);
//		}
//

//		   UART_Write(UART1, "AT+CNMI=", sizeof("AT+CNMI="));
//			//UART_Write(UART1, DBQUOTE, 1);
//			UART_Write(UART1, "2", sizeof("2"));
//			//UART_Write(UART1, DBQUOTE, 1);
//			UART_Write(UART1, ",", sizeof(","));
//			UART_Write(UART1, "2", sizeof("2"));
//			UART_Write(UART1, "\r\n", 2);

//if(strating_only)
//{
//	strating_only=0;
//	UART_Write(UART1, "AT+CMGL=", strlen("AT+CMGL="));
//	UART_Write(UART1, DBQUOTE, 1);
//	UART_Write(UART1, "ALL", strlen("ALL"));
//	UART_Write(UART1, DBQUOTE, 1);
//	UART_Write(UART1, "\r\n", 2);
//}

//	TIMER_DisableInt(TIMER1);

//	UART_Write(UART1, "AT+HTTPTERM\r\n", 13);

//	TIMER_Delay(TIMER0,100000);

//	delayCounter = 100000;
//	delay_loop();

//	UART_Write(UART1, "AT+CMGF=1", strlen("AT+CMGF=1"));
//	UART_Write(UART1, "\r\n", 2);

//	TIMER_Delay(TIMER0,100000);

	delayCounter = 1000000;
	delay_loop();

	UART_Write(UART1, "AT+CMGS=", strlen("AT+CMGS="));
	UART_Write(UART1, DBQUOTE, 1);
	UART_Write(UART1, "9032888261", strlen("9032888261"));
	UART_Write(UART1, DBQUOTE, 1);
	UART_Write(UART1, "\r\n", 2);

//	TIMER_Delay(TIMER0,100000);
	delayCounter = 1000000;
	delay_loop();

	//memcpy(fuel_sensor_buff,data_buffer,strlen(testURL));

	fuel_sensor_buff[strlen(fuel_sensor_buff)]=0X1A;

	UART_Write(UART1,fuel_sensor_buff , sizeof(fuel_sensor_buff));

	//TIMER_EnableInt(TIMER1);

//	delayCounter = 1000000;
//		delay_loop();

	//UART_Write(UART1, "AT+HTTPINIT\r\n", 13);

	memset(fuel_sensor_buff,0,sizeof(fuel_sensor_buff));

	}
}

void get_gps_data()
{
	UART_Write_main(UART2,"\r\n", 2);

	memset(lat_deg,0,2);
	memset(lat_min,0,2);

	memset(LAT_min,0,10);

	memset(minLat,0,sizeof(minLat));
	memset(minLong,0,sizeof(minLong));


	memset(lat_sec,0,sizeof(lat_sec));
	memset(long_deg,0,sizeof(long_deg));
	memset(long_min,0,sizeof(long_min));

	memset(LONG_min,0,10);

	memset(long_sec,0,sizeof(long_sec));
	memset(secLatUStr,0,sizeof(secLatUStr));
	memset(secLatLStr,0,sizeof(secLatLStr));
	memset(secLongUStr,0,sizeof(secLongUStr));
	memset(secLongLStr,0,sizeof(secLongLStr));


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

//		memset(testURL,0,BUF_LEN);

		memset(sms_google_buffer,0,sizeof(sms_google_buffer));

	  	memset(data_send_buffer,0,BUF_LEN);




//	  	strcpy(data_send_buffer, "{");
//	  	strcat(data_send_buffer, "\"");
//	  	strcat(data_send_buffer, "type");
//	  	strcat(data_send_buffer, "\"");
//	  	strcat(data_send_buffer, "}");

		 	strcat(data_send_buffer,"{");
			strcat(data_send_buffer, "\"");

		 	strcat(data_send_buffer,"type");

			strcat(data_send_buffer, "\"");
		 	strcat(data_send_buffer,":");
		 	strcat(data_send_buffer," ");
			strcat(data_send_buffer, "\"");
		 	strcat(data_send_buffer,"FUEL_TOUCHED");
			strcat(data_send_buffer, "\"");
		 	strcat(data_send_buffer,",");

			strcat(data_send_buffer, "\"");
				strcat(data_send_buffer,gpsLatitude);
				strcat(data_send_buffer, "\"");
				strcat(data_send_buffer,":");



				lat_dege=atoi(lat_deg);

				 if((lat_dege>7) && (lat_dege<38))
				 {
				   strcat(data_send_buffer,lat_deg);

					strcat(data_send_buffer,".");

					 strcat(data_send_buffer,LAT_min_cov);

					   lat_dege=0;
				 }
				 strcat(data_send_buffer,",");

					strcat(data_send_buffer, "\"");
				strcat(data_send_buffer,gpsLongitude);

				strcat(data_send_buffer, "\"");
				strcat(data_send_buffer,":");


				long_dege=atoi(long_deg);

				 if((long_dege>60) && (long_dege<97))
				 {

			         	strcat(data_send_buffer,long_deg+1);

			               	strcat(data_send_buffer,".");

			              	strcat(data_send_buffer,LONG_min_cov);

					         long_dege=0;
				    }


				 strcat(data_send_buffer,"}");


				 sizof_send_buffer_interger=strlen(data_send_buffer);

			     itoa(sizof_send_buffer_interger,sizof_send_buffer,10);

		dataUpdateReady = 1;

		UART_Write_main(UART2,"\r\n\r\n", 4);
		UART_Write_main(UART2,"/******************/\r\n", 22);
		UART_Write_main(UART2,data_send_buffer,strlen(data_send_buffer));
		UART_Write_main(UART2,"\r\n", 2);
		UART_Write_main(UART2,"/******************/\r\n", 22);
		UART_Write_main(UART2,"\r\n\r\n:", 4);
   	//	BEEPER_ON;
//		beep_on();
//    	delayCounter = 20000;
//    	delay_loop();
//    	BEEPER_OFF;
	//	if(date+1!=date_and_time)
		{

//			dates[0]=*(date+1);
//			dates[1]=*(date+2);
//			dates[2]= *(date+3);
//			dates[3]= *(date+4);
//			dates[4]= *(date+5);
//			dates[5]= *(date+6);
//
//			memcpy(dates,(date+1),6);
//
////			dates[6]= *(rtc+1);
////			dates[7]= *(rtc+2);
////			dates[8]= *(rtc+3);
////			dates[9]= *(rtc+4);
////			dates[10]= *(rtc+5);
////			dates[11]= *(rtc+6);
//
//		memcpy(dates+6,(rtc+1),4);

		// date_and_time=atoi(times);

		send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,dates);

	   // date_and_time=atoi(dates);

	//    send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,date_and_time);

	//    send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,date_and_time);


		//send_data_and_time();
		}
	}
}
void send_data_and_time()
{
	send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,date_and_time);
}
void loop_delay(uint8_t loopDel)
{
	tloopCount = loopCount;
	loopCount = 0x5f;
	loopDelay = loopDel;
}
void delay_loop()
{
    while(delayCounter)
    {

    	delayCounter--;
    }

}

void data_write_flash_write(uint8_t *mobile_number_arry,uint16_t data_flash_data,uint8_t lenght)
{

			SYS_UnlockReg();
			/* Enable FMC ISP functions */
			FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_LDUEN_Msk | FMC_ISPCTL_CFGUEN_Msk;

			for(data_index = 0; data_index < lenght/4+1; data_index++)
			{
				tempAddr = vechicle_data_flash_addres +  (data_index * 4);
				read_data_flash_buffer[data_index] = FMC_Read(tempAddr);
			}
			memcpy(calling_mobile_number,read_data_flash_buffer,sizeof(read_data_flash_buffer));

			if(phone_numbe_is_avaliable==data_flash_data)
			{
				memcpy(calling_mobile_number,mobile_number_arry,10);
			}

			if(vechical_number_avalible==data_flash_data)
			{

				memcpy(calling_mobile_number+11,mobile_number_arry,13);
			}

            if(icmi_number_avalible==data_flash_data)
            {
            	memcpy(calling_mobile_number+25,mobile_number_arry,15);
            }

			FMC_Erase(vechicle_data_flash_addres);

			for(data_index = 0; data_index < lenght/4 + 1; ++data_index)
			{
				memcpy(&u32RData,&calling_mobile_number[data_index * 4],4 );

				tempAddr = vechicle_data_flash_addres + (data_index * 4);

				FMC_Write(tempAddr, u32RData);
			}

	     	memset(calling_mobile_number,0,sizeof(calling_mobile_number));

//			for(data_index = 0; data_index < lenght/4; data_index++)
//			{
//				tempAddr = vechicle_data_flash_addres +  (data_index * 4);
//				read_data_flash_buffer[data_index] = FMC_Read(tempAddr);
//			}

		   /* Disable FMC ISP function */

			FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

					/* Lock protected registers */

					SYS_LockReg();


//			data_flash_read(vechicle_data_flash_addres,38);


}
int32_t *data_flash_read(uint16_t read_data_flash_address,uint8_t lenght)
{
	SYS_UnlockReg();

	/* Enable FMC ISP functions */
	FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_LDUEN_Msk | FMC_ISPCTL_CFGUEN_Msk;

	for(data_index = 0; data_index < lenght/4+1; data_index++)
	{
		tempAddr = read_data_flash_address +  (data_index * 4);
		data_form_falsh[data_index] = FMC_Read(tempAddr);
	}

   /* Disable FMC ISP function */
	FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

	/* Lock protected registers */
	SYS_LockReg();

//	for(int i=0;i<data_index;i++)
//	{
	return data_form_falsh;
//	}
}
void sim_calling_function()
{
	UART_Write(UART1, "ATD", sizeof("ATD"));
//	UART_Write(UART1, DBQUOTE, 1);
//	UART_Write(UART1, recived_mobile_number, sizeof(recived_mobile_number));
	UART_Write(UART1, "9032888261",strlen("9032888261"));
//	UART_Write(UART1, DBQUOTE, 1);
	UART_Write(UART1, ";", 1);
	UART_Write(UART1, "\r\n", 2);

}
void date_time_function()
{
	if(strstr(set_date_time_buffer,"+CCLK: "))
	{
		date_and_time_pointer=strstr(set_date_time_buffer,"+CCLK: ");

		memcpy(set_date_time_buffer,date_and_time_pointer+8,14);

		memcpy(year_buffer,set_date_time_buffer,8);

		memcpy(time_buffer,set_date_time_buffer+9,5);

//		strcat(fuel_sensor_buff,year_buffer);
//
//		strcat(fuel_sensor_buff,"/");
//
//		strcat(fuel_sensor_buff,"Time:");
//
//		strcat(fuel_sensor_buff,time_buffer);

//		sensor_data_sending_to_sms();

	//	memset(set_date_time_buffer,0,sizeof(set_date_time_buffer));

	}
}

void register_the_mobile_number()
{
//	mobile_number_pointer=(strstr(phone_number_buffer,"+CMT:"));

	memcpy(phone_number_buffer_1,phone_number_buffer+10,10);

	if((memcmp(phone_number_buffer_1,num1,10)==0))
	  {
		memset(phone_number_buffer_1,0,sizeof(phone_number_buffer_1));

		mobile_number_pointer=(strstr(phone_number_buffer,num1));

		memcpy(num2,mobile_number_pointer,(10));

		mobile_number_pointer=(strstr(phone_number_buffer,"SET PHONE +91"));

		memcpy(recived_password,mobile_number_pointer+23,(11));

	   if ((memcmp(num1, num2, 10) == 0)&&(memcmp(password, recived_password, 11) == 0))
	   {

	//	mobile_number_pointer=(strstr(&phone_number_buffer,"SET PHONE +91"));

		memcpy(mobile_number_arry_1,mobile_number_pointer+13,(10));

		while((mobile_number_arry_1) && mobile_number_counter<sizeof(mobile_number_arry_1))
		{
		 if (isdigit((unsigned char)mobile_number_arry_1[mobile_number_counter]))
		 {

			 mobile_number_arry[mobile_number_counter++]=mobile_number_arry_1[mobile_number_counter];
		 }
			else
			{
				strcpy(fuel_sensor_buff,"enter_the_number_is_worng");
				sensor_data_sending_to_sms();
				break; // stop at first non-digit
			}

		 mobile_number_pointer++;
		}
		mobile_number_arry[mobile_number_counter] = '\0';

		data_write_flash_write(mobile_number_arry,phone_numbe_is_avaliable,data_flash_size);

		regisrter_mobile_number=1;

		simcard_checking_command();

		strcpy(fuel_sensor_buff,"adding_mobile_numebr_successfully");

	    sensor_data_sending_to_sms();
	    memset(phone_number_buffer,0,strlen(phone_number_buffer));
	    memset(num2,0,sizeof(num2));
	    memset(recived_password,0,sizeof(recived_password));
	    read_data_form_data_flash=1;

	   }

	else
		{
//		vehicale_registrion();

//		mobile_number_pointer=(strstr(&phone_number_buffer,num1));

//		memcpy(num2,mobile_number_pointer,(10));

		mobile_number_pointer=(strstr(phone_number_buffer,"SET VEHICLE"));

		memcpy(recived_password,mobile_number_pointer+25,(11));

	    if ((memcmp(num1, num2, 10) == 0)&&(memcmp(password, recived_password, 11) == 0))
		   {
	    	  mobile_number_arry[13]='\0';

				data_write_flash_write((memcpy(mobile_number_arry,mobile_number_pointer+12,13)),vechical_number_avalible,data_flash_size);

				strcpy(fuel_sensor_buff,"adding_vehicle_numebr_successfully");
				sensor_data_sending_to_sms();
				memset(phone_number_buffer,0,strlen(phone_number_buffer));
                memset(num2,0,sizeof(num2));
                memset(recived_password,0,sizeof(recived_password));
                read_data_form_data_flash=1;
		   }
		}
	  }
	    else
	    {
	    memset(phone_number_buffer_1,0,sizeof(phone_number_buffer_1));

	    mobile_number_pointer=(strstr(phone_number_buffer,"+91"));

	    memcpy(num2,mobile_number_pointer+3,(10));
	   // mobile_number_counter=0;

//	    while(mobile_number_counter<=10)
//	    {
	     if(isdigit((unsigned char)(num2[0])))
	     {
	    	 memcpy(num2,mobile_number_pointer+3,(10));
	    //	 mobile_number_pointer++;
//	     }

	    if((memcmp(num1, num2, 10)!=0)/*&&(mobile_number_counter==10)*/)
	    {

	    	strcpy(fuel_sensor_buff,"Security Alert: An attempt was made to change the registered details from number ");
	    	strcat(fuel_sensor_buff,num2);
	    	memset(phone_number_buffer,0,strlen(phone_number_buffer));
	    	sensor_data_sending_to_sms();
	    	memset(num2,0,sizeof(num2));
	    }
	    }
		}
	  //}
}
void  default_at_commands()
{
//	  delayCounter = 100000;
//		      delay_loop();
//
//	      UART_Write(UART1,0x1A,sizeof(0x1A));

//	UART_Write(UART1, "AT+IPREX=115200\r\n", sizeof("AT+IPREX=115200\r\n"));

//	      delayCounter = 100000;
//	      delay_loop();
//
//	      UART_Write(UART1,"ATE1",strlen("ATE1"));
//
//	      delayCounter = 100000;
//	     	      delay_loop();


	      UART_Write(UART1, "AT+CMGD=1,4\r\n",strlen("AT+CMGD=1,4\r\n"));

		  delayCounter = 100000;
		  delay_loop();

		  UART_Write(UART1, "AT+CNMI=2,2,0,0,0\r\n",strlen("AT+CNMI=2,2,0,0,0\r\n"));

	      delayCounter = 100000;
	      delay_loop();

	      UART_Write(UART1, "AT+CREG?\r\n", strlen("AT+CREG?\r\n"));

	      delayCounter = 100000;
	      delay_loop();

	      UART_Write(UART1, "AT+CMGF=1\r\n",strlen("AT+CMGF=1\r\n"));

	      delayCounter = 100000;
	      delay_loop();

		 UART_Write(UART1, "AT+CSCS=",strlen("AT+CSCS="));
	     UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
	     UART_Write(UART1, "GSM",strlen("GSM"));
	     UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
	     UART_Write(UART1, "\r\n",strlen("\r\n"));

	     delayCounter = 100000;
		 delay_loop();

		 UART_Write(UART1, "AT+CMGL=",strlen("AT+CMGL="));
		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
		 UART_Write(UART1, "ALL",strlen("ALL"));
		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
		 UART_Write(UART1, "\r\n",strlen("\r\n"));

//		 delayCounter = 100000;
//		 delay_loop();
//
//		 UART_Write(UART1, " AT+CPMS=",strlen(" AT+CPMS="));
//		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
//		 UART_Write(UART1, "SM",strlen("SM"));
//		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
//
//		 UART_Write(UART1, ",",sizeof(","));
//
//		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
//		 UART_Write(UART1, "SM",strlen("SM"));
//		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
//
//		 UART_Write(UART1, ",",sizeof(","));
//
//		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
//		 UART_Write(UART1, "SM",strlen("SM"));
//		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
//
//		 UART_Write(UART1, "\r\n",strlen("\r\n"));

		 delayCounter = 100000;
		 delay_loop();

	     UART_Write(UART1, "AT+CTZU=1\r\n", strlen("AT+CTZU=1\r\n"));

	     delayCounter = 100000;
	     delay_loop();

//	     UART_Write(UART1, "AT+CLTS=1\r\n", strlen("AT+CLTS=1\r\n"));
//
//	     delayCounter = 100000;
//	   	 delay_loop();

	     UART_Write(UART1, "AT+CGREG=2\r\n", strlen("AT+CGREG=2\r\n"));

	     delayCounter = 1000000;
	     delay_loop();

	     UART_Write(UART1, "AT+CCLK?\r\n", strlen("AT+CCLK?\r\n"));

	     delayCounter = 1000000;
	   	 delay_loop();


}

void simcard_checking_command()
{
// after register the mobile number to exiqute this  command to to  get  the cimi number
	UART_Write(UART1, "AT+CIMI\r\n",strlen("AT+CIMI\r\n"));

}
//void simcard_data_flash_icmi(uint8_t buffer)
//{
//	// wirte the data flsh the icmi sim card unique number
//
//}
void compare_old_new_data()
{
   // to check the sim is chnged or not here if sim card is chnaged the enble the while loop
//	if(memcmp(sim_card_test_buffer, sim_icmi_number, 15)!= 0)
//	{
//		sim_card_checking=1;
//	}

}


//void vehicale_registrion()
//{
//	   vechical_registion_pointer=(strstr(&phone_number_buffer,num1));
//
//		memcpy(num2,vechical_registion_pointer,(10));
//
//		vechical_registion_pointer=(strstr(&phone_number_buffer,"SET VEHICLE"));
//
//		memcpy(recived_password,vechical_registion_pointer+25,(11));
//
//	    if ((memcmp(num1, num2, 10) == 0)&&(memcmp(password, recived_password, 11) == 0))
//		   {
//
//				data_write_flash_write(vechical_registion_pointer+12,vechicle_data_flash_addres,12);
//
//				strcpy(fuel_sensor_buff,"adding_vehicle_numebr_successfully");
//				sensor_data_sending_to_sms();
//
//		   }
//}
