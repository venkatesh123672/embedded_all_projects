/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 7 $
 * $Date: 17/05/04 1:36p $
 * @brief    Configure SPI0 as Master mode and demonstrate how to communicate
 *           with an off-chip SPI Slave device with FIFO mode. This sample
 *
 * @note
 *Corp. All rights reserved.
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


#define PLL_CLOCK       144000000

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

#define BEEP_ON PE3= 1
#define BEEP_OFF PE3=0


#define LED_ON PD6= 1
#define LED_OFF PD6=0

#define phone_numbe_is_avaliable  0x33
#define vechical_number_avalible  0x34
#define icmi_number_avalible      0x35


#define mobile_number /*"9481800631"*/"9032888261"

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

void buzzer_sound();

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

char before_f[20];
char after_f[20];

/*GPS DATA*/
/*uint8_t *gpsData;
uint8_t *endofStr;*/
uint8_t *rtc;

uint8_t gnssutc[11];
uint8_t gnss_lat[10];
uint8_t gnss_lon[10];
uint8_t gnsshdop[4];
uint8_t gnssalt[6];
uint8_t gnssfix[2];
uint8_t gnsscog[7];
uint8_t gnssspkm[4];
uint8_t gnssspkn[4];
uint8_t gnssdate[7];
uint8_t gnssnsat[3];

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
uint8_t GNSS_data[100];
uint8_t wlturlsize_url[5];

uint8_t gnss[BUF_LEN_SMALL];
uint8_t condition=0;

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

uint16_t length=0;

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

volatile uint8_t duty = 100;
volatile uint8_t direction = 0;

uint8_t newline[] = {0x0D, 0x0A};

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


uint8_t URLLENGTH[BUF_LEN_SMALL] = "33,8";
uint8_t type[BUF_LEN_SMALL] = "\"type\":";
uint8_t WLTLAT[BUF_LEN_SMALL] = "\"lat\":";
uint8_t WLTLON[BUF_LEN_SMALL] = "\"lng\":";
uint8_t Message[BUF_LEN_SMALL] = "\"message\":";
uint8_t WLTSPEED[BUF_LEN_SMALL] = "speed=";
uint8_t WLTIGNITION[BUF_LEN_SMALL] = "ignition=";
uint8_t WLTGPSDATE[BUF_LEN_SMALL] = "gpsdate=";
uint8_t WLTEVENTID[BUF_LEN_SMALL] = "eventid=";
uint8_t WLTALTITUDE[BUF_LEN_SMALL] = "altitude=";

uint8_t SENSOR_MESSAGE[50] ;

uint8_t SENSOR_MESSAGE_RESULT[50];

uint8_t fuel_sensor[40];
uint8_t Message_r[50];

uint8_t URL[150];
uint8_t IME[BUF_LEN_SMALL];

uint8_t gpsDebugData = 0;
uint8_t dataUpdateReady=0;

uint8_t sizof_send_buffer[10];
uint16_t sizof_send_buffer_interger;

float intBattV_conv;
float extBattV_conv;
float battint=177;


float int_battery_voltage=0;
float ext_battery_voltage=0;
float scale = 0;
static uint8_t battery_sms_sent = 0;
uint8_t battery_percentage=0;
extern uint32_t intBatt_V;
extern uint32_t extBatt_V;

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
	GNSS_QUERY,
	GSM_SLEEP,

}GSM_SM_T;

uint8_t gsmState = GSM_INIT;


uint8_t configMode = 0;
char DBQUOTE[1] ={0x22};
char steing_end[1]={0x26};
uint32_t UART_Write_main(UART_T *uart, uint8_t *pu8TxBuf,uint32_t u32WriteBytes);
uint8_t fuel_sensor_buff[200];
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

volatile uint8_t buzzer_step = 0;
volatile uint16_t buzzer_timer = 0;

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

uint8_t sms_call_chek=0;

uint8_t message_sending=0;


uint8_t    wire_rec_buf[WIRE_REC_BUF_SIZE];
uint8_t    wire_rec_count=0;
uint8_t    rec_from_wire_done=0;
uint8_t    rec_from_wireless=0;

volatile uint8_t buzzeR_sound=0;
volatile uint8_t BUZZER_OF=1;

uint16_t   keypressed_value=0;
uint8_t    key_pressed[4];

uint32_t control_panel=CONTROL_PANEL;
uint16_t conv_32to_16_cp =0;
uint16_t conv32_16_dest=0;

//signal strength measured
uint16_t  singnal_strength=0;

uint8_t mobile_number_arry_1[10];

volatile static uint8_t loopCount = 0;
uint8_t tloopCount=0;
uint8_t loopDelay=0;
uint8_t timeOut = 0 ;
volatile uint8_t sim_calling_chek=0;
uint8_t call_chek=0;
volatile uint16_t sim_timer=0;

uint8_t mahaan=0;
uint8_t value[10];
uint8_t phone_number_buffer[BUF_LEN];
uint8_t phone_number_buffer_1[REC_BUF_SIZE];

uint8_t data_send_buffer[BUF_LEN];

uint8_t recived_mobile_number[11];
int32_t data_flash_phone_vecicale_iemi_buffer[10];
int32_t *mobile_number_recived;
uint8_t recived_mobile_number_counter=0;
uint8_t read_data_form_data_flash=1;

uint8_t set_date_time_buffer[BUF_LEN];
uint8_t *date_and_time_pointer=0;
uint8_t  year_buffer[11];
uint8_t  time_buffer[6];
uint8_t vechical_registion_buffer[BUF_LEN];
uint8_t *vechical_registion_pointer=0;

uint8_t num1[10] = mobile_number/*"8688043920"*/;
uint8_t num2[10] ;

uint8_t calling_chek=0;
uint8_t police_chek=0;
uint8_t emergency_chek=0;
uint8_t call_test=0;

uint8_t datetime[40];

uint8_t panic_alert_set=0;
char *ptr=0;

//uint8_t djr_url[]="https://api.djr2innovationpvtltd.com/api/iot/events?deviceIMEI=123456789101234";

uint8_t djr_url[]="https://djr2-backend.onrender.com/api/iot/events?deviceIMEI=867400012345678";
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
void int_and_ext_battery_percentges();
void gps_data_send_sms();

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
//        	count_1=0;
           uart_data_data=1;
        }
//       if((strstr(UART1_RX_BUF,"+CCLK:"))&&(!uart_clock_data))
////    	   if((strstr(UART1_RX_BUF,"AT+CCLK?"))&&(!uart_clock_data))
//	   {
//    	   data_count_1=0;
//	       uart_clock_data=1;
//	   }


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
    /// beep of intrupet
  if(GPIO_GET_INT_FLAG(PC, BIT5))
  {
	  GPIO_CLR_INT_FLAG(PC, BIT5);
	  if(PC5)
	  {
//	  PWM_ForceStop(PWM0, 0x28);
	  buzzeR_sound=0;
	  BUZZER_OF=1;
	  }
  }
  // police_intrupt
  if(GPIO_GET_INT_FLAG(PC, BIT1))
  {
	  GPIO_CLR_INT_FLAG(PC, BIT1);
	  if(PC1)
		 police_chek=1;
  }
  // emeargency intrupt
  if(GPIO_GET_INT_FLAG(PC, BIT4))
  {
	  GPIO_CLR_INT_FLAG(PC, BIT4);
	  if(PC4)
		  emergency_chek=1;
  }
  // call cut
  if(GPIO_GET_INT_FLAG(PE, BIT0))
  {
	  GPIO_CLR_INT_FLAG(PE, BIT0);
	  if(PE0)
	  {
		  sim_calling_chek=0;
		  UART_Write(UART1,"ATH\r\n",strlen("ATH\r\n"));
	  }
  }
  /******lorry_ower_calling************/
  if(GPIO_GET_INT_FLAG(PD, BIT7))
  {
	  GPIO_CLR_INT_FLAG(PD, BIT7);
	  if(PD7)
		  calling_chek=1;
  }
  /****panic_alert*******/
  if(GPIO_GET_INT_FLAG(PC, BIT0))
  {
	  GPIO_CLR_INT_FLAG(PC, BIT0);
	  if(PC0)
		  panic_alert_set=1;
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
void TMR2_IRQHandler(void) {
	if (TIMER_GetIntFlag(TIMER2) == 1) {
		/* Clear Timer0 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER2);

		sim_timer++;
//
//		buzzer_timer++;
//
////		if (buzzer_timer >=1 )
////		if(buzzeR_sound)
//		{
//		    buzzer_timer = 0;
//
//		    switch (buzzer_step)
//		    {
//		        case 0:
// 		            PWM_ConfigOutputChannel(PWM0, 3, 1000, 50);
//		            buzzer_step++;
//		            break;
//
//		        case 1:
//		            PWM_ConfigOutputChannel(PWM0, 3, 1100, 50);
//		            buzzer_step++;
//		            break;
//
//		        case 2:
//		            PWM_ConfigOutputChannel(PWM0, 3, 1180, 50);
//		            buzzer_step++;
//		            break;
//
//		        case 3:
//		            PWM_ConfigOutputChannel(PWM0, 3, 1300, 50);
//		            buzzer_step = 0;
//		            break;
//		    }
//		}
//
//		 if(direction == 0)
//		    {
//		        duty -= 12;
//
//		        if(duty <= 12)
//		            direction = 1;
//		    }
//		    else
//		    {
//		        duty += 12;
//
//		        if(duty >= 100)
//		            direction = 0;
//		    }
//
//		 PWM_ConfigOutputChannel(PWM0, 5, 1000, 50);
//			 PWM_ConfigOutputChannel(PWM0, 3, 1000, 50);
//

	//		 PWM_ConfigOutputChannel(PWM0, 5, 1500, 50);
	//		 PWM_ConfigOutputChannel(PWM0, 3, 1500, 50);


	}
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
    CLK_EnableModuleClock(TMR1_MODULE);
   	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);


		//timer2 initial
		CLK_EnableModuleClock(TMR2_MODULE);
		CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);

//    /* Gpio init for rfm69 trans receive */
//	    GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);
//	    PD2=0;

	SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk)) | SYS_GPC_MFPL_PC0MFP_GPIO;
//	SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_UART2_RXD;


	    GPIO_SetMode(PD, BIT7, GPIO_MODE_INPUT);
	    GPIO_EnableInt(PD, 7, GPIO_INT_RISING);

	    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
	    GPIO_EnableInt(PC, 0, GPIO_INT_BOTH_EDGE);

	   	GPIO_SetMode(PC, BIT1, GPIO_MODE_INPUT);
	    GPIO_EnableInt(PC, 1, GPIO_INT_RISING);

	    GPIO_SetMode(PC, BIT4, GPIO_MODE_INPUT);
	    GPIO_EnableInt(PC, 4, GPIO_INT_RISING);


	    GPIO_SetMode(PC, BIT5, GPIO_MODE_INPUT);
	   	GPIO_EnableInt(PC, 5, GPIO_INT_RISING);

	   	GPIO_SetMode(PE, BIT0, GPIO_MODE_INPUT);
	    GPIO_EnableInt(PE, 0, GPIO_INT_RISING);


	/* Gpio init for rfm69 trans receive */
	    GPIO_SetMode(PD, BIT3, GPIO_MODE_INPUT);
	    GPIO_EnableInt(PD, 3, GPIO_INT_BOTH_EDGE);
//	    NVIC_EnableIRQ(GPCDEF_IRQn);


//	    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
//	    GPIO_ENABLE_DEBOUNCE(PD, BIT3);
//	    GPIO_SetMode(PD, BIT7, GPIO_MODE_INPUT);
//	    GPIO_EnableInt(PC, 1, GPIO_INT_BOTH_EDGE);
	    NVIC_EnableIRQ(GPCDEF_IRQn);


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

		SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD6MFP_Msk));
	    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD6MFP_GPIO;

	    GPIO_SetMode(PD, BIT6, GPIO_MODE_OUTPUT);

	    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE3MFP_Msk));
	    SYS->GPE_MFPL |= SYS_GPE_MFPL_PE3MFP_GPIO;

	    GPIO_SetMode(PE, BIT3, GPIO_MODE_OUTPUT);

//	    /*---------------------------------------------------------------------------------------------------------*/
//	     /* PWM clock frequency configuration                                                                       */
//	     /*---------------------------------------------------------------------------------------------------------*/
//	 	/* Enable PWM0 and PWM1 module clock */
//	 	CLK_EnableModuleClock(PWM0_MODULE);
//
//	 	/* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
//	     CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));
//
//	     /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
//	     /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
//	     CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_PCLK0, NULL);
//	 //    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL1_PWM1SEL_PCLK1, NULL);
//
//	     // pwm_pin initialization
//	     SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE3MFP_Msk));
//	  	 SYS->GPE_MFPL |= SYS_GPE_MFPL_PE3MFP_PWM0_CH3;
//
//	     SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD6MFP_Msk));
//	  	 SYS->GPD_MFPL |= SYS_GPD_MFPL_PD6MFP_PWM0_CH5;
//
//	     /* Reset PWM0  module */
//	     SYS_ResetModule(PWM0_RST);
//
////	 // pwm config out put which channel
////	     PWM_ConfigOutputChannel(PWM0, 3, 200, 50);
////	     PWM_ConfigOutputChannel(PWM0, 5, 200, 50);
//	 // enable the pwm out channel
//	     PWM_EnableOutput(PWM0, 0x28);
////	     // start pwm
//	     PWM_Start(PWM0, 0x28);
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
//    UART0_Init();
//    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk ));


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

	TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 5);
	TIMER_EnableInt(TIMER1);
	TIMER_Start(TIMER1);
	NVIC_EnableIRQ(TMR1_IRQn);
//
	TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 8);
	TIMER_Start(TIMER2);
	TIMER_EnableInt(TIMER2);
	NVIC_EnableIRQ(TMR2_IRQn);

//	PWM_Start(PWM0, 0x28);
/****ADC_INIT******/
	 ADC_init();

    UART_Write(UART2, "EDEL SMART DEVICES - 001\r\n", 26);
 //  NVIC_EnableIRQ(UART02_IRQn);

    /* reset RFM69 */
    reset_rfm69();
    /* Init RFM69 */
    InitRFChip();
//    set_required_baud_rate_4800();
    set_required_baud_rate();
//    sim_calling_function();

//    PWM_Start(PWM0, 0x28);

    delayCounter = 2000000;
    		  delay_loop();

    default_at_commands();

//    BEEP_ON;
//
//    BEEP_OFF;
//
    BEEP_ON;

   simcard_checking_command();

   LED_OFF;

   LED_ON;

   LED_OFF;

//   int freq=0;

while(1)
{

//	PWM_Start(PWM0, 0x28);
//	TIMER_Delay(TIMER1,100000);
////	delayCounter = 10000;
////	delay_loop();
//	PWM_ForceStop(PWM0, 0x28);
//	TIMER_Delay(TIMER1,100000);
////	delayCounter = 10000;
////	delay_loop();

//	for (duty = 100; duty >= 0; duty-=20)
//	{
//	    PWM_ConfigOutputChannel(PWM0, 5, 180000, duty);
//	    delayCounter = 100000;
//	    delay_loop();
//	}

//	for (freq = 700; freq <= 1800; freq += 20)
//	{
//	    PWM_ConfigOutputChannel(PWM0, 3, freq, 50);
//	    delayCounter = 100000;
//	    delay_loop();
//	}

//	if(buzzeR_sound)
//	{
//		PWM_Start(PWM0, 0x28);
//		buzzer_sound();
//	}
	if((message_sending)&&(call_test))
	{
		message_sending=0;
		gps_data_send_sms();

	}

	if((loopCount>25&&loopCount<80)||(loopCount>130&&loopCount<255))
	{
		loopCount=0;
	}
	if(BUZZER_OF)
	{
//       for(int i=0;i<=5;i++)
		BEEP_OFF;
       {
		sms_google_buffer[0]=10;
 		send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,&sms_google_buffer[0]);
//		delayCounter = 10000;
//	    delay_loop();
       }
		BUZZER_OF=0;
	}

	if(call_chek)
		{
			gsmState=GSM_SLEEP;

			loopCount = 0;
			call_chek=0;
		}
    if(uart_data)
     {
   	  uart_data=0;
//   	  if(count_12>count_1)
//   	      count_12=count_1-1;
//   	if (count_12 > count_1)
//   	{
//   	    count_12 = count_1;
//   	}
//
//		while(count_12<count_1)
//		{
//
//
//			UART_Write(UART2, &UART1_RX_BUF[count_12++], 1);
//			if(count_12>=BUF_LEN)
//				count_12=0;
   	while (count_12 != count_1)
   	{
   	    UART_Write(UART2, &UART1_RX_BUF[count_12], 1);

   	    count_12 = (count_12 + 1) % BUF_LEN;


			  if(memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CMT: ",strlen("+CMT: "))&&data_count_1>72)
	//	     if(sim_card_test_pointer != NULL)
			{

				  sim_card_test_pointer =memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CMT: ",strlen("+CMT: "));
		        	uart_data_data=0;
		        	uart_data_registrion=1;
		        	count_1=0;
	//				data_count_1=0x90;
					memcpy(phone_number_buffer,sim_card_test_pointer,sizeof(phone_number_buffer));
					phone_number_buffer[sizeof(phone_number_buffer) - 1] = '\0';
					memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
		        }

		}
     }
// Check whether the SIM card has been changed by comparing the CIMI (IMSI).
//		if(memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "AT+CIMI",strlen("AT+CIMI"))&&(/*count_1>16*/(strstr((char *)UART1_RX_BUF, "OK") != NULL)))

		sim_card_test_pointer=memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "AT+CIMI", strlen("AT+CIMI"));

		if((sim_card_test_pointer!=NULL)&&((strstr((char *)UART1_RX_BUF, "OK") != NULL)))
		{
//			sim_card_test_pointer=memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "AT+CIMI", strlen("AT+CIMI"));

//			sim_card_test_pointer=strstr(UART1_RX_BUF,"AT+CIMI");

//			memcpy(sim_card_test_buffer,sim_card_test_pointer+10,(sizeof(sim_card_test_buffer)));

			memset(sim_card_test_buffer, 0, sizeof(sim_card_test_buffer));

			strncpy((char *)sim_card_test_buffer,(char *)(sim_card_test_pointer + 10),sizeof(sim_card_test_buffer) );

			sim_card_error_checkingh=0;

			sim_card_test_pointer=NULL;

//			if(memmem(sim_card_test_buffer, sizeof(sim_card_test_buffer), "ERROR", 5) != NULL)
			 if (strstr((char *)sim_card_test_buffer, "ERROR") != NULL)
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
// Check whether a SIM card is removed or not .
		if(memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "SIM REMOVED",strlen("SIM REMOVED"))!=NULL)
		{
			  sim_card_checking=1;

		}
//to get the clock
		if ((strstr(UART1_RX_BUF, "+QLTS:") != NULL) &&/*(data_count_1 > 23)*/ (strstr(UART1_RX_BUF, "\r\n") != NULL))
			{
//				uart_clock_data=0;
//				uart_clcok_read=1;
//				count_1=0;
				data_count_1=0x90;
				memset(set_date_time_buffer, 0, sizeof(set_date_time_buffer));

				sim_card_test_pointer=strstr(UART1_RX_BUF, "+QLTS:");

				memcpy(set_date_time_buffer,sim_card_test_pointer,sizeof(set_date_time_buffer));

				 date_time_function();

				memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
			}
//// Mobile number registration.
//		sim_card_test_pointer =memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CMT: ",strlen("+CMT: "));
//		  if(memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CMT: ",strlen("+CMT: "))&&data_count_1>200)
////	     if(sim_card_test_pointer != NULL)
//		{
//
//			  sim_card_test_pointer =memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CMT: ",strlen("+CMT: "));
//	        	uart_data_data=0;
//	        	uart_data_registrion=1;
//	        	count_1=0;
////				data_count_1=0x90;
//				memcpy(phone_number_buffer,sim_card_test_pointer,sizeof(phone_number_buffer));
//				phone_number_buffer[sizeof(phone_number_buffer) - 1] = '\0';
//				memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
//	        }
//
//		uart_data=0;
//	}

    sim_card_test_pointer =memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "RING",strlen("RING"));

   if(sim_card_test_pointer!=NULL)
	  {
	   call_chek=1;
		loopCount=0;
		sim_calling_chek=1;
		memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
	  }

    // if call is on going do nothing if  mc is if reaet the code after to chek this

    sim_card_test_pointer =memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CPAS: 4",strlen("+CPAS: 4"));

   if(sim_card_test_pointer!=NULL)
	  {

		loopCount=0;
		sim_calling_chek=1;
		memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
	  }

    //here to chek the call is end or not if call is not end untill it will wait to completed the call
    sim_card_test_pointer =memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "+CPAS: 0",strlen("+CPAS: 0"));

    if(sim_card_test_pointer!=NULL)
       {
       	loopCount=7;
       	sim_calling_chek=0;
       	memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
       }

// to check the sim if sim card no single it will go the normal
    sim_card_test_pointer =memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "NO CARRIER",strlen("NO CARRIER"));

    if(sim_card_test_pointer!=NULL)
    {
    	loopCount=7;
    	sim_calling_chek=0;
    	memset(UART1_RX_BUF,0,sizeof(UART1_RX_BUF));
    }
//     }

    sim_card_test_pointer=memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "CONNECT",strlen("CONNECT"));

    if(sim_card_test_pointer!=NULL)
    {
    	call_test=0;
    	memset(UART1_RX_BUF, 0, sizeof(UART1_RX_BUF));
     }
    sim_card_test_pointer=memmem(UART1_RX_BUF, sizeof(UART1_RX_BUF), "OK",strlen("OK"));
    if(sim_card_test_pointer!=NULL)
       {
    	call_test=1;
    	memset(UART1_RX_BUF, 0, sizeof(UART1_RX_BUF));
//       	loopCount=0;

        }
    if((emergency_chek)&&(call_test))
    {

//    	emergency_chek=0;

    	gsmState=GSM_SLEEP;

    	loopCount=0;

    	call_test=0;

    	call_chek=1;

    	sim_calling_chek=1;

 //    	strncpy(recived_mobile_number,"108",3);

     	strncpy(recived_mobile_number,mobile_number,10);

        sim_calling_function();

    }
    if((police_chek)&&(call_test))
    {

//    	police_chek=0;

    	gsmState=GSM_SLEEP;

    	loopCount=0;

    	call_test=0;

    	call_chek=1;

    	sim_calling_chek=1;

//    	strncpy(recived_mobile_number,"100",3);

    	strncpy(recived_mobile_number,mobile_number,10);

        sim_calling_function();


    }
    if((calling_chek)&&(call_test))
	{

//    	calling_chek=0;

    	gsmState=GSM_SLEEP;

    	loopCount=0;

    	call_test=0;

    	call_chek=1;

    	sim_calling_chek=1;

    	strncpy(recived_mobile_number,mobile_number,10);

    	 sim_calling_function();

	}
    if(panic_alert_set)
    {
    	panic_alert_set=0;

    	panic_alert();
    }
    if((sim_calling_chek)&&(sim_timer>=50))
    {
    	sim_timer=0;

    	UART_Write(UART1, "AT+CPAS\r\n", sizeof("AT+CPAS\r\n"));
    }


    if(uart_data_registrion)
    {
    	uart_data_registrion=0;


    	register_the_mobile_number();

    }
//    if(uart_clcok_read)
//    {
//    	uart_clcok_read=0;
//
//    	 date_time_function();
//
//    	 memset(set_date_time_buffer,0,sizeof(set_date_time_buffer));
//    }


    int_and_ext_battery_percentges();


 if( read_data_form_data_flash)
 {
	 read_data_form_data_flash=0;

	 memcpy(data_flash_phone_vecicale_iemi_buffer,data_flash_read(vechicle_data_flash_addres,data_flash_size),/*data_flash_size*/40);

	 memcpy(long_buffer,data_flash_phone_vecicale_iemi_buffer,sizeof(data_flash_phone_vecicale_iemi_buffer));

	 memcpy(recived_mobile_number,long_buffer,10);

	 memcpy(recived_lorry_number,long_buffer+11,13);

	 memcpy(sim_icmi_number,long_buffer+25,15);

 }

 if(recFlag)
 {
	recFlag=0;
	LED_ON;
	UART_Write(UART2, "loawr_receved\r\n",strlen("loawr_receved\r\n"));
 	receive();
 	LED_OFF;
 }
 else if(rec_from_wire_done)
	  receive();
 else
 {
   receiveInit();
 }
// if(newGpsData > 10)
//	{
//		UART_Write_main(UART2,"\r\n",2);
//		UART_Write_main(UART2,"NEW GPS DATA\r\n",14);
//		UART_Write_main(UART2,GPS_data, strlen(GPS_data));
//		UART_Write_main(UART2,"\r\n\r\n",4);
//		get_gps_data();
//		memset(GPS_data,0,100);
//		newGpsData = 0;
//	}
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

  			case GNSS_QUERY:
  				gnss_query();
  				break;

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
		UART_Write(UART1, "ATS0=2\r\n", sizeof("ATS0=2\r\n"));

		break;
	case 3:
		count_1=0;
		UART_Write(UART1, "AT+QLTS=2\r\n", sizeof("AT+QLTS=2\r\n"));
		break;
	case 4:
		UART_Write(UART1, "AT+CLVL=5\r\n", sizeof("AT+CLVL=5\r\n"));
		break;
	case 5:
		UART_Write(UART1, "AT+QAUDMOD=2\r\n",sizeof("AT+QAUDMOD=2\r\n"));
		break;
	case 6:
		UART_Write(UART1,"AT+CTZR=1\r\n",sizeof("AT+CTZR=1\r\n"));

		break;
	case 7:
		UART_Write(UART1, "AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n",
		           sizeof("AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n") - 1);
		break;
	case 8:
		UART_Write(UART1, "AT&W\r\n", sizeof("AT&W\r\n"));

		break;

	case 9:
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
//		UART_Write(UART1,"\r\n",2);
		UART_Write(UART1, "AT+CLIP=1\r\n", sizeof("AT+CLIP=1\r\n"));
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
//		UART_Write(UART1, "AT+IPREX=115200\r\n", sizeof("AT+IPREX=115200\r\n"));
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
//		UART_Write(UART1, "AT+CPSI?\r\n", 10);
		UART_Write(UART1, "AT+QLTS=2\r\n", sizeof("AT+QLTS=2\r\n"));
		break;
	case 4:
//		UART_Write(UART1, "AT+CNSMOD?\r\n", 12);
		break;
	case 5:
		UART_Write(UART1, "AT+CGACT?\r\n", 11);
		break;
	case 6:
//		UART_Write(UART1, "AT+CMEE=1\r\n", 11);
//		UART_Write(UART1, "AT+CGDCONT=1",sizeof("AT+CGDCONT=1"));
//		UART_Write(UART1, "AT+CGDCONT=",sizeof("AT+CGDCONT="));
//		UART_Write(UART1, "cid",sizeof("cid"));
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "ip",sizeof("ip"));
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, DBQUOTE, 1);
//	    UART_Write(UART1, "APN",sizeof("APN"));
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "\r\n",2);

		break;
	case 7:
//		UART_Write(UART1, "AT+CGATT=1\r\n", 12);
		break;
	case 8:
		UART_Write(UART1, "AT+CGACT=1", sizeof("AT+CGACT=1"));
//		UART_Write(UART1, "cid", sizeof("cid"));
		UART_Write(UART1, "\r\n",2);
		//loop_delay(2);
		break;
	case 9:
		UART_Write(UART1, "AT+CGPADDR=1\r\n", sizeof("AT+CGPADDR=1\r\n"));
		break;
	case 10:
		UART_Write(UART1, "AT+CGACT?\r\n", sizeof("AT+CGACT?\r\n"));
		break;
	case 11:

		UART_Write(UART1, "AT+QIACT=?\r\n",strlen("AT+QIACT=?\r\n"));
//		UART_Write(UART1, "AT+NETOPEN\r\n", 11);// close
		//UART_Write(UART1, "AT+CGACT=1,1\r\n",14 );
		loop_delay(1);
		break;
	case 12:
		UART_Write(UART1, "AT+QIACT=1\r\n", strlen("AT+QIACT=1\r\n"));
		loop_delay(1);
		break;

	case 13:
		UART_Write(UART1, "AT+QGPS=1",strlen("AT+QGPS=1"));
		UART_Write(UART1, "\r\n", strlen("\r\n"));
		break;
	case 14:

		UART_Write(UART1, "AT+QHTTPCFG=", strlen("AT+QHTTPCFG="));
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, "sslctxid",strlen("sslctxid"));
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, ",", 1);
			UART_Write(UART1, "1", 1);
			UART_Write(UART1, "\r\n",2);
			loop_delay(1);
		break;
	case 15:
			UART_Write(UART1, "AT+QHTTPCFG=", strlen("AT+QHTTPCFG="));
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, "contenttype", strlen("contenttype"));
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, ",", 1);
			UART_Write(UART1, "4", 1);
			UART_Write(UART1, "\r\n",2);
			loop_delay(1);
		break;
	case 16:
            UART_Write(UART1, "AT+QSSLCFG=", strlen("AT+QSSLCFG="));
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, "sslversion", strlen("sslversion"));
			UART_Write(UART1, DBQUOTE, 1);
			UART_Write(UART1, ",", 1);
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, ",", 1);
			UART_Write(UART1, "4", 1);
			UART_Write(UART1, "\r\n",2);
			loop_delay(1);
		break;
	case 17:
			UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, "ignorelocaltime",strlen("ignorelocaltime"));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, "\r\n",strlen("\r\n"));
			loop_delay(1);
		break;
	case 18:
		    UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, "ciphersuite",strlen("ciphersuite"));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "0xFFFF",strlen("0xFFFF"));
			UART_Write(UART1, "\r\n",strlen("\r\n"));
			loop_delay(1);
		break;
	case 19:
			UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, "seclevel",strlen("seclevel"));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "0",strlen("0"));
			UART_Write(UART1, "\r\n",strlen("\r\n"));
			loop_delay(1);
		break;
	case 20:
			UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, "sni",strlen("sni"));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, "\r\n",strlen("\r\n"));
			loop_delay(1);
	break;
	case 21:
	        UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, "ignorelocaltime",strlen("ignorelocaltime"));
			UART_Write(UART1, DBQUOTE,1);
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, ",",strlen(","));
			UART_Write(UART1, "1",strlen("1"));
			UART_Write(UART1, "\r\n",strlen("\r\n"));
			loop_delay(1);

		break;

	case 22:
//		gsmState = GSM_QUERY;
		gsmState=GSM_CONNECT;
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

		UART_Write(UART0, "AT+CGACT=1,1\r\n", strlen("AT+CGACT=1,1\r\n"));
////		UART_Write(UART1, "AT+CGACT=1,1\r\n",14 );
////		UART_Write(UART1, "AT+CSSLCFG=?",sizeof("AT+CSSLCFG=?") );
////		UART_Write(UART1, "\r\n",2);
////		loop_delay(5);
//		UART_Write(UART1, "AT+CSSLCFG=",sizeof("AT+CSSLCFG=") );
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "check_cert",sizeof("check_cert") );
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "0",sizeof("0") );
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "1",sizeof("4") );
//		UART_Write(UART1, "\r\n",2);
		break;
	case 2:
//		UART_Write(UART1, "AT+NETOPEN?\r\n",14 );

//		UART_Write(UART1, "AT+CSSLCFG=",sizeof("AT+CSSLCFG=") );
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "sslversion",sizeof("sslversion") );
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "0",sizeof("0") );
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "0",sizeof("4") );
//		UART_Write(UART1, "\r\n",2);

		break;
	case 3:

		UART_Write(UART1, "AT+QHTTPCFG=", strlen("AT+QHTTPCFG="));
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, "contextid", strlen("contextid"));
		UART_Write(UART1, DBQUOTE, 1);
		UART_Write(UART1, ",", strlen(","));
		UART_Write(UART1, "1", strlen("1"));
		UART_Write(UART1, "\r\n", strlen("\r\n"));
//		UART_Write(UART1, "AT+CSSLCFG=",sizeof("AT+CSSLCFG=") );
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "authmode",sizeof("authmode") );
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "0",sizeof("0") );
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "0",sizeof("0") );
//		UART_Write(UART1, "\r\n",2);

//		UART_Write(UART1, "AT+IPADDR\r\n", 11);
		break;
//	case 4:
//
////		UART_Write(UART1, "AT+QHTTPCFG=", strlen("AT+QHTTPCFG="));
////			UART_Write(UART1, DBQUOTE, 1);
////			UART_Write(UART1, "sslctxid",strlen("sslctxid"));
////			UART_Write(UART1, DBQUOTE, 1);
////			UART_Write(UART1, ",", 1);
////			UART_Write(UART1, "1", 1);
////			UART_Write(UART1, "\r\n",2);
//
////		UART_Write(UART1, "AT+QIACT=?\r\n", strlen("AT+QIACT=?\r\n"));
//
////		UART_Write(UART1, "AT+CHTTPSSTART=",sizeof("AT+CHTTPSSTART=") );
////		UART_Write(UART1, DBQUOTE, 1);
////		UART_Write(UART1, "api.djr2innovationpvtltd.com",sizeof("api.djr2innovationpvtltd.com") );
////		UART_Write(UART1, DBQUOTE, 1);
////		UART_Write(UART1, ",", 1);
////		UART_Write(UART1, "443", sizeof("443"));
////		UART_Write(UART1, "\r\n",2);
//
//
////		UART_Write(UART1,"AT+NETOPEN\r\n",12);
//		break;
//	case 5:
//
////		        UART_Write(UART1, "AT+QHTTPCFG=", strlen("AT+QHTTPCFG="));
////				UART_Write(UART1, DBQUOTE, 1);
////				UART_Write(UART1, "contenttype", strlen("contenttype"));
////				UART_Write(UART1, DBQUOTE, 1);
////				UART_Write(UART1, ",", 1);
////				UART_Write(UART1, "4", 1);
////				UART_Write(UART1, "\r\n",2);
//
////		UART_Write(UART1, "AT+CSSLCFG=",sizeof("AT+CSSLCFG=") );
////		UART_Write(UART1, DBQUOTE, 1);
////		UART_Write(UART1, "enableSNI",sizeof("enableSNI") );
////		UART_Write(UART1, DBQUOTE, 1);
////		UART_Write(UART1, ",", 1);
////		UART_Write(UART1, "0",sizeof("0") );
////		UART_Write(UART1, ",", 1);
////		UART_Write(UART1, "1",sizeof("1") );
////		UART_Write(UART1, "\r\n",2);
//
//
////		UART_Write(UART1, "AT+IPADDR\r\n", 11);
//		break;
//	case 6:
////		        UART_Write(UART1, "AT+QSSLCFG=", strlen("AT+QSSLCFG="));
////				UART_Write(UART1, DBQUOTE, 1);
////				UART_Write(UART1, "sslversion", strlen("sslversion"));
////				UART_Write(UART1, DBQUOTE, 1);
////				UART_Write(UART1, ",", 1);
////				UART_Write(UART1, "1",strlen("1"));
////				UART_Write(UART1, ",", 1);
////				UART_Write(UART1, "4", 1);
////				UART_Write(UART1, "\r\n",2);
////				loop_delay(1);
//		break;
//
//	case 7:
////	            UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, "ignorelocaltime",strlen("ignorelocaltime"));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "1",strlen("1"));
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "1",strlen("1"));
////				UART_Write(UART1, "\r\n",strlen("\r\n"));
////				loop_delay(1);
//		break;
//	case 8:
////				UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, "ciphersuite",strlen("ciphersuite"));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "1",strlen("1"));
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "0xFFFF",strlen("0xFFFF"));
////				UART_Write(UART1, "\r\n",strlen("\r\n"));
////				loop_delay(1);
//
//		break;
//	case 9:
////			   UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, "seclevel",strlen("seclevel"));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "1",strlen("1"));
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "0",strlen("0"));
////
//////			UART_Write(UART1, "0xFFFF",strlen("0xFFFF"));
////				UART_Write(UART1, "\r\n",strlen("\r\n"));
//				loop_delay(1);
//
//
//		break;
//	case 10:
//
////				UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, "sni",strlen("sni"));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "1",strlen("1"));
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "1",strlen("1"));
////
////				UART_Write(UART1, "\r\n",strlen("\r\n"));
////				loop_delay(1);
//
//		break;
//	case 11:
//
////                UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, "ignorelocaltime",strlen("ignorelocaltime"));
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "1",strlen("1"));
////				UART_Write(UART1, ",",strlen(","));
////				UART_Write(UART1, "1",strlen("1"));
////				UART_Write(UART1, "\r\n",strlen("\r\n"));
////				loop_delay(1);
//
//		break;
	case 4:
		UART_Write(UART1, "AT+QIACT?\r\n", strlen("AT+QIACT?\r\n"));
//		UART_Write(UART1, "AT+IPADDR\r\n", 11);
		break;
	case 5:
		break;
	case 6:
//		gsmState = GSM_HTTP_REPORT_DATA;
		gsmState=GNSS_QUERY;
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
//	    gsmState = GSM_SLEEP;
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
//		UART_Write(UART1, "AT+HTTPINIT\r\n", 13);

		break;
	case 2:
//		       UART_Write(UART1, "AT+QSSLCFG=",strlen("AT+QSSLCFG="));
////					UART_Write(UART1, DBQUOTE, 1);
////					UART_Write(UART1, "URL",3);
////					UART_Write(UART1, DBQUOTE, 1);
////					UART_Write(UART1, ",",1);
//					UART_Write(UART1, DBQUOTE,1);
//					UART_Write(UART1, "ignorelocaltime",strlen("ignorelocaltime"));
//					UART_Write(UART1, DBQUOTE,1);
//					UART_Write(UART1, ",",strlen(","));
//					UART_Write(UART1, "1",strlen("1"));
//					UART_Write(UART1, ",",strlen(","));
//					UART_Write(UART1, "1",strlen("1"));
//					UART_Write(UART1, "\r\n",strlen("\r\n"));
		loop_delay(3);

//		UART_Write(UART1, "AT+HTTPSSL=1",strlen("AT+HTTPSSL=1"));
////		UART_Write(UART1, DBQUOTE, 1);
////		UART_Write(UART1, "1",1);
////		UART_Write(UART1, DBQUOTE, 1);
/////		UART_Write(UART1, ",",1);
//		UART_Write(UART1, "\r\n",2);


		break;
	case 3:
//		call_test=0;
//		loop_delay(5);
//			UART_Write(UART1, "AT+HTTPPARA=",12);
		UART_Write(UART1, "AT+QHTTPURL=",strlen("AT+QHTTPURL="));
//			UART_Write(UART1, DBQUOTE, 1);
//			UART_Write(UART1, "URL",3);
//			UART_Write(UART1, DBQUOTE, 1);
//			UART_Write(UART1, ",",1);
//			UART_Write(UART1, DBQUOTE,1);
		   UART_Write(UART1, (itoa(strlen(djr_url),wlturlsize,10)),strlen(wlturlsize));
			//if(dataUpdateReady)
//			{
			//	dataUpdateReady = 0;
////				UART_Write(UART1,webURL,sizeof(webURL));
////				UART_Write(UART1,"https://jsonplaceholder.typicode.com/posts",strlen("https://jsonplaceholder.typicode.com/posts"));
//				UART_Write(UART1,"https://api.djr2innovationpvtltd.com/api/iot/events?deviceIMEI=123456789101234",strlen("https://api.djr2innovationpvtltd.com/api/iot/events?deviceIMEI=123456789101234"));
////				UART_Write(UART1,url_tset_buffer,strlen(url_tset_buffer));
////				UART_Write(UART1,"http://tracking-backend-5nzh.onrender.com/api/v1/sensors/EDL-DEV-7236E9/data",strlen("https://tracking-backend-5nzh.onrender.com/api/v1/sensors/EDL-DEV-7236E9/data"));
////				UART_Write(UART1,"sensor-data",strlen("sensor-data"));
//				UART_Write(UART1, DBQUOTE,1);
		      UART_Write(UART1, ",",strlen(","));
		      UART_Write(UART1, "80",strlen("80"));
			  UART_Write(UART1, "\r\n",2);

//			}
		break;
	case 4:
//		        UART_Write(UART1, "AT+HTTPPARA=",12);
//				UART_Write(UART1, DBQUOTE, 1);
//				UART_Write(UART1, "SSLCFG",strlen("SSLCFG"));
//				UART_Write(UART1, DBQUOTE,1);
//				UART_Write(UART1, ",",1);
////				UART_Write(UART1, DBQUOTE,1);
////				UART_Write(UART1, "application/json",strlen("application/json"));
//			//  UART_Write(UART1, " ;charset=UTF-8",strlen("; charset=UTF-8"));
////				UART_Write(UART1, DBQUOTE,1);
//				UART_Write(UART1, "0",sizeof("0"));
//				UART_Write(UART1, "\r\n",2);

		loop_delay(3);
		break;
	case 5:

		UART_Write(UART1,djr_url,strlen(djr_url));
		UART_Write(UART1, "\r\n",2);

		break;
	case 6:
//				UART_Write(UART1, "AT+HTTPPARA=",12);
//				UART_Write(UART1, DBQUOTE, 1);
//				UART_Write(UART1, "CONTENT",strlen("CONTENT"));
//				UART_Write(UART1, DBQUOTE,1);
//				UART_Write(UART1, ",",1);
//				UART_Write(UART1, DBQUOTE,1);
//				UART_Write(UART1, "application/json",strlen("application/json"));
//		//		UART_Write(UART1, " ;charset=UTF-8",strlen("; charset=UTF-8"));
//				UART_Write(UART1, DBQUOTE,1);
//				UART_Write(UART1, "\r\n",2);

		        loop_delay(3);
		break;
	case 7:

		  UART_Write(UART1, "AT+QHTTPPOST=", strlen("AT+QHTTPPOST="));
		  UART_Write(UART1, wlturlsize_url, strlen(wlturlsize_url));
		  UART_Write(UART1, ",", strlen(","));
		  UART_Write(UART1, "80", strlen("80"));
		  UART_Write(UART1, ",", strlen(","));
		  UART_Write(UART1, "80", strlen("80"));
		  UART_Write(UART1, "\r\n", strlen("\r\n"));
		break;
	case 8:
//	//	if(dataUpdateReady)
//		{
//
//		UART_Write(UART1, "AT+HTTPDATA=",strlen("AT+HTTPDATA="));
//		UART_Write(UART1, sizof_send_buffer,strlen(sizof_send_buffer));
////		UART_Write(UART1, "38",strlen("59"));
//		UART_Write(UART1, ",",1);
//		UART_Write(UART1, "1000",strlen("1000"));
//		UART_Write(UART1, "\r\n",2);
//
//		}

    	break;
	case 9:

		     loop_delay(3);
		break;

 	case 10:

	case 11:
	//	if(dataUpdateReady)
		{

		UART_Write(UART1,URL,strlen(URL));
//		UART_Write(UART1,data_send_buffer,strlen(data_send_buffer));
		UART_Write(UART1, "\r\n",strlen("\r\n"));
//		dataUpdateReady=0;
		}

		loop_delay(1);
		break;
	case 12:
//		loop_delay(5);

		break;

	case 13:
//		UART_Write(UART1, "AT+HTTPACTION=1\r\n", strlen("AT+HTTPACTION=1\r\n"));
		UART_Write(UART1, "AT+QHTTPREAD=80\r\n", strlen("AT+QHTTPREAD=80\r\n"));
		loop_delay(1);
	    break;
	case 14:
//		UART_Write(UART1, "AT+CEER\r\n", sizeof("AT+CEER\r\n"));

//		 UART_Write(UART1, "AT+HTTPTERM\r\n", sizeof("AT+HTTPTERM\r\n"));
		break;
	case 15:
//		gsmState=GNSS_QUERY;
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
void gnss_query()
{
	char *tString;
	switch (loopCount)
	{
	case 1:
//		UART_Write(UART1, "AT+QGPSCFG=", 11);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "autogps", 7);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "1", 1);
//		UART_Write(UART1, "\r\n", 2);

		break;
	case 2:

		UART_Write(UART1, "AT+QGPSLOC=2\r\n", strlen("AT+QGPSLOC=2\r\n"));
		break;
	case 3:
		count_0=0;

		break;
	case 4:

			break;
	case 5:

		break;
	case 6:


//		condition=1;
//		loop_delay(5);
		break;


	case 7:

		memset(GNSS_data,0,sizeof(GNSS_data));
		GET_GNSS_data();
		break;

	case 8:

//		UART_Write(UART1, "AT+QGPSCFG=", 11);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "autogps", 7);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, ",", 1);
//		UART_Write(UART1, "1", 1);
//		UART_Write(UART1, "\r\n", 2);

		break;

	case 9:
//		UART_Write(UART1, "AT+QGPSGNMEA=", 13);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "RMC", 3);
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "\r\n", strlen("\r\n"));

		break;
	case 10:
		/*UART_Write(UART1, "AT+QGPSCFG=", 11);
				UART_Write(UART1, DBQUOTE, 1);
				UART_Write(UART1, "autogps", 7);
				UART_Write(UART1, DBQUOTE, 1);
				UART_Write(UART1, ",", 1);
				UART_Write(UART1, "0", 1);
				UART_Write(UART1, "\r\n", 2);*/
		break;

	case 11:
		gsmState = GSM_HTTP_REPORT_DATA;
		loopCount = 0;
		break;

	default:
		loopCount = 0;
//		if (loopDelay)
//			loopDelay--;
//		else
//			loopCount = tloopCount;
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

//		call_test=1;
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		if(((call_test)&&(sim_calling_chek))||(sms_call_chek))
		{
			loopCount = 0;
			UART_Write(UART2, "call is going on \n", strlen("call is going on \n"));
		}
		break;
	case 7:
		break;
	case 8:
//		count_1=0;
	    UART_Write(UART1, "AT+QLTS=2\r\n", strlen("AT+QLTS=2\r\n"));
		break;
	case 9:
		break;
	case 10:
		break;

	case 11:
		UART_Write(UART1, "AT+CNMI=2,2,0,0,0\r\n",strlen("AT+CNMI=2,2,0,0,0\r\n"));
		break;
	case 12:
//		count_1=0;
		// check periodically for status changes
		UART_Write_main(UART2, "\nCheck Connectivity periodically\r\n",strlen("\nCheck Connectivity periodically\r\n") );
		gsmState = GSM_CONNECT;
		loopCount = 0;
		break;
	default:
//		if(loopDelay)
//			loopDelay--;
//		else
//			loopCount = tloopCount;
		loopCount = 0;
		break;

	}

}
void  GET_GNSS_data()
{

	char *token=0;
	char temp_buf[100];
	uint8_t *tString=0;

	memset(URL,0,sizeof(URL));

//	memset(wlturlsize_url,0,sizeof(wlturlsize_url));
//
//	memset(gnssutc, 0, sizeof(gnssutc));
//	memset(gnss_lat, 0, sizeof(gnss_lat));
//	memset(gnss_lon, 0, sizeof(gnss_lon));
//	memset(gnsshdop, 0, sizeof(gnsshdop));
//	memset(gnssalt, 0, sizeof(gnssalt));
//	memset(gnssfix, 0, sizeof(gnssfix));
//	memset(gnsscog, 0, sizeof(gnsscog));
//	memset(gnssspkm, 0, sizeof(gnssspkm));
//	memset(gnssspkn, 0, sizeof(gnssspkn));
//	memset(gnssdate, 0, sizeof(gnssdate));
//	memset(gnssnsat, 0, sizeof(gnssnsat));


	tString=strstr(UART1_RX_BUF,"+QGPSLOC:");


	if (tString != NULL)
	{

		memcpy(temp_buf, tString,75);
		 temp_buf[75] = '\0';

	token = strtok(temp_buf, ":");
	token = strtok(NULL, ",");

	if(token != NULL)
	{
	    while(*token == ' ')
	        token++;            // Skip leading space

	    strcpy(gnssutc, token);
	}

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnss_lat, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnss_lon, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnsshdop, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnssalt, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnssfix, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnsscog, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnssspkm, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnssspkn, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnssdate, token);

	token = strtok(NULL, ",");
	if(token != NULL)
	    strcpy(gnssnsat, token);

//	   strcpy(fuel_sensor,"FUEL_TOUCHED");

//	strcpy(fuel_sensor,SENSOR_MESSAGE);

	    strcpy(URL,"{");

//	    strcat(URL,SENSOR_MESSAGE);

		strcat(URL,type);

//		strcat(URL,DBQUOTE);

//		strcat(URL,fuel_sensor);

		strcat(URL,SENSOR_MESSAGE);

//		strcat(URL,DBQUOTE);

		strcat(URL,",");

		strcat(URL,WLTLAT);

		strcat(URL,gnss_lat);

	    strcat(URL,",");

		strcat(URL,WLTLON);

		strcat(URL,gnss_lon);

		strcat(URL,",");

		strcat(URL,Message);

//		strcat(URL,DBQUOTE);

		strcat(URL,SENSOR_MESSAGE_RESULT);

//		strcat(URL,DBQUOTE);

//		strcat(URL, "\"Fuel Tank Cap Open\"");

//		strcat(URL,Message_r);

//		strcat(URL,WLTHEADING);
//		strcat(URL,gnsshdop);
//		strcat(URL,WLTSPEED);
//		strcat(URL,gnssspkm);
//		strcat(URL,WLTODOMETER);
//		strcat(URL,"0");
//		strcat(URL,WLTIGNITION);
//		strcat(URL,"1");
//		strcat(URL,WLTGPSDATE);
//		strcat(URL,"20");
//		strcat(URL,gnssyear);
//		strcat(URL,"-");
//		strcat(URL,gnssmon);
//		strcat(URL,"-");
//		strcat(URL,gnssday);
//		strcat(URL,"T");
//		strcat(URL,gnssutctime);
//		strcat(URL,":");
//		strcat(URL,gnssutcmin);
//		strcat(URL,":");
//		strcat(URL,gnssutcsec);
//		strcat(URL,"Z");
//		strcat(URL,WLTALTITUDE);
//		strcat(URL,gnssalt);


		strcat(URL,data_send_buffer);

		strcat(URL,"}");
		urlsize=strlen(URL);
		itoa(urlsize,wlturlsize_url,10);

		UART_Write(UART2,URL,strlen(URL));



	}

	memset(SENSOR_MESSAGE, 0, sizeof(SENSOR_MESSAGE));

		memset(SENSOR_MESSAGE_RESULT,0,sizeof(SENSOR_MESSAGE_RESULT));

		strcpy(SENSOR_MESSAGE,"\"NULL\"");

		strcpy(SENSOR_MESSAGE_RESULT,"\"NULL\"");

//

		UART_Write(UART2,"\r\n",strlen("\r\n"));

		memset(data_send_buffer,0,sizeof(data_send_buffer));
}
void check_receive_packet()
{
	if (WLC.SOF == '%') {
				crc_rec = crc_calcul(Rec_cpy, sizeof(WLC_t) - 2);
				RFM = crc_rec;
	//			crc_rec = crc_rec >> 8;
	//			crc_rec |= RFM << 8;
				memset(fuel_sensor_buff,0,sizeof(fuel_sensor_buff));
				if (crc_rec == WLC.crc){
					if (WLC.Destination == CONTROL_PANEL) {
				{
//					count_1=0;
//					UART_Write(UART1, "AT+CCLK?\r\n", strlen("AT+CCLK?\r\n"));
					  memset(fuel_sensor_buff,0,sizeof(fuel_sensor_buff));
						switch (WLC.Data)
						{
						case (CAP_STATUS):
						if ((WLC.Value[0]) == OPEN_CAP)
						{
						 BEEP_ON;

//						memcpy(fuel_sensor_buff,recived_lorry_number,13);

						 strcpy(SENSOR_MESSAGE,"\"FUEL_TANK_TOUCHED\"");

//						strncpy(fuel_sensor_buff,"fuel Cap Opened Aleart",sizeof("fuel Cap Opened Aleart"));
//						strcat(fuel_sensor_buff,"\n");
						strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
						year_buffer[sizeof(year_buffer)-1] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
					    time_buffer[sizeof(time_buffer)-1]='\0';
						strcat(fuel_sensor_buff,time_buffer);
				     	strcat(fuel_sensor_buff,"\n");

				     	recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
						strcat(fuel_sensor_buff, recived_lorry_number);

						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Alert:");
						strcat(fuel_sensor_buff," Fuel Tank Cap Open");
						strcpy(SENSOR_MESSAGE_RESULT,"\"Fuel_Tank_Cap_Open\"");
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Fuel Tank Capacity:");
						strcat(fuel_sensor_buff,WLC.Value+1);
						strcat(fuel_sensor_buff,"L");
						strcat(fuel_sensor_buff,"\n");
//						memcnpy(Message_r,"Fuel Tank Cap Open",sizeof("Fuel Tank Cap Open"));
//						send_data=1;
						}
						if ((WLC.Value[0]) == CLOSE_CAP)
						{
						 BEEP_OFF;
						 strcpy(SENSOR_MESSAGE,"\"FUEL_TANK_TOUCHED\"");
//						memcpy(fuel_sensor_buff,recived_lorry_number,13);

//							strncpy(fuel_sensor_buff,"fuel Cap Closed Aleart",sizeof("fuel Cap Closed Aleart"));
//							strcat(fuel_sensor_buff,"\n");
							strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
							year_buffer[sizeof(year_buffer)-1] ='\0';
							strcat(fuel_sensor_buff,year_buffer);
							strcat(fuel_sensor_buff,"Time:");
							time_buffer[sizeof(time_buffer)-1]='\0';
							strcat(fuel_sensor_buff,time_buffer);
							strcat(fuel_sensor_buff,"\n");

							recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
							strcat(fuel_sensor_buff, recived_lorry_number);

							strcat(fuel_sensor_buff,"\n");
//							strcat(fuel_sensor_buff,"Alert:");
							strcat(fuel_sensor_buff,"Alert: Fuel Tank Cap Closed");
							strcpy(SENSOR_MESSAGE_RESULT,"\"Fuel_Tank_Cap_Closed\"");
							strcat(fuel_sensor_buff,"\n");
							strcat(fuel_sensor_buff,"Fuel Tank Capacity:");
							strcat(fuel_sensor_buff,WLC.Value+1);
							strcat(fuel_sensor_buff,"L");
							strcat(fuel_sensor_buff,"\n");

//						strncpy(fuel_sensor_buff, recived_lorry_number, sizeof(fuel_sensor_buff)-1);
//						fuel_sensor_buff[sizeof(fuel_sensor_buff)-1] = '\0';
//						strcat(fuel_sensor_buff,"\n");
//
//						strcat(fuel_sensor_buff,"Fuel Tank Cap Close");
//
//						strcat(fuel_sensor_buff,"\n");
////						strcat(fuel_sensor_buff,"Date:");
////						year_buffer[sizeof(year_buffer)] ='\0';
////						strcat(fuel_sensor_buff,year_buffer);
////						strcat(fuel_sensor_buff,"Time:");
////						time_buffer[sizeof(time_buffer)]='\0';
////						strcat(fuel_sensor_buff,time_buffer);
////						strcat(fuel_sensor_buff,"\n");
//						strcat(fuel_sensor_buff,sms_google_buffer);
//						send_data=1;
						}
							break;
						case (BATTERY_VALUE):
////		                memcpy(fuel_sensor_buff,recived_lorry_number,13);
//
//			            strncpy(fuel_sensor_buff, recived_lorry_number, sizeof(fuel_sensor_buff)-1);
//						fuel_sensor_buff[sizeof(fuel_sensor_buff)-1] = '\0';
//
//						strcat(fuel_sensor_buff,"\n");
//						strcat(fuel_sensor_buff,"Battery level=");
//						strcat(fuel_sensor_buff,WLC.Value/*value*/);
//						strcat(fuel_sensor_buff,"\n");
////						strcat(fuel_sensor_buff,"Date:");
////						year_buffer[sizeof(year_buffer)] ='\0';
////						strcat(fuel_sensor_buff,year_buffer);
////						strcat(fuel_sensor_buff,"Time:");
////						time_buffer[sizeof(time_buffer)]='\0';
////						strcat(fuel_sensor_buff,time_buffer);
////						strcat(fuel_sensor_buff,"\n");
//						strcat(fuel_sensor_buff,sms_google_buffer);
//						send_data=1;
							break;
						case (FLOW_SENSOR):

//		                memcpy(fuel_sensor_buff,recived_lorry_number,13);

//                      strncpy(fuel_sensor_buff,"Disel Filling Alert",sizeof("Disel Filling Alert"));
//						strcat(fuel_sensor_buff,"\n");
						strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
						year_buffer[sizeof(year_buffer)-1] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
						time_buffer[sizeof(time_buffer)-1]='\0';
						strcat(fuel_sensor_buff,time_buffer);
						strcat(fuel_sensor_buff,"\n");

						recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
						strcat(fuel_sensor_buff, recived_lorry_number);

						strcat(fuel_sensor_buff,"\n");
//						strcat(fuel_sensor_buff,"Alert:");
						strcat(fuel_sensor_buff,"Alert: Diesel Filled");
						strcpy(SENSOR_MESSAGE,"\"DIESEL_FILLED\"");
						strcat(fuel_sensor_buff,"\n");

						strcat(fuel_sensor_buff,"Fuel added:");

						 ptr = strchr((char *)WLC.Value, 'F');

						if (ptr != NULL)
						{
						    size_t len = ptr -(char *) WLC.Value;

						    // Prevent buffer overflow
							if (len >= sizeof(before_f))
							len = sizeof(before_f) - 1;

						    memcpy(before_f, WLC.Value, len);
						    before_f[len] = '\0';

						    strcpy(after_f, ptr + 1);
						    after_f[sizeof(after_f) - 1] = '\0';
						}

						strcat(fuel_sensor_buff,before_f);
						strcat(fuel_sensor_buff,"L");
						strcat(fuel_sensor_buff,"\n");

						strcat(fuel_sensor_buff,"Fuel Tank Capacity:");
						strcat(fuel_sensor_buff,after_f);
						strcat(fuel_sensor_buff,"L");
						strcpy(SENSOR_MESSAGE_RESULT,DBQUOTE);
						strcat(SENSOR_MESSAGE_RESULT,after_f);
						strcat(SENSOR_MESSAGE_RESULT,"L");
						strcat(SENSOR_MESSAGE_RESULT,DBQUOTE);
//						strcat(fuel_sensor_buff,"400 L");
						strcat(fuel_sensor_buff,"\n");

//			            strncpy(fuel_sensor_buff, recived_lorry_number, sizeof(fuel_sensor_buff)-1);
//						fuel_sensor_buff[sizeof(fuel_sensor_buff)-1] = '\0';
//
//						strcat(fuel_sensor_buff,"\n");
//						strcat(fuel_sensor_buff,"Flow Sensor Count=");
//						strcat(fuel_sensor_buff,WLC.Value/*value*/);
//						strcat(fuel_sensor_buff,"\n");
////						strcat(fuel_sensor_buff,"Date:");
////						year_buffer[sizeof(year_buffer)] ='\0';
////						strcat(fuel_sensor_buff,year_buffer);
////						strcat(fuel_sensor_buff,"Time:");
////						time_buffer[sizeof(time_buffer)]='\0';
////						strcat(fuel_sensor_buff,time_buffer);
////						strcat(fuel_sensor_buff,"\n");
//						strcat(fuel_sensor_buff,sms_google_buffer);
//						send_data=1;
							break;
//						case (FUEL_GUAGE):
//
////						memcpy(fuel_sensor_buff,recived_lorry_number,13);
//
//
//
////			            strncpy(fuel_sensor_buff, recived_lorry_number, sizeof(recived_lorry_number));
////						fuel_sensor_buff[sizeof(fuel_sensor_buff)-1] = '\0';
////
////						strcat(fuel_sensor_buff,"\n");
////						strcat(fuel_sensor_buff,"Fuel level=");
////						strcat(fuel_sensor_buff,WLC.Value/*value*/);
////						strcat(fuel_sensor_buff,"L");
////						strcat(fuel_sensor_buff,"\n");
//////						strcat(fuel_sensor_buff,"Date:");
//////						year_buffer[sizeof(year_buffer)] ='\0';
//////						strcat(fuel_sensor_buff,year_buffer);
//////						strcat(fuel_sensor_buff,"Time:");
//////						time_buffer[sizeof(time_buffer)]='\0';
//////						strcat(fuel_sensor_buff,time_buffer);
//////						strcat(fuel_sensor_buff,"\n");
////						strcat(fuel_sensor_buff,sms_google_buffer);
////						send_data=1;
//							break;
						case (THAFT):
					   if(knotspeed<=5)
						{
		                BEEP_ON;

//						PWM_Start(PWM0, 0x28);
//		                buzzeR_sound=1;

//		                memcpy(fuel_sensor_buff,recived_lorry_number,13);

//		                strncpy(fuel_sensor_buff,"Disel Theft Alert",sizeof("Disel Theft Alert"));
//		                strcat(fuel_sensor_buff,"\n");
						strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
						year_buffer[sizeof(year_buffer)-1] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
						time_buffer[sizeof(time_buffer)-1]='\0';
						strcat(fuel_sensor_buff,time_buffer);
						strcat(fuel_sensor_buff,"\n");

						recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
						strcat(fuel_sensor_buff, recived_lorry_number);

						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Alert:");
						strcat(fuel_sensor_buff," Suspected Diesel Theft");
						strcpy(SENSOR_MESSAGE,"\"SUSPECTED_DIESEL_THEFT\"");
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Device Mobile No:");
						recived_mobile_number[sizeof(recived_mobile_number)-1]='\0';
						strcat(fuel_sensor_buff, recived_mobile_number);
						strcat(fuel_sensor_buff,"\n");

						strcat(fuel_sensor_buff,"Fuel Tank Capacity:");
						strcat(fuel_sensor_buff,WLC.Value+1);
						strcpy(SENSOR_MESSAGE_RESULT,DBQUOTE);
						strcat(SENSOR_MESSAGE_RESULT,WLC.Value+1);
						strcat(fuel_sensor_buff,"L");
						strcat(SENSOR_MESSAGE_RESULT,DBQUOTE);
						strcat(fuel_sensor_buff,"\n");


//		            	strncpy(fuel_sensor_buff, recived_lorry_number, sizeof(recived_lorry_number));
//		            	fuel_sensor_buff[sizeof(fuel_sensor_buff)-1] = '\0';
//						strcat(fuel_sensor_buff,"\n");
//		                strcat(fuel_sensor_buff,"DROP:");
//		                strcat(fuel_sensor_buff,WLC.Value/*value*/);
//		                strcat(fuel_sensor_buff," ");
//		                strcat(fuel_sensor_buff,"Phone:+");
////		                strcat(fuel_sensor_buff,recived_mobile_number);
//		           	    recived_mobile_number[sizeof(recived_mobile_number)]='\0';
//		           	    strcat(fuel_sensor_buff, recived_mobile_number);
////		            	strncpy(fuel_sensor_buff, recived_mobile_number, 10);
////		            	fuel_sensor_buff[sizeof(fuel_sensor_buff)-1] = '\0';
//
//		            	strcat(fuel_sensor_buff,"\n");
////						strcat(fuel_sensor_buff,"Date:");
////						year_buffer[sizeof(year_buffer)] ='\0';
////						strcat(fuel_sensor_buff,year_buffer);
////						strcat(fuel_sensor_buff,"Time:");
////						time_buffer[sizeof(time_buffer)]='\0';
////						strcat(fuel_sensor_buff,time_buffer);
////		                strcat(fuel_sensor_buff,"\n");
//		                strcat(fuel_sensor_buff,sms_google_buffer);
//		                send_data=1;
						}
						break;

			           case (DISTANCE):

//				       strncpy(fuel_sensor_buff,"fuel Tank Touched Aleart",sizeof("fuel Tank Touched Aleart"));
//			           strcat(fuel_sensor_buff,"\n");
			           strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
					   year_buffer[sizeof(year_buffer)-1] ='\0';
					   strcat(fuel_sensor_buff,year_buffer);
					   strcat(fuel_sensor_buff,"Time:");
					   time_buffer[sizeof(time_buffer)-1]='\0';
					   strcat(fuel_sensor_buff,time_buffer);
					   strcat(fuel_sensor_buff,"\n");

					   recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
					   strcat(fuel_sensor_buff, recived_lorry_number);

					   strcat(fuel_sensor_buff,"\n");
					   strcat(fuel_sensor_buff,"Alert: Fuel Tank Touched");
					   strcpy(SENSOR_MESSAGE,"\"FUEL_TANK_TOUCHED\"");
					   strcat(fuel_sensor_buff,"\n");
					   strcat(fuel_sensor_buff,"Fuel Tank Capacity:");
					   strcat(fuel_sensor_buff,WLC.Value+1);
					   strcpy(SENSOR_MESSAGE_RESULT,DBQUOTE);
					   strcat(SENSOR_MESSAGE_RESULT,WLC.Value+1);
					   strcat(SENSOR_MESSAGE_RESULT,"L");
					   strcat(SENSOR_MESSAGE_RESULT,DBQUOTE);
					   strcat(fuel_sensor_buff,"\n");

////		           strncpy (fuel_sensor_buff, "Diesel Tank Person Detected\n",sizeof("Diesel Tank Person Detected\n"));
//			           strncpy(fuel_sensor_buff, recived_lorry_number, sizeof(recived_lorry_number));
//			           fuel_sensor_buff[sizeof(fuel_sensor_buff)-1] = '\0';
//					   strcat(fuel_sensor_buff,"\n");
//					   strcat(fuel_sensor_buff, "Diesel Tank Person Detected\n");
					    break;

			           case (BATTERY_SATUS):
//	    	            strncpy(fuel_sensor_buff,"Battery Theft /Removal Alert",sizeof("Battery Theft /Removal Alert"));
//						strcat(fuel_sensor_buff,"\n");
		                strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
						year_buffer[sizeof(year_buffer)-1] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
						time_buffer[sizeof(time_buffer)-1]='\0';
						strcat(fuel_sensor_buff,time_buffer);
						strcat(fuel_sensor_buff,"\n");

						recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
						strcat(fuel_sensor_buff, recived_lorry_number);

						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Alert:");
						strcat(fuel_sensor_buff," Suspected Vehicle Battery theft!");
						strcpy(SENSOR_MESSAGE,"\"SUSPECTED_BATTERY_THEFT\"");
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Device Mobile No:");
						recived_mobile_number[sizeof(recived_mobile_number)-1]='\0';
						strcat(fuel_sensor_buff, recived_mobile_number);
						strcat(fuel_sensor_buff,"\n");
//						strcat(fuel_sensor_buff,"Fuel Tank Capacity:");
//						strcat(fuel_sensor_buff,WLC.Value+1);
//						strcat(fuel_sensor_buff,"L");
//						strcat(fuel_sensor_buff,"\n");
			           break;

			           case(AFTER_THAFT):
//		                strncpy(fuel_sensor_buff,"After_Theft_Disel Theft Alert",sizeof("After_Disel Theft Alert"));
//						strcat(fuel_sensor_buff,"\n");
		                strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
						year_buffer[sizeof(year_buffer)-1] ='\0';
						strcat(fuel_sensor_buff,year_buffer);
						strcat(fuel_sensor_buff,"Time:");
						time_buffer[sizeof(time_buffer)-1]='\0';
						strcat(fuel_sensor_buff,time_buffer);
						strcat(fuel_sensor_buff,"\n");

						recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
						strcat(fuel_sensor_buff, recived_lorry_number);

						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Alert:");
						strcat(fuel_sensor_buff," Diesel After Theft");
						strcpy(SENSOR_MESSAGE,"\"DIESEL_AFTER_THEFT\"");
						strcat(fuel_sensor_buff,"\n");
						strcat(fuel_sensor_buff,"Device Mobile No:");
						recived_mobile_number[sizeof(recived_mobile_number)-1]='\0';
						strcat(fuel_sensor_buff, recived_mobile_number);
						strcat(fuel_sensor_buff,"\n");

						strcat(fuel_sensor_buff,"Fuel Theft Capacity:");
						strcat(fuel_sensor_buff,WLC.Value+1);
						strcpy(SENSOR_MESSAGE_RESULT,DBQUOTE);
						strcat(SENSOR_MESSAGE_RESULT,WLC.Value+1);
						strcat(SENSOR_MESSAGE_RESULT,"L");
						strcat(SENSOR_MESSAGE_RESULT,DBQUOTE);
						strcat(fuel_sensor_buff,"L");
						strcat(fuel_sensor_buff,"\n");
			           break;

					default:
					   break;
						}

						memset(&WLC, 0, sizeof(WLC_t));
						message_sending=1;
//						gps_data_send_sms();
//						if(send_data)
//						{
////						send_data=0;
//						GET_GNSS_data();
//
//						strcat(fuel_sensor_buff,"https://www.google.com/maps?q=");
//						gnss_lat[sizeof(gnss_lat)-1]='\0';
//						strcat(fuel_sensor_buff, gnss_lat);
//						strcat(fuel_sensor_buff, ",");
//						gnss_lon[sizeof(gnss_lon)-1]='\0';
//						strcat(fuel_sensor_buff, gnss_lon);
//						sensor_data_sending_to_sms();
//					    memset(&WLC, 0, sizeof(WLC_t));


//						strcat(fuel_sensor_buff,"\n");
//						google_map_url[sizeof(google_map_url)]='\0';
//						strcat(fuel_sensor_buff,google_map_url);
//						strcat(fuel_sensor_buff,"https://WWW.google.com/maps?q=");
//						strcat(fuel_sensor_buff,WLTLAT);
//						gnss_lat[sizeof(gnss_lat)-1]='\0';
//						strcat(fuel_sensor_buff,gnss_lat);
//						strcat(fuel_sensor_buff,",");
//						strcat(fuel_sensor_buff,WLTLON);
//						gnss_lon[sizeof(gnss_lon)-1]='\0';
//						strcat(fuel_sensor_buff,gnss_lon);
//						strcat(fuel_sensor_buff,"\n");
//						memcpy(data_send_buffer,fuel_sensor_buff,sizeof(fuel_sensor_buff));

//						}
					}

				}
			}
		}
}
void panic_alert()
{

//	 memcpy(fuel_sensor_buff,recived_lorry_number,13);

//	       strncpy(fuel_sensor_buff,"Panic Alert",sizeof("Panic Alert"));
//	       strcat(fuel_sensor_buff,"\n");
		   strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
		   year_buffer[sizeof(year_buffer)-1] ='\0';
		   strcat(fuel_sensor_buff,year_buffer);
		   strcat(fuel_sensor_buff,"Time:");
		   time_buffer[sizeof(time_buffer)-1]='\0';
		   strcat(fuel_sensor_buff,time_buffer);
		   strcat(fuel_sensor_buff,"\n");

		   recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
		   strcat(fuel_sensor_buff, recived_lorry_number);

		   strcat(fuel_sensor_buff,"\n");
		   strcat(fuel_sensor_buff,"Alert: Panic! Immediate Assistance Required");
		   strcpy(SENSOR_MESSAGE,"\"PANIC_ALERT\"");
		   strcpy(SENSOR_MESSAGE_RESULT,"\"Immediate_Assistance_Required\"");
		   strcat(fuel_sensor_buff,"\n");
		   strcat(fuel_sensor_buff,"Device Mobile No:");
		   recived_mobile_number[sizeof(recived_mobile_number)-1]='\0';
		   strcat(fuel_sensor_buff, recived_mobile_number);
//		   strcat(fuel_sensor_buff,"\n");
//		   strcat(fuel_sensor_buff,"Fuel Tank Capacity:");
//		   strcat(fuel_sensor_buff,WLC.Value+1);
//		   strcat(fuel_sensor_buff,"L");
		   strcat(fuel_sensor_buff,"\n");
		   message_sending=1;
//		   gps_data_send_sms();

//		    GET_GNSS_data();
//	 //     strcat(fuel_sensor_buff,"\n");
//			google_map_url[strlen(google_map_url)]='\0';
//			strcat(fuel_sensor_buff,google_map_url);
//			strcat(fuel_sensor_buff,WLTLAT);
//			gnss_lat[sizeof(gnss_lat)-1]='\0';
//			strcat(fuel_sensor_buff,gnss_lat);
//			strcat(fuel_sensor_buff,",");
//			strcat(fuel_sensor_buff,WLTLON);
//			gnss_lon[sizeof(gnss_lon)-1]='\0';
//			strcat(fuel_sensor_buff,gnss_lon);
//	//		strcat(URL,"\n");
//	//		memcpy(data_send_buffer,fuel_sensor_buff,sizeof(fuel_sensor_buff));
//			sensor_data_sending_to_sms();

//	 strncpy(fuel_sensor_buff, recived_lorry_number, sizeof(fuel_sensor_buff)-1);
//	 fuel_sensor_buff[sizeof(fuel_sensor_buff)-1] = '\0';
//	 strcat(fuel_sensor_buff,"\n");
//	 strcat(fuel_sensor_buff,"Panic Alert!Help Needed:");
//	 strcat(fuel_sensor_buff,"\n");
//	 strcat(fuel_sensor_buff,"Immediately");
//	 strcat(fuel_sensor_buff,"Phone:+");
//	 recived_mobile_number[sizeof(recived_mobile_number)]='\0';
//	 strcat(fuel_sensor_buff,recived_mobile_number);
////	 memcpy(fuel_sensor_buff,recived_mobile_number,/*sizeof(recived_mobile_number)*/10);
////	 strcat(fuel_sensor_buff,"\n");
////	 strcat(fuel_sensor_buff,"Date:");
////	 strcat(fuel_sensor_buff,year_buffer);
////	 strcat(fuel_sensor_buff,"Time:");
////	 strcat(fuel_sensor_buff,time_buffer);
//	 strcat(fuel_sensor_buff,"\n");
//	 strcat(fuel_sensor_buff,sms_google_buffer);
}

void sensor_data_sending_to_sms()
{

	sms_call_chek=1;

//	UART_Write(UART1, "AT+CMGF=1\r\n", strlen("AT+CMGF=1\r\n"));
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "9032888261", strlen("9032888261"));
//		UART_Write(UART1, DBQUOTE, 1);
//		UART_Write(UART1, "\r\n", 2);

	delayCounter = 1000000;
	delay_loop();

	UART_Write(UART1, "AT+CMGS=", strlen("AT+CMGS="));
	UART_Write(UART1, DBQUOTE, 1);
	UART_Write(UART1, mobile_number, strlen(mobile_number));
	UART_Write(UART1, DBQUOTE, 1);
	UART_Write(UART1, "\r\n", 2);


	delayCounter = 100000;
	delay_loop();


	length = strlen(fuel_sensor_buff);

	fuel_sensor_buff[length]     = 0x1A;
	fuel_sensor_buff[length + 1] = '\0';

	UART_Write(UART1, fuel_sensor_buff, length+1);

//	fuel_sensor_buff[strlen(fuel_sensor_buff)]=0X1A;
//
//	UART_Write(UART1,fuel_sensor_buff +1, strlen(fuel_sensor_buff+1));


	memset(fuel_sensor_buff,0,sizeof(fuel_sensor_buff));


	sms_call_chek=0;

//	memset(fuel_sensor_buff,0,sizeof(fuel_sensor_buff));

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

     memcpy(sms_google_buffer,google_map_url,sizeof(google_map_url));


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

//		 	strcat(sms_google_buffer,"\"");
			strcat(data_send_buffer, "\"");
			strcat(data_send_buffer,gpsLatitude);
//			strcat(sms_google_buffer,gpsLatitude);
//			strcat(sms_google_buffer,"\"");
			strcat(data_send_buffer, "\"");
			strcat(data_send_buffer,":");
//			strcat(sms_google_buffer,":");



				lat_dege=atoi(lat_deg);

				 if((lat_dege>7) && (lat_dege<38))
				 {
				   strcat(data_send_buffer,lat_deg);

				   strcat(sms_google_buffer,lat_deg);

					strcat(data_send_buffer,".");

					strcat(sms_google_buffer,".");

					 strcat(data_send_buffer,LAT_min_cov);

					 strcat(sms_google_buffer,LAT_min_cov);

					   lat_dege=0;
				 }
				 strcat(data_send_buffer,",");

				strcat(data_send_buffer, "\"");
//				strcat(sms_google_buffer,"\"");
				strcat(data_send_buffer,gpsLongitude);
//				strcat(sms_google_buffer,gpsLongitude);
				strcat(data_send_buffer, "\"");
//				strcat(sms_google_buffer,"\"");
				strcat(data_send_buffer,":");
//				strcat(sms_google_buffer,":");

				long_dege=atoi(long_deg);



				 if((long_dege>60) && (long_dege<97))
				 {
					 strcat(sms_google_buffer,",");

			         	strcat(data_send_buffer,long_deg+1);

			         	strcat(sms_google_buffer,long_deg+1);

			               	strcat(data_send_buffer,".");

			            	strcat(sms_google_buffer,".");

			              	strcat(data_send_buffer,LONG_min_cov);

			              	strcat(sms_google_buffer,LONG_min_cov);

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

//		send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,dates);

	   // date_and_time=atoi(dates);

	//    send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,date_and_time);

	    send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,sms_google_buffer+29);


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
//	loopCount=0;
//	gsmState=GSM_SLEEP;
	UART_Write(UART1, "ATD", strlen("ATD"));
	UART_Write(UART1, recived_mobile_number, strlen(recived_mobile_number));
//	UART_Write(UART1, "9032888261",strlen("9032888261"));
	UART_Write(UART1, ";", 1);
	UART_Write(UART1, "\r\n", 2);
	calling_chek=0;
	emergency_chek=0;
	police_chek=0;
//	gsmState=GSM_SLEEP;
//	loopCount=0;

}
void date_time_function()
{
	char *token=0;

	memset(year_buffer, 0, sizeof(year_buffer));
	memset(time_buffer, 0, sizeof(time_buffer));

	if(strstr(set_date_time_buffer,"+QLTS: ")!=NULL)
	{

		date_and_time_pointer = strstr(set_date_time_buffer, "+QLTS: ");

		if(date_and_time_pointer == NULL)
			{
			    return;     // Invalid response
			}

		 date_and_time_pointer = strstr(date_and_time_pointer, "\"");

		 if(date_and_time_pointer == NULL)
		 {
		     return;
		 }

	    if(date_and_time_pointer != NULL)
	    {
	        date_and_time_pointer++;     // Skip the first "

	         token = strtok(date_and_time_pointer, ",");

	         if(token == NULL)
	        	 return;

	         if(token != NULL)
	           strncpy(year_buffer, token,sizeof(year_buffer));

	         token = strtok(NULL, "+");

	         if(token == NULL)
	        	   return;

	          if(token != NULL)
	            strncpy(time_buffer, token,sizeof(time_buffer));

	          strcpy((char *)datetime, (char *)year_buffer);
	          strcat((char *)datetime, " ");
	          strcat((char *)datetime, (char *)time_buffer);


//	          send(FUEL_SENSOR,CONTROL_PANEL,FUEL_SENSOR_ID,CONTROL_PANEL_ID,CAP_STATUS,datetime);
	    }

	}
}

void register_the_mobile_number()
{
	uint8_t i=0;

	strncpy(phone_number_buffer_1,phone_number_buffer+10,10);

	phone_number_buffer_1[10] = '\0';

if((strncmp(phone_number_buffer_1,num1,10)==0))
  {
//	memset(phone_number_buffer_1,0,sizeof(phone_number_buffer_1));

	 mobile_number_pointer=(strstr(phone_number_buffer,num1));

		if (mobile_number_pointer != NULL)
		{
		    strncpy(num2, mobile_number_pointer, 10);

		    num2[10] = '\0';

		   mobile_number_pointer=(strstr(phone_number_buffer,"SET PHONE +91"));

		 if (mobile_number_pointer != NULL)
		    {

		        strncpy(recived_password,mobile_number_pointer+23,(11));

		           recived_password[11] = '\0';

	          if ((strncmp(num1, num2, 10) == 0)&&(strncmp(password, recived_password, 11) == 0))
	              {

	          //	mobile_number_pointer=(strstr(&phone_number_buffer,"SET PHONE +91"));

		            strncpy(mobile_number_arry_1,mobile_number_pointer+13,(10));

		            mobile_number_arry_1[10] = '\0';

		            mobile_number_counter=0;

//		            while((mobile_number_arry_1) && mobile_number_counter<sizeof(mobile_number_arry_1))

		            while(mobile_number_counter<10)
		            {
		               if (isdigit((unsigned char)mobile_number_arry_1[mobile_number_counter]))
		                {

			              mobile_number_arry[mobile_number_counter]=mobile_number_arry_1[mobile_number_counter];

			                  mobile_number_counter++;
		                  }
			                else
			                  {
			              	    strcpy(fuel_sensor_buff,"enter_the_number_is_worng");
			                 	sensor_data_sending_to_sms();
				                break; // stop at first non-digit
			                   }


		                 }
							mobile_number_arry[mobile_number_counter] = '\0';

							data_write_flash_write(mobile_number_arry,phone_numbe_is_avaliable,data_flash_size);

							regisrter_mobile_number=1;

							simcard_checking_command();

							strcpy(fuel_sensor_buff,"adding_mobile_numebr_successfully");

							sensor_data_sending_to_sms();
							memset(phone_number_buffer,0,sizeof(phone_number_buffer));
							memset(num2,0,sizeof(num2));
							memset(recived_password,0,sizeof(recived_password));
							read_data_form_data_flash=1;

						   }
		           }

	        else
		      {

				mobile_number_pointer=(strstr(phone_number_buffer,"SET VEHICLE"));

				if (mobile_number_pointer != NULL)
				{

				strncpy(recived_password,mobile_number_pointer+25,(11));

				recived_password[11] = '\0';

				if ((strncmp(num1, num2, 10) == 0)&&(strncmp(password, recived_password, 11) == 0))
				   {
					  mobile_number_arry[13]='\0';

						data_write_flash_write((memcpy(mobile_number_arry,mobile_number_pointer+12,13)),vechical_number_avalible,data_flash_size);

						strcpy(fuel_sensor_buff,"adding_vehicle_numebr_successfully");
						sensor_data_sending_to_sms();
						memset(phone_number_buffer,0,sizeof(phone_number_buffer));
						memset(num2,0,sizeof(num2));
						memset(recived_password,0,sizeof(recived_password));
						read_data_form_data_flash=1;
				   }
		     }
	     }
	   }
	 }
 else
  {
	    memset(phone_number_buffer_1,0,sizeof(phone_number_buffer_1));

	    mobile_number_pointer=(strstr(phone_number_buffer,"+91"));

	if (mobile_number_pointer != NULL)
	{

	    strncpy(num2,mobile_number_pointer+3,(10));

	    num2[10] = '\0';

	    	for (i = 0; i < 10; i++)
	    	{
	    	    if (!isdigit((unsigned char)num2[i]))
	    	    {
	    	    	memset(phone_number_buffer, 0, sizeof(phone_number_buffer));
	    	    	memset(num2, 0, sizeof(num2));
	    	        return;
	    	    }
	    	}
	    if((strncmp(num1, num2, 10)!=0)/*&&(mobile_number_counter==10)*/)
	    {
            memset(fuel_sensor_buff,0,sizeof(fuel_sensor_buff));
	    	strcpy(fuel_sensor_buff,"Security Alert: An attempt was made to change the registered details from number ");
	    	strcat(fuel_sensor_buff,num2);
	    	memset(phone_number_buffer,0,sizeof(phone_number_buffer));
	    	sensor_data_sending_to_sms();
	    	memset(num2,0,sizeof(num2));
	    	memset(mobile_number_arry_1, 0, sizeof(mobile_number_arry_1));
	    }
	  }
   }

}
void  default_at_commands()
{

	      delayCounter = 200000;
		  delay_loop();

	      UART_Write(UART1, "AT+CMGD=1,4\r\n",strlen("AT+CMGD=1,4\r\n"));

		  delayCounter = 200000;
		  delay_loop();

		  UART_Write(UART1, "AT+CNMI=2,2,0,0,0\r\n",strlen("AT+CNMI=2,2,0,0,0\r\n"));

	      delayCounter = 200000;
	      delay_loop();

	      UART_Write(UART1, "AT+CREG?\r\n", strlen("AT+CREG?\r\n"));

	      delayCounter = 300000;
	      delay_loop();

	      UART_Write(UART1, "AT+CMGF=1\r\n",strlen("AT+CMGF=1\r\n"));

	      delayCounter = 200000;
	      delay_loop();

		 UART_Write(UART1, "AT+CSCS=",strlen("AT+CSCS="));
	     UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
	     UART_Write(UART1, "GSM",strlen("GSM"));
	     UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
	     UART_Write(UART1, "\r\n",strlen("\r\n"));

	     delayCounter = 200000;
		 delay_loop();

		 UART_Write(UART1, "AT+CMGL=",strlen("AT+CMGL="));
		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
		 UART_Write(UART1, "ALL",strlen("ALL"));
		 UART_Write(UART1, DBQUOTE,sizeof(DBQUOTE));
		 UART_Write(UART1, "\r\n",strlen("\r\n"));



		 delayCounter = 200000;
		 delay_loop();

	     UART_Write(UART1, "AT+CTZU=1\r\n", strlen("AT+CTZU=1\r\n"));

	     delayCounter = 200000;
	     delay_loop();

	     UART_Write(UART1, "AT+CMGR=2\r\n", strlen("AT+CMGR=2\r\n"));

	     delayCounter = 200000;
	   	 delay_loop();

	     UART_Write(UART1, "AT+CGREG=2\r\n", strlen("AT+CGREG=2\r\n"));

	     delayCounter = 2000000;
	     delay_loop();

//	     UART_Write(UART1, "AT+CSCS=\"GSM\"\r\n", strlen("AT+CSCS=\"\"\r\n"));
//
//	     delayCounter = 2000000;
//	     delay_loop();

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

void buzzer_sound()
{
	PWM_ConfigOutputChannel(PWM0, 3, 1000, 50);
	delayCounter = 100000;
    delay_loop();
    PWM_ConfigOutputChannel(PWM0, 3, 1100, 50);
    delayCounter = 100000;
    delay_loop();
    PWM_ConfigOutputChannel(PWM0, 3, 1180, 50);
    delayCounter = 100000;
    delay_loop();
    PWM_ConfigOutputChannel(PWM0, 3, 1300, 50);
    delayCounter = 100000;
    delay_loop();
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
void int_and_ext_battery_percentges()
{
	delayCounter = 100000;
	delay_loop();

// Read the latest ADC values for the internal and external battery channels
   ADC_measure();

   // Convert the internal battery ADC count (0-4095) to the ADC pin voltage (0-3.3V)
   intBattV_conv = ((float)intBatt_V * 3.3f) / 4095.0f;

   // Convert the ADC pin voltage back to the actual internal battery voltage
   // using the voltage divider ratio (100k + 10k) / 10k = 11
   int_battery_voltage = intBattV_conv * ((100 + 10) / 10);

   // Calculate the calibration factor.
   // 12.09V is the battery voltage measured with a multimeter.
   // Divide the actual measured voltage by the calculated voltage.
//    scale = 12.09f / int_battery_voltage;

   scale=1.106;

   // Apply the calibration factor to compensate for ADC and resistor tolerances.
   int_battery_voltage *= scale;

//   if ((int_battery_voltage <= 5.0f) && (battery_sms_sent == 0))
//   {
//	   battery_sms_sent = 1;
//       send_sms_battery();
//
//   }
//
//   if (int_battery_voltage >= 6.0f)
//   {
//       battery_sms_sent = 0;    // Battery connected again
//   }

   // Calculate the battery percentage based on the measured battery voltage.
   // Assumptions:
   // 12.6 V = 100% charged
   // 10.5 V = 0% charged

   battery_percentage = ((int_battery_voltage - 10.5f) / (12.6f - 10.5f)) * 100.0f;

   // If the calculated percentage is greater than 100%,
   // limit it to 100%.
   if (battery_percentage > 100.0f)
   {
       battery_percentage = 100.0f;
   }

   // If the calculated percentage is less than 0%,
   // limit it to 0%.
   if (battery_percentage < 0.0f)
   {
       battery_percentage = 0.0f;
   }


   // Convert the external battery ADC count (0-4095) to the ADC pin voltage (0-3.3V)
//   extBattV_conv = ((float)extBatt_V * 3.3f) / 4095.0f;;
//
//
//   // Convert the external battery voltage using the same divider ratio.
//   // (Calibration is not applied here.)
//   ext_battery_voltage = extBattV_conv * ((100 + 10) / 10);

   if ((extBatt_V <= 600) && (battery_sms_sent == 0))
   {
	   battery_sms_sent = 1;
//       send_sms_battery();

   }

   if (extBatt_V >= 601)
   {
       battery_sms_sent = 0;    // Battery connected again
   }
}
void  send_sms_battery()
{

	BEEP_ON;

//    strncpy(fuel_sensor_buff,"Battery Theft /Removal Alert",sizeof("Battery Theft /Removal Alert"));
//				strcat(fuel_sensor_buff,"\n");
				strncpy(fuel_sensor_buff,"Date:",strlen("Date:"));
				year_buffer[sizeof(year_buffer)-1] ='\0';
				strcat(fuel_sensor_buff,year_buffer);
				strcat(fuel_sensor_buff,"Time:");
				time_buffer[sizeof(time_buffer)-1]='\0';
				strcat(fuel_sensor_buff,time_buffer);
				strcat(fuel_sensor_buff,"\n");

				recived_lorry_number[sizeof(recived_lorry_number) - 1] = '\0';
				strcat(fuel_sensor_buff, recived_lorry_number);

				strcat(fuel_sensor_buff,"\n");
				strcat(fuel_sensor_buff,"Alert:");
				strcat(fuel_sensor_buff,"Suspected Vehicle Battery theft!");
				strcat(fuel_sensor_buff,"\n");
				strcat(fuel_sensor_buff,"Device Mobile No:");
				recived_mobile_number[sizeof(recived_mobile_number)-1]='\0';
				strcat(fuel_sensor_buff, recived_mobile_number);
				strcat(fuel_sensor_buff,"\n");
				strcat(fuel_sensor_buff,"Fuel Tank Capacity:");
				strcat(fuel_sensor_buff,WLC.Value+1);
				strcat(fuel_sensor_buff,"L");
				strcat(fuel_sensor_buff,"\n");

				gps_data_send_sms();

//				GET_GNSS_data();
//	//			strcat(fuel_sensor_buff,"\n");
//				google_map_url[strlen(google_map_url)]='\0';
//				strcat(fuel_sensor_buff,google_map_url);
//				strcat(fuel_sensor_buff,WLTLAT);
//				gnss_lat[sizeof(gnss_lat)-1]='\0';
//				strcat(fuel_sensor_buff,gnss_lat);
//				strcat(fuel_sensor_buff,",");
//				strcat(fuel_sensor_buff,WLTLON);
//				gnss_lon[sizeof(gnss_lon)-1]='\0';
//				strcat(fuel_sensor_buff,gnss_lon);
//	//			strcat(URL,"\n");
//	//			memcpy(data_send_buffer,fuel_sensor_buff,sizeof(fuel_sensor_buff));
//				sensor_data_sending_to_sms();
}
void gps_data_send_sms()
{
	GET_GNSS_data();

	strcat(fuel_sensor_buff,"https://www.google.com/maps?q=");
    gnss_lat[sizeof(gnss_lat)-1]='\0';
	strcat(fuel_sensor_buff, gnss_lat);
	strcat(fuel_sensor_buff, ",");
	gnss_lon[sizeof(gnss_lon)-1]='\0';
	strcat(fuel_sensor_buff, gnss_lon);
	sensor_data_sending_to_sms();

}
