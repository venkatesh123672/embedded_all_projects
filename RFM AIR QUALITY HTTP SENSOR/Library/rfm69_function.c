#include "rfm69_function.h"
#include "rfm69_registor.h"
#include <stdio.h>
#include "M0564.h"
/*******************************************************************
** Global variables                                               **
*******************************************************************/
uint8_t RFState = RF_STOP;                // RF state machine
static   uint8_t *pRFFrame;               // Pointer to the RF frame
static   uint8_t  RFFramePos;             // RF payload current position
static   uint8_t  RFFrameSize;            // RF payload size
static   uint16_t ByteCounter = 0;        // RF payload byte counter
static   uint8_t  PreMode = RF_STANDBY;   // Previous chip operating mode
static   uint8_t  SyncSize = 8;           // Size of sync word
static   uint8_t  SyncValue[8];           // Value of sync word

extern uint8_t  txFlag;
uint8_t value23;

uint16_t ReadRegister(uint8_t address)
{
	 address = address & 0x7F;
	 SPI_SET_SS_LOW(SPI0);
	 SPI_WRITE_TX(SPI0, address);
     TIMER_Delay(TIMER1,1);
	 SPI_WRITE_TX(SPI0, 0x00);
	 value23=SPI_READ_RX(SPI0);
	 TIMER_Delay(TIMER1,1);
	 SPI_SET_SS_HIGH(SPI0);
	return value23;
}
void WriteRegisterister(uint8_t address, uint16_t value)
{
	  address = address | 0x80;
	  SPI_SET_SS_LOW(SPI0);
	  SPI_WRITE_TX(SPI0, address);
	  TIMER_Delay(TIMER1,1);
	  SPI_WRITE_TX(SPI0, value);
	  TIMER_Delay(TIMER1,1);
	  SPI_SET_SS_HIGH(SPI0);

}




 uint8_t RegistersCfg[] = {          // RFM69  configuration registers values
	DEF_FIFO,                               // Left for convenience, not to be changed
	DEF_OPMODE | RF_OPMODE_SEQUENCER_OFF | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY,
	DEF_DATAMODUL | RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00,
	DEF_BITRATEMSB | RF_BITRATEMSB_4800,
	DEF_BITRATELSB | RF_BITRATELSB_4800,
	DEF_FDEVMSB | RF_FDEVMSB_5000,
	DEF_FDEVLSB | RF_FDEVLSB_5000,
	DEF_FRFMSB | RF_FRFMSB_865,
	DEF_FRFMID | RF_FRFMID_865,
	DEF_FRFLSB | RF_FRFLSB_865,
	DEF_OSC1,
	DEF_OSC2,
	DEF_LOWBAT | RF_LOWBAT_OFF | RF_LOWBAT_TRIM_1835,
	DEF_LISTEN1 | RF_LISTEN1_RESOL_4100 | RF_LISTEN1_CRITERIA_RSSI | RF_LISTEN1_END_01,
	DEF_LISTEN2 | RF_LISTEN2_COEFIDLE_VALUE,
	DEF_LISTEN3 | RF_LISTEN3_COEFRX_VALUE,
	DEF_VERSION, 			                   // Read Only

	DEF_PALEVEL | RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | RF_PALEVEL_OUTPUTPOWER_11111,
	DEF_PARAMP | RF_PARAMP_40,
	DEF_OCP | RF_OCP_ON | RF_OCP_TRIM_100,

	DEF_AGCREF | RF_AGCREF_AUTO_ON | RF_AGCREF_LEVEL_MINUS80,
	DEF_AGCTHRESH1 | RF_AGCTHRESH1_SNRMARGIN_101 | RF_AGCTHRESH1_STEP1_16,
	DEF_AGCTHRESH2 | RF_AGCTHRESH2_STEP2_3 | RF_AGCTHRESH2_STEP3_11,
	DEF_AGCTHRESH3 | RF_AGCTHRESH3_STEP4_9 | RF_AGCTHRESH3_STEP5_11,
	DEF_LNA | RF_LNA_ZIN_200 | RF_LNA_LOWPOWER_OFF | RF_LNA_GAINSELECT_AUTO,
	DEF_RXBW | RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5,
	DEF_AFCBW | RF_AFCBW_DCCFREQAFC_100 | RF_AFCBW_MANTAFC_20 | RF_AFCBW_EXPAFC_3,
	DEF_OOKPEAK | RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000,
	DEF_OOKAVG | RF_OOKAVG_AVERAGETHRESHFILT_10,
	DEF_OOKFIX | RF_OOKFIX_FIXEDTHRESH_VALUE,
	DEF_AFCFEI | RF_AFCFEI_AFCAUTOCLEAR_OFF | RF_AFCFEI_AFCAUTO_OFF,
	DEF_AFCMSB, 			                      // Read Only
	DEF_AFCLSB, 			                      // Read Only
	DEF_FEIMSB, 			                      // Read Only
	DEF_FEILSB, 			                      // Read Only
	DEF_RSSICONFIG | RF_RSSI_FASTRX_OFF,
	DEF_RSSIVALUE,  		                    // Read Only

	DEF_DIOMAPPING1 | RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_00 | RF_DIOMAPPING1_DIO3_00,
	DEF_DIOMAPPING2 | RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_01 | RF_DIOMAPPING2_CLKOUT_OFF,
	DEF_IRQFLAGS1,
	DEF_IRQFLAGS2,
	DEF_RSSITHRESH | 228,	                  // Must be set to (-Sensitivity x 2)
	DEF_RXTIMEOUT1 | RF_RXTIMEOUT1_RXSTART_VALUE,
	DEF_RXTIMEOUT2 | RF_RXTIMEOUT2_RSSITHRESH_VALUE,

	DEF_PREAMBLEMSB | RF_PREAMBLESIZE_MSB_VALUE,
	DEF_PREAMBLELSB | RF_PREAMBLESIZE_LSB_VALUE,
	DEF_SYNCCONFIG | RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_4 | RF_SYNC_TOL_0,
	DEF_SYNCVALUE1 | 0x69,
	DEF_SYNCVALUE2 | 0x81,
	DEF_SYNCVALUE3 | 0x7E,
	DEF_SYNCVALUE4 | 0x96,
	DEF_SYNCVALUE5 | RF_SYNC_BYTE5_VALUE,
	DEF_SYNCVALUE6 | RF_SYNC_BYTE6_VALUE,
	DEF_SYNCVALUE7 | RF_SYNC_BYTE7_VALUE,
	DEF_SYNCVALUE8 | RF_SYNC_BYTE8_VALUE,
	DEF_PACKETCONFIG1 | RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF,
	DEF_PAYLOADLENGTH | 255,
	DEF_NODEADRS | RF_NODEADDRESS_VALUE,
	DEF_BROADCASTADRS | RF_BROADCASTADDRESS_VALUE,
	DEF_AUTOMODES | RF_AUTOMODES_ENTER_OFF | RF_AUTOMODES_EXIT_OFF | RF_AUTOMODES_INTERMEDIATE_SLEEP,
	DEF_FIFOTHRESH | RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE,
	DEF_PACKETCONFIG2 | RF_PACKET2_RXRESTARTDELAY_1BIT | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF,
	DEF_AESKEY1 | RF_AESKEY1_VALUE,
	DEF_AESKEY2 | RF_AESKEY2_VALUE,
	DEF_AESKEY3 | RF_AESKEY3_VALUE,
	DEF_AESKEY4 | RF_AESKEY4_VALUE,
	DEF_AESKEY5 | RF_AESKEY5_VALUE,
	DEF_AESKEY6 | RF_AESKEY6_VALUE,
	DEF_AESKEY7 | RF_AESKEY7_VALUE,
	DEF_AESKEY8 | RF_AESKEY8_VALUE,
	DEF_AESKEY9 | RF_AESKEY9_VALUE,
	DEF_AESKEY10 | RF_AESKEY10_VALUE,
	DEF_AESKEY11 | RF_AESKEY11_VALUE,
	DEF_AESKEY12 | RF_AESKEY12_VALUE,
	DEF_AESKEY13 | RF_AESKEY13_VALUE,
	DEF_AESKEY14 | RF_AESKEY14_VALUE,
	DEF_AESKEY15 | RF_AESKEY15_VALUE,
	DEF_AESKEY16 | RF_AESKEY16_VALUE,

	DEF_TEMP1 | RF_TEMP1_ADCLOWPOWER_ON,
	DEF_TEMP2
};
/*******************************************************************
** InitRFChip : This routine initializes the RFChip registers     **
**              Using Pre Initialized variables                   **
********************************************************************/
void InitRFChip()
	{
	uint16_t i;
	SetRFMode(RF_STANDBY);
	WriteRegisterister(REG_OSC1, ReadRegister(REG_OSC1) | RF_OSC1_RCCAL_START);
	while ((ReadRegister(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
	WriteRegisterister(REG_OSC1, ReadRegister(REG_OSC1) | RF_OSC1_RCCAL_START);
	while ((ReadRegister(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);

	for (i = 1; i <= REG_TEMP2; i++)
	{
		WriteRegisterister(i, RegistersCfg[i]);
	}
	SyncSize = ((RegistersCfg[REG_SYNCCONFIG] >> 3) & 0x07) + 1;
	for (i = 0; i < SyncSize; i++)
	{
		SyncValue[i] = RegistersCfg[REG_SYNCVALUE1 + i];
	}
	SetRFMode(RF_SLEEP);
}
/*******************************************************************
** SetRFMode : Sets the SX1231 operating mode                     **
********************************************************************
** In  : mode                                                     **
** Out : -                                                        **
*******************************************************************/
void SetRFMode(uint8_t mode)
{
	if (mode != PreMode)
	{
		if (mode == RF_TRANSMITTER)
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_TRANSMITTER);
			while (((ReadRegister(REG_IRQFLAGS1)) & RF_IRQFLAGS1_MODEREADY) == 0x00);             // Wait for TRANSMITTER ModeReady
			PreMode = RF_TRANSMITTER;
		}
		else if (mode == RF_RECEIVER)
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_RECEIVER);
			while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);               // Wait for RECEIVER ModeReady
			PreMode = RF_RECEIVER;
		}
		else if (mode == RF_SYNTHESIZER)
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SYNTHESIZER);
			while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);               // Wait for SYNTHESIZER ModeReady
			PreMode = RF_SYNTHESIZER;
		}
		else if (mode == RF_STANDBY)
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_STANDBY);
			while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);               // Wait for STANDBY ModeReady
			PreMode = RF_STANDBY;
		}
		else
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SLEEP);
			while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);              // Wait for SLEEP ModeReady
			PreMode = RF_SLEEP;
		}
	}
}
/*******************************************************************
** SendByte : Sends a data to the transceiver through the SPI     **
**            interface                                           **
********************************************************************/
void SendByte(uint8_t b)
{
	WriteRegisterister(REG_FIFO, b); // SPI burst mode not used in this implementation
}
/*******************************************************************
** SendRfFrame : Sends a RF frame                                 **
********************************************************************
** In  : *buffer, size                                            **
** Out : *pReturnCode                                             **
*******************************************************************/
void SendRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode)
{
	if ((size + 1) > RF_BUFFER_SIZE_MAX)
	{
		RFState |= RF_STOP;
		*pReturnCode = ERROR;
		return;
	}

	RFState |= RF_BUSY;
	RFState &= ~RF_STOP;
	RFFrameSize = size;
	pRFFrame = buffer;

	WriteRegisterister(REG_DIOMAPPING1, (RegistersCfg[REG_DIOMAPPING1] & 0x3F) | RF_DIOMAPPING1_DIO0_00);           // DIO0 is "Packet Sent"
	WriteRegisterister(REG_FIFOTHRESH, (RegistersCfg[REG_FIFOTHRESH] & 0x7F) | RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY);
	SetRFMode(RF_SLEEP);

	txFlag=1;
	SendByte(RFFrameSize);
	for (ByteCounter = 0, RFFramePos = 0; ByteCounter < RFFrameSize;)
	{
		SendByte(pRFFrame[RFFramePos++]);
		ByteCounter++;
	}
	SetRFMode(RF_TRANSMITTER);                                       //   => Tx starts since FIFO is not empty
	TIMER_Delay(TIMER1,100000);
//	txFlag = 0;
	do {
	} while ((ReadRegister(REG_PALEVEL) & RF_DIOMAPPING1_DIO0_00)); // Wait for Packet sent

	SetRFMode(RF_SLEEP);
	RFState |= RF_STOP;
	RFState &= ~RF_TX_DONE;
	*pReturnCode = OK;
}
/*******************************************************************
** receiveInit :  Initialize receiver                            **
********************************************************************/
void receiveInit()
{
	uint8_t TempRFState;
	TempRFState = RFState;
	if (TempRFState & RF_STOP)
	{
		RFFramePos = 0;
		RFFrameSize = 2;
		WriteRegisterister(REG_DIOMAPPING1, (RegistersCfg[REG_DIOMAPPING1] & 0x3F) | RF_DIOMAPPING1_DIO0_01); // DIO0 is "PAYLOADREADY"
		WriteRegisterister(REG_SYNCCONFIG, (RegistersCfg[REG_SYNCCONFIG] & 0xBF) | RF_SYNC_FIFOFILL_AUTO);
//		Enable_BIT2_RasingEdge_Trig;
		GPIO_EnableInt(PD, 3, GPIO_INT_BOTH_EDGE);
		SetRFMode(RF_RECEIVER);
		RFState |= RF_BUSY;
		RFState &= ~RF_STOP;
		RFState &= ~RF_TIMEOUT;
		return;
	}
}
/*******************************************************************
** ReceiveByte : Receives a data from the transceiver through the **
**               SPI interface                                    **
********************************************************************/
uint8_t ReceiveByte()
{
	return ReadRegister(REG_FIFO); //SPI burst mode not used in this implementation
}
/*******************************************************************
** ReceiveRfFrame : Receives a RF frame                           **
********************************************************************
** In  : -                                                        **
** Out : *buffer, size, *pReturnCode                              **
*******************************************************************/
void ReceiveRfFrame(uint8_t *buffer, uint8_t *size, uint8_t *pReturnCode)
{
	uint8_t TempRFState;
	*pReturnCode = RX_RUNNING;
	TempRFState = RFState;
  pRFFrame = buffer;

	if (TempRFState & RF_RX_DONE)
	{
		SetRFMode(RF_SLEEP);
		RFFrameSize = ReceiveByte();
		for (ByteCounter = 0, RFFramePos = 0; ByteCounter < RFFrameSize;)
		{
			pRFFrame[RFFramePos++] = ReceiveByte();
			ByteCounter++;
		}

		*size = RFFrameSize;
		*pReturnCode = OK;
		RFState |= RF_STOP;
		RFState &= ~RF_RX_DONE;
		return;
	}
	else if (TempRFState & RF_ERROR)
	{
		SetRFMode(RF_SLEEP);
		RFState |= RF_STOP;
		RFState &= ~RF_ERROR;
		*pReturnCode = ERROR;
		return;
	}
	else if (TempRFState & RF_TIMEOUT)
	{
		SetRFMode(RF_SLEEP);
		RFState |= RF_STOP;
		RFState &= ~RF_TIMEOUT;
		*pReturnCode = RX_TIMEOUT;
		return;
	}
}
/*******************************************************************
** ReadRssi : Reads the Rssi value from the RFM69                **
********************************************************************/
uint16_t ReadRssi()
{
	uint16_t rssivalue;
	WriteRegisterister(REG_RSSICONFIG, RegistersCfg[REG_RSSICONFIG] | RF_RSSI_START); // Triggers RSSI measurement
	while ((ReadRegister(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00);                   // Waits for RSSI measurement to be completed
	rssivalue = ReadRegister(REG_RSSIVALUE);                                         // Reads the RSSI result
	return rssivalue;
}
/*******************************************************************
** ReadTemparature : Reads the Temparature value from the RFM69                **
********************************************************************/
uint8_t ReadTemparature()
{
	uint8_t temp;
	WriteRegisterister(REG_TEMP1, 0x08);
	temp=ReadRegister(REG_TEMP2);
  return temp;
}

