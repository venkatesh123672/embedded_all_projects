
#include "rfm69_registor.h"
#include <stdio.h>
#include "M0564.h"
/*******************************************************************
** Global variables                                               **
*******************************************************************/
uint8_t RFState = RF_STOP;                // RF state machine
static   uint8_t *pRFFrame;               // Pointer to the RF frame
static   uint8_t  RFFramePos;             // RF payload current position
uint8_t  RFFrameSize;                   // RF payload size (length byte, not a C string)
static   uint16_t ByteCounter = 0;        // RF payload byte counter
static   uint8_t  PreMode = RF_STANDBY;   // Previous chip operating mode
static   uint8_t  SyncSize = 8;           // Size of sync word
static   uint8_t  SyncValue[8];           // Value of sync word

extern uint8_t  txFlag;
extern uint8_t  recFlag;
uint8_t value23=0;
extern uint8_t sending_brust;
uint8_t rssiRaw=0;

//uint16_t ReadRegister(uint8_t address)
//{
//	 address = address & 0x7F;
//	 SPI_SET_SS_LOW(SPI0);
//	 SPI_WRITE_TX(SPI0, address);
//	 while (SPI_IS_BUSY(SPI0));
//	 SPI_WRITE_TX(SPI0, 0x00);
//	 value23=SPI_READ_RX(SPI0);
//	 while (SPI_IS_BUSY(SPI0));
//	 SPI_SET_SS_HIGH(SPI0);
//	return value23;
//}
uint16_t ReadRegister(uint8_t address)
{
//    uint16_t value;

	address = address & 0x7F;

    SPI_SET_SS_LOW(SPI0);

    /* Send register address */
    SPI_WRITE_TX(SPI0, address);
    while(SPI_IS_BUSY(SPI0));

    /* Read and discard RX from address transfer */
    SPI_READ_RX(SPI0);

    /* Send dummy byte to generate clock for register data */
    SPI_WRITE_TX(SPI0, 0x00);
    while(SPI_IS_BUSY(SPI0));

    /* Now read the actual register value */
    value23 = SPI_READ_RX(SPI0);

    SPI_SET_SS_HIGH(SPI0);

    return value23;
}
//void WriteRegisterister(uint8_t address, uint16_t value)
//{
//	  address = address | 0x80;
//	  SPI_SET_SS_LOW(SPI0);
//	  SPI_WRITE_TX(SPI0, address);
//	  while (SPI_IS_BUSY(SPI0));
//	  SPI_WRITE_TX(SPI0, value);
//	  while (SPI_IS_BUSY(SPI0));
//	  SPI_SET_SS_HIGH(SPI0);
//
//}
void WriteRegisterister(uint8_t address, uint8_t value)
{
	address = address | 0x80;

    SPI_SET_SS_LOW(SPI0);

    SPI_WRITE_TX(SPI0, address);
    while(SPI_IS_BUSY(SPI0));
    SPI_READ_RX(SPI0);       // discard RX from address transfer

    SPI_WRITE_TX(SPI0, value);
    while(SPI_IS_BUSY(SPI0));
    SPI_READ_RX(SPI0);       // discard RX from data transfer

    SPI_SET_SS_HIGH(SPI0);
}
uint8_t Read4thbyte(uint8_t address)
{
	uint8_t  rssi = 0;

	rssi= ReadRegister(address);
	rssi= ReadRegister(0x00);
	rssi= ReadRegister(0x00);
	rssi= ReadRegister(0x00);
	rssi= ReadRegister(0x00);
	return rssi;
}


uint8_t RegistersCfg[] = {          // RFM69  configuration registers values
	DEF_FIFO,                               // Left for convenience, not to be changed
	DEF_OPMODE | RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY,
	DEF_DATAMODUL | RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00,
	DEF_BITRATEMSB | RF_BITRATEMSB_9600,
	DEF_BITRATELSB | RF_BITRATELSB_9600,
	DEF_FDEVMSB | RF_FDEVMSB_10000,
	DEF_FDEVLSB | RF_FDEVLSB_10000,
	DEF_FRFMSB | RF_FRFMSB_867,
	DEF_FRFMID | RF_FRFMID_867,
	DEF_FRFLSB | RF_FRFLSB_867,
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
	DEF_LNA | RF_LNA_ZIN_50 | RF_LNA_LOWPOWER_OFF | RF_LNA_GAINSELECT_AUTO,
	DEF_RXBW | RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3,  /* 0x4B = 50 kHz */
	DEF_AFCBW | RF_AFCBW_DCCFREQAFC_010 | RF_AFCBW_MANTAFC_16 | RF_AFCBW_EXPAFC_3,  /* 0x43 = 62.5 kHz */
	DEF_OOKPEAK | RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000,
	DEF_OOKAVG | RF_OOKAVG_AVERAGETHRESHFILT_10,
	DEF_OOKFIX | RF_OOKFIX_FIXEDTHRESH_VALUE,
	DEF_AFCFEI | RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_ON,
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
	DEF_RSSITHRESH | 0xDC,	                  // -110 dBm, same as set_required_baud_rate_19200
	DEF_RXTIMEOUT1 | RF_RXTIMEOUT1_RXSTART_VALUE,
	DEF_RXTIMEOUT2 | RF_RXTIMEOUT2_RSSITHRESH_VALUE,

	DEF_PREAMBLEMSB | RF_PREAMBLESIZE_MSB_VALUE,
	DEF_PREAMBLELSB | 0x05,
	DEF_SYNCCONFIG | RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0,
	DEF_SYNCVALUE1 | 0x69,
	DEF_SYNCVALUE2 | 0x81,
	DEF_SYNCVALUE3 | RF_SYNC_BYTE3_VALUE,
	DEF_SYNCVALUE4 | RF_SYNC_BYTE4_VALUE,
	DEF_SYNCVALUE5 | RF_SYNC_BYTE5_VALUE,
	DEF_SYNCVALUE6 | RF_SYNC_BYTE6_VALUE,
	DEF_SYNCVALUE7 | RF_SYNC_BYTE7_VALUE,
	DEF_SYNCVALUE8 | RF_SYNC_BYTE8_VALUE,
	DEF_PACKETCONFIG1 | RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF,
	DEF_PAYLOADLENGTH | 65,
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
** InitRFChipRx : Standby config, then enter RX.                  **
** reset_rfm69() stays in main (RESET high 200 us, then 5 ms).    **
********************************************************************/
void InitRFChipRx(void)
{
	uint16_t i;
	uint8_t pc2;

	/* Standby, sequencer ON. PreMode starts as STANDBY, so write 0x04 here. */
	WriteRegisterister(REG_OPMODE, (uint8_t)(RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_STANDBY));
	while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);
	PreMode = RF_STANDBY;

	WriteRegisterister(REG_OSC1, (uint8_t)(ReadRegister(REG_OSC1) | RF_OSC1_RCCAL_START));
	while ((ReadRegister(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);

	/* Air settings (freq, bitrate, fdev, RxBw, AfcBw, sync, packet, DIO, RSSI, preamble) are already in RegistersCfg. */
	for (i = 1; i <= REG_TEMP2; i++)
	{
		WriteRegisterister((uint8_t)i, RegistersCfg[i]);
	}
	SyncSize = (uint8_t)(((RegistersCfg[REG_SYNCCONFIG] >> 3) & 0x07) + 1);
	for (i = 0; i < SyncSize; i++)
	{
		SyncValue[i] = RegistersCfg[REG_SYNCVALUE1 + i];
	}

	/* 50 ohm RF port, AGC auto. Bit 7 set would be 200 ohm. */
	WriteRegisterister(REG_LNA, (uint8_t)(RF_LNA_ZIN_50 | RF_LNA_LOWPOWER_OFF | RF_LNA_GAINSELECT_AUTO));
	WriteRegisterister(REG_TESTLNA, 0x2D);
	/* RegTestDagc: continuous DAGC improves RX. */
	WriteRegisterister(REG_TESTDAGC, 0x30);

	WriteRegisterister(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
	SetRFMode(RF_RECEIVER);

	pc2 = (uint8_t)ReadRegister(REG_PACKETCONFIG2);
	pc2 = (uint8_t)(pc2 & (uint8_t)~RF_PACKET2_RXRESTART);
	WriteRegisterister(REG_PACKETCONFIG2, pc2);
	WriteRegisterister(REG_PACKETCONFIG2, (uint8_t)(pc2 | RF_PACKET2_RXRESTART));

	RFState |= RF_BUSY;
	RFState &= ~RF_STOP;
}
 /*******************************************************************
 ** Long Range + Good Battery Life Configuration
 ** RFM69HCW 868 MHz
 ** TX Power : +14 dBm
 ** Bitrate  : 4.8 kbps
 ********************************************************************/

 void InitRFChip_LongRange(void)
 {
     uint16_t i;

     // Enter standby mode
     SetRFMode(RF_STANDBY);

     // RC calibration
     WriteRegisterister(REG_OSC1, ReadRegister(REG_OSC1) | RF_OSC1_RCCAL_START);
     while ((ReadRegister(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);

     WriteRegisterister(REG_OSC1, ReadRegister(REG_OSC1) | RF_OSC1_RCCAL_START);
     while ((ReadRegister(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);

     /********************************************************
      * Load default register table
      ********************************************************/
     for (i = 1; i <= REG_TEMP2; i++)
     {
         WriteRegisterister(i, RegistersCfg[i]);
     }

     /********************************************************
      * Recommended changes for stable long range operation
      ********************************************************/

     //-------------------------------------------------------
     // TX POWER = +14 dBm
     // REG_PALEVEL = 0x7C
     //
     // PA1 = ON
     // PA2 = ON
     // OutputPower = 28
     //
     // Pout = -14 + 28 = +14 dBm
     //-------------------------------------------------------
     WriteRegisterister(REG_PALEVEL, 0x7C);

     //-------------------------------------------------------
     // OCP = 120 mA
     //-------------------------------------------------------
     WriteRegisterister(REG_OCP,
         RF_OCP_ON |
         RF_OCP_TRIM_120);

     //-------------------------------------------------------
     // RX Bandwidth
     // Wider BW improves compatibility between modules
     //-------------------------------------------------------
     WriteRegisterister(REG_RXBW,
         RF_RXBW_DCCFREQ_010 |
         RF_RXBW_MANT_20 |
         RF_RXBW_EXP_2);

     //-------------------------------------------------------
     // AFC Bandwidth
     //-------------------------------------------------------
     WriteRegisterister(REG_AFCBW,
         RF_AFCBW_DCCFREQAFC_010 |
         RF_AFCBW_MANTAFC_16 |
         RF_AFCBW_EXPAFC_2);

     //-------------------------------------------------------
     // RSSI Threshold
     // -120 dBm
     //-------------------------------------------------------
     WriteRegisterister(REG_RSSITHRESH, 240);

     //-------------------------------------------------------
     // Optional:
     // Increase preamble for better synchronization
     //-------------------------------------------------------
     WriteRegisterister(REG_PREAMBLEMSB, 0x00);
     WriteRegisterister(REG_PREAMBLELSB, 0x04);

     //-------------------------------------------------------
     // Sync size
     //-------------------------------------------------------
     SyncSize = ((RegistersCfg[REG_SYNCCONFIG] >> 3) & 0x07) + 1;

     for (i = 0; i < SyncSize; i++)
     {
         SyncValue[i] = RegistersCfg[REG_SYNCVALUE1 + i];
     }

     //-------------------------------------------------------
     // Enter sleep mode
     //-------------------------------------------------------
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
	/* Bit7 cleared: sequencer ON. 0x63 drops a stale SequencerOff from the table. */
	if (mode != PreMode)
	{
		if (mode == RF_TRANSMITTER)
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0x63) | RF_TRANSMITTER);
			while (((ReadRegister(REG_IRQFLAGS1)) & RF_IRQFLAGS1_MODEREADY) == 0x00);             // Wait for TRANSMITTER ModeReady
			PreMode = RF_TRANSMITTER;
		}
		else if (mode == RF_RECEIVER)
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0x63) | RF_RECEIVER);
			while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);               // Wait for RECEIVER ModeReady
			PreMode = RF_RECEIVER;
		}
		else if (mode == RF_SYNTHESIZER)
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0x63) | RF_SYNTHESIZER);
			while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);               // Wait for SYNTHESIZER ModeReady
			PreMode = RF_SYNTHESIZER;
		}
		else if (mode == RF_STANDBY)
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0x63) | RF_STANDBY);
			while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);               // Wait for STANDBY ModeReady
			PreMode = RF_STANDBY;
		}
		else
		{
			WriteRegisterister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0x63) | RF_SLEEP);
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
	SetRFMode(RF_TRANSMITTER);                                 //   => Tx starts since FIFO is not empty
	while (!(ReadRegister(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT));//   => Tx starts since FIFO is not empty
//	while (!(ReadRegister(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT));//   => Tx starts since FIFO is not empty

	//    if(sending_brust)
//	   TIMER_Delay(TIMER0,500);
//	else
//	   TIMER_Delay(TIMER0,500);
//
//	do {
//	} while ((ReadRegister(REG_PALEVEL) & RF_DIOMAPPING1_DIO0_00)); // Wait for Packet sent

	SetRFMode(RF_SLEEP);
	txFlag = 0;
	recFlag = 0;
	if (GPIO_GET_INT_FLAG(PD, BIT3))
		GPIO_CLR_INT_FLAG(PD, BIT3);
	RFState |= RF_STOP;
	RFState &= ~RF_TX_DONE;
	RFState &= ~RF_RX_DONE;
	*pReturnCode = OK;
}
/*******************************************************************
** receiveInit :  Initialize receiver                            **
********************************************************************/
void receiveInit()
{
	uint8_t TempRFState=0;
	uint8_t i2;
	uint8_t pc2;

	TempRFState = RFState;
	if (TempRFState & RF_STOP)
	{
		/* A waiting packet must be read, not flushed. RestartRx after every
		   packet (or with PayloadReady set) drops the next frame. */
		i2 = (uint8_t)ReadRegister(REG_IRQFLAGS2);
		if (i2 & RF_IRQFLAGS2_PAYLOADREADY)
		{
			RFState |= RF_RX_DONE;
			RFState &= ~RF_BUSY;
			RFState &= ~RF_STOP;
			recFlag = 1;
			return;
		}
		RFFramePos = 0;
		RFFrameSize = 0;
		recFlag = 0;
		RFState &= ~RF_RX_DONE;
		if (GPIO_GET_INT_FLAG(PD, BIT3))
			GPIO_CLR_INT_FLAG(PD, BIT3);
		/* DIO0 = 00: RX PayloadReady, TX PacketSent. 01 is CrcOk, not PayloadReady. */
		WriteRegisterister(REG_DIOMAPPING1, (RegistersCfg[REG_DIOMAPPING1] & 0x3F) | RF_DIOMAPPING1_DIO0_00);
		WriteRegisterister(REG_SYNCCONFIG, (RegistersCfg[REG_SYNCCONFIG] & 0xBF) | RF_SYNC_FIFOFILL_AUTO);
		/* FifoOverrun is write-1-to-clear. 0x10 does not clear PayloadReady. */
		WriteRegisterister(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
		pc2 = (uint8_t)ReadRegister(REG_PACKETCONFIG2);
		pc2 = (uint8_t)(pc2 | RF_PACKET2_AUTORXRESTART_ON);
		pc2 = (uint8_t)(pc2 & (uint8_t)~RF_PACKET2_RXRESTART);
		WriteRegisterister(REG_PACKETCONFIG2, pc2);
		GPIO_EnableInt(PD, 3, GPIO_INT_RISING);
		SetRFMode(RF_RECEIVER);
		/* Packet that finished during RX entry: read it, do not RestartRx. */
		i2 = (uint8_t)ReadRegister(REG_IRQFLAGS2);
		if (i2 & RF_IRQFLAGS2_PAYLOADREADY)
		{
			RFState |= RF_RX_DONE;
			RFState &= ~RF_BUSY;
			RFState &= ~RF_STOP;
			RFState &= ~RF_TIMEOUT;
			recFlag = 1;
			return;
		}
		/* RestartRx ONLY if FifoOverrun && !PayloadReady. Never after every packet. */
		if ((i2 & RF_IRQFLAGS2_FIFOOVERRUN) && ((i2 & RF_IRQFLAGS2_PAYLOADREADY) == 0))
		{
			WriteRegisterister(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
			WriteRegisterister(REG_PACKETCONFIG2, (uint8_t)(pc2 | RF_PACKET2_RXRESTART));
		}
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
void ReceiveRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode)
{
	uint8_t i2;
	uint16_t drain;

	*pReturnCode = RX_RUNNING;
	pRFFrame = buffer;
	i2 = (uint8_t)ReadRegister(REG_IRQFLAGS2);

	/* IRQ2 bit2 survives a missed DIO0 edge. Do not require RF_RX_DONE alone. */
	if ((RFState & RF_RX_DONE) || (i2 & RF_IRQFLAGS2_PAYLOADREADY))
	{
		/* Standby keeps the FIFO. Sleep would clear it before the read. */
		SetRFMode(RF_STANDBY);
		RFFrameSize = ReceiveByte();
		/* 65-byte payloads are valid. Reject 0 or more than the caller buffer. */
		if ((RFFrameSize == 0) || (RFFrameSize > size))
		{
			drain = 0;
			while ((ReadRegister(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) && (drain < RF_BUFFER_SIZE_MAX))
			{
				(void)ReceiveByte();
				drain++;
			}
			SetRFMode(RF_RECEIVER);
			*pReturnCode = ERROR;
			RFState |= RF_STOP;
			RFState &= ~RF_RX_DONE;
			return;
		}
		for (ByteCounter = 0, RFFramePos = 0; ByteCounter < RFFrameSize; )
		{
			pRFFrame[RFFramePos++] = ReceiveByte();
			ByteCounter++;
		}
		/* FIFO is empty. Sleep would flush and leave the radio deaf. RX now. */
		SetRFMode(RF_RECEIVER);
		*pReturnCode = OK;
		RFState |= RF_BUSY;
		RFState &= ~RF_STOP;
		RFState &= ~RF_RX_DONE;
		return;
	}
	else if (RFState & RF_ERROR)
	{
		SetRFMode(RF_RECEIVER);
		RFState |= RF_STOP;
		RFState &= ~RF_ERROR;
		*pReturnCode = ERROR;
		return;
	}
	else if (RFState & RF_TIMEOUT)
	{
		SetRFMode(RF_RECEIVER);
		RFState |= RF_STOP;
		RFState &= ~RF_TIMEOUT;
		*pReturnCode = RX_TIMEOUT;
		return;
	}
	else
	{
		/* No FIFO to save. Stay in RX — Sleep would leave the radio deaf. */
		SetRFMode(RF_RECEIVER);
		RFState |= RF_BUSY;
		RFState &= ~RF_STOP;
		RFState &= ~RF_RX_DONE;
		*pReturnCode = ERROR;
		return;
	}
}
///*******************************************************************
//** ReadRssi : Reads the Rssi value from the RFM69                **
//********************************************************************/
//uint16_t ReadRssi()
//{
//	uint16_t rssivalue;
//	WriteRegisterister(REG_RSSICONFIG, (ReadRegister(REG_RSSICONFIG) | RF_RSSI_START)); // Triggers RSSI measurement
//	while ((ReadRegister(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00);                   // Waits for RSSI measurement to be completed
//	rssivalue = ReadRegister(REG_RSSIVALUE);                                         // Reads the RSSI result
//	return rssivalue;
//}
uint8_t ReadRssi()
{
    uint16_t timeout = 1;
//    uint8_t rssiRaw;

//    WriteRegisterister(REG_RSSICONFIG,
//        ReadRegister(REG_RSSICONFIG) | RF_RSSI_START);
//
    while (((ReadRegister(REG_RSSICONFIG) & RF_RSSI_DONE) == 0) && timeout--);

//    if (timeout == 0)
//        return 0;  // error / invalid

    rssiRaw = ReadRegister(REG_RSSIVALUE);
    rssiRaw = ReadRegister(REG_RSSIVALUE);
    rssiRaw = ReadRegister(REG_RSSIVALUE);
    rssiRaw = ReadRegister(REG_RSSIVALUE);
    rssiRaw = ReadRegister(REG_RSSIVALUE);

    return rssiRaw;
//    return -((uint16_t)rssiRaw / 2);
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
/*******************************************************************
** set_higest_baud rate : we can set required baud rate, changing below regisor value **
********************************************************************/
void set_required_baud_rate()
{
  WriteRegisterister(0x03, 0x00);  //REG_BITRATEMSB: 300kbps (0x006B, see DS p20)
  WriteRegisterister(0x04, 0x6B);  //REG_BITRATELSB: 300kbps (0x006B, see DS p20)
  WriteRegisterister(0x19, 0x40);  //REG_RXBW: 500kHz
  WriteRegisterister(0x1A, 0x80);  //REG_AFCBW: 500kHz
  WriteRegisterister(0x05, 0x13);  //REG_FDEVMSB: 300khz (0x1333)
  WriteRegisterister(0x06, 0x33);  //REG_FDEVLSB: 300khz (0x1333)
  WriteRegisterister(0x29, 180);   //set REG_RSSITHRESH to -120dBm
  WriteRegisterister(0x37, 0x90);  //DC=WHITENING, CRCAUTOOFF=0,DC: 00=none, 01=manchester, 10=whitening
  WriteRegisterister(0x58 ,0x2D);  //Set device to High Sencitivity
}


void set_required_baud_rate_9600()
{
    // =========================
    // Bitrate = 9600bps
    // =========================
    WriteRegisterister(0x03, 0x0D);   // REG_BITRATEMSB
    WriteRegisterister(0x04, 0x05);   // REG_BITRATELSB

    // =========================
    // Frequency deviation = 5kHz
    // =========================
    WriteRegisterister(0x05, 0x00);
    WriteRegisterister(0x06, 0xA4);

    // =========================
    // RX Bandwidth
    // =========================
    WriteRegisterister(0x19, 0x4A);   // RXBW ˜ 50kHz
    WriteRegisterister(0x1A, 0x42);   // AFCBW ˜ 83kHz

    // =========================
    // Preamble length
    // =========================
    WriteRegisterister(0x2C, 0x00);
    WriteRegisterister(0x2D, 0x05);   // 5 bytes preamble

//    // =========================
//    // Sync config
//    // =========================
//    WriteRegisterister(0x2E, 0x88);   // Sync ON, FIFO fill
//    WriteRegisterister(0x2F, 0x2D);   // Sync byte 1
//    WriteRegisterister(0x30, 0xD4);   // Sync byte 2

    // =========================
    // Packet config
    // =========================
    WriteRegisterister(0x37, 0x90);   // Whitening ON

    // =========================
    // RSSI Threshold
    // =========================
    WriteRegisterister(0x29, 0xDC);   // -110dBm

    // =========================
    // AFC FEI
    // =========================
    WriteRegisterister(0x1E, 0x0C);   // AFC auto

    // =========================
    // Sensitivity boost
    // =========================
    WriteRegisterister(0x58, 0x2D);
}
void set_required_baud_rate_19200()
{
    // =========================
    // Bitrate = 19200bps
    // =========================
    WriteRegisterister(0x03, 0x06);   // REG_BITRATEMSB
    WriteRegisterister(0x04, 0x83);   // REG_BITRATELSB

    // =========================
    // Frequency deviation matches 0x0147
    // =========================
    WriteRegisterister(0x05, 0x01);
    WriteRegisterister(0x06, 0x47);

    // =========================
    // RX Bandwidth: 0x4A = 100 kHz, 0x42 = 125 kHz (AfcBw >= RxBw)
    // =========================
    WriteRegisterister(0x19, 0x4A);   // RXBW 100 kHz (Mant 20, Exp 2)
    WriteRegisterister(0x1A, 0x42);   // AFCBW 125 kHz, wider than RxBw

    // =========================
    // Preamble length
    // =========================
    WriteRegisterister(0x2C, 0x00);
    WriteRegisterister(0x2D, 0x05);   // 5 bytes preamble

//    // =========================
//    // Sync config
//    // =========================
//    WriteRegisterister(0x2E, 0x88);   // Sync ON, FIFO fill
//    WriteRegisterister(0x2F, 0x2D);   // Sync byte 1
//    WriteRegisterister(0x30, 0xD4);   // Sync byte 2

    // =========================
    // Packet config
    // =========================
    WriteRegisterister(0x37, 0x90);   // variable length, CRC on, DC-free off

    // =========================
    // RSSI Threshold
    // =========================
    WriteRegisterister(0x29, 0xDC);   // -110dBm

    // =========================
    // AFC FEI
    // =========================
    WriteRegisterister(0x1E, 0x0C);   // AFC auto

    // =========================
    // Sensitivity boost
    // =========================
    WriteRegisterister(0x58, 0x2D);
}
/*******************************************************************
** set Listen Mode : In power saving mode, must call Listen mode
                     set Rx time and Idel time                    **
********************************************************************/
void set_Listen_mode()
{
	SetRFMode(RF_STANDBY);
	WriteRegisterister(REG_OPMODE,  ( RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_ON | RF_OPMODE_STANDBY));
	WriteRegisterister(REG_LISTEN1,  (RF_LISTEN1_RESOL_4100 | RF_LISTEN1_CRITERIA_RSSI | RF_LISTEN1_END_00));
	WriteRegisterister(REG_LISTEN2,0xff);  //Idel time
	WriteRegisterister(REG_LISTEN3,0x01);	 //RX time

	WriteRegisterister(REG_RXTIMEOUT1, 0x01); //TimeoutRxStart
	WriteRegisterister(REG_RXTIMEOUT2, 0x01); //TimeoutRssiThresh
}
/*******************************************************************z
** clr Listen Mode : come out Listen mode must call
                     other wise RFM69 do not transmit             **
********************************************************************/
void clr_Listen_mode()
{
	SetRFMode(RF_STANDBY);
	WriteRegisterister(REG_OPMODE,  ( RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF |RF_OPMODE_LISTENABORT | RF_OPMODE_STANDBY));
	WriteRegisterister(REG_OPMODE,  ( RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF  | RF_OPMODE_STANDBY));
	WriteRegisterister(REG_RXTIMEOUT1, 0);   //Clr TimeoutRxStart
	WriteRegisterister(REG_RXTIMEOUT2, 0);   //Clr TimeoutRssiThresh
}
void set_required_baud_rate_4800()
{
    // =====================================
    // Bitrate = 4800 bps
    // =====================================
    WriteRegisterister(0x03, 0x1A);   // REG_BITRATEMSB
    WriteRegisterister(0x04, 0x0B);   // REG_BITRATELSB

    // =====================================
    // Frequency deviation = 5kHz
    // =====================================
    WriteRegisterister(0x05, 0x00);   // REG_FDEVMSB
    WriteRegisterister(0x06, 0xA4);   // REG_FDEVLSB

    // =====================================
    // RX bandwidth
    // =====================================
    WriteRegisterister(0x19, 0x4B);   // RXBW ˜ 25kHz
    WriteRegisterister(0x1A, 0x43);   // AFCBW ˜ 50kHz

    // =====================================
    // RSSI threshold
    // =====================================
    WriteRegisterister(0x29, 0xDC);   // around -110dBm

    // =====================================
    // Preamble length
    // =====================================
    WriteRegisterister(0x2C, 0x00);
    WriteRegisterister(0x2D, 0x08);   // 8-byte preamble

//    // =====================================
//    // Sync configuration
//    // =====================================
//    WriteRegisterister(0x2E, 0x88);   // Sync ON, FIFO fill condition
//    WriteRegisterister(0x2F, 0x2D);   // Sync byte 1
//    WriteRegisterister(0x30, 0xD4);   // Sync byte 2

    // =====================================
    // Packet config
    // =====================================
    WriteRegisterister(0x37, 0x90);   // Whitening ON

    // =====================================
    // AFC auto
    // =====================================
    WriteRegisterister(0x1E, 0x0C);

    // =====================================
    // High sensitivity mode
    // =====================================
    WriteRegisterister(0x58, 0x2D);
}
