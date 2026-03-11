/*
 * wlc_stack.h
 *
 *  Created on: 08-Apr-2024
 *      Author: PC-2
 */

#ifndef STDDRIVER_INC_WLC_STACK_H_
#define STDDRIVER_INC_WLC_STACK_H_


/*******************************************************************
**Struct Of Radio transmotter and Receiver                                            **
********************************************************************/
//__attribute__((packed));
#pragma pack(1)
typedef struct
{
	uint8_t SOF;
	uint8_t Source;
	uint8_t Destination;
	uint16_t sourceId;
	uint16_t DestId;
	uint8_t  Sequence;
	uint8_t  ReadWrite;
	uint8_t  Misc;
} WLC_t;

//__attribute__((packed));


#pragma pack(1)
typedef struct
{
	uint8_t Length;
	uint8_t Data;
	uint8_t  *Value;
  uint16_t crc;
} WLC_t1;

WLC_t   WLC;
WLC_t1  WLC1;

/*******************************************************************
**Source and Source Id                                            **
********************************************************************/
#define  CONTROL_PANEL			        	90
#define  CONTROL_PANEL_ID			        9001

#define  WRITE                        0xAB
#define  READ                         0xBA
/*******************************************************************
** Source  Address                                                **
********************************************************************/
enum
{
  MOTION_SENSOR=101,
  CURTAIN_MOTION_SENSOR,
  MOTION_CAMERA,
  DOOR_SENSOR,
  GLASS_BREAK_SENSOR,
  SMOKE_SENSOR,
  OUTDOOR_SIREN,
  INDOOR_SIREN,
  KEYPAD,
  PANIC_BUTTON,
  SMART_SWITCH,
  REPEATERS,
  REMOTE_KEY,
  GROUP_1,
  GROUP_2,
}SOURECE;

/*******************************************************************
**Source ID                                                       **
********************************************************************/
enum
{
  MOTION_SENSOR_ID=1001,
  CURTAIN_MOTION_SENSOR_ID,
  MOTION_CAMERA_ID,
  DOOR_SENSOR_ID,
  GLASS_BREAK_SENSOR_ID,
  SMOKE_SENSOR_ID,
  OUTDOOR_SIREN_ID,
  INDOOR_SIREN_ID,
  KEYPAD_ID,
  PANIC_BUTTON_ID,
  SMART_SWITCH_ID,
  REPEATERS_ID,
  REMOTE_KEY_ID,
  GROUP_1_ID,
  GROUP_2_ID,
}SOURCE_ID;

/*******************************************************************
**common commands for all sensors and control panel	              **
********************************************************************/
enum
{
	POLLING_SIGNAL=50,
	BATTERY_LEVEL,
	HARDWARE_VERSION,
	SOFTWARE_VERSION,
	TAMPER_STATUS,
	LED_STATUS,
	WIRED_STATUS,
	SUCCESSFUL_ACKNOWLEDGEMENT,
	UNSUCCESSFUL_ACKNOWLEDGEMENT
}COMMON_COMMAND;

/*******************************************************************
**General Commands	                                              **
********************************************************************/
enum
{
	ON=0x01,
	OFF,
	OPEN,
	CLOSE,
	CONNECTED,
	DISCONNECTED,
	ACK,
	NACK,
	DETECTED,
	UNDETECTED,
}COMMON_VALUE;

// commands only for panic button sensors
enum
{
  PANIC_DETECTION=140,
}PANIC_SWITCH;
// commands only for remote key sensors
enum
{
	ARM=150,
	FORCE_ARM,
	DISARM,
	HOME_ARM,
	FORCE_HOME_ARM,
}SMART_KEY;
// commands only for Door sensors
enum
{
	DOOR_STATUS=120,
	DOOR_OPEN,
	DOOR_CLOSE,
}Door_Sensor;

// commands only for motion sensor sensors
enum
{
	MOTION_DETECTION=130,
}Motion_Sensor;

//void receive();
//void send(uint8_t source,uint8_t Dest, uint16_t sourId,uint16_t DestId,uint16_t Data,uint8_t *value);
//uint16_t CRC(uint8_t buf[], int len);





#endif /* STDDRIVER_INC_WLC_STACK_H_ */
