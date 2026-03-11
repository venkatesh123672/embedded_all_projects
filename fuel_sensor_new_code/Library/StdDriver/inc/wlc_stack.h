#ifndef STDDRIVER_INC_WLC_STACK_H_
#define STDDRIVER_INC_WLC_STACK_H_
#define SIZEOFVALUE      10
#pragma pack(1)
typedef struct
{
	uint8_t SOF;
	uint8_t Source;
	uint8_t Destination;
	uint16_t sourceId;
	uint16_t DestId;
	uint8_t  Data;
	uint8_t Value[SIZEOFVALUE];
    uint16_t crc;
} WLC_t;
WLC_t   WLC;


/*******************************************************************
** Destination and Destination ID                **
********************************************************************/
#define  CONTROL_PANEL			        	    30
#define  CONTROL_PANEL_ID			        	143
#define  SOURCEID                                555
/*******************************************************************
** Destination Device Address              **
********************************************************************/
enum
{
  FUEL_SENSOR=101,
}SOURECE;
/*******************************************************************
**          Destination Device ID                **
********************************************************************/
enum
{
	FUEL_SENSOR_ID=0X2,
}SOURCE_ID;
/*******************************************************************
**          General Commands	                                    **
********************************************************************/
enum
{
	OPEN_CAP=0x70,
	CLOSE_CAP,
	CAP_STATUS,
	BATTERY_VALUE,
	FLOW_SENSOR,
	FUEL_GUAGE,
	THAFT,
}COMMON_VALUE;

//void receive();
//void send(uint8_t source,uint8_t Dest, uint16_t sourId,uint16_t DestId,uint8_t Data);
//uint16_t CRC(uint8_t buf[], int len);


#endif /* STDDRIVER_INC_WLC_STACK_H_ */

