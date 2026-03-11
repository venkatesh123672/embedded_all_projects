/*
 * ADC.C
 *
 *  Created on: 01-Aug-2024
 *      Author: natar
 */

#include <stdio.h>
#include "NuMicro.h"
#include "ADC.h"

void ADC_init();
void ADC_measure();

uint32_t current_read;
uint32_t vtg_read;
uint32_t intBatt_V;
uint32_t extBatt_V;
uint32_t Res_Vtg_Read;
uint32_t Res_Curr_Read;
float Res_Vtg_FB;
char testStr[16];
uint16_t readV_ReadOutputV();
uint16_t readI_ReadOutputI();

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


/*---------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_IRQHandler(void)
{
	g_u32AdcIntFlag = 1;
	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* clear the A/D conversion flag */
}

void ADC_init()
{
	/* Set the ADC operation mode as single-cycle, input mode as single-end and
          enable the analog input channel 0, 1, 2 and 3 */
	ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, 0xF);
	//	ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, 0xF);
	/* Power on ADC module */
	ADC_POWER_ON(ADC);

}
void ADC_measure()
{
	uint32_t u32ChannelCount;
	int32_t  i32ConversionData;

	/* Clear the A/D interrupt flag for safe */
	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

	/* Start A/D conversion */
	ADC_START_CONV(ADC);

	/* Wait conversion done */
	while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));

	extBatt_V=  ADC_GET_CONVERSION_DATA(ADC, 1);
	intBatt_V=  ADC_GET_CONVERSION_DATA(ADC, 0);
//	Res_Vtg_FB=(4.9/4095)*vtg_FB;
	/*vtg_read= ADC_GET_CONVERSION_DATA(ADC, 2);
	Res_Vtg_Read=(4.9/4095)*vtg_read;                // Resolution
	current_read=ADC_GET_CONVERSION_DATA(ADC, 3);
	Res_Curr_Read=Res_Vtg_Read/0.1;*/
//	readV_ReadOutputV();
//	readI_ReadOutputI();
}

uint16_t readV_ReadOutputV()
{
	vtg_read= ADC_GET_CONVERSION_DATA(ADC, 2);
	Res_Vtg_Read=(vtg_read*2);	                //Res_Vtg_Read=(4.9/4095)*vtg_read; // Resolution float
    return Res_Vtg_Read;
}

uint16_t readI_ReadOutputI()
{
	current_read=ADC_GET_CONVERSION_DATA(ADC, 3);
	Res_Curr_Read=(current_read*2);
	return Res_Curr_Read;
}

