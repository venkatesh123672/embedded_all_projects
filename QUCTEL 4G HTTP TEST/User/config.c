#include <stdio.h>
#include "M0564.h"
#include <string.h>
#include "ADC.h"
#include "gpio.h"
#include "config.h"

uint8_t command[BUF_LEN_LONG];
uint32_t cmdReadBk[BUF_LEN_LONG/4];
uint16_t x;
uint32_t tempAddr;
uint8_t expDataIn = 0;
uint16_t cmdLen;
uint32_t u32FlashStartAddr = DATA_FLASH_START;
uint32_t u32Data, u32RData;
uint8_t dataWr;

uint8_t *tVarStart;
uint8_t *tVarEnd;
uint8_t SW_variant[CONFIG_NAME_LEN];

extern uint8_t configMode;
extern uint8_t UART0_RX_BUF[BUF_LEN];
extern uint8_t UART0_TX_BUF[BUF_LEN];
extern uint8_t count_0;
extern uint8_t newWord_0;

extern uint8_t UART2_RX_BUF[BUF_LEN_LONG];
extern uint8_t UART2_TX_BUF[BUF_LEN];
extern uint16_t count_2;
extern uint8_t newWord_2;
extern uint8_t webURL[API_BUFF_LEN];
extern uint8_t APN_URL[URL_LENGTH];
extern uint8_t devID[BUF_LEN_SMALL];

extern uint8_t BATTERY[BUF_LEN_SMALL];

uint8_t  SW_ver[BUF_LEN_SMALL];
uint8_t  SlNo[BUF_LEN_SMALL];
uint8_t dom[BUF_LEN_SMALL];
uint8_t bpon[BUF_LEN_SMALL];
uint8_t bpoff[BUF_LEN_SMALL];
uint8_t lockSteps[BUF_LEN_SMALL];
uint8_t unlockSteps[BUF_LEN_SMALL];
uint8_t lockDelay[BUF_LEN_SMALL];
uint8_t testURL_2[API_BUFF_LEN];



config_data_t configData[13] =
{
		{"<VER>",		"</VER>", 		"TEST",									&SW_ver[0]	},
		{"<WURL>",		"</WURL>",		"http://evegah.kritin.in/api/",			&webURL[0]	},
		{"<APN>",		"</APN>",		"airtelgrps.com",						&APN_URL[0]	},
		{"<SLNO>",		"</SLNO>",		"0000",									&SlNo[0]	},
		{"<DEVID>",		"</DEVID>",		"&dId=EMI2303",					    &devID[0]	},
		{"<DOM>",		"</DOM>",		"1/1/2023",								dom			},
		{"<BPON>",		"</BPON>",		"500",									bpon		},
		{"<BPOFF>",		"</BPOFF>",		"500",									bpoff		},
		{"<TURL>",		"</TURL>",		"www.edel-iot.com/eVega/edel.txt", 		testURL_2	},
		{"<LKSTEPS>",	"</LKSTEPS>",	"200",									lockSteps	},
		{"<UNLKSTEPS>","</UNLKSTEPS>",	"220",									unlockSteps	},
		{"<LKDELAY>",	"</LKDELAY>",	"250",									lockDelay	},
		{"<BATTV>",      "</BATTV>",     "36",                                   &BATTERY   }
};



uint8_t fmc_write_test(uint32_t data)
{

	return 1;
}

void configMode_app()
{
	UART_Write(UART2,"\r\n",2);
	UART_Write(UART2,"+----------------------------------------+\r\n",44);
	UART_Write(UART2,"|     		CALIBRATION MENU		   |\r\n",44);
	UART_Write(UART2,"+----------------------------------------+\r\n",44);
	UART_Write(UART2,"\r\n",2);
	UART_Write(UART2,"1 - Read Configuration Data\r\n",29);
	UART_Write(UART2,"2 - Write Configuration Data\r\n",30);

	/* The ROM address for erase/write/read demo */
	u32FlashStartAddr = DATA_FLASH_START;

	while(configMode)
	{
    	while(expDataIn)
    	{

    	   	if(newWord_2)
			{
				newWord_2 = 0;
				strcpy(command, UART2_RX_BUF);
				memset(UART2_RX_BUF,0,BUF_LEN_LONG);
				count_2 = 0;
				UART_Write(UART2,"New data:\r\n",11);
				UART_Write(UART2,command, strlen(command));
				UART_Write(UART2,"\r\n", 2);
				expDataIn = 0;
				cmdLen  = strlen(command);


				/* Disable register write-protection function */
				SYS_UnlockReg();
				/* Enable FMC ISP functions */
				FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_LDUEN_Msk | FMC_ISPCTL_CFGUEN_Msk;
				FMC_Erase(u32FlashStartAddr);

				for(x = 0; x < cmdLen/4 + 1; ++x)
				{
					memcpy(&u32RData,&command[x * 4],4 );
					tempAddr = u32FlashStartAddr + (x * 4);
					FMC_Write(tempAddr, u32RData);
				}

				dataWr = 1;

				UART_Write(UART2,"Data Readback:\r\n",16);
				if(dataWr)
				{
					dataWr = 0;
					for(x = 0; x < cmdLen/4 + 1; ++x)
					{
						tempAddr = u32FlashStartAddr +  (x * 4);
						cmdReadBk[x] = FMC_Read(tempAddr);
					}
					UART_Write(UART2,cmdReadBk,strlen(cmdReadBk));
				}
				/* Disable FMC ISP function */
				FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

				/* Lock protected registers */
				SYS_LockReg();
			}

    	}

    	if(newWord_2)
    	{
    		newWord_2 = 0;

    		strcpy(command, UART2_RX_BUF);
    		memset(UART2_RX_BUF,0,BUF_LEN_LONG);
    		count_2 = 0;

    		UART_Write(UART2,"\r\n\r\n", 4);

    		if(strstr(command,"1"))
    		{
    			read_config_data();
    		}

    		else if(strstr(command,"2"))
        	{
    			UART_Write(UART2,"Write Calibration Data\r\n",24);
       			expDataIn = 1;
        	}
    	}

	}
}

void read_config_data()
{
	UART_Write(UART2,"Read Calibration Data\r\n",23);

	/* Disable register write-protection function */
	SYS_UnlockReg();

	/* Enable FMC ISP functions */
	FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_LDUEN_Msk | FMC_ISPCTL_CFGUEN_Msk;

	memset(cmdReadBk,0,strlen(cmdReadBk));
	for(x = 0; x < BUF_LEN_LONG/4 + 1; ++x)
	{
			tempAddr = u32FlashStartAddr + (x * 4);
			cmdReadBk[x] = FMC_Read(tempAddr);
		}

	UART_Write(UART2,cmdReadBk,strlen(cmdReadBk));

   /* Disable FMC ISP function */
	FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

	/* Lock protected registers */
	SYS_LockReg();
}


void parse_config_data()
{
	//search for start of kw
	//if found
	//search for start of kw
	//if found copy

	uint8_t xx, yy;

	/*
	tVarStart = strstr(cmdReadBk, "<VER>");
	tVarStart += strlen("<VER>");
	if(tVarStart)
	{
		tVarEnd = strstr(tVarStart +1, "</VER>");
		if(tVarEnd > tVarStart)
		{
			strncpy(SW_variant, tVarStart, tVarEnd - tVarStart );
		}
		else
		{
			strncpy(SW_variant, "PRODUCTION", 10); //default
		}
	}
	else
	{
		strncpy(SW_variant, "PRODUCTION", 10); //default
	}
	*/
	yy = sizeof(configData)/sizeof(config_data_t);

	for(xx = 0; xx < yy; xx++)
	{
		tVarStart = strstr(cmdReadBk, configData[xx].varNameSt);
		tVarStart += strlen(configData[xx].varNameSt);

		memset(configData[xx].varAddr,0,strlen(configData[xx].varAddr));

		if(tVarStart)
		{
			tVarEnd = strstr(tVarStart +1, configData[xx].varNameEnd);
			if(tVarEnd > tVarStart)
			{
				strncpy(configData[xx].varAddr, tVarStart, tVarEnd - tVarStart );
			}
			else
			{
				strncpy(configData[xx].varAddr, configData[xx].defValue, strlen(configData[xx].defValue));
			}
		}
		else
		{
			strncpy(configData[xx].varAddr, configData[xx].defValue, strlen(configData[xx].defValue));
		}

		UART_Write(UART2,configData[xx].varAddr, strlen(configData[xx].varAddr));
		UART_Write(UART2,"\r\n\r\n", 4);
	}
}




