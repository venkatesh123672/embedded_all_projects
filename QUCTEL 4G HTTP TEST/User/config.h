/*
 * config.h
 *
 *  Created on: 29-Jun-2023
 *      Author: Kushi
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define BUF_LEN_SMALL 20
#define API_BUFF_LEN 50
#define URL_LENGTH 50
#define BUF_LEN 250
#define BUF_LEN_LONG 500
#define DATA_FLASH_START 0x10000

#define CONFIG_NAME_LEN 50

typedef struct
{
	const char varNameSt[CONFIG_NAME_LEN];
	const char varNameEnd[CONFIG_NAME_LEN];
	const char defValue[CONFIG_NAME_LEN];
	uint8_t * varAddr;
}config_data_t;

void configMode_app();
void read_config_data();
void parse_config_data();

#endif /* CONFIG_H_ */
