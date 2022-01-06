/*
 * commands.c
 *
 *  Created on: Jan 6, 2022
 *      Author: jordanvrtanoski
 */

#include "commands.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <string.h>

char psBuff[40*128];

int commandPS(UART_HandleTypeDef * huart, uint8_t **argv, int argc) {
	static uint8_t pHeader[] = "\n\rName          Steate    Pri    Stack   Num\r\n-----------------------------------------\r\n";
	vTaskList(psBuff);
	HAL_UART_Transmit(huart, pHeader, sizeof(pHeader),1000);
	HAL_UART_Transmit(huart, (uint8_t *)psBuff, strlen(psBuff),1000);
	return 0;
}

cCommand psCommand = {
		.command = (uint8_t *)"ps",
		.description = (uint8_t *)"Provides list of processes running at the system",
		.callback = commandPS,
		.next = NULL
};

int commandPSST(UART_HandleTypeDef * huart, uint8_t **argv, int argc) {
	static uint8_t pHeader[] = "\n\rName          Steate    Pri    Stack   Num\r\n-----------------------------------------\r\n";
	vTaskGetRunTimeStats(psBuff);
	HAL_UART_Transmit(huart, pHeader, sizeof(pHeader),1000);
	HAL_UART_Transmit(huart, (uint8_t *)psBuff, strlen(psBuff),1000);
	return 0;
}

cCommand psstCommand = {
		.command = (uint8_t *)"psst",
		.description = (uint8_t *)"Provides list of processes statistics",
		.callback = commandPSST,
		.next = NULL
};


int CONSOLE_CMD_init(void) {
	addCommand(&psCommand);
	addCommand(&psstCommand);
	return 0;
}
