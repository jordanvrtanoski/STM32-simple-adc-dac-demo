/*
 * terminal.h
 *
 *  Created on: Jan 5, 2022
 *      Author: jordanvrtanoski
 */

#ifndef INC_CONSOLE_H_
#define INC_CONSOLE_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Defines ------------------------------------------------------------------*/

#define WRITE_TIME_OUT 1000
#define READ_TIME_OUT 10

/* Declares ------------------------------------------------------------------*/

typedef int (* commandCallback)(UART_HandleTypeDef * huart, uint8_t **argv, int argc);

typedef struct cCommand{
	uint8_t *command;
	uint8_t *description;
	commandCallback callback;
	struct cCommand *next;
}cCommand;

int readCommand(UART_HandleTypeDef * huart, uint8_t *cmdBuff, int buffLen);
int tokenizeCommand(uint8_t *cmdBuff, int buffLen, uint8_t** argv, int argvLen);
cCommand* findCommand(uint8_t *cmd);
int addCommand(cCommand *cmd);

#ifdef __cplusplus
}
#endif

#endif /* INC_CONSOLE_H_ */
