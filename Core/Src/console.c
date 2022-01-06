/*
 * terminal.c
 *
 *  Created on: Jan 5, 2022
 *      Author: jordanvrtanoski
 */

#include "console.h"
#include <string.h>

// Initializing the command list, and the help command is the first command in the list
int helpCommand(UART_HandleTypeDef * huart, uint8_t **argv, int argc);

cCommand rootCommands = {
		.command = (uint8_t *)"help",
		.description = (uint8_t *)"Get help for a specific command. \r\nUsage:\r\nhelp [command name]\r\nExamples:\r\nhelp help",
		.callback = helpCommand,
		.next = NULL
};

uint8_t pNewLine[] = "\n\r";

int helpCommand(UART_HandleTypeDef * huart, uint8_t **argv, int argc) {
  uint8_t pTitle[] = "\r\n\e[32mList of known commands\e[0m\r\n----------------------\r\n";
  uint8_t pFooter[] = "\r\n\e[32mTo get more information on a specific command, use \e[0m\r\nhelp [command name]\r\n";
  uint8_t pWrongCommand[] = "\e[31mInvalid command\e[0m ";
  cCommand* cmd;

  switch (argc) {
  case 1:
	  HAL_UART_Transmit(huart, pTitle, sizeof(pTitle), WRITE_TIME_OUT);
	  cCommand *current = &rootCommands;
	  while(current != NULL) {
		  HAL_UART_Transmit(huart, current->command, strlen((char *)current->command), WRITE_TIME_OUT);
		  HAL_UART_Transmit(huart, pNewLine, sizeof(pNewLine), WRITE_TIME_OUT);
		  current = current->next;
	  }
	  HAL_UART_Transmit(huart, pFooter, sizeof(pFooter), WRITE_TIME_OUT);
	  break;
  case 2:
	  cmd = findCommand(argv[1]);
	  if (cmd == NULL) {
		  HAL_UART_Transmit(huart, pWrongCommand, sizeof(pWrongCommand), WRITE_TIME_OUT);
		  HAL_UART_Transmit(huart, argv[1], strlen((char *)argv[1]), WRITE_TIME_OUT);
		  HAL_UART_Transmit(huart, pNewLine, sizeof(pNewLine), WRITE_TIME_OUT);
	  } else {
		  HAL_UART_Transmit(huart, cmd->description, strlen((char *)cmd->description), WRITE_TIME_OUT);
		  HAL_UART_Transmit(huart, pNewLine, sizeof(pNewLine), WRITE_TIME_OUT);
	  }
	  break;
  default:
	  HAL_UART_Transmit(huart, pFooter, sizeof(pFooter), WRITE_TIME_OUT);
	  return -1;
	  break;
  }
  return 0;
}

/**
 *
 */
cCommand* findCommand(uint8_t *cmd) {
	  cCommand *current = &rootCommands;
	  while(current != NULL) {
		  if (strcmp((char *)current->command, (char *)cmd) == 0)
			  return current;
		  current = current->next;
	  }
	  return NULL;
}

/**
 *
 */
int addCommand(cCommand *cmd) {
	  cCommand *current = &rootCommands;
	  while(current->next != NULL) {
		  current = current->next;
	  }
	  cmd->next = NULL;
	  current->next = cmd;
	  return 0;
}

int readCommand(UART_HandleTypeDef * huart, uint8_t *cmdBuff, int buffLen) {
  uint8_t rData[2];
  uint8_t rescBuffer[64];
  uint8_t pEnterCommand[] = "\e[34mEnter command: \e[0m";
  uint8_t pOverflow[] = "\e[31mBuffer Overflow, command is to long\e[0m";
  uint8_t pClearCursorToEOL[] = "\e[K";
  uint8_t pCoursorBackOne[] = "\e[1D";
  int count=0;
  int escCount=0;
  int inEscape = 0;

  HAL_UART_Transmit(huart, pNewLine, sizeof(pNewLine), WRITE_TIME_OUT);
  HAL_UART_Transmit(huart, pEnterCommand, sizeof(pEnterCommand), WRITE_TIME_OUT);
  for(;;)
  {
	  if (HAL_UART_Receive(huart, rData, 1, READ_TIME_OUT) == HAL_OK) {
		  if (inEscape == 0)  {
			switch (rData[0]) {
			  case '\r':
				  if (count > 0) {
					  // We have captured a command in the buffer.
					  cmdBuff[count++]='\0';
					  return count;
				  }
				  HAL_UART_Transmit(huart, pNewLine, sizeof(pNewLine), WRITE_TIME_OUT);
				  HAL_UART_Transmit(huart, pEnterCommand, sizeof(pEnterCommand), WRITE_TIME_OUT);
				  count=0;
				  break;
			  case '\b':
			  case '\177':
				  if (count > 0) {
					  count--;
					  HAL_UART_Transmit(huart, pCoursorBackOne, sizeof(pCoursorBackOne), WRITE_TIME_OUT);
					  HAL_UART_Transmit(huart, pClearCursorToEOL, sizeof(pClearCursorToEOL), WRITE_TIME_OUT);
				  }
				  break;
			  case '\e':
				  // Escape Starts, enter ESC/CSI mode
				  inEscape = 1;
				  escCount=0;
				  break;
			  case 0:
				  break;
			  default:
				  HAL_UART_Transmit(huart,rData, 1, WRITE_TIME_OUT);  // echo
				  cmdBuff[count++] = rData[0];
				  break;
			}
		  } else {
			switch (rData[0]) {
			  // Handle the escape sequence characters
			  case '[':
				  // If the ESC if followed by '[', we have CSI string
				  inEscape = 2;
			  default:
				  rescBuffer[escCount++] = rData[0];
				  // If we have CSI string, the string is terminated by a valid letter,
				  // If it's not CSI string, than ESC is followed by only one character
				  if (   (rData[0] >= 'a' && rData[0] <= 'z')
				      || (rData[0] >= 'A' && rData[0] <= 'Z')
					  || inEscape == 1 ) {
					  // Return from ESC/CSI mode
					  if ( rData[0] == 'D' ) // Left
					  {}
					  if ( rData[0] == 'C' ) // Right
					  {}
					  inEscape = 0;
				  }
			}
		  }
		  if (count == buffLen) {
			  // Buffer overflow
			  HAL_UART_Transmit(huart, pNewLine, sizeof(pNewLine), WRITE_TIME_OUT);
			  HAL_UART_Transmit(huart, pOverflow, sizeof(pOverflow), WRITE_TIME_OUT);
			  HAL_UART_Transmit(huart, pNewLine, sizeof(pNewLine), WRITE_TIME_OUT);
			  HAL_UART_Transmit(huart, pEnterCommand, sizeof(pEnterCommand), WRITE_TIME_OUT);
			  count=0;
		  }
	  }
  }
  return -1;
}

/**
 * We will search for the gaps in the text created by space or special characters and convert the text buffer to array of string pointers
 * (tokens) by inserting a '\0' character at the end of the token.
 *
 */
#define _TOKENIZER_TOKEN_SEEK_START   0
#define _TOKENIZER_TOKEN_SEEK_END     1

int tokenizeCommand(uint8_t *cmdBuff, int buffLen, uint8_t** argv, int argvLen) {
	int state = _TOKENIZER_TOKEN_SEEK_START;
	int argc = 0;

	for (int i=0; i<buffLen; i++) {
		switch (state) {
		case _TOKENIZER_TOKEN_SEEK_START:
			// is a valid alphanumeric and special signs character
			if (cmdBuff[i] >= '!' && cmdBuff[i] <= '~') {
				argv[argc]=&cmdBuff[i];
				state = _TOKENIZER_TOKEN_SEEK_END;
			} else {
				cmdBuff[i] = '\0';
			}
			break;
		case _TOKENIZER_TOKEN_SEEK_END:
			if ( !(cmdBuff[i] >= '!' && cmdBuff[i] <= '~')) {
				cmdBuff[i] = '\0';
				state = _TOKENIZER_TOKEN_SEEK_START;
				argc++;
				if (argc == argvLen) {
					return argc;
				}
			}
			break;
		default:
			return -1;
		}
	}
	return argc;
}
