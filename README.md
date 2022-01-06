# STM32-simple-adc-dac-demo
Simple demo for STM32F756ZG on Nucleo-144 board providing UDP/IP, SNMP, Console and ADC/DAC demo

## Description

The demo is running on RTOS. There are 3 tasks created:
-Default task: Used for the ADC convertor reading and sending of UDP packages
-DAC task: used to drive DAC changes. User can connect the output of DAC located on the SPI_B set of pins (PA4) to the ADC input on A0 (PA3) to test the conversion accuracy
-Console task: Provides a console attached to UART3 (the ST LINK virtual COM port). Connect with 115200 N 8 1 to the virtual port and you will get a prompt. Use help to see the supported commands.

