// Jesse Jamieson A1925902 , Lucas Shaw A1886336, Lachlan Smart A1924193
// Sections of code are not original, provided by lecture and project material, and modified to suit the project requirements.
#ifndef UART_H
#define UART_H

#include <stdint.h>

void UART_Init(void);
uint8_t UART_Rx(void);
void UART_Tx(uint8_t byte);

#endif