// Jesse Jamieson A1925902 , Lucas Shaw A1886336, Lachlan Smart A1924193
// code is not original, provided by lecture and project material, and modified to suit

#ifndef ADC_H
#define ADC_H
#include <stdint.h>
#define ADC_CHANNEL_1 ((uint8_t)1U)
void ADC_Init (void) ;
uint16_t ADC_Read (uint8_t channel );
#endif