#ifndef __ADC_H
#define __ADC_H

#include "at32f4xx.h"

void       UserADC_init(void) ;
uint16_t   GetADCVal(uint8_t adcindex);

#endif //__ADC_H