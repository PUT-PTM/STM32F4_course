#ifndef _EXPANSION_BOARD_ADC_DAC_H_
#define _EXPANSION_BOARD_ADC_DAC_H_

#include <stdint.h>


void ADC_init(void);

void DAC_DMA_init(void);

void DAC_DMA_start(void);

void DAC_DMA_changeFrequency(uint16_t newPeriod);

#endif
