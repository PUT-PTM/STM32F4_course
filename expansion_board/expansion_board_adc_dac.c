#include <expansion_board/expansion_board_adc_dac.h>

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"


void ADC_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);	// wejscie ADC

	//inicjalizacja wejœcia ADC
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//ADC

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	// niezale¿ny tryb pracy przetworników
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	// zegar g³ówny podzielony przez 2
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	// opcja istotna tylko dla tryby multi ADC
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	// czas przerwy pomiêdzy kolejnymi konwersjami
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	//ustawienie rozdzielczoœci przetwornika na maksymaln¹ (12 bitów)
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	//wy³¹czenie trybu skanowania (odczytywaæ bêdziemy jedno wejœcie ADC
	//w trybie skanowania automatycznie wykonywana jest konwersja na wielu //wejœciach/kana³ach)
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	//w³¹czenie ci¹g³ego trybu pracy
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//wy³¹czenie zewnêtrznego wyzwalania
	//konwersja mo¿e byæ wyzwalana timerem, stanem wejœcia itd. (szczegó³y w //dokumentacji)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//wartoœæ binarna wyniku bêdzie podawana z wyrównaniem do prawej
	//funkcja do odczytu stanu przetwornika ADC zwraca wartoœæ 16-bitow¹
	//dla przyk³adu, wartoœæ 0xFF wyrównana w prawo to 0x00FF, w lewo 0x0FF0
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//liczba konwersji równa 1, bo 1 kana³
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	// zapisz wype³nion¹ strukturê do rejestrów przetwornika numer 1
	ADC_Init(ADC1, &ADC_InitStructure);

	// konfiguracja czasu próbkowania sygna³u
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);

	// uruchomienie modu³y ADC
	ADC_Cmd(ADC1, ENABLE);
}

// ADC DMA code - for future use
/*
volatile uint16_t valueFromADC;

void vExpansionBoardADCDMATask(void* pvparameters)
{
	#define ADC_1_ADDRESS_BASE 0x40012000
	// ADC_DR = ADC regular Data Register
	#define ADC_DR_ADDRESS_OFFSET 0x4C

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE); // wejscie ADC
	//inicjalizacja wejœcia ADC
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	// niezale¿ny tryb pracy przetworników
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	// zegar g³ówny podzielony przez 2
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	// opcja istotna tylko dla tryby multi ADC
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	// czas przerwy pomiêdzy kolejnymi konwersjami
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ADC
	ADC_InitTypeDef ADC_InitStructure;
	//ustawienie rozdzielczoœci przetwornika na maksymaln¹ (12 bitów)
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	//wy³¹czenie trybu skanowania (odczytywaæ bêdziemy jedno wejœcie ADC
	//w trybie skanowania automatycznie wykonywana jest konwersja na wielu //wejœciach/kana³ach)
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	//w³¹czenie ci¹g³ego trybu pracy
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//wy³¹czenie zewnêtrznego wyzwalania
	//konwersja mo¿e byæ wyzwalana timerem, stanem wejœcia itd. (szczegó³y w //dokumentacji)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//wartoœæ binarna wyniku bêdzie podawana z wyrównaniem do prawej
	//funkcja do odczytu stanu przetwornika ADC zwraca wartoœæ 16-bitow¹
	//dla przyk³adu, wartoœæ 0xFF wyrównana w prawo to 0x00FF, w lewo 0x0FF0
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//liczba konwersji równa 1, bo 1 kana³
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	// zapisz wype³nion¹ strukturê do rejestrów przetwornika numer 1
	ADC_Init(ADC1, &ADC_InitStructure);

	// konfiguracja czasu próbkowania sygna³u
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);
	// w³¹czenie wyzwalania DMA po ka¿dym zakoñczeniu konwersji
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_InitTypeDef DMA_initStructure;
	// wybór kana³u DMA
	DMA_initStructure.DMA_Channel = DMA_Channel_0;
	// ustalenie rodzaju transferu (memory2memory / peripheral2memory / memory2peripheral)
	DMA_initStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	// tryb pracy - pojedynczy transfer b¹dŸ powtarzany
	DMA_initStructure.DMA_Mode = DMA_Mode_Circular;
	// ustalenie priorytetu danego kana³u DMA
	DMA_initStructure.DMA_Priority = DMA_Priority_Medium;
	// liczba danych do przes³ania
	DMA_initStructure.DMA_BufferSize = (uint32_t)1;
	// adres Ÿród³owy
	DMA_initStructure.DMA_PeripheralBaseAddr = (uint32_t)(ADC_1_ADDRESS_BASE+ADC_DR_ADDRESS_OFFSET);
	// adres docelowy
	DMA_initStructure.DMA_Memory0BaseAddr = (uint32_t)&valueFromADC;
	// okreslenie, czy adresy maj¹ byæ inkrementowane po ka¿dej przes³anej paczce danych
	DMA_initStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_initStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	// ustalenie rozmiaru przesy³anych danych
	DMA_initStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_initStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	// ustalenie trybu pracy - jednorazwe przes³anie danych
	DMA_initStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_initStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	// wy³¹czenie kolejki FIFO (nie u¿ywana w tym przykadzie)
	DMA_initStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	// wype³nianie wszystkich pól struktury jest niezbêdne w celu poprawnego dzia³ania, wpisanie jednej z dozwolonych wartosci
	DMA_initStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	// zapisanie wype³nionej struktury do rejestrów wybranego po³¹czenia DMA
	DMA_Init(DMA2_Stream4, &DMA_initStructure);
	// uruchomienie odpowiedniego po³¹czenia DMA
	DMA_Cmd(DMA2_Stream4, ENABLE);

	// w³¹czenie DMA dla ADC
	ADC_DMACmd(ADC1, ENABLE);
	// uruchomienie modu³y ADC
	ADC_Cmd(ADC1, ENABLE);

	for(;;)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}
*/

void DAC_DMA_init(void)
{
	#define DAC_CHANNEL_1_ADDRESS_BASE 0x40007400
	// DAC_DHR12R1 = DAC Data Holding Register 12 bits, Right aligned channel 1
	#define DAC_DHR12R1_ADDRESS_OFFSET 0x08
	#define DMA_DAC_SIGNAL_SIZE 32
	const uint16_t DAC_DMA_sine12bit[DMA_DAC_SIGNAL_SIZE] = {
			2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
			3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
			599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647
	};


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE); // wyjscie DAC
	GPIO_InitTypeDef GPIO_InitStructure;
	//inicjalizacja wyjœcia DAC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 21-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 400;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
	TIM_SetCounter(TIM6, 0);

	TIM_Cmd(TIM6, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); //DAC

	DAC_InitTypeDef DAC_InitStructure;
	//wy³¹czenie zewnêtrznego wyzwalania
	//konwersja mo¿e byæ wyzwalana timerem, stanem wejœcia itd. (szczegó³y w //dokumentacji)
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
	//wy³¹czamy generator predefiniowanych przebiegów wyjœciowych (wartoœci //zadajemy sami, za pomoc¹ opowiedniej funkcji)
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	//w³¹czamy buforowanie sygna³u wyjœciowego
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	//DAC_SetChannel1Data(DAC_Align_12b_R, 0x000);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_initStructure;
	// wybór kana³u DMA
	DMA_initStructure.DMA_Channel = DMA_Channel_7;
	// ustalenie rodzaju transferu (memory2memory / peripheral2memory / memory2peripheral)
	DMA_initStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	// tryb pracy - pojedynczy transfer b¹dŸ powtarzany
	DMA_initStructure.DMA_Mode = DMA_Mode_Circular;
	// ustalenie priorytetu danego kana³u DMA
	DMA_initStructure.DMA_Priority = DMA_Priority_Medium;
	// liczba danych do przes³ania
	DMA_initStructure.DMA_BufferSize = (uint32_t)DMA_DAC_SIGNAL_SIZE;
	// adres Ÿród³owy
	DMA_initStructure.DMA_Memory0BaseAddr = (uint32_t)&DAC_DMA_sine12bit;
	// adres docelowy
	DMA_initStructure.DMA_PeripheralBaseAddr = (uint32_t)(DAC_CHANNEL_1_ADDRESS_BASE + DAC_DHR12R1_ADDRESS_OFFSET);
	// zezwolenie na inkrementacje adresu po ka¿dej przes³anej paczce danych
	DMA_initStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_initStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	// ustalenie rozmiaru przesy³anych danych
	DMA_initStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_initStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	// ustalenie trybu pracy - jednorazwe przes³anie danych
	DMA_initStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_initStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	// wy³¹czenie kolejki FIFO (nie u¿ywana w tym przykadzie)
	DMA_initStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	// wype³nianie wszystkich pól struktury jest niezbêdne w celu poprawnego dzia³ania, wpisanie jednej z dozwolonych wartosci
	DMA_initStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	// zapisanie wype³nionej struktury do rejestrów wybranego po³¹czenia DMA
	DMA_Init(DMA1_Stream5, &DMA_initStructure);

	// uruchomienie odpowiedniego po³¹czenia DMA
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

void DAC_DMA_start(void)
{
	DAC_Cmd(DAC_Channel_1, ENABLE);
	// uruchomienie DMA dla pierwszego kana³u DAC
	DAC_DMACmd(DAC_Channel_1, ENABLE);
}

void DAC_DMA_changeFrequency(uint16_t newPeriod)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 21-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = newPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
}
