#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"

#include "expansion_board_led.h"
#include <math.h>

#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/semphr.h"

// RGB LED:
// PC6 (TIM3_CH1 / TIM8_CH1) - R
// PC7 (TIM3_CH2 / TIM8_CH2) - used
// PC8 (TIM3_CH3 / TIM8_CH3) - G
// PC9 (TIM3_CH4 / TIM8_CH4) - B

// LEDS:
// PD0 - PD3

// BUTTONS:
// PE2, PE4-PE6

// AMP:
// PA4 (DAC1_OUT)

// POT:
// PA1 (ADC123_IN1)

// SEN:
// PA2 (ADC123_IN2)

// USART:
// PD8 (USART3_TX)
// PD9 (USART3_RX)

// TEMP:
// PB7 (I2C1_SDA)
// PB8 (I2C1_SDL)

uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.
uint64_t tickTime=0;        // Counts OS ticks (default = 1000Hz).

void vTestTask (void * pvparameters);
void vMainBoardLedsTask(void* pvparameters);
void vMainBoardButtonTask(void* pvparameters);
void vExpansionBoardButton1Task(void* pvparameters);
void vExpansionBoardButton2Task(void* pvparameters);
void vExpansionBoardButton3Task(void* pvparameters);
void vExpansionBoardButton4Task(void* pvparameters);
void vExpansionBoardRGBLedFader(void* pvparameters);
void vExpansionBoardDACTask(void* pvparameters);

/*
 * When FreeRTOS crashes, you often end up in a hard fault.
 */
void HardFault_Handler (void){
	LED_turnOnAll();
}

void BUTTONS_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	// ATTENTION - program can work just fine without button configuration - this is default, startup pin configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void EXPANSION_BOARD_test(void)
{
	SystemInit();

	LED_init();
	BUTTONS_init();

	//xTaskCreate( vTestTask, ( signed char * ) "Test Task", 100, NULL, 1, NULL );
	xTaskCreate( vMainBoardLedsTask, "Main board leds task", 100, NULL, 1, NULL );
	xTaskCreate( vMainBoardButtonTask, "Main board button task", 100, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardButton1Task, "Expansion board button1 task", 50, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton2Task, "Expansion board button2 task", 50, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton3Task, "Expansion board button3 task", 50, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton4Task, "Expansion board button4 task", 50, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardRGBLedFader, "RGB fader", 200, NULL, 1, NULL );


	//xTaskCreate( vExpansionBoardDACTask, "DAC dma", 200, NULL, 1, NULL );


    vTaskStartScheduler(); // This should never return.

    for(;;)
    {
    	// Error
    	LED_toggle(LED_number_mainBoard1);
    	uint32_t i;
    	for(i=0; i<10000000; ++i)
    	{
    		asm("nop");
    	}
    }
}

static volatile uint8_t MainBoardLedChange = 1;

void vMainBoardLedsTask(void* pvparameters)
{
	LED_number currentLed = LED_number_mainBoard1;

	for (;;)
	{
		LED_turnOff(currentLed);

		currentLed += MainBoardLedChange;

		if(currentLed == (LED_number_mainBoard4 + 1))
		{
			currentLed = LED_number_mainBoard1;
		}
		else if(currentLed == (LED_number_mainBoard1 - 1))
		{
			currentLed = LED_number_mainBoard4;
		}
		LED_turnOn(currentLed);

		vTaskDelay(250 / portTICK_RATE_MS);
	}
}

void vMainBoardButtonTask(void* pvparameters)
{
	// USER BUTTON
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	// USER BUTTON
	// ATTENTION - program can work just fine without button configuration - this is default, startup pin configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	uint8_t userButtonState;
	for(;;)
	{
		userButtonState = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		if(userButtonState != Bit_RESET)
		{
			// wait a little to debounce
			vTaskDelay(50 / portTICK_RATE_MS);
			// check once again
			uint8_t userButtonState = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
			if(userButtonState != Bit_RESET)
			{
				MainBoardLedChange = -MainBoardLedChange;
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
	}
}

void vExpansionBoardRGBLedFader(void* pvparameters)
{
	// idea from: http://www.instructables.com/id/Digispark-RGB-LED-Fader/step4/Arduino-Program/
	uint32_t i;

	uint8_t redValue = 0;
	uint8_t greenValue = 0;
	uint8_t blueValue = 0;

	#define PI			3.14

	for(;;)
	{
		for(i=0; i<360; ++i)
		{
			float valueInRadians = (float)i * 3.14 / 180.0;

			redValue = (uint8_t)(50.0 * (sinf(valueInRadians) + 1.0));
			greenValue = (uint8_t)(50.0 * (sinf(valueInRadians + 1.5*PI) + 1.0));
			blueValue = (uint8_t)(50.0 * (sinf(valueInRadians + 0.5*PI) + 1.0));

			LED_setRGBValue(LED_number_RGB_R, redValue);
			LED_setRGBValue(LED_number_RGB_G, greenValue);
			LED_setRGBValue(LED_number_RGB_B, blueValue);

			vTaskDelay(10 / portTICK_RATE_MS);
		}
	}
}

void vExpansionBoardDACTask(void* pvparameters)
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
	TIM_TimeBaseStructure.TIM_Prescaler = 42-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20;
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

	DAC_Cmd(DAC_Channel_1, ENABLE);
	// uruchomienie DMA dla pierwszego kana³u DAC
	DAC_DMACmd(DAC_Channel_1, ENABLE);

	for(;;)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void vExpansionBoardButton1Task(void* pvparameters)
{
	uint8_t buttonState;

	for (;;)
	{
		buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
		if(buttonState != Bit_SET)
		{
			// wait a little to debounce
			vTaskDelay(50 / portTICK_RATE_MS);
			// check once again
			uint8_t buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
			if(buttonState != Bit_SET)
			{
				LED_toggle(LED_number_expansionBoard1);
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
	}
}

void vExpansionBoardButton2Task(void* pvparameters)
{
	uint8_t buttonState;

	for (;;)
	{
		buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
		if(buttonState != Bit_SET)
		{
			// wait a little to debounce
			vTaskDelay(50 / portTICK_RATE_MS);
			// check once again
			uint8_t buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
			if(buttonState != Bit_SET)
			{
				LED_toggle(LED_number_expansionBoard2);
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
	}
}

void vExpansionBoardButton3Task(void* pvparameters)
{
	uint8_t buttonState;

	for (;;)
	{
		buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5);
		if(buttonState != Bit_SET)
		{
			// wait a little to debounce
			vTaskDelay(50 / portTICK_RATE_MS);
			// check once again
			uint8_t buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5);
			if(buttonState != Bit_SET)
			{
				LED_toggle(LED_number_expansionBoard3);
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
	}
}

void vExpansionBoardButton4Task(void* pvparameters)
{
	uint8_t buttonState;

	for (;;)
	{
		buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6);
		if(buttonState != Bit_SET)
		{
			// wait a little to debounce
			vTaskDelay(50 / portTICK_RATE_MS);
			// check once again
			uint8_t buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6);
			if(buttonState != Bit_SET)
			{
				LED_toggle(LED_number_expansionBoard4);
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
	}
}



/*
 * A task that read reads and debounces the user's button
 */
void vTestTask( void *pvparameters )
{
	uint8_t state = 0;

	for (;;)
	{
		if(state == 0)
		{
			LED_turnOnAll();
			state = 1;
		}
		else
		{
			LED_turnOffAll();
			state = 0;
		}

		vTaskDelay(2000 / portTICK_RATE_MS);
	}
}

// This FreeRTOS callback function gets called once per tick (default = 1000Hz).
// ----------------------------------------------------------------------------
void vApplicationTickHook( void ) {
    ++tickTime;
}

// This FreeRTOS call-back function gets when no other task is ready to execute.
// On a completely unloaded system this is getting called at over 2.5MHz!
// ----------------------------------------------------------------------------
void vApplicationIdleHook( void ) {
    ++u64IdleTicksCnt;
}

// A required FreeRTOS function.
// ----------------------------------------------------------------------------
void vApplicationMallocFailedHook( void ) {
    configASSERT( 0 );  // Latch on any failure / error.
}
