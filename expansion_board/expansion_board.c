#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"

#include "expansion_board_led.h"
#include "expansion_board_buttons.h"
#include "expansion_board_adc_dac.h"
#include "expansion_board_i2c.h"
#include <math.h>

#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/semphr.h"

static uint64_t u64IdleTicksCnt = 0; 	// Counts when the OS has no task to execute.
uint64_t tickTime=0;        			// Counts OS ticks (default = 1000Hz).

void vTestTask (void * pvparameters);
void vMainBoardLedsTask(void* pvparameters);
void vMainBoardButtonTask(void* pvparameters);
void vExpansionBoardButton1Task(void* pvparameters);
void vExpansionBoardButton2Task(void* pvparameters);
void vExpansionBoardButton3Task(void* pvparameters);
void vExpansionBoardButton4Task(void* pvparameters);
void vExpansionBoardRGBLedFader(void* pvparameters);
void vExpansionBoardADCTask(void* pvparameters);
void vExpansionBoardDACDMATask(void* pvparameters);
void vExpansionBoardUSARTTask(void* pvparameters);
void vExpansionBoardI2CTemperatureTask(void* pvparameters);

/*
 * When FreeRTOS crashes, you often end up in a hard fault.
 */
void HardFault_Handler (void){
	LED_turnOnAll();
}

void EXPANSION_BOARD_test(void)
{
	SystemInit();

	LED_init();
	BUTTONS_init();
	ADC_init();
	DAC_DMA_init();
	init_I2C1();

	xTaskCreate( vMainBoardLedsTask, "Main board leds task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vMainBoardButtonTask, "Main board button task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardButton1Task, "Expansion board button1 task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton2Task, "Expansion board button2 task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton3Task, "Expansion board button3 task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton4Task, "Expansion board button4 task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardRGBLedFader, "RGB fader", 200, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardUSARTTask, "USART sender", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardI2CTemperatureTask, "I2C sensor", configMINIMAL_STACK_SIZE, NULL, 1, NULL );


	xTaskCreate( vExpansionBoardDACDMATask, "DAC dma", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardADCTask, "ADC dma", configMINIMAL_STACK_SIZE, NULL, 1, NULL );


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



#define MPC9800_ADDRESS 0x90 // adres MCP9800

static volatile uint16_t temperature;

void vExpansionBoardI2CTemperatureTask(void* pvparameters)
{
	// na podstawie: http://eliaselectronics.com/stm32f4-tutorials/stm32f4-i2c-mastertutorial/
	I2C_start(I2C1, MPC9800_ADDRESS, I2C_Direction_Transmitter);
	I2C_write(I2C1,0x01); // ustaw adres rejestru Configuration Register
	I2C_write(I2C1,0x30); // ustaw rozdzielczosc na 12 bit
	I2C_stop(I2C1); // stop the transmission
	I2C_start(I2C1, MPC9800_ADDRESS, I2C_Direction_Transmitter);
	I2C_write(I2C1,0x00); // ustaw adres rejestru z temperatur¹
	I2C_stop(I2C1); // stop the transmission

	uint8_t lowByte;
	uint8_t highByte;

	for(;;)
	{
		I2C_start(I2C1, MPC9800_ADDRESS, I2C_Direction_Receiver); // start a transmission in Master receiver mode
		highByte = I2C_read_ack(I2C1); // read one byte and request another byte
		lowByte = I2C_read_nack(I2C1); // read one byte and don't request another byte
		I2C_stop(I2C1); // stop the transmission
		temperature = ((uint16_t)highByte << 4) + ((uint16_t)lowByte >> 4);

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void USART_write(uint8_t data)
{
	// Send a char...
	USART_SendData(USART3, data);

	// ... wait until the char is sended
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
		asm("nop");
}

void vExpansionBoardUSARTTask(void* pvparameters)
{
	//NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure USART2_Tx and USART2_Rx as alternate function */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	/* Enable USART3 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// enable oversampling by 8 for higher speeds
	USART_OverSampling8Cmd(USART3, ENABLE);

	/* USART3 configuration ------------------------------------------------------*/
	 /* USART3 configured as follow:
		   - BaudRate = 460,8k baud
		   - Word Length = 8 Bits
		   - One Stop Bit
		   - No parity
		   - Hardware flow control - none
		   - Receive and transmit enabled
	 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	/* Enable the USART3 Interrupt */
	//NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);

	/* Enable the USART3 */
	USART_Cmd(USART3, ENABLE);

	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART3, USART_IT_TC, ENABLE);

	for(;;)
	{
		USART_write('t');
		USART_write('e');
		USART_write('m');
		USART_write('p');
		USART_write(':');
		USART_write(' ');
		uint8_t temperature8bits = (temperature >> 4);
		USART_write(temperature8bits);
		USART_write(' ');
		vTaskDelay(1000 / portTICK_RATE_MS);
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
	for(;;)
	{
		if(BUTTONS_isPressed(BUTTON_number_mainBoard1))
		{
			// wait a little to debounce
			vTaskDelay(100 / portTICK_RATE_MS);
			// check once again
			if(BUTTONS_isPressed(BUTTON_number_mainBoard1))
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

	#define PI			3.14F

	for(;;)
	{
		for(i=0; i<360; ++i)
		{
			float valueInRadians = (float)i * PI / 180.0;

			redValue = (uint8_t)(50.0F * (sinf(valueInRadians) + 1.0F));
			greenValue = (uint8_t)(50.0F * (sinf(valueInRadians + 1.5F*PI) + 1.0F));
			blueValue = (uint8_t)(50.0F * (sinf(valueInRadians + 0.5F*PI) + 1.0F));

			LED_setRGBValue(LED_number_RGB_R, redValue);
			LED_setRGBValue(LED_number_RGB_G, greenValue);
			LED_setRGBValue(LED_number_RGB_B, blueValue);

			vTaskDelay(10 / portTICK_RATE_MS);
		}
	}
}

void vExpansionBoardADCTask(void* pvparameters)
{
	for(;;)
	{
		ADC_SoftwareStartConv(ADC1);
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
		{
			vTaskDelay(10 / portTICK_RATE_MS);
		}
		uint16_t valueFromADC = ADC_GetConversionValue(ADC1);

		// valueFromADC - 12 bit - 0 - 4096
		// converted to - 0 - 410
		uint16_t newPeriod = (valueFromADC >> 2) + 1;

		DAC_DMA_changeFrequency(newPeriod);

		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

void vExpansionBoardDACDMATask(void* pvparameters)
{
	DAC_DMA_start();

	for(;;)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void vExpansionBoardButton1Task(void* pvparameters)
{
	for (;;)
	{
		if(BUTTONS_isPressed(BUTTON_number_expansionBoard1))
		{
			// wait a little to debounce
			vTaskDelay(100 / portTICK_RATE_MS);
			// check once again
			if(BUTTONS_isPressed(BUTTON_number_expansionBoard1))
			{
				LED_toggle(LED_number_expansionBoard1);
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
	}
}

void vExpansionBoardButton2Task(void* pvparameters)
{
	for (;;)
	{
		if(BUTTONS_isPressed(BUTTON_number_expansionBoard2))
		{
			// wait a little to debounce
			vTaskDelay(100 / portTICK_RATE_MS);
			// check once again
			if(BUTTONS_isPressed(BUTTON_number_expansionBoard2))
			{
				LED_toggle(LED_number_expansionBoard2);
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
	}
}

void vExpansionBoardButton3Task(void* pvparameters)
{
	for (;;)
	{
		if(BUTTONS_isPressed(BUTTON_number_expansionBoard3))
		{
			// wait a little to debounce
			vTaskDelay(100 / portTICK_RATE_MS);
			// check once again
			if(BUTTONS_isPressed(BUTTON_number_expansionBoard3))
			{
				LED_toggle(LED_number_expansionBoard3);
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
	}
}

void vExpansionBoardButton4Task(void* pvparameters)
{
	for (;;)
	{
		if(BUTTONS_isPressed(BUTTON_number_expansionBoard4))
		{
			// wait a little to debounce
			vTaskDelay(100 / portTICK_RATE_MS);
			// check once again
			if(BUTTONS_isPressed(BUTTON_number_expansionBoard4))
			{
				LED_toggle(LED_number_expansionBoard4);
			}
		}
		vTaskDelay(25 / portTICK_RATE_MS);
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
