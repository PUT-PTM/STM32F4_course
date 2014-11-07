#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_i2c.h"

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
void vExpansionBoardADCTask(void* pvparameters);
void vExpansionBoardDACTask(void* pvparameters);
void vExpansionBoardUSARTTask(void* pvparameters);
void vExpansionBoardI2CTemperatureTask(void* pvparameters);

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
	xTaskCreate( vMainBoardLedsTask, "Main board leds task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vMainBoardButtonTask, "Main board button task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardButton1Task, "Expansion board button1 task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton2Task, "Expansion board button2 task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton3Task, "Expansion board button3 task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vExpansionBoardButton4Task, "Expansion board button4 task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardRGBLedFader, "RGB fader", 200, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardUSARTTask, "USART sender", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	xTaskCreate( vExpansionBoardI2CTemperatureTask, "I2C sensor", configMINIMAL_STACK_SIZE, NULL, 1, NULL );


	xTaskCreate( vExpansionBoardDACTask, "DAC dma", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
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

/* This function issues a start condition and
* transmits the slave address + R/W bit
*
* Parameters:
* I2Cx --> the I2C peripheral e.g. I2C1
* address --> the 7 bit slave address
* direction --> the tranmission direction can be:
* I2C_Direction_Tranmitter for Master transmitter mode
* I2C_Direction_Receiver for Master receiver
*/
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);
	/* wait for I2C1 EV6, check if
	* either Slave has acknowledged Master transmitter or
	* Master receiver mode, depending on the transmission
	* direction
	*/
	if(direction == I2C_Direction_Transmitter)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}
/* This function transmits one byte to the slave device
* Parameters:
* I2Cx --> the I2C peripheral e.g. I2C1
* data --> the data byte to be transmitted
*/
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
/* This function reads one byte from the slave device
* and acknowledges the byte (requests another byte)
*/
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}
/* This function reads one byte from the slave device
* and doesn't acknowledge the recieved data
*/
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
	// disabe acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}
/* This funtion issues a stop condition and therefore
* releases the bus
*/
void I2C_stop(I2C_TypeDef* I2Cx)
{
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}
void I2C_slave_start(I2C_TypeDef* I2Cx)
{
	/* Test on I2C1 EV1 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED));
}

/* This funtion issues a stop condition and therefore
* releases the bus
*/
void I2C_slave_stop(I2C_TypeDef* I2Cx)
{
	/* Test on I2C2 EV4 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_STOP_DETECTED));
	/* Clear I2C2 STOPF flag: read operation to I2C_SR1 followed by a
	write operation to I2C_CR1 */
	(void)(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
	I2C_Cmd(I2Cx, ENABLE);
}
/* This function reads one byte from the slave device
* and acknowledges the byte (requests another byte)
*/
uint8_t I2C_slave_read_ack(I2C_TypeDef* I2Cx)
{
	/* Test on I2C2 EV2 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_BYTE_RECEIVED));
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}
/* This function reads one byte from the slave device
* and doesn't acknowledge the recieved data
*/
uint8_t I2C_slave_read_nack(I2C_TypeDef* I2Cx)
{
	/* Test on I2C2 EV2 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_BYTE_RECEIVED));
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

void init_I2C1(void)
{
	#define I2C1_SLAVE_ADDRESS7 0x90

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* setup SCL and SDA pins
	* You can connect I2C1 to two different
	* pairs of pins:
	* 1. SCL on PB6 and SDA on PB7
	* 2. SCL on PB8 and SDA on PB9
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct); // init GPIOB
	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); // SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
	// configure I2C1
	I2C_InitStruct.I2C_ClockSpeed = 100000; // 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7; // own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable; // disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct); // init I2C1
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

uint16_t temperature;

void vExpansionBoardI2CTemperatureTask(void* pvparameters)
{
	#define MPC9800_ADDRESS 0x90 // adres MCP9800
	//#define I2C_Speed 200000


	init_I2C1(); // na podstawie: http://eliaselectronics.com/stm32f4-tutorials/stm32f4-i2c-mastertutorial/
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
		USART_write('h');
		USART_write('e');
		USART_write('l');
		USART_write('l');
		USART_write('o');
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
			vTaskDelay(100 / portTICK_RATE_MS);
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

void vExpansionBoardADCTask(void* pvparameters)
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

	for(;;)
	{
		ADC_SoftwareStartConv(ADC1);
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
		{
			asm("nop");
		}
		uint16_t valueFromADC = ADC_GetConversionValue(ADC1);

		// valueFromADC - 12 bit - 0 - 4096
		uint8_t newPeriod = (valueFromADC >> 2) + 1;

		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Prescaler = 21-1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = newPeriod;
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

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
			vTaskDelay(100 / portTICK_RATE_MS);
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
			vTaskDelay(100 / portTICK_RATE_MS);
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
			vTaskDelay(100 / portTICK_RATE_MS);
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
			vTaskDelay(100 / portTICK_RATE_MS);
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
