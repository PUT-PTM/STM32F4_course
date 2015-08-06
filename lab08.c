#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

void MF_USART_init(void)
{
	// wlaczenie taktowania wybranego portu
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// wlaczenie taktowania wybranego uk³adu USART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// konfiguracja linii Tx
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// konfiguracja linii Rx
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	// predkosc transmisji (mozliwe standardowe opcje: 9600, 19200, 38400, 57600, 115200, ...)
	USART_InitStructure.USART_BaudRate = 115200;
	// d³ugoœæ s³owa (USART_WordLength_8b lub USART_WordLength_9b)
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// liczba bitów stopu (USART_StopBits_1, USART_StopBits_0_5, USART_StopBits_2, USART_StopBits_1_5)
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// sprawdzanie parzystoœci (USART_Parity_No, USART_Parity_Even, USART_Parity_Odd)
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// sprzêtowa kontrola przep³ywu (USART_HardwareFlowControl_None, USART_HardwareFlowControl_RTS, USART_HardwareFlowControl_CTS, USART_HardwareFlowControl_RTS_CTS)
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// tryb nadawania/odbierania (USART_Mode_Rx, USART_Mode_Rx )
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	// konfiguracja
	USART_Init(USART3, &USART_InitStructure);

	// wlaczenie ukladu USART
	USART_Cmd(USART3, ENABLE);
}

void MF_waitALittle(void)
{
	int i,j;
	for(i=0; i<10000; ++i)
	{
		for(j=0; j<1000; ++j)
		{
			asm("nop");
		}
	}
}

void lab08_USARTsimpleSend(void)
{
	SystemInit();

	MF_USART_init();

	for(;;)
	{
		// wyslanie danych
		USART_SendData(USART3, '2');
		// czekaj az dane zostana wyslane
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		{
			asm("nop");
		}

		MF_waitALittle();
	}
}

void usartPutChar(uint8_t charToSend)
{
	USART_SendData(USART3, charToSend);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
	{
		asm("nop");
	}
}

uint8_t usartGetChar(void)
{
	// czekaj na odebranie danych
	while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
	{
		asm("nop");
	}

	return USART_ReceiveData(USART3);
}

void lab08_USARTsimpleReceive(void)
{
	SystemInit();

	MF_USART_init();

	for(;;)
	{
		usartPutChar('\n');
		usartPutChar('p');
		usartPutChar('r');
		usartPutChar('e');
		usartPutChar('s');
		usartPutChar('s');
		usartPutChar(' ');
		usartPutChar('a');
		usartPutChar('n');
		usartPutChar('y');
		usartPutChar(' ');
		usartPutChar('k');
		usartPutChar('e');
		usartPutChar('y');
		usartPutChar(' ');

		uint8_t dataFromUser = usartGetChar();

		usartPutChar(dataFromUser);
	}
}
