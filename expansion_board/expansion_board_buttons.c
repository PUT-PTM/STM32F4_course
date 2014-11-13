#include <expansion_board/expansion_board_buttons.h>

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

static void BUTTONS_unhandledCase(void)
{
	for(;;)
	{
		asm("nop");
	}
}

void BUTTONS_init(void)
{
	// main board buttons

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

	// expansion board buttons

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// ATTENTION - program can work just fine without button configuration - this is default, startup pin configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

uint8_t BUTTONS_isPressed(BUTTON_number button)
{
	uint8_t buttonState;

	switch(button)
	{
		// default state is 0
		case BUTTON_number_mainBoard1:
			buttonState = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
			break;
		// default state is 1
		case BUTTON_number_expansionBoard1:
			buttonState = !GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
			break;
		case BUTTON_number_expansionBoard2:
			buttonState = !GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
			break;
		case BUTTON_number_expansionBoard3:
			buttonState = !GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5);
			break;
		case BUTTON_number_expansionBoard4:
			buttonState = !GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6);
			break;
		default:
			BUTTONS_unhandledCase();
			break;
	}

	return buttonState;
}
