#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

void lab02(void)
{
	SystemInit();

	/* GPIOD Periph clock enable */
	// LEDS
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// USER BUTTON
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	// LEDS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// USER BUTTON
	// ATTENTION - program can work just fine without button configuration - this is default, startup pin configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	uint8_t state = 0;

	unsigned int i;
	for(;;)
	{
		uint8_t userButtonState = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		if(userButtonState != Bit_RESET)
		{
			// wait a little to debounce
			for(i=0; i<1000000; ++i)
			{
				asm("nop");
			}
			// check once again
			uint8_t userButtonState = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
			if(userButtonState != Bit_RESET)
			{
				if(state == 0)
				{
					state = 1;
				}
				else
				{
					state = 0;
				}
			}
		}

		if(state == 0)
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
		}
		else
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
		}
	}
}
