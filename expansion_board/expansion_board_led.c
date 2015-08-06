#include "expansion_board_led.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#define PWM_PERIOD_RAW			2000
#define PWM_MULTIPLIER			PWM_PERIOD_RAW/100

static inline void LED_initLedsOnMainBoard(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static inline void LED_initLedsOnExpansionBoard(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static inline void LED_initLedsOnExpansionBoard_old(void)
{
	// leds on expansion board

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static inline void LED_initLedRGB(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	uint16_t TIM_Period = PWM_PERIOD_RAW;

	TIM_TimeBaseStructure.TIM_Prescaler = 42-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM_Period;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	uint32_t CCR_ValRed = 0;
	uint32_t CCR_ValGreen = 0;
	uint32_t CCR_ValBlue = 0;

	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = CCR_ValRed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = CCR_ValGreen;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = CCR_ValBlue;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
}

static inline void LED_initLedRGB_old(void)
{
	// rgb led on expansion board
	// green
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// blue
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// red
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	uint16_t TIM_Period = PWM_PERIOD_RAW;

	TIM_TimeBaseStructure.TIM_Prescaler = 42-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM_Period;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	// not needed (only for TIM1 and TIM8) TIM_TimeBaseStructure.TIM_RepetitionCounter =

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	uint32_t CCR_Val1 = TIM_Period>>3;	// 1/8
	uint32_t CCR_Val2 = TIM_Period>>2;	// 1/4
	uint32_t CCR_Val3 = TIM_Period>>1;	// 1/2

	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = CCR_Val1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = CCR_Val2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = CCR_Val3;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
}

void LED_init(void)
{
	LED_initLedsOnMainBoard();
	#if (OLD_EXPANSION_BOARD == 1)
		LED_initLedsOnExpansionBoard_old();
		LED_initLedRGB_old();
	#else
		LED_initLedsOnExpansionBoard();
		LED_initLedRGB();
	#endif
}

static void LED_unhandledCase(void)
{
	for(;;)
	{
		asm("nop");
	}
}

void LED_turnOn(LED_number which)
{
	switch(which)
	{
		case LED_number_mainBoard1:
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			break;
		case LED_number_mainBoard2:
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
			break;
		case LED_number_mainBoard3:
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
			break;
		case LED_number_mainBoard4:
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
			break;
		#if (OLD_EXPANSION_BOARD == 1)
				case LED_number_expansionBoard1_old:
					GPIO_SetBits(GPIOE, GPIO_Pin_4);
					break;
				case LED_number_expansionBoard2_old:
					GPIO_SetBits(GPIOE, GPIO_Pin_6);
					break;
				case LED_number_expansionBoard3_old:
					GPIO_SetBits(GPIOC, GPIO_Pin_13);
					break;
				case LED_number_expansionBoard4:
					GPIO_SetBits(GPIOC, GPIO_Pin_13);
					break;
				case LED_number_RGB_R_old:
				case LED_number_RGB_G_old:
				case LED_number_RGB_B_old:
					LED_setRGBValue(which, 100);
					break;
		#else
				case LED_number_expansionBoard1:
					GPIO_SetBits(GPIOD, GPIO_Pin_0);
					break;
				case LED_number_expansionBoard2:
					GPIO_SetBits(GPIOD, GPIO_Pin_1);
					break;
				case LED_number_expansionBoard3:
					GPIO_SetBits(GPIOD, GPIO_Pin_2);
					break;
				case LED_number_expansionBoard4:
					GPIO_SetBits(GPIOD, GPIO_Pin_3);
					break;
				case LED_number_RGB_R:
				case LED_number_RGB_G:
				case LED_number_RGB_B:
					LED_setRGBValue(which, 100);
					break;
		#endif

		default:
			LED_unhandledCase();
			break;
	}
}

void LED_turnOff(LED_number which)
{
	switch(which)
	{
		case LED_number_mainBoard1:
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			break;
		case LED_number_mainBoard2:
			GPIO_ResetBits(GPIOD, GPIO_Pin_13);
			break;
		case LED_number_mainBoard3:
			GPIO_ResetBits(GPIOD, GPIO_Pin_14);
			break;
		case LED_number_mainBoard4:
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			break;
		#if (OLD_EXPANSION_BOARD == 1)
				case LED_number_expansionBoard1_old:
					GPIO_ResetBits(GPIOE, GPIO_Pin_4);
					break;
				case LED_number_expansionBoard2_old:
					GPIO_ResetBits(GPIOE, GPIO_Pin_6);
					break;
				case LED_number_expansionBoard3_old:
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);
					break;
				case LED_number_expansionBoard4:
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);
					break;
				case LED_number_RGB_R_old:
				case LED_number_RGB_G_old:
				case LED_number_RGB_B_old:
					LED_setRGBValue(which, 100);
					break;
		#else
				case LED_number_expansionBoard1:
					GPIO_ResetBits(GPIOD, GPIO_Pin_0);
					break;
				case LED_number_expansionBoard2:
					GPIO_ResetBits(GPIOD, GPIO_Pin_1);
					break;
				case LED_number_expansionBoard3:
					GPIO_ResetBits(GPIOD, GPIO_Pin_2);
					break;
				case LED_number_expansionBoard4:
					GPIO_ResetBits(GPIOD, GPIO_Pin_3);
					break;
				case LED_number_RGB_R:
				case LED_number_RGB_G:
				case LED_number_RGB_B:
					LED_setRGBValue(which, 100);
					break;
		#endif

		default:
			LED_unhandledCase();
			break;
	}
}

void LED_toggle(LED_number which)
{
	switch(which)
	{
		case LED_number_mainBoard1:
			GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
			break;
		case LED_number_mainBoard2:
			GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
			break;
		case LED_number_mainBoard3:
			GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
			break;
		case LED_number_mainBoard4:
			GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
			break;
		#if (OLD_EXPANSION_BOARD == 1)
				case LED_number_expansionBoard1_old:
					GPIO_ToggleBits(GPIOE, GPIO_Pin_4);
					break;
				case LED_number_expansionBoard2_old:
					GPIO_ToggleBits(GPIOE, GPIO_Pin_6);
					break;
				case LED_number_expansionBoard3_old:
					GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
					break;
				case LED_number_expansionBoard4:
					GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
					break;
				case LED_number_RGB_R_old:
				case LED_number_RGB_G_old:
				case LED_number_RGB_B_old:
					LED_setRGBValue(which, 100);
					break;
		#else
				case LED_number_expansionBoard1:
					GPIO_ToggleBits(GPIOD, GPIO_Pin_0);
					break;
				case LED_number_expansionBoard2:
					GPIO_ToggleBits(GPIOD, GPIO_Pin_1);
					break;
				case LED_number_expansionBoard3:
					GPIO_ToggleBits(GPIOD, GPIO_Pin_2);
					break;
				case LED_number_expansionBoard4:
					GPIO_ToggleBits(GPIOD, GPIO_Pin_3);
					break;
				case LED_number_RGB_R:
				case LED_number_RGB_G:
				case LED_number_RGB_B:
					LED_setRGBValue(which, 50);
					break;
		#endif

		default:
			LED_unhandledCase();
			break;
	}
}


void LED_setRGBValue(LED_number ledRGBchannel, uint32_t valueInPercentage)
{
	// convert form 0-100 to 0-2000
	uint32_t rawValue = valueInPercentage * PWM_MULTIPLIER;

	// TODO: add assert for input parameters (only few values from enums are allowed)
	switch(ledRGBchannel)
	{

		#if (OLD_EXPANSION_BOARD == 1)
			case LED_number_RGB_B_old:
				TIM3->CCR3 = rawValue;
				break;
			case LED_number_RGB_R_old:
				TIM3->CCR2 = rawValue;
				break;
			case LED_number_RGB_G_old:
				TIM3->CCR1 = rawValue;
				break;
		#else
			case LED_number_RGB_R:
				TIM3->CCR1 = rawValue;
				break;
			case LED_number_RGB_G:
				TIM3->CCR3 = rawValue;
				break;
			case LED_number_RGB_B:
				TIM3->CCR4 = rawValue;
				break;
		#endif

		default:
			LED_unhandledCase();
			break;
	}
}

void LED_turnOnAll(void)
{
	GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	#if (OLD_EXPANSION_BOARD == 1)
		GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_6);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	#else
		GPIO_SetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
		LED_setRGBValue(LED_number_RGB_R, 100);
		LED_setRGBValue(LED_number_RGB_G, 100);
		LED_setRGBValue(LED_number_RGB_B, 100);
	#endif
}

void LED_turnOffAll(void)
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	#if (OLD_EXPANSION_BOARD == 1)
		GPIO_SetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_6);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	#else
		GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
		LED_setRGBValue(LED_number_RGB_R, 0);
		LED_setRGBValue(LED_number_RGB_G, 0);
		LED_setRGBValue(LED_number_RGB_B, 0);
	#endif
}

