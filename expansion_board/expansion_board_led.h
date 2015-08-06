#ifndef _EXPANSION_BOARD_LED_H_
#define _EXPANSION_BOARD_LED_H_

#include <stdint.h>

#define OLD_EXPANSION_BOARD		0

typedef enum {	LED_number_mainBoard1 = 1, LED_number_mainBoard2 = 2, LED_number_mainBoard3 = 3, LED_number_mainBoard4 = 4,
				#if (OLD_EXPANSION_BOARD == 1)
					LED_number_expansionBoard1_old = 20, LED_number_expansionBoard2_old = 21, LED_number_expansionBoard3_old = 22,
					LED_number_RGB_R_old = 70, LED_number_RGB_G_old = 71, LED_number_RGB_B_old = 72
				#else
					LED_number_expansionBoard1 = 10, LED_number_expansionBoard2 = 11, LED_number_expansionBoard3 = 12, LED_number_expansionBoard4 = 13,
					LED_number_RGB_R = 60, LED_number_RGB_G = 61, LED_number_RGB_B = 62
				#endif

} LED_number;

void LED_init(void);
void LED_turnOn(LED_number which);
void LED_turnOff(LED_number which);
void LED_toggle(LED_number which);
void LED_turnOnAll(void);
void LED_turnOffAll(void);
void LED_setRGBValue(LED_number ledRGBchannel, uint32_t valueInPercentage);

#endif
