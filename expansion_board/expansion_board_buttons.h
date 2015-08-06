#ifndef _EXPANSION_BOARD_BUTTONS_H_
#define _EXPANSION_BOARD_BUTTONS_H_

#include <stdint.h>


typedef enum {	BUTTON_number_expansionBoard1 = 1, 	BUTTON_number_expansionBoard2 = 2, BUTTON_number_expansionBoard3 = 3, BUTTON_number_expansionBoard4 = 4,
				BUTTON_number_mainBoard1 = 10
} BUTTON_number;

void BUTTONS_init(void);

uint8_t BUTTONS_isPressed(BUTTON_number button);

#endif
