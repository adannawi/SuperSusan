#ifndef __BUTTONS_H
#define __BUTTONS_H

#include <inttypes.h>

enum DEBOUNCE {
	DEBOUNCE_STOP = 0, 
	DEBOUNCE_START, 
	DEBOUNCE_PENDING,
	DEBOUNCE_SET
};

void check_buttons(void);
void eval_select(uint8_t on);
void eval_back(uint8_t on);
void eval_buttons(void);

void test_select(uint8_t btndn);
void test_back(uint8_t btndn);

void meta_test_menu();


#define DEBOUNCE_CLIFFSWAY



#endif//__BUTTONS_H