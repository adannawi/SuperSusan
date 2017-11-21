#include "main.h"
#include "stm32f4xx_hal.h"

#include "buttons.h"
#include <inttypes.h>


//___________________________________________________________________
//
// external global variables
//___________________________________________________________________
//
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim9;

//___________________________________________________________________
//
// global buttons variables
//___________________________________________________________________
//
enum DEBOUNCE select_isr;
uint32_t select_db;
uint32_t select_prev;
uint32_t select_fresh;
uint8_t	 select_on;

enum DEBOUNCE back_isr;
uint32_t back_db;
uint32_t back_prev;
uint32_t back_fresh;
uint8_t	 back_on;

uint8_t menu_index;
uint8_t debounce;



//___________________________________________________________________
//
//	buttons.c
//___________________________________________________________________
//
void check_buttons() {
#ifdef DEBOUNCE_CLIFFSWAY
	// CHECK ROTSW, set on if debounced
	if(select_isr == DEBOUNCE_START) {
		select_isr = DEBOUNCE_PENDING;
		select_prev = HAL_GPIO_ReadPin(F_ROTSW_GPIO_Port, F_ROTSW_Pin);
		debounce = 1;
	}
	// 
	else if(select_isr == DEBOUNCE_SET) {
		select_isr = DEBOUNCE_STOP;
		//select_db = 0;
		select_fresh = HAL_GPIO_ReadPin(F_ROTSW_GPIO_Port, F_ROTSW_Pin);

		if (select_fresh == select_prev) {
			select_on = 1;
		}
	}

	// CHECK ROTSW, set on if debounced
	if(back_isr == DEBOUNCE_START) {
		back_isr = DEBOUNCE_PENDING;
		back_prev = HAL_GPIO_ReadPin(F_BACK_GPIO_Port, F_BACK_Pin);
		debounce = 1;
	}
	// 
	else if(back_isr == DEBOUNCE_SET) {
		//back_db = 0;
		back_isr= DEBOUNCE_STOP;
		back_fresh = HAL_GPIO_ReadPin(F_BACK_GPIO_Port, F_BACK_Pin);
		if (back_fresh == back_prev) {	
			back_on = 1;
		}
	}
	if(debounce) {
		debounce = 0;
		HAL_TIM_Base_Start_IT(&htim6);
	}
#else//DEBOUNCE_ZIADSWAY


#endif
}

void eval_buttons() {
	// check for button down
	if(select_on || back_on) {
		// process button states
		eval_select(select_on);
		eval_back(back_on);

		// clear buttons after processing
		select_on 	= 0;
		back_on 	= 0;
	}
}


void eval_select(uint8_t on) {
	test_select(on);
}

void eval_back(uint8_t on) {
	test_back(on);
}


void test_back(uint8_t btndn) {
	if(btndn) {
		switch(menu_index) {
			case 0: HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1); break;
			case 1: HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2); break;
			case 2: HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4); break;
		}
	}
}

void test_select(uint8_t btndn) {
	if(btndn) {
		switch(menu_index) {
			case 0: HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); break;
			case 1: HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); break;
			case 2: HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4); break;
		}
	}
}

void meta_test_menu() {
	int j;
	for(j = 0; j < 3; ++j) {
		menu_index = j;
		test_select(1);
		HAL_Delay(500);
		test_back(1);
		HAL_Delay(500);
	}
}
