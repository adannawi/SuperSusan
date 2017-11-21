#include "menuScreen.h"
#include "main.h"
#include <inttypes.h>
#include "basic.h"


/* Global Varible Declarations */
uint8_t 	menuScreen_period_ellapsed;
uint32_t	menuScreen_period_count;









/*
***********************************************************************
  spinwaiting: bogus processor wait
***********************************************************************
*/
void menuScreen_rst() {
#ifdef MENUSCREEN_6800_RESET
	// interface like MOTOROLA6800
	HAL_GPIO_WritePin(F_NRST_GPIO_Port, F_NRST_Pin, GPIO_PIN_SET  );
	menuScreen_wait(1);
	HAL_GPIO_WritePin(F_NRST_GPIO_Port, F_NRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(F_NRST_GPIO_Port, F_NRST_Pin, GPIO_PIN_SET  );
#else
	// interface like INTEL8080
	HAL_GPIO_WritePin(F_NRST_GPIO_Port, F_NRST_Pin, GPIO_PIN_RESET  );
	menuScreen_wait(1);
	HAL_GPIO_WritePin(F_NRST_GPIO_Port, F_NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(F_NRST_GPIO_Port, F_NRST_Pin, GPIO_PIN_RESET  );
#endif
	HAL_Delay(10);
}

/*
***********************************************************************
  spinwaiting: bogus processor wait
***********************************************************************
*/
void menuScreen_wait(uint8_t w) {
	uint32_t t0 = menuScreen_period_count;
	int t = 0;
	while(t++ < w) {
		spinwaiting(4);
	}
}

void menuScreen_initVars(){
/*
	HAL_GPIO_WritePin(F_NCS2_GPIO_Port, F_NCS2_Pin, GPIO_PIN_SET); //NCS2 = 1;
	HAL_GPIO_WritePin(F_NCS1_GPIO_Port, F_NCS1_Pin, GPIO_PIN_SET); //NCS1 = 1;

	HAL_GPIO_WritePin(F_RW_GPIO_Port, F_RW_Pin, GPIO_PIN_SET); // RW = 1 
	HAL_GPIO_WritePin(F_RS_GPIO_Port, F_RS_Pin, GPIO_PIN_RESET); // RS = 0
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_RESET); // E = 0
*/
	// P3 = 0; 	;)
	HAL_GPIO_WritePin(F_NCS2_GPIO_Port, F_NCS2_Pin, GPIO_PIN_RESET); //NCS2 = 1;
	HAL_GPIO_WritePin(F_NCS1_GPIO_Port, F_NCS1_Pin, GPIO_PIN_RESET); //NCS1 = 1;

	HAL_GPIO_WritePin(F_RW_GPIO_Port, F_RW_Pin, GPIO_PIN_RESET); // RW = 1 
	HAL_GPIO_WritePin(F_RS_GPIO_Port, F_RS_Pin, GPIO_PIN_RESET); // RS = 0
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_RESET); // E = 0

}

/*****************************************/
void Comleft(char i) {
#ifdef MENUSCREEN_6800
	// start operation
	HAL_GPIO_WritePin(F_NCS2_GPIO_Port, F_NCS2_Pin, GPIO_PIN_RESET);//nCS2 = 0;

	shiftout(i);			//P1 = i;
	//MENUSCREEN_CtlPins_W_I; //R_W = 0, D_I = 0;
	HAL_GPIO_WritePin(F_RW_GPIO_Port, F_RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(F_RS_GPIO_Port, F_RS_Pin, GPIO_PIN_RESET);
	//menuScreen_wait(1);
	//MENUSCREEN_Pulse_E_Pin;	//E = 1; wait; E = 0;
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_SET  );
	HAL_Delay(MENUSCREEN_E_DELAY);
	//menuScreen_wait(MENUSCREEN_E_DELAY);
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_RESET);

	// finish operation
	HAL_GPIO_WritePin(F_NCS2_GPIO_Port, F_NCS2_Pin, GPIO_PIN_SET);//nCS2 = 1;
#else// MENUSCREEN_8080
	/*
	 P1 = i;
	 R_W = 0;
	 D_I = 1;
	 E1 = 1;
	 delay(2);
	 E1 = 0;
	 */
	shiftout(i);
	HAL_GPIO_WritePin(GPIOC, F_NCS1_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_A0_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_NWR_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_E_Pin	, GPIO_PIN_SET   );
	HAL_Delay(MENUSCREEN_E_DELAY);
	HAL_GPIO_WritePin(GPIOC, F_E_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_NWR_Pin	, GPIO_PIN_SET   );
	HAL_GPIO_WritePin(GPIOC, F_NCS1_Pin	, GPIO_PIN_SET 	 );

#endif
}

void Comright(char i) {
#ifdef MENUSCREEN_6800
	HAL_GPIO_WritePin(F_NCS1_GPIO_Port, F_NCS1_Pin, GPIO_PIN_RESET);//nCS2 = 0;
	shiftout(i);	

	//MENUSCREEN_CtlPins_W_I; //R_W = 0, D_I = 0;
	HAL_GPIO_WritePin(F_RW_GPIO_Port, F_RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(F_RS_GPIO_Port, F_RS_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_SET  );
	HAL_Delay(MENUSCREEN_E_DELAY);
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_RESET);


	// finish operation
	HAL_GPIO_WritePin(F_NCS1_GPIO_Port, F_NCS1_Pin, GPIO_PIN_SET);//nCS2 = 1;
#else// MENUSCREEN_8080
	/*
	 P1 = i;
	 R_W = 0;
	 D_I = 1;
	 E1 = 1;
	 delay(2);
	 E1 = 0;
	 */
	shiftout(i);
	HAL_GPIO_WritePin(GPIOC, F_NCS2_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_A0_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_NWR_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_E_Pin	, GPIO_PIN_SET   );
	HAL_Delay(MENUSCREEN_E_DELAY);
	HAL_GPIO_WritePin(GPIOC, F_E_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_NWR_Pin	, GPIO_PIN_SET );
	HAL_GPIO_WritePin(GPIOC, F_NCS2_Pin	, GPIO_PIN_SET 	 );

#endif
}

void Writeleft(char i) {
#ifdef MENUSCREEN_6800
	// start operation
	HAL_GPIO_WritePin(F_NCS2_GPIO_Port, F_NCS2_Pin, GPIO_PIN_RESET);//nCS2 = 0;

	shiftout(i);			//P1 = i;
	//MENUSCREEN_CtlPins_W_D;	//R_W = 0, D_I = 1;
	HAL_GPIO_WritePin(F_RW_GPIO_Port, F_RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(F_RS_GPIO_Port, F_RS_Pin, GPIO_PIN_SET  );
	//menuScreen_wait(MENUSCREEN_E_DELAY);
	//MENUSCREEN_Pulse_E_Pin;	//E = 1; wait; E = 0;
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_SET  );
	HAL_Delay(MENUSCREEN_E_DELAY);
	//menuScreen_wait(MENUSCREEN_E_DELAY);
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_RESET);

	// finish operation
	HAL_GPIO_WritePin(F_NCS2_GPIO_Port, F_NCS2_Pin, GPIO_PIN_SET);//nCS2 = 1;
#else// MENUSCREEN_8080
	/*
	 P1 = i;
	 R_W = 0;
	 D_I = 1;
	 E1 = 1;
	 delay(2);
	 E1 = 0;
	 */
	shiftout(i);
	HAL_GPIO_WritePin(GPIOC, F_NCS1_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_A0_Pin	, GPIO_PIN_SET   );
	HAL_GPIO_WritePin(GPIOC, F_NWR_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_E_Pin	, GPIO_PIN_SET   );
	HAL_Delay(MENUSCREEN_E_DELAY);
	HAL_GPIO_WritePin(GPIOC, F_E_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_NWR_Pin	, GPIO_PIN_SET );
	HAL_GPIO_WritePin(GPIOC, F_NCS1_Pin	, GPIO_PIN_SET 	 );

#endif
}

void Writeright(char i) {
#ifdef MENUSCREEN_6800
	// start operation
	HAL_GPIO_WritePin(F_NCS1_GPIO_Port, F_NCS1_Pin, GPIO_PIN_RESET);//nCS2 = 0;

	shiftout(i);			//P1 = i;
	//MENUSCREEN_CtlPins_W_D;	//R_W = 0, D_I = 1;
	HAL_GPIO_WritePin(F_RW_GPIO_Port, F_RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(F_RS_GPIO_Port, F_RS_Pin, GPIO_PIN_SET  );
	//menuScreen_wait(MENUSCREEN_E_DELAY);

	//MENUSCREEN_Pulse_E_Pin;	//E = 1; wait; E = 0;
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_SET  );
	HAL_Delay(MENUSCREEN_E_DELAY);
	//menuScreen_wait(MENUSCREEN_E_DELAY);
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_RESET);

	// finish operation
	HAL_GPIO_WritePin(F_NCS1_GPIO_Port, F_NCS1_Pin, GPIO_PIN_SET);//nCS2 = 1;

#else// MENUSCREEN_8080
	/*
	 P1 = i;
	 R_W = 0;
	 D_I = 1;
	 E1 = 1;
	 delay(2);
	 E1 = 0;
	 */
	shiftout(i);
	HAL_GPIO_WritePin(GPIOC, F_NCS2_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_A0_Pin	, GPIO_PIN_SET   );
	HAL_GPIO_WritePin(GPIOC, F_NWR_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_E_Pin	, GPIO_PIN_SET   );
	HAL_Delay(MENUSCREEN_E_DELAY);
	HAL_GPIO_WritePin(GPIOC, F_E_Pin	, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOC, F_NWR_Pin	, GPIO_PIN_SET );
	HAL_GPIO_WritePin(GPIOC, F_NCS2_Pin	, GPIO_PIN_SET 	 );

#endif
}

/*****************************************/
void bothSides(char i) {
	Comleft(i);
	Comright(i);
}

/*****************************************/
void menuScreen_init() {
	// clear data bus
	shiftout(0);//P1 = 0;

	//command port? unsure yet
	//P3 = 0;
	
	// wiggle reset, motorolla 6800 mode
	menuScreen_rst();

	// control lines
	HAL_GPIO_WritePin(F_RS_GPIO_Port	, F_RS_Pin 		, GPIO_PIN_RESET);//D_I = 0;
	HAL_GPIO_WritePin(F_E_GPIO_Port		, F_E_Pin		, GPIO_PIN_SET  );//E = 1;
	HAL_GPIO_WritePin(F_NCS1_GPIO_Port	, F_NCS1_Pin	, GPIO_PIN_RESET);//CS1 = 0;
	HAL_GPIO_WritePin(F_NCS2_GPIO_Port	, F_NCS2_Pin	, GPIO_PIN_RESET);//CS2 = 0;
	HAL_GPIO_WritePin(F_RW_GPIO_Port	, F_RW_Pin		, GPIO_PIN_SET  );//R_W = 1;

	// init sequence	
	bothSides(MENUSCREEN_SWRST);
	//menuScreen_wait(10);
	HAL_Delay(100);
	bothSides(MENUSCREEN_RMW_START);
	bothSides(MENUSCREEN_STATIC_DR_R	+ 0b0);			//do not turn on static drive mode
	bothSides(MENUSCREEN_DUTY_R			+ 0b1);			//"1/32" display duty selected
	bothSides(MENUSCREEN_INV_COL_R		+ 0b0);			//do not invert columns
	bothSides(MENUSCREEN_RMW_END);
	
	bothSides(MENUSCREEN_STARTLINE_R	+ 0b00000);		//start line is 0
	bothSides(MENUSCREEN_ONOFF_R 		+ 0b1);			//screen is on
}
/*****************************************/	
