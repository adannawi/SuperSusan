#include "main.h"
#include "stm32f4xx_hal.h"
#include "basic.h"
#include "menuScreen.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

/*
***********************************************************************
  spinwaiting: bogus processor wait
***********************************************************************
*/
void spinwaiting(char loops) {
	int i, j;
	for(i = 0; i < loops; ++i) {
		for(j = 0; j < 10; ++j) {}
	}
}

/*
***********************************************************************
  shiftout: Transmits the character x to external shift
            register using the SPI.  It should shift MSB first.

            MISO = PA7
            SCK  = PA5
***********************************************************************
*/
void shiftout(char x){
    int a;
    HAL_GPIO_WritePin(F_RCLK_GPIO_Port, F_RCLK_Pin, GPIO_PIN_RESET);
    // read something to see if transmit data register is empty
    HAL_SPI_Transmit(&hspi1, &x, 1, 200); // write 1 character
    // wait for at least 30 cycles for data to be written
    //HAL_Delay(1);
    spinwaiting(2);
    HAL_GPIO_WritePin(F_RCLK_GPIO_Port, F_RCLK_Pin, GPIO_PIN_SET);
    spinwaiting(1);
    HAL_GPIO_WritePin(F_RCLK_GPIO_Port, F_RCLK_Pin, GPIO_PIN_RESET);
}

/*
***********************************************************************
  lcdwait: Delay for approx 2 ms
***********************************************************************
*/
void lcdwait(){
    // function to delay for approx. 2 ms
    volatile int c;
    volatile int test = 0;
    //for (c = 0; c < 5000; c++){
    //    // loop to waste 2 ms - need to figure out timing
    //    test++;
    //}
    HAL_Delay(3);
}

/*
***********************************************************************
  send_byte: writes character x to the LCD
***********************************************************************
*/
void send_byte(char x){
    // shift out character
    // pulse LCD clock line low->high->low
    // wait 2 ms for LCD to process data
    //HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
    //spinwaiting(1);
    shiftout(x);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
    spinwaiting(2);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
    spinwaiting(2);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
    lcdwait();
}

/*
***********************************************************************
  send_i: Sends instruction byte x to LCD
***********************************************************************
*/
void send_i(char x){
    // set the register select line low (instruction data)
    // send byte

    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    spinwaiting(2);
    send_byte(x);  // send byte of instruction
}

/*
***********************************************************************
  chgline: Move LCD cursor to position x
  NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
***********************************************************************
*/
void chgline(char x){
    send_i(CURMOV);
    send_i(x);
}

/*
***********************************************************************
  print_c: Print (single) character x on LCD
***********************************************************************
*/

void print_c(char x){
    // set register select line high for character
    // write character to LCD

    // Register select high for character
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    send_byte(x);  // write the character
}

/*
***********************************************************************
  pmsglcd: print character string str[] on LCD
***********************************************************************
*/
void lcdprint(char str[]){
    int i = 0;
    while (str[i] != '\0'){
        print_c(str[i++]);
    }
}

/*
***********************************************************************
  RGB_on_off: blinks the LED
***********************************************************************
*/
void RGB_on_off(TIM_HandleTypeDef * htim, uint32_t TIM_CHANNEL_i) {
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_i);
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_i);
	HAL_Delay(500);

}
/*
***********************************************************************
  RGB_LED_Test_Loop: blinks all the face RGB LEDS
***********************************************************************
*/
void RGB_LED_Test_Loop() {
	RGB_on_off(&htim5, TIM_CHANNEL_1);
	RGB_on_off(&htim5, TIM_CHANNEL_2);
	RGB_on_off(&htim5, TIM_CHANNEL_3);
	RGB_on_off(&htim5, TIM_CHANNEL_4);

	RGB_on_off(&htim9, TIM_CHANNEL_1);
	RGB_on_off(&htim9, TIM_CHANNEL_2);
}


void GPIO_Pin_on_off(uint32_t GPIOp, uint32_t GPIOpin) {
	HAL_GPIO_TogglePin(GPIOp, GPIOpin);
	HAL_Delay(1);
	HAL_GPIO_TogglePin(GPIOp, GPIOpin);
	HAL_Delay(1);
}
/*
***********************************************************************
  RGB_LED_Test_Loop: blinks all the face RGB LEDS
***********************************************************************
*/
void ControlSignals_Test_Loop() {
	GPIO_Pin_on_off(F_E_GPIO_Port, F_E_Pin);
	GPIO_Pin_on_off(F_RS_GPIO_Port, F_RS_Pin);
	GPIO_Pin_on_off(F_RW_GPIO_Port, F_RW_Pin);
	GPIO_Pin_on_off(F_NCS1_GPIO_Port, F_NCS1_Pin);
	GPIO_Pin_on_off(F_NCS2_GPIO_Port, F_NCS2_Pin);
	GPIO_Pin_on_off(F_NRST_GPIO_Port, F_NRST_Pin);
}


