#ifndef __BASIC_H
#define __BASIC_H

// basic debug lcd commands
#define CURMOV  0xFE
#define LCDON   0x0F
#define TWOLINE 0x38
#define LCDCLR  0x01
#define LINE1   0x80
#define LINE2   0xC0

// basic debug lcd function
void shiftout(char x);
void lcdwait();
void send_byte(char x);
void send_i(char x);
void chgline(char x);
void print_c(char x);
void lcdprint(char str[]);
void spinwaiting(char loops);

// test code
void RGB_on_off(TIM_HandleTypeDef * htim, uint32_t TIM_CHANNEL_i);
void RGB_LED_Test_Loop(void);
void ControlSignals_Test_Loop();
void GPIO_Pin_on_off(uint32_t GPIOp, uint32_t GPIOpin);

// pin remappings
#define LCD_RS_Pin F_RS_Pin
#define LCD_RS_GPIO_Port F_RS_GPIO_Port
#define LCD_E_Pin F_E_Pin
#define LCD_E_GPIO_Port F_E_GPIO_Port
#define LCD_RW_Pin F_RW_Pin
#define LCD_RW_GPIO_Port F_RW_GPIO_Port




#endif//__BASIC_H
