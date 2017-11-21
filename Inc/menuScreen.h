#ifndef MENUSCREEN_H
#define MENUSCREEN_H
#include <inttypes.h>
#include "stm32f4xx_hal.h"


// LCD controller special registers 
#define MENUSCREEN_SWRST 		0xE2
#define MENUSCREEN_RMW_START	0xE0
#define MENUSCREEN_RMW_END		0xEE
#define MENUSCREEN_ONOFF_R		0xAE
#define	MENUSCREEN_STARTLINE_R	0xC0
#define MENUSCREEN_PAGE_ADDR_R	0xB0
#define MENUSCREEN_COL_ADDR_R	0x00
#define MENUSCREEN_INV_COL_R	0xA0
#define MENUSCREEN_STATIC_DR_R	0xA4
#define MENUSCREEN_DUTY_R		0xA8
#define MENUSCREEN_E_DELAY		0x02
//#define MENUSCREEN_6800
//#define MENUSCREEN_6800_RESET

// "public" function prototypes
void 	menuScreen_init();
uint8_t menuScreen_read_status_r();
void 	menuScreen_wait(uint8_t w);
void 	menuScreen_initVars();

// "private" function prototypes
void 	bothSides(char i);
void 	Writeright(char i); 
void 	Writeleft(char i); 
void 	Comright(char i);
void 	Comleft(char i);

// pin and port definitions definitons




#define F_D_I_Pin		F_A0_Pin 
#define F_E_Pin 		F_NRD_Pin
#define F_E_GPIO_Port 	F_NRD_GPIO_Port  
#define F_RW_Pin 		F_NWR_Pin 
#define F_RW_GPIO_Port 	F_NWR_GPIO_Port 
#define F_RS_Pin 		F_A0_Pin
#define F_RS_GPIO_Port	F_A0_GPIO_Port
/*
#define F_NRD_Pin 		F_E_Pin
#define F_NRD_GPIO_Port F_E_GPIO_Port  
#define F_NWR_Pin 		F_RW_Pin 
#define F_NWR_GPIO_Port F_RW_GPIO_Port 
#define F_A0_Pin 		F_RS_Pin
#define F_A0_GPIO_Port	F_RS_GPIO_Port


#define F_NRD_Pin 		GPIO_PIN_13
#define F_NRD_GPIO_Port GPIOC
#define F_A0_Pin 		GPIO_PIN_0
#define F_A0_GPIO_Port 	GPIOC
#define F_NWR_Pin 		GPIO_PIN_2
#define F_NWR_GPIO_Port	GPIOC

#define F_RCLK_Pin 			GPIO_PIN_3
#define F_RCLK_GPIO_Port 	GPIOC
#define SPI_CLK_Pin 		GPIO_PIN_5
#define SPI_CLK_GPIO_Port 	GPIOA
#define SPI_MISO_Pin 		GPIO_PIN_6
#define SPI_MISO_GPIO_Port 	GPIOA
#define SPI_MOSI_Pin 		GPIO_PIN_7
#define SPI_MOSI_GPIO_Port 	GPIOA
*/

// multiline macros
#define MENUSCREEN_Pulse_E_Pin ({\
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_SET  );\
	menuScreen_wait(2);											\
	HAL_GPIO_WritePin(F_E_GPIO_Port , F_E_Pin , GPIO_PIN_RESET);\
}) 

#define MENUSCREEN_Pulse_NRST_Pin ({\
	HAL_GPIO_WritePin(F_NRST_GPIO_Port, F_NRST_Pin, GPIO_PIN_SET);\
	menuScreen_wait(1);\
	HAL_GPIO_WritePin(F_NRST_GPIO_Port, F_NRST_Pin, GPIO_PIN_RESET  );\
})

#define MENUSCREEN_CtlPins_W_D ({\
	HAL_GPIO_WritePin(F_RW_GPIO_Port, F_RW_Pin, GPIO_PIN_RESET);\
	HAL_GPIO_WritePin(F_RS_GPIO_Port, F_RS_Pin, GPIO_PIN_SET  );\
})

#define MENUSCREEN_CtlPins_W_I ({\
	HAL_GPIO_WritePin(F_RW_GPIO_Port, F_RW_Pin, GPIO_PIN_RESET);\
	HAL_GPIO_WritePin(F_RS_GPIO_Port, F_RS_Pin, GPIO_PIN_RESET);\
})

#endif//MENUSCREEN_H
