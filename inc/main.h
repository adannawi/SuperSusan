/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define F_NRD_Pin GPIO_PIN_13
#define F_NRD_GPIO_Port GPIOC
#define F_A0_Pin GPIO_PIN_0
#define F_A0_GPIO_Port GPIOC
#define F_NWR_Pin GPIO_PIN_2
#define F_NWR_GPIO_Port GPIOC
#define F_RCLK_Pin GPIO_PIN_3
#define F_RCLK_GPIO_Port GPIOC
#define SPI_CLK_Pin GPIO_PIN_5
#define SPI_CLK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define LD0_SPI5_CLK_Pin GPIO_PIN_0
#define LD0_SPI5_CLK_GPIO_Port GPIOB
#define BONUS_Pin GPIO_PIN_12
#define BONUS_GPIO_Port GPIOB
#define LD1_SPI2_CLK_Pin GPIO_PIN_13
#define LD1_SPI2_CLK_GPIO_Port GPIOB
#define LD1_SPI2_MISO_Pin GPIO_PIN_14
#define LD1_SPI2_MISO_GPIO_Port GPIOB
#define LD0_SPI5_MISO_Pin GPIO_PIN_12
#define LD0_SPI5_MISO_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LCD_RW_GPIO_Port	(F_NWR_GPIO_Port)
#define LCD_RW_Pin			(F_NWR_Pin)
#define LCD_E_GPIO_Port		(F_NRD_GPIO_Port)
#define LCD_E_Pin			(F_NRD_Pin)
#define LCD_RS_GPIO_Port	(F_A0_GPIO_Port)
#define LCD_RS_Pin			(F_A0_Pin)


/* USER CODE BEGIN Private defines */
#define CURMOV  0xFE
#define LCDON   0x0F
#define TWOLINE 0x38
#define LCDCLR  0x01
#define LINE1   0x80
#define LINE2   0xC0
void lcdprint(char str[]);
void print_c(char x);
void chgline(char x);
void send_i(char x);
void send_byte(char x);
void lcdwait();
void shiftout(char x);

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
