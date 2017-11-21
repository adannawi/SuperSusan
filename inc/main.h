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
#include <inttypes.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define F_CLK_COUNT_PERIOD 50
#define F_CLK_PRESCALER 160
#define RGB_LED_PRESCALER 5
#define RGB_LED_PERIOD 32
#define RGB_LED_DEFAULT_R 12
#define RGB_LED_DEFAULT_G 10
#define RGB_LED_DEFAULT_B 9
#define DEBOUNCE_PRESCALER 8000
#define DEBOUNCE_N_HALF_MS_WAIT 100

#define F_NWR_Pin GPIO_PIN_13
#define F_NWR_GPIO_Port GPIOC
#define F_NRST_Pin GPIO_PIN_14
#define F_NRST_GPIO_Port GPIOC
#define F_NCS1_Pin GPIO_PIN_15
#define F_NCS1_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define F_A0_Pin GPIO_PIN_0
#define F_A0_GPIO_Port GPIOC
#define F_NCS2_Pin GPIO_PIN_1
#define F_NCS2_GPIO_Port GPIOC
#define F_NRD_Pin GPIO_PIN_2
#define F_NRD_GPIO_Port GPIOC
#define F_RCLK_Pin GPIO_PIN_3
#define F_RCLK_GPIO_Port GPIOC
#define F_PWR_LED_R_Pin GPIO_PIN_0
#define F_PWR_LED_R_GPIO_Port GPIOA
#define F_WIFI_LED_B_Pin GPIO_PIN_1
#define F_WIFI_LED_B_GPIO_Port GPIOA
#define F_WIFI_LED_G_Pin GPIO_PIN_2
#define F_WIFI_LED_G_GPIO_Port GPIOA
#define F_PWR_LED_G_Pin GPIO_PIN_3
#define F_PWR_LED_G_GPIO_Port GPIOA
#define SPI_CLK_Pin GPIO_PIN_5
#define SPI_CLK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define F_WIFI_LED_R_Pin GPIO_PIN_4
#define F_WIFI_LED_R_GPIO_Port GPIOC
#define F_PWR_LED_B_Pin GPIO_PIN_5
#define F_PWR_LED_B_GPIO_Port GPIOC
#define LD0_SPI5_CLK_Pin GPIO_PIN_0
#define LD0_SPI5_CLK_GPIO_Port GPIOB
#define F_CS_DP_Pin GPIO_PIN_1
#define F_CS_DP_GPIO_Port GPIOB
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
#define F_ROTB_Pin GPIO_PIN_11
#define F_ROTB_GPIO_Port GPIOB
#define F_ROTB_EXTI_IRQn EXTI15_10_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define F_ROTSW_Pin GPIO_PIN_4
#define F_ROTSW_GPIO_Port GPIOB
#define F_ROTA_Pin GPIO_PIN_5
#define F_ROTA_GPIO_Port GPIOB
#define F_ROTA_EXTI_IRQn EXTI9_5_IRQn
#define F_BACK_Pin GPIO_PIN_8
#define F_BACK_GPIO_Port GPIOB
#define F_CLK_Pin GPIO_PIN_9
#define F_CLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
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
