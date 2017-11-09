/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <inttypes.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM9_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void read5(void);
void clock5(void);
uint32_t driver_LD5(void);
uint32_t convertGrams(uint32_t, uint32_t);
void convertG(int32_t);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t dataBuffer5;
char wbuffer[10];
char gbuffer[10];
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  //MX_SPI5_Init();
  MX_TIM9_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
  HAL_GPIO_TogglePin(BONUS_GPIO_Port, BONUS_Pin);
  //HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
  send_i(LCDON);
  send_i(TWOLINE);
  send_i(LCDCLR);
  lcdwait();
  /* USER CODE END 2 */
  int 	i 		= 0;

  wbuffer[0] = "0";
  wbuffer[1] = "x";

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t weightVal = 0;
  uint32_t sum = 0;
  int zeroi;
  int32_t zero;
  int32_t grams;
  HAL_GPIO_TogglePin(BONUS_GPIO_Port, BONUS_Pin);
  zeroi = 0;
  while (1)
  {
	  //HAL_Delay(500);
	  // output raw weight reading
	  send_i(LCDCLR);
	  chgline(LINE1);
	  lcdprint("SENS2: ");
	  lcdprint(wbuffer);
	  //HAL_SPI_Receive(&hspi5, &weightVal, 0x0003u, 200); // write 3 character
	  sum = 0;
	  for(i = 0; i < 32; ++i) {
		  weightVal = driver_LD5();
		  sum += weightVal;
	  }

	  // output interpreted weight
	  chgline(LINE2);
	  lcdprint("WEIGHT: ");
	  zeroi += 1;
	  if(zeroi == 10) {
		  HAL_GPIO_TogglePin(BONUS_GPIO_Port, BONUS_Pin);
		  zero = weightVal;
	  }
	  grams = convertGrams(weightVal, zero);
	  // averages the data
	  weightVal = sum >> 5;
	  convertW(weightVal);
	  convertG(grams);
  }
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

}

uint32_t convertGrams(uint32_t wread, uint32_t zero) {
	int32_t offs = wread - zero;
	int32_t conversion = 20;
	return (offs * conversion);
}

void convertG(int32_t gVal) {
	int i, digit;
	char c;
	int cbuf = gVal;
	for(i = 0; i < 7; ++i) {
			digit = cbuf % 10;
			switch(digit) {
				case 0: c = '0'; break;
				case 1: c = '1'; break;
				case 2: c = '2'; break;
				case 3: c = '3'; break;
				case 4: c = '4'; break;
				case 5: c = '5'; break;
				case 6: c = '6'; break;
				case 7: c = '7'; break;
				case 8: c = '8'; break;
				case 9: c = '9'; break;
				//case 0xA: c = 'A'; break;
				//case 0xB: c = 'B'; break;
				//case 0xC: c = 'C'; break;
				//case 0xD: c = 'D'; break;
				//case 0xE: c = 'E'; break;
				//case 0xF: c = 'F'; break;
				default: c = 'X';
			}
			gbuffer[2+6-i] = c;
			cbuf = cbuf >> 4;
		}
		gbuffer[ 9] = ' ';
		gbuffer[10] = 'g';
}

void convertW(int32_t wVal) {
	int i, digit;
	char c;
	int cbuf = wVal;
	for(i = 0; i < 6; ++i) {
		digit = cbuf % 16;
		switch(digit) {
			case 0: c = '0'; break;
			case 1: c = '1'; break;
			case 2: c = '2'; break;
			case 3: c = '3'; break;
			case 4: c = '4'; break;
			case 5: c = '5'; break;
			case 6: c = '6'; break;
			case 7: c = '7'; break;
			case 8: c = '8'; break;
			case 9: c = '9'; break;
			case 0xA: c = 'A'; break;
			case 0xB: c = 'B'; break;
			case 0xC: c = 'C'; break;
			case 0xD: c = 'D'; break;
			case 0xE: c = 'E'; break;
			case 0xF: c = 'F'; break;
			default: c = 'X';
		}
		wbuffer[2+5-i] = c;
		cbuf = cbuf >> 4;
	}
	wbuffer[0] = '0';
	wbuffer[1] = 'x';
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, F_RCLK_Pin|BONUS_Pin|LCD_E_Pin|LCD_RW_Pin
                          |LCD_RS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LD0_SPI5_CLK_Pin|BONUS_Pin
		  	  	  	  	  	  	  	  	  , GPIO_PIN_RESET);


  /*Configure GPIO pins : F_RCLK_Pin BONUS_Pin LCD_E_Pin LCD_RW_Pin
                           LCD_RS_Pin */
  GPIO_InitStruct.Pin = F_RCLK_Pin|BONUS_Pin|LCD_E_Pin|LCD_RW_Pin
                          |LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = LD0_SPI5_CLK_Pin|BONUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins :*/
  GPIO_InitStruct.Pin  = LD0_SPI5_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void clock5() {
	HAL_GPIO_WritePin(	LD0_SPI5_CLK_GPIO_Port,
						LD0_SPI5_CLK_Pin,
						GPIO_PIN_SET);
	HAL_GPIO_WritePin(	LD0_SPI5_CLK_GPIO_Port,
						LD0_SPI5_CLK_Pin,
						GPIO_PIN_RESET);
}

void read5() {
	GPIO_PinState bit  = HAL_GPIO_ReadPin(	LD0_SPI5_MISO_GPIO_Port,
											LD0_SPI5_MISO_Pin);
	dataBuffer5 	   = dataBuffer5 << 1;
	dataBuffer5       |= (bit == GPIO_PIN_SET) ? 1U : 0U;
}

void spinwaiting(uint8_t loops) {
	int i, j;
	for(i = 0; i < loops; ++i) {
		for(j = 0; j < 10; ++j) {}
	}
}

uint32_t driver_LD5() {
	// check for data ready
    GPIO_PinState ps;
    int timeOut = 0;
    do { ps = HAL_GPIO_ReadPin( LD0_SPI5_MISO_GPIO_Port,
    							LD0_SPI5_MISO_Pin);
    } while((ps == GPIO_PIN_SET) && (timeOut++ < 10000));
    if(timeOut >= 10000) { return(0); }

    HAL_GPIO_TogglePin(F_RCLK_GPIO_Port, F_RCLK_Pin);
    // shift 25 bits
    int bit;
 	dataBuffer5 = 0;
	HAL_GPIO_WritePin(	LD0_SPI5_CLK_GPIO_Port,
								LD0_SPI5_CLK_Pin,
								GPIO_PIN_RESET);
	//spinwaiting(2);
	for(bit = 0; bit < 25; ++bit) {
		//clock5();
		HAL_GPIO_WritePin(	LD0_SPI5_CLK_GPIO_Port,
							LD0_SPI5_CLK_Pin,
							GPIO_PIN_SET);
		//spinwaiting(1);
		read5();
		//spinwaiting(1);
		HAL_GPIO_WritePin(	LD0_SPI5_CLK_GPIO_Port,
									LD0_SPI5_CLK_Pin,
									GPIO_PIN_RESET);
		//spinwaiting(2);
	}
	//spinwaiting(2);

	// reset clock
	HAL_GPIO_WritePin(	LD0_SPI5_CLK_GPIO_Port,
								LD0_SPI5_CLK_Pin,
								GPIO_PIN_RESET);
    return(dataBuffer5);
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

/* USER CODE END 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/