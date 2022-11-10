/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GFX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SNAKE_SIZE 8
#define PLATFORM_SIZE_X 128
#define PLATFORM_SIZE_Y 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint8_t dir1=0;
	uint8_t dir2=0;
	uint16_t adcdata[2];
	uint8_t act_size=1;
	int16_t xp=3;
	int16_t yp=1;
	int16_t x=5;
	int16_t y=35;
	int16_t size=30;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_init();
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcdata, 2);

  //GFX_draw_string(3, 25, (unsigned char *)"***** ***", WHITE, BLACK, 2, 2);
  //SSD1306_draw_pixel(xp, yp, WHITE);

  //SSD1306_display_repaint();
  SSD1306_display_clear();
  Draw_Block(x, y);
  SSD1306_display_repaint();
  //SSD1306_draw_fast_vline(10, 10, 20,  WHITE);
  //SSD1306_draw_fast_hline(10, 10, 20,  WHITE);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Jump();
	  	if(y<1){
	  		SSD1306_display_clear();
	  		GFX_draw_string(3, 25, (unsigned char *)"GAME OVER", WHITE, BLACK, 2, 2);
	  		SSD1306_display_repaint();
	  		HAL_Delay(1000);

	  	}
	   HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Draw_Cursor(int16_t x, int16_t y, uint8_t size)
{
SSD1306_draw_fast_vline(x, y-size, size*2,  WHITE);
SSD1306_draw_fast_hline(x-size, y, size*2,  WHITE);
}

void Draw_Block(int16_t x, int16_t y)
{
for(int16_t i=x;i<x+SNAKE_SIZE;i++){
	SSD1306_draw_fast_vline(i, y, 8,  WHITE);
	}
}

void Drifting_X(int16_t *xp, int16_t *yp, uint8_t size, uint8_t *dir1, uint8_t *dir2)
{
	 SSD1306_display_clear();
		  Draw_Cursor(&xp, &yp, 8);
		  SSD1306_display_repaint();
		  if(dir1==0){
			  xp--;
			  if(xp>128){dir1=1;}

		  }
		  else {
			  xp--;
			  if(xp<1){dir1=0;}
		  }
		  if(dir2==0){
				  yp++;
				  if(yp>64){dir2=1;}

			  }
			  else {
				  yp--;
				  if(yp==0){dir2=0;}
			  }

}
void Move_and_ADD()
{
	SSD1306_display_clear();
	for(int i=0;i<act_size;i++){
		Draw_Block(x-i*SNAKE_SIZE, y);
		SSD1306_display_repaint();
	}
		   x++;
}
void Jump()
{
	SSD1306_display_clear();

	Draw_Block(x, y);
	SSD1306_display_repaint();
	y--;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_13){

		y=y+20;
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);



	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
