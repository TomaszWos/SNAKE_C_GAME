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
#include "rng.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GFX.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct wall {
	int16_t zero, x, y, h;
}wall;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIRD_SIZE 10
#define PLATFORM_SIZE_X 128
#define PLATFORM_SIZE_Y 64
#define STARTING_POINT_X 64
#define STARTING_POINT_Y 32

#define WALL_SIZE 10
#define GAP_SIZE 25
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
	int16_t bird_x=STARTING_POINT_X;
	int16_t bird_y=STARTING_POINT_Y;
	int16_t size=30;
	int16_t wal_x=0;
	int16_t wal_y=0;
	int16_t wal_h=15;

	wall object;
	wall object1;
	wall object2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
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
	object.x=45;
	object.y=0;
	object.h=20;
	object1.x=0;
	object1.y=0;
	object1.h=5;
	object2.x=90;
	object2.y=0;
	object2.h=32;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_init();
  HAL_RNG_Init(&hrng);
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcdata, 2);

  //GFX_draw_string(3, 25, (unsigned char *)"***** ***", WHITE, BLACK, 2, 2);
  //SSD1306_draw_pixel(xp, yp, WHITE);

  //SSD1306_display_repaint();
  SSD1306_display_clear();
  Draw_Block(bird_x, bird_y);
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
	  //Move_Wall();
	  Move_Wall2();
	  Move_Wall3();
	  Move_Wall4();



      Bird_Colision(object.x,object.y,object.h,WALL_SIZE);
      Bird_Colision(object1.x,object1.y,object1.h,WALL_SIZE);
      Bird_Colision(object2.x,object2.y,object2.h,WALL_SIZE);
      if(bird_y<1){
    	  Game_Over();
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

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

void Draw_Block(int16_t x, int16_t y,int16_t h, int16_t w)
{
for(int16_t i=x;i<x+w;i++){
	SSD1306_draw_fast_vline(i, y, h,  WHITE);
	}
}

void Move_Wall()
{
wal_x++;
if(wal_x>128){
	wal_x=0;
	wal_h=wal_h+5;
}
Draw_Block(wal_x, wal_y, wal_h, WALL_SIZE);
SSD1306_display_repaint();
}

void Move_Wall2()
{	uint8_t temp=0;
	if(object.x>128){
		object.x=0;
		temp=HAL_RNG_GetRandomNumber(&hrng);
		while(temp>40)
		{
			temp=temp/2;
		}
		object.h=temp;
	}
	object.x=object.x+1;
	Draw_Block(object.x, object.y, object.h, WALL_SIZE);
	Draw_Block(object.x, object.h+GAP_SIZE, 64-GAP_SIZE-object.h, WALL_SIZE);
	SSD1306_display_repaint();
}
void Move_Wall3()
{	uint8_t temp=0;
	if(object1.x>128){
	object1.x=0;
	temp=HAL_RNG_GetRandomNumber(&hrng);
			while(temp>40)
			{
				temp=temp/2;
			}
			object1.h=temp;
}
	object1.x=object1.x+1;
	Draw_Block(object1.x, object1.y, object1.h, WALL_SIZE);
	Draw_Block(object1.x, object1.h+GAP_SIZE, 64-GAP_SIZE-object1.h, WALL_SIZE);
	SSD1306_display_repaint();
}
void Move_Wall4()
{	uint8_t temp=0;
	if(object2.x>128){
	object2.x=0;
	temp=HAL_RNG_GetRandomNumber(&hrng);
			while(temp>40)
			{
				temp=temp/2;
			}
			object2.h=temp;
}
	object2.x=object2.x+1;
	Draw_Block(object2.x, object2.y, object2.h, WALL_SIZE);
	Draw_Block(object2.x, object2.h+GAP_SIZE, 64-GAP_SIZE-object2.h, WALL_SIZE);
	SSD1306_display_repaint();
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
void Bird_Colision(int16_t x, int16_t y,int16_t h, int16_t w)
{
	//if((x+w>=bird_x)&((y<=bird_y+BIRD_SIZE)|(y+h>=bird_y))){
	if((x+w>=bird_x)&&(x<=bird_x+BIRD_SIZE)&&((y+h>=bird_y)||(h+GAP_SIZE<=bird_y+BIRD_SIZE))){
		Game_Over();
	}
}
void Game_Over()
{

  		SSD1306_display_clear();
  		GFX_draw_string(3, 25, (unsigned char *)"GAME OVER", WHITE, BLACK, 2, 2);
  		SSD1306_display_repaint();
  		HAL_Delay(1000);

}

void Move_and_ADD()
{
	SSD1306_display_clear();
	for(int i=0;i<act_size;i++){
		Draw_Block(bird_x-i*BIRD_SIZE, bird_y, WALL_SIZE, 10);
		SSD1306_display_repaint();
	}
		   bird_x++;
}
void Jump()
{
	SSD1306_display_clear();

	Draw_Block(bird_x, bird_y,BIRD_SIZE,BIRD_SIZE);
	SSD1306_display_repaint();
	bird_y--;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_13){

		bird_y=bird_y+5;
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
