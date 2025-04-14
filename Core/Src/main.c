/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bmp2_config.h"
#include "string.h"
#include "pi.h"
#include "lcd.h"
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t  buffer[64];
uint8_t message[7];
_Bool transmiting = 0;
_Bool controler_on = 0;
int xd1 = 0;
float xd2;
struct lcd_disp disp;
//struct lcd_disp disp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
    	processMessage(message);
    	HAL_UART_Receive_IT(&huart3, message, 6);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM7)
  {
	  float measured = BMP2_ReadTemperature_degC(&bmp2dev);
	  xd2 = measured;
	  unsigned int tempVal = measured;
	  float tempFrac = measured - tempVal;
	  int8_t tempFracInt = trunc(tempFrac * 100);

	  if(transmiting == 1){
		  uint8_t len = sprintf((char*)buffer, "%d.%02d\n", tempVal, tempFracInt);
		  HAL_UART_Transmit(&huart3, buffer, len, 100);
	  }

 	  if(controler_on == 1)
 	  {
 		  float u = calculate_control(&pi, measured);
 	  	  u *= 1000;

 	  	  if(u>=0)
 	  	  {
 		  	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (int16_t)u);
 		  	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
 	  	  }
 	  	  else
 	  	  {
 	  		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
 	  		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int16_t)u * (-1));
 	  	  }
 	  	sprintf((char*)disp.s_line, "Meas:%4.2f CTR=1", measured);
 	  	lcd_display(&disp);

 	  }
 	  else
 	  {
 		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
 		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
 		 sprintf((char*)disp.s_line, "Meas:%4.2f CTR=0", measured);
 		lcd_display(&disp);
 	  }
  }
}

void processMessage(uint8_t *message)
{
	if(strcmp((char *)message, "uarton")==0)
	{
		transmiting = 1;

	}
	else if(strcmp((char *)message, "uartof")==0)
	{
		transmiting = 0;
	}
	else if(strcmp((char *)message, "ctrlon")==0)
	{
		controler_on = 1;
	}
	else if(strcmp((char *)message, "ctrlof")==0)
	{
		controler_on = 0;
	}
	 else if (strncmp((char *)message, "sp", 2) == 0)
	    {
	        char *numberStr = (char *)message + 2; // Skip "sp"
	        char *endPtr;
	        long intValue = strtol(numberStr, &endPtr, 10); // Parse as integer

	        // Validate that the conversion was successful and there is no trailing junk
	        if (endPtr != numberStr && *endPtr == '\0')
	        {
	            pi.setpoint = intValue / 100.0f; // Convert to float and divide by 10
	        }
	        sprintf((char*)disp.f_line, "Set:%4.2f [degC]", pi.setpoint);
	        lcd_display(&disp);
	    }
}

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
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  BMP2_Init(&bmp2dev);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_UART_Receive_IT(&huart3, message, 6);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  disp.addr = (0x27 << 1);
  disp.bl = true;
  disp.Timer = &htim6;
  lcd_init(&disp);
  sprintf((char*)disp.f_line, "Set:%4.2f [degC]", pi.setpoint);
  lcd_display(&disp);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
