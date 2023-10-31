/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum 
{
	aguardando = 0,
	borda_detectada,
	bot_pressionado,
}tipo_botao;
typedef enum 
{
	desligado = 0,
	doisHz,
	quatroHz,
	ligado,
}tipo_led;

typedef enum
{
	false = 0,
	true = 1,
}tipo_bool;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t EXTI_interrupt = 0;
uint16_t TIM3_interrupt = 0;

uint16_t i = 0;

tipo_botao estadoBot = aguardando;
tipo_led estadoLed = desligado;
tipo_bool sinal_bot_pressionado = false;
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (estadoBot)
	  {
	  case aguardando:
		  break;
	  case borda_detectada:
		  break;
	  case bot_pressionado:
		  sinal_bot_pressionado = true;
		  i++;
		  estadoBot = aguardando;
		  break;
	  default:
		  break;
	  }
	  
	  if (sinal_bot_pressionado == true)
	  {
		  sinal_bot_pressionado = false;
		  estadoLed++;
		  if (estadoLed > 3)
			  estadoLed = 0;
	  }
	  
	  switch (estadoLed)
	  {
	  case desligado:
		  if (sinal_bot_pressionado) {
			  htim4.Instance->CCR2 = 0;
			  estadoLed = doisHz;
		  }
		  break;
	  case doisHz:
		  if (sinal_bot_pressionado) {
			  htim4.Instance->CCR2 = 5000 - 1;
			  estadoLed = quatroHz;
		  }
		  break;
	  case quatroHz:
		  if (sinal_bot_pressionado) {
			  htim4.Instance->CCR2 = 2500 - 1;
			  estadoLed = ligado;
		  }
		  break;
	  case ligado:
		  if (sinal_bot_pressionado) {
			  htim4.Instance->CCR2 = 10000 - 1;
			  estadoLed = desligado;
		  }
		  break;
	  default:
		  break;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)
	{
		EXTI->RTSR = 0; 						// Rising trigger selection register (circuito de detecção de borda)
		EXTI->IMR = 0; 							// Interrupt mask register
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);		// Clear pending bit from EXT0
		HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);	// Clear NVIC pending bit
		htim3.Instance->CNT = 0;				// Counter is zero
		HAL_TIM_Base_Start_IT(&htim3);			// Start timer interrupt (TIM3)
		EXTI_interrupt++;						// EXTI interrupt update counter
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Stop_IT(&htim3);				// Stop timer interrupt (TIM3)
	htim3.Instance->CNT = 0;					// Counter is zero
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))		// Check if the pin is still pressed
	{
		TIM3_interrupt++;						// TIM3 interrupt update counter
		HAL_GPIO_TogglePin(GPIOD, GREEN_Pin);	// Toggle Green LED
		estadoBot = bot_pressionado;
	}
	else
	{
		estadoBot = aguardando;
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);		// Clear pending bit from EXT0
	HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);		// Clear NVIC pending bit
	EXTI->RTSR = 1;								// Rising trigger selection register activated
	EXTI->IMR = 1;								// Interrupt mask register
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);				// Enable NVIC EXTI0 interrupt
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
