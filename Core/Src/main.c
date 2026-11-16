/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "hx711.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
HX711_t loadcell = {0};
volatile float measured_weight_lb = 0.0f;
char uart_buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define GRAVITY 9.80665f

// Transmit text over UART.
static void Serial_Print(const char *message)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)message,
                      strlen(message),
                      HAL_MAX_DELAY);
}

// Read a line from the serial terminal.
// The user presses Enter to submit it.
static void Serial_ReadLine(char *buffer, size_t size)
{
    size_t index = 0;
    uint8_t ch;

    if (size == 0)
        return;

    while (1)
    {
        if (HAL_UART_Receive(&huart2,
                             &ch, 1,
                             HAL_MAX_DELAY) != HAL_OK)
            continue;

        // Enter pressed.
        if (ch == '\r' || ch == '\n')
        {
            // Ignore leftover line endings.
            if (index == 0)
                continue;

            buffer[index] = '\0';
            Serial_Print("\r\n");
            return;
        }

        // Store and echo the character.
        if (index < size - 1)
        {
            buffer[index++] = (char)ch;
            HAL_UART_Transmit(&huart2,
                              &ch, 1, 100);
        }
    }
}

// Display a weight with 2 decimal places.
// Does not require floating-point printf support.
static void Serial_PrintWeight(float pounds)
{
    long hundredths =
        (long)(pounds * 100.0f +
              (pounds >= 0.0f ? 0.5f : -0.5f));

    long magnitude = hundredths < 0
                   ? -hundredths : hundredths;

    snprintf(uart_buffer, sizeof(uart_buffer),
             "Weight: %s%ld.%02ld lb\r\n",
             hundredths < 0 ? "-" : "",
             magnitude / 100,
             magnitude % 100);

    Serial_Print(uart_buffer);
}

// Interactive calibration routine.
static bool Calibrate_LoadCell(void)
{
    char input[40];
    float known_weight_lb;

    Serial_Print("\r\n");
    Serial_Print("============================\r\n");
    Serial_Print(" HX711 CALIBRATION\r\n");
    Serial_Print("============================\r\n");

    Serial_Print("\r\nSTEP 1: TARE\r\n");
    Serial_Print("Remove all external loads.\r\n");
    Serial_Print("Type t and press Enter.\r\n");

    while (1)
    {
        Serial_ReadLine(input, sizeof(input));

        if (strcmp(input, "t") == 0)
            break;

        Serial_Print("Please enter t to continue.\r\n");
    }

    Serial_Print("Taring... please wait.\r\n");

    if (!HX711_Tare(&loadcell))
    {
        Serial_Print("ERROR: Tare failed.\r\n");
        return false;
    }

    Serial_Print("Tare completed!\r\n");

    Serial_Print("\r\nSTEP 2: CALIBRATION\r\n");
    Serial_Print("Place a known weight on the load cell.\r\n");
    Serial_Print("Enter its weight in lb (e.g. 5).\r\n");
    Serial_Print("Press Enter when stable.\r\n");


    while (1)
    {
        char *end;

        Serial_ReadLine(input, sizeof(input));

        known_weight_lb = strtof(input, &end);

        if (end != input &&
            *end == '\0' &&
            known_weight_lb > 0.0f &&
            known_weight_lb <= 1000000.0f)
        {
            break;
        }

        Serial_Print("Invalid weight. Try again.\r\n");
    }

    Serial_Print("Calibrating... please wait.\r\n");

    if (!HX711_Calibrate(&loadcell, known_weight_lb))
    {
        Serial_Print("ERROR: Calibration failed.\r\n");
        return false;
    }

    Serial_Print("\r\nCalibration completed!\r\n");
    Serial_PrintWeight(known_weight_lb);

    Serial_Print("\r\nCommands:\r\n");
    Serial_Print("t = Tare\r\n");
    Serial_Print("r = Recalibrate\r\n");

    return true;
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
      Error_Handler();

  // Allow the HX711 to stabilize.
  HAL_Delay(2000);

  Serial_Print("\r\nSTM32 HX711 SYSTEM STARTED\r\n");

  while (!Calibrate_LoadCell())
  {
      Serial_Print("Calibration failed. Retrying...\r\n");
      HAL_Delay(1000);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      float weight_lb;
      uint8_t command;

      // Read weight directly in pounds.
      if (HX711_GetWeight(&loadcell, &weight_lb))
      {
          measured_weight_lb = weight_lb;

          // Display the live measurement.
          Serial_PrintWeight(measured_weight_lb);
      }
      else
      {
          Serial_Print("ERROR: HX711 read timeout!\r\n");
      }

      // Check for serial commands.
      if (HAL_UART_Receive(&huart2,
                           &command, 1, 20) == HAL_OK)
      {
          if (command == 't')
          {
              Serial_Print("\r\nTaring...\r\n");

              if (HX711_Tare(&loadcell))
              {
                  Serial_Print("Tare complete!\r\n");
              }
              else
              {
                  Serial_Print("ERROR: Tare failed!\r\n");
              }
          }
          else if (command == 'r')
          {
              Serial_Print("\r\nStarting recalibration...\r\n");

              if (!Calibrate_LoadCell())
              {
                  Serial_Print("ERROR: Recalibration failed!\r\n");
              }
          }
      }

      HAL_Delay(500);


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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
