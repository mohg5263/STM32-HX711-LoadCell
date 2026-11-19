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
#include "cubemars.h"
#include <stdio.h>
#include <string.h>
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Hardcoded calibration: the existing HX711 driver is unchanged. */
HX711_t loadcell1 = {
    .dout_port=GPIOA, .dout_pin=GPIO_PIN_0,
    .sck_port=GPIOA,  .sck_pin=GPIO_PIN_1,
    .offset=13229, .scale=25848.474609f
};
HX711_t loadcell2 = {
    .dout_port=GPIOB, .dout_pin=GPIO_PIN_0,
    .sck_port=GPIOB,  .sck_pin=GPIO_PIN_1,
    .offset=585800, .scale=97598.601562f
};

volatile float weight1_lb = 0.0f;
volatile float weight2_lb = 0.0f;
volatile uint32_t axial_timestamp_ms = 0U;
volatile uint32_t horizontal_timestamp_ms = 0U;
volatile uint8_t axial_valid = 0U;
volatile uint8_t horizontal_valid = 0U;
volatile uint32_t control_cycle_count = 0U;

/* Motor configuration. The public motor position/setpoint globals are in
 * cubemars.c and declared extern in cubemars.h for use by other files. */
#define MOTOR_KP                   1.0f
#define MOTOR_KD                   0.2f
#define MOTOR_COMMAND_PERIOD_MS   20U     /* 50 Hz CAN commands */
#define MOTOR_START_GRACE_MS      700U
#define MOTOR_FEEDBACK_MAX_AGE_MS 400U
#define MOTOR_POSITION_STEP_RAD   0.1f   /* each + or - keypress */
#define MOTOR_POSITION_MIN_RAD   (-12.5f) /* MIT protocol range */
#define MOTOR_POSITION_MAX_RAD    12.5f

static uint8_t motor_active = 0U;
static uint8_t motor_target_ready = 0U;
static uint8_t motor_stop_pending = 0U;
static uint8_t motor_fault_lockout = 0U;
static uint32_t motor_start_ms = 0U;
static uint32_t motor_last_command_ms = 0U;
static uint32_t motor_rx_count_on_enable = 0U;

static char uart_buffer[180];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Serial_Print(const char *text)
{
    /* UART is only used in the foreground, never inside either ISR. */
    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

/* Stop sending active control commands first; request motor disable in loop. */
static void Motor_RequestStop(uint8_t fault)
{
    motor_active = 0U;
    motor_target_ready = 0U;
    motor_stop_pending = 1U;
    if (fault) motor_fault_lockout = 1U;
}

/* Each character is a complete command: no Enter and no line parser. */
static void Process_SerialInput(uint8_t ch)
{
    if (ch == 'e')
    {
        if (motor_active || motor_stop_pending || motor_fault_lockout)
        {
            Serial_Print("Motor active, stopping, or locked out.\r\n");
            return;
        }
        motor_rx_count_on_enable = motor_rx_count;
        if (!CubeMars_Enable(&hcan1))
        {
            Serial_Print("MIT enable could not be queued.\r\n");
            return;
        }
        motor_active = 1U;
        motor_target_ready = 0U;
        motor_start_ms = HAL_GetTick();  /* feedback startup grace only */
        motor_last_command_ms = motor_start_ms;
        Serial_Print("MIT enable queued; waiting for fresh position.\r\n");
    }
    else if (ch == 'd')
    {
        Motor_RequestStop(0U);
        Serial_Print("Motor disable requested.\r\n");
    }
    else if (ch == '+' || ch == '-')
    {
        if (!motor_active || !motor_target_ready || motor_stop_pending)
        {
            Serial_Print("Motor not ready; press e and wait for feedback.\r\n");
            return;
        }
        if (!motor_position_valid ||
            (uint32_t)(HAL_GetTick() - motor_last_rx_ms) > MOTOR_FEEDBACK_MAX_AGE_MS)
        {
            Serial_Print("Position change rejected: feedback stale.\r\n");
            return;
        }

        float next = motor_target_rad +
            ((ch == '+') ? MOTOR_POSITION_STEP_RAD : -MOTOR_POSITION_STEP_RAD);

        /* Encoding supports only [-12.5, 12.5] rad. This is NOT a
         * relative-displacement or mechanical-travel limit. */
        if (next < MOTOR_POSITION_MIN_RAD || next > MOTOR_POSITION_MAX_RAD)
        {
            Serial_Print("Position exceeds MIT encoding range (+/-12.5 rad).\r\n");
            return;
        }
        motor_target_rad = next;
        snprintf(uart_buffer, sizeof(uart_buffer),
                 "Target updated: %.3f rad\r\n", (double)motor_target_rad);
        Serial_Print(uart_buffer);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
if (HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();

  /* Receive all standard CAN frames, decode only MIT feedback for motor 9. */
  CAN_FilterTypeDef can_filter = {0};
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK) Error_Handler();
  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
      Error_Handler();
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();

  Serial_Print("\r\nDual HX711 + CubeMars ID 9 (MIT position control)\r\n");
  Serial_Print("Globals: motor_position_rad / motor_target_rad, in radians.\r\n");
  Serial_Print("Kp=1.0, Kd=0.2; CAN=1 Mbps, command=50 Hz, TIM6=3 ms.\r\n");
  Serial_Print("Commands: e=enable/hold, +=target +0.1 rad, -=target -0.1 rad, d=disable.\r\n");
  Serial_Print("No trial timeout and no relative displacement limit. MIT range +/-12.5 rad.\r\n");
  Serial_Print("Motor NOT enabled automatically. Have a PHYSICAL power cutoff.\r\n");
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
uint32_t last_print_ms = HAL_GetTick();
  while (1)
  {
      uint32_t now = HAL_GetTick();

      if (motor_active)
      {
          /* Acquire initial target from a NEW response after enable. */
          if (!motor_target_ready && motor_rx_count != motor_rx_count_on_enable &&
              motor_position_valid && (uint32_t)(now - motor_last_rx_ms) <= MOTOR_FEEDBACK_MAX_AGE_MS)
          {
              motor_target_rad = motor_position_rad;
              motor_target_ready = 1U;
              Serial_Print("Position received; holding initial position.\r\n");
          }

           if ((hcan1.Instance->ESR & CAN_ESR_BOFF) != 0U ||
                   ((uint32_t)(now - motor_start_ms) >= MOTOR_START_GRACE_MS &&
                    (!motor_target_ready ||
                     (uint32_t)(now - motor_last_rx_ms) > MOTOR_FEEDBACK_MAX_AGE_MS)))
          {
              Motor_RequestStop(1U);
              Serial_Print("FAULT: CAN bus-off or missing feedback. Requesting disable.\r\n");
          }
          else if ((uint32_t)(now - motor_last_command_ms) >= MOTOR_COMMAND_PERIOD_MS)
          {
              motor_last_command_ms = now;
              uint8_t ok = motor_target_ready ?
                  CubeMars_SendPosition(&hcan1, motor_target_rad, MOTOR_KP, MOTOR_KD) :
                  CubeMars_SendPassive(&hcan1);
              if (!ok)
              {
                  Motor_RequestStop(1U);
                  Serial_Print("FAULT: CAN transmission not queued. Requesting disable.\r\n");
              }
          }
      }

      /* Never block in a motor-disable retry: CAN may be disconnected. */
      if (motor_stop_pending && HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
      {
          if (CubeMars_Disable(&hcan1))
          {
              motor_stop_pending = 0U;
              Serial_Print("Motor disable queued (delivery not guaranteed).\r\n");
          }
      }

      /* Show current force and measured/commanded position twice per second. */
      if ((uint32_t)(now - last_print_ms) >= 500U)
      {
          last_print_ms = now;
          const char *a = !axial_valid ? "NO DATA" :
              (now - axial_timestamp_ms > 250U ? "STALE" : "OK");
          const char *h = !horizontal_valid ? "NO DATA" :
              (now - horizontal_timestamp_ms > 250U ? "STALE" : "OK");
          const char *m = !motor_position_valid ? "NO DATA" :
              (now - motor_last_rx_ms > MOTOR_FEEDBACK_MAX_AGE_MS ? "STALE" : "OK");
          snprintf(uart_buffer, sizeof(uart_buffer),
                   "Axial %.3f lb [%s] | Horizontal %.3f lb [%s] | "
                   "Motor %.3f rad [%s] | Target %.3f rad\r\n",
                   (double)weight1_lb, a, (double)weight2_lb, h,
                   (double)motor_position_rad, m, (double)motor_target_rad);
          Serial_Print(uart_buffer);
      }

      /* 1 ms receive timeout; each keypress is handled immediately. */
      uint8_t ch;
      if (HAL_UART_Receive(&huart2, &ch, 1U, 1U) == HAL_OK)
          Process_SerialInput(ch);
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* TIM6: check each HX711 when ready. Existing HX711 driver unchanged. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  /* D13 scope reference, 6ms waveform */
    control_cycle_count++;
    float reading;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET &&
        HX711_GetWeight(&loadcell1, &reading))
    {
        weight1_lb = reading;
        axial_timestamp_ms = HAL_GetTick();
        axial_valid = 1U;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET &&
        HX711_GetWeight(&loadcell2, &reading))
    {
        weight2_lb = reading;
        horizontal_timestamp_ms = HAL_GetTick();
        horizontal_valid = 1U;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
        CubeMars_ProcessRx(&header, data);
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
