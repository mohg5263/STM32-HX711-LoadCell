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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
HX711_t loadcell1 = {
    .dout_port = GPIOA,
    .dout_pin = GPIO_PIN_0,
    .sck_port = GPIOA,
    .sck_pin = GPIO_PIN_1,
    .offset = 13229,
    .scale = 25848.474609f
};

HX711_t loadcell2 = {
    .dout_port = GPIOB,
    .dout_pin = GPIO_PIN_0,
    .sck_port = GPIOB,
    .sck_pin = GPIO_PIN_1,
    .offset = 585800,
    .scale = 97598.601562f
};

volatile float weight1_lb = 0.0f;
volatile float weight2_lb = 0.0f;

/* Latest successful samples; TIM6 ISR writes, main loop reads. */
volatile uint32_t axial_timestamp_ms = 0;
volatile uint32_t horizontal_timestamp_ms = 0;
volatile uint32_t axial_sample_count = 0;
volatile uint32_t horizontal_sample_count = 0;
volatile uint8_t axial_valid = 0;
volatile uint8_t horizontal_valid = 0;
volatile uint32_t control_cycle_count = 0;
/* 50-Hz MIT position-monitoring trial. Never enable automatically.
 * Send zero-gain/zero-feedforward frames for up to 10 seconds.
 * Software-only timeout and disable are NOT an emergency stop. */
#define MONITOR_TX_PERIOD_MS      20U
#define MONITOR_MAX_DURATION_MS  60000U
#define MONITOR_START_GRACE_MS   700U
#define MONITOR_FEEDBACK_MAX_AGE_MS 400U

static uint8_t monitor_active = 0U;
static uint8_t monitor_stopping = 0U;
static uint8_t monitor_lockout = 0U;
static uint8_t monitor_fault_stop = 0U;
static uint32_t monitor_start_ms = 0U;
static uint32_t monitor_last_tx_ms = 0U;
static uint32_t monitor_initial_rx = 0U;
static uint32_t monitor_tx_frames = 0U;
static uint32_t monitor_tx_failures = 0U;
static uint32_t monitor_last_disable_attempt_ms = 0U;

char uart_buffer[200];
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



static void Serial_PrintCalibration(HX711_t *hx, const char *name)
{
    char buffer[160];
    Serial_Print("\r\n========== CALIBRATION PARAMETERS ==========\r\n");
    snprintf(buffer, sizeof(buffer), "%s\r\n", name);
    Serial_Print(buffer);
    snprintf(buffer, sizeof(buffer), "Offset: %ld counts\r\n", (long)hx->offset);
    Serial_Print(buffer);
    snprintf(buffer, sizeof(buffer), "Scale: %.6f counts/lb\r\n", (double)hx->scale);
    Serial_Print(buffer);
    Serial_Print("Save these for future hardcoded initialization.\r\n");
}


// Interactive calibration routine.
static bool Calibrate_LoadCell(HX711_t *hx, const char *name)
{
    char input[40];
    float known_weight_lb;

    Serial_Print("\r\n");
    Serial_Print("============================\r\n");
    Serial_Print(" HX711 CALIBRATION: ");
    Serial_Print(name);
    Serial_Print("\r\n");
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

    if (!HX711_Tare(hx))
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

    if (!HX711_Calibrate(hx, known_weight_lb))
    {
        Serial_Print("ERROR: Calibration failed.\r\n");
        return false;
    }

    Serial_Print("\r\nCalibration completed!\r\n");
    Serial_PrintWeight(known_weight_lb);

    Serial_PrintCalibration(hx, name);



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
  MX_CAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
      Error_Handler();
  }

  HAL_Delay(2000);
  Serial_Print("\r\nSTM32 DUAL HX711 - TIM6 ACQUISITION\r\n");
  Serial_Print("Axial: offset 13229, scale 25848.474609 counts/lb\r\n");
  Serial_Print("Horizontal: offset 585800, scale 97598.601562 counts/lb\r\n");
  Serial_Print("TIM6 checks both HX711 DOUT pins every 3 ms.\r\n");
  Serial_Print("PA5 toggles on each TIM6 interrupt (scope: 6 ms period).\r\n");
  Serial_Print("Commands: e = 10s position monitor (50 Hz), d = disable, t = tare, r = recalibrate.\r\n");

  /* Accept CAN frames for diagnostic purposes, decode only motor ID 9.
   * No transmission is made at startup. */
  CAN_FilterTypeDef can_filter = {0};
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK)
      Error_Handler();
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
      Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
      Error_Handler();
  Serial_Print("CAN1 listening at 1 Mbps for MIT feedback from motor ID 9.\r\n");
  Serial_Print("Motor starts DISABLED. Explicit e starts a MAX 10s, 50 Hz MIT monitor.\r\n");
  Serial_Print("Monitor sends p=0, v=0, Kp=0, Kd=0, t=0 (quantized). No zero-origin command.\r\n");
  Serial_Print("IMPORTANT: Motor is ENABLED during test; use free shaft and PHYSICAL POWER CUTOFF.\r\n");

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print_ms = HAL_GetTick();
  while (1)
  {
      uint32_t now = HAL_GetTick();
      /* Foreground-only transmit scheduler: 50 Hz, not tied to TIM6 ISR.
       * TIM6 continues HX711 acquisition every 3 ms. */
      if (monitor_active && !monitor_stopping)
      {
          CubeMars_Feedback_t status;
          CubeMars_GetFeedback(&status);
          uint32_t age = (uint32_t)(now - monitor_start_ms);
          uint8_t got_new_feedback = (status.matching_frames != monitor_initial_rx);

          if (age >= MONITOR_MAX_DURATION_MS)
          {
              monitor_stopping = 1U;
              Serial_Print("Monitor timeout (10s). Requesting motor DISABLE.\r\n");
          }
          else if (age >= MONITOR_START_GRACE_MS &&
                   (!got_new_feedback ||
                    (uint32_t)(now - status.last_rx_ms) > MONITOR_FEEDBACK_MAX_AGE_MS))
          {
              monitor_stopping = 1U;
              monitor_fault_stop = 1U;
              monitor_lockout = 1U;
              Serial_Print("FAULT: MIT feedback missing/stale. Requesting DISABLE.\r\n");
          }
          else if ((hcan1.Instance->ESR & CAN_ESR_BOFF) != 0U)
          {
              monitor_stopping = 1U;
              monitor_fault_stop = 1U;
              monitor_lockout = 1U;
              Serial_Print("FAULT: CAN bus-off. Requesting DISABLE. CHECK POWER CUTOFF.\r\n");
          }
          else if ((uint32_t)(now - monitor_last_tx_ms) >= MONITOR_TX_PERIOD_MS)
          {
              monitor_last_tx_ms = now;
              if (CubeMars_SendPassiveMIT(&hcan1))
              {
                  monitor_tx_frames++;
              }
              else
              {
                  monitor_tx_failures++;
                  monitor_stopping = 1U;
                  monitor_fault_stop = 1U;
                  monitor_lockout = 1U;
                  Serial_Print("FAULT: MIT CAN transmit not queued. Requesting DISABLE.\r\n");
              }
          }
      }

      /* Stop requests: retry non-blocking disable every 10 ms, lock out
       * re-enable if a fault occurred or disabling cannot be queued. */
      if (monitor_stopping &&
          (uint32_t)(now - monitor_last_disable_attempt_ms) >= 10U)
      {
          monitor_last_disable_attempt_ms = now;
          if (CubeMars_DisableOnce(&hcan1))
          {
              monitor_active = 0U;
              monitor_stopping = 0U;
              snprintf(uart_buffer, sizeof(uart_buffer),
                  "MIT DISABLE queued. Monitor TX=%lu, failures=%lu, fault=%u.\r\n",
                  (unsigned long)monitor_tx_frames,
                  (unsigned long)monitor_tx_failures,
                  (unsigned)monitor_fault_stop);
              Serial_Print(uart_buffer);
          }
          else
          {
              monitor_lockout = 1U;
          }
      }

       /* Display only twice per second. TIM6 acquisition continues unchanged. */
       if ((uint32_t)(now - last_print_ms) >= 500U)
       {
           last_print_ms = now;

           float axial_lb = weight1_lb;
           float horizontal_lb = weight2_lb;
           uint32_t axial_age = now - axial_timestamp_ms;
           uint32_t horizontal_age = now - horizontal_timestamp_ms;
           CubeMars_Feedback_t m;
           CubeMars_GetFeedback(&m);

           const char *a_state = !axial_valid ? "NO DATA" :
               (axial_age > 250U ? "STALE" : "OK");
           const char *h_state = !horizontal_valid ? "NO DATA" :
               (horizontal_age > 250U ? "STALE" : "OK");

           /* Feedback display never sends motor commands. */
           if (m.feedback_valid && (uint32_t)(now - m.last_rx_ms) <= 1000U)
           {
               snprintf(uart_buffer, sizeof(uart_buffer),
                        "Axial %.3f lb [%s] | Horizontal %.3f lb [%s] | Motor 9: %.3f rad, %.2f rad/s, %.2f Nm [RX #%lu]\r\n",
                        (double)axial_lb, a_state,
                        (double)horizontal_lb, h_state,
                        (double)m.position_rad, (double)m.velocity_rad_s,
                        (double)m.torque_nm, (unsigned long)m.matching_frames);
           }
           else
           {
               snprintf(uart_buffer, sizeof(uart_buffer),
                        "Axial %.3f lb [%s] | Horizontal %.3f lb [%s] | Motor 9: %s (CAN all=%lu, match=%lu, other=%lu, bad=%lu)\r\n",
                        (double)axial_lb, a_state,
                        (double)horizontal_lb, h_state,
                        m.feedback_valid ? "STALE" : "NO FEEDBACK",
                        (unsigned long)m.all_frames,
                        (unsigned long)m.matching_frames,
                        (unsigned long)m.other_frames,
                        (unsigned long)m.invalid_frames);
           }
           Serial_Print(uart_buffer);
      }

      /* UART is serviced only in foreground, never from an ISR. */
      uint8_t command;
      if (HAL_UART_Receive(&huart2, &command, 1, 1) == HAL_OK)
      {
          if (command == 'e')
          {
              if (monitor_active || monitor_stopping)
              {
                  Serial_Print("Monitor already enabled or stopping.\r\n");
              }
              else if (monitor_lockout)
              {
                  Serial_Print("ENABLE LOCKED OUT after fault. Cut motor power, investigate, reset MCU.\r\n");
              }
              else
              {
                  CubeMars_Feedback_t snap;
                  CubeMars_GetFeedback(&snap);
                  if (CubeMars_EnableOnce(&hcan1))
                  {
                      monitor_active = 1U;
                      monitor_stopping = 0U;
                      monitor_start_ms = HAL_GetTick();
                      monitor_last_tx_ms = monitor_start_ms;
                      monitor_initial_rx = snap.matching_frames;
                      monitor_tx_frames = 0U;
                      monitor_tx_failures = 0U;
                      monitor_fault_stop = 0U;
                      Serial_Print("MIT ENABLE queued: 50Hz zero-gain frames; automatic DISABLE at 10s.\r\n");
                  }
                  else
                  {
                      Serial_Print("MIT ENABLE not queued (mailbox/error).\r\n");
                  }
              }
          }
          else if (command == 'd')
          {
              monitor_active = 0U; /* Stop periodic commands immediately. */
              monitor_stopping = 1U;
              monitor_last_disable_attempt_ms = HAL_GetTick() - 10U;
              Serial_Print("Manual DISABLE requested.\r\n");
          }
          else if (command == 't' || command == 'r')
          {
              if (monitor_active || monitor_stopping)
              {
                  Serial_Print("Calibration blocked while motor is enabled or stopping.\r\n");
                  continue;
              }
              /* Prevent TIM6 from reading the ADCs during a blocking calibration. */
              if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK)
                  Error_Handler();

              if (command == 't')
              {
                  Serial_Print("\r\nRemove all external loads. Taring both...\r\n");
                  bool ok1 = HX711_Tare(&loadcell1);
                  bool ok2 = HX711_Tare(&loadcell2);
                  Serial_Print((ok1 && ok2) ? "Tare complete.\r\n" : "Tare FAILED.\r\n");
              }
              else
              {
                  Serial_Print("\r\nHorizontal load cell recalibration\r\n");
                  if (!Calibrate_LoadCell(&loadcell2, "Horizontal load cell"))
                      Serial_Print("Calibration FAILED.\r\n");
              }

              /* Offsets may have changed. Discard previously cached readings. */
              axial_valid = 0;
              horizontal_valid = 0;
              if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
                  Error_Handler();
          }
      }
      /* No HAL_Delay(500): sampling occurs in the TIM6 ISR. */
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
/* Timer ISR: no waiting for HX711 DOUT, no serial calls, no CAN code.
 * HX711_GetWeight() is safe here only after DOUT is LOW, because its
 * normal up-to-500ms ready wait is then skipped. Each read still
 * disables interrupts during the 25 SCK pulses; scope-check jitter.
 * The HX711 conversion rate (10 or 80 SPS) limits actual new samples.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6)
        return;

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); /* D13 / LD2: 3ms toggle interval */
    control_cycle_count++;

    float reading;
    if (HAL_GPIO_ReadPin(loadcell1.dout_port, loadcell1.dout_pin) == GPIO_PIN_RESET)
    {
        if (HX711_GetWeight(&loadcell1, &reading))
        {
            weight1_lb = reading;
            axial_timestamp_ms = HAL_GetTick();
            axial_sample_count++;
            axial_valid = 1;
        }
    }

    if (HAL_GPIO_ReadPin(loadcell2.dout_port, loadcell2.dout_pin) == GPIO_PIN_RESET)
    {
        if (HX711_GetWeight(&loadcell2, &reading))
        {
            weight2_lb = reading;
            horizontal_timestamp_ms = HAL_GetTick();
            horizontal_sample_count++;
            horizontal_valid = 1;
        }
    }
}
/* Interrupt-driven receive: drain hardware FIFO; never block or print here. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    /* Hardware FIFO holds at most three frames. */
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
