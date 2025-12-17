/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * YASMEN L. GUMAMA
  * BCA152- B186
  * LABORATORY ACTIVITY 4
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "drv_lcd.h"
#include "drv_aht21.h"
#include <stdlib.h> // Required for abs()
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILTER_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LCD_DrawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    LCD_DrawPixel(x1, y1, color);
    LCD_DrawPixel(x2, y2, color);
    for (uint16_t x = x1; x <= x2; x++) {
        LCD_DrawPixel(x, y1, color);
        LCD_DrawPixel(x, y2, color);
    }
    for (uint16_t y = y1; y <= y2; y++) {
        LCD_DrawPixel(x1, y, color);
        LCD_DrawPixel(x2, y, color);
    }
}

// Recursive function to print integers digit by digit
void LCD_ShowNum(uint16_t x, uint16_t y, int num, uint16_t color, uint16_t back_color) {
    char buf[12]; int i = 0;
    if (num == 0) { LCD_ShowChar(x, y, '0', color, back_color); return; }
    if (num < 0) { LCD_ShowChar(x, y, '-', color, back_color); x += 8; num = -num; }
    while (num > 0) { buf[i++] = (num % 10) + '0'; num /= 10; }
    while (--i >= 0) { LCD_ShowChar(x, y, buf[i], color, back_color); x += 8; }
}

// Manually renders a float value to the screen
void LCD_ShowFloatManual(uint16_t x, uint16_t y, float val, uint16_t color, uint16_t back_color) {
    int intPart = (int)val;
    LCD_ShowNum(x, y, intPart, color, back_color);
    int offset = 8;
    if (intPart < 0) offset += 8;
    if (abs(intPart) > 9) offset += 8;
    if (abs(intPart) > 99) offset += 8;
    LCD_ShowChar(x + offset, y, '.', color, back_color);
    int decPart = (int)((val - (float)intPart) * 100.0f);
    if (decPart < 0) decPart = -decPart;
    if (decPart < 10) {
        LCD_ShowChar(x + offset + 8, y, '0', color, back_color);
        LCD_ShowNum(x + offset + 16, y, decPart, color, back_color);
    } else {
        LCD_ShowNum(x + offset + 8, y, decPart, color, back_color);
    }
}

// Signal Processing Filter
float Filter_Value(float new_val, float *buffer, uint8_t *index, uint8_t *count) {
    if (new_val > 100.0f || new_val < -40.0f) {
        if (*count > 0) {
            float sum = 0;
            for(int i=0; i<*count; i++) sum += buffer[i];
            return sum / *count;
        }
        return new_val;
    }
    buffer[*index] = new_val;
    *index = (*index + 1) % FILTER_SIZE;
    if (*count < FILTER_SIZE) (*count)++;
    float sum = 0;
    for (uint8_t i = 0; i < *count; i++) {
        sum += buffer[i];
    }
    return sum / (float)(*count);
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
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
  // 1. Initialize LCD
    LCD_Init();
    LCD_Clear(WHITE);

    // 2. Draw User Interface
    LCD_DrawRect(5, 5, 235, 40, BLUE);
    LCD_ShowString(40, 15, "RT-Spark AHT21", BLUE, WHITE);
    LCD_ShowString(20, 80, "Temp:", BLACK, WHITE);
    LCD_ShowString(20, 140, "Hum:", BLACK, WHITE);
    LCD_ShowString(20, 200, "Sensor Active ", GREEN, WHITE);

    // 3. Initialize Sensor
    AHT21_Init();

    // Variables for Data Processing
    float raw_temp = 0.0f;
    float raw_hum = 0.0f;
    float final_temp = 0.0f;
    float final_hum = 0.0f;

    // Circular Buffers for Filtering
    float temp_history[FILTER_SIZE] = {0};
    float hum_history[FILTER_SIZE] = {0};
    uint8_t temp_idx = 0, hum_idx = 0;
    uint8_t temp_cnt = 0, hum_cnt = 0;
    uint8_t count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  uint8_t status = AHT21_Read(&raw_temp, &raw_hum);

    /* USER CODE BEGIN 3 */
  }// 5. Acquire Data

  if (status == 1)
  {
      // 6. Process Data
      final_temp = Filter_Value(raw_temp, temp_history, &temp_idx, &temp_cnt);
      final_hum  = Filter_Value(raw_hum, hum_history, &hum_idx, &hum_cnt);

      // 7. Refresh Display (Clear old value with spaces first)
      LCD_ShowString(90, 100, "      ", WHITE, WHITE);
      LCD_ShowString(90, 160, "      ", WHITE, WHITE);

      LCD_ShowFloatManual(90, 100, final_temp, RED, WHITE);
      LCD_ShowString(160, 100, "C", RED, WHITE);

      LCD_ShowFloatManual(90, 160, final_hum, BLUE, WHITE);
      LCD_ShowString(160, 160, "%", BLUE, WHITE);

      // Heartbeat pixel
      if (count++ % 2) LCD_DrawPixel(230, 230, RED);
      else             LCD_DrawPixel(230, 230, WHITE);
  }
  else
  {
      LCD_ShowString(120, 200, "Read Err", RED, WHITE);
      HAL_Delay(100);
      AHT21_Init();
  }

  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, AHT_SDA_Pin|AHT_SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AHT_SDA_Pin AHT_SCL_Pin */
  GPIO_InitStruct.Pin = AHT_SDA_Pin|AHT_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK3;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
#ifdef USE_FULL_ASSERT
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
