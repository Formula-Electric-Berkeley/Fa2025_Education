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
#include <stdio.h>
#include "lsm6dsv_reg.h"
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__ 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else 
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Assume you have implemented these functions elsewhere

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); //cs low
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, (uint8_t*)bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); //cs high
  return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  reg |= 0x80; // TODO
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); //cs low
  HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi2, bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); //cs high
  return 0;
}

void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
      HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();
  /* USER CODE BEGIN 1 */
  // 1. Initialize the Driver Handle
    // This struct links your platform functions to the ST driver.
    stmdev_ctx_t dev_ctx;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = NULL; // Optional handle for your platform functions (e.g., I2C_HandleTypeDef*)

    // 2. Check Device Communication
    // Read the WHO_AM_I register to make sure you can talk to the sensor.
    uint8_t whoamI;
    lsm6dsv_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LSM6DSV_ID) {
        printf("Device not found!\n");
        while (1); // Halt
    }
    printf("LSM6DSV Found!\n");
    // 3. Restore Default Configuration & Reset
    lsm6dsv_reset_set(&dev_ctx, PROPERTY_ENABLE);
    uint8_t rst;
    do {
        lsm6dsv_reset_get(&dev_ctx, &rst);
    } while (rst);
    // 4. Configure Sensor Settings
    // Enable Block Data Update
    lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    // Set Accelerometer Full Scale & Output Data Rate (ODR)
    lsm6dsv_xl_full_scale_set(&dev_ctx, LSM6DSV_2g);  // Set to +/- 2g
    lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_120Hz); // Set to 104 Hz
    // Set Gyroscope Full Scale & Output Data Rate (ODR)
    lsm6dsv_gy_full_scale_set(&dev_ctx, LSM6DSV_2000dps); // Set to 2000 dps
    lsm6dsv_gy_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_120Hz);  // Set to 104 Hz

    // --- Main Loop ---
    while (1) {
        uint8_t xl_data_ready;
        uint8_t gy_data_ready;
        // 5. Check if new data is available
        lsm6dsv_xl_flag_data_ready_get(&dev_ctx, &xl_data_ready);
        lsm6dsv_gy_flag_data_ready_get(&dev_ctx, &gy_data_ready);
        if (xl_data_ready) {
            int16_t raw_acc[3];
            float acc_mg[3]; // To store data in mg
            // 6. Read raw accelerometer data
            lsm6dsv_acceleration_raw_get(&dev_ctx, raw_acc);
            // 7. Convert raw data to engineering units (mg)
            acc_mg[0] = lsm6dsv_from_fs2_to_mg(raw_acc[0]);
            acc_mg[1] = lsm6dsv_from_fs2_to_mg(raw_acc[1]);
            acc_mg[2] = lsm6dsv_from_fs2_to_mg(raw_acc[2]);
            printf("Acc [mg]: X=%6.2f Y=%6.2f Z=%6.2f\n", acc_mg[0], acc_mg[1], acc_mg[2]);
        }
        if (gy_data_ready) {
            int16_t raw_gyro[3];
            float gyro_mdps[3]; // To store data in mdps
            // 6. Read raw gyroscope data
            lsm6dsv_angular_rate_raw_get(&dev_ctx, raw_gyro);
            // 7. Convert raw data to engineering units (mdps)
            gyro_mdps[0] = lsm6dsv_from_fs2000_to_mdps(raw_gyro[0]);
            gyro_mdps[1] = lsm6dsv_from_fs2000_to_mdps(raw_gyro[1]);
            gyro_mdps[2] = lsm6dsv_from_fs2000_to_mdps(raw_gyro[2]);
            printf("Gyro [mdps]: X=%6.2f Y=%6.2f Z=%6.2f\n", gyro_mdps[0], gyro_mdps[1], gyro_mdps[2]);
        }
        platform_delay(100); // Wait a bit before the next read




  // /* USER CODE END 1 */

  // /* MCU Configuration--------------------------------------------------------*/

  // /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  // HAL_Init();

  // /* USER CODE BEGIN Init */

  // /* USER CODE END Init */

  // /* Configure the system clock */
  // SystemClock_Config();

  // /* USER CODE BEGIN SysInit */

  // /* USER CODE END SysInit */

  // /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  // MX_USART2_UART_Init();
  // MX_SPI2_Init();
  // /* USER CODE BEGIN 2 */
  // // char MSG[128] = "";
  // uint8_t X = 0;
  // /* USER CODE END 2 */

  // /* Infinite loop */
  // /* USER CODE BEGIN WHILE */
  // while (1)
  // {
  //   HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  //   HAL_Delay(300);
  //   /* USER CODE END WHILE */

  //   /* USER CODE BEGIN 3 */
  //   // sprintf(MSG, "Hello! Tracing X = %u\r\n", X);
  //   // HAL_UART_Transmit(&huart2, (uint8_t *)MSG, sizeof(MSG), 100);
  //   X++;
  //   printf("Hellaur %u\r\n", X);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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