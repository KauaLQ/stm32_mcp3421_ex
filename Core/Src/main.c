#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stm32_sw_i2c.h"
#include "dwt_stm32_delay.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

// breakout MCP3421: sda1 scl1 v+1 vcc gnd v+2 scl2 sda2
#define MCP3421_ADDR   (0x68 << 1)  // HAL usa endereço deslocado
#define DIVISOR_RES_MCP1 (0.33 / 1.33)

int32_t MCP3421_ReadRaw(void)
{
    uint8_t rx[4];
    
    // Config: 18 bits, Ganho 1, Contínuo
    uint8_t config = 0x9C;
    HAL_I2C_Master_Transmit(&hi2c1, MCP3421_ADDR, &config, 1, HAL_MAX_DELAY);

    // O tempo de conversão para 18 bits é de ~266ms (3.75 SPS)
    // Se ler antes disso no modo contínuo, você pega o valor anterior.
    // O ideal é checar o bit RDY no 4º byte lido.
    HAL_Delay(270); 

    if (HAL_I2C_Master_Receive(&hi2c1, MCP3421_ADDR, rx, 4, HAL_MAX_DELAY) == HAL_OK) {
        // Monta o valor de 18 bits com extensão de sinal correta
        // rx[0] tem os bits de sinal de 7 a 2.
        int32_t raw = ((int32_t)((int8_t)rx[0]) << 16) | (rx[1] << 8) | rx[2];
        return raw;
    }
    return 0;
}

int32_t MCP3421_Soft_ReadRaw(void) {
    uint8_t rx[3];
    uint8_t config = 0x9C; // 18 bits, ganho 1, contínuo

    // 1. Enviar configuração para iniciar/garantir modo
    // O MCP3421_ADDR já deve estar deslocado (0x68 << 1)
    I2C_write_byte(MCP3421_ADDR, true, false); 
    I2C_write_byte(config, false, true);

    // 2. Aguardar a conversão (270ms para 18 bits)
    HAL_Delay(270);

    // 3. Ler os dados
    // Enviamos o endereço com o bit de leitura (LSB = 1)
    if (I2C_write_byte(MCP3421_ADDR | 0x01, true, false)) {
        rx[0] = I2C_read_byte(true, false);  // MSB (com ACK)
        rx[1] = I2C_read_byte(true, false);  // Middle Byte (com ACK)
        rx[2] = I2C_read_byte(false, true);  // LSB (com NACK e STOP)

        // Montagem do valor de 18 bits (mesma lógica do seu original)
        int32_t raw = ((int32_t)((int8_t)rx[0]) << 16) | (rx[1] << 8) | rx[2];
        return raw;
    }
    
    return 0;
}

float mcpToVoltage(int32_t adc, uint8_t resolution) {
    int32_t maxCounts;

    switch (resolution) {
        case 12: maxCounts = 2048; break;
        case 14: maxCounts = 8192; break;
        case 16: maxCounts = 32768; break;
        case 18: maxCounts = 131072; break;
        default: maxCounts = 131072;
    }

    return (adc * 2.048) / maxCounts;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  // osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  // osKernelStart();

  // IMPORTANTE: Inicializar o contador de microssegundos
  if (DWT_Delay_Init() != 0) {
      Error_Handler(); // Falha ao iniciar contador
  }

  I2C_init(); // Coloca as linhas em nível alto

  char msg[200];
  while (1)
  {
    int32_t raw = MCP3421_ReadRaw();
    float v = mcpToVoltage(raw, 18);
    float v_adjust = v / DIVISOR_RES_MCP1; // Ajusta para divisor de tensão externo
    int32_t integras = (int32_t)v_adjust;
    int32_t decimais = (int32_t)((v_adjust - integras) * 1000000); // 6 casas decimais
    if (decimais < 0) decimais *= -1; // Garante que a parte decimal seja positiva

    // Leitura do MCP via Software I2C (PB4/PB5)
    int32_t raw_soft = MCP3421_Soft_ReadRaw();
    // Conversão (usando sua função mcpToVoltage já existente)
    float v_soft = mcpToVoltage(raw_soft, 18);
    float v_soft_adjust = v_soft / DIVISOR_RES_MCP1;
    int32_t integras_soft = (int32_t)v_soft_adjust;
    int32_t decimais_soft = (int32_t)((v_soft_adjust - integras_soft) * 1000000);
    if (decimais_soft < 0) decimais_soft *= -1;

    snprintf(msg, sizeof(msg), "RAW: %ld | V: %ld.%06ld V | RAW_SOFT: %ld | V_SOFT: %ld.%06ld V\r\n", raw, integras, decimais, raw_soft, integras_soft, decimais_soft);

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);

    HAL_Delay(500);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|SW_I2C_SDA_Pin|SW_I2C_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_I2C_SDA_Pin SW_I2C_SCL_Pin */
  GPIO_InitStruct.Pin = SW_I2C_SDA_Pin|SW_I2C_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
