#include "main.h"
#include <stdio.h>
#include <math.h>
#include "liquidCrystal_i2c.h"

#define SENSOR_MIN 3600
#define SENSOR_MAX 1400
#define OUTPUT_MIN 0
#define OUTPUT_MAX 100

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;

char buffer1[16];
char buffer2[16];

// Custom char untuk menampilkan karakter "Kelembapan"
uint8_t humidityChar[8] = {
		0b00100,
		0b00100,
		0b01010,
		0b01010,
		0b10001,
		0b10001,
		0b10001,
		0b01110
};

volatile uint16_t adcValue[2];
volatile int adcConversionComplete = 0;
uint32_t prevTime = 0;
uint32_t prevTimeValve = 0;
uint32_t currentTime = 0;
uint32_t periodTime = 500;
uint16_t resultVal[2];
uint16_t averageSensorVal;
uint8_t isSpraying = 0;
int valveCondition[3] = {0,0,0};
int valveFinished[3] = {0,0,0};
int valveCount=0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

long adcToPercentage(int i)
{
  // Fungsi untuk mengubah nilai RAW ADC menjadi persentase
  long sensorRead = adcValue[i];

  return ( ((sensorRead - OUTPUT_MIN) * (OUTPUT_MAX - OUTPUT_MIN)) / ((SENSOR_MAX - SENSOR_MIN) + OUTPUT_MIN) );
  // return ((4096 - sensorRead)/4096 * 100);
}

void valveControl(int i)
{
  // Subroutine untuk kontrol valve

  if (valveCondition[i] == 0 && valveFinished[i] == 0)
  {
	  switch(i)
	  	{
	  	  case 0:
	  	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	  	    break;
	  	  case 1:
	  	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	  	  	break;
	  	  case 2:
	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	  		break;
	  	  default:
	  	    break;// code block
	  	}
	valveCondition[i] = 1;

	prevTimeValve = HAL_GetTick();
  }

  currentTime = HAL_GetTick();

  if (valveCondition[i] == 1 && valveFinished[i] == 0 && ((currentTime - prevTimeValve) > 60000))
  {
	  valveFinished[i] = 1;
	  valveCondition[i] = 0;
  
	  switch(i)
	  	{
	  	  case 0:
	  	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  	    break;
	  	  case 1:
	  	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	  	    break;
	  	  case 2:
	  	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	  	    break;
	  	  default:
	  	  	break;// code block
	  	}
  
	  valveCount++;
  
  }

  if (valveCount == 3)
  {
	  valveCount = 0;
    
	  for (int x = 0; x < 4; x++)
	  {
	    valveFinished[x] = 0;
	  }
  }

}

void ADC_DMA_Process()
{
  // Subroutine untuk proses akses DMA mengambil nilai ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcValue, 2);
	while (adcConversionComplete == 0)
	{

	}
	adcConversionComplete = 0;
}

void LCD_Update()
{
  // Tracking waktu tanpa memblocking program; HAL_GetTick() mengembalikan dalam satuan milisekon
  currentTime = HAL_GetTick();

  // Kita cek apakah waktunya untuk mengupdate display LCD atau tidak, periodTime dapat diubah
  // sesuai kebutuhan refreshrate display.
	if((currentTime - prevTime) > periodTime)
	{
    // Di bawah ini digunakan untuk kontrol LCD
	  HD44780_Clear();
	  HD44780_SetCursor(0,0);
	  sprintf(buffer1, "\1\x31:%d\%%  \1\x32:%d\%%", resultVal[0], resultVal[1]);
	  HD44780_PrintStr(buffer1);

	  HD44780_SetCursor(0,1);
	  sprintf(buffer2, "V1:%d  V2:%d", valveCondition[0], valveCondition[1]);
	  HD44780_PrintStr(buffer2);

	  prevTime = HAL_GetTick();
	}
}

void isTimeToSpray()
{
  // Subroutine untuk menghitung rata-rata persentase kelembapan
  // dan menentukan untuk menyemprot atau tidak
  uint8_t arrayLen = sizeof(resultVal)/ sizeof(resultVal[0]);

  for (int a = 0; a < arrayLen; a++)
  {
    averageSensorVal += resultVal[a];

    if (a == (arrayLen - 1))
    {
      averageSensorVal = averageSensorVal / arrayLen;
    }
  }

  if (averageSensorVal < 40)
  {
    isSpraying = 1;
  } 
  else if (averageSensorVal > 70)
  {
    isSpraying = 0;
  }

}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();

  HD44780_Init(2);
  HD44780_Clear();
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("    AUTOMATIC   ");
  HD44780_SetCursor(0,1);
  HD44780_PrintStr("    SPRINKLER   ");
  HAL_Delay(2000);
  HD44780_Clear();

  HD44780_CreateSpecialChar(1, humidityChar);
  
  while (1)
  {
    // Memulai proses pembacaan nilai sensor dengan metode DMA. Hasil akhir didapatkan nilai ADC.
	  ADC_DMA_Process();

    // Konversi masing-masing nilai dari sensor dari nilai RAW menjadi persentase.
	  for(int i = 0; i < 2; i++)
	  {
	    resultVal[i] = adcToPercentage(i);
	  }

    // Subroutine untuk kontrol valve.
	  valveControl(valveCount);
  
	  // Subroutine untuk update display LCD.
	  LCD_Update();

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_Relay0_Pin|GPIO_Relay1_Pin|GPIO_Relay2_Pin|GPIO_Relay3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_Relay0_Pin GPIO_Relay1_Pin GPIO_Relay2_Pin GPIO_Relay3_Pin */
  GPIO_InitStruct.Pin = GPIO_Relay0_Pin|GPIO_Relay1_Pin|GPIO_Relay2_Pin|GPIO_Relay3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1)
{
	adcConversionComplete = 1;
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
