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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int hour;       // UTC hour
    int minute;     // UTC minute
    int second;     // UTC second
} UtcTime;

typedef struct {
    int numberOfSVs;  // Number of satellites
    float latitude;    // Latitude in decimal degrees
    float longitude;   // Longitude in decimal degrees
    float altitude;    // Altitude in meters
    UtcTime utcTime;   // Nested UTC time struct
    char fixQuality;   // Fix quality: 0 = invalid, 1 = GPS fix, 2 = DGPS fix, etc.
    char nsIndicator;  // North/South indicator (N/S)
    char ewIndicator;  // East/West indicator (E/W)
} GpsData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Size of the DMA buffer for GPS sentences
#define GPS_RX_BUFFER_SIZE_RAW 1
// Size of the parse buffer for GPS sentences
#define GPS_RX_BUFFER_SIZE 128
// Size of the buffer where we store counter values from the 1PPS ticks
#define PPS_COUNTER_BUFFER_SIZE 600
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
// GPS Parsed Data
volatile uint8_t gps_RxBuffer_Raw[GPS_RX_BUFFER_SIZE_RAW] = {0}; // Raw DMA buffer
volatile uint8_t gps_RxBuffer[GPS_RX_BUFFER_SIZE] = {0}; // Complete sentences
volatile size_t gpsRxSentenceLen = 0; // Length of the "current" sentence
GpsData gpsData = {0};
// Counter buffer
uint32_t counterValues[PPS_COUNTER_BUFFER_SIZE] = {0};
size_t counterValueWrite = 0;
size_t counterValueRead = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
void processNMEA(void);
int _write(int file, char *ptr, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void processNMEA(void)
{
    if (!gpsRxSentenceLen || gpsRxSentenceLen >= sizeof(gps_RxBuffer)) {
        return; // Ensure there is data to process and it's within buffer limits
    }

    size_t len = gpsRxSentenceLen;
    gpsRxSentenceLen = 0;
    uint8_t *buffer = gps_RxBuffer;

    // Null-terminate the buffer for safe string operations
    buffer[len] = '\0';

    // Example parsing a GGA sentence: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    if (strncmp((char *)buffer, "$GPGGA", 6) == 0) {

    	char *token;
        int fieldIndex = 0;

        token = strtok((char *)buffer, ",");
        while (token != NULL) {
            switch (fieldIndex) {
                case 1: { // UTC time
                    int utcTime = atoi(token);
                    if (utcTime >= 0 && utcTime < 240000) { // Validate HHMMSS range
                        gpsData.utcTime.hour = utcTime / 10000;
                        gpsData.utcTime.minute = (utcTime % 10000) / 100;
                        gpsData.utcTime.second = utcTime % 100;
                    }
                    break;
                }
                case 2: // Latitude
                    gpsData.latitude = atof(token) / 100.0; // Convert to decimal degrees
                    break;
                case 3: // N/S indicator
                    if (token[0] == 'N' || token[0] == 'S') {
                        gpsData.nsIndicator = token[0];
                    }
                    break;
                case 4: // Longitude
                    gpsData.longitude = atof(token) / 100.0; // Convert to decimal degrees
                    break;
                case 5: // E/W indicator
                    if (token[0] == 'E' || token[0] == 'W') {
                        gpsData.ewIndicator = token[0];
                    }
                    break;
                case 6: // Fix quality
                    if (token[0] >= '0' && token[0] <= '9') {
                        gpsData.fixQuality = token[0];
                    }
                    break;
                case 7: // Number of satellites
                    gpsData.numberOfSVs = atoi(token);
                    if (gpsData.numberOfSVs < 0) {
                        gpsData.numberOfSVs = 0; // Set to zero if negative
                    }
                    break;
                case 9: // Altitude
                    gpsData.altitude = atof(token);
                    break;
            }
            fieldIndex++;
            token = strtok(NULL, ",");
        }

        // Handle N/S and E/W indicators
        if (gpsData.nsIndicator == 'S') {
            gpsData.latitude = -gpsData.latitude;
        }
        if (gpsData.ewIndicator == 'W') {
            gpsData.longitude = -gpsData.longitude;
        }

        // gpsData now holds the parsed information
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  // Start the input compare on timer 1; clock source is our 10MHz, the input compare is our 1PPS
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_RxBuffer_Raw, GPS_RX_BUFFER_SIZE_RAW);

  //disable half transfer interrupt; it is enabled in HAL_UARTEx_ReceiveToIdle_DMA()
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  processNMEA();
	  if (counterValueRead != counterValueWrite) {
		  printf("C@%d: %d\n", HAL_GetTick(), counterValues[counterValueRead]);
		  counterValueRead = (counterValueRead + 1) % PPS_COUNTER_BUFFER_SIZE;
	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4.294967295E9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//static unsigned long ITM_SendChar (unsigned long ch)
//{
//  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
//      ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
//  {
//    while (ITM->PORT[0U].u32 == 0UL)
//    {
//      __asm("nop");
//    }
//    ITM->PORT[0U].u8 = (uint8_t)ch;
//  }
//  return (ch);
//}

int _write(int file, char *ptr, int len)
{
    //return usart_write(platform_get_console(), (u8 *)ptr, len);
      int i=0;
      for(i=0 ; i<len ; i++)
        ITM_SendChar((*ptr++));
      return len;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint32_t captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        // Store the captured value in the circular buffer
		counterValues[counterValueWrite] = captureValue;
		// Increment the write index and wrap it around if necessary
		counterValueWrite = (counterValueWrite + 1) % PPS_COUNTER_BUFFER_SIZE;
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
//    printf("UART received: %hu byte(s)\n", Size);
//    printf("%.*s\n\n", Size, gps_RxBuffer_Raw);
	static size_t commandIndex = GPS_RX_BUFFER_SIZE;
	for (int i = 0; i < Size; i++) {
		uint8_t b = gps_RxBuffer_Raw[i];
		if (b == '$') {
			commandIndex = 0;
		}
		if (commandIndex < GPS_RX_BUFFER_SIZE - 1) {
			gps_RxBuffer[commandIndex++] = b;
		}
		if (b == '\n') {
			gpsRxSentenceLen = commandIndex;
			break; // TODO: short sequence commands will break here
		}
	}

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_RxBuffer_Raw, GPS_RX_BUFFER_SIZE_RAW);

    //disable half transfer interrupt; it is enabled in HAL_UARTEx_ReceiveToIdle_DMA()
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
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
