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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_MA_SAMPLES 10


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
#ifndef NO_USB
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adc_data[3*ADC_MA_SAMPLES];
#ifdef NO_USB
uint16_t axes[3];
#endif



#define ROLL_MIN 3400
#define ROLL_MAX 600
#define PITCH_MIN 3450
#define PITCH_MAX 650
#define TILLER_MIN 1090
#define TILLER_MAX 3650

#define RUDDER_MIN 1372
#define RUDDER_MAX 2628
#define L_BRAKE_MIN 2000
#define L_BRAKE_MAX 1100
#define R_BRAKE_MIN 1600
#define R_BRAKE_MAX 800

#define L_THR_MIN 1427
#define L_THR_MAX 2200
#define R_THR_MIN 2095
#define R_THR_MAX 2932

#define GA_THR_MIN 890
#define GA_THR_MAX 2260

#define THROTTLE_MIN 2875
#define THROTTLE_MAX 1062
#define BRAKE_MIN 250
#define BRAKE_MAX 1000
#define CLUTCH_MIN 2875
#define CLUTCH_MAX 1062

#ifndef SHIFTER_CONSOLE
uint16_t button_pins[8] = {IN7_Pin, IN9_Pin, IN8_Pin, IN1_Pin, IN2_Pin, IN3_Pin ,IN0_Pin ,IN10_Pin};
GPIO_TypeDef* button_ports[8] = {IN7_GPIO_Port, IN9_GPIO_Port, IN8_GPIO_Port, IN10_GPIO_Port,IN2_GPIO_Port, IN3_GPIO_Port, IN0_GPIO_Port, IN10_GPIO_Port};

uint16_t getADC(uint8_t input){
	uint32_t sum = 0;
	for (uint8_t i = 0; i<ADC_MA_SAMPLES; i++){
		sum += adc_data[3*i + input];
	}
	return sum/ADC_MA_SAMPLES;
}
uint8_t getButton(uint8_t index){
	if (HAL_GPIO_ReadPin(button_ports[index], button_pins[index]) == 0){
		return 1;
	}else{
		return 0;
	}
}
#endif

int32_t map (int32_t au32_IN, int32_t au32_INmin, int32_t au32_INmax, int32_t au32_OUTmin, int32_t au32_OUTmax)
{
	int32_t au32_OUT = ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
	if (au32_OUT > au32_OUTmax){
		au32_OUT = au32_OUTmax;
	}else if (au32_OUT < au32_OUTmin){
		au32_OUT = au32_OUTmin;
	}
    return au32_OUT;
}
#ifdef SHIFTER_CONSOLE
void DelayUS(uint32_t us) {
//	HAL_Delay(1);
    uint32_t start = TIM14->CNT;
    uint32_t duration = us * 16;
    while (TIM14->CNT - start < duration);
}
uint64_t shift_register_read(void) {
	uint64_t result = 0;

//	uint8_t bit_list[40];
	// Passo 2: Baixar PL para capturar entradas
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
	DelayUS(10);

	// Passo 3: Subir PL para iniciar leitura serial
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
	DelayUS(10);

	// Passo 4: Ler 40 bits (5 shift registers)
	for (uint8_t bit = 0; bit < 40; bit++) {
		// Lê bit atual no QH
		uint8_t bitVal = HAL_GPIO_ReadPin(MISO2_GPIO_Port, MISO2_Pin) == GPIO_PIN_SET;

		result |= ((uint64_t)bitVal << bit);
//		bit_list[bit] = bitVal;
		// Pulso de clock
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		DelayUS(10);
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
		DelayUS(10);
	}
	    return result;
}

#define STABLE_THRESHOLD 5

uint64_t last_value = 0;
uint64_t stable_value = 0;
uint8_t stable_count = 0;

uint64_t shift_register_read_filtered(void) {
    uint64_t current_value = shift_register_read();

    if (current_value == last_value) {
        stable_count++;
    } else {
        stable_count = 1;
        last_value = current_value;
    }

    if (stable_count >= STABLE_THRESHOLD) {
        stable_value = current_value;
    }

    return stable_value;
}

uint8_t usb_buffer[10];
int32_t encoder_count = 0;
#endif
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
  MX_ADC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc, adc_data, 3*ADC_MA_SAMPLES);
  uint32_t last_report_tick = 0;
  HAL_TIM_Base_Start(&htim14);


  uint8_t usb_buffer[10];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  DelayUS(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_GetTick() < 2000){
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ((HAL_GetTick()/40)%2));
	  }
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (HAL_GetTick()%2000)>1800);

	  if (HAL_GetTick() > last_report_tick + 10){
#ifdef YOKE
		  uint8_t usb_buffer[7];
		  usb_buffer[0] = 0;
		  for (uint8_t i = 0; i<8; i++){
			  usb_buffer[0] += (getButton(i) << i);
		  }
		  int16_t ax_value = map(getADC(0),ROLL_MIN, ROLL_MAX, -1000, 1000);
		  usb_buffer[1] = (ax_value) & 255;
		  usb_buffer[2] = (ax_value) >> 8;
		  ax_value = map(getADC(1),PITCH_MIN, PITCH_MAX, -1000, 1000);
		  usb_buffer[3] = (ax_value) & 255;
		  usb_buffer[4] = (ax_value) >> 8;
		  ax_value = map(getADC(2),TILLER_MIN, TILLER_MAX, -1000, 1000);
		  usb_buffer[5] = (ax_value) & 255;
		  usb_buffer[6] = (ax_value) >> 8;
		#ifndef NO_USB
				  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, usb_buffer, 7);
		#endif
#endif
	#ifdef PEDALS
			  uint8_t usb_buffer[6];
			  int16_t ax_value = map(getADC(0),RUDDER_MIN, RUDDER_MAX, -1000, 1000);
			  usb_buffer[0] = (ax_value) & 255;
			  usb_buffer[1] = (ax_value) >> 8;
			  ax_value = map(getADC(1),L_BRAKE_MIN, L_BRAKE_MAX, -1000, 1000);
			  usb_buffer[2] = (ax_value) & 255;
			  usb_buffer[3] = (ax_value) >> 8;
			  ax_value = map(getADC(2),R_BRAKE_MIN, R_BRAKE_MAX, -1000, 1000);
			  usb_buffer[4] = (ax_value) & 255;
			  usb_buffer[5] = (ax_value) >> 8;

		#ifndef NO_USB
				  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, usb_buffer, 6);
		#endif

	#endif
#ifdef THROTTLE
		  uint8_t usb_buffer[5];
		  usb_buffer[0] = 0;
		  for (uint8_t i = 0; i<8; i++){
			  usb_buffer[0] += (getButton(i) << i);
		  }

		  int16_t ax_value = map(getADC(0),L_THR_MIN, L_THR_MAX, -1000, 1000);
		  usb_buffer[1] = (ax_value) & 255;
		  usb_buffer[2] = (ax_value) >> 8;
		  ax_value = map(getADC(1),R_THR_MIN, R_THR_MAX, -1000, 1000);
		  usb_buffer[3] = (ax_value) & 255;
		  usb_buffer[4] = (ax_value) >> 8;
		#ifndef NO_USB
					  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, usb_buffer, 5);
		#endif
#endif
#ifdef GA_THROTTLE
		  uint8_t usb_buffer[5];
		  usb_buffer[0] = 0;
		  for (uint8_t i = 0; i<8; i++){
			  usb_buffer[0] += (getButton(i) << i);
		  }

		  int16_t ax_value = map(getADC(0),GA_THR_MIN, GA_THR_MAX, -1000, 1000);
		  usb_buffer[1] = (ax_value) & 255;
		  usb_buffer[2] = (ax_value) >> 8;
		#ifndef NO_USB
					  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, usb_buffer, 3);
		#endif

#endif
#ifdef RACING_PEDALS
		  uint8_t usb_buffer[6];
		  int16_t ax_value = map(getADC(0),THROTTLE_MIN, THROTTLE_MAX, -1000, 1000);
		  usb_buffer[0] = (ax_value) & 255;
		  usb_buffer[1] = (ax_value) >> 8;
		  ax_value = map(getADC(1),BRAKE_MIN, BRAKE_MAX, -1000, 1000);
		  usb_buffer[2] = (ax_value) & 255;
		  usb_buffer[3] = (ax_value) >> 8;
		  ax_value = map(getADC(2),CLUTCH_MIN, CLUTCH_MAX, -1000, 1000);
		  usb_buffer[4] = (ax_value) & 255;
		  usb_buffer[5] = (ax_value) >> 8;

	#ifndef NO_USB
			  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, usb_buffer, 6);
	#endif
#endif
#ifdef SHIFTER_CONSOLE


	#ifndef NO_USB
			  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, usb_buffer, 6);
	#endif
#endif

			  last_report_tick = HAL_GetTick();
	  }
#ifdef NO_USB
	for (uint8_t k = 0; k<3; k++){
		axes[k] = getADC(k);
	}
#endif





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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 2;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN8_Pin IN7_Pin */
  GPIO_InitStruct.Pin = IN8_Pin|IN7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IN9_Pin */
  GPIO_InitStruct.Pin = IN9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN0_Pin IN1_Pin IN10_Pin IN2_Pin
                           IN3_Pin */
  GPIO_InitStruct.Pin = IN0_Pin|IN1_Pin|IN10_Pin|IN2_Pin
                          |IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/*
char YokeReportDescriptor[44] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x16, 0x18, 0xfc,              //     LOGICAL_MINIMUM (-1000)
    0x26, 0xe8, 0x03,              //     LOGICAL_MAXIMUM (1000)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //     END_COLLECTION
    0xc0                           // END_COLLECTION
};
char PedalsReportDescriptor[30] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x32,                    //     USAGE (Z)
    0x09, 0x33,                    //     USAGE (Rx)
    0x09, 0x34,                    //     USAGE (Ry)
    0x16, 0x18, 0xfc,              //     LOGICAL_MINIMUM (-1000)
    0x26, 0xe8, 0x03,              //     LOGICAL_MAXIMUM (1000)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //     END_COLLECTION
    0xc0                           // END_COLLECTION
};
char ThrottleReportDescriptor[44] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x33,                    //     USAGE (Rx)
    0x09, 0x34,                    //     USAGE (Ry)
    0x16, 0x18, 0xfc,              //     LOGICAL_MINIMUM (-1000)
    0x26, 0xe8, 0x03,              //     LOGICAL_MAXIMUM (1000)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //     END_COLLECTION
    0xc0                           // END_COLLECTION
};
*/


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
