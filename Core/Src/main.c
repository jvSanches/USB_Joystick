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
#define ADC_MA_SAMPLES 4  /* Reduced from 10 to 4 for FLASH savings */


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
#if defined(NO_USB) && !defined(SHIFTER_CONSOLE)
uint16_t axes[3];
#endif



// Only define constants for the active device type
#ifdef YOKE
#define ROLL_MIN 3400
#define ROLL_MAX 600
#define PITCH_MIN 3450
#define PITCH_MAX 650
#define TILLER_MIN 1090
#define TILLER_MAX 3650
#endif

#ifdef PEDALS
#define RUDDER_MIN 1372
#define RUDDER_MAX 2628
#define L_BRAKE_MIN 2000
#define L_BRAKE_MAX 1100
#define R_BRAKE_MIN 1600
#define R_BRAKE_MAX 800
#endif

#ifdef THROTTLE
#define L_THR_MIN 1427
#define L_THR_MAX 2200
#define R_THR_MIN 2095
#define R_THR_MAX 2932
#endif

#ifdef GA_THROTTLE
#define GA_THR_MIN 890
#define GA_THR_MAX 2260
#endif

#ifdef RACING_PEDALS
#define THROTTLE_MIN 2875
#define THROTTLE_MAX 1062
#define BRAKE_MIN 250
#define BRAKE_MAX 1000
#define CLUTCH_MIN 2875
#define CLUTCH_MAX 1062
#endif

#ifdef SHIFTER_CONSOLE
#define AX1_MIN 100
#define AX1_MAX 4096
#define AX2_MIN 100
#define AX2_MAX 4096
#endif

#if !defined(SHIFTER_CONSOLE) && (defined(YOKE) || defined(THROTTLE) || defined(GA_THROTTLE))
static const uint16_t button_pins[8] = {IN7_Pin, IN9_Pin, IN8_Pin, IN1_Pin, IN2_Pin, IN3_Pin ,IN0_Pin ,IN10_Pin};
static GPIO_TypeDef* const button_ports[8] = {IN7_GPIO_Port, IN9_GPIO_Port, IN8_GPIO_Port, IN10_GPIO_Port,IN2_GPIO_Port, IN3_GPIO_Port, IN0_GPIO_Port, IN10_GPIO_Port};

static inline uint8_t getButton(uint8_t index){
	return (HAL_GPIO_ReadPin(button_ports[index], button_pins[index]) == 0) ? 1 : 0;
}
#endif
static inline uint16_t getADC(uint8_t input){
	uint32_t sum = 0;
	for (uint8_t i = 0; i < ADC_MA_SAMPLES; i++){
		sum += adc_data[3*i + input];
	}
	return sum / ADC_MA_SAMPLES;
}

static inline int32_t map (int32_t au32_IN, int32_t au32_INmin, int32_t au32_INmax, int32_t au32_OUTmin, int32_t au32_OUTmax)
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
static inline void DelayUS(uint32_t us) {
    uint32_t start = TIM14->CNT;
    uint32_t duration = us << 4; // us * 16 using bit shift
    while (TIM14->CNT - start < duration);
}
static uint64_t shift_register_read(void) {
	uint64_t result = 0;

	// Latch inputs
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
	DelayUS(10);
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
	DelayUS(10);

	// Read 40 bits (5 shift registers)
	for (uint8_t bit = 0; bit < 40; bit++) {
		uint8_t bitVal = HAL_GPIO_ReadPin(SERIAL_GPIO_Port, SERIAL_Pin) == GPIO_PIN_SET;
		result |= ((uint64_t)bitVal << bit);
		
		// Clock pulse
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
		DelayUS(10);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
		DelayUS(10);
	}
	return result;
}

#define STABLE_THRESHOLD 5

static uint64_t last_value = 0;
static uint64_t stable_value = 0;
static uint8_t stable_count = 0;

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

static int32_t encoder_count = 0;

// Optimized bit mapping table for SHIFTER_CONSOLE
static const uint8_t bit_map_0[] = {7, 4, 6, 0, 5, 1, 2, 14}; // bits for usb_buffer[0]
static const uint8_t bit_map_1[] = {13, 10, 9, 12, 8}; // bits for usb_buffer[1] (excluding conditional bits)
static const uint8_t bit_map_2[] = {11, 23, 22, 20, 21, 17, 16, 19}; // bits for usb_buffer[2]
static const uint8_t bit_map_3[] = {18, 30, 31, 38, 39, 27, 26, 25}; // bits for usb_buffer[3]
static const uint8_t bit_map_4[] = {24, 28, 29, 37, 36, 33, 32}; // bits for usb_buffer[4] (excluding encoder bit)
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
  HAL_TIM_Base_Start(&htim14);
  uint32_t last_report_tick = 0;
#ifdef SHIFTER_CONSOLE
  uint8_t usb_buffer[10];
  uint8_t last_enc_state = 0;
  uint8_t encoder_inc = 0;
  uint8_t encoder_dec = 0;
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_GetTick() < 2000){
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ((HAL_GetTick()/40)%2));
	  }
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (HAL_GetTick()%2000)>1800);
#ifdef SHIFTER_MODULE
	  stable_value = shift_register_read_filtered();
#endif
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
//buttons
		uint64_t shift_register_inputs = stable_value;
		shift_register_inputs = ~shift_register_inputs;
		usb_buffer[0] = 0;
		usb_buffer[1] = 0;
		usb_buffer[2] = 0;
		usb_buffer[3] = 0;
		usb_buffer[4] = 0;
		usb_buffer[5] = 0;

		uint8_t encoder_state = (shift_register_inputs >> 34) & 0x03;

		switch (last_enc_state){
		case 0:
			if (encoder_state == 1){
				encoder_count++;
			}else if(encoder_state == 2){
				encoder_count--;
				}
			break;
		case 1:
			if (encoder_state == 3){
				encoder_count++;
			}else if(encoder_state == 0){
				encoder_count--;
				}
			break;
		case 3:
			if (encoder_state == 2){
				encoder_count++;
			}else if(encoder_state == 1){
				encoder_count--;
				}
			break;
		case 2:
			if (encoder_state == 0){
				encoder_count++;
			}else if(encoder_state == 3){
				encoder_count--;
				}
			break;
		}
		last_enc_state = encoder_state;
		if (encoder_inc > 0){
			encoder_inc--;
		}
		if (encoder_dec > 0){
			encoder_dec--;
		}
		if (encoder_count > 4){
			encoder_inc = 5;
			encoder_count = 0;
		}else if (encoder_count < -4){
			encoder_dec = 5;
			encoder_count = 0;
		}

		// Optimized bit mapping using lookup tables
		if ((shift_register_inputs & (1ULL << 3)) == 0){
			// Map bits for usb_buffer[0] using lookup table
			for(uint8_t i = 0; i < 8; i++) {
				usb_buffer[0] |= ((shift_register_inputs >> bit_map_0[i]) & 0x01) << i;
			}
			usb_buffer[1] |= ((shift_register_inputs >> 15) & 0x01) << 0;
		}else{
			usb_buffer[1] |= ((shift_register_inputs >> 6) & 0x01) << 1;
			usb_buffer[1] |= ((shift_register_inputs >> 0) & 0x01) << 2;
		}

		// Map remaining bits for usb_buffer[1]
		for(uint8_t i = 0; i < 5; i++) {
			usb_buffer[1] |= ((shift_register_inputs >> bit_map_1[i]) & 0x01) << (i + 3);
		}

		// Map bits for usb_buffer[2-4] using lookup tables
		for(uint8_t i = 0; i < 8; i++) {
			usb_buffer[2] |= ((shift_register_inputs >> bit_map_2[i]) & 0x01) << i;
			usb_buffer[3] |= ((shift_register_inputs >> bit_map_3[i]) & 0x01) << i;
			if(i < 7) usb_buffer[4] |= ((shift_register_inputs >> bit_map_4[i]) & 0x01) << i;
		}
		usb_buffer[4] |= ((encoder_inc>0) & 0x01) << 7;

		// usb_buffer[5]
		usb_buffer[5] |= ((encoder_dec>0) & 0x01) << 0;
  //analog axes
		int16_t ax_value = map(getADC(0),AX1_MIN, AX1_MAX, -1000, 1000);
		usb_buffer[6] = (ax_value) & 255;
		usb_buffer[7] = (ax_value) >> 8;
		ax_value = map(getADC(1),AX2_MIN, AX2_MAX, -1000, 1000);
		usb_buffer[8] = (ax_value) & 255;
		usb_buffer[9] = (ax_value) >> 8;

	#ifndef NO_USB
			  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, usb_buffer, 6);
	#endif
#endif

			  last_report_tick = HAL_GetTick();
	  }

#if defined(NO_USB) && !defined(SHIFTER_CONSOLE)
	for (uint8_t k = 0; k < 3; k++){
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

#ifdef SHIFTER_CONSOLE
  /*Configure GPIO pin Output Level for CLK and LATCH pins */
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
#endif

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

#ifdef SHIFTER_CONSOLE
  /*Configure GPIO pins : CLK_Pin LATCH_pin as outputs */
  GPIO_InitStruct.Pin = CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LATCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN10_Pin IN2_Pin (excluding CLK and LATCH) */
  GPIO_InitStruct.Pin = IN1_Pin|IN10_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#else
  /*Configure GPIO pins : IN0_Pin IN1_Pin IN10_Pin IN2_Pin
                           IN3_Pin */
  GPIO_InitStruct.Pin = IN0_Pin|IN1_Pin|IN10_Pin|IN2_Pin
                          |IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

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
