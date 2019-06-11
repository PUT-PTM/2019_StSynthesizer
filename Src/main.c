/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "MY_CS43L22.h"
#include <math.h>
#include "sounds.h"
#include "key.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
#define NOTES 12
#define KEYS 18
float volume;

int key = 0;

float pressed;
uint16_t DACOut;

unsigned long time;

// current pin
uint8_t pins[NOTES];
uint8_t keys_pressed[3];
uint16_t debounce_pins_0[NOTES + 3];
uint16_t debounce_pins_1[NOTES + 3];
uint16_t i_t[NOTES];
float sumValues[NOTES];

Key keys[KEYS];
uint8_t notes_keys[NOTES];
NoteEnvelope envelopes[NOTES];

/*
void checkNotes() {
	if(debounce_pins_1[0] > 10)
		pins[0] =
	if(debounce_pins_1[1] > 10)
		pins[1] = HAL_GPIO_ReadPin(GPIOA, NOTE_2_Pin);
	if(debounce_pins_1[2] > 10)
		pins[2] = HAL_GPIO_ReadPin(GPIOA, NOTE_3_Pin);
	if(debounce_pins_1[3] > 10)
		pins[3] = HAL_GPIO_ReadPin(GPIOA, NOTE_4_Pin);
	if(debounce_pins_1[4] > 10)
		pins[4] = HAL_GPIO_ReadPin(GPIOA, NOTE_5_Pin);
	if(debounce_pins_1[5] > 10)
		pins[5] = HAL_GPIO_ReadPin(GPIOA, NOTE_6_Pin);
	if(debounce_pins_1[6] > 10)
		pins[6] = HAL_GPIO_ReadPin(GPIOC, NOTE_7_Pin);
	if(debounce_pins_1[7] > 10)
		pins[7] = HAL_GPIO_ReadPin(GPIOD, NOTE_8_Pin);
	if(debounce_pins_1[8] > 10)
		pins[8] = HAL_GPIO_ReadPin(GPIOD, NOTE_9_Pin);
	if(debounce_pins_1[9] > 10)
		pins[9] = HAL_GPIO_ReadPin(GPIOD, NOTE_10_Pin);
	if(debounce_pins_1[10] > 10)
		pins[10] = HAL_GPIO_ReadPin(GPIOD, NOTE_11_Pin);
	if(debounce_pins_1[11] > 10)
		pins[11] = HAL_GPIO_ReadPin(GPIOD, NOTE_12_Pin);
}

void checkKeys() {
	if(debounce_pins_1[12] > 10 && !keys_pressed[0]) {
		keys_pressed[0] = 1;

		if(currentOSC - 1 < 0) {
			currentOSC = 4;
		} else {
			currentOSC--;
		}
	}

	if(debounce_pins_1[13] > 10 && !keys_pressed[1]) {
		keys_pressed[1] = 1;

		if(currentOctave + 1 > 7) {
			currentOctave = 1;
		} else {
			currentOctave++;
		}
	}
}
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM3_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//checkNotes();
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// used to generate sound
int16_t I2S_dummy[4];
float sinVal;


int notesPlaying() {
	int out = 0;

	for(int i = 0; i < NOTES; i++) {
		if(envelopes[i].is_playing || envelopes[i].is_releasing) out++;
	}

	return out;
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM3_Init();
  MX_DAC_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Audio stream initiazlization
  CS43_Init(hi2c1, MODE_ANALOG);
  CS43_SetVolume(50);
  CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
  CS43_Start();

  // Start timer 2
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)I2S_dummy, 4);

  // initialize keys
  Key key_1 = {.pin_group = GPIOA, .pin = KEY_1_Pin};
  Key key_2 = {.pin_group = GPIOA, .pin = KEY_2_Pin};
  Key key_3 = {.pin_group = GPIOA, .pin = KEY_3_Pin};
  Key key_4 = {.pin_group = GPIOA, .pin = KEY_4_Pin};
  Key key_5 = {.pin_group = GPIOA, .pin = KEY_5_Pin};
  Key key_6 = {.pin_group = GPIOB, .pin = KEY_6_Pin};
  Key key_7 = {.pin_group = GPIOA, .pin = KEY_7_Pin};
  Key key_8 = {.pin_group = GPIOA, .pin = KEY_8_Pin};
  Key key_9 = {.pin_group = GPIOA, .pin = KEY_9_Pin};
  Key key_10 = {.pin_group = GPIOE, .pin = KEY_10_Pin};
  Key key_11 = {.pin_group = GPIOE, .pin = KEY_11_Pin};
  Key key_12 = {.pin_group = GPIOE, .pin = KEY_12_Pin};
  Key key_13 = {.pin_group = GPIOE, .pin = KEY_13_Pin};
  Key key_14 = {.pin_group = GPIOD, .pin = KEY_15_Pin};
  Key key_15 = {.pin_group = GPIOD, .pin = KEY_16_Pin};
  Key key_16 = {.pin_group = GPIOD, .pin = KEY_17_Pin};
  Key key_17 = {.pin_group = GPIOD, .pin = KEY_18_Pin};
  Key key_18 = {.pin_group = GPIOD, .pin = KEY_19_Pin};

  keys[0] = key_1;
  keys[1] = key_2;
  keys[2] = key_3;
  keys[3] = key_4;
  keys[4] = key_5;
  keys[5] = key_6;
  keys[6] = key_7;
  keys[7] = key_8;
  keys[8] = key_9;
  keys[9] = key_10;
  keys[10] = key_11;
  keys[11] = key_12;
  keys[12] = key_13;
  keys[13] = key_14;
  keys[14] = key_15;
  keys[15] = key_16;
  keys[16] = key_17;
  keys[17] = key_18;

  // set notes keys
  notes_keys[0] = 0;
  notes_keys[1] = 2;
  notes_keys[2] = 1;
  notes_keys[3] = 3;
  notes_keys[4] = 5;
  notes_keys[5] = 6;
  notes_keys[6] = 9;
  notes_keys[7] = 10;
  notes_keys[8] = 11;
  notes_keys[9] = 14;
  notes_keys[10] = 12;
  notes_keys[11] = 16;

  // set the envelopes
  NoteEnvelope envelope_1 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_2 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_3 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_4 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_5 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_6 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_7 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_8 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_9 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_10 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_11 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};
  NoteEnvelope envelope_12 = {.attackTime = 0.01f, .decayTime = 0.01f, .releaseTime = 0.1f, .startAmplitude = 1.0f, .sustainAmplitude = 0.8f};

  envelopes[0] = envelope_1;
  envelopes[1] = envelope_2;
  envelopes[2] = envelope_3;
  envelopes[3] = envelope_4;
  envelopes[4] = envelope_5;
  envelopes[5] = envelope_6;
  envelopes[6] = envelope_7;
  envelopes[7] = envelope_8;
  envelopes[8] = envelope_9;
  envelopes[9] = envelope_10;
  envelopes[10] = envelope_11;
  envelopes[11] = envelope_12;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /**DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /**DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  htim2.Init.Prescaler = 200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 35-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 168-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_10_Pin KEY_11_Pin KEY_12_Pin KEY_13_Pin */
  GPIO_InitStruct.Pin = KEY_10_Pin|KEY_11_Pin|KEY_12_Pin|KEY_13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_1_Pin KEY_2_Pin KEY_3_Pin KEY_4_Pin 
                           KEY_5_Pin KEY_7_Pin KEY_8_Pin KEY_9_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin|KEY_2_Pin|KEY_3_Pin|KEY_4_Pin 
                          |KEY_5_Pin|KEY_7_Pin|KEY_8_Pin|KEY_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_6_Pin */
  GPIO_InitStruct.Pin = KEY_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_15_Pin KEY_16_Pin KEY_17_Pin KEY_18_Pin 
                           KEY_19_Pin */
  GPIO_InitStruct.Pin = KEY_15_Pin|KEY_16_Pin|KEY_17_Pin|KEY_18_Pin 
                          |KEY_19_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	UNUSED(htim);

	if(htim->Instance == TIM2) {
		pressed = notesPlaying();

		if(pressed == 0) {
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1);
		}

		if(envelopes[0].is_playing || envelopes[0].is_releasing) {
		  float soundVal = generateWaves(C, time, getOSCType(currentOSC), envelopes[0]);

		  sumValues[0] = soundVal;
		}
		if(envelopes[1].is_playing || envelopes[1].is_releasing) {
		  float soundVal = generateWaves(Cs, time, getOSCType(currentOSC), envelopes[1]);

		  sumValues[1] = soundVal;
		}
		if(envelopes[2].is_playing || envelopes[2].is_releasing) {
		  float soundVal = generateWaves(D, time, getOSCType(currentOSC), envelopes[2]);

		  sumValues[2] = soundVal;
		}
		if(envelopes[3].is_playing || envelopes[3].is_releasing) {
		  float soundVal = generateWaves(Ds, time, getOSCType(currentOSC), envelopes[3]);

		  sumValues[3] = soundVal;
		}
		if(envelopes[4].is_playing || envelopes[4].is_releasing) {
		  float soundVal = generateWaves(E, time, getOSCType(currentOSC), envelopes[4]);

		  sumValues[4] = soundVal;
		}
		if(envelopes[5].is_playing || envelopes[5].is_releasing) {
		  float soundVal = generateWaves(F, time, getOSCType(currentOSC), envelopes[5]);

		  sumValues[5] = soundVal;
		}
		if(envelopes[6].is_playing || envelopes[6].is_releasing) {
		  float soundVal = generateWaves(Fs, time, getOSCType(currentOSC), envelopes[6]);

		  sumValues[6] = soundVal;
		}
		if(envelopes[7].is_playing || envelopes[7].is_releasing) {
		  float soundVal = generateWaves(G, time, getOSCType(currentOSC), envelopes[7]);

		  sumValues[7] = soundVal;
		}
		if(envelopes[8].is_playing || envelopes[8].is_releasing) {
		  float soundVal = generateWaves(Gs, time, getOSCType(currentOSC), envelopes[8]);

		  sumValues[8] = soundVal;
		}
		if(envelopes[9].is_playing || envelopes[9].is_releasing) {
		  float soundVal = generateWaves(A, time, getOSCType(currentOSC), envelopes[9]);

		  sumValues[9] = soundVal;
		}
		if(envelopes[10].is_playing || envelopes[10].is_releasing) {
		  float soundVal = generateWaves(Bf, time, getOSCType(currentOSC), envelopes[10]);

		  sumValues[10] = soundVal;
		}
		if(envelopes[11].is_playing || envelopes[11].is_releasing) {
		  float soundVal = generateWaves(B, time, getOSCType(currentOSC), envelopes[11]);

		  sumValues[11] = soundVal;
		}


		float DACTemp = 0;

		for(int i = 0; i < 12; i++) {
		  DACTemp += sumValues[i]/(float)pressed;
		}


		DACOut = (DACTemp + 1) * 2047;


		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACOut);

		for(int i = 0; i < NOTES; i++) {
			 sumValues[i] = 0;
		}

		// global timer
		time++;

		// envelope timer
		for(int i = 0; i < 12; i++) {
			if(envelopes[i].is_playing) {
				envelopes[i].time += 1.0f/16000.0f;
			}

			if(envelopes[i].is_releasing) {
				envelopes[i].time += 1.0f/16000.0f;

				if(envelopes[i].time > envelopes[i].releaseTime) {
					envelopes[i].time = 0;
					envelopes[i].is_releasing = 0;
				}
			}


		}
	}

	// Timer 3 is a timer used for key (click, hold, release) actions
	// The speed of this timer is 100 loops each second (10 ms for each loop)
	if(htim->Instance == TIM3) {
		for(int i = 0; i < KEYS; i++) {
			// on click
			if(HAL_GPIO_ReadPin(keys[i].pin_group, keys[i].pin) == GPIO_PIN_SET && keys[i].is_ready == 1) {
				// your code here, add if for given i

				if(i == 15) {
					currentOSC--;
					if(currentOSC < 0) {
						currentOSC = 5;
					}

				}
				if(i == 13) {
					currentOSC++;
					currentOSC %= 6;
				}

				if(i == 8) {
					currentOctave--;
					if(currentOctave == 0) {
						currentOctave = 7;
					}
				}

				if(i == 7) {
					currentOctave++;
					currentOctave %= 8;
					if(currentOctave == 0) {
						currentOctave = 1;
					}
				}

				for(int j = 0; j < NOTES; j++) {
					if(i == notes_keys[j]) {
						envelopes[j].is_playing = 1;
						envelopes[j].is_releasing = 0;
					}
				}

				keys[i].is_clicked = 1;
				keys[i].is_ready = 0;
			}

			// on ready
			if(HAL_GPIO_ReadPin(keys[i].pin_group, keys[i].pin) == GPIO_PIN_SET && keys[i].is_ready == 0 && keys[i].is_clicked == 0) {
				// your code

				keys[i].is_ready = 1;
			}

			// on hold
			if(HAL_GPIO_ReadPin(keys[i].pin_group, keys[i].pin) == GPIO_PIN_SET && keys[i].is_clicked == 1) {
				// your code here, add if for given i

			}

			// on release
			if(HAL_GPIO_ReadPin(keys[i].pin_group, keys[i].pin) == GPIO_PIN_RESET && keys[i].is_clicked == 1) {
				// your code here, add if for given i

				for(int j = 0; j < NOTES; j++) {
					if(i == notes_keys[j]) {
						envelopes[j].is_playing = 0;
						envelopes[j].is_releasing = 1;
						envelopes[j].time = 0;
					}
				}

				keys[i].is_clicked = 0;
			}
		}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
