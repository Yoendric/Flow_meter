/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef enum
{
	FALSE=0,
	TRUE=1
	}FLAG_T;
#define SUM_THRD 10
#define DATA_LOCA	5
#define VOLT_LOCA 9
#define VERSION_LOCA 1
int HOUR_EVERY_MSG = 1;
volatile uint8_t SUM_HOUR;
volatile uint8_t SUM_DEC;
volatile uint8_t SUM_COUNTER;
FLAG_T time_flag;
FLAG_T RTC_UPDATE_STATUS;
FLAG_T UART_OK;
#define version 0x01
uint8_t RX_UART_BUFFER[32];
time_t number = 0;
char timeraw[8];
struct tm ts;
RTC_TimeTypeDef sTime;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//RTC_DateTypeDef sDate;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void EXT_Interrup_Init(void);
void TRANSMIT_WSSFM10R2AT_WITH_DOWNLINK(char *, int);
void stm32l_lowPowerSetup(void);
void stm32l_lowPowerResume(void);
void UPDATE_FREQUENCY_MESSAGE(void);
void UPDATE_UTC(void);
void UPDATE_RTC_TIME(void);
FLAG_T CHECK_RECEPTION_OK(void);
void DECODED_TIME(void);
void CASES_CHOICE(char*);
void COUNTER_CONSUMER(void);
void TRANSMIT_WSSFM10R2AT(char * , int);
void TRANSMIT_WSSFM10R2AT_WITH_DOWNLINK(char *, int);
uint32_t CONFIG_CHANNEL_ADC(uint32_t);
uint32_t GET_MEAS_BAT(void);
uint32_t GET_MEAS_HALL(void);
void WAKE_WSSFM10R2AT(void);
void RESET_WSSFM10R2AT(void);
void DEEP_SLEEP_WSSFM10R2AT(void);
void DATA_ASSIGMENT(char* ,uint16_t ,uint8_t ,uint8_t );
void BUILD_DATA_TO_SEND(char*);
void stm32l_lowPowerSetup(void)
{
	__HAL_RCC_PWR_CLK_ENABLE(); // Enable Power Control clock
	HAL_PWR_EnableBkUpAccess (); //Enable access to the backup domain (RTC registers, RTC backup data registers).
	HAL_PWREx_EnableUltraLowPower(); // Ultra low power mode
	HAL_PWREx_EnableFastWakeUp(); // Fast wake-up for ultra low power mode
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	  /* Enable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_13|GPIO_PIN_14;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
void stm32l_lowPowerResume(void){
	HAL_ResumeTick();
	SystemClock_Config();
}
void UPDATE_FREQUENCY_MESSAGE(void){
	char hora_ref[1];
	hora_ref[0] = RX_UART_BUFFER[22];
	char frequency_msg[2];
	frequency_msg[0] = RX_UART_BUFFER[23];
	frequency_msg[1] = RX_UART_BUFFER[25];
	int freq;
	int HORA;
	freq = (int)strtol(frequency_msg, NULL, 16);
	HORA = (int)strtol(hora_ref, NULL, 16);
	if ((freq > 0) & (freq <= 24)){
		HOUR_EVERY_MSG = freq;
		if (ts.tm_hour >= HORA){
			SUM_HOUR = (ts.tm_hour - HORA) % HOUR_EVERY_MSG;
		}else{
			SUM_HOUR = (ts.tm_hour - HORA + 24) % HOUR_EVERY_MSG;
		}
	}
}
void UPDATE_UTC(){
	char utc[1];
	utc[0] = RX_UART_BUFFER[20];
	if ((utc[0] >= '0') | (utc[0] < 'D')){
		if (RX_UART_BUFFER[19] == '0'){
			ts.tm_hour += (int)strtol(utc, NULL, 16);
		}else if (RX_UART_BUFFER[19] == '1'){
			ts.tm_hour =ts.tm_hour - (int)strtol(utc, NULL, 16) + 24;
		}
		ts.tm_hour = ts.tm_hour % 24;
	}
}
void UPDATE_RTC_TIME(){
    sTime.Hours = ts.tm_hour + (int)(ts.tm_hour/10)*6;
    sTime.Minutes = ts.tm_min + (int)(ts.tm_min/10)*6;
    sTime.Seconds = ts.tm_sec + (int)(ts.tm_sec/10)*6;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
}
FLAG_T CHECK_RECEPTION_OK(){
    int i;
    FLAG_T UART_OK = TRUE;
    for (i=29;i>=7;i-=3){
        if ((RX_UART_BUFFER[i] < '0') | (RX_UART_BUFFER[i] > 'F')){
            UART_OK = FALSE;
        }
        if ((RX_UART_BUFFER[i-1] < '0') | (RX_UART_BUFFER[i-1] > 'F')){
            UART_OK = FALSE;
        }
    }
    return UART_OK;
}
void DECODED_TIME(){
	  timeraw[0] = RX_UART_BUFFER[7];
	  timeraw[1] = RX_UART_BUFFER[8];
	  timeraw[2] = RX_UART_BUFFER[10];
	  timeraw[3] = RX_UART_BUFFER[11];
	  timeraw[4] = RX_UART_BUFFER[13];
	  timeraw[5] = RX_UART_BUFFER[14];
	  timeraw[6] = RX_UART_BUFFER[16];
	  timeraw[7] = RX_UART_BUFFER[17];
	  number = (time_t)strtol(timeraw, NULL, 16);
	  ts = *localtime(&number);
}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Turn LED2 on: Alarm generation */
  if (SUM_HOUR == HOUR_EVERY_MSG - 1){
	 time_flag = TRUE;
	 SUM_HOUR = 0;
  }else{
	 SUM_HOUR++;
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  COUNTER_CONSUMER();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CASES_CHOICE(char* buff)
{
	unsigned int instruction = 0;
	if ((SUM_COUNTER == 0) || (SUM_COUNTER > SUM_THRD) || (time_flag == TRUE)) instruction = 1;
    switch (instruction)
	{
		case 0:
			break;
		case 1:
			MX_GPIO_Init();
			MX_ADC_Init();
			MX_LPUART1_UART_Init();
			HAL_Delay(30);
			if ((SUM_COUNTER==0) || (SUM_COUNTER > SUM_THRD)){
			   SUM_COUNTER = SUM_THRD;
			}else{
			   time_flag = FALSE;
			}
			BUILD_DATA_TO_SEND(buff);
			SUM_DEC=0;
			WAKE_WSSFM10R2AT();
			if (RTC_UPDATE_STATUS == FALSE){
				TRANSMIT_WSSFM10R2AT_WITH_DOWNLINK(buff,10);
				RTC_UPDATE_STATUS = TRUE;
			}else{
				TRANSMIT_WSSFM10R2AT(buff,10);
			}
			DEEP_SLEEP_WSSFM10R2AT();
			break;
	}
}
void COUNTER_CONSUMER(void){
	SUM_DEC++;
	SUM_COUNTER--;
}
void TRANSMIT_WSSFM10R2AT(char * buff, int ArrayLength){
	char AT_COMANDO[10];
	sprintf(AT_COMANDO,"AT$RC\r\n");
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT_COMANDO,(uint16_t)strlen(AT_COMANDO),(uint32_t)100);
	HAL_Delay(100);
	sprintf(AT_COMANDO,"AT$SF=");
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT_COMANDO,(uint16_t)strlen(AT_COMANDO),(uint32_t)100);
    HAL_UART_Transmit(&hlpuart1,(uint8_t*)buff,(uint16_t)ArrayLength,(uint32_t)100);
    sprintf(AT_COMANDO,"\r\n");
    HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT_COMANDO,(uint16_t)strlen(AT_COMANDO),(uint32_t)100);
    HAL_Delay(100);
}
void TRANSMIT_WSSFM10R2AT_WITH_DOWNLINK(char * buff, int ArrayLength)
{
	char AT[21];
	sprintf(AT,"AT$RC\r\n");
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT,(uint16_t)strlen(AT),(uint32_t)100);
	HAL_UART_Receive(&hlpuart1, RX_UART_BUFFER, 20, 1000);
	HAL_Delay(100);
	sprintf(AT,"AT$SF=");
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT,(uint16_t)strlen(AT),(uint32_t)100);
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)buff,(uint16_t)ArrayLength,(uint32_t)100);
    sprintf(AT,",1\r\n");
    HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT,(uint16_t)strlen(AT),(uint32_t)100);
	HAL_UART_Receive(&hlpuart1, RX_UART_BUFFER, 32, 50000);
	HAL_Delay(100);
    int i;
    UART_OK = TRUE;
    for (i=29;i>=7;i-=3){
        if ((RX_UART_BUFFER[i] < '0') | (RX_UART_BUFFER[i] > 'F')){
            UART_OK = FALSE;
        }
        if ((RX_UART_BUFFER[i-1] < '0') | (RX_UART_BUFFER[i-1] > 'F')){
            UART_OK = FALSE;
        }
    }
	if (CHECK_RECEPTION_OK() == TRUE){
	     DECODED_TIME();
	     UPDATE_UTC();
	     UPDATE_RTC_TIME();
	     UPDATE_FREQUENCY_MESSAGE();
	}
}
uint32_t CONFIG_CHANNEL_ADC(uint32_t channel){
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = channel;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	uint32_t raw = 0;
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	raw = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	sConfig.Rank = ADC_RANK_NONE;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	return raw;
}
uint32_t GET_MEAS_BAT(void){
	uint32_t bat;
	HAL_GPIO_WritePin(GPIOA, EN_BAT_MEAS_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	bat = CONFIG_CHANNEL_ADC(ADC_CHANNEL_1)*6200/4095;
	HAL_GPIO_WritePin(GPIOA, EN_BAT_MEAS_Pin, GPIO_PIN_RESET);
	return bat;
}
uint32_t GET_MEAS_HALL(void){
	uint32_t hall;
	HAL_GPIO_WritePin(GPIOA, EN_VCC3V3s_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	hall = CONFIG_CHANNEL_ADC(ADC_CHANNEL_3)*3100/4095;
	HAL_GPIO_WritePin(GPIOA, EN_VCC3V3s_Pin, GPIO_PIN_RESET);
	return hall;
}
void WAKE_WSSFM10R2AT(void)
{
	HAL_GPIO_WritePin(WAKE_DS_WISOL_GPIO_Port, WAKE_DS_WISOL_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(WAKE_DS_WISOL_GPIO_Port, WAKE_DS_WISOL_Pin, GPIO_PIN_SET);
	HAL_Delay(500);  // Wait for the wisol to be prepared
}
void RESET_WSSFM10R2AT(void)
{
	HAL_GPIO_WritePin(RST_WISOL_GPIO_Port, RST_WISOL_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(RST_WISOL_GPIO_Port, RST_WISOL_Pin, GPIO_PIN_SET);
}
void DEEP_SLEEP_WSSFM10R2AT(void)
{
	char AT[10];
	sprintf(AT,"AT$P=2\r\n");
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT,(uint16_t)strlen(AT),(uint32_t)100);
	HAL_Delay(100);
}
void CONTINUOUS_WAVE_WSSFM10R2AT(void)
{
	char AT[21];
	sprintf(AT,"AT$RC\r\n");
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT,(uint16_t)strlen(AT),(uint32_t)100);
	HAL_UART_Receive(&hlpuart1, RX_UART_BUFFER, 20, 1000);
	sprintf(AT,"AT$SF=0100000fff,1\r\n");
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)AT,(uint16_t)strlen(AT),(uint32_t)100);
	HAL_UART_Receive(&hlpuart1, RX_UART_BUFFER, 32, 40000);
	HAL_UART_Receive(&hlpuart1, RX_UART_BUFFER, 32, 1000);
}
void DATA_ASSIGMENT(char* buff,uint16_t s,uint8_t loc,uint8_t t)
{
	uint8_t i;//Data position counter
	uint8_t j=0;//Conversion data position counter
	uint8_t dig_count=0;//Digit Counter
	char conversion_buff[5];
	memset(conversion_buff,'\0',5);
	itoa(s,conversion_buff,t);//Convert int S to String in HEX
	while(conversion_buff[dig_count] != '\0') ++dig_count;//Converted data digit counter function

	for (i = (loc-dig_count)+1; j < dig_count; ++i)//Data arrangement in output buffer
	{
		 buff[i] = conversion_buff[j];
		 ++j;
	}
}
void BUILD_DATA_TO_SEND(char* buff)
{
	memset(buff,'0',10); //Clear memory variable
	DATA_ASSIGMENT(buff,GET_MEAS_BAT(),VOLT_LOCA,16);
	DATA_ASSIGMENT(buff,SUM_DEC,DATA_LOCA,16);
	DATA_ASSIGMENT(buff,version,VERSION_LOCA,16);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char data_buff[10];
  SUM_DEC=0;
  SUM_HOUR=0;
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
  MX_ADC_Init();
  //MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  SUM_COUNTER=0;
  RTC_UPDATE_STATUS = FALSE;
  EXT_Interrup_Init();
  /* USER CODE END 2 */
  HAL_Delay(2000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      CASES_CHOICE(data_buff);
	  //HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // Get Time
	  //HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // Get Date
      //mn = sTime.Minutes;
      //sc = sTime.Seconds;
	  HAL_Delay(50);
	  stm32l_lowPowerSetup();
	  stm32l_lowPowerResume();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A 
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_HOURS|RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_BAT_MEAS_Pin|EN_VCC3V3s_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, WAKE_DS_WISOL_Pin|RST_WISOL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : EN_BAT_MEAS_Pin EN_VCC3V3s_Pin */
  GPIO_InitStruct.Pin = EN_BAT_MEAS_Pin|EN_VCC3V3s_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : WAKE_DS_WISOL_Pin RST_WISOL_Pin */
  GPIO_InitStruct.Pin = WAKE_DS_WISOL_Pin|RST_WISOL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void EXT_Interrup_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

     /*Configure GPIO pin : HALL_SENS_IRQ_Pin */
    GPIO_InitStruct.Pin = HALL_SENS_IRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_Init(HALL_SENS_IRQ_GPIO_Port, &GPIO_InitStruct);

    __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
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
