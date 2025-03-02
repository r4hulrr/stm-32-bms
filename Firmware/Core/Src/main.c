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

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "liquidcrystal_i2c.h"				// Uses the STM32 HAL I2C HD44780 driver by eziya
#include <stdlib.h>


#define CHARGING_MOSFET_PIN GPIO_PIN_8  	// PB8 - Charge MOSFET
#define DISCHARGING_MOSFET_PIN GPIO_PIN_7  	// PB7 - Discharge MOSFET
#define MOSFET_GPIO_PORT GPIOB
#define MIN_BATTERY_VOLTAGE 3.0				// Stop discharging if voltage too low
#define MAX_BATTERY_VOLTAGE 4.2  			// Stop charging if voltage too high
#define NUM_SAMPLES 20  					// Number of samples for averaging
#define ACS712_SENSITIVITY 0.185			// Sensitivity in V/A for the current sensor
#define ADC_VREF 3.3						// ADC reference voltage
#define ADC_MAX 4095						// 12 bit ADC Max value
#define ZERO_CURRENT_VOLTAGE 2.5			// Voltage at 0A (for the current sensor)
#define CHARGING_MAX_TEMP 45.0				// These safe temperature and current values are for a
#define DISCHARGING_MAX_TEMP 60.0			// 3000 mAh 18650 Li-ion connected to a standard load.
#define MAX_CHARGE_CURRENT 3.0				// May differ based on load and battery used
#define MAX_DISCHARGE_CURRENT 9.0


ADC_HandleTypeDef hadc1;					// ADC handles for the sensors
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
I2C_HandleTypeDef hi2c1;					// I2C handle for LCD


float vol;									// stores voltage value from voltage sensor
volatile uint8_t mode = 0;  				// 0 = Charging, 1 = Discharging
volatile uint8_t manual_override = 0;  		// 0 = Auto mode, 1 = Manual mode, controlled by push button
float rawVoltage;							// stores voltage value from current sensor
float current;								// stores current value from current sensor
float temp;									// stores temperature value from temperature sensor
char msg1[50];								// message buffer for LCD
char msg2[50];



void SystemClock_Config(void);				// STM32 provided functions for peripheral configurations
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
float getFilteredCurrent(void);				// Functions used to average ADC values obtained
float getFilteredVoltage(void);
float getFilteredTemperature(void);
void checkBatteryTemp(float temp);			// Functions for overcurrent and overheat protection
void checkBatteryCurrent(float current);

int main(void)
{
  HAL_Init();								// Resets all peripherals, Initializes the Flash interface and the Systick
  SystemClock_Config();						// System clock configuration

  MX_GPIO_Init();							// Initialize all configured peripherals
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();

  HD44780_Clear();							// Clears LCD screen

  while (1)
  {
	vol = getFilteredVoltage();				// Gets voltage and current values from sensors
	current = getFilteredCurrent();
	temp = getFilteredTemperature();

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET)					// Checks if Push button is pressed
	{
		HAL_Delay(100);															// Debounce delay
		while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);  		// Wait for release
		manual_override = !manual_override;  									// Toggle manual mode
		mode = !mode;  															// Toggle charge/discharge
	}

	if (manual_override == 0)  													// Auto mode if push button not pressed
	{
		if (vol < MIN_BATTERY_VOLTAGE)											// Switches states based on voltage value
			mode = 0;
		else if (vol >= MAX_BATTERY_VOLTAGE)
			mode = 1;
	}

	checkBatteryTemp(temp);														// Checks that the temp and current are within the
	checkBatteryCurrent(current);												// safe limit

	if (mode == 0)																// MOSFET control logic based on charging state
	{
		HAL_GPIO_WritePin(MOSFET_GPIO_PORT, CHARGING_MOSFET_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOSFET_GPIO_PORT, DISCHARGING_MOSFET_PIN, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(MOSFET_GPIO_PORT, CHARGING_MOSFET_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOSFET_GPIO_PORT, DISCHARGING_MOSFET_PIN, GPIO_PIN_SET);
	}

	sprintf(msg1,"V: %.2f, C: %.2f",vol,current);								// Prints voltage, current and state info on the LCD
	HD44780_SetCursor(0,0);
	HD44780_PrintStr(msg1);
	sprintf(msg2,"M: %d",!mode);
	HD44780_SetCursor(0,1);
	HD44780_PrintStr(msg2);
	HAL_Delay(1000);
  }
}

float getFilteredCurrent()														// Gets average current value from sensor for certain time period
{
	float sum = 0;
	for (int i = 0; i < NUM_SAMPLES; i++)
	{																			// rawvoltage is multiplied by 2 as we are dividing ADC value by 2
		HAL_ADC_Start(&hadc1);													// due to limitations in ADC VREF and VCC required by sensor
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		rawVoltage = ((float)HAL_ADC_GetValue(&hadc1) / ADC_MAX) * ADC_VREF * 2;// IMPORTANT! rawvoltage must also be multiplied by a factor which
		sum += (rawVoltage - ZERO_CURRENT_VOLTAGE)/ACS712_SENSITIVITY;			// results in a 2.5V reading when no current is flowing
	}																			// May not be initially 2.5 due to ADC accuracy and resistor tolerance
	HAL_ADC_Stop(&hadc1);
	return (sum / NUM_SAMPLES);
}


float getFilteredVoltage()														// Gets average voltage value from sensor for certain time period
{
    float sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
        sum += (float)HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);														// The numbers 37.5 and 7.5 represent resistor values for the voltage
    return (sum / NUM_SAMPLES) * ( (37.5/7.5) * ADC_VREF / ADC_MAX);  			// divider present in my sensor. Could differ based on design
}


float getFilteredTemperature()													// Gets average temperature value from sensor for certain time period
{
	float sum = 0;
	for(int i=0;i<NUM_SAMPLES;i++){
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
		sum += (float)HAL_ADC_GetValue(&hadc3);
	}
	HAL_ADC_Stop(&hadc3);														// the *100 comes from the sensitivity of my sensor as it is
	return (sum/NUM_SAMPLES) * 100 * ADC_VREF / ADC_MAX;						// 10mV/celcius_degree . Could differ based on sensor used.
}


void checkBatteryTemp(float temp)												// function to protect against overheat
{
	if (temp > (mode == 0 ? CHARGING_MAX_TEMP : DISCHARGING_MAX_TEMP) )
	{																			// both charge and discharge are stopped in the case of overheat
		HAL_GPIO_WritePin(MOSFET_GPIO_PORT, CHARGING_MOSFET_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOSFET_GPIO_PORT, DISCHARGING_MOSFET_PIN, GPIO_PIN_RESET);

		sprintf(msg1,"System Stopped");
		sprintf(msg2,"Battery Temp High");
		HD44780_SetCursor(0,0);
		HD44780_PrintStr(msg1);
		HD44780_SetCursor(0,1);
		HD44780_PrintStr(msg2);

		while(temp > ( mode == 0 ? CHARGING_MAX_TEMP : DISCHARGING_MAX_TEMP))	// remains shut down until temp is back within the safety range
		{
			temp = getFilteredTemperature();
			HAL_Delay(500);
		}

		HD44780_Clear();														// clears LCD screen if back to normal
	}
}


void checkBatteryCurrent(float current)											// function to protect against overcurrent
{
	if (current > (mode == 0 ? MAX_CHARGE_CURRENT : MAX_DISCHARGE_CURRENT) )	// both charge and discharge are again stopped in the case of overheat
	{
		HAL_GPIO_WritePin(MOSFET_GPIO_PORT, CHARGING_MOSFET_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOSFET_GPIO_PORT, DISCHARGING_MOSFET_PIN, GPIO_PIN_RESET);

		sprintf(msg1,"System Stopped");
		if (mode == 0){
			sprintf(msg2,"Charge Overload");
		}else{
			sprintf(msg2,"Discharg Overload");
		}

		HD44780_SetCursor(0,0);
		HD44780_PrintStr(msg1);
		HD44780_SetCursor(0,1);
		HD44780_PrintStr(msg2);

		while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET);			// IMPORTANT! remains shut down until push button is pressed
		HAL_Delay(500);
		while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);
		HAL_Delay(500);

		HD44780_Clear();														// clears LCD screen if back to normal
	}
}

// STM32 Functions

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{


  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* Error_Handler_Debug */
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
