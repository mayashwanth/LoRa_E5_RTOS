/*
 * MQ6_Sensor.c
 *
 *  Created on: Mar 1, 2024
 *      Author: Delta Iot
 */



#include "Sensors.h"
#include "stdint.h"
#include "sys_app.h"
#include "stm32wlxx_hal.h"

 extern ADC_HandleTypeDef hadc;


int16_t ppm;
int16_t max_ppm=10000;
int16_t min_ppm=300;
uint16_t adc_val;

uint16_t My_ADC_ReadChannels(uint32_t channel)
{

	adc_val=0;

	int i;

	 ADC_ChannelConfTypeDef sConfig = {0};

	  MX_ADC_Init();

	  /* Start Calibration */
	  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Configure Regular Channel */
	  sConfig.Channel = channel;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_ADC_Start(&hadc) != HAL_OK)
	  {
	    /* Start Error */
	    Error_Handler();
	  }
	  /** Wait for end of conversion */
	  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);

	 for(i=0;i<100;i++)
	 {
		 adc_val+= HAL_ADC_GetValue(&hadc);
	 }

	 adc_val=adc_val/100;

	 adc_val = ( ((float)adc_val/4095) * (max_ppm-min_ppm) + min_ppm);

	  HAL_ADC_Stop(&hadc);

	  return adc_val;
}

int16_t MQ6_sensor(void)
{
	 ppm = My_ADC_ReadChannels(ADC_CHANNEL_6);
	 return ppm;
}
