/*
 * DS18B20_Sensor.c
 *
 *  Created on: Mar 1, 2024
 *      Author: Delta Iot
 */

#include "Sensors.h"
#include "stdint.h"
#include "sys_app.h"
#include "stm32wlxx_hal.h"
#include <string.h>


#define DS18B20_PORT    				GPIOB
#define DS18B20_PIN						GPIO_PIN_7
#define RESOLUTION_FACTOR_FOR_12_BIT	0.0625

uint8_t DPresence = 0;
uint8_t DTemp_byte1 = 0, DTemp_byte2 = 0;
float temp_data = 0;
int8_t str[10] = {0};



void DSet_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void DSet_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay(uint32_t us)
{
	volatile int i = 0;
	for(i= 0;i < (4*us);i++);
}
uint8_t DS18B20_Start(void)
{
	uint8_t Response = 0;
	DSet_Pin_Output(DS18B20_PORT, DS18B20_PIN);		//set pin as o/p
	HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);		//pin pulled to low
	delay(480);		//acc to datasheet

	DSet_Pin_Input(DS18B20_PORT, DS18B20_PIN);		//set pin as i/p
	delay(80);

	if(!(HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)))
		Response = 1;		//pin is low==presence pulse detected
	else
		Response =-1;

	delay(400);		//480us total

	return Response;
}

void DS18B20_Write(uint8_t data)
{
	DSet_Pin_Output(DS18B20_PORT, DS18B20_PIN);		//set pin as o/p
	for(int i=0;i<8;i++)
	{
		if((data&(1<<i))!=0)		//if bit high
		{
			//write 1
			DSet_Pin_Output(DS18B20_PORT, DS18B20_PIN);		//set pin as o/p
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);		//pull pin low
			delay(1);

			DSet_Pin_Input(DS18B20_PORT, DS18B20_PIN);		//set pin as i/p
			delay(50);		//wait for 60 us
		}
		else		//if bit low
		{
			//write 0
			DSet_Pin_Output(DS18B20_PORT, DS18B20_PIN);		//set pin as i/p
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);		//pull pin low
			delay(50);		//wait for 60 us

			DSet_Pin_Input(DS18B20_PORT, DS18B20_PIN);		//set pin as i/p
		}
	}
}

uint8_t DS18B20_Read(void)
{
	uint8_t Value = 0;
	DSet_Pin_Input(DS18B20_PORT, DS18B20_PIN);		//set pin as i/p
	for(int i=0;i<8;i++)
	{
		DSet_Pin_Output(DS18B20_PORT, DS18B20_PIN);		//set pin as o/p
		HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);		//pull data pin low
		delay(2);		//2us delay

		DSet_Pin_Input(DS18B20_PORT, DS18B20_PIN);		//set pin as i/p
		if(HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))		//if pin high
		{
			Value |= 1<<i;		//read =1
		}
		delay(60);		//wait 60us
	}
	return Value;
}

uint16_t DS18B20_Sensor_read()
{
	  uint16_t Dtemperature= 0,Dhumidity = 65535,Dpressure =6;
	  DPresence = DS18B20_Start();
	  HAL_Delay (1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t
	  HAL_Delay (800);

	  DPresence = DS18B20_Start();
	  HAL_Delay(1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad

	  DTemp_byte1 = DS18B20_Read();
	  DTemp_byte2 = DS18B20_Read();

	  temp_data = (float)(((DTemp_byte2<<8)|(DTemp_byte1))*RESOLUTION_FACTOR_FOR_12_BIT);

	  Dtemperature = temp_data  * 100;

	  sprintf(str,"%0.2f\r\n", temp_data);

	//  temperature1 = temp_data  * 100;

	  return Dtemperature;
}
