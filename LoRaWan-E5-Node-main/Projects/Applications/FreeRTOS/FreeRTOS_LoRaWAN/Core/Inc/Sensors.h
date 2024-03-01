/*
 * Sensors.h
 *
 *  Created on: Mar 1, 2024
 *      Author: Delta Iot
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "stdint.h"

uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);


/**************************** DHT *********************************/
void DHT11_Start();
uint8_t  DHT11_Check_Response();
uint8_t  DHT11_Read ();
//uint8_t DHT11_Read ();
//uint8_t DHT11_Read ();
//uint8_t DHT11_Read ();


/****************************MQ6*******************************/

int16_t MQ6_sensor(void);
uint16_t My_ADC_ReadChannels(uint32_t channel);


#endif /* INC_SENSORS_H_ */
