/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include<stdio.h>
//#include "Sensors.h"
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "adc_if.h"
#include "sys_conf.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "stdlib.h"

/* USER CODE BEGIN Includes */
/***************   BME280_VARAIBLES__***************/
float Temp, Press, Hum;
int ret=0;
/***************   DHT11_VARAIBLES__***************/
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

uint8_t Tmsg[7],Hmsg[7];

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;


/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRa application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/*!
 * Will be called each time a Radio IRQ is handled by the MAC layer
 *
 */
static void OnMacProcessNotify(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =           GetBatteryLevel,
  .GetTemperature =            GetTemperatureLevel,
  .GetUniqueId =               GetUniqueId,
  .GetDevAddr =                GetDevAddr,
  .OnMacProcess =              OnMacProcessNotify,
  .OnJoinRequest =             OnJoinRequest,
  .OnTxData =                  OnTxData,
  .OnRxData =                  OnRxData
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingPeriodicity =          LORAWAN_DEFAULT_PING_SLOT_PERIODICITY
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;


/* USER CODE BEGIN PV */
/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/**
  * @brief Specifies the state of the application LED
  */
static uint8_t AppLedStateOn = RESET;

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

void BME_280_Init(void)
{
	ret= BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
}

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_1 */

  BSP_LED_Init(LED_RED);
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);

  /* Get LoRa APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION:        V%X.%X.%X\r\n",
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW LoraWAN info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:   V%X.%X.%X\r\n",
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB2_SHIFT));
  APP_LOG(TS_OFF, VLEVEL_M, "\r\n\r\n\r\n----------LoRaWAN_End_Node_FreeRTOS----------\r\n\r\n\r\n");
  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks);

  LmHandlerConfigure(&LmHandlerParams);


  /* USER CODE END LoRaWAN_Init_2 */

  LmHandlerJoin(ActivationType);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_SetPeriod(&TxTimer,  APP_TX_DUTYCYCLE);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* send every time button is pushed */
    BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */
/* Note: Current the stm32wlxx_it.c generated by STM32CubeMX does not support BSP for PB in EXTI mode. */
/* In order to get a push button IRS by code automatically generated */
/* HAL_GPIO_EXTI_Callback is today the only available possibility. */
/* Using HAL_GPIO_EXTI_Callback() shortcuts the BSP. */
/* If users wants to go through the BSP, stm32wlxx_it.c should be updated  */
/* in the USER CODE SESSION of the correspondent EXTIn_IRQHandler() */
/* to call the BSP_PB_IRQHandler() or the HAL_EXTI_IRQHandler(&H_EXTI_n);. */
/* Then the below HAL_GPIO_EXTI_Callback() can be replaced by BSP callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case  BUTTON_SW1_PIN:
      /* Note: when "EventType == TX_ON_TIMER" this GPIO is not initialized */
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      break;
    default:
      break;
  }
}

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  if ((appData != NULL) || (params != NULL))
  {
    static const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };

    APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Indication ==========\r\n");
    APP_LOG(TS_OFF, VLEVEL_M, "###### D/L FRAME:%04d | SLOT:%s | PORT:%d | DR:%d | RSSI:%d | SNR:%d\r\n",
            params->DownlinkCounter, slotStrings[params->RxSlot], appData->Port, params->Datarate, params->Rssi, params->Snr);
    switch (appData->Port)
    {
      case LORAWAN_SWITCH_CLASS_PORT:
        /*this port switches the class*/
        if (appData->BufferSize == 1)
        {
          switch (appData->Buffer[0])
          {
            case 0:
            {
              LmHandlerRequestClass(CLASS_A);
              break;
            }
            case 1:
            {
              LmHandlerRequestClass(CLASS_B);
              break;
            }
            case 2:
            {
              LmHandlerRequestClass(CLASS_C);
              break;
            }
            default:
              break;
          }
        }
        break;
      case LORAWAN_USER_APP_PORT:
        if (appData->BufferSize == 1)
        {
          AppLedStateOn = appData->Buffer[0] & 0x01;
          if (AppLedStateOn == RESET)
          {
            APP_LOG(TS_OFF, VLEVEL_H,   "LED OFF\r\n");
            BSP_LED_Off(LED_RED) ;
          }
          else
          {
            APP_LOG(TS_OFF, VLEVEL_H, "LED ON\r\n");
            BSP_LED_On(LED_RED) ;
          }
        }
        break;

      default:

        break;
    }
  }
  /* USER CODE END OnRxData_1 */
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
	UTIL_TIMER_Time_t nextTxIn = 0;
	/**************************BME280 *************************************/
	uint16_t Tx_temp=0, Tx_press=0, Tx_hum=0;
	uint8_t temp_val[7], press_val[7], Hum_val[7] ;
	int  value=BME280_Measure();
	Tx_temp=Temp*100;
	Tx_press = (Press / 1000) *100;   //pressure in pascal converted to kilo pascal
	Tx_hum = Hum *100;
	if(value==0)
	{
		gcvt(Temp,7,temp_val);
		APP_LOG(TS_ON, VLEVEL_L, "BME280 Temperature  : ~%s\r\n",temp_val);
		gcvt(Press,7,press_val);
		APP_LOG(TS_ON, VLEVEL_L, "BME280 pressure  : ~%s\r\n",press_val );
		gcvt(Hum,7,Hum_val);
		APP_LOG(TS_ON, VLEVEL_L, "BME280 Humidity  : ~%s\r\n",Hum_val );
	}
/*****************************************DS18B20*************************************/

  uint16_t ds18b20_temp;
  ds18b20_temp =  DS18B20_Sensor_read();
  APP_LOG(TS_ON, VLEVEL_M, " DS18B20 Temperature ~%d \r\n", ds18b20_temp/100);
  uint16_t MQ6_value = MQ6_sensor();// MQ6 Sensor
  APP_LOG(TS_OFF, VLEVEL_L, "MQ6_value = %d\n\r", MQ6_value);
/************************************************************************************/

/******************************************DHT11 Sensor Data**************************/
	int16_t temperature = 0,humidity=0,Pressure = 65535,mode = 1,Battery =3300;
	DHT11_Start();
	Presence = DHT11_Check_Response();
	Rh_byte1 = DHT11_Read ();
	Rh_byte2 = DHT11_Read ();
	Temp_byte1 = DHT11_Read ();
	Temp_byte2 = DHT11_Read ();
	SUM = DHT11_Read();
    TEMP = Temp_byte1;
	RH = Rh_byte1;
	Temperature = (float) TEMP;
	Humidity = (float) RH;
	gcvt(Temperature,7,Tmsg);// Convert intp string
	gcvt(Humidity,7,Hmsg);
	temperature = Temperature * 100;
	humidity = Humidity * 100;
	/****** Print the Values on UART*****/
	APP_LOG(TS_ON, VLEVEL_L, "DHT11 Temperature  : %s\r\n",Tmsg);
	APP_LOG(TS_ON, VLEVEL_L, "DHT11 Humidity : %s\r\n",Hmsg);


#ifdef CAYENNE_LPP
  uint8_t channel = 0;
#else
  uint32_t i = 0;
#endif /* CAYENNE_LPP */
  AppData.Port = LORAWAN_USER_APP_PORT;
#ifdef CAYENNE_LPP
  CayenneLppReset();
  CayenneLppAddBarometricPressure(channel++, pressure);
  CayenneLppAddTemperature(channel++, temperature);
  CayenneLppAddRelativeHumidity(channel++, (uint16_t)(sensor_data.humidity));

  if ((LmHandlerParams.ActiveRegion != LORAMAC_REGION_US915) && (LmHandlerParams.ActiveRegion != LORAMAC_REGION_AU915)
      && (LmHandlerParams.ActiveRegion != LORAMAC_REGION_AS923))
  {
    CayenneLppAddDigitalInput(channel++, GetBatteryLevel());
    CayenneLppAddDigitalOutput(channel++, AppLedStateOn);
  }

  CayenneLppCopy(AppData.Buffer);
  AppData.BufferSize = CayenneLppGetSize();
#else  /* not CAYENNE_LPP */
/* App Buffer where the Data has been sent to the GateWay*/
  AppData.Buffer[i++] = mode;//Mode
  AppData.Buffer[i++] = (Battery >> 8) & 0xFF;/*first two bytes*/
  AppData.Buffer[i++] = Battery & 0xFF;/*default 3000 mv*/
  AppData.Buffer[i++] = (uint8_t)((temperature >> 8) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(temperature & 0xFF);
  AppData.Buffer[i++] = (uint8_t)((humidity >> 8) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(humidity & 0xFF);
  AppData.Buffer[i++] = (uint8_t)((Pressure >> 8) & 0xFF);/**Pressure**/
  AppData.Buffer[i++] = (uint8_t)(Pressure & 0xFF);/**Pressure**/

  AppData.BufferSize = i;
#endif /* CAYENNE_LPP */

  if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
  {

    APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
  }
  else if (nextTxIn > 0)
  {
    APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : %d second(s)\r\n", (nextTxIn / 1000));
  }

  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  if ((params != NULL))
  {
    /* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
    if (params->IsMcpsConfirm != 0)
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
      APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
              params->AppData.Port, params->Datarate, params->TxPower);

      APP_LOG(TS_OFF, VLEVEL_H, " | Tmsg TYPE:");
      if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
      {
        APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
      }
    }
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  if (joinParams != NULL)
  {
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
      }
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    }
  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/