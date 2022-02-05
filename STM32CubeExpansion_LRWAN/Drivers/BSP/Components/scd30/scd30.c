/**
  ******************************************************************************
  * @file    DrvSCD30.c
  * @brief   This file provides code for the configuration
  *          of the SCD30 instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "scd30.h"
#include "timeServer.h"
// #include "stm32l0xx_hal_def.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

#ifdef USE_SCD30

/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle4;
extern bool debug_flags;
/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 32 MHz */
#define I2C_TIMING    0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */ 
//#define I2C_TIMING      0x00B1112E /* 400 kHz with analog Filter ON, Rise Time 250ns, Fall Time 100ns */

uint16_t SCD30_MEASUREMENT_INTERVAL = 150;

#pragma anon_unions

struct __SCD30Descriptor
{
  uint16_t co2m;          /*!< High order 16 bit word of CO2 */
  uint16_t co2l;          /*!< Low  order 16 bit word of CO2 */
  uint16_t tempm;         /*!< High order 16 bit word of Temp */
  uint16_t templ;         /*!< Low order 16 bit word of Temp */
  uint16_t humm;          /*!< High order 16 bit word of Hum */
  uint16_t huml;          /*!< Low order 16 bit word of Hum */

  uint16_t ready;         /*!< 1 = ready, 0 = busy */

  union co2{
     uint32_t co2i;          /*!< 32 bit int of CO2 */
     float co2f;             /*!< float of CO2 concentration */
  };
 
  union temp{
    uint32_t tempi;         /*!< 32 bit int of Temp */
    float tempf;            /*!< float of Temp */
  };
 
  union hum{
    uint32_t humi;          /*!< 32 bit int of Hum */ 
    float humf;             /*!< float of Hum */  
  };
  
  

  uint16_t acode;         /*!< Article code number?? */
  uint8_t sn[24];         /*!< ASCII Serial Number */
};

static SCD30DescriptorTypeDef SCD30descr;


/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/* Private functions   ---------------------------------------------------------*/


uint16_t SCD30_readRegister(uint16_t registerAddress){
	
	int returnValue = 0;
	
	uint8_t regAddr[2];
	// uint8_t receive_buffer[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t receive_buffer[2] = {0,0};
	
	regAddr[1] = (0xFFu & registerAddress);
	regAddr[0] = (0xFFu & (registerAddress >> 8));
	
	// set the register pointer to the register wanted to read  
  uint32_t currentTime = TimerGetCurrentTime();
  while(HAL_I2C_Master_Transmit(&I2cHandle4, SCD30_BASE_ADDR, regAddr, 2, 1000) != HAL_OK)
  {
      if(TimerGetElapsedTime(currentTime) >= 1000)
      {
        returnValue =  -1;
        break;
      }
      if(HAL_I2C_GetError(&I2cHandle4) != HAL_I2C_ERROR_AF)
      {}
  }
  
  currentTime = TimerGetCurrentTime();
  while(HAL_I2C_Master_Receive(&I2cHandle4,SCD30_BASE_ADDR,receive_buffer, sizeof(receive_buffer)/sizeof(receive_buffer[0]), 1000) != HAL_OK)
  {
      if(TimerGetElapsedTime(currentTime) >= 1000)
      {
        returnValue = -1;
        break;
      }
      if(HAL_I2C_GetError(&I2cHandle4) != HAL_I2C_ERROR_AF)
      {}
  }
  
  
  /*
	
	if(HAL_I2C_Master_Transmit(&I2cHandle4, SCD30_BASE_ADDR, regAddr, 2 , 150) != 0x00U){
		returnValue = -1;
	}else{
		// receive the n x 8bit data into the receive_buffer
		HAL_I2C_Master_Receive(&I2cHandle4, SCD30_BASE_ADDR, receive_buffer, sizeof(receive_buffer)/sizeof(receive_buffer[0]), 150);
		

		returnValue = ((uint8_t)receive_buffer[1] << 8 | receive_buffer[0]);
	}
  
  */
  
  if(HAL_I2C_GetError(&I2cHandle4) == HAL_I2C_ERROR_NONE)
  {
    returnValue = ((uint8_t)receive_buffer[1] << 8 | receive_buffer[0]);
  }
  
  
  return returnValue;

}


uint8_t SCD30_calcCrc2b(uint16_t seed)
{
  uint8_t bit;                  // bit mask
  uint8_t crc = SCD30_CRC_INIT; // calculated checksum
  
  // calculates 8-Bit checksum with given polynomial
  crc ^= (seed >> 8) & 255;
  for(bit = 8; bit > 0; --bit)
  {
    if(crc & 0x80) crc = (crc << 1) ^ SCD30_POLYNOMIAL;
    else           crc = (crc << 1);
  }

  crc ^= seed & 255;
  for(bit = 8; bit > 0; --bit)
  {
    if(crc & 0x80) crc = (crc << 1) ^ SCD30_POLYNOMIAL;
    else           crc = (crc << 1);
  }
  
  return crc;  
}

void SCD30_init(void){
  
  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle4.Instance              = I2Cx;
  I2cHandle4.Init.Timing           = I2C_TIMING;
  I2cHandle4.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle4.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2cHandle4.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle4.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  I2cHandle4.Init.OwnAddress1      = 0xF0;
  I2cHandle4.Init.OwnAddress2      = 0xFE;
  
  if(HAL_I2C_Init(&I2cHandle4) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle4,I2C_ANALOGFILTER_ENABLE);
  /* Infinite loop */
 
  
}


SCD30ErrCodeType SCD30_checkCrc2b(uint16_t seed, uint8_t crcIn)
{
  uint8_t crcCalc = SCD30_calcCrc2b(seed);
  if(crcCalc != crcIn) return SCDcrcERROR;
  return SCDnoERROR;
}

SCD30ErrCodeType SCD30_startMeasurement(uint16_t baro)
{
    uint8_t i2cBuffer[64];
    HAL_StatusTypeDef respErrValue;
  
    i2cBuffer[0] = (0xFFu & (CMD_CONTINUOUS_MEASUREMENT >> 8));
    i2cBuffer[1] = (0xFFu & CMD_CONTINUOUS_MEASUREMENT);  
    
    i2cBuffer[2] = baro >> 8;
    i2cBuffer[3] = baro & 255;
    i2cBuffer[4] = SCD30_calcCrc2b(baro);

    respErrValue = HAL_I2C_Master_Transmit(&I2cHandle4, SCD30_BASE_ADDR, i2cBuffer, 5, 150);

    if(respErrValue == HAL_ERROR){
      return SCDnoAckERROR;
    }else if(respErrValue == HAL_TIMEOUT){
      return SCDtimeoutERROR;
    }else{
      return SCDnoERROR;
    }
    
}

SCD30ErrCodeType SDC30_stopMeasurement(void)
{
    uint8_t i2cBuffer[64];
    HAL_StatusTypeDef respErrValue;
    
    i2cBuffer[0] = (0xFFu & (CMD_STOP_CONT_MEASUREMENT >> 8));
    i2cBuffer[1] = (0xFFu & CMD_STOP_CONT_MEASUREMENT); 
  
    respErrValue = HAL_I2C_Master_Transmit(&I2cHandle4, SCD30_BASE_ADDR, i2cBuffer, 2, 150);
  
    if(respErrValue == HAL_ERROR){
      return SCDnoAckERROR;
    }else if(respErrValue == HAL_TIMEOUT){
      return SCDtimeoutERROR;
    }else{
      return SCDnoERROR;
    }
}


SCD30ErrCodeType SCD30_readMeasurement(void)
{
  uint8_t i2cBuffer[64];
  uint8_t regAddr[2];
  HAL_StatusTypeDef respErrValue;
  
  regAddr[0] = (0xFFu & (CMD_READ_MEASUREMENT >> 8));
  regAddr[1] = (0xFFu & CMD_READ_MEASUREMENT);  
  
  respErrValue = HAL_I2C_Master_Transmit(&I2cHandle4, SCD30_BASE_ADDR, regAddr, 2, 1000);
  
  if(respErrValue == HAL_ERROR){
    return SCDnoAckERROR;
  }else if(respErrValue == HAL_TIMEOUT){
    return SCDtimeoutERROR;
  }
  
  int i = 0;
  for(i = 0; i < sizeof(i2cBuffer)/sizeof(i2cBuffer[0]); i++){
    i2cBuffer[i] = 0;
  }
  
  respErrValue = HAL_I2C_Master_Receive(&I2cHandle4, SCD30_BASE_ADDR, i2cBuffer, 18, 1000);
  
  if(respErrValue == HAL_ERROR){
    return SCDnoAckERROR;
  }else if(respErrValue == HAL_TIMEOUT){
    return SCDtimeoutERROR;
  }
  
  uint16_t stat = (i2cBuffer[0] << 8) | i2cBuffer[1];
  SCD30descr.co2m = stat;
  uint8_t dat = SCD30_checkCrc2b(stat, i2cBuffer[2]);
  if(dat == SCDcrcERROR) return SCDcrcERRORv1;

  stat = (i2cBuffer[3] << 8) | i2cBuffer[4];
  SCD30descr.co2l = stat;
  dat = SCD30_checkCrc2b(stat, i2cBuffer[5]);
  if(dat == SCDcrcERROR) return SCDcrcERRORv2;
  
  stat = (i2cBuffer[6] << 8) | i2cBuffer[7];
  SCD30descr.tempm = stat;
  dat = SCD30_checkCrc2b(stat, i2cBuffer[8]);
  if(dat == SCDcrcERROR) return SCDcrcERRORv3;
  
  stat = (i2cBuffer[9] << 8) | i2cBuffer[10];
  SCD30descr.templ = stat;
  dat = SCD30_checkCrc2b(stat, i2cBuffer[11]);
  if(dat == SCDcrcERROR) return SCDcrcERRORv4;
  
  stat = (i2cBuffer[12] << 8) | i2cBuffer[13];
  SCD30descr.humm = stat;
  dat = SCD30_checkCrc2b(stat, i2cBuffer[14]);
  if(dat == SCDcrcERROR) return SCDcrcERRORv5;
  
  stat = (i2cBuffer[15] << 8) | i2cBuffer[16];
  SCD30descr.huml = stat;
  dat = SCD30_checkCrc2b(stat, i2cBuffer[17]);
  if(dat == SCDcrcERROR) return SCDcrcERRORv6;
  
  SCD30descr.co2i = (SCD30descr.co2m << 16) | SCD30descr.co2l ;
  SCD30descr.tempi = (SCD30descr.tempm << 16) | SCD30descr.templ ;
  SCD30descr.humi = (SCD30descr.humm << 16) | SCD30descr.huml ;
    
  
  return SCDnoERROR;
  
}


SCD30ErrCodeType SCD30_setMeasurementInterval(uint16_t time_sec)
{

    uint8_t i2cBuffer[64];
    HAL_StatusTypeDef respErrValue;
  
    if (time_sec < 2) time_sec = 2;
    if (time_sec > 1800) time_sec = 1800;

    i2cBuffer[0] = (0xFFu & (CMD_SET_MEASUREMENT_INTERVAL >> 8));
    i2cBuffer[1] = (0xFFu & CMD_SET_MEASUREMENT_INTERVAL);  
    
    i2cBuffer[2] = (0xFFu & (time_sec >> 8));
    i2cBuffer[3] = (0xFFu & time_sec);
    i2cBuffer[4] = SCD30_calcCrc2b(time_sec);

    respErrValue = HAL_I2C_Master_Transmit(&I2cHandle4, SCD30_BASE_ADDR, i2cBuffer, 5, 150);

    if(respErrValue == HAL_ERROR){
      return SCDnoAckERROR;
    }else if(respErrValue == HAL_TIMEOUT){
      return SCDtimeoutERROR;
    }else{
      return SCDnoERROR;
    }

}


SCD30ErrCodeType SCD30_setTemperatureOffs(uint16_t temp)
{
    uint8_t i2cBuffer[64];
    HAL_StatusTypeDef respErrValue;
    
    i2cBuffer[0] = CMD_SET_TEMPERATURE_OFFSET >> 8;
    i2cBuffer[1] = CMD_SET_TEMPERATURE_OFFSET & 0xFFu;
    i2cBuffer[2] = temp >> 8;
    i2cBuffer[3] = temp & 0xFFu;
    i2cBuffer[4] = SCD30_calcCrc2b(temp);
  
    respErrValue = HAL_I2C_Master_Transmit(&I2cHandle4, SCD30_BASE_ADDR, i2cBuffer, 5, 150);

    if(respErrValue == HAL_ERROR){
      return SCDnoAckERROR;
    }else if(respErrValue == HAL_TIMEOUT){
      return SCDtimeoutERROR;
    }else{
      return SCDnoERROR;
    }
}

SCD30ErrCodeType SCD30_getSerialNumber(void)
{
  uint8_t i2cBuffer[64];
  uint8_t regAddr[2];
  HAL_StatusTypeDef respErrValue;
  
  regAddr[0] = (0xFFu & (CMD_READ_SERIALNBR >> 8));
  regAddr[1] = (0xFFu & CMD_READ_SERIALNBR);  
  
  respErrValue = HAL_I2C_Master_Transmit(&I2cHandle4, SCD30_BASE_ADDR, regAddr, 2, 150);
  
  if(respErrValue == HAL_ERROR){
    return SCDnoAckERROR;
  }else if(respErrValue == HAL_TIMEOUT){
    return SCDtimeoutERROR;
  }
  
  int i = 0;
  for(i = 0; i < sizeof(SCD30descr.sn)/sizeof(SCD30descr.sn[0]); i++){
    SCD30descr.sn[i] = 0;
  }
  for(i = 0; i < sizeof(i2cBuffer)/sizeof(i2cBuffer[0]); i++){
    i2cBuffer[i] = 0;
  }
  
  respErrValue = HAL_I2C_Master_Receive(&I2cHandle4, SCD30_BASE_ADDR, i2cBuffer, SCD30_SN_SIZE, 150);
  
  if(respErrValue == HAL_ERROR){
    return SCDnoAckERROR;
  }else if(respErrValue == HAL_TIMEOUT){
    return SCDtimeoutERROR;
  }
  
  int t = 0;
  for(i = 0; i < SCD30_SN_SIZE; i += 3){
    uint16_t stat = (i2cBuffer[i] << 8) | i2cBuffer[i+1];
    SCD30descr.sn[i - t] = stat >> 8;
    SCD30descr.sn[i - t + 1] = stat & 0xFFu;
    uint8_t dat = SCD30_checkCrc2b(stat, i2cBuffer[i + 2]);
    t++;
    if(dat == SCDcrcERROR) return SCDcrcERRORv1;
    if(stat == 0) break;    
  }
  
  
  return SCDnoERROR;
}


float SCD30_getTemperatur(void)
{
  if(debug_flags==1)
	{	
		PPRINTF("\r\n");		
		PPRINTF("Temperature = %0.1f\r\n", SCD30descr.tempf);
	}
  
  return SCD30descr.tempf;
}


float SCD30_getCO2(void)
{
  if(debug_flags==1)
	{	
		PPRINTF("\r\n");		
		PPRINTF("CO2 = %0.2f\r\n", SCD30descr.co2f);
	}
  
  return SCD30descr.co2f;
}


float SCD30_getHumidity(void)
{
  if(debug_flags==1)
	{	
		PPRINTF("\r\n");		
		PPRINTF("Humidity = %0.2f\r\n", SCD30descr.humf);
	}
  
  return SCD30descr.humf;
}




/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#endif
