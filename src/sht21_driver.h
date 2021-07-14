/**
  ******************************************************************************
  * @file   : sht21_driver.h
  * @brief  : Header for main.c file.
  * This file contains the common defines of the application.
  ******************************************************************************
  * Author: Anastasis Vagenas -> Contact: anasvag29@gmail.com
  *
  *
  ******************************************************************************
  */
#ifndef __SHT2x_DRIV_H //Define to prevent recursive inclusion//
#define __SHT2x_DRIV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"
#include "i2c_driver.h"
#include <string.h>
#include <stdio.h>

/* Data structures  ----------------------------------------------------------*/
// Holds the measurements from the SHT21 //
typedef struct {
  float temperature;
  float humidity;

  uint16_t raw_temperature;
  uint16_t raw_humidity;

} SHT21_t;

/* Global variables ----------------------------------------------------------*/
//uint8_t state;

/* GPIO Defines --------------------------------------------------------------*/
#define SCL_Port 	GPIOB
#define SCL_Pin		LL_GPIO_PIN_6
#define SDA_Port 	GPIOB
#define SDA_Pin		LL_GPIO_PIN_7

/* USER APP CONSTANTS --------------------------------------------------------*/
#define CRC_ENABLE
//#define SHT21_USE_HAL

// SHT21 Return Codes //
#define SHT21_SUCCESS   0
#define SHT21_FAILURE   1

/* Peripheral Handlers -------------------------------------------------------*/
#ifdef SHT21_USE_HAL
extern I2C_HandleTypeDef hi2c1;		// I2C handle for HAL//
#else
extern I2C_TypeDef *i2c_handle;		// I2C1 handle //
#endif

/* CONSTANTS -----------------------------------------------------------------*/
// I2C Related and CRC //
#define SHT21_ADDRESS		      0x40
#define SHT21_WRITE_ADDRESS	  0x80
#define SHT21_READ_ADDRESS	  0x81
#define CRC_POLYNOMIAL 		    0x131

// Resolution related settings //
#define TEMP_RES		            65536
#define HUM_RES			            65536
#define TEMP_CALIBRATION_CONST	0
#define HUM_CALIBRATION_CONST	  0

/*------------------------- OPERATIONS -----------------------------*/
/*------------------------------------------------------------------*/
// Commands //
#define READ_TEMP_STRETCH	    0xe3
#define READ_HUM_STRETCH	    0xe5
#define READ_TEMP_NO_STRETCH	0xf3
#define READ_HUM_NO_STRETCH	  0xf5
#define WRITE_USER_REGISTER	  0xe6
#define READ_USER_REGISTER	  0xe7
#define RESET_SHT2X		        0xfe

// User register settings //
#define USR_REG_MASK		0xc7
#define ENABLE_HEATER		0x04
#define DISABLE_HEATER	0x00
#define DISABLE_OTP		  0x02
#define RH_12_TEM_14		0x00
#define RH_8_TEM_12		  0x01
#define RH_10_TEM_13		0x80
#define RH_11_TEM_11		0x81

/* Private function prototypes -----------------------------------------------*/
void SHT2x_CalcTemp(SHT21_t * data_str);
void SHT2x_CalcHumid(SHT21_t * data_str);
void SHT2x_CalcAll(SHT21_t * data_str);
void SHT2x_write_user_reg(uint8_t config);
uint8_t SHT2x_read_user_reg(void);
void SHT2x_reset(void);
void SHT2x_peripheral_config(void);
uint8_t SHT2x_init(uint8_t config);

#endif	//Define to prevent recursive inclusion//
