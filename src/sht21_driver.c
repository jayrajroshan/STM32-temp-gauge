/**
  ******************************************************************************
  * @file      : sht21_driver.c
  * @brief     : Source code for the SHT21 driver
  ******************************************************************************
  * Author: Anastasis Vagenas -> Contact: anasvag29@gmail.com
  *
  * This Library has routines in order to communicate with SHT21 sensor either via HAL
  * or Direct access (using LL or Direct access to registers).
  * For the I2C Driver user can choose either HAL or our own implementation in the i2c_driver.h
  *
  * ------- ROUTINES -------
  * I2C Routines:
  * - i2c_SHT2x_write_reg() -> Writes a SHT21 register
  * - i2c_SHT2x_write_cmd() -> Writes a write command to the I2C
  * - i2c_SHT2x_read_reg() -> Reads from the SHT21 register
  *
  * Temperature/Humidity Data Routines:
  * - SHT2x_CalcTemp() -> Reads the temperature data
  * - SHT2x_CalcHumid() -> Reads the humidity data
  * - SHT2x_CalcAll() -> Reads all of the above data
  *
  * Configuration Routines:
  * - SHT2x_write_user_reg()/SHT2x_read_user_reg -> Writes/Reads to the User register
  * - SHT2x_reset() -> Perform a reset command
  * - SHT2x_init() -> Initializes the sensor with default configuration
  * - SHT2x_peripheral_config() -> Initializes the I2C peripheral and the GPIOs used
  *
  * Extra Routines:
  * - SHT2x_CheckCrc() -> Checks the CRC of the received data (if it is enabled)
  *
  * ------- MAIN DATA STRUCTURES -------
  * SHT21_t -> Holds measurements for humidity/temperature
  *
  * ------- USAGE -------
  * In order to simply use the driver, set:
  * 1.An I2C handle based on the header file naming used
  * 2.Set the Defines for the GPIOs used (if they are a different board)
  *
  * Then call:
  * 1.SHT2x_peripheral_config() -> Initialize the peripherals
  * 2.SHT2x_init() -> Initialize the SHT21 with the basic configuration
  * 3.Use the Data Routines (i.e SHT2x_CalcAll()) to read data from the MPU6050.
  *
**/

/* Includes ------------------------------------------------------------------*/
#include "sht21_driver.h"

/* -- i2c_SHT2x_write_reg() --
 * Input: Register address, Register Write data
 * Return: None
 * Description:
 *
 * Internal function of the SHT2x driver, that uses I2C to write the specified data
 * to the specified register address of the SHT2x.
 *
 * */
static inline void i2c_SHT2x_write_reg(uint8_t reg_address, uint8_t data)
{
  #ifdef SHT21_USE_HAL
    HAL_I2C_Mem_Write(&hi2c1, SHT21_WRITE_ADDRESS, reg_address, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  #else
    uint8_t trx_data[] = {reg_address, data} ;
    i2c_write_data(i2c_handle, SHT21_WRITE_ADDRESS,trx_data, 2);
  #endif

}

/* -- i2c_SHT2x_write_cmd() --
 * Input: Command
 * Return: None
 * Description:
 *
 * Internal function of the SHT2x driver, that uses I2C to write a command to the I2C bus.
 * Follows Master_Transmit_Data format nut only for 1 byte.
 *
 * */
static inline void i2c_SHT2x_write_cmd(uint8_t data)
{
  #ifdef SHT21_USE_HAL
    HAL_I2C_Master_Transmit(&hi2c1, SHT21_WRITE_ADDRESS, &data, 1, 100);
  #else
    i2c_write_data(i2c_handle, SHT21_WRITE_ADDRESS,&data, 1);
  #endif

}

/* -- i2c_SHT2x_read_reg() --
 * Input: Register address, Register Write data , Number of data
 * Return: None
 * Description:
 *
 * Internal function of the SHT2x driver, that uses I2C to read the specified data
 * from to the specified register address of the SHT2x.
 * Can be used to read conversion data (temperature - humidity) or
 *
 * */
static inline void i2c_SHT2x_read_reg(uint8_t reg_address, uint8_t *data, uint8_t nbr_of_data)
{

  #ifdef SHT21_USE_HAL
    HAL_I2C_Master_Transmit(&hi2c1, SHT21_WRITE_ADDRESS, &reg_address, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, SHT21_READ_ADDRESS, data, nbr_of_data, 100);
  #else
    i2c_write_data(i2c_handle, SHT21_WRITE_ADDRESS, &reg_address, 1);
    i2c_recv_data(i2c_handle, SHT21_READ_ADDRESS, data, nbr_of_data);
  #endif

}

/* -- SHT2x_CheckCrc() --
 * Input: Register address, Register Write data , Number of data
 * Return: None
 * Description:
 *
 * Internal function of the SHT2x driver, that uses I2C to read the specified data
 * from to the specified register address of the SHT2x.
 * Can be used to read conversion data (temperature - humidity) or
 *
 * */
static inline uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//==============================================================================
{
  uint8_t crc = 0;
  uint8_t byteCtr;

  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr){
    crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit){
      if (crc & 0x80)
	crc = (crc << 1) ^ CRC_POLYNOMIAL;
      else
	crc = (crc << 1);
    }
  }

  if (crc != checksum)
    return 0;
  else
    return 1;
}

/* -- SHT2x_CalcTemp() --
 * Input: SHT21_t data structure
 * Return: None
 * Description:
 *
 * Performs a read sequence with the I2C , checks the CRC for transmission errors(if it is enabled)
 * and converts the integer value to a float.
 *
 *
 * */
void SHT2x_CalcTemp(SHT21_t *data_str)
{
  uint8_t data[3];

  i2c_SHT2x_read_reg(READ_TEMP_STRETCH, data, 3);

  #ifdef CRC_ENABLE
    // Error in transmission //
    if(SHT2x_CheckCrc(data, 2, data[2]) == 0){
      data_str->temperature = -400000.5;
      return;
    }
  #endif

  // Get the MSB and the MSB of the temperature //
  data_str->raw_temperature = (data[0] << 8);
  data_str->raw_temperature += data[1];
  data_str->raw_temperature &= ~0x0003;	  // Clear the status bits //

  // Calculate temperature //
  data_str->temperature = -46.85 + 175.72 * (((float) data_str->raw_temperature) / (float) TEMP_RES) + TEMP_CALIBRATION_CONST;
  
  // printf("Temperature is %f \n", data_str->temperature);


  return;
}

/* -- SHT2x_CalcHumid() --
 * Input: SHT21_t data structure
 * Return: None
 * Description:
 *
 * Performs a read sequence with the I2C , checks the CRC for transmission errors(if it is enabled)
 * and converts the integer value to a float.
 *
 *
 * */
void SHT2x_CalcHumid(SHT21_t *data_str)
{
  uint8_t data[3];

  i2c_SHT2x_read_reg(READ_HUM_STRETCH, data, 3);

  #ifdef CRC_ENABLE
    // Error in transmission //
    if(SHT2x_CheckCrc(data, 2, data[2]) == 0){
      data_str->humidity = -400000.5;
      return;
    }
  #endif

  // Get the MSB and the MSB of the humidity //
  data_str->raw_humidity = (data[0] << 8);
  data_str->raw_humidity += data[1];
  data_str->raw_humidity &= ~0x0003;	// Clear the status bits //

  // Calculate relative humidity //
  data_str->humidity = -6 + 125 * (((float) data_str->raw_humidity) / (float) HUM_RES) + HUM_CALIBRATION_CONST;

  // printf("Humidity is %f\n", data_str->humidity);

  return;
}

/* -- SHT2x_CalcAll() --
 * Input: SHT21_t data structure
 * Return: None
 * Description:
 *
 * Performs a read sequence with the I2C , checks the CRC for transmission errors(if it is enabled)
 * and converts the integer values to a floats.
 *
 * */
void SHT2x_CalcAll(SHT21_t *data_str)
{
  uint8_t data[6];

  i2c_SHT2x_read_reg(READ_HUM_STRETCH, data, 3);
  i2c_SHT2x_read_reg(READ_TEMP_STRETCH, data + 3, 3);

  #ifdef CRC_ENABLE
    // Error in transmission of humidity //
    if(SHT2x_CheckCrc(data, 2, data[2]) == 0){
      data_str->humidity = -400000.5;
      return;
    }

    // Error in transmission of temperature //
    if(SHT2x_CheckCrc(data + 3, 2, data[5]) == 0){
      data_str->temperature = -400000.5;
      return;
    }
  #endif

  // Get the MSB and the MSB of the humidity //
  data_str->raw_humidity = (data[0] << 8);
  data_str->raw_humidity += data[1];
  data_str->raw_humidity &= ~0x0003;   // Clear the status bits //

  // Calculate relative humidity //
  data_str->humidity = -6 + 125 * (((float) data_str->raw_humidity) / (float) HUM_RES) + HUM_CALIBRATION_CONST;

  // Get the MSB and the MSB of the temperature //
  data_str->raw_temperature = (data[3] << 8);
  data_str->raw_temperature += data[4];
  data_str->raw_temperature &= ~0x0003;	// Clear the status bits //

  // Calculate temperature //
  data_str->temperature = -46.85 + 175.72 * (((float) data_str->raw_temperature) / (float) TEMP_RES) + TEMP_CALIBRATION_CONST;

  return;
}

/* -- SHT2x_write_user_reg() --
 * Input: None
 * Return: None
 * Description:
 *
 * Utility function of the driver, used only for debugging.
 * Writes to the user register.
 *
 * */
void SHT2x_write_user_reg(uint8_t config)
{
  i2c_SHT2x_write_reg(WRITE_USER_REGISTER, config);
}

/* -- SHT2x_read_user_reg() --
 * Input: None
 * Return: None
 * Description:
 *
 * Utility function of the driver, used only for debugging.
 * Reads from the user register.
 *
 * */
uint8_t SHT2x_read_user_reg(void)
{
  uint8_t data;

  i2c_SHT2x_read_reg(READ_USER_REGISTER, &data, 1);

  return data;
}

/* -- SHT2x_reset() --
 * Input: None
 * Return: None
 * Description:
 *
 * Utility function of the driver, used only for debugging.
 * Sends a software reset to the sensor.
 *
 * */
void SHT2x_reset(void)
{
  // Write the Reset command //
  i2c_SHT2x_write_cmd(RESET_SHT2X);

  // Minimum delay after reset is 15ms //
  HAL_Delay(30);

}

/* -- SHT2x_init() --
 * Input: Configuration
 * Return: None
 * Description:
 *
 * Initialization function of the driver, resets the sensor
 */
 
uint8_t SHT2x_init(uint8_t config) {
  uint8_t temp;

  SHT2x_reset();
  i2c_SHT2x_write_reg(WRITE_USER_REGISTER, USR_REG_MASK & config);
  i2c_SHT2x_read_reg(READ_USER_REGISTER, &temp, 1);

  if(temp == (USR_REG_MASK & config))
    return SHT21_SUCCESS;
  else
    return SHT21_FAILURE;
}

/* -- SHT2x_peripheral_config() --
 * Input: None
 * Return: None
 * Description:
 *
 * Initializes and configures the GPIOs and the I2C peripheral
 *
 * */
void SHT2x_peripheral_config(void)
{
  // Enable the GPIO Port Clock //
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure the GPIOs //
  LL_GPIO_SetPinSpeed(SCL_Port, SCL_Pin, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(SCL_Port, SCL_Pin, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(SCL_Port, SCL_Pin, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(SCL_Port, SCL_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(SCL_Port, SCL_Pin, LL_GPIO_AF_4);

  LL_GPIO_SetPinSpeed(SDA_Port, SDA_Pin, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(SDA_Port, SDA_Pin, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(SDA_Port, SDA_Pin, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(SDA_Port, SDA_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(SDA_Port, SDA_Pin, LL_GPIO_AF_4);

  /* Peripheral clock enable */
  __HAL_RCC_I2C1_CLK_ENABLE();

  #ifdef SHT21_USE_HAL
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
      while(1);
    }
  #endif

  #ifndef SHT21_USE_HAL

    // Use I2C1 peripheral //
    i2c_handle = I2C1;

    // Reset the peripheral
    i2c_handle->CR1 |= I2C_CR1_SWRST;
    i2c_handle->CR1 &= ~I2C_CR1_SWRST;

    LL_I2C_SetMode(i2c_handle, LL_I2C_MODE_I2C);
    LL_I2C_DisableClockStretching(i2c_handle);
    LL_I2C_ConfigSpeed(i2c_handle,
		       HAL_RCC_GetPCLK1Freq(),
		       100000,
		       LL_I2C_DUTYCYCLE_2);

  #endif
}
