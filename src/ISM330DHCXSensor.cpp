/* Includes-----------------------------------------------------------------------------------*/
#include "ISM330DHCXSensor.h"

/* Class Implementations ---------------------------------------------------------------------*/

/** Constructor I2C
 *  @param i2c object
 *  @param address the address of the component's instance
 */
ISM330DHCXSensor::ISM330DHCXSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  reg_ctx.write_reg = ISM330DHCX_io_write;
  reg_ctx.read_reg = ISM330DHCX_io_read;
  reg_ctx.handle = (void *) this;
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

/** Constructor I2C
 *  @param spi object
 *  @param cs_pin the chip select pin
 *  @param spi_speed the SPI speed
 */
ISM330DHCXSensor::ISM330DHCXSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = ISM330DHCX_io_write;
  reg_ctx.read_reg = ISM330DHCX_io_read;
  reg_ctx.handle = (void *) this;
  dev_i2c = NULL;
  address = 0U;
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

/**
 *  @brief Initialize the sensor
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::Init()
{
  /* SW reset */
  if (ism330dhcx_reset_set(&(reg_ctx), PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
     access with a serial interface */
  if (ism330dhcx_auto_increment_set(&(reg_ctx), PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable BDU */
  if (ism330dhcx_block_data_update_set(&(reg_ctx), PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* FIFO mode selection */
  if (ism330dhcx_fifo_mode_set(&(reg_ctx), ISM330DHCX_BYPASS_MODE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* ACCELEROMETER Output data rate selection - power down */
  if (ism330dhcx_xl_data_rate_set(&(reg_ctx), ISM330DHCX_XL_ODR_OFF) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* ACCELEROMETER Full scale selection */
  if (ism330dhcx_xl_full_scale_set(&(reg_ctx), ISM330DHCX_2g) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* GYROSCOPE Output data rate selection - power down */
  if (ism330dhcx_gy_data_rate_set(&(reg_ctx), ISM330DHCX_GY_ODR_OFF) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* GYROSCOPE Full scale selection */
  if (ism330dhcx_gy_full_scale_set(&(reg_ctx), ISM330DHCX_2000dps) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;

  return ISM330DHCX_OK;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::begin()
{
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }

  if (Init() != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::end()
{
  /* Disable both acc and gyro */
  if (ACC_Disable() != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (GYRO_Disable() != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset CS configuration */
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, INPUT);
  }

  return ISM330DHCX_OK;
}

/**
 * @brief Read component ID
 * @param Id pointer to store the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ReadID(uint8_t *Id)
{
  if (ism330dhcx_device_id_get(&(reg_ctx), Id) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief Enabled the ISM330DHCX accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Enable()
{
  if (acc_is_enabled == 1U) {
    return ISM330DHCX_OK;
  }

  /*ODR selection*/
  if (ism330dhcx_xl_data_rate_set(&(reg_ctx), ISM330DHCX_XL_ODR_104Hz) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  acc_is_enabled = 1U;
  return ISM330DHCX_OK;
}

/**
 * @brief Disable the ISM330DHCX accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Disable()
{
  if (acc_is_enabled == 0U) {
    return ISM330DHCX_ERROR;
  }

  /*ODR selection*/
  if (ism330dhcx_xl_data_rate_set(&(reg_ctx), ISM330DHCX_XL_ODR_OFF) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  acc_is_enabled = 0U;

  return ISM330DHCX_OK;
}

/**
 * @brief Get the ISM330DHCX accelerometer sensitivity
 * @param Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_GetSensitivity(float *Sensitivity)
{
  ism330dhcx_fs_xl_t full_scale;
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;

  /**Read actual full scale selection from sensor  */
  if (ism330dhcx_xl_full_scale_get(&reg_ctx, &full_scale) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /*Store the value of Sensitivity based on actual full scale*/
  switch (full_scale) {
    case ISM330DHCX_2g:
      *Sensitivity = ISM330DHCX_ACC_SENSITIVITY_FS_2G;
      break;

    case ISM330DHCX_4g:
      *Sensitivity = ISM330DHCX_ACC_SENSITIVITY_FS_4G;
      break;

    case ISM330DHCX_8g:
      *Sensitivity = ISM330DHCX_ACC_SENSITIVITY_FS_8G;
      break;

    case ISM330DHCX_16g:
      *Sensitivity = ISM330DHCX_ACC_SENSITIVITY_FS_16G;
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief Get the ISM330DHCX accelerometer sensor Output Data Rate
 * @param Odr pointer
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_GetOutputDataRate(float *Odr)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;
  ism330dhcx_odr_xl_t odr_low_level;

  /* Get current output data rate */
  if (ism330dhcx_xl_data_rate_get(&reg_ctx, &odr_low_level) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  switch (odr_low_level) {
    case ISM330DHCX_XL_ODR_OFF:
      *Odr = 0.0f;
      break;

    case ISM330DHCX_XL_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case ISM330DHCX_XL_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case ISM330DHCX_XL_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case ISM330DHCX_XL_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case ISM330DHCX_XL_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case ISM330DHCX_XL_ODR_417Hz:
      *Odr = 417.0f;
      break;

    case ISM330DHCX_XL_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case ISM330DHCX_XL_ODR_1667Hz:
      *Odr = 1667.0f;
      break;

    case ISM330DHCX_XL_ODR_3333Hz:
      *Odr = 3333.0f;
      break;

    case ISM330DHCX_XL_ODR_6667Hz:
      *Odr = 6667.0f;
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Set the ISM330DHCX accelerometer Output Data Rate
 * @param Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetOutputDataRate(float Odr)
{
  ism330dhcx_odr_xl_t new_odr;

  new_odr = (Odr <=   12.5f) ? ISM330DHCX_XL_ODR_12Hz5
            : (Odr <=   26.0f) ? ISM330DHCX_XL_ODR_26Hz
            : (Odr <=   52.0f) ? ISM330DHCX_XL_ODR_52Hz
            : (Odr <=  104.0f) ? ISM330DHCX_XL_ODR_104Hz
            : (Odr <=  208.0f) ? ISM330DHCX_XL_ODR_208Hz
            : (Odr <=  417.0f) ? ISM330DHCX_XL_ODR_417Hz
            : (Odr <=  833.0f) ? ISM330DHCX_XL_ODR_833Hz
            : (Odr <= 1667.0f) ? ISM330DHCX_XL_ODR_1667Hz
            : (Odr <= 3333.0f) ? ISM330DHCX_XL_ODR_3333Hz
            :                    ISM330DHCX_XL_ODR_6667Hz;

  /* Output data rate selection */
  if (ism330dhcx_xl_data_rate_set(&(reg_ctx), new_odr) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief Get the ISM330DHCX accelerometer sensor full scale
 * @param Fullscale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_GetFullScale(int32_t *FullScale)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;
  ism330dhcx_fs_xl_t fs_low_level;

  /* Read actual full scale selection from sensor */
  if (ism330dhcx_xl_full_scale_get(&(reg_ctx), &fs_low_level) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  switch (fs_low_level) {
    case ISM330DHCX_2g:
      *FullScale =  2;
      break;

    case ISM330DHCX_4g:
      *FullScale =  4;
      break;

    case ISM330DHCX_8g:
      *FullScale =  8;
      break;

    case ISM330DHCX_16g:
      *FullScale = 16;
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Set the ISM330DHCX accelerometer sensor full scale
 * @param fullscale the fullscale to be set
 * @retval  0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetFullScale(int32_t FullScale)
{
  ism330dhcx_fs_xl_t new_fs;

  new_fs = (FullScale <= 2) ? ISM330DHCX_2g
           : (FullScale <= 4) ? ISM330DHCX_4g
           : (FullScale <= 8) ? ISM330DHCX_8g
           :                    ISM330DHCX_16g;

  if (ism330dhcx_xl_full_scale_set(&(reg_ctx), new_fs) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief Get the ISM330DHCX accelerometer sensor raw axes
 * @param value pointer where the raw values are written
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_GetAxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /*Read raw data values */
  if (ism330dhcx_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /*Formatting data */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return ISM330DHCX_OK;
}

/**
 * @brief Get the ISM330DHCX accelerometer sensor axes
 * @param acceleration pointer where the axes are written
 * @retval 0 in case of success, an error code otherwise
*/
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_GetAxes(int32_t *Acceleration)
{
  float sensitivity = 0.0f;
  int16_t data_raw[3];

  /* Get actualSensitivity */
  if (ACC_GetSensitivity(&sensitivity) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /*Get Data Raw*/
  if (ACC_GetAxesRaw(data_raw) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /*Calculate data */
  Acceleration[0] = (int32_t)((float) data_raw[0] * sensitivity);
  Acceleration[1] = (int32_t)((float) data_raw[1] * sensitivity);
  Acceleration[2] = (int32_t)((float) data_raw[2] * sensitivity);

  return ISM330DHCX_OK;
}

/**
 * @brief Enabled the ISM330DHCX gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_Enable()
{
  /*Check if the component is already enabled*/
  if (gyro_is_enabled == 1U) {
    return ISM330DHCX_OK;
  }

  /*Output data rate selection */
  if (ism330dhcx_gy_data_rate_set(&reg_ctx, ISM330DHCX_GY_ODR_104Hz) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  gyro_is_enabled = 1U;

  return ISM330DHCX_OK;
}

/**
 * @brief Disable the ISM330DHCX gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_Disable()
{
  /*Check if the component is already disabled */
  if (gyro_is_enabled == 0U) {
    return ISM330DHCX_ERROR;
  }

  /*Output data rate selection: off */
  if (ism330dhcx_gy_data_rate_set(&reg_ctx, ISM330DHCX_GY_ODR_OFF) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  gyro_is_enabled = 0U;

  return ISM330DHCX_OK;
}

/**
 * @brief Get the ISM330DHCX gyroscope sensitivity
 * @param Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_GetSensitivity(float *Sensitivity)
{
  ism330dhcx_fs_g_t full_scale;
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;

  if (ism330dhcx_gy_full_scale_get(&reg_ctx, &full_scale) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  switch (full_scale) {
    case ISM330DHCX_125dps:
      *Sensitivity = ISM330DHCX_GYRO_SENSITIVITY_FS_125DPS;
      break;
    case ISM330DHCX_250dps:
      *Sensitivity = ISM330DHCX_GYRO_SENSITIVITY_FS_250DPS;
      break;
    case ISM330DHCX_500dps:
      *Sensitivity = ISM330DHCX_GYRO_SENSITIVITY_FS_500DPS;
      break;
    case ISM330DHCX_1000dps:
      *Sensitivity = ISM330DHCX_GYRO_SENSITIVITY_FS_1000DPS;
      break;
    case ISM330DHCX_2000dps:
      *Sensitivity = ISM330DHCX_GYRO_SENSITIVITY_FS_2000DPS;
      break;
    case ISM330DHCX_4000dps:
      *Sensitivity = ISM330DHCX_GYRO_SENSITIVITY_FS_4000DPS;
      break;
    default:
      ret = ISM330DHCX_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief Get the ISM330DHCX gyroscope sensor Output Data Rate
 * @param Odr pointer
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_GetOutputDataRate(float *Odr)
{
  ism330dhcx_odr_g_t odr_low_level;
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;

  if (ism330dhcx_gy_data_rate_get(&reg_ctx, &odr_low_level) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  switch (odr_low_level) {
    case ISM330DHCX_GY_ODR_OFF:
      *Odr = 0.0f;
      break;

    case ISM330DHCX_GY_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case ISM330DHCX_GY_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case ISM330DHCX_GY_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case ISM330DHCX_GY_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case ISM330DHCX_GY_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case ISM330DHCX_GY_ODR_417Hz:
      *Odr = 417.0f;
      break;

    case ISM330DHCX_GY_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case ISM330DHCX_GY_ODR_1667Hz:
      *Odr =  1667.0f;
      break;

    case ISM330DHCX_GY_ODR_3333Hz:
      *Odr =  3333.0f;
      break;

    case ISM330DHCX_GY_ODR_6667Hz:
      *Odr =  6667.0f;
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief Set the ISM330DHCX gyroscope Output Data Rate
 * @param Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_SetOutputDataRate(float Odr)
{
  ism330dhcx_odr_g_t new_odr;

  new_odr = (Odr <=   12.5f) ? ISM330DHCX_GY_ODR_12Hz5
            : (Odr <=   26.0f) ? ISM330DHCX_GY_ODR_26Hz
            : (Odr <=   52.0f) ? ISM330DHCX_GY_ODR_52Hz
            : (Odr <=  104.0f) ? ISM330DHCX_GY_ODR_104Hz
            : (Odr <=  208.0f) ? ISM330DHCX_GY_ODR_208Hz
            : (Odr <=  417.0f) ? ISM330DHCX_GY_ODR_417Hz
            : (Odr <=  833.0f) ? ISM330DHCX_GY_ODR_833Hz
            : (Odr <= 1667.0f) ? ISM330DHCX_GY_ODR_1667Hz
            : (Odr <= 3333.0f) ? ISM330DHCX_GY_ODR_3333Hz
            :                    ISM330DHCX_GY_ODR_6667Hz;

  /* Output data rate selection */
  if (ism330dhcx_gy_data_rate_set(&reg_ctx, new_odr) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief Get the ISM330DHCX gyroscope sensor full scale
 * @param Fullscale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_GetFullScale(int32_t *FullScale)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;
  ism330dhcx_fs_g_t fs_low_level;

  /* Read actual full scale */
  if (ism330dhcx_gy_full_scale_get(&reg_ctx, &fs_low_level) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  switch (fs_low_level) {
    case ISM330DHCX_125dps:
      *FullScale =  125;
      break;

    case ISM330DHCX_250dps:
      *FullScale =  250;
      break;

    case ISM330DHCX_500dps:
      *FullScale =  500;
      break;

    case ISM330DHCX_1000dps:
      *FullScale = 1000;
      break;

    case ISM330DHCX_2000dps:
      *FullScale = 2000;
      break;

    case ISM330DHCX_4000dps:
      *FullScale = 4000;
      break;
    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Set the ISM330DHCX gyroscope sensor full scale
 * @param fullscale the fullscale to be set
 * @retval  0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_SetFullScale(int32_t FullScale)
{
  ism330dhcx_fs_g_t new_fs;

  new_fs = (FullScale <= 125) ? ISM330DHCX_125dps
           : (FullScale <= 250) ? ISM330DHCX_250dps
           : (FullScale <= 500) ? ISM330DHCX_500dps
           : (FullScale <= 1000) ? ISM330DHCX_1000dps
           : (FullScale <= 2000) ? ISM330DHCX_2000dps
           :                       ISM330DHCX_4000dps;

  if (ism330dhcx_gy_full_scale_set(&reg_ctx, new_fs) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }


  return ISM330DHCX_OK;
}

/**
 * @brief Get the ISM330DHCX gyroscope sensor raw axes
 * @param value pointer where the raw values are written
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_GetAxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values */
  if (ism330dhcx_angular_rate_raw_get(&reg_ctx, data_raw.u8bit) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Format the data */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return ISM330DHCX_OK;
}

/**
 * @brief Get the ISM330DHCX gyroscope sensor axes
 * @param acceleration pointer where the axes are written
 * @retval 0 in case of success, an error code otherwise
*/
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_GetAxes(int32_t *AngularRate)
{
  float sensitivity;
  int16_t raw_data[3];

  /*Read raw data values */
  if (GYRO_GetAxesRaw(raw_data) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /*Read actual sensitivity */
  if (GYRO_GetSensitivity(&sensitivity) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  AngularRate[0] = (int32_t)((float) raw_data[0]  * sensitivity);
  AngularRate[1] = (int32_t)((float) raw_data[1]  * sensitivity);
  AngularRate[2] = (int32_t)((float) raw_data[2]  * sensitivity);

  return ISM330DHCX_OK;
}

/**
 * @brief  Get the IIS2MDC register value for magnetic sensor
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ReadReg(uint8_t reg, uint8_t *data)
{
  if (ism330dhcx_read_reg(&reg_ctx, reg, data, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Set the IIS2MDC register value for magnetic sensor
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::WriteReg(uint8_t reg, uint8_t data)
{
  if (ism330dhcx_write_reg(&reg_ctx, reg, &data, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the ISM330DHCX Status of the Event
 *  @param Status pointer to a structure Event_Status_t
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_GetEventStatus(ISM330DHCX_Event_Status_t *Status)
{
  ism330dhcx_wake_up_src_t wake_up_src;
  ism330dhcx_tap_src_t tap_src;
  ism330dhcx_d6d_src_t d6d_src;
  ism330dhcx_md1_cfg_t md1_cfg;
  ism330dhcx_md2_cfg_t md2_cfg;
  ism330dhcx_int1_ctrl_t int1_ctrl;

  (void)memset((void *)Status, 0x0, sizeof(ISM330DHCX_Event_Status_t));

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_WAKE_UP_SRC, (uint8_t *)&wake_up_src, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_TAP_SRC, (uint8_t *)&tap_src, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_D6D_SRC, (uint8_t *)&d6d_src, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_MD1_CFG, (uint8_t *)&md1_cfg, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_MD2_CFG, (uint8_t *)&md2_cfg, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_INT1_CTRL, (uint8_t *)&int1_ctrl, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if ((md1_cfg.int1_ff == 1U) || (md2_cfg.int2_ff == 1U)) {
    if (wake_up_src.ff_ia == 1U) {
      Status->FreeFallStatus = 1;
    }
  }

  if ((md1_cfg.int1_wu == 1U) || (md2_cfg.int2_wu == 1U)) {
    if (wake_up_src.wu_ia == 1U) {
      Status->WakeUpStatus = 1;
    }
  }

  if ((md1_cfg.int1_single_tap == 1U) || (md2_cfg.int2_single_tap == 1U)) {
    if (tap_src.single_tap == 1U) {
      Status->TapStatus = 1;
    }
  }

  if ((md1_cfg.int1_double_tap == 1U) || (md2_cfg.int2_double_tap == 1U)) {
    if (tap_src.double_tap == 1U) {
      Status->DoubleTapStatus = 1;
    }
  }

  if ((md1_cfg.int1_6d == 1U) || (md2_cfg.int2_6d == 1U)) {
    if (d6d_src.d6d_ia == 1U) {
      Status->D6DOrientationStatus = 1;
    }
  }

  return ISM330DHCX_OK;
}

/**
 * @brief Set Self Test of ISM330DHCX accelerometer
 * @param Status the value to set
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Set_SelfTest(uint8_t Status)
{
  ism330dhcx_st_xl_t reg;

  reg = (Status == 0U)  ? ISM330DHCX_XL_ST_DISABLE
        : (Status == 1U)  ? ISM330DHCX_XL_ST_POSITIVE
        : (Status == 2U)  ? ISM330DHCX_XL_ST_NEGATIVE
        :                   ISM330DHCX_XL_ST_DISABLE;

  if (ism330dhcx_xl_self_test_set(&reg_ctx, reg) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the Self Test value of ISM330DHCX accelerometer
 *  @param SelfStatus pointer where the value of self_test is written
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Get_SelfTest(uint8_t *SelfStatus)
{
  ism330dhcx_st_xl_t val;
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;

  if (ism330dhcx_xl_self_test_get(&reg_ctx, &val) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  switch (val) {
    case ISM330DHCX_XL_ST_DISABLE:
      *SelfStatus = 0U;
      break;

    case ISM330DHCX_XL_ST_POSITIVE:
      *SelfStatus = 1U;
      break;

    case ISM330DHCX_XL_ST_NEGATIVE:
      *SelfStatus = 2U;
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Get the ISM330DHCX accelerometer sensor data ready bit value
 * @param Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Get_DRDY_Status(uint8_t *Status)
{
  if (ism330dhcx_xl_flag_data_ready_get(&reg_ctx, Status) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief Set HP filter for ISM330DHCX accelerometer
 * @param CutOff value to set frequency
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Enable_HP_Filter(ism330dhcx_hp_slope_xl_en_t CutOff)
{
  if (ism330dhcx_xl_hp_path_on_out_set(&reg_ctx, CutOff) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }
  return ISM330DHCX_OK;
}

/**
 * @brief Set Self Test of ISM330DHCX gyroscope
 * @param Status the value to set
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_Set_SelfTest(uint8_t Status)
{
  ism330dhcx_st_g_t reg;

  reg = (Status == 0U)  ? ISM330DHCX_GY_ST_DISABLE
        : (Status == 1U)  ? ISM330DHCX_GY_ST_POSITIVE
        : (Status == 3U)  ? ISM330DHCX_GY_ST_NEGATIVE
        :                ISM330DHCX_GY_ST_DISABLE;


  if (ism330dhcx_gy_self_test_set(&reg_ctx, reg) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the Self Test value of ISM330DHCX gyroscope
 *  @param SelfStatus pointer where the value of self_test is written
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_Get_SelfTest(uint8_t *SelfStatus)
{
  ism330dhcx_st_g_t val;
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;

  if (ism330dhcx_gy_self_test_get(&reg_ctx, &val) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  switch (val) {
    case ISM330DHCX_GY_ST_DISABLE:
      *SelfStatus = 0U;
      break;

    case ISM330DHCX_GY_ST_POSITIVE:
      *SelfStatus = 1U;
      break;

    case ISM330DHCX_GY_ST_NEGATIVE:
      *SelfStatus = 3U;
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Get the ISM330DHCX gyroscope sensor data ready bit value
 * @param Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_Get_DRDY_Status(uint8_t *Status)
{
  if (ism330dhcx_gy_flag_data_ready_get(&reg_ctx, Status) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief Set HP filter for ISM330DHCX gyroscope
 * @param CutOff value to set frequency
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_Enable_HP_Filter(ism330dhcx_hpm_g_t CutOff)
{
  if (ism330dhcx_gy_hp_path_internal_set(&reg_ctx, CutOff) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }
  return ISM330DHCX_OK;
}

/*
 *  @brief Enable the detection of the Free Fall event for ISM330DHCX accelerometer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_EnableFreeFallDetection(ISM330DHCX_SensorIntPin_t IntPin)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (ACC_SetOutputDataRate(417.0f) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Full scale selection */
  if (ACC_SetFullScale(2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* FF_DUR setting */
  if (ism330dhcx_ff_dur_set(&reg_ctx, 0x06) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* WAKE_DUR setting */
  if (ism330dhcx_wkup_dur_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* SLEEP_DUR setting */
  if (ism330dhcx_act_sleep_dur_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* FF_THS setting */
  if (ism330dhcx_ff_threshold_set(&reg_ctx, ISM330DHCX_FF_TSH_312mg) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM330DHCX_INT1_PIN:
      if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val1.md1_cfg.int1_ff = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    case ISM330DHCX_INT2_PIN:
      if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val2.md2_cfg.int2_ff = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the Free Fall event for ISM330DHCX accelerometer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_DisableFreeFallDetection()
{
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Disable free fall event on both INT1 and INT2 pins */
  if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val1.md1_cfg.int1_ff = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val2.md2_cfg.int2_ff = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* FF_DUR setting */
  if (ism330dhcx_ff_dur_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* FF_THS setting */
  if (ism330dhcx_ff_threshold_set(&reg_ctx, ISM330DHCX_FF_TSH_156mg) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Threshold for the Free Fall event
 *  @param Threshold integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetFreeFallThreshold(uint8_t Threshold)
{
  if (ism330dhcx_ff_threshold_set(&reg_ctx, (ism330dhcx_ff_ths_t) Threshold) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Duration for the Free Fall event
 *  @param Duration integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetFreeFallDuration(uint8_t Duration)
{
  if (ism330dhcx_ff_dur_set(&reg_ctx, Duration) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Enable the detection of the Wake Up event
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_EnableWakeUpDetection(ISM330DHCX_SensorIntPin_t IntPin)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (ACC_SetOutputDataRate(417.0f) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Full scale selection */
  if (ACC_SetFullScale(2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* WAKE_DUR setting */
  if (ism330dhcx_wkup_dur_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Set wake up threshold. */
  if (ism330dhcx_wkup_threshold_set(&reg_ctx, 0x02) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable wake up event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM330DHCX_INT1_PIN:
      if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val1.md1_cfg.int1_wu = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    case ISM330DHCX_INT2_PIN:
      if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val2.md2_cfg.int2_wu = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the Wake Up event
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_DisableWakeUpDetection()
{
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Disable wake up event on both INT1 and INT2 pins */
  if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val1.md1_cfg.int1_wu = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val2.md2_cfg.int2_wu = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset wake up threshold. */
  if (ism330dhcx_wkup_threshold_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* WAKE_DUR setting */
  if (ism330dhcx_wkup_dur_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Threshold for the Wake Up event
 *  @param Threshold integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetWakeUpThreshold(uint8_t Threshold)
{
  /* Set wake up threshold */
  if (ism330dhcx_wkup_threshold_set(&reg_ctx, Threshold) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Duration for the Wake Up event
 *  @param Duration integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetWakeUpDuration(uint8_t Duration)
{
  /* Set wake up duration */
  if (ism330dhcx_wkup_dur_set(&reg_ctx, Duration) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Enable the detection of the Single Tap event
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_EnableSingleTapDetection(ISM330DHCX_SensorIntPin_t IntPin)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (ACC_SetOutputDataRate(417.0f) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Full scale selection */
  if (ACC_SetFullScale(2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_x_set(&reg_ctx, PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_y_set(&reg_ctx, PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_z_set(&reg_ctx, PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Set tap threshold. */
  if (ism330dhcx_tap_threshold_x_set(&reg_ctx, 0x09) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_tap_threshold_y_set(&reg_ctx, 0x09) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_tap_threshold_z_set(&reg_ctx, 0x09) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Set tap shock time window. */
  if (ism330dhcx_tap_shock_set(&reg_ctx, 0x02) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Set tap quiet time window. */
  if (ism330dhcx_tap_quiet_set(&reg_ctx, 0x01) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /*Disable double tap*/
  ism330dhcx_tap_mode_set(&reg_ctx, ISM330DHCX_ONLY_SINGLE);

  /* Enable single tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM330DHCX_INT1_PIN:
      if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val1.md1_cfg.int1_single_tap = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    case ISM330DHCX_INT2_PIN:
      if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val2.md2_cfg.int2_single_tap = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the Single Tap event
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_DisableSingleTapDetection()
{
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Disable single tap event on both INT1 and INT2 pins */
  if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val1.md1_cfg.int1_single_tap = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val2.md2_cfg.int2_single_tap = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset tap quiet time window. */
  if (ism330dhcx_tap_quiet_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset tap shock time window. */
  if (ism330dhcx_tap_shock_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset tap threshold. */
  if (ism330dhcx_tap_threshold_x_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_tap_threshold_y_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_tap_threshold_z_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_z_set(&reg_ctx, PROPERTY_DISABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_y_set(&reg_ctx, PROPERTY_DISABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_x_set(&reg_ctx, PROPERTY_DISABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Enable the detection of the Double Tap event
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_EnableDoubleTapDetection(ISM330DHCX_SensorIntPin_t IntPin)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (ACC_SetOutputDataRate(417.0f) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Full scale selection */
  if (ACC_SetFullScale(2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_x_set(&reg_ctx, PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_y_set(&reg_ctx, PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (ism330dhcx_tap_detection_on_z_set(&reg_ctx, PROPERTY_ENABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Set tap threshold. */
  if (ism330dhcx_tap_threshold_x_set(&reg_ctx, 0x0C) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_tap_threshold_y_set(&reg_ctx, 0x0C) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_tap_threshold_z_set(&reg_ctx, 0x0C) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Set tap shock time window. */
  if (ism330dhcx_tap_shock_set(&reg_ctx, 0x03) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Set tap quiet time window. */
  if (ism330dhcx_tap_quiet_set(&reg_ctx, 0x03) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Set tap duration time window. */
  if (ism330dhcx_tap_dur_set(&reg_ctx, 0x07) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Single and double tap enabled. */
  if (ism330dhcx_tap_mode_set(&reg_ctx, ISM330DHCX_BOTH_SINGLE_DOUBLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable double tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM330DHCX_INT1_PIN:
      if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val1.md1_cfg.int1_double_tap = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    case ISM330DHCX_INT2_PIN:
      if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val2.md2_cfg.int2_double_tap = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the Double Tap event
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_DisableDoubleTapDetection()
{
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Disable double tap event on both INT1 and INT2 pins */
  if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val1.md1_cfg.int1_double_tap = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val2.md2_cfg.int2_double_tap = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Only single tap enabled */
  if (ism330dhcx_tap_mode_set(&reg_ctx, ISM330DHCX_ONLY_SINGLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset tap duration time window */
  if (ism330dhcx_tap_dur_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset tap quiet time window */
  if (ism330dhcx_tap_quiet_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset tap shock time window */
  if (ism330dhcx_tap_shock_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset tap threshold */
  if (ism330dhcx_tap_threshold_x_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_tap_threshold_y_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_tap_threshold_z_set(&reg_ctx, 0x00) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Disable Z direction in tap recognition */
  if (ism330dhcx_tap_detection_on_z_set(&reg_ctx, PROPERTY_DISABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Disable Y direction in tap recognition */
  if (ism330dhcx_tap_detection_on_y_set(&reg_ctx, PROPERTY_DISABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Disable X direction in tap recognition */
  if (ism330dhcx_tap_detection_on_x_set(&reg_ctx, PROPERTY_DISABLE) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Threshold for the Tap event
 *  @param Threshold integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetTapThreshold(uint8_t Threshold)
{

  if (ism330dhcx_tap_threshold_x_set(&reg_ctx, Threshold) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Shock Time for the Tap event
 *  @param Time integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetTapShockTime(uint8_t Time)
{
  /* Set tap shock time window */
  if (ism330dhcx_tap_shock_set(&reg_ctx, Time) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Quiet Time for the Tap event
 *  @param Time integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetTapQuietTime(uint8_t Time)
{

  if (ism330dhcx_tap_quiet_set(&reg_ctx, Time) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Duration Time for the Tap event
 *  @param Time integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_SetTapDurationTime(uint8_t Time)
{
  if (ism330dhcx_tap_dur_set(&reg_ctx, Time) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Enable the detection of the 6DOrientation event
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Enable6DOrientation(ISM330DHCX_SensorIntPin_t IntPin)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (ACC_SetOutputDataRate(417.0f) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Full scale selection */
  if (ACC_SetFullScale(2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* 6D orientation enabled */
  if (ism330dhcx_6d_threshold_set(&reg_ctx, ISM330DHCX_DEG_60) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Enable 6D orientation event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM330DHCX_INT1_PIN:
      if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val1.md1_cfg.int1_6d = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    case ISM330DHCX_INT2_PIN:
      if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }

      val2.md2_cfg.int2_6d = PROPERTY_ENABLE;

      if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
        return ISM330DHCX_ERROR;
      }
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }
  return ret;
}

/*
 *  @brief Disable the detection of the 6DOrientation event
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Disable6DOrientation()
{
  ism330dhcx_pin_int1_route_t val1;
  ism330dhcx_pin_int2_route_t val2;

  /* Disable 6D orientation event on both INT1 and INT2 pins */
  if (ism330dhcx_pin_int1_route_get(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val1.md1_cfg.int1_6d = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int1_route_set(&reg_ctx, &val1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  if (ism330dhcx_pin_int2_route_get(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  val2.md2_cfg.int2_6d = PROPERTY_DISABLE;

  if (ism330dhcx_pin_int2_route_set(&reg_ctx, &val2) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  /* Reset 6D orientation */
  if (ism330dhcx_6d_threshold_set(&reg_ctx, ISM330DHCX_DEG_80) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Set the Threshold for the 6DOrientation event
 *  @param Threshold integer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Set6DOrientationThreshold(uint8_t Threshold)
{
  if (ism330dhcx_6d_threshold_set(&reg_ctx, (ism330dhcx_sixd_ths_t)Threshold) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the XL Axes for the 6DOrientation event
 *  @param XLow pointer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Get6DOrientationXL(uint8_t *XLow)
{
  ism330dhcx_d6d_src_t data;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_D6D_SRC, (uint8_t *) &data, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  *XLow = data.xl;

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the XH Axes for the 6DOrientation event
 *  @param XHigh pointer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Get6DOrientationXH(uint8_t *XHigh)
{
  ism330dhcx_d6d_src_t data;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_D6D_SRC, (uint8_t *) &data, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  *XHigh = data.xh;

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the YL Axes for the 6DOrientation event
 *  @param YLow pointer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Get6DOrientationYL(uint8_t *YLow)
{
  ism330dhcx_d6d_src_t data;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_D6D_SRC, (uint8_t *) &data, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  *YLow = data.yl;

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the YH Axes for the 6DOrientation event
 *  @param YHigh pointer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Get6DOrientationYH(uint8_t *YHigh)
{
  ism330dhcx_d6d_src_t data;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_D6D_SRC, (uint8_t *) &data, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  *YHigh = data.yh;

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the ZL Axes for the 6DOrientation event
 *  @param ZLow pointer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Get6DOrientationZL(uint8_t *ZLow)
{
  ism330dhcx_d6d_src_t data;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_D6D_SRC, (uint8_t *) &data, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  *ZLow = data.zl;

  return ISM330DHCX_OK;
}

/*
 *  @brief Get the ZH Axes for the 6DOrientation event
 *  @param ZHigh pointer
 *  @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Get6DOrientationZH(uint8_t *ZHigh)
{
  ism330dhcx_d6d_src_t data;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_D6D_SRC, (uint8_t *) &data, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  *ZHigh = data.zh;

  return ISM330DHCX_OK;
}

/**
 * @brief  Get the ISM330DHCX FIFO number of samples
 * @param  NumSamples number of samples
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Get_Num_Samples(uint16_t *NumSamples)
{
  if (ism330dhcx_fifo_data_level_get(&reg_ctx, NumSamples) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Get the ISM330DHCX FIFO full status
 * @param  Status pointer where store FIFO full status
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Get_Full_Status(uint8_t *Status)
{
  ism330dhcx_reg_t reg;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_FIFO_STATUS2, &reg.byte, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  *Status = reg.fifo_status2.fifo_full_ia;

  return ISM330DHCX_OK;
}

/**
 * @brief  Set the ISM330DHCX FIFO ACC ODR value
 * @param  Odr FIFO ODR value
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_ACC_Set_BDR(float Bdr)
{
  ism330dhcx_bdr_xl_t new_odr;

  new_odr = (Bdr <=   12.5f) ? ISM330DHCX_XL_BATCHED_AT_12Hz5
            : (Bdr <=   26.0f) ? ISM330DHCX_XL_BATCHED_AT_26Hz
            : (Bdr <=   52.0f) ? ISM330DHCX_XL_BATCHED_AT_52Hz
            : (Bdr <=  104.0f) ? ISM330DHCX_XL_BATCHED_AT_104Hz
            : (Bdr <=  208.0f) ? ISM330DHCX_XL_BATCHED_AT_208Hz
            : (Bdr <=  417.0f) ? ISM330DHCX_XL_BATCHED_AT_417Hz
            : (Bdr <=  833.0f) ? ISM330DHCX_XL_BATCHED_AT_833Hz
            : (Bdr <= 1667.0f) ? ISM330DHCX_XL_BATCHED_AT_1667Hz
            : (Bdr <= 3333.0f) ? ISM330DHCX_XL_BATCHED_AT_3333Hz
            :                    ISM330DHCX_XL_BATCHED_AT_6667Hz;

  if (ism330dhcx_fifo_xl_batch_set(&reg_ctx, new_odr) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Set the ISM330DHCX FIFO GYRO ODR value
 * @param  Odr FIFO ODR value
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_GYRO_Set_BDR(float Bdr)
{
  ism330dhcx_bdr_gy_t new_odr;

  new_odr = (Bdr <=   12.5f) ? ISM330DHCX_GY_BATCHED_AT_12Hz5
            : (Bdr <=   26.0f) ? ISM330DHCX_GY_BATCHED_AT_26Hz
            : (Bdr <=   52.0f) ? ISM330DHCX_GY_BATCHED_AT_52Hz
            : (Bdr <=  104.0f) ? ISM330DHCX_GY_BATCHED_AT_104Hz
            : (Bdr <=  208.0f) ? ISM330DHCX_GY_BATCHED_AT_208Hz
            : (Bdr <=  417.0f) ? ISM330DHCX_GY_BATCHED_AT_417Hz
            : (Bdr <=  833.0f) ? ISM330DHCX_GY_BATCHED_AT_833Hz
            : (Bdr <= 1667.0f) ? ISM330DHCX_GY_BATCHED_AT_1667Hz
            : (Bdr <= 3333.0f) ? ISM330DHCX_GY_BATCHED_AT_3333Hz
            :                    ISM330DHCX_GY_BATCHED_AT_6667Hz;

  if (ism330dhcx_fifo_gy_batch_set(&reg_ctx, new_odr) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Set the ISM330DHCX FIFO full interrupt on INT1 pin
 * @param  Status FIFO full interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Set_INT1_FIFO_Full(uint8_t Status)
{
  ism330dhcx_reg_t reg;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_INT1_CTRL, &reg.byte, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  reg.int1_ctrl.int1_fifo_full = Status;

  if (ism330dhcx_write_reg(&reg_ctx, ISM330DHCX_INT1_CTRL, &reg.byte, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Set the ISM330DHCX FIFO full interrupt on INT2 pin
 * @param  Status FIFO full interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Set_INT2_FIFO_Full(uint8_t Status)
{
  ism330dhcx_reg_t reg;

  if (ism330dhcx_read_reg(&reg_ctx, ISM330DHCX_INT2_CTRL, &reg.byte, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  reg.int2_ctrl.int2_fifo_full = Status;

  if (ism330dhcx_write_reg(&reg_ctx, ISM330DHCX_INT2_CTRL, &reg.byte, 1) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Set the ISM330DHCX FIFO watermark level
 * @param  Watermark FIFO watermark level
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Set_Watermark_Level(uint16_t Watermark)
{
  if (ism330dhcx_fifo_watermark_set(&reg_ctx, Watermark) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Set the ISM330DHCX FIFO stop on watermark
 * @param  Status FIFO stop on watermark status
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Set_Stop_On_Fth(uint8_t Status)
{
  if (ism330dhcx_fifo_stop_on_wtm_set(&reg_ctx, Status) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Set the ISM330DHCX FIFO mode
 * @param  Mode FIFO mode
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Set_Mode(uint8_t Mode)
{
  ISM330DHCXStatusTypeDef ret = ISM330DHCX_OK;

  /* Verify that the passed parameter contains one of the valid values */
  switch ((ism330dhcx_fifo_mode_t)Mode) {
    case ISM330DHCX_BYPASS_MODE:
    case ISM330DHCX_FIFO_MODE:
    case ISM330DHCX_STREAM_TO_FIFO_MODE:
    case ISM330DHCX_BYPASS_TO_STREAM_MODE:
    case ISM330DHCX_STREAM_MODE:
      break;

    default:
      ret = ISM330DHCX_ERROR;
      break;
  }

  if (ret == ISM330DHCX_ERROR) {
    return ret;
  }

  if (ism330dhcx_fifo_mode_set(&reg_ctx, (ism330dhcx_fifo_mode_t)Mode) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the ISM330DHCX FIFO tag
 * @param  Tag FIFO tag
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Get_Tag(uint8_t *Tag)
{
  ism330dhcx_fifo_tag_t tag_local;

  if (ism330dhcx_fifo_sensor_tag_get(&reg_ctx, &tag_local) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  *Tag = (uint8_t)tag_local;

  return ISM330DHCX_OK;
}

/**
 * @brief  Get the ISM330DHCX FIFO raw data
 * @param  Data pointer to save data
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_Get_Data(uint8_t *Data)
{
  if (ism330dhcx_fifo_out_raw_get(&reg_ctx, Data) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Get the ISM330DHCX FIFO accelero single sample (16-bit data per 3 axes) and calculate acceleration [mg]
 * @param  Acceleration FIFO accelerometer axes [mg]
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_ACC_Get_Axes(int32_t *Acceleration)
{
  uint8_t data[6];
  int16_t data_raw[3];
  float sensitivity = 0.0f;
  float acceleration_float[3];

  if (FIFO_Get_Data(data) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  data_raw[0] = ((int16_t)data[1] << 8) | data[0];
  data_raw[1] = ((int16_t)data[3] << 8) | data[2];
  data_raw[2] = ((int16_t)data[5] << 8) | data[4];

  if (ACC_GetSensitivity(&sensitivity) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  acceleration_float[0] = (float)data_raw[0] * sensitivity;
  acceleration_float[1] = (float)data_raw[1] * sensitivity;
  acceleration_float[2] = (float)data_raw[2] * sensitivity;

  Acceleration[0] = (int32_t)acceleration_float[0];
  Acceleration[1] = (int32_t)acceleration_float[1];
  Acceleration[2] = (int32_t)acceleration_float[2];

  return ISM330DHCX_OK;
}

/**
 * @brief  Get the ISM330DHCX FIFO gyro single sample (16-bit data per 3 axes) and calculate angular velocity [mDPS]
 * @param  AngularVelocity FIFO gyroscope axes [mDPS]
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::FIFO_GYRO_Get_Axes(int32_t *AngularVelocity)
{
  uint8_t data[6];
  int16_t data_raw[3];
  float sensitivity = 0.0f;
  float angular_velocity_float[3];

  if (FIFO_Get_Data(data) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  data_raw[0] = ((int16_t)data[1] << 8) | data[0];
  data_raw[1] = ((int16_t)data[3] << 8) | data[2];
  data_raw[2] = ((int16_t)data[5] << 8) | data[4];

  if (GYRO_GetSensitivity(&sensitivity) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  angular_velocity_float[0] = (float)data_raw[0] * sensitivity;
  angular_velocity_float[1] = (float)data_raw[1] * sensitivity;
  angular_velocity_float[2] = (float)data_raw[2] * sensitivity;

  AngularVelocity[0] = (int32_t)angular_velocity_float[0];
  AngularVelocity[1] = (int32_t)angular_velocity_float[1];
  AngularVelocity[2] = (int32_t)angular_velocity_float[2];

  return ISM330DHCX_OK;
}

/**
 * @brief  Enable ISM330DHCX accelerometer DRDY interrupt on INT1
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Enable_DRDY_On_INT1()
{
  ism330dhcx_pin_int1_route_t pin_int1_route;

  /* Enable accelerometer DRDY Interrupt on INT1 */
  if (ism330dhcx_pin_int1_route_get(&reg_ctx, &pin_int1_route) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  pin_int1_route.int1_ctrl.int1_drdy_xl = 1;
  pin_int1_route.int1_ctrl.int1_drdy_g = 0;

  if (ism330dhcx_pin_int1_route_set(&reg_ctx, &pin_int1_route) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Disable ISM330DHCX accelerometer DRDY interrupt on INT1
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::ACC_Disable_DRDY_On_INT1()
{
  ism330dhcx_pin_int1_route_t pin_int1_route;

  /* Disable accelerometer DRDY Interrupt on INT1 */
  if (ism330dhcx_pin_int1_route_get(&reg_ctx, &pin_int1_route) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  pin_int1_route.int1_ctrl.int1_drdy_xl = 0;

  if (ism330dhcx_pin_int1_route_set(&reg_ctx, &pin_int1_route) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Enable ISM330DHCX gyroscope DRDY interrupt on INT2
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_Enable_DRDY_On_INT2()
{
  ism330dhcx_pin_int2_route_t pin_int2_route;

  /* Enable gyroscope DRDY Interrupt on INT2 */
  if (ism330dhcx_pin_int2_route_get(&reg_ctx, &pin_int2_route) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  pin_int2_route.int2_ctrl.int2_drdy_xl = 0;
  pin_int2_route.int2_ctrl.int2_drdy_g = 1;

  if (ism330dhcx_pin_int2_route_set(&reg_ctx, &pin_int2_route) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

/**
 * @brief  Disable ISM330DHCX gyroscope DRDY interrupt on INT2
 * @retval 0 in case of success, an error code otherwise
 */
ISM330DHCXStatusTypeDef ISM330DHCXSensor::GYRO_Disable_DRDY_On_INT2()
{
  ism330dhcx_pin_int2_route_t pin_int2_route;

  /* Disable gyroscope DRDY Interrupt on INT2 */
  if (ism330dhcx_pin_int2_route_get(&reg_ctx, &pin_int2_route) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  pin_int2_route.int2_ctrl.int2_drdy_g = 0;

  if (ism330dhcx_pin_int2_route_set(&reg_ctx, &pin_int2_route) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

ISM330DHCXStatusTypeDef ISM330DHCXSensor::DRDY_Set_Mode(uint8_t Mode)
{

  if (ism330dhcx_data_ready_mode_set(&reg_ctx, (ism330dhcx_dataready_pulsed_t)Mode) != ISM330DHCX_OK) {
    return ISM330DHCX_ERROR;
  }

  return ISM330DHCX_OK;
}

int32_t ISM330DHCX_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((ISM330DHCXSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t ISM330DHCX_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((ISM330DHCXSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
