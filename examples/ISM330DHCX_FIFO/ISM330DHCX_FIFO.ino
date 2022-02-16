/*
   @file    ISM330DHCX_FIFO.ino
   @author  STMicroelectronics
   @brief   Example to use the ISM330DHCX 3D accelerometer and 3D gyroscope FIFO
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

// Includes
#include <ISM330DHCXSensor.h>

#if defined(ARDUINO_STM32WB5MM_DK) || defined(ARDUINO_B_U585I_IOT02A)
#define USE_I2C_INTERFACE
#else
// Uncomment one communication interface to use:
// X-NUCLEO-IKS02A1 uses I2C interface (set by default)
// #define USE_SPI_INTERFACE
#define USE_I2C_INTERFACE
#endif

#if !defined(USE_SPI_INTERFACE) && !defined(USE_I2C_INTERFACE) || \
    defined(USE_SPI_INTERFACE) && defined(USE_I2C_INTERFACE)
#error "Uncomment one communication interface to use!"
#endif

#ifdef USE_SPI_INTERFACE
// Uncomment to set SPI pins to use else default instance will be used
// #define ISM330DHCX_SPI_MOSI PB10
// #define ISM330DHCX_SPI_MISO PB11
// #define ISM330DHCX_SPI_SCLK PB11
// Define Software Chip Select pin to use (default: SS)
#define ISM330DHCX_SPI_SSEL    SS

#if defined(ISM330DHCX_SPI_MOSI) && defined(ISM330DHCX_SPI_MISO) && \
    defined(ISM330DHCX_SPI_SCLK) && defined(ISM330DHCX_SPI_SSEL)
SPIClass dev_interface(ISM330DHCX_SPI_MOSI, ISM330DHCX_SPI_MISO, ISM330DHCX_SPI_SCLK);
#else
#define dev_interface       SPI
#endif

ISM330DHCXSensor AccGyr(&dev_interface, ISM330DHCX_SPI_SSEL);
#else // USE_I2C_INTERFACE
#if defined(ARDUINO_STM32WB5MM_DK)
#define ISM330DHCX_I2C_SCL     PB13
#define ISM330DHCX_I2C_SDA     PB11
#elif defined(ARDUINO_B_U585I_IOT02A)
#define ISM330DHCX_I2C_SCL     PH4
#define ISM330DHCX_I2C_SDA     PH5
#else
// Uncomment to set I2C pins to use else default instance will be used
// #define ISM330DHCX_I2C_SCL  PYn
// #define ISM330DHCX_I2C_SDA  PYn
#endif
#if defined(ISM330DHCX_I2C_SCL) && defined(ISM330DHCX_I2C_SDA)
TwoWire dev_interface(ISM330DHCX_I2C_SDA, ISM330DHCX_I2C_SCL);
#else
// X-NUCLEO-IKS02A1 uses default instance
#define dev_interface       Wire
#endif

ISM330DHCXSensor AccGyr(&dev_interface);
#endif

#define SENSOR_ODR 104.0f // In Hertz
#define ACC_FS 2 // In g
#define GYR_FS 2000 // In dps
#define MEASUREMENT_TIME_INTERVAL (1000.0f/SENSOR_ODR) // In ms
#define FIFO_SAMPLE_THRESHOLD 199
#define FLASH_BUFF_LEN 8192

unsigned long timestamp_count = 0;
bool acc_available = false;
bool gyr_available = false;
int32_t acc_value[3];
int32_t gyr_value[3];
char buff[FLASH_BUFF_LEN];
uint32_t pos = 0;

void Read_FIFO_Data(uint16_t samples_to_read);

void setup() {
  Serial.begin(115200);

  dev_interface.begin();
  
  // Initialize IMU.
  AccGyr.begin();
  AccGyr.ACC_Enable();
  AccGyr.GYRO_Enable();
  // Configure ODR and FS of the acc and gyro
  AccGyr.ACC_SetOutputDataRate(SENSOR_ODR);
  AccGyr.ACC_SetFullScale(ACC_FS);
  AccGyr.GYRO_SetOutputDataRate(SENSOR_ODR);
  AccGyr.GYRO_SetFullScale(GYR_FS);
  // Configure FIFO BDR for acc and gyro
  AccGyr.FIFO_ACC_Set_BDR(SENSOR_ODR);
  AccGyr.FIFO_GYRO_Set_BDR(SENSOR_ODR);
  // Set FIFO in Continuous mode
  AccGyr.FIFO_Set_Mode(ISM330DHCX_STREAM_MODE);
  Serial.println("ISM330DHCX FIFO Demo");
}

void loop() {
  uint16_t fifo_samples;

  // Check the number of samples inside FIFO
  AccGyr.FIFO_Get_Num_Samples(&fifo_samples);

  // If we reach the threshold we can empty the FIFO
  if (fifo_samples > FIFO_SAMPLE_THRESHOLD) {
    // Empty the FIFO
    Read_FIFO_Data(fifo_samples);
    // Print FIFO data
    Serial.print(buff);
  }
}

void Read_FIFO_Data(uint16_t samples_to_read)
{
  uint16_t i;

  for (i = 0; i < samples_to_read; i++) {
    uint8_t tag;
    // Check the FIFO tag
    AccGyr.FIFO_Get_Tag(&tag);
    switch (tag) {
      // If we have a gyro tag, read the gyro data
      case ISM330DHCX_GYRO_NC_TAG: {
          AccGyr.FIFO_GYRO_Get_Axes(gyr_value);
          gyr_available = true;
          break;
        }
      // If we have an acc tag, read the acc data
      case ISM330DHCX_XL_NC_TAG: {
          AccGyr.FIFO_ACC_Get_Axes(acc_value);
          acc_available = true;
          break;
        }
      // We can discard other tags
      default: {
          break;
        }
    }
    // If we have the measurements of both acc and gyro, we can store them with timestamp
    if (acc_available && gyr_available) {
      int num_bytes;
      num_bytes = snprintf(&buff[pos], (FLASH_BUFF_LEN - pos), "%lu %d %d %d %d %d %d\r\n", (unsigned long)((float)timestamp_count * MEASUREMENT_TIME_INTERVAL), (int)acc_value[0], (int)acc_value[1], (int)acc_value[2], (int)gyr_value[0], (int)gyr_value[1], (int)gyr_value[2]);
      pos += num_bytes;
      timestamp_count++;
      acc_available = false;
      gyr_available = false;
    }
  }
  // We can add the termination character to the string, so we are ready to print it on hyper-terminal
  buff[pos] = '\0';
  pos = 0;
}
