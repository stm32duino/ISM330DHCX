/*
   @file    ISM330DHCX_DataLogTerminal.ino
   @author  Frederic Pillon <frederic.pillon@st.com>
   @brief   Example to use the ISM330DHCX 3D accelerometer and 3D gyroscope
 *******************************************************************************
   Copyright (c) 2021, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/


// Includes
#include <ISM330DHCXSensor.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN PNUM_NOT_DEFINED
#warning "LED_BUILTIN is not defined."
#endif

#define SerialPort  Serial

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

void setup() {
  // Led
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output
  SerialPort.begin(9600);

  // Initialize bus interface
  dev_interface.begin();

  // Initlialize component
  AccGyr.begin();
  AccGyr.ACC_Enable();  
  AccGyr.GYRO_Enable();
}

void loop() {
  // Led blinking
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // Read accelerometer and gyroscope
  int32_t accelerometer[3];
  int32_t gyroscope[3];
  AccGyr.ACC_GetAxes(accelerometer);  
  AccGyr.GYRO_GetAxes(gyroscope);

  // Print compatible with Serial plotter
  Serial.print("Acc[mg]:");
  Serial.print(accelerometer[0]);
  Serial.print(", ");
  Serial.print(accelerometer[1]);
  Serial.print(", ");
  Serial.print(accelerometer[2]);
  Serial.print(", Gyro[mdps]:");
  Serial.print(gyroscope[0]);
  Serial.print(", ");
  Serial.print(gyroscope[1]);
  Serial.print(", ");
  Serial.println(gyroscope[2]);
}
