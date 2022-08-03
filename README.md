# ISM330DHCX
Arduino library to support the ISM330DHCX 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    ISM330DHCXSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.ACC_Enable();  
    AccGyr.GYRO_Enable();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    ISM330DHCXSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.ACC_Enable();  
    AccGyr.GYRO_Enable();

The access to the sensor value is done as explained below:

  Read accelerometer and gyroscope.

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr.ACC_GetAxes(accelerometer);  
    AccGyr.GYRO_GetAxes(gyroscope);

## Units

All the MEMS drivers in stm32duino repositories have the same units when returning physical (scaled) values:

    - mg for accelerometer (thousandth of 1g that is 9.81 m/s^2)
    - mdps for gyroscope (thousandth of degree/s)
    - mgauss for magnetometer (thousandth of gauss)

Readings can be obtained either as raw (unscaled) ```uint16_t``` values from the ```XXX_GetAxesRaw``` functions, or as physical (scaled) ```uint32_t``` values from the ```XXX_GetAxes``` functions, where ```XXX``` is eithet ```ACC``` or ```GYRO```. The factor to use for scaling from a raw to a physical value is from the ```XXX_GetSensitivity``` functions.

For example, to convert the raw acceleration reading to float m/s^2, the following can be done:

```cpp
int16_t acc_raw[3];
float acc_g[3];
float acc_sensitivity;

AccGyr.ACC_GetAxesRaw(acc_raw);
AccGyr.ACC_GetSensitivity(&acc_sensitivity);

acc_g[0] = ((float)acc_raw[0] * acc_sensitivity) / 1000.0f * 9.81f;
acc_g[1] = ((float)acc_raw[1] * acc_sensitivity) / 1000.0f * 9.81f;
acc_g[2] = ((float)acc_raw[2] * acc_sensitivity) / 1000.0f * 9.81f;
```

## Documentation 
You can find the source files at  
https://github.com/stm32duino/ISM330DHCX

The ISM330DHCX datasheet is available at  
https://www.st.com/en/mems-and-sensors/ism330dhcx.html
