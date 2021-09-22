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

## Documentation 
You can find the source files at  
https://github.com/stm32duino/ISM330DHCX

The ISM330DHCX datasheet is available at  
https://www.st.com/en/mems-and-sensors/ism330dhcx.html
