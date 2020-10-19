# ISM330DHCX
Arduino library to support the ISM330DHCX 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    dev_i2c = new TwoWire(I2C_SDA, I2C_SCL);  
    dev_i2c->begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    dev_spi = new SPIClass(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi->begin();

An instance can be create and enabled when the I2C bus is used following the procedure below:

	AccGyro = new ISM330DHCXSensor(dev_i2c);

Enable accelerometer.

	AccGyro->ACC_Enable();

Enable gyroscope.

	AccGyro->GYRO_Enable();

An instance can be create and enabled when the SPI bus is used following the procedure below:

	AccGyro = new ISM330DHCXSensor(dev_spi, CS_PIN);

Enable accelerometer.

	AccGyro->ACC_Enable();

Enable gyroscope.

	AccGyro->GYRO_Enable();

The access to the sensor value is done as explained below:

  Read accelerometer.

	AccGyro->ACC_GetAxes();

  Read gyroscope.

	AccGyro->GYRO_GetAxes();

## Documentation 
You can find the source files at  
https://github.com/stm32duino/ISM330DHCX

The IIS2MDC datasheet is available at  
https://www.st.com/en/mems-and-sensors/ism330dhcx.html