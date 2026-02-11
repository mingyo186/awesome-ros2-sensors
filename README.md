# Awesome ROS2 Sensors 🤖⚡

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

> A curated list of ROS2 drivers for **breakout board sensors** (I2C/SPI/GPIO).
>
> Focused on **maker-friendly sensors** — the ones you actually use for prototyping, education, and hobby robotics.

Most "awesome ROS2" lists focus on industrial sensors. This list fills the gap for breakout board sensors that makers, students, and prototype builders use every day.

---

## Contents

- [IMU / Motion](#imu--motion)
- [Environment (Temp / Humidity / Pressure)](#environment)
- [Distance / Proximity](#distance--proximity)
- [Light / Color](#light--color)
- [Magnetometer / Compass](#magnetometer--compass)
- [ADC / DAC](#adc--dac)
- [Gas / Air Quality](#gas--air-quality)
- [Heart Rate / SpO2](#heart-rate--spo2)
- [GPS / GNSS](#gps--gnss)
- [Current / Power](#current--power)
- [Contributing](#contributing)

---

## Legend

| Symbol | Meaning |
|--------|---------|
| ✅ | ROS2 driver available |
| ⚠️ | ROS1 only (no ROS2 port) |
| ❌ | No known ROS driver |
| 🔧 | Work in progress |

---

## IMU / Motion

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **MPU6050** | 6-axis IMU | I2C | ✅ | [mingyo186/mpu6050_imu](https://github.com/mingyo186/mpu6050_imu) | Jazzy, Python, fake_mode for testing |
| **MPU9250** | 9-axis IMU | I2C/SPI | ✅ | [hiwad-aziz/ros2_mpu9250_driver](https://github.com/hiwad-aziz/ros2_mpu9250_driver) | C++, auto-calibration |
| **MPU9250** | 9-axis IMU | I2C | ✅ | [schnili/mpu9250](https://github.com/schnili/mpu9250) | Python wrapper |
| **ICM-20948** | 9-axis IMU | I2C/SPI | ✅ | [mingyo186/icm20948_imu](https://github.com/mingyo186/icm20948_imu) | Jazzy, Python, accel+gyro+AK09916 mag, fake_mode |
| **BNO055** | 9-axis AHRS | I2C | ✅ | [mingyo186/bno055_imu](https://github.com/mingyo186/bno055_imu) | Jazzy, Python, NDOF/IMU/Compass modes, fake_mode |
| **ADXL345** | 3-axis Accel | I2C/SPI | ✅ | [mingyo186/adxl345_accel](https://github.com/mingyo186/adxl345_accel) | Jazzy, Python, ±2/4/8/16g, full resolution, fake_mode |
| **LSM6DS3** | 6-axis IMU | I2C/SPI | ❌ | — | — |

## Environment

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **BMP280** | Pressure/Temp | I2C/SPI | ✅ | [mingyo186/bmp280_barometer](https://github.com/mingyo186/bmp280_barometer) | Jazzy, Python, FluidPressure + Temperature, fake_mode |
| **BME280** | Pressure/Temp/Humidity | I2C/SPI | ✅ | [mingyo186/bme280_env](https://github.com/mingyo186/bme280_env) | Jazzy, Python, FluidPressure + Temperature + RelativeHumidity, fake_mode |
| **BME680** | Pressure/Temp/Humidity/Gas | I2C/SPI | ❌ | — | + VOC air quality |
| **DHT11** | Temp/Humidity | GPIO | ❌ | — | Low accuracy |
| **DHT22** | Temp/Humidity | GPIO | ❌ | — | Better accuracy than DHT11 |
| **SHT31** | Temp/Humidity | I2C | ❌ | — | High precision |
| **DS18B20** | Temperature | 1-Wire | ❌ | — | Waterproof probe available |
| **TMP102** | Temperature | I2C | ❌ | — | ±0.5°C accuracy |

## Distance / Proximity

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **HC-SR04** | Ultrasonic | GPIO | ✅ | [mataruzz/libHCSR04](https://github.com/mataruzz/libHCSR04) | With ROS2 wrapper |
| **VL53L0X** | ToF Laser | I2C | ✅ | [mingyo186/vl53l0x_range](https://github.com/mingyo186/vl53l0x_range) | Jazzy, Python, short/medium/long modes, fake_mode |
| **VL53L1X** | ToF Laser | I2C | ✅ | [slaghuis/ROS2-VL53L1X](https://github.com/slaghuis/ROS2-VL53L1X) | sensor_msgs/Range |
| **VL53L5CX** | ToF Array (8×8) | I2C | ✅ | [adityakamath/tof_imager_ros](https://github.com/adityakamath/tof_imager_ros) | Lifecycle node, Humble |
| **TFMini** | LiDAR | UART | ⚠️ | — | ROS1 drivers exist |
| **GP2Y0A21** | IR Distance | Analog | ❌ | — | Needs ADC |
| **RCWL-1601** | Ultrasonic | GPIO | ❌ | — | HC-SR04 alternative |

## Light / Color

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **BH1750** | Ambient Light | I2C | ❌ | — | Lux measurement |
| **TSL2561** | Light/IR | I2C | ❌ | — | Visible + IR |
| **TCS34725** | RGB Color | I2C | ❌ | — | — |
| **APDS-9960** | Gesture/Proximity/Color | I2C | ❌ | — | Multi-function |
| **VEML7700** | Ambient Light | I2C | ❌ | — | High accuracy lux |

## Magnetometer / Compass

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **HMC5883L** | 3-axis Mag | I2C | ✅ | [mingyo186/hmc5883l_compass](https://github.com/mingyo186/hmc5883l_compass) | Jazzy, Python, hard-iron calibration, fake_mode |
| **QMC5883L** | 3-axis Mag | I2C | ❌ | — | HMC5883L clone |
| **LIS3MDL** | 3-axis Mag | I2C/SPI | ❌ | — | ±4/8/12/16 gauss |
| **MMC5603** | 3-axis Mag | I2C | ❌ | — | — |

## ADC / DAC

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **ADS1115** | 16-bit ADC | I2C | ✅ | [mingyo186/ads1115_adc](https://github.com/mingyo186/ads1115_adc) | Jazzy, Python, 4-ch, PGA configurable, fake_mode |
| **ADS1015** | 12-bit ADC | I2C | ❌ | — | Faster, lower resolution |
| **MCP3008** | 10-bit ADC | SPI | ❌ | — | 8 channels |
| **MCP4725** | 12-bit DAC | I2C | ❌ | — | Single channel |
| **PCF8591** | 8-bit ADC+DAC | I2C | ❌ | — | 4 ADC + 1 DAC |

## Gas / Air Quality

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **MQ-2** | Gas (LPG/Smoke) | Analog | ❌ | — | Needs ADC |
| **MQ-135** | Air Quality | Analog | ❌ | — | Needs ADC |
| **CCS811** | eCO2/TVOC | I2C | ❌ | — | Digital output |
| **SGP30** | eCO2/TVOC | I2C | ❌ | — | — |
| **SCD40** | CO2/Temp/Humidity | I2C | ❌ | — | True CO2 (photoacoustic) |

## Heart Rate / SpO2

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **MAX30102** | HR/SpO2 | I2C | ❌ | — | Pulse oximeter |
| **MAX30105** | HR/SpO2/Particle | I2C | ❌ | — | + Smoke detection |

## GPS / GNSS

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **NEO-6M** | GPS | UART | ⚠️ | — | ROS1 nmea drivers |
| **NEO-M8N** | GPS/GLONASS | UART/I2C | ⚠️ | — | u-blox, better accuracy |
| **BN-880** | GPS/Compass | UART/I2C | ❌ | — | GPS + HMC5883L combo |
| **PA1010D** | GPS | I2C | ❌ | — | I2C GPS module |

## Current / Power

| Sensor | Type | Interface | ROS2 | Link | Notes |
|--------|------|-----------|------|------|-------|
| **INA219** | Current/Voltage | I2C | ❌ | — | Bidirectional, 26V max |
| **INA226** | Current/Voltage | I2C | ❌ | — | Higher precision |
| **ACS712** | Current | Analog | ❌ | — | Needs ADC, 5/20/30A |

---

## Summary

| Category | Total Sensors | ✅ ROS2 | ⚠️ ROS1 Only | ❌ None |
|----------|--------------|---------|--------------|--------|
| IMU / Motion | 7 | 5 | 0 | 2 |
| Environment | 8 | 2 | 0 | 6 |
| Distance / Proximity | 7 | 4 | 1 | 2 |
| Light / Color | 5 | 0 | 0 | 5 |
| Magnetometer | 4 | 1 | 0 | 3 |
| ADC / DAC | 5 | 1 | 0 | 4 |
| Gas / Air Quality | 5 | 0 | 0 | 5 |
| Heart Rate / SpO2 | 2 | 0 | 0 | 2 |
| GPS / GNSS | 4 | 0 | 2 | 2 |
| Current / Power | 3 | 0 | 0 | 3 |
| **Total** | **50** | **13 (26%)** | **3 (6%)** | **34 (68%)** |

> **68% of common maker sensors have NO ROS2 driver.** This is a massive opportunity for the community.

---

## ROS2 Message Types Reference

Common `sensor_msgs` for breakout sensors:

| Message | Use Case | Sensors |
|---------|----------|---------|
| `sensor_msgs/Imu` | Accel + Gyro + Orientation | MPU6050, MPU9250, BNO055 |
| `sensor_msgs/MagneticField` | Magnetometer | HMC5883L, QMC5883L |
| `sensor_msgs/Temperature` | Temperature | BMP280, DHT22, DS18B20 |
| `sensor_msgs/FluidPressure` | Barometric pressure | BMP280, BME280 |
| `sensor_msgs/RelativeHumidity` | Humidity | BME280, DHT22, SHT31 |
| `sensor_msgs/Range` | Single-point distance | VL53L0X, HC-SR04, TFMini |
| `sensor_msgs/NavSatFix` | GPS coordinates | NEO-6M, NEO-M8N |
| `sensor_msgs/Illuminance` | Lux (light level) | BH1750, TSL2561 |

---

## Contributing

Found a ROS2 driver not listed here? Want to add a sensor?

1. Fork this repo
2. Add the sensor to the appropriate table
3. Submit a Pull Request

**Rules:**
- Sensors must be breakout boards
- Drivers must be **ROS2** (not ROS1)
- Include: sensor name, type, interface, ROS2 status, link, notes
- Test the driver before submitting if possible

---

## Why This List?

The ROS2 ecosystem has great support for industrial sensors (Velodyne LiDAR, Intel RealSense, SICK scanners), but **hobby/education sensors are severely underserved**. If you're building a robot with a breakout board IMU or distance sensor, finding a working ROS2 driver is frustratingly difficult.

This list aims to:
- 📋 **Map the gap** — show exactly which sensors need ROS2 drivers
- 🔗 **Connect** existing drivers with users who need them
- 🚀 **Motivate** the community to fill the ❌ gaps

---

## Related

- [awesome-ros2](https://github.com/fkromer/awesome-ros2) — General ROS2 resources
- [awesome-robotic-tooling](https://github.com/Ly0n/awesome-robotic-tooling) — Industrial robotics tools
- [awesome-micro-ros-projects](https://github.com/kaiaai/awesome-micro-ros-projects) — micro-ROS projects

---

## License

[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](https://creativecommons.org/publicdomain/zero/1.0/)

This list is dedicated to the public domain under CC0 1.0.
