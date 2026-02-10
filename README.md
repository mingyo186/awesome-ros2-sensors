# Awesome ROS2 Sensors 🤖⚡

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

> A curated list of ROS2 drivers for **low-cost breakout board sensors** (I2C/SPI/GPIO).
>
> Focused on **maker-friendly sensors under $20** — the ones you actually use for prototyping, education, and hobby robotics.

Most "awesome ROS2" lists focus on industrial sensors ($500+). This list fills the gap for affordable breakout board sensors that makers, students, and prototype builders use every day.

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

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **MPU6050** | 6-axis IMU | I2C | $2 | ✅ | [mingyo186/mpu6050_imu](https://github.com/mingyo186/mpu6050_imu) | Jazzy, Python, fake_mode for testing |
| **MPU9250** | 9-axis IMU | I2C/SPI | $5 | ✅ | [hiwad-aziz/ros2_mpu9250_driver](https://github.com/hiwad-aziz/ros2_mpu9250_driver) | C++, auto-calibration |
| **MPU9250** | 9-axis IMU | I2C | $5 | ✅ | [schnili/mpu9250](https://github.com/schnili/mpu9250) | Python wrapper |
| **ICM-20948** | 9-axis IMU | I2C/SPI | $8 | ❌ | — | Successor to MPU9250 |
| **BNO055** | 9-axis AHRS | I2C | $12 | ⚠️ | — | Built-in sensor fusion, ROS1 only |
| **ADXL345** | 3-axis Accel | I2C/SPI | $3 | ❌ | — | — |
| **LSM6DS3** | 6-axis IMU | I2C/SPI | $5 | ❌ | — | — |

## Environment

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **BMP280** | Pressure/Temp | I2C/SPI | $3 | ❌ | — | Barometric pressure + altitude |
| **BME280** | Pressure/Temp/Humidity | I2C/SPI | $5 | ❌ | — | BMP280 + humidity |
| **BME680** | Pressure/Temp/Humidity/Gas | I2C/SPI | $10 | ❌ | — | + VOC air quality |
| **DHT11** | Temp/Humidity | GPIO | $1 | ❌ | — | Low accuracy |
| **DHT22** | Temp/Humidity | GPIO | $3 | ❌ | — | Better accuracy than DHT11 |
| **SHT31** | Temp/Humidity | I2C | $5 | ❌ | — | High precision |
| **DS18B20** | Temperature | 1-Wire | $2 | ❌ | — | Waterproof probe available |
| **TMP102** | Temperature | I2C | $3 | ❌ | — | ±0.5°C accuracy |

## Distance / Proximity

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **HC-SR04** | Ultrasonic | GPIO | $1 | ✅ | [mataruzz/libHCSR04](https://github.com/mataruzz/libHCSR04) | With ROS2 wrapper |
| **VL53L0X** | ToF Laser | I2C | $5 | ⚠️ | [Andrew-rw/vl53l0x_driver](https://github.com/Andrew-rw/vl53l0x_driver) | ROS1 only |
| **VL53L1X** | ToF Laser | I2C | $7 | ✅ | [slaghuis/ROS2-VL53L1X](https://github.com/slaghuis/ROS2-VL53L1X) | sensor_msgs/Range |
| **VL53L5CX** | ToF Array (8×8) | I2C | $15 | ✅ | [adityakamath/tof_imager_ros](https://github.com/adityakamath/tof_imager_ros) | Lifecycle node, Humble |
| **TFMini** | LiDAR | UART | $15 | ⚠️ | — | ROS1 drivers exist |
| **GP2Y0A21** | IR Distance | Analog | $3 | ❌ | — | Needs ADC |
| **RCWL-1601** | Ultrasonic | GPIO | $2 | ❌ | — | HC-SR04 alternative |

## Light / Color

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **BH1750** | Ambient Light | I2C | $2 | ❌ | — | Lux measurement |
| **TSL2561** | Light/IR | I2C | $3 | ❌ | — | Visible + IR |
| **TCS34725** | RGB Color | I2C | $5 | ❌ | — | — |
| **APDS-9960** | Gesture/Proximity/Color | I2C | $5 | ❌ | — | Multi-function |
| **VEML7700** | Ambient Light | I2C | $3 | ❌ | — | High accuracy lux |

## Magnetometer / Compass

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **HMC5883L** | 3-axis Mag | I2C | $3 | ✅ | [GitHub](https://github.com/topics/hmc5883l) | Publishes /imu/mag |
| **QMC5883L** | 3-axis Mag | I2C | $2 | ❌ | — | HMC5883L clone |
| **LIS3MDL** | 3-axis Mag | I2C/SPI | $5 | ❌ | — | ±4/8/12/16 gauss |
| **MMC5603** | 3-axis Mag | I2C | $3 | ❌ | — | — |

## ADC / DAC

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **ADS1115** | 16-bit ADC | I2C | $3 | ❌ | — | 4-ch, programmable gain |
| **ADS1015** | 12-bit ADC | I2C | $3 | ❌ | — | Faster, lower resolution |
| **MCP3008** | 10-bit ADC | SPI | $2 | ❌ | — | 8 channels |
| **MCP4725** | 12-bit DAC | I2C | $3 | ❌ | — | Single channel |
| **PCF8591** | 8-bit ADC+DAC | I2C | $1 | ❌ | — | 4 ADC + 1 DAC |

## Gas / Air Quality

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **MQ-2** | Gas (LPG/Smoke) | Analog | $2 | ❌ | — | Needs ADC |
| **MQ-135** | Air Quality | Analog | $2 | ❌ | — | Needs ADC |
| **CCS811** | eCO2/TVOC | I2C | $8 | ❌ | — | Digital output |
| **SGP30** | eCO2/TVOC | I2C | $8 | ❌ | — | — |
| **SCD40** | CO2/Temp/Humidity | I2C | $15 | ❌ | — | True CO2 (photoacoustic) |

## Heart Rate / SpO2

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **MAX30102** | HR/SpO2 | I2C | $3 | ❌ | — | Pulse oximeter |
| **MAX30105** | HR/SpO2/Particle | I2C | $5 | ❌ | — | + Smoke detection |

## GPS / GNSS

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **NEO-6M** | GPS | UART | $5 | ⚠️ | — | ROS1 nmea drivers |
| **NEO-M8N** | GPS/GLONASS | UART/I2C | $10 | ⚠️ | — | u-blox, better accuracy |
| **BN-880** | GPS/Compass | UART/I2C | $10 | ❌ | — | GPS + HMC5883L combo |
| **PA1010D** | GPS | I2C | $12 | ❌ | — | I2C GPS module |

## Current / Power

| Sensor | Type | Interface | ~Price | ROS2 | Link | Notes |
|--------|------|-----------|--------|------|------|-------|
| **INA219** | Current/Voltage | I2C | $3 | ❌ | — | Bidirectional, 26V max |
| **INA226** | Current/Voltage | I2C | $3 | ❌ | — | Higher precision |
| **ACS712** | Current | Analog | $2 | ❌ | — | Needs ADC, 5/20/30A |

---

## Summary

| Category | Total Sensors | ✅ ROS2 | ⚠️ ROS1 Only | ❌ None |
|----------|--------------|---------|--------------|--------|
| IMU / Motion | 7 | 3 | 1 | 3 |
| Environment | 8 | 0 | 0 | 8 |
| Distance / Proximity | 7 | 3 | 2 | 2 |
| Light / Color | 5 | 0 | 0 | 5 |
| Magnetometer | 4 | 1 | 0 | 3 |
| ADC / DAC | 5 | 0 | 0 | 5 |
| Gas / Air Quality | 5 | 0 | 0 | 5 |
| Heart Rate / SpO2 | 2 | 0 | 0 | 2 |
| GPS / GNSS | 4 | 0 | 2 | 2 |
| Current / Power | 3 | 0 | 0 | 3 |
| **Total** | **50** | **7 (14%)** | **5 (10%)** | **38 (76%)** |

> **76% of common maker sensors have NO ROS2 driver.** This is a massive opportunity for the community.

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
- Sensors must be **under $20** breakout boards
- Drivers must be **ROS2** (not ROS1)
- Include: sensor name, type, interface, price, ROS2 status, link, notes
- Test the driver before submitting if possible

---

## Why This List?

The ROS2 ecosystem has great support for industrial sensors (Velodyne LiDAR, Intel RealSense, SICK scanners), but **hobby/education sensors are severely underserved**. If you're building a robot with a $3 IMU or a $5 distance sensor, finding a working ROS2 driver is frustratingly difficult.

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
