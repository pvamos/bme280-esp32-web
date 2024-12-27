# BME280 ESP32 Web Server with ESP-IDF

## Features

- **Wi-Fi Connectivity**: Connects the ESP32 to a specified Wi-Fi network.
- **HTTP Web Server**: Serves a minimal webpage displaying raw and compensated sensor values.
- **BME280 Sensor Integration**: Using I2C for sensor communication and reading environmental sensor data.
- **Efficient URL Handling**: Serves the main webpage at `/` and efficiently drops invalid URL requests (e.g., `/favicon.ico`) without logging unnecessary errors.


## Usage

1. Connect your ESP32 to the I2C pins of the BME280 sensor:
- **SDA**: GPIO21
- **SCL**: GPIO22

2. Once connected to Wi-Fi, open a browser and navigate to the IP address displayed in the serial monitor.

3. View the webpage to see live raw and compensated temperature, humidity, and pressure readings.


## Dependencies

- **BME280 Sensor**: I2C-compatible environmental sensor https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
- **ESP32 SOC**: https://www.espressif.com/en/products/socs/esp32
- **ESP-IDF**: v5.3.2 or newer https://github.com/espressif/esp-idf 


## Detailed Description

### BME280 Driver Logic

The BME280 driver is adapted from Bosch's official [BME280 Sensor API](https://github.com/boschsensortec/BME280_SensorAPI/).
Bosch's API provides high-level functionality for accessing sensor data via I2C or SPI.

**In this project**:
- Bosch's `i2c.h` (C++) is replaced by ESP-IDF's `i2c_master.h` (C).
- Functions such as `bme280_read_raw()` and `bme280_calculate()` implement low-level I2C operations and data compensation using ESP-IDF.

#### `bme280_read_raw()`

- Reads raw temperature, pressure, and humidity data from the sensor.
- Utilizes calibration data stored in the sensor to preprocess raw readings.
- Sends I2C commands using `i2c_master_transmit()` and receives data using `i2c_master_receive()`.

#### `bme280_calculate()`

- Performs compensation of raw data using formulas provided in Bosch's BME280 documentation.
- Implements temperature, pressure, and humidity compensation calculations for real-world accuracy.

**Compared to Bosch's API**:
- This implementation focuses on minimal integration with ESP-IDF while maintaining accuracy.
- The logic for raw data acquisition and compensation mirrors Bosch's methods but is customized for ESP32.

### Wi-Fi Connection

- Configured in `wifi_init()` using `esp_wifi_set_mode()` and `esp_wifi_set_config()`.
- Connects to a predefined Wi-Fi SSID and password.
- Logs the assigned IP address for accessing the web server.

### Web Server

- Hosted at the root URL `/`, serving an HTML page with raw and compensated sensor values.
- Handles invalid requests (e.g., `/favicon.ico`) by silently dropping them to minimize load and logging.

#### Webpage Parameters

- **Endpoint**: `/`
- **Response**: HTML page with:
  - Raw temperature, pressure, and humidity values.
  - Compensated temperature (in Celsius), pressure (in hPa), and humidity (in %).

### Error Handling

- Unused or invalid requests are ignored without returning HTTP 404 responses.
- Reduces server overhead for invalid URL requests.


## Setup

1. Clone the repository:
```bash
git clone <repository_url>
cd bme280
```

2. Configure the Wi-Fi credentials by updating the following lines in `main/main.c`:
```c
#define WIFI_SSID "YourSSID"
#define WIFI_PASS "YourPassword"
```

3. Set I2C configuration parameters at `components/bme280/bme280.h`:

```c
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 100000
#define BME280_I2C_ADDRESS 0x77
```

4. Build the firmware:
```bash
idf.py build
```

5. Flash the firmware:
```bash
idf.py -p /dev/ttyUSB0 flash
```

6. Monitor the serial output:
```bash
idf.py -p /dev/ttyUSB0 monitor
```


## References

### Hardware
- https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
- https://www.espressif.com/en/products/socs/esp32

### Copyright Notice

Copyright (c) 2024 Péter Vámos

https://github.com/pvamos
pvamos@gmail.com
https://linkedin.com/in/pvamos

#### Bosch's official BME280 Sensor API
- This software's Bosch BME280 sensor handling is based on Bosch's official BME280 Sensor API, originally created and maintained by Bosch Sensortec (copyright (c) 2016-2023), published and available at:
https://github.com/boschsensortec/BME280_SensorAPI 
- Distributed under the BSD-3-Clause License, which is included in the original repository:
https://github.com/boschsensortec/BME280_SensorAPI/blob/master/LICENSE
and in this repository in the `LICENSE_BME280` file.

Bosch's API is the reference implementation of functionality for accessing BME280 sensor data via I2C or SPI.
Bosch's `i2c.h` (C++) BME280 I2C usage pattern has been re-implemented using ESP-IDF's `i2c_master.h` (C).

#### Espressif Systems ESP-IDF framework
This software also uses the ESP-IDF framework, created and maintained by Espressif Systems (Shanghai) Co., Ltd., published and available at:  
https://github.com/espressif/esp-idf  
ESP-IDF is distributed under the Apache License 2.0, which is included in its repository:
https://github.com/espressif/esp-idf/blob/master/LICENSE
and in this repository in the `LICENSE_ESP-IDF` file.

#### MIT license
This version includes modifications and re-implementation by Péter Vámos in 2024, published and available at:
https://github.com/pvamos/bme280-esp32-web

I publish my own work under the MIT License.


## License

MIT License

Copyright (c) 2024 Péter Vámos  pvamos@gmail.com  https://github.com/pvamos

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
