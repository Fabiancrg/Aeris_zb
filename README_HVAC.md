# Zigbee Air Quality Sensor - Aeris

This project implements a Zigbee End Device that reads air quality sensors and exposes them as standard Zigbee sensor endpoints.

## Hardware Requirements

- ESP32-C6 development board (or compatible)
- Air quality sensors:
  - Temperature and Humidity sensor (e.g., SHT4x, BME280) via I2C
  - **LPS22HB Pressure sensor** via I2C
  - **PMSA003A Particulate Matter sensor** via UART
  - VOC Index sensor (e.g., SGP40, BME680) via I2C
  - CO2 sensor (e.g., SCD40, SCD41) via I2C
- I2C connection (SDA/SCL pins configurable)
- UART connection for PMSA003A (RX pin configurable)

## Features

The Zigbee air quality sensor exposes the following sensor endpoints:

### Endpoint 1: Temperature and Humidity Sensor
- **Temperature Measurement Cluster (0x0402)**: Temperature in °C
- **Relative Humidity Measurement Cluster (0x0405)**: Humidity in %

### Endpoint 2: Pressure Sensor
- **Pressure Measurement Cluster (0x0403)**: Atmospheric pressure in hPa
- **Sensor**: STMicroelectronics LPS22HB (I2C)
- **Update Rate**: 25 Hz (configurable)
- **Range**: 260-1260 hPa
- **Accuracy**: ±0.025 hPa (typical)

### Endpoint 3: Particulate Matter (PM) Sensor
- **PM2.5 Measurement**: Fine particulate matter (µg/m³) from PMSA003A
- **PM10 Measurement**: Coarse particulate matter (µg/m³) from PMSA003A
- **PM1.0 Measurement**: Ultra-fine particulate matter (µg/m³) from PMSA003A
- **Sensor**: Plantower PMSA003A (UART, 9600 baud)
- **Update Rate**: ~1 second (automatic)

### Endpoint 4: VOC Index Sensor
- **VOC Index** (1-500): Volatile Organic Compounds air quality index

### Endpoint 5: CO2 Sensor
- **CO2 Concentration Cluster (0x040D)**: Carbon dioxide in ppm

## I2C and UART Configuration

### I2C Bus (Temperature, Humidity, Pressure, VOC, CO2)

The device communicates with most sensors via I2C:

- **SDA Pin**: GPIO 6 (configurable in `aeris_driver.h`)
- **SCL Pin**: GPIO 7 (configurable in `aeris_driver.h`)
- **Frequency**: 100 kHz
- **Pull-ups**: Internal pull-ups enabled

**I2C Addresses:**
- LPS22HB (Pressure): 0x5C (default, SA0=0) or 0x5D (SA0=1)

### UART for PMSA003A (Particulate Matter Sensor)

The PMSA003A sensor uses UART communication:

- **RX Pin**: GPIO 20 (receives data from PMSA003A TX)
- **TX Pin**: GPIO 18 (not used - PMSA003A is read-only)
- **Baud Rate**: 9600
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Frame Length**: 32 bytes
- **Update Rate**: ~1 second (automatic from sensor)

**PMSA003A Data Format:**
- Start characters: 0x42 0x4D
- Frame length: 28 data bytes + 2 byte checksum
- PM1.0, PM2.5, PM10 concentrations (both CF=1 and atmospheric)
- Particle counts for various size ranges

## Project Structure

```
Aeris_zb/
├── main/
│   ├── esp_zb_aeris.c         # Main Zigbee application
│   ├── esp_zb_aeris.h         # Zigbee configuration header
│   ├── aeris_driver.c         # Air quality sensor driver implementation
│   ├── aeris_driver.h         # Sensor driver header
│   ├── esp_zb_ota.c           # OTA update support
│   ├── esp_zb_ota.h           # OTA header
│   ├── board.h                # Board pin definitions
│   ├── CMakeLists.txt         # Component build configuration
│   └── idf_component.yml      # Component dependencies
├── CMakeLists.txt             # Project CMakeLists
├── sdkconfig                  # ESP-IDF configuration
└── README.md                  # This file
```

## Building and Flashing

1. Set up ESP-IDF v5.5.1 or later
2. Configure the project:
   ```bash
   idf.py set-target esp32c6
   idf.py menuconfig
   ```
3. Build the project:
   ```bash
   idf.py build
   ```
4. Flash to device:
   ```bash
   idf.py -p COMx flash monitor
   ```

## Configuration

### Zigbee Configuration

The device is configured as a Zigbee End Device (ZED):
- **Endpoints**: 1-5 (one per sensor type)
- **Profile**: Home Automation (0x0104)
- **Device IDs**: Various sensor types (Temperature, Humidity, Pressure, etc.)
- **Channel Mask**: All channels

### Sensor Configuration

Configure sensor I2C addresses and settings in `aeris_driver.c`:
- Default I2C pins: SDA=GPIO6, SCL=GPIO7
- Adjust pins in `aeris_driver.h` if needed

### Joining the Network

On first boot, the device will automatically enter network steering mode. Once joined, the device will save the network credentials and automatically rejoin on subsequent boots.

## Usage

### From Zigbee Coordinator (e.g., Zigbee2MQTT, Home Assistant)

1. Put your coordinator in pairing mode
2. Power on the ESP32-C6 device
3. Wait for the device to join (check logs)
4. The air quality sensor will appear with the following entities:
   - Temperature (°C)
   - Humidity (%)
   - Pressure (hPa)
   - PM1.0, PM2.5, PM10 (µg/m³)
   - VOC Index (1-500)
   - CO2 (ppm)

### Sensor Reading Updates

- Sensors are polled periodically (configurable interval)
- Values are reported to the Zigbee coordinator when they change
- All endpoints support binding and reporting configuration

## Supported Sensors

This project provides a framework for air quality sensors. You'll need to implement the actual sensor drivers:

### Temperature/Humidity
- **SHT4x** (recommended): High accuracy, low power
- **BME280**: Temperature, humidity, and pressure in one chip
- **DHT22**: Budget option

### Pressure
- **LPS22HB** (implemented): STMicroelectronics MEMS pressure sensor
  - I2C interface (address 0x5C or 0x5D)
  - Measures 260-1260 hPa
  - ±0.025 hPa accuracy
  - Internal temperature sensor (bonus)
  - Low power: 3 µA @ 1Hz
  - Configurable output rate (1-75 Hz)
- **Alternative**: BMP280, BME280 (Bosch sensors)

### Particulate Matter
- **PMSA003A** (implemented): Plantower laser PM sensor
  - UART interface (9600 baud)
  - Measures PM1.0, PM2.5, PM10
  - Automatic 1-second updates
  - Low power consumption
  - Atmospheric environment correction included
- **Alternative**: PMS5003 (similar protocol, same manufacturer)

### VOC Index
- **SGP40**: VOC sensor with built-in algorithm
- **BME680**: VOC + temperature/humidity/pressure

### CO2
- **SCD40**: NDIR CO2 sensor (400-2000 ppm)
- **SCD41**: Extended range (400-5000 ppm)

## Implementation Notes

The `aeris_driver.c` file contains:

1. **PMSA003A implementation** (complete):
   - UART initialization and configuration
   - Background task for continuous reading
   - Frame parsing with checksum validation
   - Automatic atmospheric environment values
   - Particle count data available

2. **LPS22HB implementation** (complete):
   - I2C initialization and device detection
   - WHO_AM_I verification (0xB1)
   - 25 Hz output data rate
   - Block data update enabled
   - Pressure reading in hPa
   - Temperature reading included

3. **I2C sensor stubs** (need implementation):
   - Temperature/Humidity sensor
   - VOC sensor
   - CO2 sensor

### PMSA003A Wiring

```
PMSA003A Pin → ESP32-C6
VCC (Pin 1)  → 5V
GND (Pin 2)  → GND
SET (Pin 3)  → Not connected (or 5V for continuous mode)
RX  (Pin 4)  → Not connected
TX  (Pin 5)  → GPIO 20 (ESP32 UART RX)
RESET (Pin 6)→ Not connected (or 5V)
NC  (Pin 7)  → Not connected
NC  (Pin 8)  → Not connected
```

**Note**: PMSA003A requires 5V power supply. TX output is 3.3V compatible.

### LPS22HB Wiring

```
LPS22HB Pin → ESP32-C6
VDD         → 3.3V
GND         → GND
SCL         → GPIO 7 (I2C SCL)
SDA         → GPIO 6 (I2C SDA)
SA0         → GND (for address 0x5C) or 3.3V (for 0x5D)
```

**Note**: LPS22HB operates at 1.7-3.6V. Use 3.3V supply.

## Removed Features

This project was adapted from an HVAC controller. The following features were removed:

- ❌ HVAC thermostat control
- ❌ Fan speed control
- ❌ Eco/Night/Swing modes
- ❌ UART communication with HVAC unit

All HVAC control logic has been replaced with air quality sensor reading functionality.

## Troubleshooting

### Device won't join network
- Check that Zigbee coordinator is in pairing mode
- Verify ESP32-C6 is powered correctly
- Check serial logs for error messages
- Try factory reset (hold button during boot)

### Sensors not responding
- **I2C sensors**:
  - Verify I2C wiring (SDA/SCL connections)
  - Check I2C pull-up resistors (usually 4.7kΩ)
  - Verify sensor power supply (3.3V for LPS22HB)
  - Use `i2cdetect` to scan for sensor addresses
  - Monitor I2C traffic in logs
  - **LPS22HB**: Check WHO_AM_I register (should be 0xB1)
  - **LPS22HB**: Verify address 0x5C or 0x5D (depends on SA0 pin)
- **PMSA003A (PM sensor)**:
  - Verify UART RX connection (PMSA003A TX → GPIO20)
  - Check 5V power supply to sensor
  - Ensure baud rate is 9600
  - Look for "PMSA003A data:" log messages
  - Check frame checksum errors in logs

### Values not updating
- Check that sensors are initialized correctly
- Verify periodic read task is running
- Check log output for sensor read errors
- Ensure reporting is configured on Zigbee coordinator

### Build errors
- Make sure ESP-IDF v5.5.1+ is installed
- Run `idf.py fullclean` and rebuild
- Check that all sensor libraries are in `idf_component.yml`

## License

This code is based on Espressif's Zigbee examples and inherits their licensing.

## Credits

- Base Zigbee framework: Espressif ESP-IDF examples
- Air quality sensor concept: Community contributions
- Original ESPHome component: See `CodeToReusePartially/acw02_esphome/`

## Future Enhancements

- [ ] Add button for factory reset
- [ ] Implement command retry logic
- [ ] Add OTA firmware update support
- [ ] Support horizontal swing control
- [ ] Add night mode support
- [ ] Implement filter status monitoring
- [ ] Add clean mode support
