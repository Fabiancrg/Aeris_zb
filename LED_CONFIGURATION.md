# RGB LED Air Quality Indicators

## Overview
The Aeris_zb air quality sensor includes **6 separate RGB LEDs (SK6812)** that provide independent visual feedback for each air quality parameter and system status:

- **CO2 LED** - Carbon dioxide level indicator
- **VOC LED** - Volatile organic compounds indicator
- **NOx LED** - Nitrogen oxides indicator
- **PM2.5 LED** - Particulate matter indicator
- **Humidity LED** - Humidity level indicator
- **Status LED** - Network / firmware status indicator (join, connected, error)

Each LED shows:
- **Green** - Parameter is in good range
- **Orange** - Parameter is not ideal (warning level)
- **Red** - Parameter is poor (danger level)

All 4 LEDs can be enabled/disabled together with a single Zigbee On/Off command.

## Hardware Connection
Connect 6 SK6812 RGB LEDs to the Xiao ESP32-C6:

| LED Purpose | GPIO | Connection |
|-------------|------|------------|
| CO2 | GPIO21 | Data line |
| VOC | GPIO4 | Data line |
| NOx | GPIO8 | Data line |
| PM2.5 | GPIO5 | Data line |
| Humidity | GPIO10 | Data line |
| Status | GPIO23 | Data line (status LED, controlled by firmware)

For each LED:
- Data → GPIO pin (see table above)
- VCC → 3.3V or 5V (depending on LED spec)
- GND → GND

You can change the GPIO pins in `main/board.h` if needed.

## Zigbee Configuration
The LED is controlled via **Endpoint 8** with the following clusters:

### On/Off Cluster (0x0006)
- **Attribute 0x0000 (OnOff)**: Enable/disable parameter LEDs (CO2, VOC, NOx, PM2.5, Humidity)
    - `true` = Parameter LEDs enabled (show air quality status)
    - `false` = Parameter LEDs disabled (always off)
    - **Note**: The Status LED is firmware-controlled and may remain active for critical network indicators depending on configuration.

### Analog Output Cluster (0x000D) - Threshold Attributes
Custom manufacturer-specific attributes for configuring thresholds:

| Attribute ID | Parameter | Default | Description |
|--------------|-----------|---------|-------------|
| 0xF000 | VOC Orange | 150 | VOC Index warning level (1-500 scale) |
| 0xF001 | VOC Red | 250 | VOC Index danger level |
| 0xF002 | CO2 Orange | 1000 | CO2 warning level (ppm) |
| 0xF003 | CO2 Red | 1500 | CO2 danger level (ppm) |
| 0xF004 | Humidity Orange Low | 30 | Too dry warning (%) |
| 0xF005 | Humidity Orange High | 70 | Too humid warning (%) |
| 0xF006 | Humidity Red Low | 20 | Too dry danger (%) |
| 0xF007 | Humidity Red High | 80 | Too humid danger (%) |
| 0xF008 | PM2.5 Orange | 25 | PM2.5 warning level (µg/m³) |
| 0xF009 | PM2.5 Red | 55 | PM2.5 danger level (µg/m³) |
| 0xF00A | NOx Orange | 200 | NOx warning level (ppb) |
| 0xF00B | NOx Red | 400 | NOx danger level (ppb) |

## Zigbee2MQTT Configuration

### Example Configuration
Add this to your Zigbee2MQTT device configuration or external converter:

```javascript
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

const definition = {
    zigbeeModel: ['aeris-z'],
    model: 'aeris-z',
    vendor: 'ESPRESSIF',
    description: 'Air Quality Sensor with RGB LED',
    
    fromZigbee: [fz.on_off, fz.analog_input],
    toZigbee: [tz.on_off],
    
    exposes: [
        // Endpoint 1: Temperature & Humidity
        e.temperature().withEndpoint('temp_hum'),
        e.humidity().withEndpoint('temp_hum'),
        
        // Endpoint 2: Pressure
        e.pressure().withEndpoint('pressure'),
        
        // Endpoint 3-5: PM sensors
        e.numeric('pm1_0', ea.STATE).withUnit('µg/m³').withDescription('PM1.0').withEndpoint('pm1'),
        e.numeric('pm2_5', ea.STATE).withUnit('µg/m³').withDescription('PM2.5').withEndpoint('pm25'),
        e.numeric('pm10', ea.STATE).withUnit('µg/m³').withDescription('PM10').withEndpoint('pm10'),
        
        // Endpoint 6: VOC
        e.numeric('voc_index', ea.STATE).withDescription('VOC Index (1-500)').withEndpoint('voc'),
        
        // Endpoint 7: CO2
        e.co2().withEndpoint('co2'),
        
        // Endpoint 8: LED Configuration
        e.binary('led_enabled', ea.ALL, true, false)
            .withDescription('Enable/disable air quality LED indicator')
            .withEndpoint('led_config'),
        
        // LED Thresholds
        e.numeric('voc_orange_threshold', ea.ALL).withValueMin(0).withValueMax(500)
            .withDescription('VOC Index orange (warning) threshold').withEndpoint('led_config'),
        e.numeric('voc_red_threshold', ea.ALL).withValueMin(0).withValueMax(500)
            .withDescription('VOC Index red (danger) threshold').withEndpoint('led_config'),
        
        e.numeric('co2_orange_threshold', ea.ALL).withValueMin(400).withValueMax(5000).withUnit('ppm')
            .withDescription('CO2 orange (warning) threshold').withEndpoint('led_config'),
        e.numeric('co2_red_threshold', ea.ALL).withValueMin(400).withValueMax(5000).withUnit('ppm')
            .withDescription('CO2 red (danger) threshold').withEndpoint('led_config'),
        
        e.numeric('humidity_orange_low', ea.ALL).withValueMin(0).withValueMax(100).withUnit('%')
            .withDescription('Humidity too dry - orange threshold').withEndpoint('led_config'),
        e.numeric('humidity_orange_high', ea.ALL).withValueMin(0).withValueMax(100).withUnit('%')
            .withDescription('Humidity too humid - orange threshold').withEndpoint('led_config'),
        
        e.numeric('humidity_red_low', ea.ALL).withValueMin(0).withValueMax(100).withUnit('%')
            .withDescription('Humidity too dry - red threshold').withEndpoint('led_config'),
        e.numeric('humidity_red_high', ea.ALL).withValueMin(0).withValueMax(100).withUnit('%')
            .withDescription('Humidity too humid - red threshold').withEndpoint('led_config'),
        
        e.numeric('pm25_orange_threshold', ea.ALL).withValueMin(0).withValueMax(500).withUnit('µg/m³')
            .withDescription('PM2.5 orange (warning) threshold').withEndpoint('led_config'),
        e.numeric('pm25_red_threshold', ea.ALL).withValueMin(0).withValueMax(500).withUnit('µg/m³')
            .withDescription('PM2.5 red (danger) threshold').withEndpoint('led_config'),
        // NOx thresholds (ppb)
        e.numeric('nox_orange_threshold', ea.ALL).withValueMin(0).withValueMax(10000).withUnit('ppb')
            .withDescription('NOx orange (warning) threshold').withEndpoint('led_config'),
        e.numeric('nox_red_threshold', ea.ALL).withValueMin(0).withValueMax(10000).withUnit('ppb')
            .withDescription('NOx red (danger) threshold').withEndpoint('led_config'),
    ],
    
    endpoint: (device) => {
        return {
            'temp_hum': 1,
            'pressure': 2,
            'pm1': 3,
            'pm25': 4,
            'pm10': 5,
            'voc': 6,
            'co2': 7,
            'led_config': 8,
        };
    },
    
    meta: {
        multiEndpoint: true,
    },
};

module.exports = definition;
```
    # Set NOx thresholds (ppb)
        {"nox_orange_threshold": 200, "nox_red_threshold": 400}

### Setting Thresholds via MQTT

To configure LED thresholds via MQTT, publish to:
```
zigbee2mqtt/[DEVICE_NAME]/set
```

Examples:
```json
# Enable LED
{"led_enabled": true}

# Disable LED
{"led_enabled": false}

# Set VOC thresholds
{"voc_orange_threshold": 150, "voc_red_threshold": 250}

# Set CO2 thresholds (ppm)
{"co2_orange_threshold": 1000, "co2_red_threshold": 1500}

# Set humidity thresholds (%)
{"humidity_orange_low": 30, "humidity_orange_high": 70, "humidity_red_low": 20, "humidity_red_high": 80}

# Set PM2.5 thresholds (µg/m³)
{"pm25_orange_threshold": 25, "pm25_red_threshold": 55}

# Set NOx thresholds (ppb)
{"nox_orange_threshold": 200, "nox_red_threshold": 400}
```

## Air Quality Guidelines

### VOC Index (Sensirion SGP41)
- 0-100: Excellent
- 101-150: Good
- **151-250: Not ideal** (Orange)
- **251-500: Poor** (Red)

### CO2 (ppm)
- 400-1000: Normal indoor levels
- **1001-1500: Not ideal** (Orange) - ventilation recommended
- **1501+: Poor** (Red) - increase ventilation

### Humidity (%)
- **<20%: Too dry** (Red) - health concerns
- **20-30%: Dry** (Orange) - not ideal
- 30-70%: Comfortable range
- **70-80%: Humid** (Orange) - may feel uncomfortable
- **>80%: Too humid** (Red) - mold risk

### PM2.5 (µg/m³) - EPA Air Quality Index
- 0-12: Good
- 13-25: Moderate
- **26-55: Unhealthy for sensitive groups** (Orange)
- **56+: Unhealthy** (Red)

## Home Assistant Integration

The LED configuration will appear as separate entities in Home Assistant:
- `switch.aeris_led_enabled` - Enable/disable LED
- `number.aeris_voc_orange_threshold` - VOC warning level
- `number.aeris_voc_red_threshold` - VOC danger level
- etc.

You can create automations or scripts to adjust thresholds based on your preferences.

## Troubleshooting

**LED not lighting up:**
- Check GPIO21 connection
- Verify LED power (3.3V or 5V depending on SK6812 spec)
- Check that LED is enabled via Zigbee (OnOff attribute = true)
- Monitor logs for LED initialization errors

**LED showing wrong color:**
- Verify sensor readings are accurate
- Check threshold configuration
- Remember: worst sensor condition determines color

**Cannot change thresholds:**
- Ensure device is properly paired with Zigbee2MQTT
- Check that endpoint 8 is accessible
- Verify attribute IDs are correctly mapped in Z2M converter
