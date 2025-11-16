# RGB LED Implementation Summary

## Overview
Successfully added **6 separate SK6812 RGB LEDs** with configurable air quality thresholds via Zigbee2MQTT. Each sensor parameter (CO2, VOC, PM2.5, Humidity, NOx) has its own dedicated LED indicator, plus a separate **status LED** for network/firmware state.

## Files Created/Modified

### New Files
1. **main/led_indicator.h** - LED driver API for 6 independent LEDs
2. **main/led_indicator.c** - SK6812 RMT driver with 6-channel support
3. **doc/LED_CONFIGURATION.md** - Complete user documentation for LED configuration

### Modified Files
1. **main/board.h** - Added 6 LED GPIO pin definitions:
   - CO2: GPIO21
   - VOC: GPIO4
   - NOx: GPIO8
   - PM2.5: GPIO5
   - Humidity: GPIO10
   - Status: GPIO23 (network/firmware status indicator)
2. **main/esp_zb_aeris.h** - Added:
   - LED configuration endpoint (endpoint 8)
   - 12 custom attribute IDs for thresholds (0xF000-0xF00B)
3. **main/esp_zb_aeris.c** - Added:
   - LED driver include
   - LED initialization in deferred_driver_init()
   - Endpoint 8 creation with On/Off + custom threshold attributes
   - Attribute handler for LED configuration changes
   - LED update call in sensor_update_zigbee_attributes()
4. **main/CMakeLists.txt** - Added esp_driver_rmt dependency

## Features Implemented

### Hardware
- **6 separate SK6812 RGB LEDs** using ESP32-C6 RMT peripheral
- Each LED on dedicated GPIO (GPIO21, GPIO4, GPIO8, GPIO5, GPIO10, GPIO23)
- **6 independent RMT channels** for simultaneous control (5 parameter LEDs + 1 status)
- Three color states per LED: Green (good), Orange (warning), Red (danger)
- Efficient RMT encoding for precise SK6812 timing

### Zigbee Integration
- **Endpoint 8**: LED Configuration
   - **Single On/Off cluster** for global enable/disable of all parameter LEDs (CO2, VOC, NOx, PM2.5, Humidity)
   - Analog Output cluster containing 12 custom threshold attributes
   - All parameter thresholds configurable via Zigbee2MQTT

### Threshold Parameters
All configurable via Zigbee attributes:

**VOC Index:**
- Orange threshold: 150 (default)
- Red threshold: 250 (default)

**CO2 (ppm):**
- Orange threshold: 1000 (default)
- Red threshold: 1500 (default)

**Humidity (%):**
- Orange low: 30% (default) - too dry warning
- Orange high: 70% (default) - too humid warning
- Red low: 20% (default) - too dry danger
- Red high: 80% (default) - too humid danger

**PM2.5 (µg/m³):**
- Orange threshold: 25 (default)
- Red threshold: 55 (default)

**NOx (ppb):**
- Orange threshold: 200 (default)
- Red threshold: 400 (default)

### LED Behavior
- **Each sensor has its own dedicated LED** showing independent status
- Evaluates sensors every 30 seconds
- **CO2 LED**: Shows CO2 level (green/orange/red based on CO2 thresholds)
- **VOC LED**: Shows VOC Index (green/orange/red based on VOC thresholds)
- **NOx LED**: Shows NOx level (green/orange/red based on NOx thresholds)
- **PM2.5 LED**: Shows particulate matter (green/orange/red based on PM2.5 thresholds)
- **Humidity LED**: Shows humidity level (green/orange/red based on humidity thresholds)
- **Status LED**: Shows network/firmware status (joining, connected, error) — behavior controlled by firmware, not thresholds
- **Single On/Off control** disables/enables all parameter LEDs; the status LED may remain active for critical network indicators depending on configuration
- Only updates each LED when its color changes (reduces RMT traffic)

## Default Thresholds Rationale

### VOC Index (Sensirion SGP41)
- 0-150: Good air quality → **Green**
- 151-250: Moderate, not ideal → **Orange**
- 251-500: Poor air quality → **Red**

### CO2 Levels
- 400-1000 ppm: Normal indoor → **Green**
- 1001-1500 ppm: Needs ventilation → **Orange**
- 1501+ ppm: Poor ventilation → **Red**

### Humidity
- 30-70%: Comfort zone → **Green**
- 20-30% or 70-80%: Outside comfort → **Orange**
- <20% or >80%: Health/mold risk → **Red**

### PM2.5 (EPA AQI)
- 0-25 µg/m³: Good/Moderate → **Green**
- 26-55 µg/m³: Unhealthy for sensitive → **Orange**
- 56+ µg/m³: Unhealthy → **Red**

## Usage Example

### From Zigbee2MQTT:
```json
# Enable all LEDs
{"led_enabled": true}

# Disable all LEDs
{"led_enabled": false}

# Adjust CO2 thresholds for stricter monitoring
{"co2_orange_threshold": 800, "co2_red_threshold": 1200}

# Make humidity range wider (20-75% before warning)
{"humidity_orange_low": 20, "humidity_orange_high": 75}
```

### In Home Assistant:
All thresholds appear as `number` entities. The single `switch.aeris_led_enabled` controls all parameter LEDs together (CO2, VOC, NOx, PM2.5, Humidity); the status LED behavior is firmware-controlled.

### LED Status Display:
With 4 separate LEDs, you can see the status of all parameters at a glance:
- All green = Excellent air quality across all parameters
- Mixed colors = Some parameters need attention
- Individual LED shows specific problem (e.g., red CO2 LED = ventilation needed)

## Testing Checklist
- [ ] Build project with `idf.py build`
- [ ] Flash to ESP32-C6
- [ ] Verify all 6 LEDs initialize (check serial logs)
- [ ] Pair with Zigbee coordinator
- [ ] Check that endpoint 8 appears in Zigbee2MQTT
- [ ] Test enabling/disabling all LEDs via On/Off attribute
- [ ] Adjust thresholds and verify appropriate LED changes color
- [ ] Test all 4 sensor thresholds independently:
  - [ ] VOC LED changes when VOC Index crosses thresholds
  - [ ] CO2 LED changes when CO2 crosses thresholds
  - [ ] Humidity LED changes when humidity crosses thresholds
  - [ ] PM2.5 LED changes when PM2.5 crosses thresholds
- [ ] Verify each LED operates independently

## Troubleshooting

**LEDs don't light:**
- Check GPIO wiring (GPIO21, GPIO4, GPIO8, GPIO5, GPIO10, GPIO23)
- Verify LED power supply for all 4 LEDs
- Check serial logs for RMT initialization errors
- Ensure LEDs are enabled (On/Off = true)

**Only some LEDs work:**
- Check individual GPIO connections
- Verify all 4 RMT channels initialized in logs
- Test each LED individually

**Can't change thresholds:**
- Verify endpoint 8 is exposed in Zigbee2MQTT
- Check custom attribute IDs are mapped correctly
- Try re-pairing device

**Wrong colors displayed:**
- Verify sensor readings are accurate (check sensor endpoints)
- Review threshold configuration for specific sensor
- Check logs for individual LED update messages

## Next Steps
1. Build and flash firmware
2. **Connect 4 SK6812 LEDs:**
   - CO2 LED → GPIO21
   - VOC LED → GPIO4
   - PM2.5 LED → GPIO5
   - Humidity LED → GPIO10
3. Pair with Zigbee coordinator
4. Configure thresholds in Zigbee2MQTT/Home Assistant
5. Fine-tune thresholds based on your environment
6. Observe each LED independently showing sensor status
