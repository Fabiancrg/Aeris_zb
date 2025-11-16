/*
 * Aeris Air Quality Sensor Driver Header
 *
 * Handles I2C/UART communication with air quality sensors
 * Supports Temperature, Humidity, Pressure, PM, VOC Index, and CO2 sensors
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Air Quality Sensor State structure */
typedef struct {
    float temperature_c;        // Temperature in Celsius
    float humidity_percent;     // Relative humidity in %
    float pressure_hpa;         // Atmospheric pressure in hPa
    float pm1_0_ug_m3;         // PM1.0 concentration in µg/m³
    float pm2_5_ug_m3;         // PM2.5 concentration in µg/m³
    float pm10_ug_m3;          // PM10 concentration in µg/m³
    uint16_t voc_index;        // VOC Index (1-500)
    uint16_t nox_index;        // NOx Index (1-500)
    uint16_t voc_raw;          // VOC raw signal
    uint16_t nox_raw;          // NOx raw signal
    uint16_t co2_ppm;          // CO2 concentration in ppm
    bool sensor_error;         // Sensor error flag
    char error_text[64];       // Error description
} aeris_sensor_state_t;

/* I2C Configuration */
#define AERIS_I2C_NUM           I2C_NUM_0
#define AERIS_I2C_SDA_PIN       6    // GPIO6 (adjust for your board)
#define AERIS_I2C_SCL_PIN       7    // GPIO7 (adjust for your board)
#define AERIS_I2C_FREQ_HZ       100000

/* SHT45 Temperature/Humidity Sensor I2C Address */
#define SHT45_I2C_ADDR          0x44  // Fixed I2C address

/* LPS22HB Pressure Sensor I2C Address */
#define LPS22HB_I2C_ADDR        0x5C  // 0x5C when SA0=0, 0x5D when SA0=1

/* SGP41 VOC/NOx Sensor I2C Address */
#define SGP41_I2C_ADDR          0x59  // Fixed I2C address

/* SCD40 CO2 Sensor I2C Address */
#define SCD40_I2C_ADDR          0x62  // Fixed I2C address

/* UART Configuration for PMSA003A Particulate Matter Sensor
 * 
 * PMSA003-A 10-pin Connector Pinout (from datasheet):
 *   Pin 1-2:  VCC (Positive power 5V)
 *   Pin 3-4:  GND (Negative power)
 *   Pin 5:    RESET (TTL 3.3V, active LOW, internal pull-up)
 *   Pin 6:    NC (Not connected - DO NOT CONNECT)
 *   Pin 7:    RXD (Serial port receiving pin, TTL 3.3V)
 *   Pin 8:    NC (Not connected - DO NOT CONNECT)
 *   Pin 9:    TXD (Serial port transmission pin, TTL 3.3V)
 *   Pin 10:   SET (Sleep/Wake, TTL 3.3V - HIGH=normal, LOW=sleep, internal pull-up)
 * 
 * Circuit Attentions:
 *   - 5V required for fan, but all signals are 3.3V (no level conversion for ESP32)
 *   - SET and RESET have internal pull-ups (can leave floating for continuous operation)
 *   - Pin 6 and Pin 8 MUST NOT be connected
 *   - Wait 30 seconds after wake from sleep for stable readings (fan stabilization)
 * 
 * ESP32-C6 Connections:
 *   GPIO 18 (UART TX) → PMSA003-A Pin 7 (RXD) - for commands (optional)
 *   GPIO 20 (UART RX) ← PMSA003-A Pin 9 (TXD) - for data (required)
 *   GPIO 19 → PMSA003-A Pin 10 (SET) - for sleep/wake control (recommended)
 *   GPIO 2 → PMSA003-A Pin 5 (RESET) - for hardware reset (optional)
 *   5V → Pin 1-2, GND → Pin 3-4
 * 
 * Power Savings with SET Pin Control:
 *   - Continuous operation: ~100 mA
 *   - Sleep mode: <1 mA
 *   - Polling every 5 minutes: ~28.5 mA average (71.5 mA savings)
 */
#define PMSA003A_UART_NUM       UART_NUM_1
#define PMSA003A_UART_TX_PIN    18   // GPIO18 → PMSA003A Pin 7 (RXD) for sending commands
#define PMSA003A_UART_RX_PIN    20   // GPIO20 ← PMSA003A Pin 9 (TXD) for receiving data
#define PMSA003A_UART_BAUD      9600
#define PMSA003A_UART_BUF_SIZE  512

/* PMSA003A operating modes */
typedef enum {
    PMSA003A_MODE_PASSIVE = 0,  // Continuous automatic output
    PMSA003A_MODE_ACTIVE = 1    // Command-driven with sleep/wake
} pmsa003a_mode_t;

/**
 * @brief Initialize air quality sensor driver
 * 
 * @return ESP_OK on success
 */
esp_err_t aeris_driver_init(void);

/**
 * @brief Get current sensor readings
 * 
 * @param state Pointer to state structure to fill
 * @return ESP_OK on success
 */
esp_err_t aeris_get_sensor_data(aeris_sensor_state_t *state);

/**
 * @brief Read temperature and humidity
 * 
 * @param temp_c Pointer to temperature in Celsius
 * @param humidity Pointer to humidity in %
 * @return ESP_OK on success
 */
esp_err_t aeris_read_temp_humidity(float *temp_c, float *humidity);

/**
 * @brief Read atmospheric pressure
 * 
 * @param pressure_hpa Pointer to pressure in hPa
 * @return ESP_OK on success
 */
esp_err_t aeris_read_pressure(float *pressure_hpa);

/**
 * @brief Read particulate matter concentrations
 * 
 * @param pm1_0 Pointer to PM1.0 concentration
 * @param pm2_5 Pointer to PM2.5 concentration
 * @param pm10 Pointer to PM10 concentration
 * @return ESP_OK on success
 */
esp_err_t aeris_read_pm(float *pm1_0, float *pm2_5, float *pm10);

/**
 * @brief Read VOC Index
 * 
 * @param voc_index Pointer to VOC Index value
 * @return ESP_OK on success
 */
esp_err_t aeris_read_voc(uint16_t *voc_index);

/**
 * @brief Read NOx Index
 * 
 * @param nox_index Pointer to NOx Index value
 * @return ESP_OK on success
 */
esp_err_t aeris_read_nox(uint16_t *nox_index);

/**
 * @brief Read VOC and NOx raw signals from SGP41
 * 
 * @param voc_raw Pointer to VOC raw signal
 * @param nox_raw Pointer to NOx raw signal
 * @return ESP_OK on success
 */
esp_err_t aeris_read_voc_nox_raw(uint16_t *voc_raw, uint16_t *nox_raw);

/**
 * @brief Read CO2 concentration
 * 
 * @param co2_ppm Pointer to CO2 in ppm
 * @return ESP_OK on success
 */
esp_err_t aeris_read_co2(uint16_t *co2_ppm);

/**
 * @brief Set PMSA003A polling interval (active mode only)
 * 
 * @param interval_seconds Polling interval in seconds (0 = continuous/passive mode)
 * @return ESP_OK on success
 */
esp_err_t aeris_set_pm_polling_interval(uint32_t interval_seconds);

/**
 * @brief Get current PMSA003A polling interval
 * 
 * @return Current polling interval in seconds (0 = continuous/passive mode)
 */
uint32_t aeris_get_pm_polling_interval(void);

/**
 * @brief Wake PMSA003A sensor from sleep
 * 
 * @return ESP_OK on success
 */
esp_err_t pmsa003a_wake(void);

/**
 * @brief Put PMSA003A sensor to sleep
 * 
 * @return ESP_OK on success
 */
esp_err_t pmsa003a_sleep(void);

/**
 * @brief Request PM reading from PMSA003A (active mode)
 * 
 * @return ESP_OK on success
 */
esp_err_t pmsa003a_request_read(void);

#ifdef __cplusplus
}
#endif
