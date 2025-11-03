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
    uint16_t co2_ppm;          // CO2 concentration in ppm
    bool sensor_error;         // Sensor error flag
    char error_text[64];       // Error description
} aeris_sensor_state_t;

/* I2C Configuration */
#define AERIS_I2C_NUM           I2C_NUM_0
#define AERIS_I2C_SDA_PIN       6    // GPIO6 (adjust for your board)
#define AERIS_I2C_SCL_PIN       7    // GPIO7 (adjust for your board)
#define AERIS_I2C_FREQ_HZ       100000

/* LPS22HB Pressure Sensor I2C Address */
#define LPS22HB_I2C_ADDR        0x5C  // 0x5C when SA0=0, 0x5D when SA0=1

/* UART Configuration for PMSA003A */
#define PMSA003A_UART_NUM       UART_NUM_1
#define PMSA003A_UART_TX_PIN    18   // GPIO18 (TX - not used for PMSA003A)
#define PMSA003A_UART_RX_PIN    20   // GPIO20 (RX from PMSA003A)
#define PMSA003A_UART_BAUD      9600
#define PMSA003A_UART_BUF_SIZE  512

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
 * @brief Read CO2 concentration
 * 
 * @param co2_ppm Pointer to CO2 in ppm
 * @return ESP_OK on success
 */
esp_err_t aeris_read_co2(uint16_t *co2_ppm);

#ifdef __cplusplus
}
#endif
