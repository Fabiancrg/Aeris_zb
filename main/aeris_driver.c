/*
 * Aeris Air Quality Sensor Driver Implementation
 *
 * I2C communication with air quality sensors
 * UART communication with PMSA003A particulate matter sensor
 * Supports Temperature, Humidity, Pressure, PM, VOC Index, and CO2 sensors
 */

#include "aeris_driver.h"
#include "board.h"
#include "esp_log.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "AERIS_DRIVER";

/* SHT45 Commands (high repeatability) */
#define SHT45_CMD_MEASURE_HIGH  0xFD  // Measure T & RH with high precision (8.2ms)
#define SHT45_CMD_SOFT_RESET    0x94  // Soft reset
#define SHT45_CMD_READ_SERIAL   0x89  // Read serial number

/* SHT45 timing constants (in ms) */
#define SHT45_MEASURE_TIME_MS   10    // Measurement duration for high precision
#define SHT45_RESET_TIME_MS     10    // Time after soft reset (datasheet says 1ms max, but add margin)

/* LPS22HB Register Addresses */
#define LPS22HB_WHO_AM_I        0x0F  // Device identification (should return 0xB1)
#define LPS22HB_CTRL_REG1       0x10  // Control register 1
#define LPS22HB_CTRL_REG2       0x11  // Control register 2
#define LPS22HB_CTRL_REG3       0x12  // Control register 3
#define LPS22HB_STATUS_REG      0x27  // Status register
#define LPS22HB_PRESS_OUT_XL    0x28  // Pressure output low byte
#define LPS22HB_PRESS_OUT_L     0x29  // Pressure output middle byte
#define LPS22HB_PRESS_OUT_H     0x2A  // Pressure output high byte
#define LPS22HB_TEMP_OUT_L      0x2B  // Temperature output low byte
#define LPS22HB_TEMP_OUT_H      0x2C  // Temperature output high byte

/* LPS22HB Constants */
#define LPS22HB_DEVICE_ID       0xB1  // WHO_AM_I expected value

/* SGP41 Commands */
#define SGP41_CMD_EXECUTE_CONDITIONING  0x2612  // Execute conditioning (10s)
#define SGP41_CMD_MEASURE_RAW_SIGNALS   0x2619  // Measure raw signals (50ms)
#define SGP41_CMD_EXECUTE_SELF_TEST     0x280E  // Execute self test (320ms)
#define SGP41_CMD_TURN_HEATER_OFF       0x3615  // Turn heater off (1ms)
#define SGP41_CMD_GET_SERIAL_NUMBER     0x3682  // Get serial number (1ms)

/* SGP41 timing constants (in ms) */
#define SGP41_CONDITIONING_TIME_MS      10
#define SGP41_MEASURE_TIME_MS           50
#define SGP41_SELFTEST_TIME_MS          320
#define SGP41_STARTUP_TIME_MS           170  // Time after power-on

/* SCD40 Commands */
#define SCD40_CMD_START_PERIODIC_MEASUREMENT    0x21B1  // Start periodic measurement
#define SCD40_CMD_READ_MEASUREMENT              0xEC05  // Read measurement
#define SCD40_CMD_STOP_PERIODIC_MEASUREMENT     0x3F86  // Stop periodic measurement
#define SCD40_CMD_SET_TEMPERATURE_OFFSET        0x241D  // Set temperature offset
#define SCD40_CMD_GET_TEMPERATURE_OFFSET        0x2318  // Get temperature offset
#define SCD40_CMD_SET_SENSOR_ALTITUDE           0x2427  // Set sensor altitude
#define SCD40_CMD_GET_SENSOR_ALTITUDE           0x2322  // Get sensor altitude
#define SCD40_CMD_SET_AMBIENT_PRESSURE          0xE000  // Set ambient pressure
#define SCD40_CMD_PERFORM_FORCED_RECALIBRATION  0x362F  // Perform forced recalibration
#define SCD40_CMD_SET_AUTOMATIC_SELF_CALIB      0x2416  // Set automatic self-calibration
#define SCD40_CMD_GET_AUTOMATIC_SELF_CALIB      0x2313  // Get automatic self-calibration
#define SCD40_CMD_GET_DATA_READY_STATUS         0xE4B8  // Get data ready status
#define SCD40_CMD_PERSIST_SETTINGS              0x3615  // Persist settings
#define SCD40_CMD_GET_SERIAL_NUMBER             0x3682  // Get serial number
#define SCD40_CMD_PERFORM_SELF_TEST             0x3639  // Perform self test
#define SCD40_CMD_PERFORM_FACTORY_RESET         0x3632  // Perform factory reset
#define SCD40_CMD_REINIT                        0x3646  // Re-initialization

/* SCD40 timing constants (in ms) */
#define SCD40_MEASUREMENT_INTERVAL_MS   5000   // Measurement interval (5 seconds)
#define SCD40_INITIAL_STARTUP_MS        1000   // Initial startup time
#define SCD40_STOP_PERIODIC_MS          500    // Time to stop periodic measurement
#define SCD40_READ_MEASUREMENT_MS       1      // Time to read measurement

/* PMSA003A frame structure */
#define PMSA003A_FRAME_LENGTH   32
#define PMSA003A_START_CHAR_1   0x42
#define PMSA003A_START_CHAR_2   0x4D

typedef struct {
    uint16_t pm1_0_cf1;      // PM1.0 concentration (CF=1, standard particle)
    uint16_t pm2_5_cf1;      // PM2.5 concentration (CF=1)
    uint16_t pm10_cf1;       // PM10 concentration (CF=1)
    uint16_t pm1_0_atm;      // PM1.0 concentration (atmospheric environment)
    uint16_t pm2_5_atm;      // PM2.5 concentration (atmospheric environment)
    uint16_t pm10_atm;       // PM10 concentration (atmospheric environment)
    uint16_t particles_0_3;  // Number of particles > 0.3µm in 0.1L of air
    uint16_t particles_0_5;  // Number of particles > 0.5µm
    uint16_t particles_1_0;  // Number of particles > 1.0µm
    uint16_t particles_2_5;  // Number of particles > 2.5µm
    uint16_t particles_5_0;  // Number of particles > 5.0µm
    uint16_t particles_10;   // Number of particles > 10µm
} pmsa003a_data_t;

/* Current sensor state */
static aeris_sensor_state_t current_state = {
    .temperature_c = 25.0f,
    .humidity_percent = 50.0f,
    .pressure_hpa = 1013.25f,
    .pm1_0_ug_m3 = 0.0f,
    .pm2_5_ug_m3 = 0.0f,
    .pm10_ug_m3 = 0.0f,
    .voc_index = 100,
    .nox_index = 1,
    .voc_raw = 0,
    .nox_raw = 0,
    .co2_ppm = 400,
    .sensor_error = false,
    .error_text = ""
};

/* PMSA003A latest data */
static pmsa003a_data_t pmsa003a_data = {0};
static bool pmsa003a_data_valid = false;

/* PMSA003A mode control */
static pmsa003a_mode_t pmsa003a_current_mode = PMSA003A_MODE_PASSIVE;
static uint32_t pmsa003a_polling_interval_s = 300; // Default 5 minutes
static TimerHandle_t pmsa003a_polling_timer = NULL;

/* SHT45 sensor state */
static bool sht45_initialized = false;
static uint32_t sht45_serial_number = 0;

/* Temperature offset compensation for self-heating (in °C)
 * Positive value = sensor reads higher than actual, so we subtract
 * Typical value: 2.0 to 4.0°C depending on PCB layout and airflow */
static float temperature_offset_c = 3.0f;  // Default 3°C offset

/* LPS22HB sensor state */
static bool lps22hb_initialized = false;

/* SGP41 sensor state */
static bool sgp41_initialized = false;
static uint64_t sgp41_serial_number = 0;
static TickType_t sgp41_last_measure_time = 0;

/* SCD40 sensor state */
static bool scd40_initialized = false;
static uint64_t scd40_serial_number = 0;

/* I2C master bus and device handles (new driver) */
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t sht45_dev_handle = NULL;
static i2c_master_dev_handle_t lps22hb_dev_handle = NULL;
static i2c_master_dev_handle_t sgp41_dev_handle = NULL;
static i2c_master_dev_handle_t scd40_dev_handle = NULL;

/**
 * @brief Calculate CRC8 for SHT45 and SGP41 (polynomial 0x31, init 0xFF)
 */
static uint8_t sensirion_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Initialize SHT45 temperature and humidity sensor
 */
static esp_err_t sht45_init(void)
{
    ESP_LOGI(TAG, "Initializing SHT45 temperature/humidity sensor...");
    
    if (!sht45_dev_handle) {
        ESP_LOGE(TAG, "SHT45 device handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send soft reset command
    uint8_t reset_cmd = SHT45_CMD_SOFT_RESET;
    esp_err_t ret = i2c_master_transmit(sht45_dev_handle, &reset_cmd, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT45 soft reset failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(SHT45_RESET_TIME_MS));
    
    // Read serial number to verify communication
    uint8_t serial_cmd = SHT45_CMD_READ_SERIAL;
    ret = i2c_master_transmit(sht45_dev_handle, &serial_cmd, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT45 read serial command failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for serial number to be ready
    
    // Read 6 bytes (2 bytes serial + CRC, 2 bytes serial + CRC)
    uint8_t serial_data[6];
    ret = i2c_master_receive(sht45_dev_handle, serial_data, 6, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT45 read serial data failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Verify CRC for both words
    uint8_t crc1 = sensirion_crc8(&serial_data[0], 2);
    uint8_t crc2 = sensirion_crc8(&serial_data[3], 2);
    if (crc1 != serial_data[2] || crc2 != serial_data[5]) {
        ESP_LOGE(TAG, "SHT45 serial number CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Store serial number
    sht45_serial_number = ((uint32_t)serial_data[0] << 24) | 
                          ((uint32_t)serial_data[1] << 16) |
                          ((uint32_t)serial_data[3] << 8) | 
                          serial_data[4];
    
    ESP_LOGI(TAG, "SHT45 initialized successfully. Serial: 0x%08lX", sht45_serial_number);
    sht45_initialized = true;
    
    return ESP_OK;
}

/**
 * @brief Read temperature and humidity from SHT45
 */
static esp_err_t sht45_read_temp_humidity(float *temp_c, float *humidity_percent)
{
    if (!sht45_initialized || !sht45_dev_handle) {
        ESP_LOGE(TAG, "SHT45 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send measurement command (high repeatability)
    uint8_t measure_cmd = SHT45_CMD_MEASURE_HIGH;
    esp_err_t ret = i2c_master_transmit(sht45_dev_handle, &measure_cmd, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT45 measure command failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(SHT45_MEASURE_TIME_MS));
    
    // Read 6 bytes (2 temp + CRC, 2 RH + CRC)
    uint8_t data[6];
    ret = i2c_master_receive(sht45_dev_handle, data, 6, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT45 read measurement failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Verify CRC for both words
    uint8_t temp_crc = sensirion_crc8(&data[0], 2);
    uint8_t rh_crc = sensirion_crc8(&data[3], 2);
    if (temp_crc != data[2] || rh_crc != data[5]) {
        ESP_LOGE(TAG, "SHT45 measurement CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Convert raw values to temperature and humidity
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t rh_raw = (data[3] << 8) | data[4];
    
    // SHT45 conversion formulas from datasheet
    float raw_temp = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    
    // Apply temperature offset compensation for self-heating
    *temp_c = raw_temp - temperature_offset_c;
    
    *humidity_percent = -6.0f + 125.0f * ((float)rh_raw / 65535.0f);
    
    // Clamp humidity to valid range
    if (*humidity_percent < 0.0f) *humidity_percent = 0.0f;
    if (*humidity_percent > 100.0f) *humidity_percent = 100.0f;
    
    return ESP_OK;
}

/**
 * @brief Send command to SCD40 and read response
 */
static esp_err_t scd40_send_command(uint16_t cmd, uint8_t *response, size_t response_len, uint32_t delay_ms)
{
    if (!scd40_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t cmd_buf[2];
    cmd_buf[0] = cmd >> 8;
    cmd_buf[1] = cmd & 0xFF;
    
    // Send command
    esp_err_t ret = i2c_master_transmit(scd40_dev_handle, cmd_buf, 2, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD40 send command 0x%04X failed: %s", cmd, esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for command execution
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    // Read response if expected
    if (response && response_len > 0) {
        ret = i2c_master_receive(scd40_dev_handle, response, response_len, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SCD40 read response failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Verify CRC for each 2-byte word (every 3 bytes: 2 data + 1 CRC)
        for (size_t i = 0; i < response_len; i += 3) {
            uint8_t crc_calc = sensirion_crc8(&response[i], 2);
            if (crc_calc != response[i + 2]) {
                ESP_LOGE(TAG, "SCD40 CRC mismatch at byte %d: calc=0x%02X, recv=0x%02X", 
                         i, crc_calc, response[i + 2]);
                return ESP_ERR_INVALID_CRC;
            }
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Initialize SCD40 CO2 sensor
 */
static esp_err_t scd40_init(void)
{
    ESP_LOGI(TAG, "Initializing SCD40 CO2 sensor...");
    
    // Wait for sensor startup
    vTaskDelay(pdMS_TO_TICKS(SCD40_INITIAL_STARTUP_MS));
    
    // Stop any ongoing periodic measurement
    esp_err_t ret = scd40_send_command(SCD40_CMD_STOP_PERIODIC_MEASUREMENT, NULL, 0, SCD40_STOP_PERIODIC_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD40 stop periodic measurement failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Read serial number to verify communication (9 bytes: 3 words of 2 bytes + CRC)
    uint8_t serial_data[9];
    ret = scd40_send_command(SCD40_CMD_GET_SERIAL_NUMBER, serial_data, 9, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD40 read serial number failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Extract serial number (48 bits from 3 words)
    scd40_serial_number = ((uint64_t)serial_data[0] << 40) | 
                          ((uint64_t)serial_data[1] << 32) |
                          ((uint64_t)serial_data[3] << 24) | 
                          ((uint64_t)serial_data[4] << 16) |
                          ((uint64_t)serial_data[6] << 8) | 
                          serial_data[7];
    
    ESP_LOGI(TAG, "SCD40 serial number: 0x%012llX", scd40_serial_number);
    
    // Start periodic measurement
    ret = scd40_send_command(SCD40_CMD_START_PERIODIC_MEASUREMENT, NULL, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD40 start periodic measurement failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "SCD40 initialized successfully. Measurements will be available in ~5 seconds.");
    scd40_initialized = true;
    
    return ESP_OK;
}

/**
 * @brief Read CO2, temperature, and humidity from SCD40
 */
static esp_err_t scd40_read_measurement(uint16_t *co2_ppm, float *temp_c, float *humidity_percent)
{
    if (!scd40_initialized) {
        ESP_LOGE(TAG, "SCD40 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if data is ready (3 bytes: 1 word + CRC)
    uint8_t status_data[3];
    esp_err_t ret = scd40_send_command(SCD40_CMD_GET_DATA_READY_STATUS, status_data, 3, SCD40_READ_MEASUREMENT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD40 data ready check failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint16_t data_ready = (status_data[0] << 8) | status_data[1];
    if ((data_ready & 0x07FF) == 0) {
        // Data not ready yet
        return ESP_ERR_NOT_FOUND;
    }
    
    // Read measurement (9 bytes: CO2, temp, RH - each 2 bytes + CRC)
    uint8_t meas_data[9];
    ret = scd40_send_command(SCD40_CMD_READ_MEASUREMENT, meas_data, 9, SCD40_READ_MEASUREMENT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD40 read measurement failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Extract CO2 (ppm)
    *co2_ppm = (meas_data[0] << 8) | meas_data[1];
    
    // Extract temperature (°C) = -45 + 175 * (value / 65536)
    uint16_t temp_raw = (meas_data[3] << 8) | meas_data[4];
    *temp_c = -45.0f + 175.0f * ((float)temp_raw / 65536.0f);
    
    // Extract humidity (%) = 100 * (value / 65536)
    uint16_t rh_raw = (meas_data[6] << 8) | meas_data[7];
    *humidity_percent = 100.0f * ((float)rh_raw / 65536.0f);
    
    return ESP_OK;
}

/**
 * @brief Set ambient pressure compensation for SCD40
 * @param pressure_hpa Ambient pressure in hPa (700-1200 hPa range)
 */
static esp_err_t scd40_set_ambient_pressure(uint16_t pressure_hpa)
{
    if (!scd40_initialized) {
        ESP_LOGE(TAG, "SCD40 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Pressure must be in range 700-1200 hPa and in units of hPa
    if (pressure_hpa < 700 || pressure_hpa > 1200) {
        ESP_LOGW(TAG, "SCD40 pressure %d hPa out of range (700-1200), clamping", pressure_hpa);
        if (pressure_hpa < 700) pressure_hpa = 700;
        if (pressure_hpa > 1200) pressure_hpa = 1200;
    }
    
    // Prepare command with pressure parameter
    uint8_t cmd_buf[5];
    cmd_buf[0] = SCD40_CMD_SET_AMBIENT_PRESSURE >> 8;
    cmd_buf[1] = SCD40_CMD_SET_AMBIENT_PRESSURE & 0xFF;
    cmd_buf[2] = pressure_hpa >> 8;
    cmd_buf[3] = pressure_hpa & 0xFF;
    cmd_buf[4] = sensirion_crc8(&cmd_buf[2], 2);
    
    if (!scd40_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = i2c_master_transmit(scd40_dev_handle, cmd_buf, 5, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD40 set ambient pressure failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "SCD40 ambient pressure set to %d hPa", pressure_hpa);
    return ESP_OK;
}

/**
 * @brief Calculate CRC8 for SGP41 (polynomial 0x31, init 0xFF)
 */
static uint8_t sgp41_crc8(const uint8_t *data, size_t len)
{
    return sensirion_crc8(data, len);
}

/**
 * @brief Send command to SGP41 and read response
 */
static esp_err_t sgp41_send_command(uint16_t cmd, const uint8_t *params, size_t param_len,
                                     uint8_t *response, size_t response_len, uint32_t delay_ms)
{
    if (!sgp41_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Build complete command buffer with parameters and CRCs
    // Max size: 2 (cmd) + 6 * 3 (6 params with CRCs) = 20 bytes
    uint8_t full_cmd[20];
    size_t full_cmd_len = 0;
    
    // Add command bytes
    full_cmd[full_cmd_len++] = cmd >> 8;
    full_cmd[full_cmd_len++] = cmd & 0xFF;
    
    // Add parameters with CRCs (each parameter is 2 bytes + 1 CRC)
    if (params && param_len > 0) {
        for (size_t i = 0; i < param_len; i += 2) {
            full_cmd[full_cmd_len++] = params[i];
            full_cmd[full_cmd_len++] = params[i + 1];
            full_cmd[full_cmd_len++] = sgp41_crc8(&params[i], 2);
        }
    }
    
    // Send complete command with all parameters
    esp_err_t ret = i2c_master_transmit(sgp41_dev_handle, full_cmd, full_cmd_len, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SGP41 send command 0x%04X failed: %s", cmd, esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for measurement
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    // Read response if expected
    if (response && response_len > 0) {
        ret = i2c_master_receive(sgp41_dev_handle, response, response_len, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SGP41 read response failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Verify CRC for each 2-byte word
        for (size_t i = 0; i < response_len; i += 3) {
            uint8_t crc_calc = sgp41_crc8(&response[i], 2);
            if (crc_calc != response[i + 2]) {
                ESP_LOGE(TAG, "SGP41 CRC mismatch: calc=0x%02X, recv=0x%02X", 
                         crc_calc, response[i + 2]);
                return ESP_ERR_INVALID_CRC;
            }
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Initialize SGP41 sensor
 */
static esp_err_t sgp41_init(void)
{
    ESP_LOGI(TAG, "Initializing SGP41 VOC/NOx sensor...");
    
    // Wait for sensor startup
    vTaskDelay(pdMS_TO_TICKS(SGP41_STARTUP_TIME_MS));
    
    // Get serial number
    uint8_t serial_data[9];  // 3 words of 2 bytes + CRC each
    esp_err_t ret = sgp41_send_command(SGP41_CMD_GET_SERIAL_NUMBER, NULL, 0,
                                        serial_data, sizeof(serial_data), 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SGP41 not responding on I2C");
        return ret;
    }
    
    // Extract serial number (remove CRC bytes)
    sgp41_serial_number = ((uint64_t)serial_data[0] << 40) |
                          ((uint64_t)serial_data[1] << 32) |
                          ((uint64_t)serial_data[3] << 24) |
                          ((uint64_t)serial_data[4] << 16) |
                          ((uint64_t)serial_data[6] << 8) |
                          ((uint64_t)serial_data[7]);
    
    ESP_LOGI(TAG, "SGP41 detected, serial: 0x%012llX", sgp41_serial_number);
    
    // Execute self-test
    uint8_t test_result[3];
    ret = sgp41_send_command(SGP41_CMD_EXECUTE_SELF_TEST, NULL, 0,
                            test_result, sizeof(test_result), SGP41_SELFTEST_TIME_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SGP41 self-test failed");
        return ret;
    }
    
    uint16_t test_value = (test_result[0] << 8) | test_result[1];
    if (test_value != 0xD400) {  // 0xD400 = all tests passed
        ESP_LOGW(TAG, "SGP41 self-test result: 0x%04X (expected 0xD400)", test_value);
    } else {
        ESP_LOGI(TAG, "SGP41 self-test passed");
    }
    
    sgp41_initialized = true;
    sgp41_last_measure_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "SGP41 initialized successfully");
    return ESP_OK;
}

/**
 * @brief Read raw VOC and NOx signals from SGP41
 */
static esp_err_t sgp41_measure_raw_signals(uint16_t *voc_raw, uint16_t *nox_raw,
                                           float rh_percent, float temp_c)
{
    if (!sgp41_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert RH and temperature to SGP41 format (RH% * 65535 / 100, Temp°C * 65535 / 175)
    uint16_t rh_ticks = (uint16_t)((rh_percent * 65535.0f) / 100.0f);
    uint16_t temp_ticks = (uint16_t)(((temp_c + 45.0f) * 65535.0f) / 175.0f);
    
    // Prepare parameters: RH (2 bytes) + Temp (2 bytes)
    uint8_t params[4];
    params[0] = rh_ticks >> 8;
    params[1] = rh_ticks & 0xFF;
    params[2] = temp_ticks >> 8;
    params[3] = temp_ticks & 0xFF;
    
    // Read raw signals
    uint8_t response[6];  // VOC (2 bytes + CRC) + NOx (2 bytes + CRC)
    esp_err_t ret = sgp41_send_command(SGP41_CMD_MEASURE_RAW_SIGNALS, params, 4,
                                        response, sizeof(response), SGP41_MEASURE_TIME_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *voc_raw = (response[0] << 8) | response[1];
    *nox_raw = (response[3] << 8) | response[4];
    
    sgp41_last_measure_time = xTaskGetTickCount();
    
    return ESP_OK;
}

/**
 * @brief Write to LPS22HB register
 */
static esp_err_t lps22hb_write_reg(uint8_t reg, uint8_t value)
{
    if (!lps22hb_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t write_buf[2] = {reg, value};
    esp_err_t ret = i2c_master_transmit(lps22hb_dev_handle, write_buf, sizeof(write_buf),
                                        pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LPS22HB write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Read from LPS22HB register
 */
static esp_err_t lps22hb_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    if (!lps22hb_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = i2c_master_transmit_receive(lps22hb_dev_handle, &reg, 1, data, len,
                                                 pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LPS22HB read reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Initialize LPS22HB pressure sensor
 */
static esp_err_t lps22hb_init(void)
{
    ESP_LOGI(TAG, "Initializing LPS22HB pressure sensor...");
    
    // Check WHO_AM_I
    uint8_t device_id;
    esp_err_t ret = lps22hb_read_reg(LPS22HB_WHO_AM_I, &device_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LPS22HB not responding on I2C");
        return ret;
    }
    
    if (device_id != LPS22HB_DEVICE_ID) {
        ESP_LOGE(TAG, "LPS22HB WHO_AM_I mismatch: expected 0x%02X, got 0x%02X",
                 LPS22HB_DEVICE_ID, device_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "LPS22HB detected, device ID: 0x%02X", device_id);
    
    // Software reset
    ret = lps22hb_write_reg(LPS22HB_CTRL_REG2, 0x04);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure sensor
    // CTRL_REG1: ODR=25Hz (0b011), Enable block data update (BDU=1)
    // ODR bits [6:4]: 000=one-shot, 001=1Hz, 010=10Hz, 011=25Hz, 100=50Hz, 101=75Hz
    ret = lps22hb_write_reg(LPS22HB_CTRL_REG1, 0x3A);  // 0b00111010 = 25Hz, BDU=1, LPF enabled
    if (ret != ESP_OK) return ret;
    
    lps22hb_initialized = true;
    ESP_LOGI(TAG, "LPS22HB initialized successfully (25 Hz, BDU enabled)");
    
    return ESP_OK;
}

/**
 * @brief Read pressure and temperature from LPS22HB
 */
static esp_err_t lps22hb_read_data(float *pressure_hpa, float *temperature_c)
{
    if (!lps22hb_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if new data is available
    uint8_t status;
    esp_err_t ret = lps22hb_read_reg(LPS22HB_STATUS_REG, &status, 1);
    if (ret != ESP_OK) return ret;
    
    // Read pressure (24-bit value)
    uint8_t press_data[3];
    ret = lps22hb_read_reg(LPS22HB_PRESS_OUT_XL, press_data, 3);
    if (ret != ESP_OK) return ret;
    
    // Read temperature (16-bit value)
    uint8_t temp_data[2];
    ret = lps22hb_read_reg(LPS22HB_TEMP_OUT_L, temp_data, 2);
    if (ret != ESP_OK) return ret;
    
    // Convert pressure (24-bit signed integer to hPa)
    int32_t press_raw = (int32_t)((press_data[2] << 16) | (press_data[1] << 8) | press_data[0]);
    // Sign extend from 24-bit to 32-bit
    if (press_raw & 0x800000) {
        press_raw |= 0xFF000000;
    }
    *pressure_hpa = press_raw / 4096.0f;  // LSB = 1/4096 hPa
    
    // Convert temperature (16-bit signed integer to °C)
    int16_t temp_raw = (int16_t)((temp_data[1] << 8) | temp_data[0]);
    *temperature_c = temp_raw / 100.0f;  // LSB = 1/100 °C
    
    return ESP_OK;
}

/**
 * @brief Initialize PMSA003A UART
 */
static esp_err_t pmsa003a_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = PMSA003A_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t err = uart_param_config(PMSA003A_UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PMSA003A UART param config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = uart_set_pin(PMSA003A_UART_NUM, PMSA003A_UART_TX_PIN, PMSA003A_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PMSA003A UART set pin failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = uart_driver_install(PMSA003A_UART_NUM, PMSA003A_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PMSA003A UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Initialize SET pin (sleep/wake control)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PMSA003A_SET_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,  // Internal pull-up exists on sensor
    };
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PMSA003A SET GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Start with sensor awake (SET pin HIGH)
    gpio_set_level(PMSA003A_SET_GPIO, 1);
    ESP_LOGI(TAG, "PMSA003A SET pin (GPIO%d) initialized HIGH (sensor active)", PMSA003A_SET_GPIO);
    
    // Optional: Initialize RESET pin (for hardware reset capability)
    #ifdef PMSA003A_RESET_GPIO
    io_conf.pin_bit_mask = (1ULL << PMSA003A_RESET_GPIO);
    err = gpio_config(&io_conf);
    if (err == ESP_OK) {
        gpio_set_level(PMSA003A_RESET_GPIO, 1);  // Keep HIGH for normal operation
        ESP_LOGI(TAG, "PMSA003A RESET pin (GPIO%d) initialized HIGH (normal operation)", PMSA003A_RESET_GPIO);
    }
    #endif
    
    ESP_LOGI(TAG, "PMSA003A UART initialized on RX=%d, baud=%d", 
             PMSA003A_UART_RX_PIN, PMSA003A_UART_BAUD);
    return ESP_OK;
}

/**
 * @brief Calculate checksum for PMSA003A frame
 */
static uint16_t pmsa003a_checksum(const uint8_t *data, size_t len)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief Send command to PMSA003A sensor
 */
static esp_err_t pmsa003a_send_command(const uint8_t *cmd, size_t len)
{
    int written = uart_write_bytes(PMSA003A_UART_NUM, (const char *)cmd, len);
    if (written != len) {
        ESP_LOGE(TAG, "PMSA003A command send failed: %d/%d bytes", written, len);
        return ESP_FAIL;
    }
    
    uart_wait_tx_done(PMSA003A_UART_NUM, pdMS_TO_TICKS(100));
    ESP_LOGD(TAG, "PMSA003A command sent: %d bytes", len);
    return ESP_OK;
}

/**
 * @brief Wake up PMSA003A sensor
 */
esp_err_t pmsa003a_wake(void)
{
    // Set GPIO HIGH to wake sensor (must be done before UART command)
    gpio_set_level(PMSA003A_SET_GPIO, 1);
    ESP_LOGD(TAG, "PMSA003A SET pin HIGH (wake)");
    
    // Wait a moment for sensor to wake
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Wake command: 0x42 0x4D 0xE4 0x00 0x01 0x01 0x74
    const uint8_t wake_cmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
    
    esp_err_t err = pmsa003a_send_command(wake_cmd, sizeof(wake_cmd));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PMSA003A waking up, waiting 30s for stabilization");
        pmsa003a_current_mode = PMSA003A_MODE_ACTIVE;
    }
    return err;
}

/**
 * @brief Put PMSA003A sensor to sleep
 */
esp_err_t pmsa003a_sleep(void)
{
    // Sleep command: 0x42 0x4D 0xE4 0x00 0x00 0x01 0x73
    const uint8_t sleep_cmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
    
    esp_err_t err = pmsa003a_send_command(sleep_cmd, sizeof(sleep_cmd));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PMSA003A entering sleep mode");
        pmsa003a_current_mode = PMSA003A_MODE_PASSIVE;
        pmsa003a_data_valid = false; // Invalidate old data
        
        // Wait for command to complete
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Set GPIO LOW to put sensor to sleep (hardware sleep)
        gpio_set_level(PMSA003A_SET_GPIO, 0);
        ESP_LOGD(TAG, "PMSA003A SET pin LOW (sleep)");
    }
    return err;
}

/**
 * @brief Request read from PMSA003A in active mode
 */
esp_err_t pmsa003a_request_read(void)
{
    // Active mode read command: 0x42 0x4D 0xE2 0x00 0x00 0x01 0x71
    const uint8_t read_cmd[] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};
    
    ESP_LOGD(TAG, "PMSA003A requesting read in active mode");
    return pmsa003a_send_command(read_cmd, sizeof(read_cmd));
}

/**
 * @brief Parse PMSA003A data frame
 */
static bool pmsa003a_parse_frame(const uint8_t *frame)
{
    // Verify start characters
    if (frame[0] != PMSA003A_START_CHAR_1 || frame[1] != PMSA003A_START_CHAR_2) {
        return false;
    }
    
    // Verify frame length (should be 28 data bytes)
    uint16_t frame_len = (frame[2] << 8) | frame[3];
    if (frame_len != 28) {
        ESP_LOGW(TAG, "PMSA003A invalid frame length: %d", frame_len);
        return false;
    }
    
    // Verify checksum
    uint16_t checksum_calc = pmsa003a_checksum(frame, 30);
    uint16_t checksum_recv = (frame[30] << 8) | frame[31];
    if (checksum_calc != checksum_recv) {
        ESP_LOGW(TAG, "PMSA003A checksum mismatch: calc=0x%04X, recv=0x%04X", 
                 checksum_calc, checksum_recv);
        return false;
    }
    
    // Parse data (all values are big-endian uint16)
    pmsa003a_data.pm1_0_cf1 = (frame[4] << 8) | frame[5];
    pmsa003a_data.pm2_5_cf1 = (frame[6] << 8) | frame[7];
    pmsa003a_data.pm10_cf1 = (frame[8] << 8) | frame[9];
    pmsa003a_data.pm1_0_atm = (frame[10] << 8) | frame[11];
    pmsa003a_data.pm2_5_atm = (frame[12] << 8) | frame[13];
    pmsa003a_data.pm10_atm = (frame[14] << 8) | frame[15];
    pmsa003a_data.particles_0_3 = (frame[16] << 8) | frame[17];
    pmsa003a_data.particles_0_5 = (frame[18] << 8) | frame[19];
    pmsa003a_data.particles_1_0 = (frame[20] << 8) | frame[21];
    pmsa003a_data.particles_2_5 = (frame[22] << 8) | frame[23];
    pmsa003a_data.particles_5_0 = (frame[24] << 8) | frame[25];
    pmsa003a_data.particles_10 = (frame[26] << 8) | frame[27];
    
    // Update current state with atmospheric environment values (more accurate for real conditions)
    current_state.pm1_0_ug_m3 = pmsa003a_data.pm1_0_atm;
    current_state.pm2_5_ug_m3 = pmsa003a_data.pm2_5_atm;
    current_state.pm10_ug_m3 = pmsa003a_data.pm10_atm;
    
    pmsa003a_data_valid = true;
    
    ESP_LOGD(TAG, "PMSA003A: PM1.0=%d, PM2.5=%d, PM10=%d µg/m³", 
             pmsa003a_data.pm1_0_atm, pmsa003a_data.pm2_5_atm, pmsa003a_data.pm10_atm);
    
    return true;
}

/**
 * @brief Timer callback for PMSA003A polling
 */
static void pmsa003a_polling_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "PMSA003A polling timer triggered");
    
    // Wake sensor
    pmsa003a_wake();
    
    // Sensor will automatically start sending data in passive mode
    // Data will be read by pmsa003a_task
    
    // After 35 seconds (30s warm-up + 5s reading), put sensor back to sleep
    // This is handled by a task notification or another timer
    // For simplicity, we'll use vTaskDelay in a simple approach
}

/**
 * @brief PMSA003A read task
 */
static void pmsa003a_task(void *arg)
{
    uint8_t frame[PMSA003A_FRAME_LENGTH];
    uint8_t buffer[256];
    size_t buffer_len = 0;
    TickType_t wake_time = 0;
    bool waiting_for_sleep = false;
    
    ESP_LOGI(TAG, "PMSA003A read task started");
    
    // Initialize uart_read_bytes return value check
    memset(buffer, 0, sizeof(buffer));
    
    // Start in sleep mode with default 5-minute polling
    pmsa003a_sleep();
    
    while (1) {
        // Check if we need to sleep the sensor after wake period
        if (waiting_for_sleep && wake_time != 0 && 
            (xTaskGetTickCount() - wake_time) > pdMS_TO_TICKS(35000)) {
            ESP_LOGI(TAG, "PMSA003A wake period complete, going to sleep");
            pmsa003a_sleep();
            waiting_for_sleep = false;
            wake_time = 0;  // Reset for next cycle
        }
        
        // If sensor just woke up, track the time
        if (pmsa003a_current_mode == PMSA003A_MODE_ACTIVE && !waiting_for_sleep && wake_time == 0) {
            wake_time = xTaskGetTickCount();
            waiting_for_sleep = true;
            ESP_LOGI(TAG, "PMSA003A starting wake period");
        }
        
        // Read available data with safety check
        size_t space_available = sizeof(buffer) - buffer_len;
        if (space_available < 32) {
            ESP_LOGW(TAG, "PMSA003A buffer nearly full, resetting");
            buffer_len = 0;
            space_available = sizeof(buffer);
        }
        
        int len = uart_read_bytes(PMSA003A_UART_NUM, buffer + buffer_len, 
                                  space_available, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            buffer_len += len;
            
            // Look for frame start
            for (size_t i = 0; i < buffer_len - 1; i++) {
                if (buffer[i] == PMSA003A_START_CHAR_1 && buffer[i+1] == PMSA003A_START_CHAR_2) {
                    // Found potential frame start
                    if (buffer_len - i >= PMSA003A_FRAME_LENGTH) {
                        // Have complete frame
                        memcpy(frame, buffer + i, PMSA003A_FRAME_LENGTH);
                        
                        if (pmsa003a_parse_frame(frame)) {
                            ESP_LOGI(TAG, "PMSA003A data: PM1.0=%d, PM2.5=%d, PM10=%d µg/m³",
                                     pmsa003a_data.pm1_0_atm, pmsa003a_data.pm2_5_atm, 
                                     pmsa003a_data.pm10_atm);
                            
                            // Data is valid - wake_time is managed by sleep logic, not here
                        }
                        
                        // Remove processed frame from buffer
                        size_t remaining = buffer_len - i - PMSA003A_FRAME_LENGTH;
                        if (remaining > 0) {
                            memmove(buffer, buffer + i + PMSA003A_FRAME_LENGTH, remaining);
                        }
                        buffer_len = remaining;
                        break;
                    }
                }
            }
            
            // Prevent buffer overflow (should not happen with above check, but safety)
            if (buffer_len > sizeof(buffer) - 64) {
                ESP_LOGW(TAG, "PMSA003A buffer unexpectedly full, resetting");
                buffer_len = 0;
            }
        } else if (len < 0) {
            // UART error occurred
            ESP_LOGE(TAG, "PMSA003A UART read error: %d", len);
            uart_flush_input(PMSA003A_UART_NUM);
            buffer_len = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Initialize I2C bus for sensors (new i2c_master driver)
 */
static esp_err_t i2c_master_init(void)
{
    /* Configure I2C bus */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = AERIS_I2C_SCL_PIN,
        .sda_io_num = AERIS_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus creation failed: %s", esp_err_to_name(err));
        return err;
    }
    
    /* Add SHT45 device */
    i2c_device_config_t sht45_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT45_I2C_ADDR,
        .scl_speed_hz = AERIS_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(i2c_bus_handle, &sht45_cfg, &sht45_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add SHT45 device: %s", esp_err_to_name(err));
    }
    
    /* Add LPS22HB device */
    i2c_device_config_t lps22hb_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LPS22HB_I2C_ADDR,
        .scl_speed_hz = AERIS_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(i2c_bus_handle, &lps22hb_cfg, &lps22hb_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add LPS22HB device: %s", esp_err_to_name(err));
    }
    
    /* Add SGP41 device */
    i2c_device_config_t sgp41_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SGP41_I2C_ADDR,
        .scl_speed_hz = AERIS_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(i2c_bus_handle, &sgp41_cfg, &sgp41_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add SGP41 device: %s", esp_err_to_name(err));
    }
    
    /* Add SCD40 device */
    i2c_device_config_t scd40_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD40_I2C_ADDR,
        .scl_speed_hz = AERIS_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(i2c_bus_handle, &scd40_cfg, &scd40_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add SCD40 device: %s", esp_err_to_name(err));
    }
    
    ESP_LOGI(TAG, "I2C master bus initialized on SDA=%d, SCL=%d", 
             AERIS_I2C_SDA_PIN, AERIS_I2C_SCL_PIN);
    
    /* Probe known I2C addresses to see which devices are present */
    ESP_LOGI(TAG, "Probing I2C devices...");
    
    const struct {
        uint8_t addr;
        const char *name;
    } devices[] = {
        {SHT45_I2C_ADDR, "SHT45"},
        {LPS22HB_I2C_ADDR, "LPS22HB"},
        {SGP41_I2C_ADDR, "SGP41"},
        {SCD40_I2C_ADDR, "SCD40"},
        {0x5C, "LPS22HB (alt)"},  // LPS22HB alternate address (SA0=LOW)
        {0x45, "SHT45 (alt)"},    // Some SHT4x variants
    };
    
    for (size_t i = 0; i < sizeof(devices)/sizeof(devices[0]); i++) {
        esp_err_t probe_ret = i2c_master_probe(i2c_bus_handle, devices[i].addr, pdMS_TO_TICKS(100));
        if (probe_ret == ESP_OK) {
            ESP_LOGI(TAG, "  [OK] %s found at 0x%02X", devices[i].name, devices[i].addr);
        } else {
            ESP_LOGD(TAG, "  [--] %s not found at 0x%02X", devices[i].name, devices[i].addr);
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Initialize air quality sensor driver
 */
esp_err_t aeris_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing Aeris Air Quality Sensor Driver");
    
    // Initialize I2C bus
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        // Continue anyway - PM sensor uses UART
    } else {
        // Initialize SHT45 temperature/humidity sensor
        ret = sht45_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize SHT45: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Continuing without temperature/humidity sensor");
        }
        
        // Initialize LPS22HB pressure sensor
        ret = lps22hb_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize LPS22HB: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Continuing without pressure sensor");
        }
        
        // Initialize SGP41 VOC/NOx sensor
        ret = sgp41_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize SGP41: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Continuing without VOC/NOx sensor");
        }
        
        // Initialize SCD40 CO2 sensor
        ret = scd40_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize SCD40: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Continuing without CO2 sensor");
        }
    }
    
    // Initialize PMSA003A UART
    ret = pmsa003a_uart_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PMSA003A UART: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create PMSA003A read task
    BaseType_t task_ret = xTaskCreate(pmsa003a_task, "pmsa003a_task", 3072, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create PMSA003A task");
        return ESP_FAIL;
    }
    
    // Create polling timer for PMSA003A (default 5 minutes)
    pmsa003a_polling_timer = xTimerCreate(
        "pmsa_poll",
        pdMS_TO_TICKS(pmsa003a_polling_interval_s * 1000),
        pdTRUE,  // Auto-reload
        NULL,
        pmsa003a_polling_timer_callback
    );
    
    if (pmsa003a_polling_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create PMSA003A polling timer");
        ESP_LOGW(TAG, "Continuing with PM sensor in continuous mode");
        // Non-fatal - sensor will work in continuous mode
    } else {
        // Start the timer
        if (xTimerStart(pmsa003a_polling_timer, 0) != pdPASS) {
            ESP_LOGE(TAG, "Failed to start PMSA003A polling timer");
            ESP_LOGW(TAG, "Continuing with PM sensor in continuous mode");
        } else {
            ESP_LOGI(TAG, "PMSA003A polling timer started with %lu second interval", 
                     pmsa003a_polling_interval_s);
        }
    }
    
    ESP_LOGI(TAG, "Aeris driver initialized successfully");
    
    return ESP_OK;
}

/**
 * @brief Get current sensor readings
 */
esp_err_t aeris_get_sensor_data(aeris_sensor_state_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(state, &current_state, sizeof(aeris_sensor_state_t));
    return ESP_OK;
}

/**
 * @brief Read temperature and humidity
 */
esp_err_t aeris_read_temp_humidity(float *temp_c, float *humidity)
{
    if (!temp_c || !humidity) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read from SHT45 sensor
    if (!sht45_initialized) {
        ESP_LOGW(TAG, "SHT45 not initialized, returning cached values");
        *temp_c = current_state.temperature_c;
        *humidity = current_state.humidity_percent;
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = sht45_read_temp_humidity(temp_c, humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SHT45: %s", esp_err_to_name(ret));
        *temp_c = current_state.temperature_c;
        *humidity = current_state.humidity_percent;
        return ret;
    }
    
    // Update current state
    current_state.temperature_c = *temp_c;
    current_state.humidity_percent = *humidity;
    
    ESP_LOGD(TAG, "Temp: %.2f°C, Humidity: %.2f%%", *temp_c, *humidity);
    return ESP_OK;
}

/**
 * @brief Read atmospheric pressure
 */
esp_err_t aeris_read_pressure(float *pressure_hpa)
{
    if (!pressure_hpa) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read from LPS22HB sensor
    if (!lps22hb_initialized) {
        ESP_LOGW(TAG, "LPS22HB not initialized");
        *pressure_hpa = current_state.pressure_hpa;
        return ESP_ERR_INVALID_STATE;
    }
    
    float temp_c;
    esp_err_t ret = lps22hb_read_data(pressure_hpa, &temp_c);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LPS22HB: %s", esp_err_to_name(ret));
        *pressure_hpa = current_state.pressure_hpa;
        return ret;
    }
    
    // Update current state
    current_state.pressure_hpa = *pressure_hpa;
    
    ESP_LOGD(TAG, "Pressure: %.2f hPa, Temp: %.2f°C", *pressure_hpa, temp_c);
    return ESP_OK;
}

/**
 * @brief Read particulate matter concentrations
 */
esp_err_t aeris_read_pm(float *pm1_0, float *pm2_5, float *pm10)
{
    if (!pm1_0 || !pm2_5 || !pm10) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read from PMSA003A data (updated by background task)
    if (!pmsa003a_data_valid) {
        ESP_LOGW(TAG, "PMSA003A data not yet available");
        *pm1_0 = 0.0f;
        *pm2_5 = 0.0f;
        *pm10 = 0.0f;
        return ESP_ERR_NOT_FOUND;
    }
    
    *pm1_0 = current_state.pm1_0_ug_m3;
    *pm2_5 = current_state.pm2_5_ug_m3;
    *pm10 = current_state.pm10_ug_m3;
    
    ESP_LOGD(TAG, "PM1.0: %.2f, PM2.5: %.2f, PM10: %.2f µg/m³", 
             *pm1_0, *pm2_5, *pm10);
    return ESP_OK;
}

/**
 * @brief Read VOC Index
 */
esp_err_t aeris_read_voc(uint16_t *voc_index)
{
    if (!voc_index) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw VOC signal from SGP41
    uint16_t voc_raw, nox_raw;
    esp_err_t ret = sgp41_measure_raw_signals(&voc_raw, &nox_raw,
                                               current_state.humidity_percent,
                                               current_state.temperature_c);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read SGP41");
        *voc_index = current_state.voc_index;
        return ret;
    }
    
    // Store raw values
    current_state.voc_raw = voc_raw;
    current_state.nox_raw = nox_raw;
    
    // TODO: Process raw signal with Sensirion Gas Index Algorithm
    // For now, return a placeholder based on raw value
    // Typical raw VOC range: 20000-60000, we map to index 1-500
    if (voc_raw > 20000) {
        *voc_index = (uint16_t)(((voc_raw - 20000) * 500) / 40000);
        if (*voc_index > 500) *voc_index = 500;
    } else {
        *voc_index = 1;
    }
    
    current_state.voc_index = *voc_index;
    
    ESP_LOGD(TAG, "VOC raw: %d, index: %d", voc_raw, *voc_index);
    return ESP_OK;
}

/**
 * @brief Read NOx Index
 */
esp_err_t aeris_read_nox(uint16_t *nox_index)
{
    if (!nox_index) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // NOx data is already read with VOC, use stored value
    uint16_t nox_raw = current_state.nox_raw;
    
    // TODO: Process raw signal with Sensirion Gas Index Algorithm
    // For now, return a placeholder based on raw value
    // Typical raw NOx range: 10000-50000, we map to index 1-500
    if (nox_raw > 10000) {
        *nox_index = (uint16_t)(((nox_raw - 10000) * 500) / 40000);
        if (*nox_index > 500) *nox_index = 500;
    } else {
        *nox_index = 1;
    }
    
    current_state.nox_index = *nox_index;
    
    ESP_LOGD(TAG, "NOx raw: %d, index: %d", nox_raw, *nox_index);
    return ESP_OK;
}

/**
 * @brief Read VOC and NOx raw signals
 */
esp_err_t aeris_read_voc_nox_raw(uint16_t *voc_raw, uint16_t *nox_raw)
{
    if (!voc_raw || !nox_raw) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = sgp41_measure_raw_signals(voc_raw, nox_raw,
                                               current_state.humidity_percent,
                                               current_state.temperature_c);
    if (ret == ESP_OK) {
        current_state.voc_raw = *voc_raw;
        current_state.nox_raw = *nox_raw;
    }
    
    return ret;
}

/**
 * @brief Read CO2 concentration
 */
esp_err_t aeris_read_co2(uint16_t *co2_ppm)
{
    if (!co2_ppm) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read from SCD40 sensor
    if (!scd40_initialized) {
        ESP_LOGW(TAG, "SCD40 not initialized, returning cached value");
        *co2_ppm = current_state.co2_ppm;
        return ESP_ERR_INVALID_STATE;
    }
    
    // Update pressure compensation if LPS22HB is available
    if (lps22hb_initialized && current_state.pressure_hpa > 0) {
        // Convert pressure to hPa (it's already in hPa from our state)
        uint16_t pressure_hpa = (uint16_t)(current_state.pressure_hpa + 0.5f);
        scd40_set_ambient_pressure(pressure_hpa);
    }
    
    float temp_c, humidity_percent;
    esp_err_t ret = scd40_read_measurement(co2_ppm, &temp_c, &humidity_percent);
    if (ret == ESP_ERR_NOT_FOUND) {
        // Data not ready yet, return cached value
        *co2_ppm = current_state.co2_ppm;
        return ESP_ERR_NOT_FOUND;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SCD40: %s", esp_err_to_name(ret));
        *co2_ppm = current_state.co2_ppm;
        return ret;
    }
    
    // Update current state
    current_state.co2_ppm = *co2_ppm;
    
    // Note: SCD40 also provides temp/humidity but we use SHT45 as primary
    ESP_LOGD(TAG, "CO2: %d ppm (SCD40 temp: %.2f°C, RH: %.2f%%)", 
             *co2_ppm, temp_c, humidity_percent);
    
    return ESP_OK;
}

/**
 * @brief Set PM sensor polling interval
 */
esp_err_t aeris_set_pm_polling_interval(uint32_t interval_s)
{
    if (interval_s != 0 && interval_s < 60) {
        ESP_LOGW(TAG, "PM polling interval %lu s too short, minimum 60s or 0 for continuous", interval_s);
        return ESP_ERR_INVALID_ARG;
    }
    
    pmsa003a_polling_interval_s = interval_s;
    
    if (interval_s == 0) {
        // Continuous mode - wake sensor and stop timer
        ESP_LOGI(TAG, "Setting PMSA003A to continuous mode");
        if (pmsa003a_polling_timer != NULL) {
            xTimerStop(pmsa003a_polling_timer, 0);
        }
        pmsa003a_wake();
    } else {
        // Polled mode - put sensor to sleep and restart timer with new interval
        ESP_LOGI(TAG, "Setting PMSA003A polling interval to %lu seconds", interval_s);
        
        if (pmsa003a_polling_timer != NULL) {
            // Stop current timer
            xTimerStop(pmsa003a_polling_timer, 0);
            
            // Change timer period
            if (xTimerChangePeriod(pmsa003a_polling_timer, 
                                   pdMS_TO_TICKS(interval_s * 1000), 
                                   pdMS_TO_TICKS(100)) != pdPASS) {
                ESP_LOGE(TAG, "Failed to change timer period");
                return ESP_FAIL;
            }
            
            // Start timer
            if (xTimerStart(pmsa003a_polling_timer, 0) != pdPASS) {
                ESP_LOGE(TAG, "Failed to start timer");
                return ESP_FAIL;
            }
        }
        
        // Put sensor to sleep immediately
        pmsa003a_sleep();
    }
    
    return ESP_OK;
}

/**
 * @brief Get PM sensor polling interval
 */
uint32_t aeris_get_pm_polling_interval(void)
{
    return pmsa003a_polling_interval_s;
}

/**
 * @brief Set temperature offset compensation
 * @param offset_c Temperature offset in degrees Celsius
 *                 Positive value means sensor reads higher than actual
 *                 (will be subtracted from raw reading)
 */
void aeris_set_temperature_offset(float offset_c)
{
    temperature_offset_c = offset_c;
    ESP_LOGI(TAG, "Temperature offset set to %.2f°C", offset_c);
}

/**
 * @brief Get current temperature offset compensation
 * @return Temperature offset in degrees Celsius
 */
float aeris_get_temperature_offset(void)
{
    return temperature_offset_c;
}
