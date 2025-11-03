/*
 * Aeris Air Quality Sensor Driver Implementation
 *
 * I2C communication with air quality sensors
 * UART communication with PMSA003A particulate matter sensor
 * Supports Temperature, Humidity, Pressure, PM, VOC Index, and CO2 sensors
 */

#include "aeris_driver.h"
#include "esp_log.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "AERIS_DRIVER";

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
    .co2_ppm = 400,
    .sensor_error = false,
    .error_text = ""
};

/* PMSA003A latest data */
static pmsa003a_data_t pmsa003a_data = {0};
static bool pmsa003a_data_valid = false;

/* LPS22HB sensor state */
static bool lps22hb_initialized = false;

/**
 * @brief Write to LPS22HB register
 */
static esp_err_t lps22hb_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    esp_err_t ret = i2c_master_write_to_device(AERIS_I2C_NUM, LPS22HB_I2C_ADDR,
                                                write_buf, sizeof(write_buf),
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
    esp_err_t ret = i2c_master_write_read_device(AERIS_I2C_NUM, LPS22HB_I2C_ADDR,
                                                  &reg, 1, data, len,
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
 * @brief PMSA003A read task
 */
static void pmsa003a_task(void *arg)
{
    uint8_t frame[PMSA003A_FRAME_LENGTH];
    uint8_t buffer[256];
    size_t buffer_len = 0;
    
    ESP_LOGI(TAG, "PMSA003A read task started");
    
    while (1) {
        // Read available data
        int len = uart_read_bytes(PMSA003A_UART_NUM, buffer + buffer_len, 
                                  sizeof(buffer) - buffer_len, pdMS_TO_TICKS(100));
        
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
            
            // Prevent buffer overflow
            if (buffer_len > sizeof(buffer) - 64) {
                ESP_LOGW(TAG, "PMSA003A buffer overflow, resetting");
                buffer_len = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Initialize I2C bus for sensors
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AERIS_I2C_SDA_PIN,
        .scl_io_num = AERIS_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AERIS_I2C_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(AERIS_I2C_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(AERIS_I2C_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C master initialized on SDA=%d, SCL=%d", 
             AERIS_I2C_SDA_PIN, AERIS_I2C_SCL_PIN);
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
        // Initialize LPS22HB pressure sensor
        ret = lps22hb_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize LPS22HB: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Continuing without pressure sensor");
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
    
    // TODO: Initialize other sensors here
    // - Temperature/Humidity sensor (e.g., SHT4x, BME280)
    // - VOC sensor (e.g., SGP40, BME680)
    // - CO2 sensor (e.g., SCD40, SCD41)
    
    ESP_LOGI(TAG, "Aeris driver initialized successfully");
    ESP_LOGW(TAG, "Note: Some I2C sensors still need implementation");
    
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
    
    // TODO: Implement actual sensor reading
    // Example for SHT4x or BME280
    
    *temp_c = current_state.temperature_c;
    *humidity = current_state.humidity_percent;
    
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
    
    // TODO: Implement actual sensor reading
    // Example for SGP40 or BME680
    
    *voc_index = current_state.voc_index;
    
    ESP_LOGD(TAG, "VOC Index: %d", *voc_index);
    return ESP_OK;
}

/**
 * @brief Read CO2 concentration
 */
esp_err_t aeris_read_co2(uint16_t *co2_ppm)
{
    if (!co2_ppm) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // TODO: Implement actual sensor reading
    // Example for SCD40 or SCD41
    
    *co2_ppm = current_state.co2_ppm;
    
    ESP_LOGD(TAG, "CO2: %d ppm", *co2_ppm);
    return ESP_OK;
}
