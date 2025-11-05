/*
 * RGB LED Indicator Driver Implementation (SK6812)
 * Controls 5 separate LEDs for CO2, VOC, NOx, PM2.5, and Humidity monitoring
 */

#include "led_indicator.h"
#include "board.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static const char *TAG = "LED_INDICATOR";

/* SK6812 timing parameters (in nanoseconds for RMT) */
#define SK6812_T0H_NS   300     // 0 bit high time
#define SK6812_T0L_NS   900     // 0 bit low time
#define SK6812_T1H_NS   600     // 1 bit high time
#define SK6812_T1L_NS   600     // 1 bit low time
#define SK6812_RESET_US 80      // Reset time (>80µs)

/* RMT resolution */
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1µs

/* GPIO pin mapping for each LED */
static const gpio_num_t LED_GPIO_MAP[LED_ID_MAX] = {
    [LED_ID_CO2] = LED_CO2_GPIO,
    [LED_ID_VOC] = LED_VOC_GPIO,
    [LED_ID_NOX] = LED_NOX_GPIO,
    [LED_ID_PM25] = LED_PM25_GPIO,
    [LED_ID_HUMIDITY] = LED_HUM_GPIO,
};

/* LED names for logging */
static const char* LED_NAMES[LED_ID_MAX] = {
    [LED_ID_CO2] = "CO2",
    [LED_ID_VOC] = "VOC",
    [LED_ID_NOX] = "NOx",
    [LED_ID_PM25] = "PM2.5",
    [LED_ID_HUMIDITY] = "Humidity",
};

/* Default thresholds */
static led_thresholds_t s_thresholds = {
    .enabled = true,
    .voc_orange = 150,
    .voc_red = 250,
    .nox_orange = 150,
    .nox_red = 250,
    .co2_orange = 1000,
    .co2_red = 1500,
    .humidity_orange_low = 30,
    .humidity_orange_high = 70,
    .humidity_red_low = 20,
    .humidity_red_high = 80,
    .pm25_orange = 25,
    .pm25_red = 55,
};

/* RMT channel handles - one per LED */
static rmt_channel_handle_t s_led_channels[LED_ID_MAX] = {NULL};
static rmt_encoder_handle_t s_led_encoder = NULL;

/* LED state tracking */
static led_color_t s_current_colors[LED_ID_MAX] = {LED_COLOR_OFF, LED_COLOR_OFF, LED_COLOR_OFF, LED_COLOR_OFF, LED_COLOR_OFF};

/* RGB color values (GRB order for SK6812) */
typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} rgb_t;

static const rgb_t COLOR_MAP[] = {
    [LED_COLOR_OFF]    = {0,   0,   0},
    [LED_COLOR_GREEN]  = {255, 0,   0},    // Bright green
    [LED_COLOR_ORANGE] = {128, 255, 0},    // Orange (mix of red and green)
    [LED_COLOR_RED]    = {0,   255, 0},    // Bright red
};

/* RMT encoder for SK6812 */
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                    const void *primary_data, size_t data_size,
                                    rmt_encode_state_t *ret_state)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    
    switch (led_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t *led_encoder = NULL;
    
    led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
    ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for led strip encoder");
    
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    
    // SK6812 uses MSB first encoding
    // Calculate timing: duration = (time_ns * resolution_hz) / 1,000,000,000
    // At 10MHz: 1 tick = 100ns, so T0H=300ns=3 ticks, T0L=900ns=9 ticks, etc.
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = (SK6812_T0H_NS * (uint64_t)RMT_LED_STRIP_RESOLUTION_HZ) / 1000000000ULL,
            .level1 = 0,
            .duration1 = (SK6812_T0L_NS * (uint64_t)RMT_LED_STRIP_RESOLUTION_HZ) / 1000000000ULL,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = (SK6812_T1H_NS * (uint64_t)RMT_LED_STRIP_RESOLUTION_HZ) / 1000000000ULL,
            .level1 = 0,
            .duration1 = (SK6812_T1L_NS * (uint64_t)RMT_LED_STRIP_RESOLUTION_HZ) / 1000000000ULL,
        },
        .flags.msb_first = 1,
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");
    
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder), err, TAG, "create copy encoder failed");
    
    uint32_t reset_ticks = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * SK6812_RESET_US;
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };
    
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
    
err:
    if (led_encoder) {
        if (led_encoder->bytes_encoder) {
            rmt_del_encoder(led_encoder->bytes_encoder);
        }
        if (led_encoder->copy_encoder) {
            rmt_del_encoder(led_encoder->copy_encoder);
        }
        free(led_encoder);
    }
    return ret;
}

esp_err_t led_indicator_init(void)
{
    ESP_LOGI(TAG, "Initializing 4 RGB LEDs (CO2, VOC, PM2.5, Humidity)");
    
    // Create shared LED strip encoder (can be reused for all channels)
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&s_led_encoder));
    
    // Initialize each LED channel
    for (int i = 0; i < LED_ID_MAX; i++) {
        ESP_LOGI(TAG, "Initializing %s LED on GPIO%d", LED_NAMES[i], LED_GPIO_MAP[i]);
        
        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = LED_GPIO_MAP[i],
            .mem_block_symbols = 64,
            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
            .trans_queue_depth = 4,
        };
        
        esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &s_led_channels[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create RMT channel for %s LED: %s", 
                     LED_NAMES[i], esp_err_to_name(ret));
            return ret;
        }
        
        // Enable RMT channel
        ret = rmt_enable(s_led_channels[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable RMT channel for %s LED: %s", 
                     LED_NAMES[i], esp_err_to_name(ret));
            return ret;
        }
        
        // Initialize to OFF
        led_set_color(i, LED_COLOR_OFF);
    }
    
    ESP_LOGI(TAG, "All 4 RGB LEDs initialized successfully");
    return ESP_OK;
}

esp_err_t led_set_thresholds(const led_thresholds_t *thresholds)
{
    if (!thresholds) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&s_thresholds, thresholds, sizeof(led_thresholds_t));
    ESP_LOGI(TAG, "LED thresholds updated");
    return ESP_OK;
}

esp_err_t led_get_thresholds(led_thresholds_t *thresholds)
{
    if (!thresholds) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(thresholds, &s_thresholds, sizeof(led_thresholds_t));
    return ESP_OK;
}

esp_err_t led_set_color(led_id_t led_id, led_color_t color)
{
    if (led_id >= LED_ID_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (color > LED_COLOR_RED) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_led_channels[led_id]) {
        ESP_LOGW(TAG, "%s LED not initialized", LED_NAMES[led_id]);
        return ESP_ERR_INVALID_STATE;
    }
    
    // If disabled, always turn off
    if (!s_thresholds.enabled && color != LED_COLOR_OFF) {
        color = LED_COLOR_OFF;
    }
    
    rgb_t rgb = COLOR_MAP[color];
    uint8_t led_data[3] = {rgb.g, rgb.r, rgb.b};
    
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    
    esp_err_t ret = rmt_transmit(s_led_channels[led_id], s_led_encoder, led_data, sizeof(led_data), &tx_config);
    if (ret == ESP_OK) {
        s_current_colors[led_id] = color;
        rmt_tx_wait_all_done(s_led_channels[led_id], portMAX_DELAY);
    }
    
    return ret;
}

esp_err_t led_set_enable(bool enable)
{
    s_thresholds.enabled = enable;
    
    if (!enable) {
        // Turn off all LEDs when disabled
        for (int i = 0; i < LED_ID_MAX; i++) {
            led_set_color(i, LED_COLOR_OFF);
        }
    }
    
    ESP_LOGI(TAG, "All LEDs %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

bool led_is_enabled(void)
{
    return s_thresholds.enabled;
}

static led_color_t evaluate_voc(uint16_t voc_index)
{
    if (voc_index >= s_thresholds.voc_red) {
        return LED_COLOR_RED;
    } else if (voc_index >= s_thresholds.voc_orange) {
        return LED_COLOR_ORANGE;
    } else {
        return LED_COLOR_GREEN;
    }
}

static led_color_t evaluate_nox(uint16_t nox_index)
{
    if (nox_index >= s_thresholds.nox_red) {
        return LED_COLOR_RED;
    } else if (nox_index >= s_thresholds.nox_orange) {
        return LED_COLOR_ORANGE;
    } else {
        return LED_COLOR_GREEN;
    }
}

static led_color_t evaluate_co2(uint16_t co2_ppm)
{
    if (co2_ppm >= s_thresholds.co2_red) {
        return LED_COLOR_RED;
    } else if (co2_ppm >= s_thresholds.co2_orange) {
        return LED_COLOR_ORANGE;
    } else {
        return LED_COLOR_GREEN;
    }
}

static led_color_t evaluate_humidity(float humidity_percent)
{
    if (humidity_percent <= s_thresholds.humidity_red_low || 
        humidity_percent >= s_thresholds.humidity_red_high) {
        return LED_COLOR_RED;
    } else if (humidity_percent <= s_thresholds.humidity_orange_low || 
               humidity_percent >= s_thresholds.humidity_orange_high) {
        return LED_COLOR_ORANGE;
    } else {
        return LED_COLOR_GREEN;
    }
}

static led_color_t evaluate_pm25(float pm25_ug_m3)
{
    if (pm25_ug_m3 >= s_thresholds.pm25_red) {
        return LED_COLOR_RED;
    } else if (pm25_ug_m3 >= s_thresholds.pm25_orange) {
        return LED_COLOR_ORANGE;
    } else {
        return LED_COLOR_GREEN;
    }
}

esp_err_t led_update_from_sensors(const led_sensor_data_t *sensor_data)
{
    if (!sensor_data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_thresholds.enabled) {
        // Turn off all LEDs if disabled
        for (int i = 0; i < LED_ID_MAX; i++) {
            if (s_current_colors[i] != LED_COLOR_OFF) {
                led_set_color(i, LED_COLOR_OFF);
            }
        }
        return ESP_OK;
    }
    
    // Evaluate each sensor independently and update its LED
    led_color_t voc_color = evaluate_voc(sensor_data->voc_index);
    led_color_t nox_color = evaluate_nox(sensor_data->nox_index);
    led_color_t co2_color = evaluate_co2(sensor_data->co2_ppm);
    led_color_t humidity_color = evaluate_humidity(sensor_data->humidity_percent);
    led_color_t pm25_color = evaluate_pm25(sensor_data->pm25_ug_m3);
    
    // Update VOC LED
    if (voc_color != s_current_colors[LED_ID_VOC]) {
        ESP_LOGI(TAG, "VOC LED: %s (index: %d)", 
                 voc_color == LED_COLOR_GREEN ? "GREEN" : 
                 voc_color == LED_COLOR_ORANGE ? "ORANGE" : "RED",
                 sensor_data->voc_index);
        led_set_color(LED_ID_VOC, voc_color);
    }
    
    // Update NOx LED
    if (nox_color != s_current_colors[LED_ID_NOX]) {
        ESP_LOGI(TAG, "NOx LED: %s (index: %d)", 
                 nox_color == LED_COLOR_GREEN ? "GREEN" : 
                 nox_color == LED_COLOR_ORANGE ? "ORANGE" : "RED",
                 sensor_data->nox_index);
        led_set_color(LED_ID_NOX, nox_color);
    }
    
    // Update CO2 LED
    if (co2_color != s_current_colors[LED_ID_CO2]) {
        ESP_LOGI(TAG, "CO2 LED: %s (CO2: %d ppm)", 
                 co2_color == LED_COLOR_GREEN ? "GREEN" : 
                 co2_color == LED_COLOR_ORANGE ? "ORANGE" : "RED",
                 sensor_data->co2_ppm);
        led_set_color(LED_ID_CO2, co2_color);
    }
    
    // Update Humidity LED
    if (humidity_color != s_current_colors[LED_ID_HUMIDITY]) {
        ESP_LOGI(TAG, "Humidity LED: %s (%.1f%%)", 
                 humidity_color == LED_COLOR_GREEN ? "GREEN" : 
                 humidity_color == LED_COLOR_ORANGE ? "ORANGE" : "RED",
                 sensor_data->humidity_percent);
        led_set_color(LED_ID_HUMIDITY, humidity_color);
    }
    
    // Update PM2.5 LED
    if (pm25_color != s_current_colors[LED_ID_PM25]) {
        ESP_LOGI(TAG, "PM2.5 LED: %s (%.1f µg/m³)", 
                 pm25_color == LED_COLOR_GREEN ? "GREEN" : 
                 pm25_color == LED_COLOR_ORANGE ? "ORANGE" : "RED",
                 sensor_data->pm25_ug_m3);
        led_set_color(LED_ID_PM25, pm25_color);
    }
    
    return ESP_OK;
}
