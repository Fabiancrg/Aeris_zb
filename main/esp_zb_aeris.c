/*
 * Zigbee Aeris Air Quality Sensor
 *
 * This code reads air quality sensors and exposes them as Zigbee sensor endpoints
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "board.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_aeris.h"
#include "aeris_driver.h"
#include "esp_zb_ota.h"
#include "esp_zigbee_trace.h"
#include "sdkconfig.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/timers.h"
#include "led_indicator.h"

#if !defined ZB_ROUTER_ROLE
#error Define ZB_ROUTER_ROLE in idf.py menuconfig to compile Router source code.
#endif

static const char *TAG = "AERIS_ZIGBEE";

/* Sensor update interval */
#define SENSOR_UPDATE_INTERVAL_MS   30000  // 30 seconds

/* Boot button configuration for factory reset */
#define BOOT_BUTTON_GPIO            GPIO_NUM_9
#define BUTTON_LONG_PRESS_TIME_MS   5000

/* Status LED blink state */
static TimerHandle_t status_led_blink_timer = NULL;
static bool status_led_blink_state = false;

/********************* Function Declarations **************************/
static esp_err_t deferred_driver_init(void);
static void sensor_update_zigbee_attributes(uint8_t param);
static void sensor_periodic_update(uint8_t param);
static esp_err_t button_init(void);
static void button_task(void *arg);
static void factory_reset_device(uint8_t param);
static void status_led_blink_callback(TimerHandle_t xTimer);

/* OTA validation functions */
void ota_validation_start(void);
void ota_validation_hw_init_ok(void);
void ota_validation_zigbee_init_ok(void);
void ota_validation_zigbee_connected(void);
void ota_validation_mark_invalid(void);

/* Factory reset function */
static void factory_reset_device(uint8_t param)
{
    ESP_LOGW(TAG, "[RESET] Performing factory reset...");
    esp_zb_factory_reset();
    ESP_LOGI(TAG, "[RESET] Factory reset successful - device will restart");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/* Status LED blink timer callback for join animation */
static void status_led_blink_callback(TimerHandle_t xTimer)
{
    // Toggle between green and orange during joining
    status_led_blink_state = !status_led_blink_state;
    led_set_status(status_led_blink_state ? LED_COLOR_GREEN : LED_COLOR_ORANGE);
}

/* Start status LED blinking (green/orange during join) */
static void status_led_start_blink(void)
{
    if (status_led_blink_timer == NULL) {
        status_led_blink_timer = xTimerCreate(
            "status_blink",
            pdMS_TO_TICKS(500),  // 500ms interval
            pdTRUE,              // Auto-reload
            NULL,
            status_led_blink_callback
        );
    }
    
    if (status_led_blink_timer != NULL) {
        status_led_blink_state = false;
        led_set_status(LED_COLOR_ORANGE);
        xTimerStart(status_led_blink_timer, 0);
        ESP_LOGI(TAG, "[STATUS_LED] Started join blink animation");
    }
}

/* Stop status LED blinking */
static void status_led_stop_blink(void)
{
    if (status_led_blink_timer != NULL) {
        xTimerStop(status_led_blink_timer, 0);
        ESP_LOGI(TAG, "[STATUS_LED] Stopped blink animation");
    }
}

/* Boot button queue and ISR handler */
static QueueHandle_t button_evt_queue = NULL;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    uint32_t gpio_num = BOOT_BUTTON_GPIO;
    xQueueSendFromISR(button_evt_queue, &gpio_num, NULL);
}

/* Button monitoring task */
static void button_task(void *arg)
{
    uint32_t io_num;
    TickType_t press_start_time = 0;
    bool long_press_triggered = false;
    const TickType_t LONG_PRESS_DURATION = pdMS_TO_TICKS(BUTTON_LONG_PRESS_TIME_MS);
    
    ESP_LOGI(TAG, "[BUTTON] Task started - waiting for button events");
    
    for (;;) {
        if (xQueueReceive(button_evt_queue, &io_num, portMAX_DELAY)) {
            gpio_intr_disable(BOOT_BUTTON_GPIO);
            int button_level = gpio_get_level(BOOT_BUTTON_GPIO);
            
            if (button_level == 0) {  // Button pressed
                press_start_time = xTaskGetTickCount();
                long_press_triggered = false;
                ESP_LOGI(TAG, "[BUTTON] Pressed - hold 5 sec for factory reset");
                
                while (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
                    TickType_t current_time = xTaskGetTickCount();
                    if ((current_time - press_start_time) >= LONG_PRESS_DURATION && !long_press_triggered) {
                        long_press_triggered = true;
                        ESP_LOGW(TAG, "[BUTTON] Long press detected! Triggering factory reset...");
                        esp_zb_scheduler_alarm((esp_zb_callback_t)factory_reset_device, 0, 100);
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                
                if (!long_press_triggered) {
                    uint32_t press_duration = pdTICKS_TO_MS(xTaskGetTickCount() - press_start_time);
                    ESP_LOGI(TAG, "[BUTTON] Released (held for %lu ms)", press_duration);
                }
            }
            
            gpio_intr_enable(BOOT_BUTTON_GPIO);
        }
    }
}

/* Initialize boot button */
static esp_err_t button_init(void)
{
    ESP_LOGI(TAG, "[INIT] Initializing boot button on GPIO%d", BOOT_BUTTON_GPIO);
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!button_evt_queue) {
        ESP_LOGE(TAG, "[ERROR] Failed to create event queue");
        return ESP_FAIL;
    }
    
    esp_err_t isr_ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "[ERROR] Failed to install ISR service: %s", esp_err_to_name(isr_ret));
        return isr_ret;
    }
    
    ret = gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to add ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    BaseType_t task_ret = xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "[ERROR] Failed to create button task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "[OK] Boot button initialization complete");
    return ESP_OK;
}

static esp_err_t deferred_driver_init(void)
{
    ESP_LOGI(TAG, "[INIT] Starting deferred driver initialization...");
    
    /* Initialize boot button for factory reset */
    esp_err_t button_ret = button_init();
    if (button_ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Button initialization failed");
        return button_ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Initialize RGB LED indicator */
    ESP_LOGI(TAG, "[INIT] Initializing RGB LED indicator...");
    esp_err_t led_ret = led_indicator_init();
    if (led_ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] LED initialization failed: %s", esp_err_to_name(led_ret));
        ESP_LOGW(TAG, "[WARN] Continuing without LED indicator");
    } else {
        ESP_LOGI(TAG, "[OK] LED indicator initialized");
    }
    
    /* Initialize air quality sensor driver */
    ESP_LOGI(TAG, "[INIT] Initializing air quality sensors...");
    esp_err_t ret = aeris_driver_init();
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to initialize sensors: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "[WARN] Continuing without sensors - endpoints will still be created");
    } else {
        ESP_LOGI(TAG, "[OK] Air quality sensors initialized successfully");
    }
    
    ESP_LOGI(TAG, "[INIT] Deferred initialization complete");
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , 
                       TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "[JOIN] Initialize Zigbee stack");
        led_set_status(LED_COLOR_ORANGE);  // Not joined yet
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        ESP_LOGI(TAG, "[JOIN] Device first start - factory new device");
        if (err_status == ESP_OK) {
            deferred_driver_init();
            status_led_start_blink();  // Start blinking during join
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            led_set_status(LED_COLOR_RED);  // Error
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "[JOIN] Device reboot - previously joined network");
        if (err_status == ESP_OK) {
            deferred_driver_init();
            if (esp_zb_bdb_is_factory_new()) {
                status_led_start_blink();  // Start blinking during join
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                led_set_status(LED_COLOR_GREEN);  // Previously joined, should reconnect
            }
        } else {
            led_set_status(LED_COLOR_RED);  // Error
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "[JOIN] *** SUCCESSFULLY JOINED NETWORK ***");
            ESP_LOGI(TAG, "[JOIN] PAN ID: 0x%04hx, Channel: %d", 
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            
            status_led_stop_blink();  // Stop blinking
            led_set_status(LED_COLOR_GREEN);  // Connected!
            ota_validation_zigbee_connected();
            
            /* Start periodic sensor updates */
            esp_zb_scheduler_alarm((esp_zb_callback_t)sensor_periodic_update, 0, 5000);
            ESP_LOGI(TAG, "[JOIN] Setup complete!");
        } else {
            ESP_LOGW(TAG, "[JOIN] Network steering failed, retrying...");
            status_led_stop_blink();  // Stop blinking
            led_set_status(LED_COLOR_RED);  // Join failed - show error
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                  ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
        
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "[LEAVE] Device left the network");
        status_led_stop_blink();  // Stop any blinking
        led_set_status(LED_COLOR_ORANGE);  // Left network
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                              ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        break;
        
    default:
        ESP_LOGI(TAG, "[ZDO] Signal: %s (0x%x), status: %s", 
                esp_zb_zdo_signal_to_string(sig_type), sig_type,
                esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, 
                       TAG, "Received message: error status(%d)", message->info.status);
    
    ESP_LOGI(TAG, "RX: endpoint(%d), cluster(0x%x), attr(0x%x)", 
             message->info.dst_endpoint, message->info.cluster, message->attribute.id);
    
    /* Handle LED configuration endpoint */
    if (message->info.dst_endpoint == HA_ESP_LED_CONFIG_ENDPOINT) {
        led_thresholds_t thresholds;
        led_get_thresholds(&thresholds);
        
        /* Handle On/Off cluster for LED enable/disable */
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
            message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
            bool on_off = *(bool *)message->attribute.data.value;
            ESP_LOGI(TAG, "LED %s", on_off ? "enabled" : "disabled");
            led_set_enable(on_off);
            thresholds.enabled = on_off;
            led_set_thresholds(&thresholds);
        }
        /* Handle threshold attributes */
        else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT) {
            uint16_t value = *(uint16_t *)message->attribute.data.value;
            bool updated = true;
            
            switch (message->attribute.id) {
                case ZCL_LED_ATTR_VOC_ORANGE:
                    thresholds.voc_orange = value;
                    ESP_LOGI(TAG, "VOC orange threshold: %d", value);
                    break;
                case ZCL_LED_ATTR_VOC_RED:
                    thresholds.voc_red = value;
                    ESP_LOGI(TAG, "VOC red threshold: %d", value);
                    break;
                case ZCL_LED_ATTR_NOX_ORANGE:
                    thresholds.nox_orange = value;
                    ESP_LOGI(TAG, "NOx orange threshold: %d", value);
                    break;
                case ZCL_LED_ATTR_NOX_RED:
                    thresholds.nox_red = value;
                    ESP_LOGI(TAG, "NOx red threshold: %d", value);
                    break;
                case ZCL_LED_ATTR_CO2_ORANGE:
                    thresholds.co2_orange = value;
                    ESP_LOGI(TAG, "CO2 orange threshold: %d ppm", value);
                    break;
                case ZCL_LED_ATTR_CO2_RED:
                    thresholds.co2_red = value;
                    ESP_LOGI(TAG, "CO2 red threshold: %d ppm", value);
                    break;
                case ZCL_LED_ATTR_HUM_ORANGE_LOW:
                    thresholds.humidity_orange_low = value;
                    ESP_LOGI(TAG, "Humidity orange low threshold: %d%%", value);
                    break;
                case ZCL_LED_ATTR_HUM_ORANGE_HIGH:
                    thresholds.humidity_orange_high = value;
                    ESP_LOGI(TAG, "Humidity orange high threshold: %d%%", value);
                    break;
                case ZCL_LED_ATTR_HUM_RED_LOW:
                    thresholds.humidity_red_low = value;
                    ESP_LOGI(TAG, "Humidity red low threshold: %d%%", value);
                    break;
                case ZCL_LED_ATTR_HUM_RED_HIGH:
                    thresholds.humidity_red_high = value;
                    ESP_LOGI(TAG, "Humidity red high threshold: %d%%", value);
                    break;
                case ZCL_LED_ATTR_PM25_ORANGE:
                    thresholds.pm25_orange = value;
                    ESP_LOGI(TAG, "PM2.5 orange threshold: %d µg/m³", value);
                    break;
                case ZCL_LED_ATTR_PM25_RED:
                    thresholds.pm25_red = value;
                    ESP_LOGI(TAG, "PM2.5 red threshold: %d µg/m³", value);
                    break;
                case ZCL_LED_ATTR_ENABLE_MASK:
                    thresholds.led_mask = (uint8_t)value;
                    ESP_LOGI(TAG, "LED enable mask: 0x%02X (CO2:%d VOC:%d NOx:%d PM2.5:%d Hum:%d)", 
                             (uint8_t)value,
                             !!(value & (1<<0)), !!(value & (1<<1)), !!(value & (1<<2)),
                             !!(value & (1<<3)), !!(value & (1<<4)));
                    break;
                case ZCL_LED_ATTR_PM_POLL_INTERVAL:
                    {
                        uint32_t interval = *(uint32_t *)message->attribute.data.value;
                        ESP_LOGI(TAG, "PM sensor polling interval: %lu seconds %s", 
                                 interval, interval == 0 ? "(continuous)" : "");
                        esp_err_t err = aeris_set_pm_polling_interval(interval);
                        if (err != ESP_OK) {
                            ESP_LOGW(TAG, "Failed to set PM polling interval: %s", 
                                     esp_err_to_name(err));
                        }
                        updated = false;  // Don't update thresholds
                    }
                    break;
                default:
                    updated = false;
                    break;
            }
            
            if (updated) {
                led_set_thresholds(&thresholds);
            }
        }
    }
    
    /* Handle Status LED endpoint */
    if (message->info.dst_endpoint == HA_ESP_STATUS_LED_ENDPOINT) {
        /* Handle On/Off cluster for status LED enable/disable */
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
            message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
            bool on_off = *(bool *)message->attribute.data.value;
            ESP_LOGI(TAG, "Status LED %s", on_off ? "enabled" : "disabled");
            led_set_status_enable(on_off);
        }
    }
    
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void sensor_update_zigbee_attributes(uint8_t param)
{
    aeris_sensor_state_t state;
    esp_err_t ret = aeris_get_sensor_data(&state);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get sensor data");
        return;
    }
    
    ESP_LOGI(TAG, "Updating Zigbee attributes:");
    ESP_LOGI(TAG, "  Temp: %.2f°C, Humidity: %.2f%%", state.temperature_c, state.humidity_percent);
    ESP_LOGI(TAG, "  Pressure: %.2fhPa", state.pressure_hpa);
    ESP_LOGI(TAG, "  PM1.0: %.2f, PM2.5: %.2f, PM10: %.2f µg/m³", 
             state.pm1_0_ug_m3, state.pm2_5_ug_m3, state.pm10_ug_m3);
    ESP_LOGI(TAG, "  VOC Index: %d, NOx Index: %d, CO2: %d ppm", state.voc_index, state.nox_index, state.co2_ppm);
    
    /* Update Endpoint 1: Temperature and Humidity */
    int16_t temp_zigbee = (int16_t)(state.temperature_c * 100);  // °C to 0.01°C
    uint16_t hum_zigbee = (uint16_t)(state.humidity_percent * 100);  // % to 0.01%
    
    esp_zb_zcl_set_attribute_val(HA_ESP_TEMP_HUM_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                  &temp_zigbee, false);
    esp_zb_zcl_set_attribute_val(HA_ESP_TEMP_HUM_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                                  &hum_zigbee, false);
    
    /* Update Endpoint 2: Pressure */
    int16_t pressure_zigbee = (int16_t)(state.pressure_hpa * 10);  // hPa to 0.1 hPa
    esp_zb_zcl_set_attribute_val(HA_ESP_PRESSURE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
                                  &pressure_zigbee, false);
    
    /* Update Endpoint 3: PM1.0 */
    float pm1_value = state.pm1_0_ug_m3;
    esp_zb_zcl_set_attribute_val(HA_ESP_PM1_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                                  &pm1_value, false);
    
    /* Update Endpoint 4: PM2.5 */
    float pm25_value = state.pm2_5_ug_m3;
    esp_zb_zcl_set_attribute_val(HA_ESP_PM25_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                                  &pm25_value, false);
    
    /* Update Endpoint 5: PM10 */
    float pm10_value = state.pm10_ug_m3;
    esp_zb_zcl_set_attribute_val(HA_ESP_PM10_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                                  &pm10_value, false);
    
    /* Update Endpoint 6: VOC Index */
    float voc_value = (float)state.voc_index;
    esp_zb_zcl_set_attribute_val(HA_ESP_VOC_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                                  &voc_value, false);
    
    /* Update Endpoint 7: NOx Index */
    float nox_value = (float)state.nox_index;
    esp_zb_zcl_set_attribute_val(HA_ESP_NOX_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                                  &nox_value, false);
    
    /* Update Endpoint 8: CO2 */
    float co2_value = (float)state.co2_ppm;
    esp_zb_zcl_set_attribute_val(HA_ESP_CO2_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
                                  &co2_value, false);
    
    /* Update LED based on sensor readings */
    led_sensor_data_t led_data = {
        .voc_index = state.voc_index,
        .nox_index = state.nox_index,
        .co2_ppm = state.co2_ppm,
        .humidity_percent = state.humidity_percent,
        .pm25_ug_m3 = state.pm2_5_ug_m3,
    };
    led_update_from_sensors(&led_data);
}

static void sensor_periodic_update(uint8_t param)
{
    sensor_update_zigbee_attributes(0);
    
    /* Schedule next update */
    esp_zb_scheduler_alarm((esp_zb_callback_t)sensor_periodic_update, 0, SENSOR_UPDATE_INTERVAL_MS);
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack as Router (matching working example) */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ROUTER_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    
    /* Create endpoint list */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    
    /* Endpoint 1: Temperature and Humidity with REPORTING flag */
    esp_zb_cluster_list_t *temp_hum_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Add Basic cluster */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(temp_hum_clusters, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* Add Temperature measurement cluster with REPORTING flag */
    static int16_t temp_value = 0x8000;  // Invalid value (0x8000)
    static int16_t temp_min = 0x954D;    // -40°C in 0.01°C units
    static int16_t temp_max = 0x7FFF;    // Max valid value
    esp_zb_attribute_list_t *temp_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &temp_value));
    ESP_ERROR_CHECK(esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &temp_min));
    ESP_ERROR_CHECK(esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &temp_max));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(temp_hum_clusters, temp_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* Add Humidity measurement cluster with REPORTING flag */
    static uint16_t hum_value = 0xFFFF;  // Invalid value
    static uint16_t hum_min = 0;         // 0% RH
    static uint16_t hum_max = 10000;     // 100% RH (in 0.01% units)
    esp_zb_attribute_list_t *humidity_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(humidity_cluster, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &hum_value));
    ESP_ERROR_CHECK(esp_zb_humidity_meas_cluster_add_attr(humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &hum_min));
    ESP_ERROR_CHECK(esp_zb_humidity_meas_cluster_add_attr(humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &hum_max));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(temp_hum_clusters, humidity_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* Add Identify cluster */
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(temp_hum_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint1_config = {
        .endpoint = HA_ESP_TEMP_HUM_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, temp_hum_clusters, endpoint1_config);
    
    /* Endpoint 2: Pressure with REPORTING flag */
    esp_zb_cluster_list_t *pressure_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Create Pressure measurement cluster with REPORTING flag */
    static int16_t pressure_value = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_UNKNOWN;
    static int16_t pressure_min = 300 * 10;   // 300 hPa in 0.1 kPa units
    static int16_t pressure_max = 1100 * 10;  // 1100 hPa in 0.1 kPa units
    esp_zb_attribute_list_t *pressure_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(pressure_cluster, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
                                            ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &pressure_value));
    ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID, &pressure_min));
    ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID, &pressure_max));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_pressure_meas_cluster(pressure_clusters, pressure_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(pressure_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint2_config = {
        .endpoint = HA_ESP_PRESSURE_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, pressure_clusters, endpoint2_config);
    
    /* Endpoint 3: PM1.0 - using Analog Input cluster with REPORTING flag */
    esp_zb_cluster_list_t *pm1_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Create Analog Input cluster with REPORTING flag */
    static float pm1_present_value = 0.0f;
    esp_zb_attribute_list_t *pm1_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(pm1_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &pm1_present_value));
    char pm1_desc[] = "\x0B""PM1.0 µg/m³";
    esp_zb_analog_input_cluster_add_attr(pm1_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, pm1_desc);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(pm1_clusters, pm1_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(pm1_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint3_config = {
        .endpoint = HA_ESP_PM1_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, pm1_clusters, endpoint3_config);
    
    /* Endpoint 4: PM2.5 - using Analog Input cluster with REPORTING flag */
    esp_zb_cluster_list_t *pm25_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Create Analog Input cluster with REPORTING flag */
    static float pm25_present_value = 0.0f;
    esp_zb_attribute_list_t *pm25_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(pm25_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &pm25_present_value));
    char pm25_desc[] = "\x0C""PM2.5 µg/m³";
    esp_zb_analog_input_cluster_add_attr(pm25_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, pm25_desc);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(pm25_clusters, pm25_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(pm25_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint4_config = {
        .endpoint = HA_ESP_PM25_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, pm25_clusters, endpoint4_config);
    
    /* Endpoint 5: PM10 - using Analog Input cluster with REPORTING flag */
    esp_zb_cluster_list_t *pm10_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Create Analog Input cluster with REPORTING flag */
    static float pm10_present_value = 0.0f;
    esp_zb_attribute_list_t *pm10_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(pm10_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &pm10_present_value));
    char pm10_desc[] = "\x0B""PM10 µg/m³";
    esp_zb_analog_input_cluster_add_attr(pm10_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, pm10_desc);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(pm10_clusters, pm10_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(pm10_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint5_config = {
        .endpoint = HA_ESP_PM10_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, pm10_clusters, endpoint5_config);
    
    /* Endpoint 6: VOC Index - using Analog Input cluster with REPORTING flag */
    esp_zb_cluster_list_t *voc_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Create Analog Input cluster with REPORTING flag */
    static float voc_present_value = 0.0f;
    esp_zb_attribute_list_t *voc_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(voc_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &voc_present_value));
    char voc_desc[] = "\x09""VOC Index";
    esp_zb_analog_input_cluster_add_attr(voc_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, voc_desc);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(voc_clusters, voc_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(voc_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint6_config = {
        .endpoint = HA_ESP_VOC_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, voc_clusters, endpoint6_config);
    
    /* Endpoint 7: NOx Index - using Analog Input cluster with REPORTING flag */
    esp_zb_cluster_list_t *nox_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Create Analog Input cluster with REPORTING flag */
    static float nox_present_value = 0.0f;
    esp_zb_attribute_list_t *nox_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(nox_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &nox_present_value));
    char nox_desc[] = "\x09""NOx Index";
    esp_zb_analog_input_cluster_add_attr(nox_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, nox_desc);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(nox_clusters, nox_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(nox_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint7_config = {
        .endpoint = HA_ESP_NOX_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, nox_clusters, endpoint7_config);
    
    /* Endpoint 8: CO2 - using Carbon Dioxide Measurement cluster with REPORTING flag */
    esp_zb_cluster_list_t *co2_clusters = esp_zb_zcl_cluster_list_create();
    
    /* CO2 measurement cluster with REPORTING flag */
    static float co2_measured_value = 0;
    static float co2_min_measured_value = 400;   // 400 ppm
    static float co2_max_measured_value = 5000;  // 5000 ppm
    esp_zb_attribute_list_t *co2_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(co2_cluster, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                                            ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &co2_measured_value));
    ESP_ERROR_CHECK(esp_zb_carbon_dioxide_measurement_cluster_add_attr(co2_cluster, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MIN_MEASURED_VALUE_ID, &co2_min_measured_value));
    ESP_ERROR_CHECK(esp_zb_carbon_dioxide_measurement_cluster_add_attr(co2_cluster, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MAX_MEASURED_VALUE_ID, &co2_max_measured_value));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(co2_clusters, co2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(co2_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint8_config = {
        .endpoint = HA_ESP_CO2_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, co2_clusters, endpoint8_config);
    
    /* Endpoint 9: LED Configuration */
    esp_zb_cluster_list_t *led_clusters = esp_zb_zcl_cluster_list_create();
    
    /* On/Off cluster for LED enable/disable */
    esp_zb_on_off_cluster_cfg_t led_on_off_cfg = {
        .on_off = true,  // LED enabled by default
    };
    esp_zb_attribute_list_t *led_on_off_cluster = esp_zb_on_off_cluster_create(&led_on_off_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(led_clusters, led_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* Add custom attributes for thresholds using Analog Output cluster as a container */
    esp_zb_analog_output_cluster_cfg_t led_config_cfg = {
        .present_value = 0.0f,  // Unused, just a placeholder
    };
    esp_zb_attribute_list_t *led_config_cluster = esp_zb_analog_output_cluster_create(&led_config_cfg);
    
    /* Add threshold attributes (all uint16_t) */
    uint16_t voc_orange_default = 150;
    uint16_t voc_red_default = 250;
    uint16_t nox_orange_default = 150;
    uint16_t nox_red_default = 250;
    uint16_t co2_orange_default = 1000;
    uint16_t co2_red_default = 1500;
    uint16_t hum_orange_low_default = 30;
    uint16_t hum_orange_high_default = 70;
    uint16_t hum_red_low_default = 20;
    uint16_t hum_red_high_default = 80;
    uint16_t pm25_orange_default = 25;
    uint16_t pm25_red_default = 55;
    uint8_t led_mask_default = 0x1F;  // All 5 LEDs enabled by default (bits 0-4)
    uint32_t pm_poll_interval_default = 300;  // 5 minutes default polling interval
    
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_VOC_ORANGE, &voc_orange_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_VOC_RED, &voc_red_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_NOX_ORANGE, &nox_orange_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_NOX_RED, &nox_red_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_CO2_ORANGE, &co2_orange_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_CO2_RED, &co2_red_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_HUM_ORANGE_LOW, &hum_orange_low_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_HUM_ORANGE_HIGH, &hum_orange_high_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_HUM_RED_LOW, &hum_red_low_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_HUM_RED_HIGH, &hum_red_high_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_PM25_ORANGE, &pm25_orange_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_PM25_RED, &pm25_red_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_ENABLE_MASK, &led_mask_default);
    esp_zb_analog_output_cluster_add_attr(led_config_cluster, ZCL_LED_ATTR_PM_POLL_INTERVAL, &pm_poll_interval_default);
    
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(led_clusters, led_config_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(led_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint9_config = {
        .endpoint = HA_ESP_LED_CONFIG_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, led_clusters, endpoint9_config);
    
    /* Endpoint 10: Zigbee Status LED */
    esp_zb_cluster_list_t *status_led_clusters = esp_zb_zcl_cluster_list_create();
    
    /* On/Off cluster for status LED enable/disable */
    esp_zb_on_off_cluster_cfg_t status_led_on_off_cfg = {
        .on_off = true,  // Status LED enabled by default
    };
    esp_zb_attribute_list_t *status_led_on_off_cluster = esp_zb_on_off_cluster_create(&status_led_on_off_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(status_led_clusters, status_led_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(status_led_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint10_config = {
        .endpoint = HA_ESP_STATUS_LED_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, status_led_clusters, endpoint10_config);
    
    /* Register the device */
    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    
    /* OTA validation */
    ota_validation_start();
    ota_validation_hw_init_ok();
    ota_validation_zigbee_init_ok();
    
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
