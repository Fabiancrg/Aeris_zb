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

#if !defined ZB_ROUTER_ROLE
#error Define ZB_ROUTER_ROLE in idf.py menuconfig to compile Router source code.
#endif

static const char *TAG = "AERIS_ZIGBEE";

/* Sensor update interval */
#define SENSOR_UPDATE_INTERVAL_MS   30000  // 30 seconds

/* Boot button configuration for factory reset */
#define BOOT_BUTTON_GPIO            GPIO_NUM_9
#define BUTTON_LONG_PRESS_TIME_MS   5000

/********************* Function Declarations **************************/
static esp_err_t deferred_driver_init(void);
static void sensor_update_zigbee_attributes(uint8_t param);
static void sensor_periodic_update(uint8_t param);
static esp_err_t button_init(void);
static void button_task(void *arg);
static void factory_reset_device(uint8_t param);

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
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        ESP_LOGI(TAG, "[JOIN] Device first start - factory new device");
        if (err_status == ESP_OK) {
            deferred_driver_init();
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "[JOIN] Device reboot - previously joined network");
        if (err_status == ESP_OK) {
            deferred_driver_init();
            if (esp_zb_bdb_is_factory_new()) {
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "[JOIN] *** SUCCESSFULLY JOINED NETWORK ***");
            ESP_LOGI(TAG, "[JOIN] PAN ID: 0x%04hx, Channel: %d", 
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            
            ota_validation_zigbee_connected();
            
            /* Start periodic sensor updates */
            esp_zb_scheduler_alarm((esp_zb_callback_t)sensor_periodic_update, 0, 5000);
            ESP_LOGI(TAG, "[JOIN] Setup complete!");
        } else {
            ESP_LOGW(TAG, "[JOIN] Network steering failed, retrying...");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                  ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
        
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "[LEAVE] Device left the network");
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
    
    // Air quality sensors are read-only, no attribute writes expected
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
    ESP_LOGI(TAG, "  VOC Index: %d, CO2: %d ppm", state.voc_index, state.co2_ppm);
    
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
    
    /* Update Endpoint 7: CO2 */
    float co2_value = (float)state.co2_ppm;
    esp_zb_zcl_set_attribute_val(HA_ESP_CO2_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
                                  &co2_value, false);
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
    
    /* Endpoint 1: Temperature and Humidity */
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = 0x8000,  // Invalid value (0x8000)
        .min_value = 0x954D,       // -40°C in 0.01°C units
        .max_value = 0x7FFF,       // Max valid value
    };
    
    esp_zb_humidity_meas_cluster_cfg_t humidity_cfg = {
        .measured_value = 0xFFFF,  // Invalid value
        .min_value = 0,            // 0% RH
        .max_value = 10000,        // 100% RH (in 0.01% units)
    };
    
    esp_zb_cluster_list_t *temp_hum_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Add Basic cluster */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(temp_hum_clusters, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* Add Temperature measurement cluster */
    esp_zb_attribute_list_t *temp_cluster = esp_zb_temperature_meas_cluster_create(&temp_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(temp_hum_clusters, temp_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* Add Humidity measurement cluster */
    esp_zb_attribute_list_t *humidity_cluster = esp_zb_humidity_meas_cluster_create(&humidity_cfg);
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
    
    /* Endpoint 2: Pressure */
    esp_zb_pressure_meas_cluster_cfg_t pressure_cfg = {
        .measured_value = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_UNKNOWN,
    };
    
    esp_zb_cluster_list_t *pressure_clusters = esp_zb_zcl_cluster_list_create();
    
    esp_zb_basic_cluster_cfg_t basic_pressure_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(pressure_clusters, esp_zb_basic_cluster_create(&basic_pressure_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_pressure_meas_cluster(pressure_clusters, esp_zb_pressure_meas_cluster_create(&pressure_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(pressure_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint2_config = {
        .endpoint = HA_ESP_PRESSURE_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, pressure_clusters, endpoint2_config);
    
    /* Endpoint 3: PM1.0 - using Analog Input cluster */
    esp_zb_cluster_list_t *pm1_clusters = esp_zb_zcl_cluster_list_create();
    
    esp_zb_basic_cluster_cfg_t basic_pm1_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(pm1_clusters, esp_zb_basic_cluster_create(&basic_pm1_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_analog_input_cluster_cfg_t pm1_cfg = {
        .present_value = 0.0f,
    };
    esp_zb_attribute_list_t *pm1_cluster = esp_zb_analog_input_cluster_create(&pm1_cfg);
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
    
    /* Endpoint 4: PM2.5 - using Analog Input cluster */
    esp_zb_cluster_list_t *pm25_clusters = esp_zb_zcl_cluster_list_create();
    
    esp_zb_basic_cluster_cfg_t basic_pm25_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(pm25_clusters, esp_zb_basic_cluster_create(&basic_pm25_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_analog_input_cluster_cfg_t pm25_cfg = {
        .present_value = 0.0f,
    };
    esp_zb_attribute_list_t *pm25_cluster = esp_zb_analog_input_cluster_create(&pm25_cfg);
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
    
    /* Endpoint 5: PM10 - using Analog Input cluster */
    esp_zb_cluster_list_t *pm10_clusters = esp_zb_zcl_cluster_list_create();
    
    esp_zb_basic_cluster_cfg_t basic_pm10_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(pm10_clusters, esp_zb_basic_cluster_create(&basic_pm10_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_analog_input_cluster_cfg_t pm10_cfg = {
        .present_value = 0.0f,
    };
    esp_zb_attribute_list_t *pm10_cluster = esp_zb_analog_input_cluster_create(&pm10_cfg);
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
    
    esp_zb_ep_list_add_ep(ep_list, pm10_clusters, endpoint5_config);
    
    /* Endpoint 6: VOC Index - using Analog Input cluster */
    esp_zb_cluster_list_t *voc_clusters = esp_zb_zcl_cluster_list_create();
    
    esp_zb_basic_cluster_cfg_t basic_voc_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(voc_clusters, esp_zb_basic_cluster_create(&basic_voc_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_analog_input_cluster_cfg_t voc_cfg = {
        .present_value = 0.0f,
    };
    esp_zb_attribute_list_t *voc_cluster = esp_zb_analog_input_cluster_create(&voc_cfg);
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
    
    esp_zb_ep_list_add_ep(ep_list, voc_clusters, endpoint6_config);
    
    /* Endpoint 7: CO2 - using Carbon Dioxide Measurement cluster */
    esp_zb_cluster_list_t *co2_clusters = esp_zb_zcl_cluster_list_create();
    
    esp_zb_basic_cluster_cfg_t basic_co2_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(co2_clusters, esp_zb_basic_cluster_create(&basic_co2_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* CO2 measurement cluster */
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_cfg = {
        .measured_value = 0,
        .min_measured_value = 400,   // 400 ppm
        .max_measured_value = 5000,  // 5000 ppm
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(co2_clusters, esp_zb_carbon_dioxide_measurement_cluster_create(&co2_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(co2_clusters, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    esp_zb_endpoint_config_t endpoint7_config = {
        .endpoint = HA_ESP_CO2_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, co2_clusters, endpoint7_config);
    
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
