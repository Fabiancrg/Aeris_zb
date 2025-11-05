/*
 * Zigbee Aeris Air Quality Sensor Header
 */

#ifndef ESP_ZB_AERIS_H
#define ESP_ZB_AERIS_H

#include "esp_zigbee_core.h"
#include "aeris_driver.h"
#include "zcl_utility.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false                                /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN        /* aging timeout of device */
#define ED_KEEP_ALIVE                   3000                                 /* 3000 millisecond */

/* Air Quality Sensor Endpoints */
#define HA_ESP_TEMP_HUM_ENDPOINT        1                                    /* Temperature and Humidity sensor endpoint */
#define HA_ESP_PRESSURE_ENDPOINT        2                                    /* Pressure sensor endpoint */
#define HA_ESP_PM1_ENDPOINT             3                                    /* PM1.0 sensor endpoint */
#define HA_ESP_PM25_ENDPOINT            4                                    /* PM2.5 sensor endpoint */
#define HA_ESP_PM10_ENDPOINT            5                                    /* PM10 sensor endpoint */
#define HA_ESP_VOC_ENDPOINT             6                                    /* VOC Index sensor endpoint */
#define HA_ESP_NOX_ENDPOINT             7                                    /* NOx Index sensor endpoint */
#define HA_ESP_CO2_ENDPOINT             8                                    /* CO2 sensor endpoint */
#define HA_ESP_LED_CONFIG_ENDPOINT      9                                    /* LED configuration endpoint */

/* Custom attribute IDs for LED thresholds (manufacturer-specific range 0xF000-0xFFFF) */
#define ZCL_LED_ATTR_VOC_ORANGE         0xF000
#define ZCL_LED_ATTR_VOC_RED            0xF001
#define ZCL_LED_ATTR_CO2_ORANGE         0xF002
#define ZCL_LED_ATTR_CO2_RED            0xF003
#define ZCL_LED_ATTR_HUM_ORANGE_LOW     0xF004
#define ZCL_LED_ATTR_HUM_ORANGE_HIGH    0xF005
#define ZCL_LED_ATTR_HUM_RED_LOW        0xF006
#define ZCL_LED_ATTR_HUM_RED_HIGH       0xF007
#define ZCL_LED_ATTR_PM25_ORANGE        0xF008
#define ZCL_LED_ATTR_PM25_RED           0xF009

#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/* Button configuration */
#define ESP_INTR_FLAG_DEFAULT 0

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07""aeris-z"         /* Air quality sensor model */

/* Router configuration (was ZED_CONFIG - changed to match working example) */
#define ESP_ZB_ROUTER_CONFIG()                                      \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                   \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg = {                                                \
            .zczr_cfg = {                                           \
                .max_children = 10,                                 \
            },                                                      \
        },                                                          \
    }

/* Legacy ZED config kept for reference (not used) */
#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

#endif
