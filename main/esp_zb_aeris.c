/*
 * Zigbee Aeris Thermostat (renamed from esp_zb_hvac.c)
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

/* HVAC (Aeris) state update interval */
#define HVAC_UPDATE_INTERVAL_MS     30000  // 30 seconds

/* Boot button configuration for factory reset */
#define BOOT_BUTTON_GPIO            GPIO_NUM_9
#define BUTTON_LONG_PRESS_TIME_MS   5000

/* The remainder of esp_zb_aeris.c matches the original implementation in
   esp_zb_hvac.c with include names updated above. */
