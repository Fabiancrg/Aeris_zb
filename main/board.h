/* Board-specific hardware configuration for ESP32-C6 Supermini
 * GPIO pin definitions for sensors, LEDs, and peripherals
 */
#pragma once

#include "driver/gpio.h"

/* RGB LED configuration (SK6812) - 5 separate LEDs for air quality indicators
 * Optimized for ESP32-C6 Supermini available GPIOs
 */
#ifndef LED_CO2_GPIO
#define LED_CO2_GPIO GPIO_NUM_1   /* CO2 level indicator */
#endif

#ifndef LED_VOC_GPIO
#define LED_VOC_GPIO GPIO_NUM_18  /* VOC Index indicator */
#endif

#ifndef LED_NOX_GPIO
#define LED_NOX_GPIO GPIO_NUM_15  /* NOx Index indicator */
#endif

#ifndef LED_PM25_GPIO
#define LED_PM25_GPIO GPIO_NUM_20 /* PM2.5 level indicator */
#endif

#ifndef LED_HUM_GPIO
#define LED_HUM_GPIO GPIO_NUM_14  /* Humidity level indicator */
#endif

#ifndef LED_STATUS_GPIO
#define LED_STATUS_GPIO GPIO_NUM_8  /* Zigbee network status indicator (built-in LED) */
#endif

#define LED_STATUS_NUM_LEDS 1  /* Number of LEDs per indicator (1 each) */

/* PMSA003A PM sensor control pins */
#ifndef PMSA003A_SET_GPIO
#define PMSA003A_SET_GPIO GPIO_NUM_19  /* Sleep/Wake control (HIGH=active, LOW=sleep) */
#endif

#ifndef PMSA003A_RESET_GPIO
#define PMSA003A_RESET_GPIO GPIO_NUM_2  /* Hardware reset (active LOW, optional) */
#endif
