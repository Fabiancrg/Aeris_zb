/* Board-specific hardware configuration
 * Toggle XIAO external antenna switching and pin definitions here.
 *
 * To enable external antenna selection, set ENABLE_XIAO_EXTERNAL_ANT to 1.
 * The code will drive the FM8625H switch pins as follows (per board docs):
 *   - GPIO3 = LOW
 *   - GPIO14 = HIGH
 */
#pragma once

#include "driver/gpio.h"

/* Enable (1) or disable (0) automatic external antenna switching in firmware */
#ifndef ENABLE_XIAO_EXTERNAL_ANT
#define ENABLE_XIAO_EXTERNAL_ANT 1
#endif

/* Pin definitions for antenna switch (XIAO ESP32-C6) */
#ifndef XIAO_ANT_GPIO_SEL0
#define XIAO_ANT_GPIO_SEL0 GPIO_NUM_3
#endif

#ifndef XIAO_ANT_GPIO_SEL1
#define XIAO_ANT_GPIO_SEL1 GPIO_NUM_14
#endif

/* RGB LED configuration (SK6812) - 5 separate LEDs for air quality indicators */
#ifndef LED_CO2_GPIO
#define LED_CO2_GPIO GPIO_NUM_21  /* CO2 level indicator */
#endif

#ifndef LED_VOC_GPIO
#define LED_VOC_GPIO GPIO_NUM_4   /* VOC Index indicator */
#endif

#ifndef LED_NOX_GPIO
#define LED_NOX_GPIO GPIO_NUM_8   /* NOx Index indicator */
#endif

#ifndef LED_PM25_GPIO
#define LED_PM25_GPIO GPIO_NUM_5  /* PM2.5 level indicator */
#endif

#ifndef LED_HUM_GPIO
#define LED_HUM_GPIO GPIO_NUM_10  /* Humidity level indicator */
#endif

#define LED_STATUS_NUM_LEDS 1  /* Number of LEDs per indicator (1 each) */
