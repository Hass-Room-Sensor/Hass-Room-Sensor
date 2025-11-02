#pragma once

#include "soc/gpio_num.h"

/**
 * Compile the application for the seed studio XIAO ESPC6.
 * Docs: https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/
 **/
#define HASS_SENSOR_DEVICE_SEED_STUDIO_XIAO_ESPC6

/**
 * Compile the application for the official ESP32 H2 developer kit.
 * Docs: https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32h2/index.html
 **/
// #define HASS_SENSOR_DEVICE_H2_DEVKIT

/**
 * Compile the application for the official ESP32 C6 developer kit.
 * Docs: https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/index.html
 **/
// #define HASS_SENSOR_DEVICE_C6_DEVKIT

#define CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_MOCK
// ------------------------------------------------------------
#ifdef HASS_SENSOR_DEVICE_SEED_STUDIO_XIAO_ESPC6
// Use the internal or external antenna
constexpr bool HASS_SENSOR_ANTENNA_EXTERNAL = true;

// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_22;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_23;

// Status LED
#define HASS_SENSOR_STATUS_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_STATUS_LED_GPIO = gpio_num_t::GPIO_NUM_15;
constexpr bool HASS_SENSOR_STATUS_LED_LOW_ACTIVE = true;

// Validation
#if defined(HASS_SENSOR_DEVICE_H2_DEVKIT) || defined(HASS_SENSOR_DEVICE_C6_DEVKIT)
#error "Only a single hardware target can be defined!"
#endif
#endif // HASS_SENSOR_DEVICE_SEED_STUDIO_XIAO_ESPC6

// ------------------------------------------------------------
#ifdef HASS_SENSOR_DEVICE_H2_DEVKIT
// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_12;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_22;

// RGB Debug LED
#define HASS_SENSOR_DEBUG_RGB_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_DEBUG_RGB_LED_GPIO = gpio_num_t::GPIO_NUM_8;

// Validation
#if defined(HASS_SENSOR_DEVICE_SEED_STUDIO_XIAO_ESPC6) || defined(HASS_SENSOR_DEVICE_C6_DEVKIT)
#error "Only a single hardware target can be defined!"
#endif
#endif // HASS_SENSOR_DEVICE_H2_DEVKIT

// ------------------------------------------------------------
#ifdef HASS_SENSOR_DEVICE_C6_DEVKIT
// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_12;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_22;

// RGB Debug LED
#define HASS_SENSOR_DEBUG_RGB_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_DEBUG_RGB_LED_GPIO = gpio_num_t::GPIO_NUM_8;

// Validation
#if defined(HASS_SENSOR_DEVICE_H2_DEVKIT) || defined(HASS_SENSOR_DEVICE_SEED_STUDIO_XIAO_ESPC6)
#error "Only a single hardware target can be defined!"
#endif
#endif // HASS_SENSOR_DEVICE_C6_DEVKIT
