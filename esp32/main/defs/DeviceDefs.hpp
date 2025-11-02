// Option to disable include resolution inside the CI when we only want to run preprocessor steps.
#ifndef HASS_SENSOR_SKIP_INCLUDES

#pragma once

#include <cstdint>
#include <sdkconfig.h>
#include <soc/gpio_num.h>

#endif // !HASS_SENSOR_SKIP_INCLUDES

// ------------------------------------------------------------
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6

// OTA image type to correctly identify which image is for which device.
constexpr uint16_t OTA_IMAGE_TYPE = 100;

// Use the internal or external antenna
constexpr bool HASS_SENSOR_ANTENNA_EXTERNAL = true;

// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_22;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_23;

// Status LED
#define HASS_SENSOR_STATUS_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_STATUS_LED_GPIO = gpio_num_t::GPIO_NUM_15;
constexpr bool HASS_SENSOR_STATUS_LED_LOW_ACTIVE = true;

#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6

// ------------------------------------------------------------
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT

// OTA image type to correctly identify which image is for which device.
constexpr uint16_t OTA_IMAGE_TYPE = 200;

// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_12;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_22;

// RGB Debug LED
#define HASS_SENSOR_DEBUG_RGB_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_DEBUG_RGB_LED_GPIO = gpio_num_t::GPIO_NUM_8;

#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT

// ------------------------------------------------------------
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT

// OTA image type to correctly identify which image is for which device.
constexpr uint16_t OTA_IMAGE_TYPE = 300;

// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_12;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_22;

// RGB Debug LED
#define HASS_SENSOR_DEBUG_RGB_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_DEBUG_RGB_LED_GPIO = gpio_num_t::GPIO_NUM_8;

#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT
