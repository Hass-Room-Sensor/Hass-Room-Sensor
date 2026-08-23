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

// Use the internal or external antenna.
constexpr bool HASS_SENSOR_ANTENNA_EXTERNAL = true;

// Input high when the device is battery powered.
constexpr gpio_num_t HASS_SENSOR_POWER_SOURCE_GPIO = gpio_num_t::GPIO_NUM_21;

// The XIAO BOOT button pulls GPIO9 low while pressed.
constexpr gpio_num_t HASS_SENSOR_FACTORY_RESET_GPIO = gpio_num_t::GPIO_NUM_9;
constexpr bool HASS_SENSOR_FACTORY_RESET_LOW_ACTIVE = true;

// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_22;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_23;
constexpr gpio_num_t HASS_SENSOR_BME690_SDA_GPIO = HASS_SENSOR_SCD4X_SDA_GPIO;
constexpr gpio_num_t HASS_SENSOR_BME690_SCL_GPIO = HASS_SENSOR_SCD4X_SCL_GPIO;

// Status LED
#define HASS_SENSOR_STATUS_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_STATUS_LED_GPIO = gpio_num_t::GPIO_NUM_15;
constexpr bool HASS_SENSOR_STATUS_LED_LOW_ACTIVE = true;
constexpr uint8_t HASS_SENSOR_STATUS_LED_MAX_BRIGHTNESS_PERCENT = 100;

constexpr gpio_num_t HASS_SENSOR_RED_LED_GPIO = gpio_num_t::GPIO_NUM_1;
constexpr bool HASS_SENSOR_RED_LED_LOW_ACTIVE = true;
constexpr uint8_t HASS_SENSOR_RED_LED_MAX_BRIGHTNESS_PERCENT = 1; // Yes, 1% max brightness is enough

constexpr gpio_num_t HASS_SENSOR_GREEN_LED_GPIO = gpio_num_t::GPIO_NUM_0;
constexpr bool HASS_SENSOR_GREEN_LED_LOW_ACTIVE = true;
constexpr uint8_t HASS_SENSOR_GREEN_LED_MAX_BRIGHTNESS_PERCENT = 1; // Yes, 1% max brightness is enough

#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6

// ------------------------------------------------------------
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT

// OTA image type to correctly identify which image is for which device.
constexpr uint16_t OTA_IMAGE_TYPE = 200;

// Input high when the device is battery powered.
constexpr gpio_num_t HASS_SENSOR_POWER_SOURCE_GPIO = gpio_num_t::GPIO_NUM_3;

// External active-high factory-reset input used by the development kit setup.
constexpr gpio_num_t HASS_SENSOR_FACTORY_RESET_GPIO = gpio_num_t::GPIO_NUM_1;
constexpr bool HASS_SENSOR_FACTORY_RESET_LOW_ACTIVE = false;

// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_12;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_22;
constexpr gpio_num_t HASS_SENSOR_BME690_SDA_GPIO = HASS_SENSOR_SCD4X_SDA_GPIO;
constexpr gpio_num_t HASS_SENSOR_BME690_SCL_GPIO = HASS_SENSOR_SCD4X_SCL_GPIO;

// RGB Debug LED
#define HASS_SENSOR_DEBUG_RGB_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_DEBUG_RGB_LED_GPIO = gpio_num_t::GPIO_NUM_8;

#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT

// ------------------------------------------------------------
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT

// OTA image type to correctly identify which image is for which device.
constexpr uint16_t OTA_IMAGE_TYPE = 300;

// Input high when the device is battery powered.
constexpr gpio_num_t HASS_SENSOR_POWER_SOURCE_GPIO = gpio_num_t::GPIO_NUM_3;

// External active-high factory-reset input used by the development kit setup.
constexpr gpio_num_t HASS_SENSOR_FACTORY_RESET_GPIO = gpio_num_t::GPIO_NUM_1;
constexpr bool HASS_SENSOR_FACTORY_RESET_LOW_ACTIVE = false;

// SCD4X I2C GPIOs
constexpr gpio_num_t HASS_SENSOR_SCD4X_SDA_GPIO = gpio_num_t::GPIO_NUM_12;
constexpr gpio_num_t HASS_SENSOR_SCD4X_SCL_GPIO = gpio_num_t::GPIO_NUM_22;
constexpr gpio_num_t HASS_SENSOR_BME690_SDA_GPIO = HASS_SENSOR_SCD4X_SDA_GPIO;
constexpr gpio_num_t HASS_SENSOR_BME690_SCL_GPIO = HASS_SENSOR_SCD4X_SCL_GPIO;

// RGB Debug LED
#define HASS_SENSOR_DEBUG_RGB_LED_ENABLED
constexpr gpio_num_t HASS_SENSOR_DEBUG_RGB_LED_GPIO = gpio_num_t::GPIO_NUM_8;

#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT
