#include "devices/SeedStudioXiaoEspC6Device.hpp"
#include "defs/DeviceDefs.hpp"
#include "driver/gpio.h"
#include <chrono>
#include <cstdint>
#include <esp_log.h>
#include <optional>

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6

namespace devices {
SeedStudioXiaoEspC6Device::SeedStudioXiaoEspC6Device() : statusLed(HASS_SENSOR_STATUS_LED_GPIO, HASS_SENSOR_STATUS_LED_LOW_ACTIVE, HASS_SENSOR_STATUS_LED_MAX_BRIGHTNESS_PERCENT), redLed(HASS_SENSOR_RED_LED_GPIO, HASS_SENSOR_RED_LED_LOW_ACTIVE, HASS_SENSOR_RED_LED_MAX_BRIGHTNESS_PERCENT), greenLed(HASS_SENSOR_GREEN_LED_GPIO, HASS_SENSOR_GREEN_LED_LOW_ACTIVE, HASS_SENSOR_GREEN_LED_MAX_BRIGHTNESS_PERCENT) {}

void SeedStudioXiaoEspC6Device::init() {
    statusLed.init();
    statusLed.set_off();

    redLed.init();
    redLed.set_off();

    greenLed.init();
    greenLed.set_off();

    // Enable the RF Switch
    gpio_reset_pin(gpio_num_t::GPIO_NUM_3);
    gpio_set_direction(gpio_num_t::GPIO_NUM_3, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num_t::GPIO_NUM_3, 0);

    // Switch between the built-in and external antenna
    gpio_reset_pin(gpio_num_t::GPIO_NUM_14);
    gpio_set_direction(gpio_num_t::GPIO_NUM_14, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num_t::GPIO_NUM_14, HASS_SENSOR_ANTENNA_EXTERNAL ? 1 : 0);

    ESP_LOGI(TAG, "Antenna mode: %s", ::HASS_SENSOR_ANTENNA_EXTERNAL ? "external" : "internal");
}

void SeedStudioXiaoEspC6Device::reset_state_leds() {
    redLed.set_off();
    greenLed.set_off();
}

void SeedStudioXiaoEspC6Device::on_device_state_changed(zigbee::ZigbeeDeviceState state) {
    reset_state_leds();

    switch (state) {
        case zigbee::ZigbeeDeviceState::SETUP:
            redLed.set_blink(std::chrono::milliseconds{500});
            break;
        case zigbee::ZigbeeDeviceState::CONNECTING:
            greenLed.set_blink(std::chrono::milliseconds{500});
            break;
        case zigbee::ZigbeeDeviceState::OTA:
            redLed.set_blink(std::chrono::milliseconds{500});
            break;
        case zigbee::ZigbeeDeviceState::CONNECTED:
        default:
            break;
    }
}

void SeedStudioXiaoEspC6Device::on_identify(uint16_t identifyTime) {
    if (identifyTime > 0) {
        statusLed.set_blink(std::chrono::milliseconds(500), std::make_optional<size_t>(identifyTime * 2));
    } else {
        statusLed.set_off();
    }
}

bool SeedStudioXiaoEspC6Device::has_debug_led() const {
    return false;
}

bool SeedStudioXiaoEspC6Device::is_debug_led_enabled() const {
    return false;
}

void SeedStudioXiaoEspC6Device::set_debug_led(bool /*enabled*/) {}

void SeedStudioXiaoEspC6Device::indicate_error() {
    redLed.set_on();
}
} // namespace devices

#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6
