#include "devices/SeedStudioXiaoEspC6Device.hpp"
#include "defs/DeviceDefs.hpp"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <chrono>
#include <cstdint>
#include <esp_err.h>
#include <esp_log.h>
#include <optional>

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6

namespace devices {
SeedStudioXiaoEspC6Device::SeedStudioXiaoEspC6Device() : statusLed(HASS_SENSOR_STATUS_LED_GPIO, HASS_SENSOR_STATUS_LED_LOW_ACTIVE, HASS_SENSOR_STATUS_LED_MAX_BRIGHTNESS_PERCENT), redLed(HASS_SENSOR_RED_LED_GPIO, HASS_SENSOR_RED_LED_LOW_ACTIVE, HASS_SENSOR_RED_LED_MAX_BRIGHTNESS_PERCENT), greenLed(HASS_SENSOR_GREEN_LED_GPIO, HASS_SENSOR_GREEN_LED_LOW_ACTIVE, HASS_SENSOR_GREEN_LED_MAX_BRIGHTNESS_PERCENT) {}

SeedStudioXiaoEspC6Device::~SeedStudioXiaoEspC6Device() {
    stop_identify_effect();
    if (identifyRestoreTimer_ != nullptr) {
        ESP_ERROR_CHECK(esp_timer_delete(identifyRestoreTimer_));
        identifyRestoreTimer_ = nullptr;
    }
}

void SeedStudioXiaoEspC6Device::init() {
    gpio_hold_dis(HASS_SENSOR_STATUS_LED_GPIO);

    statusLed.init();
    statusLed.set_off();

    redLed.init();
    redLed.set_off();

    greenLed.init();
    greenLed.set_off();

    if (identifyRestoreTimer_ == nullptr) {
        const esp_timer_create_args_t identifyRestoreTimerArgs{
            .callback = &SeedStudioXiaoEspC6Device::on_identify_restore_timer,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "xiao_identify",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&identifyRestoreTimerArgs, &identifyRestoreTimer_));
    }

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
            greenLed.set_on();
            break;
        case zigbee::ZigbeeDeviceState::CONNECTED:
        default:
            break;
    }
}

void SeedStudioXiaoEspC6Device::on_identify(uint16_t identifyTime) {
    if (identifyTime > 0) {
        stop_identify_effect();
        {
            const std::scoped_lock lock(statusLedMutex_);
            identifyActive_ = true;
        }
        statusLed.set_blink(std::chrono::milliseconds(500), std::make_optional<size_t>(identifyTime * 2));
        if (identifyRestoreTimer_ != nullptr) {
            ESP_ERROR_CHECK(esp_timer_start_once(identifyRestoreTimer_, std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::seconds{identifyTime}).count()));
        }
    } else {
        stop_identify_effect();
        refresh_status_led();
    }
}

bool SeedStudioXiaoEspC6Device::has_debug_led() const {
    return false;
}

bool SeedStudioXiaoEspC6Device::is_debug_led_enabled() const {
    return false;
}

void SeedStudioXiaoEspC6Device::set_debug_led(bool /*enabled*/) {}

void SeedStudioXiaoEspC6Device::set_sleep_indicator(bool sleeping) {
    {
        const std::scoped_lock lock(statusLedMutex_);
        sleepIndicatorActive_ = sleeping;
    }
    refresh_status_led();
}

void SeedStudioXiaoEspC6Device::prepare_for_deep_sleep() {
    stop_identify_effect();
    {
        const std::scoped_lock lock(statusLedMutex_);
        sleepIndicatorActive_ = true;
        identifyActive_ = false;
    }
    statusLed.set_on();
    ESP_ERROR_CHECK(gpio_hold_en(HASS_SENSOR_STATUS_LED_GPIO));
}

void SeedStudioXiaoEspC6Device::indicate_error() {
    redLed.set_on();
}

void SeedStudioXiaoEspC6Device::refresh_status_led() {
    bool identifyActive = false;
    bool sleepIndicatorActive = false;
    {
        const std::scoped_lock lock(statusLedMutex_);
        identifyActive = identifyActive_;
        sleepIndicatorActive = sleepIndicatorActive_;
    }

    if (identifyActive) {
        return;
    }

    if (sleepIndicatorActive) {
        statusLed.set_on();
    } else {
        statusLed.set_off();
    }
}

void SeedStudioXiaoEspC6Device::stop_identify_effect() {
    if (identifyRestoreTimer_ != nullptr) {
        const esp_err_t stopResult = esp_timer_stop(identifyRestoreTimer_);
        if (stopResult != ESP_OK && stopResult != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(stopResult);
        }
    }

    const std::scoped_lock lock(statusLedMutex_);
    identifyActive_ = false;
}

void SeedStudioXiaoEspC6Device::on_identify_restore_timer(void* arg) {
    auto* device = static_cast<SeedStudioXiaoEspC6Device*>(arg);
    if (device == nullptr) {
        return;
    }

    {
        const std::scoped_lock lock(device->statusLedMutex_);
        device->identifyActive_ = false;
    }
    device->refresh_status_led();
}
} // namespace devices

#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6
