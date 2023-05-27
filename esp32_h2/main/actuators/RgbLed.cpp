#include "actuators/RgbLed.hpp"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include <cmath>
#include <cstdint>

namespace actuators {
RgbLed::RgbLed(gpio_num_t gpio) : gpio(gpio) {
    init();
}

void RgbLed::init() {
    gpio_reset_pin(gpio);
    // Set the GPIO as  output:
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);

    // Make sure the LED is off by default:
    gpio_set_level(gpio, 0);

    led_strip_config_t strip_config{};
    strip_config.strip_gpio_num = gpio;
    strip_config.max_leds = 1;
    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;  // 10MHz

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // Set all LED off to clear all pixels:
    led_strip_clear(led_strip);
}

void RgbLed::on(color_t color) {
    led_strip_set_pixel(led_strip, 0, static_cast<uint32_t>(color.r), static_cast<uint32_t>(color.g), static_cast<uint32_t>(color.b));
    led_strip_refresh(led_strip);
}

void RgbLed::on(double hue) {
    on(hueToRgb(hue));
}

void RgbLed::off() {
    led_strip_clear(led_strip);
}

color_t RgbLed::hueToRgb(double hue) {
    double r = std::abs(hue * 6.0 - 3.0) - 1.0;
    double g = 2.0 - std::abs(hue * 6.0 - 2.0);
    double b = 2.0 - std::abs(hue * 6.0 - 4.0);

    r = std::max(0.0, std::min(1.0, r));
    g = std::max(0.0, std::min(1.0, g));
    b = std::max(0.0, std::min(1.0, b));

    return {static_cast<uint8_t>(r * 255), static_cast<uint8_t>(g * 255), static_cast<uint8_t>(b * 255)};
}
}  // namespace actuators