#pragma once

#include "hal/gpio_types.h"
#include "led_strip.h"
#include <cstdint>

namespace actuators {
struct color_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} __attribute__((aligned(4)));

class RgbLed {
 private:
    gpio_num_t gpio;
    led_strip_handle_t led_strip{};

 public:
    explicit RgbLed(gpio_num_t gpio);
    RgbLed(RgbLed&&) = default;
    RgbLed(const RgbLed&) = default;
    RgbLed& operator=(RgbLed&&) = default;
    RgbLed& operator=(const RgbLed&) = default;
    ~RgbLed() = default;

    void init();

    void on(color_t color);
    void on(double hue);
    void off();

 private:
    static color_t hueToRgb(double hue);
};
}  // namespace actuators