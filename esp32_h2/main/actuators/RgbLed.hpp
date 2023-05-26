#pragma once

#include "hal/gpio_types.h"
namespace actuators {
class RgbLed {
 private:
    gpio_num_t gpio;

 public:
    explicit RgbLed(gpio_num_t gpio);
    RgbLed(RgbLed&&) = default;
    RgbLed(const RgbLed&) = default;
    RgbLed& operator=(RgbLed&&) = default;
    RgbLed& operator=(const RgbLed&) = default;
    ~RgbLed() = default;

    void on();
    void off();

 private:
    void init();
};
}  // namespace actuators