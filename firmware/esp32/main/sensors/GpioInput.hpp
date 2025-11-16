#pragma once

#include "hal/gpio_types.h"
extern "C" {
#include "driver/gpio.h"
}

namespace sensors {
class GpioInput {
  private:
    gpio_num_t gpio;

  public:
    GpioInput(gpio_num_t gpio, gpio_pullup_t pullUp = GPIO_PULLUP_ENABLE, gpio_pulldown_t pullDown = GPIO_PULLDOWN_ENABLE);
    GpioInput(GpioInput&&) = default;
    GpioInput(const GpioInput&) = default;
    GpioInput& operator=(GpioInput&&) = default;
    GpioInput& operator=(const GpioInput&) = default;
    ~GpioInput() = default;

    bool is_powered();
};
} // namespace sensors
