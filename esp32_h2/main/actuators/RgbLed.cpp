#include "actuators/RgbLed.hpp"
#include "driver/gpio.h"
#include "hal/gpio_types.h"

namespace actuators {
RgbLed::RgbLed(gpio_num_t gpio) : gpio(gpio) {
    init();
}

void RgbLed::init() {
    gpio_config_t io_conf = {};

    // Set the pin to output:
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << gpio);

    // Disable everything else:
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

    // Make sure the LED is off by default:
    gpio_set_level(gpio, 0);
}

void RgbLed::on() {
    gpio_set_level(gpio, 1);
}

void RgbLed::off() {
    gpio_set_level(gpio, 0);
}
}  // namespace actuators