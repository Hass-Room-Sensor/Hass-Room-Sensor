#include "sensors/GpioInput.hpp"

namespace sensors {
GpioInput::GpioInput(gpio_num_t gpio, gpio_pullup_t pullUp, gpio_pulldown_t pullDown) : gpio(gpio) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << gpio);
    io_conf.pull_down_en = pullDown;
    io_conf.pull_up_en = pullUp;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

bool GpioInput::is_powered() {
    return gpio_get_level(gpio) == 1;
}
} // namespace sensors