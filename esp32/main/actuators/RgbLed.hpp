#pragma once

#include "hal/gpio_types.h"
#include "led_strip.h"
#include <cstdint>
#include <string_view>

namespace actuators {
struct color_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} __attribute__((aligned(4)));

class RgbLed {
  private:
    static constexpr std::string_view NVS_NAMESPACE = "RgbLed";
    static constexpr std::string_view NVS_ENABLED_KEY = "enabled";

    gpio_num_t gpio;
    led_strip_handle_t led_strip{};
    bool enabled{true};
    color_t currentColor{};

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

    void enable();
    void disable();

    [[nodiscard]] bool is_enabled() const;

  private:
    static color_t hue_tu_rgb(double hue);

    void load_enabled();
    void save_enabled() const;
};
} // namespace actuators