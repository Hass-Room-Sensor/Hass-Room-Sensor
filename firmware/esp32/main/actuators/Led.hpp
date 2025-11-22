#pragma once

#include "driver/ledc.h"
#include "soc/gpio_num.h"
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

namespace actuators {

class Led {
  private:
    enum class Mode : uint8_t { OFF, ON, BLINK };

    static constexpr ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
    static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT;
    static constexpr uint32_t PWM_FREQUENCY_HZ = 5000;

    Mode mode{Mode::OFF};
    std::chrono::milliseconds blinkInterval{500};
    std::optional<size_t> blinkCount{std::nullopt};

    gpio_num_t gpio;
    // True indicates the LED is on in case the GPIO pin is set to 0
    bool lowActive;
    // If the LED is on, define the percentage of that time it is actually on to simulate reduced LED brightness.
    uint8_t onDutyPercent{100};
    ledc_channel_t channel{};
    // Will be increased every time the LED state changes. Allows the worker thread to sleep until something changed.
    uint64_t stateVersion{0};

    // Thread & synchronization
    std::jthread worker{};
    std::mutex mtx;
    std::condition_variable_any cv;
    bool started{false};

  public:
    Led(gpio_num_t gpio, bool lowActive, uint8_t onDutyPercent = 100);
    Led(Led&&) = default;
    Led(const Led&) = default;
    Led& operator=(Led&&) = default;
    Led& operator=(const Led&) = default;
    ~Led();

    void init();

    void set_on();
    void set_off();
    void set_blink(std::chrono::milliseconds interval = std::chrono::milliseconds{500}, std::optional<size_t> count = std::nullopt);
    void set_on_duty_cycle(uint8_t dutyPercent);

  private:
    void run(std::stop_token st);
    void apply_off();
    void apply_on_duty();
    void apply_duty(uint32_t duty);
    [[nodiscard]] uint32_t compute_on_duty_ticks() const;

    static uint32_t max_duty();
    static void ensure_timer_initialized();
};

} // namespace actuators
