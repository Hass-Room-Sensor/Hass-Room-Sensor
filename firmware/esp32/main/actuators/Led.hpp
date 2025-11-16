#pragma once

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
    enum class Mode : uint8_t { Off, On, Blink };

    Mode mode{Mode::Off};
    std::chrono::milliseconds blinkInterval{500};
    std::optional<size_t> blinkCount{std::nullopt};

    gpio_num_t gpio;
    // True indicates the LED is on in case the GPIO pin is set to 0
    bool lowActive;

    // Thread & synchronization
    std::jthread worker{};
    std::mutex mtx;
    std::condition_variable_any cv;
    bool started{false};

  public:
    Led(gpio_num_t gpio, bool lowActive);
    Led(Led&&) = default;
    Led(const Led&) = default;
    Led& operator=(Led&&) = default;
    Led& operator=(const Led&) = default;
    ~Led();

    void init();

    void set_on();
    void set_off();
    void set_blink(std::chrono::milliseconds interval = std::chrono::milliseconds{500}, std::optional<size_t> count = std::nullopt);

  private:
    void run(std::stop_token st);
};

} // namespace actuators
