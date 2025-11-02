#include "actuators/Led.hpp"

#include "driver/gpio.h"
#include "soc/gpio_num.h"

namespace actuators {
Led::Led(gpio_num_t gpio, bool lowActive) : gpio(gpio), lowActive(lowActive) {}

Led::~Led() {
    // std::jthread's destructor requests stop and joins automatically.
    // Wake the worker if it's waiting.
    cv.notify_all();
}

void Led::init() {
    // Configure GPIO
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, lowActive ? 1 : 0);

    // Launch worker exactly once
    const std::scoped_lock lock(mtx);
    if (started) {
        return;
    }
    started = true;

    worker = std::jthread([this](std::stop_token st) { this->run(st); });
}

void Led::set_on() {
    {
        const std::scoped_lock lock(mtx);
        mode = Mode::On;
    }
    cv.notify_all();
}

void Led::set_off() {
    {
        const std::scoped_lock lock(mtx);
        mode = Mode::Off;
    }
    cv.notify_all();
}

void Led::set_blink(std::chrono::milliseconds interval, std::optional<size_t> count) {
    {
        const std::scoped_lock lock(mtx);
        blinkInterval = interval.count() > 0 ? interval : std::chrono::milliseconds{1};
        blinkCount = blinkCount;
        mode = Mode::Blink;
    }
    cv.notify_all();
}

void Led::run(std::stop_token st) {
    // Local helper to wait for a mode change (or stop)
    auto waitForModeChange = [this, &st](Mode current) {
        std::unique_lock lock(mtx);
        cv.wait(lock, st, [this, current] { return mode != current; });
    };

    // Main loop
    while (!st.stop_requested()) {
        Mode current{Mode::Off};
        {
            const std::scoped_lock lock(mtx);
            current = mode;
        }

        if (current == Mode::Off) {
            gpio_set_level(gpio, lowActive ? 1 : 0);
            waitForModeChange(Mode::Off);
            continue;
        }

        if (current == Mode::On) {
            gpio_set_level(gpio, lowActive ? 0 : 1);
            waitForModeChange(Mode::On);
            continue;
        }

        // Blink mode: toggle with interruptable waits so we react instantly
        // to mode changes or stop requests.
        bool level = false;
        for (;;) {
            // Re-check stop or mode change before each toggle
            if (st.stop_requested()) {
                return;
            }

            if (blinkCount) {
                if (*blinkCount <= 0) {
                    const std::scoped_lock lock(mtx);
                    mode = Mode::Off;
                    break;
                }

                if (level) {
                    *blinkCount = *blinkCount - 1;
                }
            }

            {
                const std::scoped_lock lock(mtx);
                if (mode != Mode::Blink) {
                    break;
                }
            }

            level = !level;
            if (level) {
                gpio_set_level(gpio, lowActive ? 0 : 1);
            } else {
                gpio_set_level(gpio, lowActive ? 1 : 0);
            }

            std::unique_lock lock(mtx);
            std::chrono::milliseconds nextBlinkState = blinkInterval;
            if (cv.wait_for(lock, st, nextBlinkState, [this] { return mode != Mode::Blink; })) {
                // Predicate true -> mode changed
                break;
            }
        }
    }
}
} // namespace actuators
