#include "actuators/Led.hpp"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "soc/gpio_num.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <mutex>

namespace {
std::once_flag g_timerInitFlag;
std::atomic_int g_nextChannel{0};

ledc_channel_t acquire_channel() {
    const int channelIndex = g_nextChannel.fetch_add(1);
    assert(channelIndex < LEDC_CHANNEL_MAX);
    return static_cast<ledc_channel_t>(channelIndex);
}
} // namespace

namespace actuators {
Led::Led(gpio_num_t gpio, bool lowActive, uint8_t onDutyPercent) : gpio(gpio), lowActive(lowActive), onDutyPercent(onDutyPercent), channel(acquire_channel()) {}

Led::~Led() {
    // std::jthread's destructor requests stop and joins automatically.
    // Wake the worker if it's waiting.
    cv.notify_all();
}

void Led::init() {
    // Configure GPIO and set it to an off state while PWM is prepared.
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, lowActive ? 1 : 0);

    ensure_timer_initialized();

    ledc_channel_config_t channelConf = {};
    channelConf.gpio_num = gpio;
    channelConf.speed_mode = PWM_MODE;
    channelConf.channel = channel;
    channelConf.intr_type = LEDC_INTR_DISABLE;
    channelConf.timer_sel = PWM_TIMER;
    channelConf.duty = 0;
    channelConf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channelConf));

    apply_off();

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
        mode = Mode::ON;
        ++stateVersion;
    }
    cv.notify_all();
}

void Led::set_off() {
    {
        const std::scoped_lock lock(mtx);
        mode = Mode::OFF;
        ++stateVersion;
    }
    cv.notify_all();
}

void Led::set_blink(std::chrono::milliseconds interval, std::optional<size_t> count) {
    {
        const std::scoped_lock lock(mtx);
        blinkInterval = interval.count() > 0 ? interval : std::chrono::milliseconds{1};
        blinkCount = count;
        mode = Mode::BLINK;
        ++stateVersion;
    }
    cv.notify_all();
}

void Led::set_on_duty_cycle(uint8_t dutyPercent) {
    {
        const std::scoped_lock lock(mtx);
        onDutyPercent = std::clamp<uint8_t>(dutyPercent, 0, 100);
        ++stateVersion;
    }
    cv.notify_all();
}

void Led::run(std::stop_token st) {
    auto waitForChange = [this, &st](Mode expected, uint64_t version) {
        std::unique_lock lock(mtx);
        cv.wait(lock, st, [this, expected, version] { return mode != expected || stateVersion != version; });
    };

    while (!st.stop_requested()) {
        // Get the current mode
        Mode current{Mode::OFF};
        uint64_t version{0};
        {
            const std::scoped_lock lock(mtx);
            current = mode;
            version = stateVersion;
        }

        // If off
        if (current == Mode::OFF) {
            apply_off();
            waitForChange(Mode::OFF, version);
            continue;
        }

        // If on
        if (current == Mode::ON) {
            apply_on_duty();
            waitForChange(Mode::ON, version);
            continue;
        }

        // Blink mode: toggle with interruptable waits so we react instantly
        // to mode changes, duty changes, or stop requests.
        bool level = false;
        for (;;) {
            if (st.stop_requested()) {
                return;
            }

            Mode modeSnapshot{Mode::OFF};
            uint64_t versionSnapshot{0};
            std::chrono::milliseconds intervalSnapshot{0};
            std::optional<size_t> countSnapshot{0};
            {
                const std::scoped_lock lock(mtx);
                modeSnapshot = mode;
                versionSnapshot = stateVersion;
                intervalSnapshot = blinkInterval;
                countSnapshot = blinkCount;
            }

            if (modeSnapshot != Mode::BLINK) {
                break;
            }

            if (countSnapshot && *countSnapshot == 0) {
                {
                    const std::scoped_lock lock(mtx);
                    mode = Mode::OFF;
                    ++stateVersion;
                }
                cv.notify_all();
                break;
            }

            level = !level;
            if (level) {
                apply_on_duty();
            } else {
                apply_off();
                if (countSnapshot) {
                    const std::scoped_lock lock(mtx);
                    if (blinkCount && *blinkCount > 0) {
                        *blinkCount = *blinkCount - 1;
                    }
                }
            }

            std::unique_lock lock(mtx);
            bool changed = cv.wait_for(lock, st, intervalSnapshot, [this, versionSnapshot] { return mode != Mode::BLINK || stateVersion != versionSnapshot; });
            lock.unlock();

            if (st.stop_requested()) {
                return;
            }

            if (changed) {
                Mode updatedMode;
                {
                    const std::scoped_lock guard(mtx);
                    updatedMode = mode;
                }
                if (updatedMode != Mode::BLINK) {
                    break;
                }

                if (level) {
                    apply_on_duty();
                } else {
                    apply_off();
                }
            }
        }
    }
}

void Led::apply_off() {
    const uint32_t idleLevel = lowActive ? 1u : 0u;
    ESP_ERROR_CHECK(ledc_stop(PWM_MODE, channel, idleLevel));
}

void Led::apply_on_duty() {
    apply_duty(compute_on_duty_ticks());
}

void Led::apply_duty(uint32_t duty) {
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, channel));
}

uint32_t Led::compute_on_duty_ticks() const {
    const uint8_t clampedPercent = std::clamp<uint8_t>(onDutyPercent, 0, 100);
    const uint32_t dutyTicks = (max_duty() * clampedPercent) / 100;
    if (lowActive) {
        return max_duty() - dutyTicks;
    }
    return dutyTicks;
}

uint32_t Led::max_duty() {
    return (1u << static_cast<uint32_t>(PWM_RESOLUTION)) - 1u;
}

void Led::ensure_timer_initialized() {
    std::call_once(g_timerInitFlag, []() {
        ledc_timer_config_t timerConf = {};
        timerConf.speed_mode = PWM_MODE;
        timerConf.duty_resolution = PWM_RESOLUTION;
        timerConf.timer_num = PWM_TIMER;
        timerConf.freq_hz = PWM_FREQUENCY_HZ;
        timerConf.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&timerConf));
    });
}
} // namespace actuators
