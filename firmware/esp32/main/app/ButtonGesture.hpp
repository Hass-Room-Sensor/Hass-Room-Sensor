#pragma once

#include <chrono>
#include <cstdint>

namespace app {
/** Deterministic BOOT-button gesture recognizer with monotonic input debounce. */
class ButtonGesture {
  public:
    enum class Event : std::uint8_t { NONE, PRESSED, SHORT_PRESS, FACTORY_RESET };
    using Clock = std::chrono::steady_clock;

    explicit ButtonGesture(bool pressedAtStartup = false) : ignoreUntilRelease_(pressedAtStartup), pressed_(pressedAtStartup), candidate_(pressedAtStartup), candidateSet_(true) {}

    Event update(bool pressed, Clock::time_point now) {
        if (!candidateSet_ || pressed != candidate_) {
            candidate_ = pressed;
            candidateSince_ = now;
            candidateSet_ = true;
            return Event::NONE;
        }
        if (pressed != pressed_ && now - candidateSince_ < std::chrono::milliseconds{50})
            return Event::NONE;

        if (ignoreUntilRelease_) {
            if (!pressed)
                ignoreUntilRelease_ = false;
            pressed_ = pressed;
            return Event::NONE;
        }

        if (pressed && !pressed_) {
            pressed_ = true;
            pressedAt_ = now;
            resetEmitted_ = false;
            return Event::PRESSED;
        }

        if (!pressed && pressed_) {
            pressed_ = false;
            if (!resetEmitted_)
                return Event::SHORT_PRESS;
            return Event::NONE;
        }

        if (pressed_ && !resetEmitted_ && now - pressedAt_ >= std::chrono::seconds{10}) {
            resetEmitted_ = true;
            return Event::FACTORY_RESET;
        }
        return Event::NONE;
    }

    [[nodiscard]] bool is_pressed() const {
        return pressed_;
    }

  private:
    bool ignoreUntilRelease_{false};
    bool pressed_{false};
    bool resetEmitted_{false};
    bool candidate_{false};
    bool candidateSet_{false};
    Clock::time_point candidateSince_{};
    Clock::time_point pressedAt_{};
};
} // namespace app
