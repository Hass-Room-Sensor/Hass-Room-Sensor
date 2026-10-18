#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>

namespace app {
/** Deterministic BOOT-button gesture recognizer with monotonic input debounce. */
class ButtonGesture {
  public:
    /** Debounced events emitted for a short press or a ten-second factory-reset hold. */
    enum class Event : std::uint8_t { NONE, PRESSED, SHORT_PRESS, FACTORY_RESET };
    /** Monotonic clock used to debounce inputs and measure press duration. */
    using Clock = std::chrono::steady_clock;
    /** Required stable time before accepting a raw input transition. */
    static constexpr std::chrono::milliseconds DEBOUNCE_INTERVAL{50};
    /** Continuous debounced hold required to request a factory reset. */
    static constexpr std::chrono::seconds FACTORY_RESET_HOLD_TIME{10};

    /**
     * Creates a recognizer and ignores an already-held button until it is released, if requested.
     *
     * @param pressedAtStartup Whether the input is active when the recognizer is created.
     */
    explicit ButtonGesture(bool pressedAtStartup = false) : ignoreUntilRelease_(pressedAtStartup), pressed_(pressedAtStartup), candidate_(pressedAtStartup), candidateSet_(true) {}

    /**
     * Feeds the current raw button level and returns any newly recognized event.
     *
     * Input transitions must remain stable for 50 ms. A stable press emits PRESSED. A ten-second
     * hold emits FACTORY_RESET once, including when a delayed task first observes its release.
     * Releasing a shorter press emits SHORT_PRESS.
     */
    Event update(bool pressed, Clock::time_point now) {
        if (!candidateSet_ || pressed != candidate_) {
            candidate_ = pressed;
            candidateSince_ = now;
            candidateSet_ = true;
            return Event::NONE;
        }
        if (pressed != pressed_ && now - candidateSince_ < DEBOUNCE_INTERVAL) {
            return Event::NONE;
        }

        if (ignoreUntilRelease_) {
            if (!pressed) {
                ignoreUntilRelease_ = false;
            }
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
            if (!resetEmitted_) {
                resetEmitted_ = candidateSince_ - pressedAt_ >= FACTORY_RESET_HOLD_TIME;
                return resetEmitted_ ? Event::FACTORY_RESET : Event::SHORT_PRESS;
            }
            return Event::NONE;
        }

        if (pressed_ && !resetEmitted_ && now - pressedAt_ >= FACTORY_RESET_HOLD_TIME) {
            resetEmitted_ = true;
            return Event::FACTORY_RESET;
        }
        return Event::NONE;
    }

    /** Returns the debounced pressed state, independent of whether an event was emitted. */
    [[nodiscard]] bool is_pressed() const {
        return pressed_;
    }

    /** Returns true while a fresh press or its release debounce needs to hold the CPU awake. */
    [[nodiscard]] bool is_processing() const {
        return !ignoreUntilRelease_ && (candidate_ || pressed_);
    }

    /** Returns the delay until another sample is required, or no value when an edge is needed. */
    [[nodiscard]] std::optional<Clock::duration> next_update_delay(Clock::time_point now) const {
        if (candidate_ != pressed_) {
            return std::max(Clock::duration::zero(), DEBOUNCE_INTERVAL - (now - candidateSince_));
        }
        if (pressed_ && !ignoreUntilRelease_ && !resetEmitted_) {
            return std::max(Clock::duration::zero(), FACTORY_RESET_HOLD_TIME - (now - pressedAt_));
        }
        return std::nullopt;
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
