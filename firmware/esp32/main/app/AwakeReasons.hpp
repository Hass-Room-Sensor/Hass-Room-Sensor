#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace app {
/** Reasons that independently keep the application awake and block automatic sleep. */
enum class AwakeReason : uint8_t { MAINTENANCE, BUTTON, COMMISSIONING, RECOVERY, OTA, IDENTIFY, COUNT };

/** Tracks independent awake owners; the caller must serialize access. Releasing one preserves all others. */
class AwakeReasons {
  public:
    /** Monotonic clock used for awake-window deadlines. */
    using Clock = std::chrono::steady_clock;
    /** Startup maintenance duration; timer-based deep-sleep wakes skip the USB window. */
    static constexpr std::chrono::seconds startup_maintenance(bool timerWake) {
        return timerWake ? std::chrono::seconds{0} : std::chrono::seconds{60};
    }
    /** Maintenance window opened by a short BOOT-button press. */
    static constexpr std::chrono::minutes short_press_maintenance{5};
    /** Activates or releases one awake reason without changing other owners. */
    void set(AwakeReason reason, bool active) {
        const auto bit = uint32_t{1} << static_cast<uint8_t>(reason);
        if (active) {
            reasons_ |= bit;
        } else {
            reasons_ &= ~bit;
            deadlines_.at(static_cast<size_t>(reason)).reset();
        }
    }
    /** Returns whether the specified awake reason is currently active. */
    [[nodiscard]] bool active(AwakeReason reason) const {
        return (reasons_ & (uint32_t{1} << static_cast<uint8_t>(reason))) != 0;
    }
    /** Returns true while at least one awake reason remains active. */
    [[nodiscard]] bool any() const {
        return reasons_ != 0;
    }
    /** Opens a maintenance awake window that expires at the supplied monotonic deadline. */
    void set_maintenance_until(Clock::time_point deadline) {
        set_until(AwakeReason::MAINTENANCE, deadline);
    }
    /** Activates an awake reason until the supplied monotonic deadline. */
    void set_until(AwakeReason reason, Clock::time_point deadline) {
        deadlines_.at(static_cast<size_t>(reason)) = deadline;
        set(reason, true);
    }
    /** Returns the remaining timed window, or no value for an untimed or inactive reason. */
    [[nodiscard]] std::optional<Clock::duration> remaining(AwakeReason reason, Clock::time_point now) const {
        const auto deadline = deadlines_.at(static_cast<size_t>(reason));
        return deadline ? std::make_optional(std::max(Clock::duration::zero(), *deadline - now)) : std::nullopt;
    }
    /** Releases timed reasons whose deadlines have passed, preserving all other owners. */
    void expire(Clock::time_point now) {
        for (size_t index = 0; index < deadlines_.size(); ++index) {
            const auto deadline = deadlines_.at(index);
            if (deadline && now >= *deadline) {
                set(static_cast<AwakeReason>(index), false);
            }
        }
    }

  private:
    uint32_t reasons_{0};
    std::array<std::optional<Clock::time_point>, static_cast<size_t>(AwakeReason::COUNT)> deadlines_{};
};
} // namespace app
