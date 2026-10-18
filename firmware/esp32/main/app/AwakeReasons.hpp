#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace app {
enum class AwakeReason : uint8_t { MAINTENANCE, BUTTON, COMMISSIONING, RECOVERY, OTA, IDENTIFY };

/** Tracks independent owners of an awake window. Releasing one owner preserves all others. */
class AwakeReasons {
  public:
    using Clock = std::chrono::steady_clock;
    static constexpr std::chrono::seconds startup_maintenance(bool timerWake) {
        return timerWake ? std::chrono::seconds{0} : std::chrono::seconds{60};
    }
    static constexpr std::chrono::minutes short_press_maintenance{5};
    void set(AwakeReason reason, bool active) {
        const auto bit = uint32_t{1} << static_cast<uint8_t>(reason);
        if (active)
            reasons_ |= bit;
        else {
            reasons_ &= ~bit;
            if (reason == AwakeReason::MAINTENANCE)
                maintenanceDeadline_.reset();
        }
    }
    [[nodiscard]] bool active(AwakeReason reason) const {
        return (reasons_ & (uint32_t{1} << static_cast<uint8_t>(reason))) != 0;
    }
    [[nodiscard]] bool any() const {
        return reasons_ != 0;
    }
    void set_maintenance_until(Clock::time_point deadline) {
        maintenanceDeadline_ = deadline;
        set(AwakeReason::MAINTENANCE, true);
    }
    void expire(Clock::time_point now) {
        if (maintenanceDeadline_ && now >= *maintenanceDeadline_) {
            maintenanceDeadline_.reset();
            set(AwakeReason::MAINTENANCE, false);
        }
    }

  private:
    uint32_t reasons_{0};
    std::optional<Clock::time_point> maintenanceDeadline_{};
};
} // namespace app
