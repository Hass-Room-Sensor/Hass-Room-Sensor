#pragma once

#include <chrono>

namespace app {
class PeriodicDeadline {
  public:
    using Clock = std::chrono::steady_clock;
    PeriodicDeadline(Clock::time_point start, Clock::duration interval) : interval_(interval), next_(start + interval) {}

    [[nodiscard]] Clock::time_point next() const {
        return next_;
    }

    void advance_past(Clock::time_point now) {
        do {
            next_ += interval_;
        } while (next_ <= now);
    }

  private:
    Clock::duration interval_;
    Clock::time_point next_;
};
} // namespace app
