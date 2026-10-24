#pragma once

#include <chrono>

namespace app {
/** Maintains a periodic monotonic deadline without accumulating work-duration drift. */
class PeriodicDeadline {
  public:
    /** Clock used for periodic deadlines. */
    using Clock = std::chrono::steady_clock;
    /** Starts the schedule with its first deadline one interval after `start`. */
    PeriodicDeadline(Clock::time_point start, Clock::duration interval) : interval_(interval), next_(start + interval) {}

    /** Returns the next scheduled deadline. */
    [[nodiscard]] Clock::time_point next() const {
        return next_;
    }

    /** Advances by whole intervals until the next deadline lies after `now`. */
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
