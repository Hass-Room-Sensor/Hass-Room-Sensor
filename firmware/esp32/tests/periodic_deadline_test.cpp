#include "app/PeriodicDeadline.hpp"

#include <cassert>
#include <chrono>

using namespace std::chrono_literals;

/** Exercises the periodic deadline behavior. */
int main() {
    using Clock = app::PeriodicDeadline::Clock;
    const auto start = Clock::time_point{};
    app::PeriodicDeadline deadline{start, 5min};
    assert(deadline.next() == start + 5min);
    deadline.advance_past(start + 5min + 20s); // Long measurement or late scheduling.
    assert(deadline.next() == start + 10min);
    deadline.advance_past(start + 16min);
    assert(deadline.next() == start + 20min);
}
