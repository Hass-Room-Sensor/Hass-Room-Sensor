#include "app/AwakeReasons.hpp"

#include <cassert>
#include <chrono>

int main() {
    app::AwakeReasons reasons;
    reasons.set(app::AwakeReason::MAINTENANCE, true);
    reasons.set(app::AwakeReason::OTA, true);
    reasons.set(app::AwakeReason::MAINTENANCE, false);
    assert(reasons.active(app::AwakeReason::OTA));
    assert(reasons.any());
    reasons.set(app::AwakeReason::OTA, false);
    assert(!reasons.any());

    const auto now = app::AwakeReasons::Clock::time_point{};
    reasons.set_maintenance_until(now + std::chrono::seconds{60});
    reasons.set(app::AwakeReason::COMMISSIONING, true);
    reasons.set(app::AwakeReason::IDENTIFY, true);
    reasons.expire(now + std::chrono::seconds{60});
    assert(!reasons.active(app::AwakeReason::MAINTENANCE));
    assert(reasons.active(app::AwakeReason::COMMISSIONING));
    assert(reasons.active(app::AwakeReason::IDENTIFY));
    assert(reasons.any());
    reasons.set(app::AwakeReason::IDENTIFY, false);
    assert(reasons.active(app::AwakeReason::COMMISSIONING));
    assert(reasons.any());
    reasons.set(app::AwakeReason::COMMISSIONING, false);
    assert(!reasons.any());

    assert(app::AwakeReasons::startup_maintenance(false) == std::chrono::seconds{60});
    assert(app::AwakeReasons::startup_maintenance(true) == std::chrono::seconds{0});
    assert(app::AwakeReasons::short_press_maintenance == std::chrono::minutes{5});
}
