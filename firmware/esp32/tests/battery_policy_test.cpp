#include "app/BatteryPolicy.hpp"

#include <cassert>

int main() {
    using app::BatteryPolicy;
    assert(BatteryPolicy::should_sample(0, true));
    assert(!BatteryPolicy::should_sample(1, false));
    assert(!BatteryPolicy::should_sample(11, false));
    assert(BatteryPolicy::should_sample(12, false));
    assert(!BatteryPolicy::should_report(51, 50, 12, 0, true, false));
    assert(BatteryPolicy::should_report(52, 50, 12, 0, true, false));
    assert(BatteryPolicy::should_report(48, 50, 12, 0, true, false));
    assert(BatteryPolicy::should_report(50, 50, 288, 0, true, false));
    assert(BatteryPolicy::should_report(50, 50, 0, 0, false, true));
}
