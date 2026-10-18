#pragma once

#include <cstdint>

namespace app {
class BatteryPolicy {
  public:
    static constexpr uint32_t CYCLES_PER_HOUR = 60U / 5U;
    static constexpr uint32_t CYCLES_PER_DAY = 24U * CYCLES_PER_HOUR;
    static constexpr uint8_t REPORT_DELTA_PERCENT = 2;

    [[nodiscard]] static constexpr bool should_sample(uint32_t cycle, bool startup) {
        return startup || cycle % CYCLES_PER_HOUR == 0;
    }

    [[nodiscard]] static constexpr bool should_report(uint8_t currentPercent, uint8_t reportedPercent, uint32_t cycle, uint32_t lastReportCycle, bool hasReport, bool startup) {
        const int delta = static_cast<int>(currentPercent) - static_cast<int>(reportedPercent);
        return startup || !hasReport || delta >= REPORT_DELTA_PERCENT || delta <= -REPORT_DELTA_PERCENT || cycle - lastReportCycle >= CYCLES_PER_DAY;
    }
};
} // namespace app
