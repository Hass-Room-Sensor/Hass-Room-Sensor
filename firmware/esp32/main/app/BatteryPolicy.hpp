#pragma once

#include <cstdint>

namespace app {
/** Determines when to sample and publish battery state during periodic sensor cycles. */
class BatteryPolicy {
  public:
    /** Number of five-minute wake cycles in one hour. */
    static constexpr uint32_t CYCLES_PER_HOUR = 60U / 5U;
    /** Number of five-minute wake cycles in one day. */
    static constexpr uint32_t CYCLES_PER_DAY = 24U * CYCLES_PER_HOUR;
    /** Minimum absolute percentage-point change that triggers a report. */
    static constexpr uint8_t REPORT_DELTA_PERCENT = 2;

    /** Returns true on startup and once per hour for a battery ADC sample. */
    [[nodiscard]] static constexpr bool should_sample(uint32_t cycle, bool startup) {
        return startup || cycle % CYCLES_PER_HOUR == 0;
    }

    /**
     * Returns true for startup, the first report, a percentage change of at least two points, or a
     * daily keepalive. Cycle values are compared against the cycle of the last successful report.
     */
    [[nodiscard]] static constexpr bool should_report(uint8_t currentPercent, uint8_t reportedPercent, uint32_t cycle, uint32_t lastReportCycle, bool hasReport, bool startup) {
        const int delta = static_cast<int>(currentPercent) - static_cast<int>(reportedPercent);
        return startup || !hasReport || delta >= REPORT_DELTA_PERCENT || delta <= -REPORT_DELTA_PERCENT || cycle - lastReportCycle >= CYCLES_PER_DAY;
    }
};
} // namespace app
