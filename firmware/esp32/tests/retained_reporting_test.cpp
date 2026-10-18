#include "app/RetainedState.hpp"
#include <cassert>

/** Verifies periodic temperature/humidity reports and change-only pressure/CO2 reports. */
int main() {
    app::RetainedState::reset();
    app::RetainedState state;
    const models::QuantizedEnvironmentalReadings readings{.temperature_centi_celsius = 2150, .humidity_centi_percent = 4000, .pressure_deci_kpa = 1004, .co2_ppm = 800};
    assert(state.environment_to_report(readings) == readings);
    state.mark_environment_reported(readings);
    state.mark_startup_report_completed();
    const auto unchanged = state.environment_to_report(readings);
    assert(unchanged.temperature_centi_celsius == 2150);
    assert(unchanged.humidity_centi_percent == 4000);
    assert(!unchanged.pressure_deci_kpa && !unchanged.co2_ppm);
    auto changed = readings;
    changed.pressure_deci_kpa = 1005;
    changed.co2_ppm = 850;
    assert(state.environment_to_report(changed) == changed);
    // Without a successful report, a changed value stays eligible for the next attempt.
    assert(state.environment_to_report(changed).co2_ppm == 850);
    state.mark_environment_reported(changed);
    assert(!state.environment_to_report(changed).co2_ppm);
    changed.co2_ppm.reset();
    assert(!state.environment_to_report(changed).co2_ppm);
}
