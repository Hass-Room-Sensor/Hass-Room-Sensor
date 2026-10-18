#include "models/EnvironmentalSource.hpp"

#include <cassert>

/** Exercises the environmental source behavior. */
int main() {
    const models::EnvironmentalReadings bme{.temperature_celsius = 21.5, .humidity_percent = 38.0, .pressure_hpa = 1004.0};
    const models::EnvironmentalReadings scd{.temperature_celsius = 22.0, .humidity_percent = 41.0, .co2_ppm = 812};

    const auto both = models::select_environmental_sources(bme, scd);
    assert(both.temperature_celsius == 22.0);
    assert(both.humidity_percent == 41.0);
    assert(both.pressure_hpa == 1004.0);
    assert(both.co2_ppm == 812);

    // Both an SCD41 initialization failure and a failed single-shot measurement
    // produce no SCD sample, so this cycle must retain the BME690 values.
    const auto scdInitializationFailure = models::select_environmental_sources(bme, {});
    assert(scdInitializationFailure.temperature_celsius == 21.5);
    assert(scdInitializationFailure.humidity_percent == 38.0);
    assert(scdInitializationFailure.pressure_hpa == 1004.0);
    assert(!scdInitializationFailure.co2_ppm);

    const auto scdMeasurementFailure = models::select_environmental_sources(bme, {});
    assert(scdMeasurementFailure.temperature_celsius == 21.5);
    assert(scdMeasurementFailure.humidity_percent == 38.0);
    assert(scdMeasurementFailure.pressure_hpa == 1004.0);
    assert(!scdMeasurementFailure.co2_ppm);

    const auto bothFailed = models::select_environmental_sources({}, {});
    assert(!bothFailed.temperature_celsius && !bothFailed.humidity_percent && !bothFailed.pressure_hpa && !bothFailed.co2_ppm);
}
