#pragma once

#include "models/SensorModels.hpp"

namespace models {
/** Combines one cycle of physical readings, keeping BME temperature/humidity as SCD fallback. */
[[nodiscard]] inline EnvironmentalReadings select_environmental_sources(const EnvironmentalReadings& bme690, const EnvironmentalReadings& scd41) {
    EnvironmentalReadings selected = bme690;
    if (scd41.temperature_celsius) {
        selected.temperature_celsius = scd41.temperature_celsius;
    }
    if (scd41.humidity_percent) {
        selected.humidity_percent = scd41.humidity_percent;
    }
    selected.co2_ppm = scd41.co2_ppm;
    return selected;
}
} // namespace models
