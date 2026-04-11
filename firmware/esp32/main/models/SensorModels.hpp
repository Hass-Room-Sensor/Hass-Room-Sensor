#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

namespace models {
/**
 * Battery reading in user-facing units.
 */
struct BatteryReading {
    /** Instantaneous battery voltage in millivolts. */
    uint16_t millivolts{};
    /** Rounded battery percentage derived from the configured voltage window. */
    uint8_t percentage{};
};

/**
 * Environmental readings in physical units.
 */
struct EnvironmentalReadings {
    /** Air temperature in degrees Celsius. */
    std::optional<double> temperature_celsius{};
    /** Relative humidity in percent RH. */
    std::optional<double> humidity_percent{};
    /** Ambient pressure in hectopascal. */
    std::optional<double> pressure_hpa{};
    /** Carbon dioxide concentration in parts per million. */
    std::optional<uint16_t> co2_ppm{};
};

/**
 * Quantized environmental readings in the unit resolution expected by Zigbee attributes.
 */
struct QuantizedEnvironmentalReadings {
    /** Air temperature in 0.01 degrees Celsius. */
    std::optional<int16_t> temperature_centi_celsius{};
    /** Relative humidity in 0.01 percent RH. */
    std::optional<uint16_t> humidity_centi_percent{};
    /** Ambient pressure in 0.1 kPa, matching the Zigbee pressure cluster. */
    std::optional<int16_t> pressure_deci_kpa{};
    /** Carbon dioxide concentration in parts per million. */
    std::optional<uint16_t> co2_ppm{};

    [[nodiscard]] bool operator==(const QuantizedEnvironmentalReadings& other) const = default;
};

/**
 * Quantized battery reading in the units expected by the Zigbee power configuration cluster.
 */
struct QuantizedBatteryReading {
    /** Instantaneous battery voltage in millivolts. */
    uint16_t millivolts{};
    /** Integer battery percentage used for retained-state comparisons. */
    uint8_t percentage{};

    [[nodiscard]] bool operator==(const QuantizedBatteryReading& other) const = default;
};

/**
 * Combined sensor snapshot collected during one wake cycle.
 */
struct SensorSnapshot {
    /** Environmental values collected during the wake cycle. */
    EnvironmentalReadings environmental{};
    /** Battery data collected during the same wake cycle, if available. */
    std::optional<BatteryReading> battery{};
};

/**
 * Converts temperature to Zigbee's 0.01 °C resolution.
 */
[[nodiscard]] constexpr int16_t quantize_temperature_celsius(double temperature_celsius) {
    return static_cast<int16_t>(std::lround(temperature_celsius * 100.0));
}

/**
 * Converts humidity to Zigbee's 0.01 %RH resolution.
 */
[[nodiscard]] constexpr uint16_t quantize_humidity_percent(double humidity_percent) {
    return static_cast<uint16_t>(std::lround(humidity_percent * 100.0));
}

/**
 * Converts pressure in hPa to Zigbee pressure cluster resolution.
 *
 * The Zigbee pressure measurement cluster uses 0.1 kPa resolution which is numerically identical to hPa.
 */
[[nodiscard]] constexpr int16_t quantize_pressure_hpa(double pressure_hpa) {
    return static_cast<int16_t>(std::lround(pressure_hpa));
}

/**
 * Converts the physical sensor readings to quantized values for comparison and Zigbee reporting.
 */
[[nodiscard]] inline QuantizedEnvironmentalReadings quantize(const EnvironmentalReadings& readings) {
    return {
            .temperature_centi_celsius = readings.temperature_celsius ? std::make_optional(quantize_temperature_celsius(*readings.temperature_celsius)) : std::nullopt,
            .humidity_centi_percent = readings.humidity_percent ? std::make_optional(quantize_humidity_percent(*readings.humidity_percent)) : std::nullopt,
            .pressure_deci_kpa = readings.pressure_hpa ? std::make_optional(quantize_pressure_hpa(*readings.pressure_hpa)) : std::nullopt,
            .co2_ppm = readings.co2_ppm,
    };
}

/**
 * Converts a physical battery reading to quantized values used for retained-state comparisons.
 */
[[nodiscard]] constexpr QuantizedBatteryReading quantize(const BatteryReading& reading) {
    return {
            .millivolts = reading.millivolts,
            .percentage = reading.percentage,
    };
}
} // namespace models
