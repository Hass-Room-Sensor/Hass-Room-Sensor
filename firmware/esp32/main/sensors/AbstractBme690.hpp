#pragma once

#include <optional>

namespace sensors {
/**
 * Minimal BME690 reading used by the firmware.
 */
struct bme690_measurement_t {
    /** Air temperature in degrees Celsius. */
    double temp_celsius{};
    /** Relative humidity in percent RH. */
    double humidity_percent{};
    /** Ambient pressure in hectopascal. */
    double pressure_hpa{};
} __attribute__((aligned(32)));

/**
 * Interface for real and mock BME690 implementations.
 */
class AbstractBme690 {
  public:
    AbstractBme690() = default;
    AbstractBme690(AbstractBme690&&) = default;
    AbstractBme690(const AbstractBme690&) = default;
    AbstractBme690& operator=(AbstractBme690&&) = default;
    AbstractBme690& operator=(const AbstractBme690&) = default;
    virtual ~AbstractBme690() = default;

    /** Initializes the sensor backend for the current wake cycle. */
    [[nodiscard]] virtual bool init() = 0;
    /** Checks whether the sensor is present on the configured bus. */
    [[nodiscard]] virtual bool probe_device() = 0;
    /** Performs one environmental measurement and returns the converted values. */
    [[nodiscard]] virtual std::optional<bme690_measurement_t> read_measurement() = 0;
};
} // namespace sensors
