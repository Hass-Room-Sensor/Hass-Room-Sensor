#pragma once

#include "sensors/AbstractBme690.hpp"

namespace sensors {
/**
 * Fixed-value mock used when the board is assembled without a BME690.
 */
class Bme690Mock : public AbstractBme690 {
  public:
    Bme690Mock() = default;
    Bme690Mock(Bme690Mock&&) = default;
    Bme690Mock(const Bme690Mock&) = default;
    Bme690Mock& operator=(Bme690Mock&&) = default;
    Bme690Mock& operator=(const Bme690Mock&) = default;
    ~Bme690Mock() override = default;

    /** Pretends to initialize the mock backend successfully. */
    [[nodiscard]] bool init() override;
    /** Always reports that the mock device is reachable. */
    [[nodiscard]] bool probe_device() override;
    /** Returns a fixed mock environmental sample. */
    [[nodiscard]] std::optional<bme690_measurement_t> read_measurement() override;
};
} // namespace sensors
