#pragma once

#include "sensors/AbstractScd41.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace sensors {
/**
 * Mock without hardware for the SCD41.
 **/
class Scd41Mock : public AbstractScd41 {
  public:
    Scd41Mock() = default;
    Scd41Mock(Scd41Mock&&) = default;
    Scd41Mock(const Scd41Mock&) = default;
    Scd41Mock& operator=(Scd41Mock&&) = default;
    Scd41Mock& operator=(const Scd41Mock&) = default;
    ~Scd41Mock() override = default;

    /** Pretends to initialize the mock backend successfully. */
    [[nodiscard]] bool init() const override;

    /** Returns a fixed mock measurement. */
    [[nodiscard]] std::optional<measurement_t> read_measurement() const override;
    /** Returns the same fixed mock measurement as the non-triggered read path. */
    [[nodiscard]] std::optional<measurement_t> read_single_shot(std::optional<uint16_t> ambientPressureHpa) const override;
    /** Always reports mock data as ready. */
    [[nodiscard]] bool get_data_ready_status() const override;
    /** No-op for the mock backend. */
    [[nodiscard]] bool start_periodic_measurement() const override;
    /** No-op for the mock backend. */
    [[nodiscard]] bool stop_periodic_measurement() const override;
    /** No-op for the mock backend. */
    [[nodiscard]] bool perform_factory_reset() const override;
    /** No-op for the mock backend. */
    [[nodiscard]] bool reinit() const override;
    /** Always reports a successful self test. */
    [[nodiscard]] bool perform_self_test() const override;
    /** Returns a deterministic mock serial number. */
    [[nodiscard]] uint64_t get_serial_number() const override;
    /** No-op for the mock backend. */
    void set_temperature_offset(double offset) const override;
    /** Returns a deterministic mock offset. */
    [[nodiscard]] double get_temperature_offset() const override;
    /** No-op for the mock backend. */
    void set_sensor_altitude(uint16_t altitude) const override;
    /** Returns a deterministic mock altitude. */
    [[nodiscard]] uint16_t get_sensor_altitude() const override;
    /** No-op for the mock backend. */
    void set_ambient_pressure(uint16_t pressureHpa) const override;
    /** No-op for the mock backend. */
    void persist_settings() const override;
    /** Always reports that the mock device is reachable. */
    [[nodiscard]] bool probe_device() const override;
};
} // namespace sensors
