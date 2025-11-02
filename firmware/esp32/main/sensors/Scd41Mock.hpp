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

    [[nodiscard]] bool init() const override;

    [[nodiscard]] std::optional<measurement_t> read_measurement() const override;
    [[nodiscard]] bool get_data_ready_status() const override;
    [[nodiscard]] bool start_periodic_measurement() const override;
    [[nodiscard]] bool stop_periodic_measurement() const override;
    [[nodiscard]] bool perform_factory_reset() const override;
    [[nodiscard]] bool reinit() const override;
    [[nodiscard]] bool perform_self_test() const override;
    [[nodiscard]] uint64_t get_serial_number() const override;
    void set_temperature_offset(double offset) const override;
    [[nodiscard]] double get_temperature_offset() const override;
    void set_sensor_altitude(uint16_t altitude) const override;
    [[nodiscard]] uint16_t get_sensor_altitude() const override;
    void persist_settings() const override;
    [[nodiscard]] bool probe_device() const override;
};
} // namespace sensors