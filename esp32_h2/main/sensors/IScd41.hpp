#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace sensors {
struct measurement_t {
    uint16_t co2{};
    double temp{};
    double hum{};
} __attribute__((aligned(32)));

class IScd41 {
  public:
    IScd41() = default;
    IScd41(IScd41&&) = default;
    IScd41(const IScd41&) = default;
    IScd41& operator=(IScd41&&) = default;
    IScd41& operator=(const IScd41&) = default;
    virtual ~IScd41() = default;

    [[nodiscard]] virtual bool init() const = 0;

    [[nodiscard]] virtual std::optional<measurement_t> read_measurement() const = 0;
    [[nodiscard]] virtual bool get_data_ready_status() const = 0;
    [[nodiscard]] virtual bool start_periodic_measurement() const = 0;
    [[nodiscard]] virtual bool stop_periodic_measurement() const = 0;
    [[nodiscard]] virtual bool perform_factory_reset() const = 0;
    [[nodiscard]] virtual bool reinit() const = 0;
    [[nodiscard]] virtual bool perform_self_test() const = 0;
    [[nodiscard]] virtual uint64_t get_serial_number() const = 0;
    virtual void set_temperature_offset(double offset) const = 0;
    [[nodiscard]] virtual double get_temperature_offset() const = 0;
    virtual void set_sensor_altitude(uint16_t altitude) const = 0;
    [[nodiscard]] virtual uint16_t get_sensor_altitude() const = 0;
    virtual void persist_settings() const = 0;
    [[nodiscard]] virtual bool probe_device() const = 0;
};
} // namespace sensors