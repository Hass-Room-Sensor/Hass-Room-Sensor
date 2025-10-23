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

class AbstractScd41 {
  public:
    AbstractScd41() = default;
    AbstractScd41(AbstractScd41&&) = default;
    AbstractScd41(const AbstractScd41&) = default;
    AbstractScd41& operator=(AbstractScd41&&) = default;
    AbstractScd41& operator=(const AbstractScd41&) = default;
    virtual ~AbstractScd41() = default;

    [[nodiscard]] virtual bool init() const = 0;

    [[nodiscard]] virtual std::optional<measurement_t> read_measurement() const = 0;
    [[nodiscard]] virtual bool get_data_ready_status() const = 0;
    [[nodiscard]] virtual bool start_periodic_measurement() const = 0;
    [[nodiscard]] virtual bool stop_periodic_measurement() const = 0;
    [[nodiscard]] virtual bool perform_factory_reset() const = 0;
    [[nodiscard]] virtual bool reinit() const = 0;
    [[nodiscard]] virtual bool perform_self_test() const = 0;
    /**
     * In case the self test fails, this function can be tried to recover the sensor by performing a factory reset followed by a reinit.
     * Returns true in case it worked and the self test at the end reported success.
     **/
    [[nodiscard]] bool try_recover() const;
    [[nodiscard]] virtual uint64_t get_serial_number() const = 0;
    virtual void set_temperature_offset(double offset) const = 0;
    [[nodiscard]] virtual double get_temperature_offset() const = 0;
    virtual void set_sensor_altitude(uint16_t altitude) const = 0;
    [[nodiscard]] virtual uint16_t get_sensor_altitude() const = 0;
    virtual void persist_settings() const = 0;
    [[nodiscard]] virtual bool probe_device() const = 0;
};
} // namespace sensors