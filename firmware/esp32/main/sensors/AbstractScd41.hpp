#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace sensors {
/**
 * One SCD41 measurement converted to engineering units.
 */
struct measurement_t {
    /** Carbon dioxide concentration in parts per million. */
    uint16_t co2{};
    /** Air temperature in degrees Celsius. */
    double temp{};
    /** Relative humidity in percent RH. */
    double hum{};
} __attribute__((aligned(32)));

/**
 * Interface for real and mock SCD41 implementations.
 */
class AbstractScd41 {
  public:
    AbstractScd41() = default;
    AbstractScd41(AbstractScd41&&) = default;
    AbstractScd41(const AbstractScd41&) = default;
    AbstractScd41& operator=(AbstractScd41&&) = default;
    AbstractScd41& operator=(const AbstractScd41&) = default;
    virtual ~AbstractScd41() = default;

    /** Initializes the sensor backend for the current wake cycle. */
    [[nodiscard]] virtual bool init() const = 0;

    /** Reads the last completed measurement from the sensor. */
    [[nodiscard]] virtual std::optional<measurement_t> read_measurement() const = 0;
    /** Triggers a single-shot measurement and optionally injects ambient pressure compensation in hPa. */
    [[nodiscard]] virtual std::optional<measurement_t> read_single_shot(std::optional<uint16_t> ambientPressureHpa) const = 0;
    /** Returns true when the sensor reports that fresh data is ready. */
    [[nodiscard]] virtual bool get_data_ready_status() const = 0;
    /** Starts periodic SCD41 measurements. */
    [[nodiscard]] virtual bool start_periodic_measurement() const = 0;
    /** Stops periodic SCD41 measurements and returns the device to idle. */
    [[nodiscard]] virtual bool stop_periodic_measurement() const = 0;
    /** Restores factory settings on the sensor. */
    [[nodiscard]] virtual bool perform_factory_reset() const = 0;
    /** Reinitializes the sensor after a stop or reset. */
    [[nodiscard]] virtual bool reinit() const = 0;
    /** Runs the built-in self test. */
    [[nodiscard]] virtual bool perform_self_test() const = 0;
    /**
     * In case the self test fails, this function can be tried to recover the sensor by performing a factory reset followed by a reinit.
     * Returns true in case it worked and the self test at the end reported success.
     **/
    [[nodiscard]] bool try_recover() const;
    /** Returns the 48-bit SCD41 serial number. */
    [[nodiscard]] virtual uint64_t get_serial_number() const = 0;
    /** Programs the temperature offset in degrees Celsius. */
    virtual void set_temperature_offset(double offset) const = 0;
    /** Reads the programmed temperature offset in degrees Celsius. */
    [[nodiscard]] virtual double get_temperature_offset() const = 0;
    /** Programs the installation altitude in meters above sea level. */
    virtual void set_sensor_altitude(uint16_t altitude) const = 0;
    /** Reads the stored installation altitude in meters above sea level. */
    [[nodiscard]] virtual uint16_t get_sensor_altitude() const = 0;
    /** Injects ambient pressure in hPa for the next single-shot measurement. */
    virtual void set_ambient_pressure(uint16_t pressureHpa) const = 0;
    /** Writes the current configuration to non-volatile sensor memory. */
    virtual void persist_settings() const = 0;
    /** Returns true when the SCD41 responds on the configured I2C bus. */
    [[nodiscard]] virtual bool probe_device() const = 0;
};
} // namespace sensors
