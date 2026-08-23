#pragma once

#include "FreeRTOSConfig.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "sensors/AbstractScd41.hpp"
#include "soc/gpio_num.h"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace sensors {
/**
 * Sensirion SCD41 wrapper.
 *
 * Specification:
 * https://sensirion.com/products/catalog/SCD41
 */
class Scd41 : public AbstractScd41 {
  private:
    /** Fixed I2C address used by all SCD4x sensors. */
    static constexpr uint8_t DEVICE_ADDR = 0x62;
    /** Upper bound used for I2C transactions. */
    static constexpr std::chrono::milliseconds WRITE_READ_TIMEOUT{2500};

    static const char* TAG;

    /** I2C SDA pin used by the sensor bus. */
    gpio_num_t sda;
    /** I2C SCL pin used by the sensor bus. */
    gpio_num_t scl;

    /** IDF I2C bus configuration. */
    i2c_master_bus_config_t busConf{};
    /** IDF I2C master bus handle. */
    i2c_master_bus_handle_t bus{};
    /** IDF I2C device configuration. */
    i2c_device_config_t devConf{};
    /** IDF I2C device handle for the SCD41. */
    i2c_master_dev_handle_t dev{};

    /** Scratch buffer reused for short command payloads. */
    std::array<uint8_t, 2> buffer;
    /** True once the sensor answered its probe and completed initialization. */
    mutable bool initialized{false};

  public:
    /** Creates the SCD41 wrapper on the given I2C pins. */
    Scd41(gpio_num_t sda, gpio_num_t scl);
    Scd41(Scd41&&) = default;
    Scd41(const Scd41&) = default;
    Scd41& operator=(Scd41&&) = default;
    Scd41& operator=(const Scd41&) = default;
    ~Scd41() override;

    /** Prepares the SCD41 for idle single-shot measurements. */
    [[nodiscard]] bool init() const override;

    /** Reads the last completed measurement from the sensor. */
    [[nodiscard]] std::optional<measurement_t> read_measurement() const override;
    /** Triggers a single-shot measurement and applies optional pressure compensation first. */
    [[nodiscard]] std::optional<measurement_t> read_single_shot(std::optional<uint16_t> ambientPressureHpa) const override;
    /** Returns true if the sensor reports that new data is available. */
    [[nodiscard]] bool get_data_ready_status() const override;
    /** Starts the continuous five-second measurement mode. */
    [[nodiscard]] bool start_periodic_measurement() const override;
    /** Stops the continuous measurement mode and returns the device to idle. */
    [[nodiscard]] bool stop_periodic_measurement() const override;
    /** Restores factory settings on the SCD41. */
    [[nodiscard]] bool perform_factory_reset() const override;
    /** Reinitializes the SCD41 after a stop or reset. */
    [[nodiscard]] bool reinit() const override;
    /** Runs the built-in SCD41 self test. */
    [[nodiscard]] bool perform_self_test() const override;
    /** Returns the unique 48-bit SCD41 serial number. */
    [[nodiscard]] uint64_t get_serial_number() const override;
    /** Programs the temperature offset in degrees Celsius. */
    void set_temperature_offset(double offset) const override;
    /** Reads the configured temperature offset in degrees Celsius. */
    [[nodiscard]] double get_temperature_offset() const override;
    /** Programs the sensor altitude in meters above sea level. */
    void set_sensor_altitude(uint16_t altitude) const override;
    /** Reads the configured sensor altitude in meters above sea level. */
    [[nodiscard]] uint16_t get_sensor_altitude() const override;
    /** Injects the current ambient pressure in hPa for the next single-shot measurement. */
    void set_ambient_pressure(uint16_t pressureHpa) const override;
    /** Persists the current sensor settings to on-chip EEPROM. */
    void persist_settings() const override;
    /** Returns true when the sensor acknowledges its I2C address. */
    [[nodiscard]] bool probe_device() const override;

    /** Issues the measure-single-shot command without reading back the result. */
    [[nodiscard]] bool measure_single_shot() const;

  private:
    /** Calculates the SCD4x CRC over one 16-bit data word. */
    static uint8_t calc_crc(const std::span<uint8_t> data);

    /** Converts one or more 16-bit words into SCD4x wire format with CRC bytes. */
    static void transform_to_send_data(const std::span<uint16_t> input, std::span<uint8_t> output, size_t outputOffset);
    /** Validates received CRC bytes and converts the payload back into 16-bit words. */
    static bool validate_transform_received_data(const std::span<uint8_t> input, std::span<uint16_t> output);

    /** Sends a command without payload. */
    [[nodiscard]] bool write(uint16_t reg, std::chrono::milliseconds timeout) const;
    /** Sends a command with one 16-bit payload word. */
    [[nodiscard]] bool write(uint16_t reg, uint16_t payload, std::chrono::milliseconds timeout) const;
    /** Sends a command and reads back one or more 16-bit response words. */
    [[nodiscard]] bool write_read(uint16_t reg, std::span<uint16_t> response, std::chrono::milliseconds timeout) const;

    /** Converts a raw SCD41 response into engineering units. */
    static measurement_t convert_measurement(std::span<uint16_t, 3> data);
    /** Converts raw temperature data into degrees Celsius. */
    static double convert_measurement_to_temp(uint16_t mTemp);
    /** Converts raw humidity data into percent RH. */
    static double convert_measurement_to_hum(uint16_t mHum);
    /** Converts raw CO2 data into ppm. */
    static uint16_t convert_measurement_to_co2(uint16_t mCo2);
};
} // namespace sensors
