#pragma once

#include "FreeRTOSConfig.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "sensors/IScd41.hpp"
#include "soc/gpio_num.h"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace sensors {
/**
 * Specification: https://sensirion.com/products/catalog/SCD41
 **/
class Scd41 : public IScd41 {
  private:
    static constexpr uint8_t DEVICE_ADDR = 0x62;
    static constexpr std::chrono::milliseconds WRITE_READ_TIMEOUT{2500};

    static const char* TAG;

    gpio_num_t sda;
    gpio_num_t scl;

    i2c_master_bus_config_t busConf{};
    i2c_master_bus_handle_t bus{};
    i2c_device_config_t devConf{};
    i2c_master_dev_handle_t dev{};

    std::array<uint8_t, 2> buffer;

  public:
    Scd41(gpio_num_t sda, gpio_num_t scl);
    Scd41(Scd41&&) = default;
    Scd41(const Scd41&) = default;
    Scd41& operator=(Scd41&&) = default;
    Scd41& operator=(const Scd41&) = default;
    ~Scd41() override;

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

  private:
    static uint8_t calc_crc(const std::span<uint8_t> data);

    static void transform_to_send_data(const std::span<uint16_t> input, std::span<uint8_t> output, size_t outputOffset);
    static bool validate_transform_received_data(const std::span<uint8_t> input, std::span<uint16_t> output);

    [[nodiscard]] bool write(uint16_t reg, std::chrono::milliseconds timeout) const;
    [[nodiscard]] bool write(uint16_t reg, uint16_t payload, std::chrono::milliseconds timeout) const;
    [[nodiscard]] bool write_read(uint16_t reg, std::span<uint16_t> response, std::chrono::milliseconds timeout) const;

    static measurement_t convert_measurement(std::span<uint16_t, 3> data);
    static double convert_measurement_to_temp(uint16_t mTemp);
    static double convert_measurement_to_hum(uint16_t mHum);
    static uint16_t convert_measurement_to_co2(uint16_t mCo2);
};
} // namespace sensors