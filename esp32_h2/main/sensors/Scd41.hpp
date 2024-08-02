#pragma once

#include "FreeRTOSConfig.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace sensors {
struct measurement_t {
    uint16_t co2;
    double temp;
    double hum;
} __attribute__((aligned(32)));

/**
 * Specification: https://sensirion.com/media/documents/48C4B7FB/6426E14D/CD_DS_SCD40_SCD41_Datasheet_D1_052023.pdf
 **/
class Scd41 {
  private:
    static constexpr uint8_t DEVICE_ADDR = 0x62;
    static const char* TAG;

    i2c_port_t port;
    gpio_num_t sda;
    gpio_num_t scl;

    std::array<uint8_t, 2> buffer;

  public:
    Scd41(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
    Scd41(Scd41&&) = default;
    Scd41(const Scd41&) = default;
    Scd41& operator=(Scd41&&) = default;
    Scd41& operator=(const Scd41&) = default;
    ~Scd41();

    [[nodiscard]] bool init() const;

    [[nodiscard]] std::optional<measurement_t> read_measurement() const;
    [[nodiscard]] bool get_data_ready_status() const;
    [[nodiscard]] bool start_periodic_measurement() const;
    [[nodiscard]] bool stop_periodic_measurement() const;
    [[nodiscard]] bool perform_factory_reset() const;
    [[nodiscard]] bool reinit() const;
    [[nodiscard]] bool perform_self_test() const;
    [[nodiscard]] uint64_t get_serial_number() const;
    void set_temperature_offset(double offset) const;
    [[nodiscard]] double get_temperature_offset() const;
    void set_sensor_altitude(uint16_t altitude) const;
    [[nodiscard]] uint16_t get_sensor_altitude() const;
    void persist_settings() const;

  private:
    static uint8_t calc_crc(const std::span<uint8_t> data);

    static void transform_to_send_data(const std::span<uint16_t> data, std::span<uint8_t> dataPrepared);
    static bool validate_transform_received_data(const std::span<uint8_t> dataReceived, std::span<uint16_t> data);

    [[nodiscard]] bool write(uint16_t reg, std::chrono::milliseconds timeout) const;
    [[nodiscard]] bool write(uint16_t reg, uint16_t payload, std::chrono::milliseconds timeout) const;
    [[nodiscard]] bool write_read(uint16_t reg, std::span<uint16_t> response, std::chrono::milliseconds timeout) const;

    static measurement_t convert_measurement(std::span<uint16_t, 3> data);
    static double convert_measurement_to_temp(uint16_t mTemp);
    static double convert_measurement_to_hum(uint16_t mHum);
    static uint16_t convert_measurement_to_co2(uint16_t mCo2);
};
} // namespace sensors