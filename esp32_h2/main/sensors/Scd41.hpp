#pragma once

#include "FreeRTOSConfig.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "portmacro.h"
#include <array>
#include <cstdint>
#include <span>

namespace sensors {
/**
 * Specification: https://sensirion.com/media/documents/48C4B7FB/6426E14D/CD_DS_SCD40_SCD41_Datasheet_D1_052023.pdf
 **/
class Scd41 {
 private:
    static constexpr uint8_t DEVICE_ADDR = 0x62;
    static constexpr TickType_t TIMEOUT_MS = 1000;
    static constexpr char* TAG = "SCD41";

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

    void read_measurement(std::array<uint8_t, 9>& response);
    bool is_ready();

 private:
    void init();

    void write(const std::array<uint8_t, 2>& buf);
    void write(uint16_t data);

    template <size_t N>
    void write_read(const std::array<uint8_t, 2>& command, std::span<uint8_t, N>& response) {
        ESP_ERROR_CHECK(i2c_master_write_read_device(port, DEVICE_ADDR, command.data(), command.size(), response.data(), response.size(), TIMEOUT_MS / portTICK_PERIOD_MS));
    }
    template <size_t N>
    void write_read(uint16_t command, std::span<uint8_t, N>& response) {
        std::array<uint8_t, 2> buf;
        buf[0] = static_cast<uint8_t>((command & 0xFF00) >> 8);
        buf[1] = static_cast<uint8_t>(command & 0xFF);
        write_read(buf, response);
    }

    void start_measurement();
    void stop_measurement();
};
}  // namespace sensors