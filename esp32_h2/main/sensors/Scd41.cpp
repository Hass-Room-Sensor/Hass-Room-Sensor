#include "sensors/Scd41.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include <array>
#include <span>
#include <sys/_stdint.h>

namespace sensors {
Scd41::Scd41(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) : port(port), sda(sda), scl(scl) {
    init();
}

void Scd41::init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    i2c_param_config(port, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(port, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "Driver installed.");

    start_measurement();
}

Scd41::~Scd41() {
    stop_measurement();
}

void Scd41::write(uint16_t command) {
    std::array<uint8_t, 2> buf;
    buf[0] = static_cast<uint8_t>((command & 0xFF00) >> 8);
    buf[1] = static_cast<uint8_t>(command & 0xFF);
    write(buf);
}

void Scd41::write(const std::array<uint8_t, 2>& command) {
    esp_err_t result = i2c_master_write_to_device(port, DEVICE_ADDR, command.data(), command.size(), TIMEOUT_MS / portTICK_PERIOD_MS);
    switch (result) {
        case ESP_OK:
            ESP_LOGI(TAG, "All good when writing.");
            break;

        case ESP_FAIL:
            ESP_LOGE(TAG, "ESP_FAIL during write.");
            break;

        case ESP_ERR_INVALID_STATE:
            ESP_LOGE(TAG, "ESP_ERR_INVALID_STATE during write.");
            break;

        case ESP_ERR_TIMEOUT:
            ESP_LOGE(TAG, "ESP_ERR_TIMEOUT during write.");
            break;

        default:
            ESP_LOGE(TAG, "Unknown write result!");
            break;
    }
    // ESP_ERROR_CHECK(result);
}

void Scd41::start_measurement() {
    write(0x21b1);
    ESP_LOGI(TAG, "Measurement started.");
}

void Scd41::stop_measurement() {
    write(0x3f86);
    ESP_LOGI(TAG, "Measurement stoped.");
}

void Scd41::read_measurement(std::array<uint8_t, 9>& response) {
    std::span<uint8_t, 9> respSpan{response};
    write_read(0x21b1, respSpan);
}

bool Scd41::is_ready() {
    std::array<uint8_t, 3> response{};
    std::span<uint8_t, 3> respSpan{response};
    write_read(0xe4b8, respSpan);
    ESP_LOGI(TAG, "Is ready response: %d, %d .", response[0], response[1]);

    return !(response[2] == 0 && (response[1] & 0b111) == 0);
}
}  // namespace sensors