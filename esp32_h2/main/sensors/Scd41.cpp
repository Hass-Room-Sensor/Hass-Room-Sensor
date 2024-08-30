#include "sensors/Scd41.hpp"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "soc/clk_tree_defs.h"
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <optional>
#include <span>
#include <stdexcept>
#include <sys/_stdint.h>
#include <thread>
#include <vector>

/**
 * I2C Docs:
 * https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32/api-reference/peripherals/i2c.html
 **/

namespace sensors {
const char* Scd41::TAG = "SCD41";

Scd41::Scd41(gpio_num_t sda, gpio_num_t scl) : sda(sda), scl(scl) {
    busConf.clk_source = I2C_CLK_SRC_DEFAULT; // TODO(Fabian): Check if it works with power save mode
    busConf.sda_io_num = sda;
    busConf.scl_io_num = scl;
    busConf.i2c_port = -1;
    busConf.glitch_ignore_cnt = 7;

    ESP_ERROR_CHECK(i2c_new_master_bus(&busConf, &bus)); // No ports available -> ESP_ERR_NOT_FOUND

    devConf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    devConf.device_address = DEVICE_ADDR;
    devConf.scl_speed_hz = 100000;
    devConf.scl_wait_us = 0; // Use default

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &devConf, &dev));

    ESP_LOGI(TAG, "Bus and device created.");
}

bool Scd41::init() const {
    {
        // Ensure CRC calculation is correct:
        std::array<uint8_t, 2> arrA{0xBE, 0xEF};
        const uint8_t crcA = calc_crc(arrA);
        assert(crcA == 0x92);
        std::array<uint8_t, 2> arrB{0x00, 0x00};
        const uint8_t crcB = calc_crc(arrB);
        assert(crcB == 0x81);

        ESP_LOGD(TAG, "CRC check successful.");
    }


    // Make sure the sensor has enough time to enter idle state:
    ESP_LOGI(TAG, "Waiting for sensor to enter idle state...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ESP_LOGI(TAG, "Idle state reached.");
    ESP_LOGI(TAG, "Stopping periodic measurement...");

    // Stop all existing measurements:
    if (!stop_periodic_measurement()) {
        return false;
    }
    ESP_LOGI(TAG, "Periodic measurement stopped.");
    ESP_LOGI(TAG, "Performing self test...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Test if everything works:
    if (!perform_self_test()) {
        ESP_LOGE(TAG, "Self test failed.");
        return false;
    }
    ESP_LOGI(TAG, "Self test done.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ESP_LOGI(TAG, "Reinit started...");
    // if (!reinit()) {
    //     return false;
    // }
    // ESP_LOGI(TAG, "Reinit done.");

    /*ESP_LOGI(TAG, "Performing factory reset...");
    if (!perform_factory_reset()) {
        return false;
    }
    ESP_LOGI(TAG, "Factory reset done.");*/

    // For Dagersheim: https://de-de.topographic-map.com/map-27vsrr/Dagersheim/
    set_sensor_altitude(438);
    // persist_settings(); // "The EEPROM is guaranteed to withstand at least 2000 write cycles."

    ESP_LOGI(TAG, "SCD41 serial number: %012llx", get_serial_number());
    ESP_LOGI(TAG, "SCD41 altitude: %u", get_sensor_altitude());
    ESP_LOGI(TAG, "SCD41 temperature offset: %f", get_temperature_offset());

    if (!start_periodic_measurement()) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ESP_LOGI(TAG, "Setup and ready to use.");
    return true;
}

Scd41::~Scd41() {
    (void) stop_periodic_measurement();

    // Cleanup I2C devices
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
    ESP_LOGI(TAG, "Bus and device deleted.");
}

void Scd41::transform_to_send_data(const std::span<uint16_t> input, std::span<uint8_t> output, size_t outputOffset) {
    assert(output.size() == (input.size() * 3) + outputOffset);
    if (output.size() != (input.size() * 3) + outputOffset) {
        throw std::invalid_argument("output.size() != input.size() * 3");
    }

    size_t preparedDataIndex = outputOffset;
    for (size_t i = 0; i < input.size(); i++) {
        output[preparedDataIndex] = static_cast<uint8_t>(input[i] >> 8);
        output[preparedDataIndex + 1] = static_cast<uint8_t>(input[i] & 0xFF);
        output[preparedDataIndex + 2] = calc_crc(output.subspan(preparedDataIndex, 2));
        preparedDataIndex += 3;
    }
}

bool Scd41::validate_transform_received_data(const std::span<uint8_t> input, std::span<uint16_t> output) {
    assert(input.size() == output.size() * 3);
    if (input.size() != output.size() * 3) {
        throw std::invalid_argument("input.size() != output.size() * 3");
    }

    size_t dataIndex = 0;
    for (size_t i = 0; i < input.size(); i += 3) {
        ESP_LOGD(TAG, "input[%d] = '0x%02x', input[%d] = '0x%02x', input[%d] = '0x%02x'", i, input[i], i + 1, input[i + 1], i + 2, input[i + 2]);

        uint8_t crc = calc_crc(input.subspan(i, 2));
        if (crc != input[i + 2]) {
            ESP_LOGE(TAG, "Received data with invalid CRC. Received '0x%02x', but expected '0x%02x'.", input[i + 2], crc);
            return false;
        }
        output[dataIndex++] = (static_cast<uint16_t>(input[i]) << 8) | static_cast<uint16_t>(input[i + 1]);
    }
    ESP_LOGD(TAG, "Received %u words of valid data.", output.size());
    return true;
}

bool Scd41::write(uint16_t reg, std::chrono::milliseconds timeout) const {
    std::array<uint8_t, 2> data{static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};

    esp_err_t result = i2c_master_transmit(dev, data.data(), data.size(), timeout.count());
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write for write command.");
        return false;
    }
    return true;
}

bool Scd41::write(uint16_t reg, uint16_t payload, std::chrono::milliseconds timeout) const {
    std::array<uint8_t, 5> data{static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF), 0x0, 0x0, 0x0};

    // Payload
    std::array<uint16_t, 1> payloadArr{payload};
    transform_to_send_data(payloadArr, data, 2);

    esp_err_t result = i2c_master_transmit(dev, data.data(), data.size(), timeout.count());
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write for write command.");
        return false;
    }
    return true;
}

bool Scd41::write_read(uint16_t reg, std::span<uint16_t> response, std::chrono::milliseconds timeout) const {
    if (!write(reg, timeout)) {
        return false;
    }
    std::this_thread::sleep_for(timeout);

    // Response:
    std::vector<uint8_t> tmpResponse;
    tmpResponse.resize(response.size() * 3);
    esp_err_t result = i2c_master_receive(dev, tmpResponse.data(), tmpResponse.size(), timeout.count());

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write for write_read command.");
        return false;
    }

    // Check and transform the response:
    return validate_transform_received_data(tmpResponse, response);
}

bool Scd41::start_periodic_measurement() const {
    ESP_LOGD(TAG, "Starting measurement...");
    if (write(0x21b1, std::chrono::milliseconds(10))) {
        ESP_LOGD(TAG, "Measurement started.");
        return true;
    } else {
        ESP_LOGE(TAG, "Started measurement failed.");
        return false;
    }
}

bool Scd41::stop_periodic_measurement() const {
    ESP_LOGD(TAG, "Stopping measurement...");
    if (write(0x3f86, std::chrono::milliseconds(500))) {
        ESP_LOGD(TAG, "Measurement stopped.");
        return true;
    } else {
        ESP_LOGE(TAG, "Stopping measurement failed.");
        return false;
    }
}

std::optional<measurement_t> Scd41::read_measurement() const {
    ESP_LOGD(TAG, "Reading measurement...");
    std::array<uint16_t, 3> data{};
    if (!write_read(0xec05, data, std::chrono::milliseconds(10))) {
        ESP_LOGE(TAG, "Failed to read measurement.");
        return std::nullopt;
    }
    return convert_measurement(data);
}

bool Scd41::get_data_ready_status() const {
    ESP_LOGD(TAG, "Getting data ready state...");
    std::array<uint16_t, 1> response{};
    if (!write_read(0xe4b8, response, std::chrono::milliseconds(1))) {
        ESP_LOGE(TAG, "Failed to check if device is ready.");
        return false;
    }

    ESP_LOGD(TAG, "Is ready response: 0x%02x", response[0]);
    return (response[0] & ((static_cast<uint64_t>(1) << 12) - 1)) != 0;
}

bool Scd41::perform_factory_reset() const {
    ESP_LOGD(TAG, "Starting factory reset...");
    std::array<uint16_t, 1> response{};
    if (!write_read(0x3632, response, std::chrono::milliseconds(1200))) {
        ESP_LOGE(TAG, "Failed to perform factory reset.");
        return false;
    }
    return true;
}

bool Scd41::reinit() const {
    ESP_LOGD(TAG, "Starting reinit...");
    std::array<uint16_t, 1> response{};
    if (!write_read(0x3646, response, std::chrono::milliseconds(30))) {
        ESP_LOGE(TAG, "Failed to perform reinit.");
        return false;
    }
    return true;
}

bool Scd41::perform_self_test() const {
    ESP_LOGD(TAG, "Starting self test...");
    std::array<uint16_t, 1> response{};
    if (!write_read(0x3639, response, std::chrono::milliseconds(10000))) {
        ESP_LOGE(TAG, "Failed to perform self test.");
        return false;
    }
    return response[0] == 0;
}

uint64_t Scd41::get_serial_number() const {
    ESP_LOGD(TAG, "Requesting serial number...");
    std::array<uint16_t, 3> response{};
    if (!write_read(0x3682, response, std::chrono::milliseconds(1))) {
        ESP_LOGE(TAG, "Failed to get serial number.");
        return false;
    }

    return (static_cast<uint64_t>(response[0]) << 32) | (static_cast<uint64_t>(response[1]) << 16) | static_cast<uint64_t>(response[2]);
}

void Scd41::set_temperature_offset(double offset) const {
    ESP_LOGD(TAG, "Setting temperature offset...");
    uint16_t temp = static_cast<uint16_t>(std::round(offset * (static_cast<double>(0xFFFF) / 175.0)));
    if (!write(0x241d, temp, std::chrono::milliseconds(1))) {
        ESP_LOGE(TAG, "Failed to perform write_read inside set_temperature_offset.");
        return;
    }
}

double Scd41::get_temperature_offset() const {
    ESP_LOGD(TAG, "Requesting temperature offset...");
    std::array<uint16_t, 1> data{};
    if (!write_read(0x2318, data, std::chrono::milliseconds(1))) {
        ESP_LOGE(TAG, "Failed to perform write_read inside get_temperature_offset.");
        return -1;
    }
    return static_cast<double>(data[0]) * (175.0 / static_cast<double>(0xFFFF));
}

void Scd41::set_sensor_altitude(uint16_t altitude) const {
    ESP_LOGD(TAG, "Setting altitude...");
    if (!write(0x2427, altitude, std::chrono::milliseconds(1))) {
        ESP_LOGE(TAG, "Failed to perform write_read inside set_sensor_altitude.");
        return;
    }
}

uint16_t Scd41::get_sensor_altitude() const {
    ESP_LOGD(TAG, "Requesting altitude...");
    std::array<uint16_t, 1> data{};
    if (!write_read(0x2322, data, std::chrono::milliseconds(1))) {
        ESP_LOGE(TAG, "Failed to perform write_read inside get_sensor_altitude.");
        return -1;
    }
    return data[0];
}

void Scd41::persist_settings() const {
    ESP_LOGD(TAG, "Storing settings persistent...");
    if (write(0x3615, std::chrono::milliseconds(800))) {
        ESP_LOGD(TAG, "Settings stored persistent.");
    } else {
        ESP_LOGE(TAG, "Storing settings persistent failed.");
    }
}
uint8_t Scd41::calc_crc(const std::span<uint8_t> data) {
    static const uint8_t CRC8_POLYNOMIAL = 0x31;
    static const uint8_t CRC8_INIT = 0xFF;

    uint8_t crc = CRC8_INIT;
    uint8_t crcBit;
    /* calculates 8-Bit checksum with given polynomial */
    for (const uint8_t d : data) {
        crc ^= (d);
        for (crcBit = 8; crcBit > 0; --crcBit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

measurement_t Scd41::convert_measurement(std::span<uint16_t, 3> data) {
    uint16_t co2 = convert_measurement_to_co2(data[0]);
    double temp = convert_measurement_to_temp(data[1]);
    double hum = convert_measurement_to_hum(data[2]);

    return {co2, temp, hum};
}

double Scd41::convert_measurement_to_temp(uint16_t mTemp) {
    return -45.0 + 175.0 * (static_cast<double>(mTemp) / static_cast<double>(0xFFFF));
}

double Scd41::convert_measurement_to_hum(uint16_t mHum) {
    return 100 * static_cast<double>(mHum) / static_cast<double>((2 << 16) - 1);
}

uint16_t Scd41::convert_measurement_to_co2(uint16_t mCo2) {
    return mCo2; // Nothing needs to be done here
}
} // namespace sensors