#include "sensors/Scd41.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include <array>
#include <cassert>
#include <chrono>
#include <span>
#include <stdexcept>
#include <thread>
#include <vector>
#include <sys/_stdint.h>

namespace sensors {
const char* Scd41::TAG = "SCD41";

Scd41::Scd41(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) : port(port), sda(sda), scl(scl) {}

bool Scd41::init() const {
    {
        // Ensure CRC calculation is correct:
        std::array<uint8_t, 2> arrA{0xBE, 0xEF};
        const uint8_t crcA = calc_crc(arrA);
        assert(crcA == 0x92);
        std::array<uint8_t, 2> arrB{0x00, 0x00};
        const uint8_t crcB = calc_crc(arrB);
        assert(crcB == 0x81);
    }

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

    // Make sure the sensor has enough time to enter idle state:
    ESP_LOGI(TAG, "Waiting for sensor to enter idle state...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ESP_LOGI(TAG, "Idle state reached.");

    // Stop all existing measurements:
    if (!stop_measurement()) {
        return false;
    }

    // Test if everything works:
    if (!perform_self_test()) {
        return false;
    }

    // Get the serial number to validate everything works:
    if (!get_serial_number()) {
        return false;
    }

    if (!start_measurement()) {
        return false;
    }
    ESP_LOGI(TAG, "Setup and ready to use.");
    return true;
}

Scd41::~Scd41() {
    (void) stop_measurement();
}

void Scd41::prepare_data_to_send(const std::span<uint16_t> data, std::span<uint8_t> dataPrepared) {
    assert(dataPrepared.size() == (data.size() * 2) + (data.size() / 2));
    if (dataPrepared.size() != (data.size() * 2) + (data.size() / 2)) {
        throw std::invalid_argument("dataPrepared.size() != (data.size() * 2) + (data.size() / 2)");
    }

    size_t preparedDataIndex = 0;
    for (size_t i = 0; i < data.size(); i++) {
        dataPrepared[preparedDataIndex] = static_cast<uint8_t>(data[i] >> 8);
        dataPrepared[preparedDataIndex + 1] = static_cast<uint8_t>(data[i] & 0xFF);
        dataPrepared[preparedDataIndex + 2] = calc_crc(dataPrepared.subspan(preparedDataIndex, 2));
        preparedDataIndex += 3;
    }
}

bool Scd41::prepare_received_data(const std::span<uint8_t> dataReceived, std::span<uint16_t> data) {
    assert(dataReceived.size() == (data.size() / 3) * 2);
    if (dataReceived.size() != (data.size() / 3) * 2) {
        throw std::invalid_argument("dataReceived.size() != (data.size() / 3) * 2");
    }

    size_t dataIndex = 0;
    for (size_t i = 0; i < dataReceived.size(); i += 3) {
        uint8_t crc = calc_crc(dataReceived.subspan(i, 2));
        if (crc != dataReceived[2]) {
            ESP_LOGE(TAG, "Received data with invalid CRC. Received '%02x', but expected '%02x'.", dataReceived[2], crc);
            return false;
        }
        data[dataIndex++] = (static_cast<uint16_t>(dataReceived[i])) | static_cast<uint16_t>(dataReceived[i + 1]);
    }
    ESP_LOGD(TAG, "Received %u words of valid data.", data.size());
    return true;
}

bool Scd41::write(uint16_t data, std::chrono::milliseconds timeout) const {
    std::array<uint16_t, 1> dataArr{data};
    return write(dataArr, timeout);
}

bool Scd41::write(const std::span<uint16_t> data, std::chrono::milliseconds timeout) const {
    // Prepare the data and add the CRC:
    std::vector<uint8_t> tmpSend;
    tmpSend.resize((data.size() * 2) + (data.size() / 2));
    prepare_data_to_send(data, tmpSend);

    // Send it to the device:
    esp_err_t result = i2c_master_write_to_device(port, DEVICE_ADDR, tmpSend.data(), tmpSend.size(), static_cast<TickType_t>(timeout.count()) / portTICK_PERIOD_MS);
    switch (result) {
        case ESP_OK:
            ESP_LOGI(TAG, "All good when writing.");
            return true;

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
    return false;
}

bool Scd41::write_read(uint16_t data, std::span<uint16_t> response, std::chrono::milliseconds timeout) const {
    std::array<uint16_t, 1> dataArr{data};
    return write_read(dataArr, response, timeout);
}

bool Scd41::write_read(const std::span<uint16_t> data, std::span<uint16_t> response, std::chrono::milliseconds timeout) const {
    // Prepare the data and add the CRC:
    std::vector<uint8_t> tmpSend;
    tmpSend.resize((data.size() * 2) + (data.size() / 2));
    prepare_data_to_send(data, tmpSend);

    std::vector<uint8_t> tmpResponse;
    tmpResponse.resize((response.size() * 2) + (response.size() / 2));

    // Send it to the device and read the response:
    ESP_ERROR_CHECK(i2c_master_write_read_device(port, DEVICE_ADDR, tmpSend.data(), tmpSend.size(), tmpResponse.data(), tmpResponse.size(), static_cast<TickType_t>(timeout.count()) / portTICK_PERIOD_MS));

    // Check and transform the response:
    return prepare_received_data(tmpResponse, response);
}

bool Scd41::start_measurement() const {
    ESP_LOGD(TAG, "Starting measurement...");
    if (write(0x21b1, std::chrono::milliseconds(10))) {
        ESP_LOGD(TAG, "Measurement started.");
        return true;
    } else {
        ESP_LOGE(TAG, "Started measurement failed.");
        return true;
    }
}

bool Scd41::stop_measurement() const {
    ESP_LOGD(TAG, "Stopping measurement...");
    if (write(0x3f86, std::chrono::milliseconds(500))) {
        ESP_LOGD(TAG, "Measurement stopped.");
        return true;
    } else {
        ESP_LOGE(TAG, "Stopping measurement failed.");
        return true;
    }
}

bool Scd41::read_measurement(std::span<uint16_t, 3> response) const {
    return write_read(0xec05, response, std::chrono::milliseconds(10));
}

bool Scd41::is_ready() const {
    std::array<uint16_t, 1> response{};
    if (!write_read(0xe4b8, response, std::chrono::milliseconds(10))) {
        ESP_LOGI(TAG, "Failed to check if device is ready.");
        return false;
    }

    ESP_LOGD(TAG, "Is ready response: %d, %d .", response[0], response[1]);
    return (response[0] & ((static_cast<uint64_t>(1) << 12) - 1)) != 0;
}

bool Scd41::perform_self_test() const {
    ESP_LOGD(TAG, "Starting self test...");
    std::array<uint16_t, 1> response{};
    if (!write_read(0x3639, response, std::chrono::milliseconds(10000))) {
        ESP_LOGI(TAG, "Failed to perform self test.");
        return false;
    }
    return response[0] == 0;
}

uint64_t Scd41::get_serial_number() const {
    ESP_LOGD(TAG, "Requesting serial number...");
    std::array<uint16_t, 3> response{};
    if (!write_read(0x3682, response, std::chrono::milliseconds(10))) {
        ESP_LOGI(TAG, "Failed to get serial number.");
        return false;
    }

    uint64_t result = (static_cast<uint64_t>(response[0]) << 32) | (static_cast<uint64_t>(response[1]) << 16) | static_cast<uint64_t>(response[2]);
    ESP_LOGI(TAG, "Serial number: %012llx", result);
    return result;
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
}  // namespace sensors