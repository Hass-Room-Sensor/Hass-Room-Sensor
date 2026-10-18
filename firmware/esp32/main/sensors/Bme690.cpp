#include "sensors/Bme690.hpp"

#include "esp_log.h"

#include <esp_check.h>
#include <thread>
#include <vector>

namespace sensors {
const char* Bme690::TAG = "BME690";

Bme690::Bme690(gpio_num_t sda, gpio_num_t scl) : sda_(sda), scl_(scl) {
    busConfig_.clk_source = I2C_CLK_SRC_DEFAULT;
    busConfig_.sda_io_num = sda_;
    busConfig_.scl_io_num = scl_;
    busConfig_.i2c_port = 0;
    busConfig_.glitch_ignore_cnt = 7;
    busConfig_.flags.enable_internal_pullup = 0;

    deviceConfig_.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    deviceConfig_.scl_speed_hz = 100000;
    deviceConfig_.scl_wait_us = 0;
    deviceConfig_.flags.disable_ack_check = 0;

    sensor_.intf = BME69X_I2C_INTF;
    sensor_.read = read_register;
    sensor_.write = write_register;
    sensor_.delay_us = delay_microseconds;
    sensor_.intf_ptr = &interfaceContext_;
    sensor_.amb_temp = 25;

    sensorConfig_.os_hum = BME69X_OS_1X;
    sensorConfig_.os_pres = BME69X_OS_16X;
    sensorConfig_.os_temp = BME69X_OS_2X;
    sensorConfig_.filter = BME69X_FILTER_OFF;
    sensorConfig_.odr = BME69X_ODR_NONE;

    // The firmware only needs pressure/temperature/humidity. Gas measurement is kept disabled so each
    // forced-mode sample naturally returns to the sensor's low-power sleep state afterwards.
    heaterConfig_.enable = BME69X_DISABLE;
    heaterConfig_.heatr_dur = 0;
    heaterConfig_.heatr_temp = 0;

    create_bus();
}

Bme690::~Bme690() {
    destroy_bus();
}

void Bme690::create_bus() {
    ESP_ERROR_CHECK(i2c_new_master_bus(&busConfig_, &bus_));
}

void Bme690::destroy_bus() noexcept {
    if (device_) {
        i2c_master_bus_rm_device(device_);
        device_ = nullptr;
    }
    if (bus_) {
        i2c_del_master_bus(bus_);
        bus_ = nullptr;
    }
}

bool Bme690::attach_device(uint8_t deviceAddress) {
    if (device_) {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(device_));
        device_ = nullptr;
    }

    deviceConfig_.device_address = deviceAddress;
    if (const esp_err_t err = i2c_master_bus_add_device(bus_, &deviceConfig_, &device_); err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BME690 I2C device at 0x%02x: %s", deviceAddress, esp_err_to_name(err));
        return false;
    }

    interfaceContext_.dev = device_;
    deviceAddress_ = deviceAddress;
    return true;
}

bool Bme690::probe_device() {
    for (const uint8_t address : {PRIMARY_ADDRESS, SECONDARY_ADDRESS}) {
        if (const esp_err_t err = i2c_master_probe(bus_, address, BUS_TIMEOUT.count()); err == ESP_OK) {
            ESP_LOGI(TAG, "Detected BME690 at I2C address 0x%02x.", address);
            return attach_device(address);
        }
    }

    ESP_LOGW(TAG, "No BME690 detected on the configured I2C bus.");
    return false;
}

bool Bme690::init() {
    if (!probe_device()) {
        return false;
    }

    if (const int8_t result = bme69x_init(&sensor_); result != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_init failed with status %d.", static_cast<int>(result));
        return false;
    }

    if (const int8_t result = bme69x_set_conf(&sensorConfig_, &sensor_); result != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_set_conf failed with status %d.", static_cast<int>(result));
        return false;
    }

    if (const int8_t result = bme69x_set_heatr_conf(BME69X_FORCED_MODE, &heaterConfig_, &sensor_); result != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_set_heatr_conf failed with status %d.", static_cast<int>(result));
        return false;
    }

    initialized_ = true;
    return true;
}

std::optional<bme690_measurement_t> Bme690::read_measurement() {
    if (!initialized_) {
        ESP_LOGW(TAG, "BME690 measurement requested before initialization.");
        return std::nullopt;
    }

    if (const int8_t result = bme69x_set_op_mode(BME69X_FORCED_MODE, &sensor_); result != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_set_op_mode failed with status %d.", static_cast<int>(result));
        return std::nullopt;
    }

    const uint32_t measurementDurationUs = bme69x_get_meas_dur(BME69X_FORCED_MODE, &sensorConfig_, &sensor_);
    std::this_thread::sleep_for(std::chrono::microseconds(measurementDurationUs) + MEASUREMENT_GUARD_TIME);

    bme69x_data rawData{};
    uint8_t numberOfFields = 0;
    if (const int8_t result = bme69x_get_data(BME69X_FORCED_MODE, &rawData, &numberOfFields, &sensor_); result != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_get_data failed with status %d.", static_cast<int>(result));
        return std::nullopt;
    }

    if (numberOfFields == 0 || (rawData.status & BME69X_NEW_DATA_MSK) == 0) {
        ESP_LOGW(TAG, "BME690 returned no fresh environmental sample.");
        return std::nullopt;
    }

    return bme690_measurement_t{
            .temp_celsius = static_cast<double>(rawData.temperature),
            .humidity_percent = static_cast<double>(rawData.humidity),
            .pressure_hpa = static_cast<double>(rawData.pressure) / 100.0,
    };
}

BME69X_INTF_RET_TYPE Bme690::read_register(uint8_t regAddress, uint8_t* registerData, uint32_t length, void* interfacePointer) {
    auto* context = static_cast<InterfaceContext*>(interfacePointer);
    if (!context || !context->dev) {
        return BME69X_E_NULL_PTR;
    }

    if (const esp_err_t err = i2c_master_transmit_receive(context->dev, &regAddress, sizeof(regAddress), registerData, length, BUS_TIMEOUT.count()); err != ESP_OK) {
        ESP_LOGE(TAG, "BME690 register read failed at 0x%02x: %s", regAddress, esp_err_to_name(err));
        return BME69X_E_COM_FAIL;
    }

    return BME69X_OK;
}

BME69X_INTF_RET_TYPE Bme690::write_register(uint8_t regAddress, const uint8_t* registerData, uint32_t length, void* interfacePointer) {
    auto* context = static_cast<InterfaceContext*>(interfacePointer);
    if (!context || !context->dev || !registerData) {
        return BME69X_E_NULL_PTR;
    }

    std::vector<uint8_t> payload{};
    payload.reserve(length + 1U);
    payload.push_back(regAddress);
    payload.insert(payload.end(), registerData, registerData + length);

    if (const esp_err_t err = i2c_master_transmit(context->dev, payload.data(), payload.size(), BUS_TIMEOUT.count()); err != ESP_OK) {
        ESP_LOGE(TAG, "BME690 register write failed at 0x%02x: %s", regAddress, esp_err_to_name(err));
        return BME69X_E_COM_FAIL;
    }

    return BME69X_OK;
}

void Bme690::delay_microseconds(uint32_t period, void* /*interfacePointer*/) {
    std::this_thread::sleep_for(std::chrono::microseconds(period));
}
} // namespace sensors
