#include "actuators/RgbLed.hpp"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "nvs_flash.h"
#include "sensors/Scd41.hpp"
#include "zigbee/ZDevice.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

namespace {
const char* TAG = "hassSensor";
} // namespace

void mainLoop() {
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    // Initialize the flash:
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize the LED:
    std::shared_ptr<actuators::RgbLed> rgbLed = std::make_shared<actuators::RgbLed>(GPIO_NUM_8);
    rgbLed->init();
    rgbLed->on(actuators::color_t{0, 0, 30});
    zigbee::ZDevice::get_instance()->set_led(rgbLed);

    // Initialize the SCD41 sensor:
    sensors::Scd41 scd41(I2C_NUM_0, GPIO_NUM_12, GPIO_NUM_22);
    if (!scd41.init()) {
        rgbLed->on(actuators::color_t{30, 0, 0});
        ESP_LOGE(TAG, "Initializing SCD41 failed. Rebooting...");
        esp_restart();
    }

    // Setup ZigBee device with initial measurements:
    std::optional<sensors::measurement_t> measurement = std::nullopt;
    do {
        if (scd41.get_data_ready_status()) {
            measurement = scd41.read_measurement();
            if (measurement) {
                ESP_LOGI(TAG, "Initial measurement: %d ppm, %.2lf °C, %.2lf %%", measurement->co2, measurement->temp, measurement->hum);
                zigbee::ZDevice::get_instance()->init(measurement->temp, measurement->hum, measurement->co2);
            }
        } else {
            ESP_LOGI(TAG, "Waiting for initial measurements. SCD41 is not ready yet. Sleeping for a second before rechecking...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } while (!measurement);

    ESP_LOGI(TAG, "Everything initialized.");

    // Main loop:
    while (true) {
        if (scd41.get_data_ready_status()) {
            ESP_LOGI(TAG, "Data ready. Reading...");
            std::optional<sensors::measurement_t> measurement = scd41.read_measurement();
            if (measurement) {
                ESP_LOGI(TAG, "[Measurement]: %d ppm, %.2lf °C, %.2lf %%", measurement->co2, measurement->temp, measurement->hum);
                zigbee::ZDevice::get_instance()->update_temp(measurement->temp);
                zigbee::ZDevice::get_instance()->update_hum(measurement->hum);
                zigbee::ZDevice::get_instance()->update_co2(measurement->co2);
                std::this_thread::sleep_for(std::chrono::seconds(60)); // Looks like we can update values via ZigBee every 30 seconds anyway. Ref: https://github.com/espressif/esp-zigbee-sdk/issues/65
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }
}

/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(void) {
    mainLoop();
}
