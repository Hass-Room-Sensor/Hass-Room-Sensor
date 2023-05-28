#include "actuators/RgbLed.hpp"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "sensors/Scd41.hpp"

#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

namespace {
const char* TAG = "hassSensor";
}  // namespace

void mainLoop() {
    esp_log_set_level_master(ESP_LOG_VERBOSE);

    actuators::RgbLed rgbLed(GPIO_NUM_8);
    // Sleep 100ms so the LED is initialized and ready for the next command
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rgbLed.on(actuators::color_t{0, 0, 30});

    sensors::Scd41 scd41(I2C_NUM_0, GPIO_NUM_12, GPIO_NUM_22);
    if (!scd41.init()) {
        rgbLed.on(actuators::color_t{30, 0, 0});
        ESP_LOGE(TAG, "Initializing SCD41 failed. Rebooting...");
        esp_restart();
    }
    ESP_LOGI(TAG, "Everything initialized.");
    rgbLed.on(actuators::color_t{0, 30, 0});

    std::array<uint16_t, 3> data;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (scd41.get_data_ready_status()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            ESP_LOGI(TAG, "Data ready. Reading...");
            if (scd41.read_measurement(data)) {
            }
        }
    }
}

/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(void) {
    mainLoop();
}
