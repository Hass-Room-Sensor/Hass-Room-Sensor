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
    sensors::Scd41 scd41(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_12);
    if (scd41.init()) {
        ESP_LOGE(TAG, "Initializing SCD41 failed. Rebooting...");
        esp_restart();
    }

    actuators::RgbLed rgbLed(GPIO_NUM_8);
    ESP_LOGI(TAG, "Everything initialized.");

    std::array<uint16_t, 3> data;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (scd41.is_ready()) {
            if (scd41.read_measurement(data)) {
                std::cout << "Data ready: ";
                for (const uint8_t d : data) {
                    std::cout << d << ", ";
                }
                std::cout << std::endl;
            }
        }

        // for (size_t i = 0; i < 360 * 10; i++) {
        //     double hue = static_cast<double>(i) / 3600;
        //     rgbLed.on(hue);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // }
    }
}

/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(void) {
    mainLoop();
}
