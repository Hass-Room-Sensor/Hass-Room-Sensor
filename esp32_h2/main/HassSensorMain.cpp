#include "actuators/RgbLed.hpp"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "sensors/Scd41.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

namespace {
const char* TAG = "hassSensor";
}  // namespace

void mainLoop() {
    sensors::Scd41 scd41;
    actuators::RgbLed rgbLed(GPIO_NUM_8);
    ESP_LOGI(TAG, "Everything initialized.");

    while (true) {
        for (size_t i = 0; i < 360 * 10; i++) {
            double hue = static_cast<double>(i) / 3600;
            rgbLed.on(hue);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(void) {
    mainLoop();
}
