#include "actuators/RgbLed.hpp"
#include "hal/gpio_types.h"
#include "sensors/Scd41.hpp"

#include <chrono>
#include <iostream>
#include <thread>

void mainLoop() {
    sensors::Scd41 scd41;
    actuators::RgbLed rgbLed(GPIO_NUM_8);
    while (true) {
        rgbLed.off();
        std::cout << "OFF\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rgbLed.on();
        std::cout << "ON\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(void) {
    mainLoop();
}
