#include "devices/DebugRgbLedDevice.hpp"
#include "actuators/RgbLed.hpp"
#include "zigbee/ZigbeeDeviceState.hpp"
#include <memory>

namespace devices {
DebugRgbLedDevice::DebugRgbLedDevice(gpio_num_t rgbLedGpio) : debugLed(rgbLedGpio) {}

void DebugRgbLedDevice::init() {
    debugLed.init();
    debugLed.on(actuators::color_t{.r = 0, .g = 0, .b = 30});
}

void DebugRgbLedDevice::apply_state_color(zigbee::ZigbeeDeviceState state) {
    switch (state) {
        case zigbee::ZigbeeDeviceState::SETUP:
            debugLed.on(actuators::color_t{.r = 30, .g = 0, .b = 30}); // Purple
            break;
        case zigbee::ZigbeeDeviceState::OTA:
            debugLed.on(actuators::color_t{.r = 30, .g = 30, .b = 0}); // Yellow
            break;
        case zigbee::ZigbeeDeviceState::CONNECTING:
        case zigbee::ZigbeeDeviceState::CONNECTED:
            debugLed.on(actuators::color_t{.r = 0, .g = 0, .b = 30}); // Blue
            break;
        default:
            debugLed.on(actuators::color_t{.r = 30, .g = 0, .b = 0}); // Red
            break;
    }
}

void DebugRgbLedDevice::on_device_state_changed(zigbee::ZigbeeDeviceState state) {
    apply_state_color(state);
}

void DebugRgbLedDevice::on_identify(uint16_t /*identifyTime*/) {}

bool DebugRgbLedDevice::has_debug_led() const {
    return true;
}

bool DebugRgbLedDevice::is_debug_led_enabled() const {
    return debugLed.is_enabled();
}

void DebugRgbLedDevice::set_debug_led(bool enabled) {
    if (enabled) {
        debugLed.enable();
    } else {
        debugLed.disable();
    }
}

void DebugRgbLedDevice::indicate_error() {
    debugLed.on(actuators::color_t{.r = 30, .g = 0, .b = 0});
}
} // namespace devices
