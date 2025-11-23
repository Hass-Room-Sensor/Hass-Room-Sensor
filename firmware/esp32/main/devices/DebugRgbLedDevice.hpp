#pragma once

#include "actuators/RgbLed.hpp"
#include "devices/AbstractDeviceEventListener.hpp"
#include "hal/gpio_types.h"
#include "zigbee/ZigbeeDeviceState.hpp"
#include <memory>

namespace devices {
class DebugRgbLedDevice : public AbstractDeviceEventListener {
  public:
    explicit DebugRgbLedDevice(gpio_num_t rgbLedGpio);
    DebugRgbLedDevice(DebugRgbLedDevice&&) = default;
    DebugRgbLedDevice(const DebugRgbLedDevice&) = default;
    DebugRgbLedDevice& operator=(DebugRgbLedDevice&&) = default;
    DebugRgbLedDevice& operator=(const DebugRgbLedDevice&) = default;
    ~DebugRgbLedDevice() override = default;

    void init() override;
    void on_device_state_changed(zigbee::ZigbeeDeviceState state) override;
    void on_identify(uint16_t identifyTime) override;
    [[nodiscard]] bool has_debug_led() const override;
    [[nodiscard]] bool is_debug_led_enabled() const override;
    void set_debug_led(bool enabled) override;
    void indicate_error() override;

  private:
    actuators::RgbLed debugLed;

    void apply_state_color(zigbee::ZigbeeDeviceState state);
};
} // namespace devices
