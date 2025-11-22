#pragma once

#include "zigbee/ZigbeeDeviceState.hpp"
#include <cstdint>

namespace devices {
class AbstractDeviceEventListener {
  public:
    AbstractDeviceEventListener() = default;
    AbstractDeviceEventListener(AbstractDeviceEventListener&& old) = default;
    AbstractDeviceEventListener(const AbstractDeviceEventListener& other) = default;
    virtual ~AbstractDeviceEventListener() = default;

    AbstractDeviceEventListener& operator=(AbstractDeviceEventListener&&) = default;
    AbstractDeviceEventListener& operator=(const AbstractDeviceEventListener&) = default;

    virtual void init() = 0;
    virtual void on_device_state_changed(zigbee::ZigbeeDeviceState state) = 0;
    virtual void on_identify(uint16_t identifyTime) = 0;
    [[nodiscard]] virtual bool has_debug_led() const = 0;
    [[nodiscard]] virtual bool is_debug_led_enabled() const = 0;
    virtual void set_debug_led(bool enabled) = 0;
    virtual void indicate_error() = 0;
};
} // namespace devices
