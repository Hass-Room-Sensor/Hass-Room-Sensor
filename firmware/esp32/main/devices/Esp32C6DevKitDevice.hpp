#pragma once

#include "defs/DeviceDefs.hpp"

#include "devices/DebugRgbLedDevice.hpp"
#include "hal/gpio_types.h"

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT
namespace devices {
class Esp32C6DevKitDevice : public DebugRgbLedDevice {
  private:
    /**
     * Log tag.
     **/
    const char* TAG = "ESP32C6DevKit";

  public:
    Esp32C6DevKitDevice();
};
} // namespace devices
#endif