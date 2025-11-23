#pragma once

#include "defs/DeviceDefs.hpp"
#include "devices/DebugRgbLedDevice.hpp"

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT
namespace devices {
class Esp32H2DevKitDevice : public DebugRgbLedDevice {
  private:
    /**
     * Log tag.
     **/
    const char* TAG = "ESP32H2DevKit";

  public:
    Esp32H2DevKitDevice();
};
} // namespace devices
#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT
