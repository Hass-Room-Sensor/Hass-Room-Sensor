#include "devices/Esp32H2DevKitDevice.hpp"
#include <defs/DeviceDefs.hpp>

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT
namespace devices {
Esp32H2DevKitDevice::Esp32H2DevKitDevice() : DebugRgbLedDevice(HASS_SENSOR_DEBUG_RGB_LED_GPIO) {}
} // namespace devices
#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT
