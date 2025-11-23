#include "devices/Esp32C6DevKitDevice.hpp"
#include "defs/DeviceDefs.hpp"

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT
namespace devices {
Esp32C6DevKitDevice::Esp32C6DevKitDevice() : DebugRgbLedDevice(HASS_SENSOR_DEBUG_RGB_LED_GPIO) {}
} // namespace devices
#endif