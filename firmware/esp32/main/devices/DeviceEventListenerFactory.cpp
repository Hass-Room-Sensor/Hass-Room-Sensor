#include "devices/DeviceEventListenerFactory.hpp"

#include "defs/DeviceDefs.hpp"
#include "devices/AbstractDeviceEventListener.hpp"

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6
#include "devices/SeedStudioXiaoEspC6Device.hpp"
#elifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT
#include "devices/Esp32H2DevKitDevice.hpp"
#elifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT
#include "devices/Esp32C6DevKitDevice.hpp"
#endif

#include <memory>

namespace devices {
std::shared_ptr<AbstractDeviceEventListener> create_device_event_listener() {
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6
    return std::make_shared<SeedStudioXiaoEspC6Device>();
#elifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT
    return std::make_shared<Esp32H2DevKitDevice>();
#elifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT
    return std::make_shared<Esp32C6DevKitDevice>();
#else
    static_assert(false, "Invalid/Unknown device type.");
    return nullptr;
#endif
}
} // namespace devices
