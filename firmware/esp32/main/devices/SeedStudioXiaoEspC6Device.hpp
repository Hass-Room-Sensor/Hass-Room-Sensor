#pragma once

#include "defs/DeviceDefs.hpp"

#include "actuators/Led.hpp"
#include "devices/AbstractDeviceEventListener.hpp"
#include <memory>

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6
namespace devices {
class SeedStudioXiaoEspC6Device : public AbstractDeviceEventListener {
  private:
    /**
     * Log tag.
     **/
    const char* TAG = "SeedStudioESPC6";

    actuators::Led statusLed;
    actuators::Led redLed;
    actuators::Led greenLed;

  public:
    SeedStudioXiaoEspC6Device();
    SeedStudioXiaoEspC6Device(SeedStudioXiaoEspC6Device&&) = default;
    SeedStudioXiaoEspC6Device(const SeedStudioXiaoEspC6Device&) = default;
    SeedStudioXiaoEspC6Device& operator=(SeedStudioXiaoEspC6Device&&) = default;
    SeedStudioXiaoEspC6Device& operator=(const SeedStudioXiaoEspC6Device&) = default;
    ~SeedStudioXiaoEspC6Device() override = default;

    void init() override;
    void on_device_state_changed(zigbee::ZigbeeDeviceState state) override;
    void on_identify(uint16_t identifyTime) override;
    [[nodiscard]] bool has_debug_led() const override;
    [[nodiscard]] bool is_debug_led_enabled() const override;
    void set_debug_led(bool enabled) override;
    void indicate_error() override;

  private:
    void reset_state_leds();
};
} // namespace devices
#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6
