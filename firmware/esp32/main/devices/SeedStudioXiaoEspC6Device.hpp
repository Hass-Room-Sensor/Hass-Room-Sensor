#pragma once

#include "defs/DeviceDefs.hpp"

#include "actuators/Led.hpp"
#include "devices/AbstractDeviceEventListener.hpp"
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

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
    void set_sleep_indicator(bool sleeping) override;
    void prepare_for_deep_sleep() override;
    void indicate_error() override;

  private:
    void reset_state_leds();
    void refresh_status_led();
    void stop_identify_effect();

    std::mutex statusLedMutex_{};
    std::condition_variable_any identifyCv_{};
    std::jthread identifyRestoreWorker_{};
    bool sleepIndicatorActive_{false};
    bool identifyActive_{false};
};
} // namespace devices
#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6
