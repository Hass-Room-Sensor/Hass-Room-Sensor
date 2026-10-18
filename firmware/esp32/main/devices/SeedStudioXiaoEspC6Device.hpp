#pragma once

#include "actuators/Led.hpp"
#include "devices/AbstractDeviceEventListener.hpp"
#include "esp_timer.h"
#include <mutex>

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

    std::mutex statusLedMutex;
    /** Restores the normal status indication when the current Identify duration expires. */
    esp_timer_handle_t identifyRestoreTimer{nullptr};
    /** Protects the status LED from concurrent Identify and Zigbee-state updates. */
    bool identifyActive{false};

  public:
    SeedStudioXiaoEspC6Device();
    SeedStudioXiaoEspC6Device(SeedStudioXiaoEspC6Device&&) = default;
    SeedStudioXiaoEspC6Device(const SeedStudioXiaoEspC6Device&) = default;
    SeedStudioXiaoEspC6Device& operator=(SeedStudioXiaoEspC6Device&&) = default;
    SeedStudioXiaoEspC6Device& operator=(const SeedStudioXiaoEspC6Device&) = default;
    ~SeedStudioXiaoEspC6Device() override;

    void init() override;
    void on_device_state_changed(zigbee::ZigbeeDeviceState state) override;
    void on_identify(uint16_t identifyTime) override;
    [[nodiscard]] bool has_debug_led() const override;
    [[nodiscard]] bool is_debug_led_enabled() const override;
    void set_debug_led(bool enabled) override;
    void prepare_for_deep_sleep() override;
    void indicate_error() override;

  private:
    /** Turns off the red and green state indicators before selecting a new state pattern. */
    void reset_state_leds();
    /** Stops the active Identify timer and blink pattern. */
    void stop_identify_effect();
    /** Applies the LED pattern for the latest Zigbee state unless Identify is active. */
    void refresh_status_led();
    /** Timer callback that ends Identify and restores the current Zigbee status pattern. */
    static void on_identify_restore_timer(void* arg);
};
} // namespace devices
#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6
