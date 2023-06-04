#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace zigbee {
class ZDevice {
  public:
    static const char* TAG;

    std::string manufacturerName = " DOOP";
    std::string modelName = "HASS_Sensor";
    std::string version = "0.1.0_alpha";

  private:
    static constexpr int ENDPOINT_ID = 10;

  public:
    ZDevice() = default;
    ZDevice(ZDevice&&) = default;
    ZDevice(const ZDevice&) = default;
    ZDevice& operator=(ZDevice&&) = default;
    ZDevice& operator=(const ZDevice&) = default;
    ~ZDevice() = default;

    static const std::unique_ptr<ZDevice>& get_instance();

    void init();

    static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);

  private:
    static void zb_main_task(void* arg);
    static void on_attr_changed(uint8_t status, uint8_t endpoint, uint16_t clusterId, uint16_t attrId, void* newVal);
};
} // namespace zigbee
