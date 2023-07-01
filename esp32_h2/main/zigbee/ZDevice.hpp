#pragma once

#include <cstdint>

#include "esp_zigbee_type.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include <memory>
#include <string>
#include <vector>

namespace zigbee {
class ZDevice {
  public:
    static const char* TAG;

  private:
    static constexpr int ENDPOINT_ID = 10;

    // Basic cluster information:
    esp_zb_basic_cluster_cfg_t basicClusterConfig{ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE, ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE};
    esp_zb_attribute_list_t* basicAttrList{nullptr};

    std::vector<char> modelId;
    std::vector<char> manufacturer;
    std::vector<char> version;

    // Temperature cluster information:

    // Humidity cluster information:

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

    esp_zb_attribute_list_t* setup_basic_cluster(const std::string& modelIdStr, const std::string& manufacturerStr, const std::string& versionStr);

  private:
    static void zb_main_task(void* arg);
    static void on_attr_changed(uint8_t status, uint8_t endpoint, uint16_t clusterId, uint16_t attrId, void* newVal);

    void set_model_id(const std::string& modelIdStr);
    void set_manufacturer(const std::string& manufacturerStr);
    void set_version_details(const std::string& versionStr);
    void set_basic_attr(const std::string& basicAttrStr, std::vector<char>& basicAttrStrCache, esp_zb_zcl_basic_attr_t attrId);
};
} // namespace zigbee
