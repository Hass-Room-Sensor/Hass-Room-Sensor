#pragma once

#include <cstdint>

#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include <memory>
#include <string>
#include <sys/_stdint.h>
#include <vector>

namespace zigbee {
class ZDevice {
  public:
    static const char* TAG;

  private:
    static constexpr int ENDPOINT_ID = 10;

    /**
     * The ZigBee spec defines the power source (8 bits) in "3.2.2.2.8 PowerSource Attribute".
     * https://zigbeealliance.org/wp-content/uploads/2019/12/07-5123-06-zigbee-cluster-library-specification.pdf
     * Bit | Value
     * 0x0 | Unknown
     * 0x1 | Mains (single phase)
     * 0x2 | Mains (3 phases)
     * 0x3 | Battery
     * 0x4 | DC source
     * 0x5 | Emergency mains constantly powered
     * 0x6 | Emergency mains and transfer switch
     * 0x7 | Has a secondary power backup.
     **/
    static constexpr uint8_t POWER_SOURCE = 0b00001000; // Bit 0x4 is set to indicate DC source

    // Basic cluster information:
    esp_zb_basic_cluster_cfg_t basicClusterConfig{ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE, POWER_SOURCE};
    esp_zb_attribute_list_t* basicAttrList{nullptr};

    std::vector<char> modelId;
    std::vector<char> manufacturer;
    std::vector<char> version;

    // Temperature cluster information:
    esp_zb_temperature_sensor_cfg_t tempCfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    esp_zb_cluster_list_t* tempClusterList{nullptr};
    esp_zb_attribute_list_t* tempAttrList{nullptr};
    int16_t curTemp{-1};

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
    esp_zb_cluster_list_t* setup_temp_cluster(double temp);

    void update_temp(double temp);

  private:
    static void zb_main_task(void* arg);
    static void on_attr_changed(uint8_t status, uint8_t endpoint, uint16_t clusterId, uint16_t attrId, void* newVal);

    void set_model_id(const std::string& modelIdStr);
    void set_manufacturer(const std::string& manufacturerStr);
    void set_version_details(const std::string& versionStr);
    void set_basic_attr(const std::string& basicAttrStr, std::vector<char>& basicAttrStrCache, esp_zb_zcl_basic_attr_t attrId);
};
} // namespace zigbee
