#pragma once

#include "actuators/RgbLed.hpp"
#include "sensors/GpioInput.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

extern "C" {
#include "esp_zigbee_ota.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "hal/gpio_types.h"
#include "zcl/esp_zigbee_zcl_basic.h"
}

namespace zigbee {
class ZDevice {
  public:
    static const char* TAG;

  private:
    static constexpr esp_zb_endpoint_config_t ENDPOINT_ID{10, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, 12};

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
    static constexpr uint8_t DEFAULT_POWER_SOURCE = 0b1 << 0x4; // Bit 0x4 is set to indicate DC source

    // Basic cluster information:
    esp_zb_basic_cluster_cfg_t basicClusterConfig{ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE, DEFAULT_POWER_SOURCE};
    esp_zb_attribute_list_t* basicAttrList{nullptr};

    std::vector<char> modelId;
    std::vector<char> manufacturer;
    std::vector<char> version;

    // Cluster list:
    esp_zb_cluster_list_t* clusterList{nullptr};
    esp_zb_temperature_sensor_cfg_t clusterListCfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();

    // OTA:
    esp_zb_ota_cluster_cfg_t otaCfg{};
    esp_zb_zcl_ota_upgrade_client_variable_t otaClientCfg{};
    esp_zb_attribute_list_t* otaAttrList{nullptr};

    // Temperature cluster information:
    esp_zb_temperature_meas_cluster_cfg_t tempCfg{};
    esp_zb_attribute_list_t* tempAttrList{nullptr};
    int16_t curTemp{-1};

    // Humidity cluster information:
    esp_zb_humidity_meas_cluster_cfg_t humCfg{};
    esp_zb_attribute_list_t* humAttrList{nullptr};
    int16_t curHum{-1};

    // Carbon Dioxide information:
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2Cfg{};
    esp_zb_attribute_list_s* co2AttrList{nullptr};
    float_t curCo2{-1};

    std::shared_ptr<actuators::RgbLed> rgbLed{nullptr};

    // Reset GPIO used for factory resetting the ZigBee stack.
    sensors::GpioInput resetGpio{GPIO_NUM_1};
    // If set to high the ZigBee stack will be initialized as battery connected device.
    sensors::GpioInput powerSourceBattery{GPIO_NUM_3};

  public:
    ZDevice() = default;
    ZDevice(ZDevice&&) = default;
    ZDevice(const ZDevice&) = default;
    ZDevice& operator=(ZDevice&&) = default;
    ZDevice& operator=(const ZDevice&) = default;
    ~ZDevice() = default;

    static const std::unique_ptr<ZDevice>& get_instance();

    void init(double temp, double hum, uint16_t co2);

    static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
    static esp_err_t deferred_driver_init();

    esp_zb_cluster_list_t* setup_temp_sensor();

    void setup_basic_cluster(const std::string& modelIdStr, const std::string& manufacturerStr, const std::string& versionStr);
    void setup_ota_cluster();
    void setup_hum_cluster();
    void setup_co2_cluster();

    void update_temp(double temp);
    void update_hum(double hum);
    void update_co2(uint16_t co2);
    void set_led(std::shared_ptr<actuators::RgbLed> rgbLed);
    void set_led_color(const actuators::color_t& color);

    void reset() const;

  private:
    static esp_err_t power_saver_init();

    static void zb_main_task(void* arg);
    static esp_err_t on_zb_action(esp_zb_core_action_callback_id_t callback_id, const void* message);
    static esp_err_t on_attr_changed(const esp_zb_zcl_set_attr_value_message_t* msg);
    static esp_err_t on_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* messsage);

    void set_model_id(const std::string& modelIdStr);
    void set_manufacturer(const std::string& manufacturerStr);
    void set_version_details(const std::string& versionStr);
    void set_basic_attr(const std::string& basicAttrStr, std::vector<char>& basicAttrStrCache, esp_zb_zcl_basic_attr_t attrId);
};
} // namespace zigbee
