#include "zigbee/ZDevice.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_type.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zb_config_platform.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"
#include "zdo/esp_zigbee_zdo_common.h"
#include <array>
#include <cassert>
#include <chrono>
#include <vector>

namespace zigbee {
const char* ZDevice::TAG = "ZDevice";

const std::unique_ptr<ZDevice>& ZDevice::get_instance() {
    static const std::unique_ptr<ZDevice> instance = std::make_unique<ZDevice>();
    return instance;
}

void ZDevice::init(double temp, double hum) {
    ESP_LOGI(TAG, "Initializing ZigBee device...");

    // Set initial measurements
    curTemp = static_cast<int16_t>(temp * 100);
    curHum = static_cast<int16_t>(hum * 100);

    esp_zb_platform_config_t config = {};
    config.radio_config.radio_mode = RADIO_MODE_NATIVE;
    config.host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(ZDevice::zb_main_task, "Zigbee_main", 4096, this, 5, nullptr);
    ESP_LOGI(TAG, "ZigBee device initialized.");
}

void ZDevice::set_basic_attr(const std::string& basicAttrStr, std::vector<char>& basicAttrStrCache, esp_zb_zcl_basic_attr_t attrId) {
    assert(basicAttrStr.length() <= 0xFF);
    // The first byte of the attribute string is the length of the following string.
    // Source: https://github.com/espressif/esp-idf/issues/10662#issuecomment-1424903170
    basicAttrStrCache.push_back(static_cast<char>(basicAttrStr.length()));
    basicAttrStrCache.insert(basicAttrStrCache.end(), basicAttrStr.begin(), basicAttrStr.end());
    esp_zb_basic_cluster_add_attr(basicAttrList, attrId, basicAttrStrCache.data());
}

void ZDevice::set_model_id(const std::string& modelIdStr) {
    set_basic_attr(modelIdStr, modelId, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID);
}

void ZDevice::set_manufacturer(const std::string& manufacturerStr) {
    set_basic_attr(manufacturerStr, manufacturer, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID);
}

void ZDevice::set_version_details(const std::string& versionStr) {
    set_basic_attr(versionStr, version, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_VERSION_DETAILS_ID);
}

void ZDevice::update_temp(double temp) {
    curTemp = static_cast<int16_t>(temp * 100); // Temperature values are multiplied by 100 to avoid floating point numbers
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, static_cast<void*>(&curTemp), false);
}

void ZDevice::update_hum(double hum) {
    curHum = static_cast<int16_t>(hum * 100); // Humidity values are multiplied by 100 to avoid floating point numbers
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, static_cast<void*>(&curHum), false);
}

void ZDevice::zb_main_task(void* /*arg*/) {
    ESP_LOGI(TAG, "ZigBee task started.");

    // ZigBee end device config:
    esp_zb_cfg_t zb_nwk_cfg{};
    zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN;
    zb_nwk_cfg.nwk_cfg.zed_cfg.keep_alive = std::chrono::milliseconds(3000).count();
    esp_zb_init(&zb_nwk_cfg);

    // Clusters:

    // Temperature:
    esp_zb_cluster_list_t* tempClusterList = ZDevice::get_instance()->setup_temp_cluster();

    // Basic information:
    esp_zb_attribute_list_t* basicAttrList = ZDevice::get_instance()->setup_basic_cluster("HASS Env Sensor", "DOOP", "1.0.0");
    esp_zb_cluster_list_update_basic_cluster(tempClusterList, basicAttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Humidity:
    esp_zb_attribute_list_t* humAttrList = ZDevice::get_instance()->setup_hum_cluster();
    esp_zb_cluster_list_add_humidity_meas_cluster(tempClusterList, humAttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t* endpointList = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(endpointList, tempClusterList, ENDPOINT_ID, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID);
    esp_zb_device_register(endpointList);

    esp_zb_device_add_set_attr_value_cb(ZDevice::on_attr_changed);

    // Set the device config:
    // esp_zb_color_dimmable_light_cfg_t light_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
    // esp_zb_ep_list_t* esp_zb_color_dimmable_light_ep = esp_zb_color_dimmable_light_ep_create(ENDPOINT_ID, &light_cfg);
    // esp_zb_device_register(esp_zb_color_dimmable_light_ep);
    // esp_zb_device_add_set_attr_value_cb(ZDevice::on_attr_changed);

    // Advertise on all channels:
    esp_zb_set_primary_network_channel_set(0x07FFF800);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();

    ESP_LOGI(TAG, "ZigBee task ended.");
}

int ZDevice::on_attr_changed(esp_zb_zcl_set_attr_value_message_s msg) {
    ESP_LOGI(TAG, "Attribute changed. status: %d, endpoint: %d, clusterId: %d, attrId: %d, ", msg.info.status, msg.info.dst_endpoint, msg.info.cluster, msg.attribute);
    return 0; // Not handled
}

void ZDevice::bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

esp_zb_attribute_list_t* ZDevice::setup_basic_cluster(const std::string& modelIdStr, const std::string& manufacturerStr, const std::string& versionStr) {
    // Ensure this is called only once
    assert(!basicAttrList);

    basicAttrList = esp_zb_basic_cluster_create(&basicClusterConfig);
    set_model_id(modelIdStr);
    set_manufacturer(manufacturerStr);
    set_version_details(versionStr);
    return basicAttrList;
}

esp_zb_cluster_list_t* ZDevice::setup_temp_cluster() {
    tempCfg.temp_meas_cfg.measured_value = curTemp;
    tempCfg.temp_meas_cfg.min_value = -40 * 100;
    tempCfg.temp_meas_cfg.max_value = 100 * 100;

    tempClusterList = esp_zb_temperature_sensor_clusters_create(&tempCfg);
    tempAttrList = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    ESP_ERROR_CHECK(esp_zb_temperature_meas_cluster_add_attr(tempAttrList, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, static_cast<void*>(&curTemp)));
    return tempClusterList;
}

esp_zb_attribute_list_t* ZDevice::setup_hum_cluster() {
    humCfg.min_value = 0;
    humCfg.max_value = 0;
    humCfg.measured_value = curHum;

    humAttrList = esp_zb_humidity_meas_cluster_create(&humCfg);
    esp_zb_humidity_meas_cluster_add_attr(humAttrList, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, static_cast<void*>(&curHum));
    return humAttrList;
}
} // namespace zigbee

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = static_cast<esp_zb_app_signal_type_t>(*signal_struct->p_app_signal);

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(zigbee::ZDevice::TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(zigbee::ZDevice::TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                /* commissioning failed */
                ESP_LOGW(zigbee::ZDevice::TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(zigbee::ZDevice::TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)", extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel());
            } else {
                ESP_LOGI(zigbee::ZDevice::TAG, "Network steering was not successful (status: %d)", err_status);
                esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;

        default:
            ESP_LOGI(zigbee::ZDevice::TAG, "ZDO signal: %d, status: %d", sig_type, err_status);
            break;
    }
}
