#include "zigbee/ZDevice.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"
#include "esp_zigbee_type.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zb_config_platform.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_carbon_dioxide_measurement.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_ota.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"
#include "zdo/esp_zigbee_zdo_common.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#endif
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <vector>

namespace zigbee {
const char* ZDevice::TAG = "ZDevice";

void ZDevice::set_led(std::shared_ptr<actuators::RgbLed> rgbLed) {
    this->rgbLed = std::move(rgbLed);
}

void ZDevice::set_led_color(const actuators::color_t& color) {
    // rgbLed->on(color);
}

const std::unique_ptr<ZDevice>& ZDevice::get_instance() {
    static const std::unique_ptr<ZDevice> instance = std::make_unique<ZDevice>();
    return instance;
}

esp_err_t ZDevice::power_saver_init() {
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {.max_freq_mhz = cur_cpu_freq_mhz,
                                 .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
                                 .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

void ZDevice::init(double temp, double hum, uint16_t co2) {
    ESP_LOGI(TAG, "Initializing ZigBee device...");

    // Set initial measurements. Values are multiplied by 100 to avoid floating point numbers.
    curTemp = static_cast<int16_t>(temp * 100);
    curHum = static_cast<int16_t>(hum * 100);

    // Calculation based on: https://www.rapidtables.com/convert/number/PPM_to_Percent.html
    curCo2 = static_cast<float_t>(static_cast<double>(co2) / 1000000.0);

    ESP_ERROR_CHECK(power_saver_init());

    esp_zb_platform_config_t config = {};
    config.radio_config.radio_mode = ZB_RADIO_MODE_NATIVE;
    config.host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE;

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(ZDevice::zb_main_task, "zigbee_main", 4096, this, 5, nullptr);
    ESP_LOGI(TAG, "ZigBee device initialized.");
}

void ZDevice::reset() const {
    ESP_LOGW(TAG, "Performing ZigBee factory reset...");
    esp_zb_factory_reset();
    ESP_LOGW(TAG, "ZigBee factory reset done.");
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
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, static_cast<void*>(&curTemp), false);
}

void ZDevice::update_hum(double hum) {
    curHum = static_cast<int16_t>(hum * 100); // Humidity values are multiplied by 100 to avoid floating point numbers
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, static_cast<void*>(&curHum), false);
}

void ZDevice::update_co2(uint16_t co2) {
    curCo2 = static_cast<float_t>(static_cast<double>(co2) / 1000000.0); // Calculation based on: https://www.rapidtables.com/convert/number/PPM_to_Percent.html
    esp_zb_zcl_set_attribute_val(ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, static_cast<void*>(&curCo2), false);
}

void ZDevice::zb_main_task(void* /*arg*/) {
    ESP_LOGI(TAG, "ZigBee task started.");

    // ZigBee power source:
    if (ZDevice::get_instance()->powerSourceBattery.is_powered()) {
        ZDevice::get_instance()->basicClusterConfig.power_source = 0b1 << 0x3; // Set as battery powered device
        ESP_LOGI(TAG, "ZigBee device categorized as battery powered.");
    } else {
        ESP_LOGI(TAG, "ZigBee device categorized as DC powered.");
    }

    // Enable zigbee light sleep:
    esp_zb_sleep_enable(true);

    // ZigBee end device config:
    esp_zb_cfg_t zb_nwk_cfg{};
    zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN;
    zb_nwk_cfg.nwk_cfg.zed_cfg.keep_alive = std::chrono::milliseconds(4000).count();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_sleep_set_threshold(std::chrono::milliseconds(50).count());

    // Cluster list and temperature:
    esp_zb_cluster_list_t* clusterList = ZDevice::get_instance()->setup_temp_sensor();

    // OTA:
    ZDevice::get_instance()->setup_ota_cluster();

    // Humidity:
    ZDevice::get_instance()->setup_hum_cluster();

    // CO2:
    ZDevice::get_instance()->setup_co2_cluster();

    // Basic information:
    ZDevice::get_instance()->setup_basic_cluster("HASS Env Sensor", "DOOP", "1.0.0");

    esp_zb_ep_list_t* endpointList = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(endpointList, clusterList, ENDPOINT_ID);
    esp_zb_device_register(endpointList);

    esp_zb_core_action_handler_register(ZDevice::on_zb_action);

    // Advertise on all channels:
    esp_zb_set_primary_network_channel_set(0x07FFF800);
    ESP_ERROR_CHECK(esp_zb_start(false));

    if (ZDevice::get_instance()->resetGpio.is_powered()) {
        ZDevice::get_instance()->reset();
    }

    esp_zb_main_loop_iteration();

    ESP_LOGI(TAG, "ZigBee task ended.");
}

esp_err_t ZDevice::on_zb_action(esp_zb_core_action_callback_id_t callback_id, const void* message) {
    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            return ZDevice::on_attr_changed(static_cast<const esp_zb_zcl_set_attr_value_message_t*>(message));
        case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
            return on_ota_upgrade_status(static_cast<const esp_zb_zcl_ota_upgrade_value_message_t*>(message));
        default:
            ESP_LOGI(TAG, "Receive unhandled Zigbee action(0x%x) callback", callback_id);
            break;
    }
    return ESP_OK;
}

esp_err_t ZDevice::on_attr_changed(const esp_zb_zcl_set_attr_value_message_t* msg) {
    ESP_LOGI(TAG, "Attribute changed. status: %d, endpoint: %d, clusterId: %d, attrId: %d, attrTypeId: %d", msg->info.status, msg->info.dst_endpoint, msg->info.cluster, msg->attribute.id, msg->attribute.data.type);
    return ESP_OK;
}

esp_err_t ZDevice::on_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* messsage) {
    if (messsage->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        if (messsage->upgrade_status == ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START) {
            ESP_LOGI(TAG, "OTA started.");
        } else if (messsage->upgrade_status == ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE) {
            ESP_LOGI(TAG, "OTA receiving...");
        } else if (messsage->upgrade_status == ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH) {
            ESP_LOGI(TAG, "OTA finished.");
        } else {
            ESP_LOGI(TAG, "OTA status: %d", messsage->upgrade_status);
        }
    }
    return ESP_OK;
}

void ZDevice::bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    zigbee::ZDevice::get_instance()->set_led_color(actuators::color_t{0, 0, 30});
    ESP_LOGI(TAG, "Rejoining network...");
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

esp_zb_cluster_list_t* ZDevice::setup_temp_sensor() {
    // Ensure this is called only once
    assert(!clusterList);
    assert(!tempAttrList);

    clusterListCfg.temp_meas_cfg.measured_value = curTemp;
    clusterListCfg.temp_meas_cfg.min_value = -40 * 100;
    clusterListCfg.temp_meas_cfg.max_value = 100 * 100;

    clusterList = esp_zb_temperature_sensor_clusters_create(&clusterListCfg);

    return clusterList;
}

void ZDevice::setup_basic_cluster(const std::string& modelIdStr, const std::string& manufacturerStr, const std::string& versionStr) {
    // Ensure this is called only once
    assert(!basicAttrList);
    assert(clusterList);

    basicAttrList = esp_zb_basic_cluster_create(&basicClusterConfig);
    set_model_id(modelIdStr);
    set_manufacturer(manufacturerStr);
    set_version_details(versionStr);
    ESP_ERROR_CHECK(esp_zb_cluster_list_update_cluster(clusterList, basicAttrList, ESP_ZB_ZCL_CLUSTER_ID_BASIC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void ZDevice::setup_ota_cluster() {
    // Ensure this is called only once
    assert(!otaAttrList);
    assert(clusterList);

    otaCfg.ota_upgrade_file_version = 1;
    otaCfg.ota_upgrade_manufacturer = 42;
    otaCfg.ota_upgrade_image_type = 0;
    otaAttrList = esp_zb_ota_cluster_create(&otaCfg);

    otaClientCfg.timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF;
    otaClientCfg.hw_version = 1;
    otaClientCfg.max_data_size = 64;
    esp_zb_ota_cluster_add_attr(otaAttrList, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, static_cast<void*>(&otaClientCfg));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(clusterList, otaAttrList, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
}

void ZDevice::setup_hum_cluster() {
    // Ensure this is called only once
    assert(!humAttrList);
    assert(clusterList);

    humCfg.min_value = 0;
    humCfg.max_value = 100;
    humCfg.measured_value = curHum;
    humAttrList = esp_zb_humidity_meas_cluster_create(&humCfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(clusterList, humAttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void ZDevice::setup_co2_cluster() {
    // Ensure this is called only once
    assert(!co2AttrList);
    assert(clusterList);

    co2Cfg.min_measured_value = static_cast<float_t>(static_cast<double>(400) / 1000000.0);
    co2Cfg.max_measured_value = static_cast<float_t>(static_cast<double>(5000) / 1000000.0);
    co2Cfg.measured_value = curCo2;

    co2AttrList = esp_zb_carbon_dioxide_measurement_cluster_create(&co2Cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(clusterList, co2AttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}
} // namespace zigbee

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sigType = static_cast<esp_zb_app_signal_type_t>(*signal_struct->p_app_signal);

    switch (sigType) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            zigbee::ZDevice::get_instance()->set_led_color(actuators::color_t{30, 0, 30});
            ESP_LOGI(zigbee::ZDevice::TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                zigbee::ZDevice::get_instance()->set_led_color(actuators::color_t{0, 0, 30});
                ESP_LOGI(zigbee::ZDevice::TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(zigbee::ZDevice::TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(zigbee::ZDevice::TAG, "Device rebooted or woke up from sleep.");
                    // This has to be done here as well. Else the device won't connect...
                    esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
                }
            } else {
                /* commissioning failed */
                zigbee::ZDevice::get_instance()->set_led_color(actuators::color_t{30, 0, 0});
                ESP_LOGW(zigbee::ZDevice::TAG, "Failed to initialize Zigbee stack (status: %s). Restarting...", esp_err_to_name(err_status));
                esp_restart();
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                zigbee::ZDevice::get_instance()->set_led_color(actuators::color_t{0, 30, 0});
                ESP_LOGI(zigbee::ZDevice::TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)", extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel());
            } else {
                zigbee::ZDevice::get_instance()->set_led_color(actuators::color_t{30, 20, 0});
                ESP_LOGI(zigbee::ZDevice::TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;

        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            ESP_LOGD(zigbee::ZDevice::TAG, "ZigBee device can sleep signal received.");
            esp_zb_sleep_now();
            break;

        case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            ESP_LOGI(zigbee::ZDevice::TAG, "ZigBee device production config ready.");
            esp_zb_set_node_descriptor_manufacturer_code(42);
            break;

        case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
            ESP_LOGI(zigbee::ZDevice::TAG, "ZigBee device unavailable with error status: %s.", esp_err_to_name(err_status));
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            const esp_zb_zdo_signal_leave_params_t* leaveParams = static_cast<esp_zb_zdo_signal_leave_params_t*>(esp_zb_app_signal_get_params(signal_struct->p_app_signal));
            if (leaveParams->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
                ESP_LOGW(zigbee::ZDevice::TAG, "ZigBee leave signal with device reset request and error status '%s' received.", esp_err_to_name(err_status));
                esp_zb_factory_reset();
            } else {
                ESP_LOGW(zigbee::ZDevice::TAG, "ZigBee leave signal with error status '%s' received.", esp_err_to_name(err_status));
            }
        } break;

        case ESP_ZB_NLME_STATUS_INDICATION:
            ESP_LOGI(zigbee::ZDevice::TAG, "%s NLME status '0x%x\n' with error status: %s", esp_zb_zdo_signal_to_string(sigType), *static_cast<uint8_t*>(esp_zb_app_signal_get_params(signal_struct->p_app_signal)), esp_err_to_name(err_status));
            break;

        default:
            ESP_LOGW(zigbee::ZDevice::TAG, "ZDO unhandled signal: %s, error status: %s", esp_zb_zdo_signal_to_string(sigType), esp_err_to_name(err_status));
            break;
    }
}
