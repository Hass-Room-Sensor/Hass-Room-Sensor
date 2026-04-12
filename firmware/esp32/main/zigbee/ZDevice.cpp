#include "zigbee/ZDevice.hpp"

#include "defs/DeviceDefs.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_type.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_carbon_dioxide_measurement.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_identify.h"
#include "zcl/esp_zigbee_zcl_on_off.h"
#include "zcl/esp_zigbee_zcl_ota.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zcl/esp_zigbee_zcl_pressure_meas.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"
#include "zdo/esp_zigbee_zdo_common.h"

#include <array>
#include <cassert>
#include <cstdio>

extern "C" {
/**
 * Internal ZBOSS helper used by Espressif's Zigbee stack to override the end-device node descriptor.
 *
 * Espressif's Arduino Zigbee layer calls this after stack startup so Home Assistant ZHA classifies
 * sleepy battery-powered end devices correctly and exposes the battery entity.
 * More/Source: https://github.com/espressif/arduino-esp32/blob/3.3.7/libraries/Zigbee/src/ZigbeeCore.cpp#L141-L144
 */
extern void zb_set_ed_node_descriptor(bool power_src, bool rx_on_when_idle, bool alloc_addr);
}

namespace zigbee {
const char* ZDevice::TAG = "ZDevice";

void ZDevice::set_device_listener(std::shared_ptr<devices::AbstractDeviceEventListener> listener) {
    deviceListener = std::move(listener);
}

const std::unique_ptr<ZDevice>& ZDevice::get_instance() {
    static const std::unique_ptr<ZDevice> instance = std::make_unique<ZDevice>();
    return instance;
}

void ZDevice::init() {
    if (eventGroup_ == nullptr) {
        eventGroup_ = xEventGroupCreate();
        assert(eventGroup_ != nullptr);
    }

    if (initialized_) {
        xEventGroupClearBits(eventGroup_, CONNECTED_BIT);
        return;
    }

    if (!deviceListener) {
        ESP_LOGW(TAG, "No device listener registered; hardware callbacks are disabled.");
    }

    esp_zb_platform_config_t config = {};
    config.radio_config.radio_mode = ZB_RADIO_MODE_NATIVE;
    config.host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE;
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(ZDevice::zb_main_task, "zigbee_main", 4096, this, 5, nullptr);
    initialized_ = true;
}

bool ZDevice::has_connection() const {
    return eventGroup_ != nullptr && (xEventGroupGetBits(eventGroup_) & CONNECTED_BIT) != 0;
}

bool ZDevice::wait_for_connection(std::chrono::milliseconds timeout) const {
    const EventBits_t bits = xEventGroupWaitBits(eventGroup_, CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout.count()));
    return (bits & CONNECTED_BIT) != 0;
}

void ZDevice::publish(const PublishRequest& request) {
    if (request.environmental) {
        if (request.environmental->temperature_centi_celsius) {
            update_temp(*request.environmental->temperature_centi_celsius);
        }
        if (request.environmental->humidity_centi_percent) {
            update_hum(*request.environmental->humidity_centi_percent);
        }
        if (request.environmental->pressure_deci_kpa) {
            update_pressure(*request.environmental->pressure_deci_kpa);
        }
        if (request.environmental->co2_ppm) {
            update_co2(*request.environmental->co2_ppm);
        }
    }

    if (request.battery) {
        update_battery(*request.battery);
    }
}

void ZDevice::reset() const {
    ESP_LOGW(TAG, "Performing Zigbee factory reset...");
    esp_zb_factory_reset();
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

void ZDevice::update_temp(int16_t temperatureCentiCelsius) {
    curTemp = temperatureCentiCelsius;
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, static_cast<void*>(&curTemp), false));
}

void ZDevice::update_hum(uint16_t humidityCentiPercent) {
    curHum = humidityCentiPercent;
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, static_cast<void*>(&curHum), false));
}

void ZDevice::update_pressure(int16_t pressureDeciKpa) {
    curPressure = pressureDeciKpa;
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, static_cast<void*>(&curPressure), false));
}

void ZDevice::update_co2(uint16_t co2Ppm) {
    if (co2Ppm < CO2_MIN_PPM || co2Ppm > CO2_MAX_PPM) {
        ESP_LOGW(TAG, "Skipping CO2 publish: %u ppm is outside Zigbee cluster range [%u, %u] ppm.", co2Ppm, CO2_MIN_PPM, CO2_MAX_PPM);
        return;
    }

    // Calculation based on: https://www.rapidtables.com/convert/number/PPM_to_Percent.html
    curCo2 = static_cast<float_t>(static_cast<double>(co2Ppm) / 1000000.0);
    const esp_err_t err = esp_zb_zcl_set_attribute_val(
        DEFAULT_ENDPOINT_ID.endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
        static_cast<void*>(&curCo2),
        false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update CO2 attribute for %u ppm: %s.", co2Ppm, esp_err_to_name(err));
    }
}

void ZDevice::update_battery(const models::QuantizedBatteryReading& battery) {
    curBatteryMv = static_cast<uint8_t>(battery.millivolts / 100U);       // ZigBee battery voltage is the multiple of 100 mV
    curBatteryPercentage = static_cast<uint8_t>(battery.percentage * 2U); // 0–200 in 0.5% steps

    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, static_cast<void*>(&curBatteryPercentage), false));
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, static_cast<void*>(&curBatteryMv), false));
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, static_cast<void*>(&curBatterySize), false));
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_A_HR_RATING_ID, static_cast<void*>(&curBatteryMAhRating), false));
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID, static_cast<void*>(&curBatteryRatedVoltage), false));
}

void ZDevice::zb_main_task(void* /*arg*/) {
    ESP_LOGI(TAG, "Zigbee task started.");

    const bool batteryPowered = ZDevice::get_instance()->powerSourceBattery.is_powered();
    if (batteryPowered) {
        ZDevice::get_instance()->basicClusterConfig.power_source = 0x03;
    } else {
        ZDevice::get_instance()->basicClusterConfig.power_source = DEFAULT_POWER_SOURCE;
    }

    // In light-sleep mode the Zigbee stack remains joined and may request sleepy-end-device light
    // sleep between polls. In deep-sleep mode the application tears the whole SoC down explicitly
    // after each publish cycle.
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
    esp_zb_sleep_enable(true);
#else
    esp_zb_sleep_enable(false);
#endif

    // ZigBee end device config:
    esp_zb_cfg_t networkConfig{};
    networkConfig.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED;
    networkConfig.install_code_policy = false;
    networkConfig.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN;
    networkConfig.nwk_cfg.zed_cfg.keep_alive = std::chrono::seconds(3).count();
    esp_zb_init(&networkConfig);
    ESP_LOGD(TAG, "esp_zb_init.");

    esp_zb_cluster_list_t* clusterList = ZDevice::get_instance()->setup_temp_sensor();
    ZDevice::get_instance()->setup_ota_cluster();
    ZDevice::get_instance()->setup_hum_cluster();
    ZDevice::get_instance()->setup_pressure_cluster();
    ZDevice::get_instance()->setup_co2_cluster();
    if (ZDevice::get_instance()->deviceListener && ZDevice::get_instance()->deviceListener->has_debug_led()) {
        ZDevice::get_instance()->setup_debug_led_cluster();
    }
    ZDevice::get_instance()->setup_battery_cluster();
    ESP_LOGD(TAG, "setup_battery_cluster.");

    std::array<char, 32> version{};
    snprintf(version.data(), version.size(), "%d.%d.%d", CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MAJOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MINOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_PATCH);
    ZDevice::get_instance()->setup_basic_cluster("HASS Env Sensor", "DOOP", std::string{version.data()});

    esp_zb_ep_list_t* endpointList = esp_zb_ep_list_create();
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpointList, clusterList, DEFAULT_ENDPOINT_ID));
    if (ZDevice::get_instance()->debugLedClusterList) {
        ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpointList, ZDevice::get_instance()->debugLedClusterList, LIGHT_ON_OFF_ENDPOINT_ID));
    }

    ESP_ERROR_CHECK(esp_zb_device_register(endpointList));
    ESP_LOGD(TAG, "esp_zb_device_register.");
    esp_zb_core_action_handler_register(ZDevice::on_zb_action);
    ESP_LOGD(TAG, "esp_zb_core_action_handler_register.");

    // Advertise on all 2.4 GHz channels:
    constexpr uint32_t EU_CHANNEL_MASK = 0x07FFF800; // bits 11–26 set since those are the channels used by ZigBee
    ESP_ERROR_CHECK(esp_zb_set_channel_mask(EU_CHANNEL_MASK));
    ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(EU_CHANNEL_MASK));
    ESP_ERROR_CHECK(esp_zb_set_secondary_network_channel_set(EU_CHANNEL_MASK));

    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGD(TAG, "esp_zb_start.");
    if (batteryPowered) {
        // ZHA derives battery-entity support from the node descriptor instead of the Basic cluster
        // power-source attribute. Apply the same end-device workaround Espressif uses in its Zigbee
        // Arduino wrapper so the node is interviewed as battery powered.
        // More/Source: https://github.com/espressif/arduino-esp32/blob/3.3.7/libraries/Zigbee/src/ZigbeeCore.cpp#L141-L144
        zb_set_ed_node_descriptor(true, false, true);
        ESP_LOGI(TAG, "Applied battery-powered end-device node descriptor workaround for ZHA.");
    }

    if (ZDevice::get_instance()->resetGpio.is_powered()) {
        ZDevice::get_instance()->reset();
    }

    ESP_LOGD(TAG, "esp_zb_stack_main_loop.");
    esp_zb_stack_main_loop();
}

esp_err_t ZDevice::on_zb_action(esp_zb_core_action_callback_id_t callbackId, const void* message) {
    switch (callbackId) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            return ZDevice::on_attr_changed(static_cast<const esp_zb_zcl_set_attr_value_message_t*>(message));
        case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
            return on_ota_upgrade_status(static_cast<const esp_zb_zcl_ota_upgrade_value_message_t*>(message));
        case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
            return on_ota_upgrade_query_image_resp(static_cast<const esp_zb_zcl_ota_upgrade_query_image_resp_message_t*>(message));
        default:
            ESP_LOGW(TAG, "Unhandled Zigbee action callback: 0x%x", callbackId);
            return ESP_OK;
    }
}

esp_err_t ZDevice::on_attr_changed(const esp_zb_zcl_set_attr_value_message_t* message) {
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY && message->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
        uint16_t identifyTime = 0;
        if (message->attribute.data.value) {
            identifyTime = *static_cast<uint16_t*>(message->attribute.data.value);
        }

        if (ZDevice::get_instance()->deviceListener) {
            ZDevice::get_instance()->deviceListener->on_identify(identifyTime);
        }
        return ESP_OK;
    }

    if (message->info.dst_endpoint == LIGHT_ON_OFF_ENDPOINT_ID.endpoint && message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF && message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
        bool& curDebugLed = ZDevice::get_instance()->curDebugLed;
        curDebugLed = message->attribute.data.value ? *static_cast<bool*>(message->attribute.data.value) : curDebugLed;
        if (ZDevice::get_instance()->deviceListener && ZDevice::get_instance()->deviceListener->has_debug_led()) {
            ZDevice::get_instance()->deviceListener->set_debug_led(curDebugLed);
        }
    }

    return ESP_OK;
}

esp_err_t ZDevice::on_ota_upgrade_data_message(uint32_t totalSize, void* payload, uint16_t payloadSize, void** outBuffer, uint16_t* outLength) {
    void* dataBuffer = nullptr;
    uint16_t dataLength = 0;

    if (!otaStatus.tagReceived) {
        if (!payload || !outLength || !outBuffer || payloadSize <= OTA_ELEMENT_HEADER_LEN) {
            return ESP_ERR_INVALID_ARG;
        }

        uint8_t* raw = static_cast<uint8_t*>(payload);
        uint32_t length = 0;
        memcpy(&otaStatus.tag, raw, sizeof(otaStatus.tag));
        memcpy(&length, raw + sizeof(otaStatus.tag), sizeof(length));

        if ((length + OTA_ELEMENT_HEADER_LEN) != totalSize) {
            return ESP_ERR_INVALID_ARG;
        }

        otaStatus.tagReceived = true;
        dataBuffer = raw + OTA_ELEMENT_HEADER_LEN;
        dataLength = payloadSize - OTA_ELEMENT_HEADER_LEN;
    } else {
        dataBuffer = payload;
        dataLength = payloadSize;
    }

    if (otaStatus.tag != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *outBuffer = dataBuffer;
    *outLength = dataLength;
    return ESP_OK;
}

esp_err_t ZDevice::on_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* message) {
    static uint32_t totalSize = 0;
    static uint32_t writtenTotal = 0;
    static esp_ota_handle_t otaHandle{};
    static const esp_partition_t* otaPartition{nullptr};

    if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_OK;
    }

    switch (message->upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            otaPartition = esp_ota_get_next_update_partition(nullptr);
            ESP_ERROR_CHECK(esp_ota_begin(otaPartition, 0, &otaHandle));
            ZDevice::get_instance()->set_device_state(ZigbeeDeviceState::OTA);
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            totalSize = message->ota_header.image_size;
            if (message->payload_size > 0 && message->payload) {
                void* payload = nullptr;
                uint16_t payloadSize = 0;
                ESP_RETURN_ON_ERROR(ZDevice::get_instance()->on_ota_upgrade_data_message(totalSize, message->payload, message->payload_size, &payload, &payloadSize), TAG, "Failed to parse OTA element.");
                if (payloadSize > 0) {
                    ESP_RETURN_ON_ERROR(esp_ota_write(otaHandle, payload, payloadSize), TAG, "esp_ota_write failed.");
                    writtenTotal += payloadSize;
                }
            }
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ZDevice::get_instance()->otaStatus.tagReceived = false;
            writtenTotal = 0;
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_RETURN_ON_ERROR(esp_ota_end(otaHandle), TAG, "Failed to end OTA.");
            ESP_RETURN_ON_ERROR(esp_ota_set_boot_partition(otaPartition), TAG, "Failed to select OTA partition.");
            ESP_LOGI(TAG, "OTA finished: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes", message->ota_header.file_version, message->ota_header.manufacturer_code, message->ota_header.image_type, message->ota_header.image_size);
            esp_restart();
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
            if (otaHandle) {
                esp_ota_end(otaHandle); // ignore return; best-effort
                otaHandle = 0;
            }
            writtenTotal = 0;
            ZDevice::get_instance()->otaStatus.tagReceived = false;
            ZDevice::get_instance()->set_device_state(ZigbeeDeviceState::CONNECTED);
            break;

        default:
            ESP_LOGW(TAG, "OTA unknown status: %d", message->upgrade_status);
            break;
    }

    return ESP_OK;
}

esp_err_t ZDevice::on_ota_upgrade_query_image_resp(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t* message) {
    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Queried OTA image version 0x%lx from 0x%04hx.", message->file_version, message->server_addr.u.short_addr);
    }
    return ESP_OK;
}

void ZDevice::bdb_start_top_level_commissioning_cb(uint8_t modeMask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(modeMask));
}

esp_zb_cluster_list_t* ZDevice::setup_temp_sensor() {
    assert(!clusterList);
    assert(!tempAttrList);

    clusterListCfg.temp_meas_cfg.measured_value = curTemp;
    clusterListCfg.temp_meas_cfg.min_value = -40 * 100;
    clusterListCfg.temp_meas_cfg.max_value = 85 * 100;
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

    otaCfg.ota_upgrade_file_version = (CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MAJOR << 24) | (CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MINOR << 12) | (CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_PATCH);
    otaCfg.ota_upgrade_manufacturer = CONFIG_HASS_ENVIRONMENT_SENSOR_OTA_MANUFACTURER;
    otaCfg.ota_upgrade_image_type = OTA_IMAGE_TYPE;

    otaAttrList = esp_zb_ota_cluster_create(&otaCfg);
    otaClientCfg.timer_query = 1;
    otaClientCfg.hw_version = 1;
    otaClientCfg.max_data_size = 223;

    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(otaAttrList, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, static_cast<void*>(&otaClientCfg)));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(otaAttrList, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, static_cast<void*>(&otaUpgradeServerAddr)));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(otaAttrList, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, static_cast<void*>(&otaUpgradeServerEp)));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(clusterList, otaAttrList, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
}

void ZDevice::setup_hum_cluster() {
    // Ensure this is called only once
    assert(!humAttrList);
    assert(clusterList);

    humCfg.measured_value = curHum;
    humCfg.min_value = 0;
    humCfg.max_value = 10000;
    humAttrList = esp_zb_humidity_meas_cluster_create(&humCfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(clusterList, humAttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void ZDevice::setup_pressure_cluster() {
    assert(!pressureAttrList);
    assert(clusterList);

    pressureCfg.measured_value = curPressure;
    pressureCfg.min_value = 300;
    pressureCfg.max_value = 1100;
    pressureAttrList = esp_zb_pressure_meas_cluster_create(&pressureCfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_pressure_meas_cluster(clusterList, pressureAttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void ZDevice::setup_co2_cluster() {
    // Ensure this is called only once
    assert(!co2AttrList);
    assert(clusterList);

    // Zigbee stores CO2 as a volumetric fraction, so the ppm limits are converted to "fraction of one".
    co2Cfg.min_measured_value = static_cast<float_t>(static_cast<double>(CO2_MIN_PPM) / 1000000.0);
    co2Cfg.max_measured_value = static_cast<float_t>(static_cast<double>(CO2_MAX_PPM) / 1000000.0);
    co2Cfg.measured_value = curCo2;
    co2AttrList = esp_zb_carbon_dioxide_measurement_cluster_create(&co2Cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(clusterList, co2AttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void ZDevice::setup_debug_led_cluster() {
    assert(!debugLedClusterList);
    if (!deviceListener || !deviceListener->has_debug_led()) {
        ESP_LOGW(TAG, "Debug LED cluster setup requested without a matching device listener.");
        return;
    }

    debugLedCfg.basic_cfg.power_source = DEFAULT_POWER_SOURCE;
    debugLedCfg.basic_cfg.zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    debugLedClusterList = esp_zb_on_off_light_clusters_create(&debugLedCfg);
    curDebugLed = deviceListener->is_debug_led_enabled();
}

void ZDevice::setup_battery_cluster() {
    // Ensure this is called only once
    assert(!powerAttrList);
    assert(clusterList);

    powerCfg.main_voltage = 37;     // 3.7V battery in 100 mV steps
    powerCfg.main_voltage_max = 55; // 5.5V battery in 100 mV steps
    powerCfg.main_voltage_min = 32; // 3.2V battery in 100 mV steps

    powerAttrList = esp_zb_power_config_cluster_create(&powerCfg);
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &curBatteryPercentage));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &curBatteryMv));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, &curBatterySize));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_A_HR_RATING_ID, &curBatteryMAhRating));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID, &curBatteryRatedVoltage));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(clusterList, powerAttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void ZDevice::set_device_state(ZigbeeDeviceState newState) {
    if (deviceState == newState) {
        return;
    }

    deviceState = newState;
    if (eventGroup_) {
        if (deviceState == ZigbeeDeviceState::CONNECTED) {
            xEventGroupSetBits(eventGroup_, CONNECTED_BIT);
        } else {
            xEventGroupClearBits(eventGroup_, CONNECTED_BIT);
        }
    }
    if (deviceListener) {
        deviceListener->on_device_state_changed(deviceState);
    }
}

void ZDevice::on_connected() {
    set_device_state(ZigbeeDeviceState::CONNECTED);

    esp_zb_ieee_addr_t extendedPanId;
    esp_zb_get_extended_pan_id(extendedPanId);
    ESP_LOGI(TAG, "Connected (PAN ID 0x%04hx, channel %d).", esp_zb_get_pan_id(), esp_zb_get_current_channel());
}
} // namespace zigbee

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signalStruct) {
    const esp_err_t errorStatus = signalStruct->esp_err_status;
    const auto signalType = static_cast<esp_zb_app_signal_type_t>(*signalStruct->p_app_signal);

    switch (signalType) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZigbeeDeviceState::CONNECTING);
            ESP_LOGI(zigbee::ZDevice::TAG, "Zigbee stack initialized after startup skipped");
            esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (errorStatus == ESP_OK) {
                ESP_LOGI(zigbee::ZDevice::TAG, "Device state: %s", esp_zb_bdb_is_factory_new() ? "factory new" : "configured");
                if (esp_zb_bdb_is_factory_new()) {
                    zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZigbeeDeviceState::SETUP);
                    ESP_LOGI(zigbee::ZDevice::TAG, "Scanning for available Zigbee networks and joining one that's open to new devices...");
                    esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
                } else {
                    zigbee::ZDevice::get_instance()->on_connected();
                }
            } else {
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZigbeeDeviceState::CONNECTING);
                ESP_LOGW(zigbee::ZDevice::TAG, "Failed to initialize Zigbee stack (status: %s).", esp_err_to_name(errorStatus));
                ESP_LOGI(zigbee::ZDevice::TAG, "Scanning for available Zigbee networks and joining one that's open to new devices....");
                esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (errorStatus == ESP_OK) {
                zigbee::ZDevice::get_instance()->on_connected();
            } else {
                if (esp_zb_bdb_is_factory_new()) {
                    zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZigbeeDeviceState::SETUP);
                    ESP_LOGI(zigbee::ZDevice::TAG, "Scanning for available networks to join was not successful (status: %s). Attempting to join again...", esp_err_to_name(errorStatus));
                } else {
                    zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZigbeeDeviceState::CONNECTING);
                    ESP_LOGI(zigbee::ZDevice::TAG, "Rejoining a known network was not successful (status: %s). Attempting to join again...", esp_err_to_name(errorStatus));
                }
                esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;

        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            ESP_LOGD(zigbee::ZDevice::TAG, "Zigbee stack entered CAN_SLEEP during the current wake session.");
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
            esp_zb_sleep_now();
#endif
            break;

        case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
            zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZigbeeDeviceState::CONNECTING);
            ESP_LOGI(zigbee::ZDevice::TAG, "ZigBee device unavailable (status: %s). Trying to rejoin...", esp_err_to_name(errorStatus));
            esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            break;

        case ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE:
            if (errorStatus == ESP_OK) {
                zigbee::ZDevice::get_instance()->on_connected();
                ESP_LOGI(zigbee::ZDevice::TAG, "TC rejoin completed.");
            } else {
                ESP_LOGW(zigbee::ZDevice::TAG, "TC rejoin failed: %s", esp_err_to_name(errorStatus));
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            // Espressif emits this during normal startup even when no production config blob is present.
            // Treat it as informational instead of warning noise and still set the manufacturer code so
            // the node descriptor remains stable for interview/debugging purposes.
            ESP_LOGI(zigbee::ZDevice::TAG, "Production configuration is %s.", errorStatus == ESP_OK ? "ready" : "not present");
            esp_zb_set_node_descriptor_manufacturer_code(CONFIG_HASS_ENVIRONMENT_SENSOR_OTA_MANUFACTURER);
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            const auto* leaveParams = static_cast<esp_zb_zdo_signal_leave_params_t*>(esp_zb_app_signal_get_params(signalStruct->p_app_signal));
            if (leaveParams && leaveParams->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZigbeeDeviceState::SETUP);
                ESP_LOGW(zigbee::ZDevice::TAG, "ZigBee leave signal with device reset request and error status '%s' received.", esp_err_to_name(errorStatus));
                esp_zb_factory_reset();
            } else {
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZigbeeDeviceState::CONNECTING);
                ESP_LOGW(zigbee::ZDevice::TAG, "ZigBee leave signal with error status '%s' received.", esp_err_to_name(errorStatus));
            }
        } break;

        case ESP_ZB_NLME_STATUS_INDICATION:
            ESP_LOGI(zigbee::ZDevice::TAG, "%s NLME status '0x%x' with error status: %s", esp_zb_zdo_signal_to_string(static_cast<esp_zb_app_signal_type_t>(*signalStruct->p_app_signal)), *static_cast<uint8_t*>(esp_zb_app_signal_get_params(signalStruct->p_app_signal)), esp_err_to_name(errorStatus));
            break;

        default:
            ESP_LOGW(zigbee::ZDevice::TAG, "Unhandled ZDO signal %s (status: %s).", esp_zb_zdo_signal_to_string(signalType), esp_err_to_name(errorStatus));
            break;
    }
}
