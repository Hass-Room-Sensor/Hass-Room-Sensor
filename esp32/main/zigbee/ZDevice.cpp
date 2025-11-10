#include "zigbee/ZDevice.hpp"
#include "defs/DeviceDefs.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"
#include "esp_zigbee_type.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_carbon_dioxide_measurement.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_identify.h"
#include "zcl/esp_zigbee_zcl_ota.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"
#include "zdo/esp_zigbee_zdo_common.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <esp_check.h>
#include <optional>
#include <vector>

namespace zigbee {
const char* ZDevice::TAG = "ZDevice";

void ZDevice::set_led(std::shared_ptr<actuators::RgbLed> rgbLed) {
    this->rgbLed = std::move(rgbLed);
}

void ZDevice::set_led(std::shared_ptr<actuators::Led> led) {
    this->led = std::move(led);
}

void ZDevice::set_led_color(const actuators::color_t& color) {
    if (rgbLed) {
        rgbLed->on(color);
    }
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
#else
                                 .light_sleep_enable = false
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
    esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, static_cast<void*>(&curTemp), false);
}

void ZDevice::update_hum(double hum) {
    curHum = static_cast<int16_t>(hum * 100); // Humidity values are multiplied by 100 to avoid floating point numbers
    esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, static_cast<void*>(&curHum), false);
}

void ZDevice::update_co2(uint16_t co2) {
    curCo2 = static_cast<float_t>(static_cast<double>(co2) / 1000000.0); // Calculation based on: https://www.rapidtables.com/convert/number/PPM_to_Percent.html
    esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, static_cast<void*>(&curCo2), false);
}

void ZDevice::update_battery(uint8_t batteryPercentage, uint16_t batteryMv) {
    curBatteryMv = batteryMv / 100;               // ZigBee battery voltage is the multiple of 100 mV
    curBatteryPercentage = batteryPercentage * 2; // 0–200 in 0.5% steps
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, static_cast<void*>(&curBatteryPercentage), false));
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, static_cast<void*>(&curBatteryMv), false));
}

void ZDevice::zb_main_task(void* /*arg*/) {
    ESP_LOGI(TAG, "ZigBee task started.");

    // ZigBee power source:
    if (ZDevice::get_instance()->powerSourceBattery.is_powered()) {
        ZDevice::get_instance()->basicClusterConfig.power_source = 0b1 << 0x3; // Set as battery powered device
        ESP_LOGI(TAG, "ZigBee device categorized as battery powered.");
    } else {
        ZDevice::get_instance()->basicClusterConfig.power_source = DEFAULT_POWER_SOURCE;
        ESP_LOGI(TAG, "ZigBee device categorized as DC powered.");
    }

    // By default sleep is disabled. It will be enabled as soon as the device is connected to a ZigBee network.
    esp_zb_sleep_enable(false);
    esp_zb_sleep_set_threshold(std::chrono::milliseconds(20).count());
    ESP_LOGI(TAG, "ZigBee sleep threshold set.");

    // ZigBee end device config:
    esp_zb_cfg_t zb_nwk_cfg{};
    zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN;
    zb_nwk_cfg.nwk_cfg.zed_cfg.keep_alive = std::chrono::milliseconds(3000).count();
    esp_zb_init(&zb_nwk_cfg);
    ESP_LOGI(TAG, "ZigBee init done.");

    // Cluster list and temperature:
    esp_zb_cluster_list_t* clusterList = ZDevice::get_instance()->setup_temp_sensor();

    // OTA:
    ZDevice::get_instance()->setup_ota_cluster();

    // Humidity:
    ZDevice::get_instance()->setup_hum_cluster();

    // CO2:
    ZDevice::get_instance()->setup_co2_cluster();

    // Debug LED ON/OFF:
    if (ZDevice::get_instance()->rgbLed) {
        ZDevice::get_instance()->setup_debug_led_cluster();
    }

    // Battery:
    ZDevice::get_instance()->setup_battery_cluster();

    // Basic information:
    std::array<char, 32> version{};
    snprintf(version.data(), version.size(), "%d.%d.%d", CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MAJOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MINOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_PATCH);
    ZDevice::get_instance()->setup_basic_cluster("HASS Env Sensor", "DOOP", std::string{version.data()});

    // Generic endpoint
    esp_zb_ep_list_t* endpointList = esp_zb_ep_list_create();
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpointList, clusterList, DEFAULT_ENDPOINT_ID));

    // Light endpoint
    if (ZDevice::get_instance()->debugLedClusterList) {
        ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpointList, ZDevice::get_instance()->debugLedClusterList, LIGHT_ON_OFF_ENDPOINT_ID));
    }

    ESP_ERROR_CHECK(esp_zb_device_register(endpointList));

    esp_zb_core_action_handler_register(ZDevice::on_zb_action);

    // Advertise on all 2.4 GHz channels:
    constexpr uint32_t EU_CHANNEL_MASK = 0x07FFF800; // bits 11–26 set since those are the channels used by ZigBee in the EU
    ESP_ERROR_CHECK(esp_zb_set_channel_mask(EU_CHANNEL_MASK));
    ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(EU_CHANNEL_MASK));
    ESP_ERROR_CHECK(esp_zb_set_secondary_network_channel_set(EU_CHANNEL_MASK));

    // Start:
    ESP_ERROR_CHECK(esp_zb_start(false));

    if (ZDevice::get_instance()->resetGpio.is_powered()) {
        ZDevice::get_instance()->reset();
    }

    esp_zb_stack_main_loop();

    ESP_LOGI(TAG, "ZigBee task ended.");
}

esp_err_t ZDevice::on_zb_action(esp_zb_core_action_callback_id_t callback_id, const void* message) {
    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            return ZDevice::on_attr_changed(static_cast<const esp_zb_zcl_set_attr_value_message_t*>(message));
        case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
            return on_ota_upgrade_status(static_cast<const esp_zb_zcl_ota_upgrade_value_message_t*>(message));
        case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
            return on_ota_upgrade_query_image_resp(static_cast<const esp_zb_zcl_ota_upgrade_query_image_resp_message_t*>(message));
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
            // const esp_zb_zcl_cmd_default_resp_message_t* msg = static_cast<const esp_zb_zcl_cmd_default_resp_message_t*>(message);
            ESP_LOGW(TAG, "Receive unhandled Zigbee 'ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID' callback.");
            break;
        default:
            ESP_LOGW(TAG, "Receive unhandled Zigbee action(0x%x) callback", callback_id);
            break;
    }
    return ESP_OK;
}

esp_err_t ZDevice::on_attr_changed(const esp_zb_zcl_set_attr_value_message_t* msg) {
    // Identify cluster
    if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY && msg->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID && msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
        uint16_t identifyTime = 0;
        if (msg->attribute.data.value) {
            identifyTime = *static_cast<uint16_t*>(msg->attribute.data.value);
        }

        if (zigbee::ZDevice::get_instance()->led) {
            if (identifyTime > 0) {
                zigbee::ZDevice::get_instance()->led->set_blink(std::chrono::milliseconds(500), std::make_optional<size_t>(identifyTime * 2));
            } else {
                zigbee::ZDevice::get_instance()->led->set_off();
            }
        }
        ESP_LOGD(TAG, "identifyTime=%u -> %s blinking", static_cast<unsigned>(identifyTime), (identifyTime > 0 ? "start" : "stop"));
        return ESP_OK;
    }

    // Debug LED:
    if (msg->info.dst_endpoint == LIGHT_ON_OFF_ENDPOINT_ID.endpoint && msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF && msg->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
        bool& curDebugLed = zigbee::ZDevice::get_instance()->curDebugLed;
        curDebugLed = msg->attribute.data.value ? *(bool*) msg->attribute.data.value : curDebugLed;
        if (zigbee::ZDevice::get_instance()->rgbLed) {
            if (curDebugLed) {
                zigbee::ZDevice::get_instance()->rgbLed->enable();
            } else {
                zigbee::ZDevice::get_instance()->rgbLed->disable();
            }
        }
        ESP_LOGD(TAG, "Debug LED changed to: %s", curDebugLed ? "on" : "off");
    }

    // Misc:
    else {
        ESP_LOGI(TAG, "Unhandled attribute changed. status: %d, endpoint: %d, clusterId: %d, attrId: %d, attrTypeId: %d", msg->info.status, msg->info.dst_endpoint, msg->info.cluster, msg->attribute.id, msg->attribute.data.type);
    }
    return ESP_OK;
}

esp_err_t ZDevice::on_ota_upgrade_data_message(uint32_t totalSize, void* payload, uint16_t payloadSize, void** outbuf, uint16_t* outlen) {
    void* dataBuf = nullptr;
    uint16_t dataLen;

    if (!otaStatus.tagReceived) {
        if (!payload || !outlen || !outbuf || payloadSize <= OTA_ELEMENT_HEADER_LEN) {
            ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Invalid element format");
        }

        uint8_t* p = static_cast<uint8_t*>(payload);
        uint32_t length = 0;

        // element header: [uint16_t tag][uint32_t length], little-endian on ESP32
        memcpy(&otaStatus.tag, p, sizeof(otaStatus.tag));
        memcpy(&length, p + sizeof(otaStatus.tag), sizeof(length));

        if ((length + OTA_ELEMENT_HEADER_LEN) != totalSize) {
            ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Invalid element length [%ld/%ld]", length, totalSize);
        }

        otaStatus.tagReceived = true;

        dataBuf = static_cast<void*>(p + OTA_ELEMENT_HEADER_LEN);
        dataLen = payloadSize - OTA_ELEMENT_HEADER_LEN;
    } else {
        dataBuf = payload;
        dataLen = payloadSize;
    }

    switch (otaStatus.tag) {
        case 0: // upgrade image
            *outbuf = dataBuf;
            *outlen = dataLen;
            break;
        default:
            ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Unsupported element tag identifier %d", otaStatus.tag);
            break;
    }

    return ESP_OK;
}

esp_err_t ZDevice::on_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* message) {
    static uint32_t totalSize = 0;
    static uint32_t rxTotal = 0;
    static uint32_t writtenTotal = 0;
    static esp_err_t ret = ESP_OK;

    static esp_ota_handle_t otaHandle{};
    static const esp_partition_t* otaPartition{nullptr};

    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        switch (message->upgrade_status) {
            case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
                ESP_LOGI(TAG, "OTA started.");
                otaPartition = esp_ota_get_next_update_partition(NULL);
                assert(otaPartition);
                ret = esp_ota_begin(otaPartition, 0, &otaHandle);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::OTA);
                break;

            case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
                totalSize = message->ota_header.image_size;
                rxTotal += message->payload_size;

                if (message->payload_size > 0 && message->payload) {
                    uint16_t payloadSize = 0;
                    void* payload = nullptr;
                    ret = zigbee::ZDevice::get_instance()->on_ota_upgrade_data_message(totalSize, message->payload, message->payload_size, &payload, &payloadSize);
                    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to find/extract OTA data from element. Status: %s", esp_err_to_name(ret));

                    if (payloadSize > 0) {
                        ret = esp_ota_write(otaHandle, payload, payloadSize);
                        ESP_RETURN_ON_ERROR(ret, TAG, "esp_ota_write failed: %s", esp_err_to_name(ret));
                        writtenTotal += payloadSize;
                    }
                }
                ESP_LOGI(TAG, "OTA Client receives data: progress [%ld/%ld]", writtenTotal, totalSize - OTA_ELEMENT_HEADER_LEN);
                break;

            case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
                ESP_LOGI(TAG, "OTA apply.");
                break;

            case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK: {
                const uint32_t expectedWritten = (totalSize >= OTA_ELEMENT_HEADER_LEN) ? (totalSize - OTA_ELEMENT_HEADER_LEN) : 0;

                ret = (writtenTotal == expectedWritten) ? ESP_OK : ESP_FAIL;

                zigbee::ZDevice::get_instance()->otaStatus.tagReceived = false;
                ESP_LOGI(TAG, "OTA CHECK: %s (written=%lu, expected=%lu)", (ret == ESP_OK ? "OK" : "MISMATCH"), static_cast<unsigned long>(writtenTotal), static_cast<unsigned long>(expectedWritten));

                // Reset counters for next session:
                rxTotal = 0;
                writtenTotal = 0;
                break;
            }

            case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
                ret = esp_ota_end(otaHandle);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to end OTA with status: %s", esp_err_to_name(ret));

                ret = esp_ota_set_boot_partition(otaPartition);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set OTA boot partition with status: %s", esp_err_to_name(ret));

                ESP_LOGI(TAG, "OTA finished: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes", message->ota_header.file_version, message->ota_header.manufacturer_code, message->ota_header.image_type, message->ota_header.image_size);
                esp_restart();
                break;

            case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
                if (otaHandle) {
                    esp_ota_end(otaHandle); // ignore return; best-effort
                    otaHandle = 0;
                }

                // Reset counters for next session:
                rxTotal = 0;
                writtenTotal = 0;
                zigbee::ZDevice::get_instance()->otaStatus.tagReceived = false;
                ESP_LOGE(TAG, "OTA aborted: %s", esp_err_to_name(ret));
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::CONNECTED);
                break;

            default:
                ESP_LOGW(TAG, "OTA unknown status: %d", message->upgrade_status);
                break;
        }
    }
    return ret;
}

esp_err_t ZDevice::on_ota_upgrade_query_image_resp(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t* message) {
    esp_err_t ret = ESP_OK;
    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Queried OTA image from address: 0x%04hx, endpoint: %d", message->server_addr.u.short_addr, message->server_endpoint);
        ESP_LOGI(TAG, "Image version: 0x%lx, manufacturer code: 0x%x, image size: %ld", message->file_version, message->manufacturer_code, message->image_size);
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Approving OTA image upgrade.");
    } else {
        ESP_LOGI(TAG, "Rejecting OTA image upgrade, status: %s", esp_err_to_name(ret));
    }
    return ret;
}

void ZDevice::bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
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

void ZDevice::setup_debug_led_cluster() {
    // Ensure this is called only once
    assert(!debugLedClusterList);

    debugLedCfg.basic_cfg.power_source = DEFAULT_POWER_SOURCE;
    debugLedCfg.basic_cfg.zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;

    debugLedClusterList = esp_zb_on_off_light_clusters_create(&debugLedCfg);
    curDebugLed = rgbLed->is_enabled();
}

void ZDevice::setup_battery_cluster() {
    // Ensure this is called only once
    assert(!powerAttrList);
    assert(clusterList);

    powerCfg.main_voltage = 37;      // 3.7V battery ln 100 mV steps
    powerCfg.main_voltage_max = 55;  // 5.5V battery ln 100 mV steps
    powerCfg.main_voltage_min = 3.2; // 3.2V battery ln 100 mV steps

    powerAttrList = esp_zb_power_config_cluster_create(&powerCfg);
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &curBatteryPercentage));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &curBatteryMv));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(clusterList, powerAttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void ZDevice::set_device_state(DeviceState newState) {
    if (deviceState == newState) {
        return;
    }
    deviceState = newState;

    // The device can only go to sleep when it is actually connected to a network.
    switch (deviceState) {
        case DeviceState::SETUP:
            set_led_color(actuators::color_t{30, 0, 30}); // Purple
            esp_zb_sleep_enable(false);
            break;

        case DeviceState::OTA:
            set_led_color(actuators::color_t{30, 30, 0}); // Yellow
            esp_zb_sleep_enable(false);
            break;

        case DeviceState::CONNECTING:
            set_led_color(actuators::color_t{0, 0, 30}); // Blue
            esp_zb_sleep_enable(false);
            break;

        case DeviceState::CONNECTED:
            set_led_color(actuators::color_t{0, 0, 30}); // Green
            esp_zb_sleep_enable(true);
            break;

        default:
            set_led_color(actuators::color_t{30, 0, 0}); // Red
            ESP_LOGE(TAG, "Unknown device state: %d", static_cast<uint8_t>(deviceState));
            esp_zb_sleep_enable(false);
            break;
    }
}

void ZDevice::on_connected() {
    zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::CONNECTED);

    esp_zb_ieee_addr_t extended_pan_id;
    esp_zb_get_extended_pan_id(extended_pan_id);
    ESP_LOGI(zigbee::ZDevice::TAG, "Connected (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)", extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel());

    // Report the current debug LED state
    if (rgbLed) {
        ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(LIGHT_ON_OFF_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &ZDevice::get_instance()->curDebugLed, false));
    }
    // Report the current battery percentage
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &ZDevice::get_instance()->curBatteryPercentage, false));
    ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &ZDevice::get_instance()->curBatteryMv, false));
}
} // namespace zigbee

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sigType = static_cast<esp_zb_app_signal_type_t>(*signal_struct->p_app_signal);

    switch (sigType) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::CONNECTING);
            ESP_LOGI(zigbee::ZDevice::TAG, "Zigbee stack initialized");
            esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(zigbee::ZDevice::TAG, "Device state: %s", esp_zb_bdb_is_factory_new() ? "factory new" : "configured");
                if (esp_zb_bdb_is_factory_new()) {
                    zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::SETUP);
                    ESP_LOGI(zigbee::ZDevice::TAG, "Scanning for available Zigbee networks and joining one that's open to new devices....");
                    esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
                } else {
                    zigbee::ZDevice::get_instance()->on_connected();
                }
            } else {
                /* commissioning failed */
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::CONNECTING);
                ESP_LOGW(zigbee::ZDevice::TAG, "Failed to initialize Zigbee stack (status: %s).", esp_err_to_name(err_status));
                ESP_LOGI(zigbee::ZDevice::TAG, "Scanning for available Zigbee networks and joining one that's open to new devices....");
                esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                zigbee::ZDevice::get_instance()->on_connected();
            } else {
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::CONNECTING);
                ESP_LOGI(zigbee::ZDevice::TAG, "Rejoining a known network was not successful (status: %s). Attempting to join again...", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;

        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            ESP_LOGI(zigbee::ZDevice::TAG, "ZigBee device can sleep signal received.");
            esp_zb_sleep_now();
            break;

        case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            ESP_LOGI(zigbee::ZDevice::TAG, "ZigBee device production config ready.");
            esp_zb_set_node_descriptor_manufacturer_code(42);
            break;

        case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
            zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::CONNECTING);
            ESP_LOGI(zigbee::ZDevice::TAG, "ZigBee device unavailable (status: %s). Trying to rejoin...", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(zigbee::ZDevice::bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            break;

        case ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE:
            if (err_status == ESP_OK) {
                zigbee::ZDevice::get_instance()->on_connected();
                ESP_LOGI(zigbee::ZDevice::TAG, "TC rejoin completed.");
            } else {
                ESP_LOGW(zigbee::ZDevice::TAG, "TC rejoin failed: %s", esp_err_to_name(err_status));
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            const esp_zb_zdo_signal_leave_params_t* leaveParams = static_cast<esp_zb_zdo_signal_leave_params_t*>(esp_zb_app_signal_get_params(signal_struct->p_app_signal));
            if (leaveParams->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::SETUP);
                ESP_LOGW(zigbee::ZDevice::TAG, "ZigBee leave signal with device reset request and error status '%s' received.", esp_err_to_name(err_status));
                esp_zb_factory_reset();
            } else {
                zigbee::ZDevice::get_instance()->set_device_state(zigbee::ZDevice::DeviceState::CONNECTING);
                ESP_LOGW(zigbee::ZDevice::TAG, "ZigBee leave signal with error status '%s' received.", esp_err_to_name(err_status));
            }
        } break;

        case ESP_ZB_NLME_STATUS_INDICATION:
            ESP_LOGI(zigbee::ZDevice::TAG, "%s NLME status '0x%x' with error status: %s", esp_zb_zdo_signal_to_string(sigType), *static_cast<uint8_t*>(esp_zb_app_signal_get_params(signal_struct->p_app_signal)), esp_err_to_name(err_status));
            break;

        default:
            ESP_LOGW(zigbee::ZDevice::TAG, "ZDO unhandled signal: %s, error status: %s", esp_zb_zdo_signal_to_string(sigType), esp_err_to_name(err_status));
            break;
    }
}
