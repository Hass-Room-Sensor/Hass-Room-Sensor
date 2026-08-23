#include "zigbee/ZDevice.hpp"

#include "app/RetainedState.hpp"
#include "defs/DeviceDefs.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_type.h"
#include "freertos/task.h"
#include "nvs_flash.h"
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

namespace {
ZDevice& self() {
    return *ZDevice::get_instance();
}
} // namespace

void ZDevice::set_device_listener(std::shared_ptr<devices::AbstractDeviceEventListener> listener) {
    deviceListener = std::move(listener);
}

const std::unique_ptr<ZDevice>& ZDevice::get_instance() {
    static const std::unique_ptr<ZDevice> instance = std::make_unique<ZDevice>();
    return instance;
}

void ZDevice::init() {
    if (eventGroup == nullptr) {
        eventGroup = xEventGroupCreate();
        assert(eventGroup != nullptr);
    }

#if defined(CONFIG_PM_ENABLE) && defined(CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP)
    init_light_sleep_blocker();
#endif

    if (initialized_) {
        return;
    }

    if (!deviceListener) {
        ESP_LOGW(TAG, "No device listener registered; hardware callbacks are disabled.");
    }

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
    const gpio_int_type_t resetWakeLevel = HASS_SENSOR_FACTORY_RESET_LOW_ACTIVE ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL;
    ESP_ERROR_CHECK(gpio_wakeup_enable(HASS_SENSOR_FACTORY_RESET_GPIO, resetWakeLevel));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
#endif

    esp_zb_platform_config_t config = {};
    config.radio_config.radio_mode = ZB_RADIO_MODE_NATIVE;
    config.host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE;
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(ZDevice::zb_main_task, "zigbee_main", 4096, this, 5, nullptr);
    xTaskCreate(ZDevice::factory_reset_button_task, "factory_reset", 2048, this, 5, &factoryResetTask);
    assert(factoryResetTask != nullptr);

    const gpio_int_type_t resetInterrupt = HASS_SENSOR_FACTORY_RESET_LOW_ACTIVE ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
    ESP_ERROR_CHECK(gpio_set_intr_type(HASS_SENSOR_FACTORY_RESET_GPIO, resetInterrupt));
    const esp_err_t isrServiceResult = gpio_install_isr_service(0);
    if (isrServiceResult != ESP_OK && isrServiceResult != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isrServiceResult);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(HASS_SENSOR_FACTORY_RESET_GPIO, ZDevice::factory_reset_button_isr, this));
    initialized_ = true;
}

bool ZDevice::has_connection() const {
    return eventGroup != nullptr && (xEventGroupGetBits(eventGroup) & CONNECTED_BIT) != 0;
}

bool ZDevice::wait_for_connection(std::chrono::milliseconds timeout) const {
    const EventBits_t bits = xEventGroupWaitBits(eventGroup, CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout.count()));
    return (bits & CONNECTED_BIT) != 0;
}

void ZDevice::set_connected_light_sleep_allowed(bool allowed) {
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
    if (connectedLightSleepAllowed == allowed) {
        return;
    }

    connectedLightSleepAllowed = allowed;
#if defined(CONFIG_PM_ENABLE)
    sync_light_sleep_permission();
#endif
#else
    static_cast<void>(allowed);
#endif
}

#if defined(CONFIG_PM_ENABLE) && defined(CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP)
void ZDevice::init_light_sleep_blocker() {
    if (noLightSleepLock != nullptr) {
        return;
    }

    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "zigbee_join", &noLightSleepLock));
    sync_light_sleep_permission();
}

void ZDevice::sync_light_sleep_permission() {
    if (noLightSleepLock == nullptr) {
        return;
    }

    const bool shouldBlockLightSleep = deviceState != ZigbeeDeviceState::CONNECTED || !connectedLightSleepAllowed;
    if (shouldBlockLightSleep && !lightSleepBlocked) {
        ESP_ERROR_CHECK(esp_pm_lock_acquire(noLightSleepLock));
        lightSleepBlocked = true;
        if (deviceState == ZigbeeDeviceState::CONNECTED) {
            ESP_LOGI(TAG, "Blocking ESP light sleep while the initial Zigbee commissioning window is still active.");
        } else {
            ESP_LOGI(TAG, "Blocking ESP light sleep until the Zigbee network is joined.");
        }
    } else if (!shouldBlockLightSleep && lightSleepBlocked) {
        ESP_ERROR_CHECK(esp_pm_lock_release(noLightSleepLock));
        lightSleepBlocked = false;
        ESP_LOGI(TAG, "Allowing ESP light sleep while the Zigbee session stays joined.");
    }
}
#endif

bool ZDevice::is_light_sleep_allowed_now() const {
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
    return has_connection() && connectedLightSleepAllowed;
#else
    return false;
#endif
}

void ZDevice::handle_can_sleep_signal() {
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
    if (!is_light_sleep_allowed_now()) {
        ESP_LOGD(TAG, "Ignoring CAN_SLEEP while light sleep is still blocked by the application.");
        return;
    }

    if (deviceListener) {
        ESP_LOGI(TAG, "Sleeping....");
        deviceListener->set_sleep_indicator(true);
    }
    esp_zb_sleep_now();
    if (deviceListener) {
        ESP_LOGI(TAG, "Woke up");
        deviceListener->set_sleep_indicator(false);
    }
#endif
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
    ESP_LOGW(TAG, "Performing full factory reset...");
    app::RetainedState::reset();
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_LOGI(TAG, "Erased application NVS. Erasing Zigbee storage and rebooting...");
    esp_zb_factory_reset();
}

bool ZDevice::is_factory_reset_button_pressed() const {
    const bool inputHigh = factoryResetButton.is_powered();
    return HASS_SENSOR_FACTORY_RESET_LOW_ACTIVE ? !inputHigh : inputHigh;
}

void ZDevice::factory_reset_button_task(void* arg) {
    auto* device = static_cast<ZDevice*>(arg);
    assert(device != nullptr);

    constexpr std::chrono::milliseconds DEBOUNCE_TIME{100};

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME.count()));
        if (device->is_factory_reset_button_pressed()) {
            device->reset();
        }
    }
}

void ZDevice::factory_reset_button_isr(void* arg) {
    auto* device = static_cast<ZDevice*>(arg);
    assert(device != nullptr);

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(device->factoryResetTask, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
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
    const esp_err_t err = esp_zb_zcl_set_attribute_val(DEFAULT_ENDPOINT_ID.endpoint, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, static_cast<void*>(&curCo2), false);
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

    ZDevice& device = self();

    const bool batteryPowered = !device.powerSourceBattery.is_powered();
    if (batteryPowered) {
        device.basicClusterConfig.power_source = 0x03;
    } else {
        device.basicClusterConfig.power_source = DEFAULT_POWER_SOURCE;
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
    networkConfig.nwk_cfg.zed_cfg.keep_alive = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(5)).count(); // Wake up from sleep every 5 seconds to send a keep alive via ZigBee
    esp_zb_init(&networkConfig);
    ESP_LOGD(TAG, "esp_zb_init.");

    esp_zb_cluster_list_t* clusterList = device.setup_temp_sensor();
    device.setup_ota_cluster();
    device.setup_hum_cluster();
    device.setup_pressure_cluster();
    device.setup_co2_cluster();
    if (device.deviceListener && device.deviceListener->has_debug_led()) {
        device.setup_debug_led_cluster();
    }
    device.setup_battery_cluster();
    ESP_LOGD(TAG, "setup_battery_cluster.");

    std::array<char, 32> version{};
    snprintf(version.data(), version.size(), "%d.%d.%d", CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MAJOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MINOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_PATCH);
    device.setup_basic_cluster("HASS Env Sensor", "DOOP", std::string{version.data()});

    esp_zb_ep_list_t* endpointList = esp_zb_ep_list_create();
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpointList, clusterList, DEFAULT_ENDPOINT_ID));
    if (device.debugLedClusterList) {
        ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpointList, device.debugLedClusterList, LIGHT_ON_OFF_ENDPOINT_ID));
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
        zb_set_ed_node_descriptor(false, false, true);
        ESP_LOGI(TAG, "Applied battery-powered end-device node descriptor workaround for ZHA.");
    }

    ESP_LOGD(TAG, "esp_zb_stack_main_loop.");
    esp_zb_stack_main_loop();
}

esp_err_t ZDevice::on_zb_action(esp_zb_core_action_callback_id_t callbackId, const void* message) {
    switch (callbackId) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            return ZDevice::on_attr_changed(static_cast<const esp_zb_zcl_set_attr_value_message_t*>(message));
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
            return ZDevice::on_default_response(static_cast<const esp_zb_zcl_cmd_default_resp_message_t*>(message));
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

esp_err_t ZDevice::on_default_response(const esp_zb_zcl_cmd_default_resp_message_t* message) {
    if (!message) {
        return ESP_ERR_INVALID_ARG;
    }

    if (message->status_code == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGD(TAG, "Default Zigbee response acknowledged command 0x%x on cluster 0x%04x from endpoint %u.", message->resp_to_cmd, message->info.cluster, message->info.src_endpoint);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Zigbee default response for command 0x%x on cluster 0x%04x returned status 0x%02x (src ep %u -> dst ep %u).", message->resp_to_cmd, message->info.cluster, message->status_code, message->info.src_endpoint, message->info.dst_endpoint);
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

void ZDevice::schedule_commissioning(uint8_t modeMask) const {
    esp_zb_scheduler_alarm(ZDevice::bdb_start_top_level_commissioning_cb, modeMask, 1000);
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

    powerAttrList = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    assert(powerAttrList);

    constexpr uint8_t REPORTABLE_READ_ONLY = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
    constexpr uint8_t READ_ONLY = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY;

    // ZHA's "Reconfigure" action configures reporting for battery voltage and percentage. Espressif's
    // generic power-config helper exposes the values but does not make battery voltage reportable for
    // this device profile, which causes ZCL status 0x8c (UNREPORTABLE_ATTRIBUTE). Build the cluster
    // explicitly so both runtime battery attributes advertise reporting support.
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, ESP_ZB_ZCL_ATTR_TYPE_U8, REPORTABLE_READ_ONLY, &curBatteryMv));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, ESP_ZB_ZCL_ATTR_TYPE_U8, REPORTABLE_READ_ONLY, &curBatteryPercentage));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, ESP_ZB_ZCL_ATTR_TYPE_U8, READ_ONLY, &curBatterySize));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID, ESP_ZB_ZCL_ATTR_TYPE_U8, READ_ONLY, &curBatteryQuantity));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_A_HR_RATING_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, READ_ONLY, &curBatteryMAhRating));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(powerAttrList, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID, ESP_ZB_ZCL_ATTR_TYPE_U8, READ_ONLY, &curBatteryRatedVoltage));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(clusterList, powerAttrList, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void ZDevice::set_device_state(ZigbeeDeviceState newState) {
    if (deviceState == newState) {
        return;
    }

    deviceState = newState;
#if defined(CONFIG_PM_ENABLE) && defined(CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP)
    sync_light_sleep_permission();
#endif
    if (eventGroup) {
        if (deviceState == ZigbeeDeviceState::CONNECTED) {
            xEventGroupSetBits(eventGroup, CONNECTED_BIT);
        } else {
            xEventGroupClearBits(eventGroup, CONNECTED_BIT);
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

void ZDevice::on_app_signal(esp_zb_app_signal_t* signal) {
    self().handle_app_signal(signal);
}

void ZDevice::handle_app_signal(esp_zb_app_signal_t* signalStruct) {
    const esp_err_t errorStatus = signalStruct->esp_err_status;
    const auto signalType = static_cast<esp_zb_app_signal_type_t>(*signalStruct->p_app_signal);

    switch (signalType) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            set_device_state(ZigbeeDeviceState::CONNECTING);
            ESP_LOGI(TAG, "Zigbee stack initialized after startup skipped");
            schedule_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (errorStatus == ESP_OK) {
                ESP_LOGI(TAG, "Device state: %s", esp_zb_bdb_is_factory_new() ? "factory new" : "configured");
                if (esp_zb_bdb_is_factory_new()) {
                    set_device_state(ZigbeeDeviceState::SETUP);
                    ESP_LOGI(TAG, "Scanning for available Zigbee networks and joining one that's open to new devices...");
                    schedule_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    on_connected();
                }
            } else {
                set_device_state(ZigbeeDeviceState::CONNECTING);
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s).", esp_err_to_name(errorStatus));
                ESP_LOGI(TAG, "Scanning for available Zigbee networks and joining one that's open to new devices....");
                schedule_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (errorStatus == ESP_OK) {
                on_connected();
            } else {
                if (esp_zb_bdb_is_factory_new()) {
                    set_device_state(ZigbeeDeviceState::SETUP);
                    ESP_LOGI(TAG, "Scanning for available networks to join was not successful (status: %s). Attempting to join again...", esp_err_to_name(errorStatus));
                } else {
                    set_device_state(ZigbeeDeviceState::CONNECTING);
                    ESP_LOGI(TAG, "Rejoining a known network was not successful (status: %s). Attempting to join again...", esp_err_to_name(errorStatus));
                }
                schedule_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            break;

        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            ESP_LOGD(TAG, "Zigbee stack entered CAN_SLEEP during the current wake session.");
            handle_can_sleep_signal();
            break;

        case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
            set_device_state(ZigbeeDeviceState::CONNECTING);
            ESP_LOGI(TAG, "ZigBee device unavailable (status: %s). Trying to rejoin...", esp_err_to_name(errorStatus));
            schedule_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            break;

        case ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE:
            if (errorStatus == ESP_OK) {
                on_connected();
                ESP_LOGI(TAG, "TC rejoin completed.");
            } else {
                ESP_LOGW(TAG, "TC rejoin failed: %s", esp_err_to_name(errorStatus));
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            ESP_LOGI(TAG, "Production configuration is %s.", errorStatus == ESP_OK ? "ready" : "not present");
            esp_zb_set_node_descriptor_manufacturer_code(CONFIG_HASS_ENVIRONMENT_SENSOR_OTA_MANUFACTURER);
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            const auto* leaveParams = static_cast<esp_zb_zdo_signal_leave_params_t*>(esp_zb_app_signal_get_params(signalStruct->p_app_signal));
            if (leaveParams && leaveParams->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
                set_device_state(ZigbeeDeviceState::SETUP);
                ESP_LOGW(TAG, "ZigBee leave signal with device reset request and error status '%s' received.", esp_err_to_name(errorStatus));
                esp_zb_factory_reset();
            } else {
                set_device_state(ZigbeeDeviceState::CONNECTING);
                ESP_LOGW(TAG, "ZigBee leave signal with error status '%s' received.", esp_err_to_name(errorStatus));
            }
        } break;

        case ESP_ZB_NLME_STATUS_INDICATION:
            ESP_LOGI(TAG, "%s NLME status '0x%x' with error status: %s", esp_zb_zdo_signal_to_string(signalType), *static_cast<uint8_t*>(esp_zb_app_signal_get_params(signalStruct->p_app_signal)), esp_err_to_name(errorStatus));
            break;

        default:
            ESP_LOGW(TAG, "Unhandled ZDO signal %s (status: %s).", esp_zb_zdo_signal_to_string(signalType), esp_err_to_name(errorStatus));
            break;
    }
}
} // namespace zigbee

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signalStruct) {
    zigbee::ZDevice::on_app_signal(signalStruct);
}
