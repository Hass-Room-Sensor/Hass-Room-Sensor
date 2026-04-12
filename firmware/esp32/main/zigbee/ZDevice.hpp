#pragma once

#include "devices/AbstractDeviceEventListener.hpp"
#include "models/SensorModels.hpp"
#include "sensors/GpioInput.hpp"
#include "zigbee/ZigbeeDeviceState.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

extern "C" {
#include "esp_zigbee_ota.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "soc/gpio_num.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_carbon_dioxide_measurement.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_pressure_meas.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"
}

namespace zigbee {
/**
 * Wake-local publish request for one short Zigbee session.
 */
struct PublishRequest {
    /** Environmental attributes to publish during the current wake session. */
    std::optional<models::QuantizedEnvironmentalReadings> environmental{};
    /** Battery attributes to publish during the current wake session. */
    std::optional<models::QuantizedBatteryReading> battery{};
};

/**
 * Zigbee end-device wrapper.
 *
 * The firmware boots, optionally rejoins the network, publishes the requested attributes and then returns to ESP deep sleep.
 */
class ZDevice {
  public:
    /** Log tag used by the Zigbee device wrapper. */
    static const char* TAG;

    ZDevice() = default;
    ZDevice(ZDevice&&) = default;
    ZDevice(const ZDevice&) = delete;
    ZDevice& operator=(ZDevice&&) = default;
    ZDevice& operator=(const ZDevice&) = delete;
    ~ZDevice() = default;

    /** Returns the singleton Zigbee device wrapper used by the firmware. */
    [[nodiscard]] static const std::unique_ptr<ZDevice>& get_instance();

    /** Starts the Zigbee stack if needed and prepares the wake-local session state. */
    void init();
    /** Returns true when the Zigbee session is currently connected to a network. */
    [[nodiscard]] bool has_connection() const;
    /** Waits until the device rejoins or forms a connection within the given timeout. */
    [[nodiscard]] bool wait_for_connection(std::chrono::milliseconds timeout) const;
    /** Publishes the requested attributes into the already joined Zigbee network. */
    void publish(const PublishRequest& request);
    /** Registers the hardware-specific listener used for LEDs and identify effects. */
    void set_device_listener(std::shared_ptr<devices::AbstractDeviceEventListener> listener);

    /** Triggers a Zigbee factory reset. */
    void reset() const;
    /** Updates the externally visible device state. */
    void set_device_state(ZigbeeDeviceState newState);
    /** Marks the current session as connected and wakes any waiter. */
    void on_connected();

    /** Starts Zigbee BDB commissioning with the provided mode mask. */
    static void bdb_start_top_level_commissioning_cb(uint8_t modeMask);

  private:
    /** Main sensor endpoint exposed to Zigbee. */
    static constexpr esp_zb_endpoint_config_t DEFAULT_ENDPOINT_ID{10, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, 4};
    /** Optional debug-light endpoint used on boards with a controllable LED. */
    static constexpr esp_zb_endpoint_config_t LIGHT_ON_OFF_ENDPOINT_ID{11, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, 4};

    /**
     * The ZigBee spec defines the power source in "3.2.2.2.8 PowerSource Attribute".
     * https://zigbeealliance.org/wp-content/uploads/2019/12/07-5123-06-zigbee-cluster-library-specification.pdf
     *
     * 0x0 Unknown
     * 0x1 Mains (single phase)
     * 0x2 Mains (3 phases)
     * 0x3 Battery
     * 0x4 DC source
     * 0x5 Emergency mains constantly powered
     * 0x6 Emergency mains and transfer switch
     * 0x7 Secondary power backup
     */
    static constexpr uint8_t DEFAULT_POWER_SOURCE = 0x04;
    /** Event-group bit that signals a successful Zigbee connection for the current wake session. */
    static constexpr EventBits_t CONNECTED_BIT = BIT0;
    /**
     * Supported carbon dioxide publish range in parts per million.
     *
     * Zigbee stores the CO2 measurement as a volumetric fraction, but Espressif validates the
     * value against the configured min/max bounds when `esp_zb_zcl_set_attribute_val()` is called.
     * Keeping the ppm limits explicit here makes the conversion logic easier to audit.
     */
    static constexpr uint16_t CO2_MIN_PPM = 400;
    /** Upper bound for the Zigbee CO2 cluster in parts per million. */
    static constexpr uint16_t CO2_MAX_PPM = 5000;

    // Basic cluster information.
    /** Backing storage for the Zigbee Basic cluster. */
    esp_zb_basic_cluster_cfg_t basicClusterConfig{ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE, DEFAULT_POWER_SOURCE};
    /** Basic cluster attribute list once it has been created. */
    esp_zb_attribute_list_t* basicAttrList{nullptr};
    /** Cached Zigbee string buffer for the model identifier attribute. */
    std::vector<char> modelId{};
    /** Cached Zigbee string buffer for the manufacturer attribute. */
    std::vector<char> manufacturer{};
    /** Cached Zigbee string buffer for the firmware version attribute. */
    std::vector<char> version{};

    // Cluster list.
    /** Cluster list attached to the main sensor endpoint. */
    esp_zb_cluster_list_t* clusterList{nullptr};
    /** Composite cluster configuration used when creating the main sensor endpoint. */
    esp_zb_temperature_sensor_cfg_t clusterListCfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();

    // OTA.
    /** OTA cluster configuration exposed to Zigbee. */
    esp_zb_ota_cluster_cfg_t otaCfg{};
    /** OTA client state required by Espressif's OTA cluster implementation. */
    esp_zb_zcl_ota_upgrade_client_variable_t otaClientCfg{};
    /** OTA cluster attribute list once it has been created. */
    esp_zb_attribute_list_t* otaAttrList{nullptr};
    /** Address of the OTA server that ZHA assigned to this device. */
    uint16_t otaUpgradeServerAddr = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_ADDR_DEF_VALUE;
    /** Endpoint of the OTA server that ZHA assigned to this device. */
    uint8_t otaUpgradeServerEp = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_ENDPOINT_DEF_VALUE;
    // Hardcoded OTA element header size including the tag identifier and length field.
    // Source: Espressif's OTA client example.
    // https://github.com/espressif/esp-zigbee-sdk/blob/5e065f3285f89a32f8dec84e42227049af6d4324/examples/esp_zigbee_ota/ota_client/main/esp_ota_client.h#L55C9-L55C31
    static constexpr uint16_t OTA_ELEMENT_HEADER_LEN = 6;

    // Measurement clusters.
    /** Temperature measurement cluster configuration. */
    esp_zb_temperature_meas_cluster_cfg_t tempCfg{};
    /** Temperature measurement cluster attribute list. */
    esp_zb_attribute_list_t* tempAttrList{nullptr};
    /** Current temperature attribute in 0.01 degrees Celsius. */
    int16_t curTemp{ESP_ZB_ZCL_TEMP_MEASUREMENT_MEASURED_VALUE_UNKNOWN};

    /** Humidity measurement cluster configuration. */
    esp_zb_humidity_meas_cluster_cfg_t humCfg{};
    /** Humidity measurement cluster attribute list. */
    esp_zb_attribute_list_t* humAttrList{nullptr};
    /** Current humidity attribute in 0.01 percent RH. */
    uint16_t curHum{ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_UNKNOWN};

    /** Pressure measurement cluster configuration. */
    esp_zb_pressure_meas_cluster_cfg_t pressureCfg{};
    /** Pressure measurement cluster attribute list. */
    esp_zb_attribute_list_t* pressureAttrList{nullptr};
    /** Current pressure attribute in 0.1 kPa. */
    int16_t curPressure{ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_UNKNOWN};

    /** Carbon dioxide measurement cluster configuration. */
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2Cfg{};
    /** Carbon dioxide measurement cluster attribute list. */
    esp_zb_attribute_list_t* co2AttrList{nullptr};
    /** Current carbon dioxide attribute in volumetric fraction units expected by Zigbee. */
    float_t curCo2{std::numeric_limits<float_t>::quiet_NaN()};

    // Debug LED.
    /** On/off light cluster configuration used for the optional debug LED endpoint. */
    esp_zb_on_off_light_cfg_t debugLedCfg{};
    /** Cluster list for the optional debug LED endpoint. */
    esp_zb_cluster_list_t* debugLedClusterList{nullptr};
    /** Cached state of the optional debug LED. */
    bool curDebugLed{true};

    // Battery.
    /** Power configuration cluster backing storage. */
    esp_zb_power_config_cluster_cfg_t powerCfg{};
    /** Power configuration cluster attribute list. */
    esp_zb_attribute_list_t* powerAttrList{nullptr};
    /** Current battery percentage in Zigbee half-percent steps. Unknown is 0xFF. */
    uint8_t curBatteryPercentage{0xFF};
    /** Current battery voltage in 100 mV units. */
    uint8_t curBatteryMv{37};
    /** Battery form factor. 0x1 means built-in battery. */
    uint8_t curBatterySize{0x1};
    /** Rated battery voltage in 100 mV units. */
    uint8_t curBatteryRatedVoltage{37};
    /** Rated battery capacity in 10 mAh units. */
    uint16_t curBatteryMAhRating{50};

    // The underlying hardware device, e.g. a Seed Studio XIAO ESP32-C6 or ESP32 dev kit,
    // listening for Zigbee state changes and identify effects.
    std::shared_ptr<devices::AbstractDeviceEventListener> deviceListener{nullptr};
    /** Reset GPIO used for factory-resetting the Zigbee stack. */
    sensors::GpioInput resetGpio{GPIO_NUM_1};
    /** Input that selects whether the device should advertise itself as battery powered. */
    sensors::GpioInput powerSourceBattery{GPIO_NUM_3};
    /** Current high-level Zigbee state used by the hardware listener. */
    ZigbeeDeviceState deviceState{ZigbeeDeviceState::SETUP};
    /** Event group used to wake the main application once Zigbee reconnects. */
    EventGroupHandle_t eventGroup_{nullptr};
    /** True once the Zigbee stack task has been created. */
    bool initialized_{false};

    /** Small state bundle used while parsing OTA element frames. */
    struct OtaStatus {
        /** OTA element tag identifier. */
        uint16_t tag{0};
        /** True once the first OTA frame provided the tag and total-size header. */
        bool tagReceived{false};
    } __attribute__((aligned(4)));
    /** OTA receive state for the currently running OTA session. */
    OtaStatus otaStatus{};

    /** Entry point of the long-lived Zigbee task. */
    static void zb_main_task(void* arg);
    /** Top-level Zigbee action callback registered with the stack. */
    static esp_err_t on_zb_action(esp_zb_core_action_callback_id_t callbackId, const void* message);
    /** Handles attribute writes from Zigbee, such as identify or debug-light commands. */
    static esp_err_t on_attr_changed(const esp_zb_zcl_set_attr_value_message_t* message);
    /** Handles OTA state-machine updates from the Zigbee stack. */
    static esp_err_t on_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* message);
    /** Strips the OTA element header and validates the payload framing used by Espressif's OTA path. */
    esp_err_t on_ota_upgrade_data_message(uint32_t totalSize, void* payload, uint16_t payloadSize, void** outBuffer, uint16_t* outLength);
    /** Handles the OTA query-image response event from the server. */
    static esp_err_t on_ota_upgrade_query_image_resp(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t* message);

    /** Creates the main sensor cluster list with the mandatory temperature sensor scaffold. */
    [[nodiscard]] esp_zb_cluster_list_t* setup_temp_sensor();
    /** Adds the Basic cluster and its string attributes to the main endpoint. */
    void setup_basic_cluster(const std::string& modelIdStr, const std::string& manufacturerStr, const std::string& versionStr);
    /** Adds the OTA client cluster to the main endpoint. */
    void setup_ota_cluster();
    /** Adds the humidity cluster to the main endpoint. */
    void setup_hum_cluster();
    /** Adds the pressure cluster to the main endpoint. */
    void setup_pressure_cluster();
    /** Adds the carbon dioxide cluster to the main endpoint. */
    void setup_co2_cluster();
    /** Creates the optional debug LED endpoint on boards that expose one. */
    void setup_debug_led_cluster();
    /** Adds the battery/power configuration cluster to the main endpoint. */
    void setup_battery_cluster();

    /** Updates the in-memory Zigbee temperature attribute. */
    void update_temp(int16_t temperatureCentiCelsius);
    /** Updates the in-memory Zigbee humidity attribute. */
    void update_hum(uint16_t humidityCentiPercent);
    /** Updates the in-memory Zigbee pressure attribute. */
    void update_pressure(int16_t pressureDeciKpa);
    /** Updates the in-memory Zigbee carbon dioxide attribute. */
    void update_co2(uint16_t co2Ppm);
    /** Updates the in-memory Zigbee battery attributes. */
    void update_battery(const models::QuantizedBatteryReading& battery);

    /** Writes the model identifier into the Basic cluster. */
    void set_model_id(const std::string& modelIdStr);
    /** Writes the manufacturer name into the Basic cluster. */
    void set_manufacturer(const std::string& manufacturerStr);
    /** Writes the firmware version string into the Basic cluster. */
    void set_version_details(const std::string& versionStr);
    /** Encodes a Zigbee character string and stores it into the Basic cluster. */
    void set_basic_attr(const std::string& basicAttrStr, std::vector<char>& basicAttrStrCache, esp_zb_zcl_basic_attr_t attrId);
};
} // namespace zigbee
