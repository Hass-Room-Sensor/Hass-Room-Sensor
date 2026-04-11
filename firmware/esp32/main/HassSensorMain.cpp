#include "app/RetainedState.hpp"
#include "defs/DeviceDefs.hpp"
#include "devices/AbstractDeviceEventListener.hpp"
#include "devices/DeviceEventListenerFactory.hpp"
#include "esp_log.h"
#include "esp_log_level.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "sensors/AbstractBme690.hpp"
#include "sensors/AbstractScd41.hpp"
#include "sensors/Battery.hpp"
#include "sensors/Bme690.hpp"
#include "sensors/Bme690Mock.hpp"
#include "zigbee/ZDevice.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <thread>

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_MOCK
#include "sensors/Scd41Mock.hpp"
#else
#include "sensors/Scd41.hpp"
#endif

namespace {
const char* TAG = "hassSensor";
// Give the Zigbee stack a short moment to flush attribute updates before power is removed again.
constexpr std::chrono::seconds SESSION_SETTLE_TIME{2};
// First boot may need a much longer join window than normal wake-up reports.
constexpr std::chrono::minutes INITIAL_JOIN_TIMEOUT{5};
constexpr std::chrono::seconds REJOIN_TIMEOUT{20};

/**
 * Reads the battery voltage once and maps it into a coarse percentage for reporting.
 */
[[nodiscard]] std::optional<models::BatteryReading> read_battery() {
    sensors::Battery battery{};
    if (!battery.init()) {
        ESP_LOGW(TAG, "Battery ADC initialization failed.");
        return std::nullopt;
    }

    const std::optional<int> millivolts = battery.read_milli_volt();
    if (!millivolts) {
        return std::nullopt;
    }

    constexpr int MV_MIN = 3000;
    constexpr int MV_MAX = 4200;
    int percentage = (*millivolts - MV_MIN) * 100 / (MV_MAX - MV_MIN);
    percentage = std::clamp(percentage, 0, 100);

    return std::make_optional<models::BatteryReading>(models::BatteryReading{
            .millivolts = static_cast<uint16_t>(*millivolts),
            .percentage = static_cast<uint8_t>(percentage),
    });
}

/**
 * Collects one forced-mode BME690 sample or returns no data if the sensor is not assembled.
 */
[[nodiscard]] std::optional<sensors::bme690_measurement_t> read_bme690() {
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_BME690_MOCK
    std::unique_ptr<sensors::AbstractBme690> sensor = std::make_unique<sensors::Bme690Mock>();
#else
    std::unique_ptr<sensors::AbstractBme690> sensor = std::make_unique<sensors::Bme690>(HASS_SENSOR_BME690_SDA_GPIO, HASS_SENSOR_BME690_SCL_GPIO);
#endif

    if (!sensor->init()) {
        ESP_LOGW(TAG, "BME690 not available. Continuing without pressure measurements.");
        return std::nullopt;
    }

    return sensor->read_measurement();
}

/**
 * Collects one single-shot SCD41 measurement and forwards ambient pressure compensation when available.
 */
[[nodiscard]] std::optional<sensors::measurement_t> read_scd41(std::optional<uint16_t> pressureHpa) {
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_MOCK
    std::unique_ptr<sensors::AbstractScd41> sensor = std::make_unique<sensors::Scd41Mock>();
#else
    std::unique_ptr<sensors::AbstractScd41> sensor = std::make_unique<sensors::Scd41>(HASS_SENSOR_SCD4X_SDA_GPIO, HASS_SENSOR_SCD4X_SCL_GPIO);
#endif

    if (!sensor->init()) {
        ESP_LOGW(TAG, "SCD41 not available. Continuing without CO2 measurements.");
        return std::nullopt;
    }

    return sensor->read_single_shot(pressureHpa);
}

/**
 * Reads all available environmental sensors for the current wake cycle.
 *
 * The BME690 is sampled first so its pressure value can be injected into the SCD41 single-shot
 * command path for a more accurate CO2 reading.
 */
[[nodiscard]] models::EnvironmentalReadings collect_environmental_readings() {
    models::EnvironmentalReadings readings{};

    const std::optional<sensors::bme690_measurement_t> bme690 = read_bme690();
    if (bme690) {
        readings.pressure_hpa = bme690->pressure_hpa;
        readings.temperature_celsius = bme690->temp_celsius;
        readings.humidity_percent = bme690->humidity_percent;
    }

    const std::optional<sensors::measurement_t> scd41 = read_scd41(bme690 ? std::make_optional(static_cast<uint16_t>(std::lround(bme690->pressure_hpa))) : std::nullopt);
    if (scd41) {
        readings.co2_ppm = scd41->co2;
        readings.temperature_celsius = scd41->temp;
        readings.humidity_percent = scd41->hum;
    }

    return readings;
}

void schedule_next_wake_and_sleep() {
    ESP_LOGI(TAG, "Entering deep sleep for %lld seconds.", std::chrono::duration_cast<std::chrono::seconds>(app::RetainedState::WAKE_INTERVAL).count());
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(std::chrono::duration_cast<std::chrono::microseconds>(app::RetainedState::WAKE_INTERVAL).count()));
    esp_deep_sleep_start();
}

void mark_running_partition_valid() {
    const esp_partition_t* runningPartition = esp_ota_get_running_partition();
    esp_ota_img_states_t imageState{};
    if (esp_ota_get_state_partition(runningPartition, &imageState) == ESP_OK && imageState == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
    }
}

void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}
} // namespace

void mainLoop() {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // ESP-IDF's VFS layer emits very noisy verbose traces under the `vfs_calls` tag. Keep the global
    // log level unchanged for firmware debugging, but silence that tag completely.
    esp_log_level_set("vfs_calls", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Starting HASS environment sensor version %d.%d.%d", CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MAJOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MINOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_PATCH);

    init_nvs();
    mark_running_partition_valid();

    std::shared_ptr<devices::AbstractDeviceEventListener> deviceListener = devices::create_device_event_listener();
    deviceListener->init();

    app::RetainedState retainedState{};
    const models::EnvironmentalReadings environmentalReadings = collect_environmental_readings();
    const models::QuantizedEnvironmentalReadings quantizedEnvironmental = models::quantize(environmentalReadings);
    const std::optional<models::BatteryReading> batteryReading = read_battery();
    const std::optional<models::QuantizedBatteryReading> quantizedBattery = batteryReading ? std::make_optional(models::quantize(*batteryReading)) : std::nullopt;

    // Environmental values are reported every five minutes only when any quantized attribute changed.
    const bool shouldReportEnvironment = retainedState.should_report_environment(quantizedEnvironmental);
    // Battery is reported on startup, on percentage changes, and once per day as a keepalive.
    const bool shouldReportBattery = quantizedBattery && retainedState.should_report_battery(*quantizedBattery);

    if (!shouldReportEnvironment && !shouldReportBattery && !retainedState.is_initial_startup()) {
        ESP_LOGI(TAG, "No attribute changed. Skipping Zigbee wake-up for this cycle.");
        schedule_next_wake_and_sleep();
    }

    zigbee::PublishRequest publishRequest{};
    if (shouldReportEnvironment) {
        publishRequest.environmental = quantizedEnvironmental;
    }
    if (shouldReportBattery && quantizedBattery) {
        publishRequest.battery = *quantizedBattery;
    }

    zigbee::ZDevice::get_instance()->set_device_listener(deviceListener);
    zigbee::ZDevice::get_instance()->init();

    const auto joinTimeout = retainedState.is_initial_startup() ? INITIAL_JOIN_TIMEOUT : REJOIN_TIMEOUT;
    if (!zigbee::ZDevice::get_instance()->wait_for_connection(joinTimeout)) {
        ESP_LOGW(TAG, "Zigbee network is not available in this wake window.");
        schedule_next_wake_and_sleep();
    }

    zigbee::ZDevice::get_instance()->publish(publishRequest);
    std::this_thread::sleep_for(SESSION_SETTLE_TIME);

    if (shouldReportEnvironment) {
        retainedState.mark_environment_reported(quantizedEnvironmental);
    }
    if (shouldReportBattery && quantizedBattery) {
        retainedState.mark_battery_reported(*quantizedBattery);
    }
    retainedState.mark_startup_report_completed();

    schedule_next_wake_and_sleep();
}

extern "C" void app_main(void) {
    mainLoop();
}
