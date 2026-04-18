#include "app/RetainedState.hpp"
#include "defs/DeviceDefs.hpp"
#include "devices/AbstractDeviceEventListener.hpp"
#include "devices/DeviceEventListenerFactory.hpp"
#include "esp_log.h"
#include "esp_log_level.h"
#include "esp_ota_ops.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
constexpr bool USE_LIGHT_SLEEP =
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
        true;
#else
        false;
#endif
// Give the Zigbee stack a short moment to flush attribute updates before the firmware either deep
// sleeps or resumes its long-lived light-sleep idle period.
constexpr std::chrono::seconds SESSION_SETTLE_TIME{2};
// Keep the device fully awake after the first successful join so Home Assistant can complete the
// initial interview before the sleepy end device starts using automatic ESP light sleep again.
constexpr std::chrono::minutes INITIAL_INTERVIEW_AWAKE_TIME{1};
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

/**
 * Enables automatic light sleep for the long-lived Zigbee sleepy-end-device mode.
 */
void init_power_management() {
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
#ifdef CONFIG_PM_ENABLE
    esp_pm_config_t pmConfig{};
    pmConfig.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    pmConfig.min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    pmConfig.light_sleep_enable = true;
#endif
    ESP_ERROR_CHECK(esp_pm_configure(&pmConfig));
    ESP_LOGI(TAG, "Configured automatic light sleep for the Zigbee sleepy end device.");
#else
    ESP_LOGW(TAG, "Light sleep mode was selected, but CONFIG_PM_ENABLE is disabled in sdkconfig.");
#endif
#endif
}

/**
 * Returns the joined Zigbee session to deep sleep until the next measurement interval.
 */
[[noreturn]] void schedule_next_wake_and_sleep(const std::shared_ptr<devices::AbstractDeviceEventListener>& deviceListener) {
    if (deviceListener) {
        deviceListener->prepare_for_deep_sleep();
    }
    ESP_LOGI(TAG, "Entering deep sleep for %lld seconds.", std::chrono::duration_cast<std::chrono::seconds>(app::RetainedState::WAKE_INTERVAL).count());
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(std::chrono::duration_cast<std::chrono::microseconds>(app::RetainedState::WAKE_INTERVAL).count()));
    esp_deep_sleep_start();
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/**
 * Suspends the current FreeRTOS task for the requested duration.
 *
 * Uses `vTaskDelay()` instead of `std::this_thread::sleep_for()` so the scheduler can continue
 * running the Zigbee stack and power-management code that drives automatic light sleep.
 */
template <class Rep, class Period>
void free_rtos_sleep(const std::chrono::duration<Rep, Period> d) {
    if constexpr (std::is_same_v<std::chrono::duration<Rep, Period>, std::chrono::milliseconds>) {
        vTaskDelay(pdMS_TO_TICKS(d.count()));
    } else {
        vTaskDelay(pdMS_TO_TICKS(std::chrono::duration_cast<std::chrono::milliseconds>(d).count()));
    }
}

/**
 * Blocks the application task until the next scheduled measurement cycle while the Zigbee stack
 * remains joined and can use automatic light sleep between polls.
 */
void wait_for_next_cycle_light_sleep() {
    ESP_LOGI(TAG, "Waiting %lld seconds before the next measurement cycle while Zigbee stays joined.", std::chrono::duration_cast<std::chrono::seconds>(app::RetainedState::WAKE_INTERVAL).count());
    free_rtos_sleep(app::RetainedState::WAKE_INTERVAL);
}

/**
 * Holds the device awake long enough for the first Home Assistant interview to finish.
 */
void keep_awake_for_initial_interview() {
    ESP_LOGI(TAG, "Keeping the device awake for %lld seconds so the initial Home Assistant interview can complete.", std::chrono::duration_cast<std::chrono::seconds>(INITIAL_INTERVIEW_AWAKE_TIME).count());
    free_rtos_sleep(INITIAL_INTERVIEW_AWAKE_TIME);
}

/**
 * Keeps the application awake until Zigbee finishes joining or rejoining a network.
 *
 * The Zigbee stack already retries commissioning internally. This helper only prevents the main
 * application from entering its normal timed sleep path while the device is still factory new or
 * otherwise disconnected from the mesh.
 */
void wait_until_connected_awake() {
    constexpr std::chrono::seconds LOG_INTERVAL{30};
    ESP_LOGW(TAG, "Zigbee is not connected. Sleep is disabled until the device joins a network.");
    while (!zigbee::ZDevice::get_instance()->has_connection()) {
        if (zigbee::ZDevice::get_instance()->wait_for_connection(LOG_INTERVAL)) {
            return;
        }
        ESP_LOGI(TAG, "Still waiting for a Zigbee connection before entering the configured sleep mode.");
    }
}

/**
 * Waits for a connected Zigbee session if the current cycle needs to publish data.
 */
[[nodiscard]] bool ensure_zigbee_connection(const app::RetainedState& retainedState) {
    if (zigbee::ZDevice::get_instance()->has_connection()) {
        return true;
    }

    const auto joinTimeout = retainedState.is_initial_startup() ? INITIAL_JOIN_TIMEOUT : REJOIN_TIMEOUT;
    return zigbee::ZDevice::get_instance()->wait_for_connection(joinTimeout);
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

/**
 * Executes one sensor-measurement and optional Zigbee publish cycle.
 */
void run_measurement_cycle(app::RetainedState& retainedState, const std::shared_ptr<devices::AbstractDeviceEventListener>& deviceListener) {
    const models::EnvironmentalReadings environmentalReadings = collect_environmental_readings();
    const models::QuantizedEnvironmentalReadings quantizedEnvironmental = models::quantize(environmentalReadings);
    const std::optional<models::BatteryReading> batteryReading = read_battery();
    const std::optional<models::QuantizedBatteryReading> quantizedBattery = batteryReading ? std::make_optional(models::quantize(*batteryReading)) : std::nullopt;

    // Temperature and humidity are refreshed on every wake; pressure and CO2 still rely on change detection.
    const bool shouldReportEnvironment = retainedState.should_report_environment(quantizedEnvironmental);
    // Battery is reported on startup, on percentage changes, and once per day as a keepalive.
    const bool shouldReportBattery = quantizedBattery && retainedState.should_report_battery(*quantizedBattery);

    if (!shouldReportEnvironment && !shouldReportBattery && !retainedState.is_initial_startup()) {
        ESP_LOGI(TAG, "No reportable attribute changed in this measurement cycle.");
        return;
    }

    zigbee::PublishRequest publishRequest{};
    if (shouldReportEnvironment) {
        publishRequest.environmental = quantizedEnvironmental;
    }
    if (shouldReportBattery && quantizedBattery) {
        publishRequest.battery = *quantizedBattery;
    }

    if constexpr (!USE_LIGHT_SLEEP) {
        zigbee::ZDevice::get_instance()->set_device_listener(deviceListener);
        zigbee::ZDevice::get_instance()->init();
    }

    if (!ensure_zigbee_connection(retainedState)) {
        ESP_LOGW(TAG, "Zigbee network is not available in this measurement cycle.");
        return;
    }

    zigbee::ZDevice::get_instance()->publish(publishRequest);
    std::this_thread::sleep_for(SESSION_SETTLE_TIME);

    if (shouldReportEnvironment) {
        retainedState.mark_environment_reported(quantizedEnvironmental);
    }
    if (shouldReportBattery && quantizedBattery) {
        retainedState.mark_battery_reported(*quantizedBattery);
    }

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
    if (retainedState.is_initial_startup()) {
        keep_awake_for_initial_interview();
        zigbee::ZDevice::get_instance()->set_connected_light_sleep_allowed(true);
    }
#endif
    retainedState.mark_startup_report_completed();
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
    init_power_management();

    std::shared_ptr<devices::AbstractDeviceEventListener> deviceListener = devices::create_device_event_listener();
    deviceListener->init();

    app::RetainedState retainedState{};

    if constexpr (USE_LIGHT_SLEEP) {
        zigbee::ZDevice::get_instance()->set_device_listener(deviceListener);
        zigbee::ZDevice::get_instance()->set_connected_light_sleep_allowed(!retainedState.is_initial_startup());
        zigbee::ZDevice::get_instance()->init();
    }

    while (true) {
        run_measurement_cycle(retainedState, deviceListener);

        if (!zigbee::ZDevice::get_instance()->has_connection()) {
            wait_until_connected_awake();
            continue;
        }

        if constexpr (USE_LIGHT_SLEEP) {
            wait_for_next_cycle_light_sleep();
            retainedState.advance_wake_cycle();
        } else {
            schedule_next_wake_and_sleep(deviceListener);
        }
    }
}

extern "C" void app_main(void) {
    mainLoop();
}
