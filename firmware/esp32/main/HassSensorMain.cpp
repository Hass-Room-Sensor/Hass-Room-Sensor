#include "app/PeriodicDeadline.hpp"
#include "app/RetainedState.hpp"
#include "defs/DeviceDefs.hpp"
#include "devices/AbstractDeviceEventListener.hpp"
#include "devices/DeviceEventListenerFactory.hpp"
#include "esp_log.h"
#include "esp_log_level.h"
#include "esp_ota_ops.h"
#include "models/EnvironmentalSource.hpp"
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
#include "zigbee/StackLock.hpp"
#include "zigbee/ZDevice.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <thread>

#if defined(CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_MOCK)
#include "sensors/Scd41Mock.hpp"
#elif defined(CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_REAL)
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
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_DISABLED
    static_cast<void>(pressureHpa);
    return std::nullopt;
#else
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
#endif
}

/**
 * Reads all available environmental sensors for the current wake cycle.
 *
 * The BME690 is sampled first so its pressure value can be injected into the SCD41 single-shot
 * command path for a more accurate CO2 reading.
 */
[[nodiscard]] models::EnvironmentalReadings collect_environmental_readings() {
    models::EnvironmentalReadings bmeReadings{};

    const std::optional<sensors::bme690_measurement_t> bme690 = read_bme690();
    if (bme690) {
        bmeReadings.pressure_hpa = bme690->pressure_hpa;
        bmeReadings.temperature_celsius = bme690->temp_celsius;
        bmeReadings.humidity_percent = bme690->humidity_percent;
    }

    const std::optional<sensors::measurement_t> scd41 = read_scd41(bme690 ? std::make_optional(static_cast<uint16_t>(std::lround(bme690->pressure_hpa))) : std::nullopt);
    models::EnvironmentalReadings scdReadings{};
    if (scd41) {
        scdReadings.co2_ppm = scd41->co2;
        scdReadings.temperature_celsius = scd41->temp;
        scdReadings.humidity_percent = scd41->hum;
    }

    return models::select_environmental_sources(bmeReadings, scdReadings);
}

/**
 * Enables automatic light sleep for the long-lived Zigbee sleepy-end-device mode.
 */
void init_power_management() {
#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
#ifdef CONFIG_PM_ENABLE
    esp_pm_config_t pmConfig{};
    pmConfig.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    pmConfig.min_freq_mhz = CONFIG_XTAL_FREQ; // Minimum frequency of the crystal. Saves power when the chip is for example waiting for I/O.
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

/** Enters timed deep sleep only after OTA ends, excluding a new OTA start during the transition. */
[[noreturn]] void start_deep_sleep(std::chrono::microseconds duration, const std::shared_ptr<devices::AbstractDeviceEventListener>& deviceListener) {
    for (;;) {
        zigbee::ZDevice::get_instance()->wait_for_ota_completion();
        const zigbee::StackLock stackLock;
        if (zigbee::ZDevice::get_instance()->is_ota_active()) {
            continue;
        }
        if (deviceListener) {
            deviceListener->prepare_for_deep_sleep();
        }
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(duration.count()));
        esp_deep_sleep_start();
    }
}

/** Returns the joined Zigbee session to deep sleep until the next measurement interval. */
[[noreturn]] void schedule_next_wake_and_sleep(const std::shared_ptr<devices::AbstractDeviceEventListener>& deviceListener) {
    ESP_LOGI(TAG, "Entering deep sleep for %lld seconds.", std::chrono::duration_cast<std::chrono::seconds>(app::RetainedState::WAKE_INTERVAL).count());
    start_deep_sleep(app::RetainedState::WAKE_INTERVAL, deviceListener);
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
void wait_for_next_cycle_light_sleep(app::PeriodicDeadline& deadline) {
    const auto now = app::PeriodicDeadline::Clock::now();
    if (now < deadline.next()) {
        free_rtos_sleep(std::chrono::duration_cast<std::chrono::milliseconds>(deadline.next() - now));
    }
    deadline.advance_past(app::PeriodicDeadline::Clock::now());
}

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SLEEP_MODE_LIGHT_SLEEP
/**
 * Holds the device awake long enough for the first Home Assistant interview to finish.
 */
void keep_awake_for_initial_interview() {
    ESP_LOGI(TAG, "Keeping the device awake for %lld seconds so the initial Home Assistant interview can complete.", std::chrono::duration_cast<std::chrono::seconds>(INITIAL_INTERVIEW_AWAKE_TIME).count());
    free_rtos_sleep(INITIAL_INTERVIEW_AWAKE_TIME);
}
#endif

/**
 * Waits for a connected Zigbee session if the current cycle needs to publish data.
 */
[[nodiscard]] bool ensure_zigbee_connection() {
    zigbee::ZDevice& device = *zigbee::ZDevice::get_instance();
    if (device.has_connection()) {
        return true;
    }

    // The BDB first-start/reboot signal is the authoritative source for whether this device has
    // stored network credentials. Allow the Zigbee task to deliver it before selecting a timeout.
    constexpr std::chrono::seconds NETWORK_CLASSIFICATION_TIMEOUT{5};
    if (!device.wait_for_network_classification(NETWORK_CLASSIFICATION_TIMEOUT)) {
        ESP_LOGW(TAG, "Zigbee network state was not classified in time; using the commissioned retry timeout.");
    }
    const bool factoryNew = device.is_factory_new();
    const std::chrono::milliseconds attemptTimeout = app::NetworkRetryPolicy::attempt_timeout(factoryNew);
    ESP_LOGW(TAG, "Waiting up to %lld seconds for the %s Zigbee connection attempt.", std::chrono::duration_cast<std::chrono::seconds>(attemptTimeout).count(), factoryNew ? "factory-new" : "commissioned");
    return device.wait_for_connection(attemptTimeout);
}

/**
 * Keeps the awake retry alive only while an explicit BOOT maintenance gesture/window is active.
 */
bool wait_for_connection_during_maintenance() {
    zigbee::ZDevice& device = *zigbee::ZDevice::get_instance();
    while (device.has_explicit_maintenance()) {
        if (device.wait_for_connection(std::chrono::seconds{1})) {
            return true;
        }
    }
    return device.has_connection();
}

/**
 * Backs off with RTC-retained state while the device has no usable Zigbee parent/coordinator.
 */
[[noreturn]] void schedule_outage_retry(app::RetainedState& retainedState, const std::shared_ptr<devices::AbstractDeviceEventListener>& deviceListener) {
    const auto retryDelay = retainedState.outage_retry_delay();
    retainedState.note_outage_attempt_failed();
    ESP_LOGW(TAG, "Zigbee remains disconnected; retrying in %lld minutes.", retryDelay.count());
    start_deep_sleep(retryDelay, deviceListener);
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

#if defined(CONFIG_HASS_ENVIRONMENT_SENSOR_PM_DIAGNOSTICS) && defined(CONFIG_PM_ENABLE)
/** Prints cumulative PM lock and sleep statistics for the current boot. */
void dump_pm_diagnostics() {
    if (esp_pm_dump_locks(stdout) != ESP_OK) {
        ESP_LOGW(TAG, "Could not dump power-management diagnostics.");
    }
}
#endif

/**
 * Executes one sensor-measurement and optional Zigbee publish cycle.
 */
void run_measurement_cycle(app::RetainedState& retainedState, const std::shared_ptr<devices::AbstractDeviceEventListener>& deviceListener) {
    const models::EnvironmentalReadings environmentalReadings = collect_environmental_readings();
    const models::QuantizedEnvironmentalReadings quantizedEnvironmental = models::quantize(environmentalReadings);
    const std::optional<models::BatteryReading> batteryReading = retainedState.should_sample_battery() ? read_battery() : std::nullopt;
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
    app::PeriodicDeadline environmentalDeadline{app::PeriodicDeadline::Clock::now(), app::RetainedState::WAKE_INTERVAL};

    zigbee::ZDevice::get_instance()->set_device_listener(deviceListener);
    zigbee::ZDevice::get_instance()->set_connected_light_sleep_allowed(!retainedState.is_initial_startup());
    zigbee::ZDevice::get_instance()->init();
    const auto startupMaintenance = app::AwakeReasons::startup_maintenance(retainedState.is_timer_wakeup());
    if (startupMaintenance > std::chrono::seconds::zero()) {
        zigbee::ZDevice::get_instance()->open_maintenance_window(startupMaintenance);
    }
#if defined(CONFIG_HASS_ENVIRONMENT_SENSOR_PM_DIAGNOSTICS) && defined(CONFIG_PM_ENABLE)
    dump_pm_diagnostics();
#endif

    while (true) {
        zigbee::ZDevice::get_instance()->wait_for_ota_completion();
        if (!ensure_zigbee_connection()) {
            if (wait_for_connection_during_maintenance()) {
                retainedState.reset_outage_retry();
            } else {
                schedule_outage_retry(retainedState, deviceListener);
            }
        } else {
            retainedState.reset_outage_retry();
        }

        run_measurement_cycle(retainedState, deviceListener);

        if (!zigbee::ZDevice::get_instance()->has_connection()) {
            if (wait_for_connection_during_maintenance()) {
                retainedState.reset_outage_retry();
                continue;
            }
            schedule_outage_retry(retainedState, deviceListener);
        }

        if constexpr (USE_LIGHT_SLEEP) {
            wait_for_next_cycle_light_sleep(environmentalDeadline);
#if defined(CONFIG_HASS_ENVIRONMENT_SENSOR_PM_DIAGNOSTICS) && defined(CONFIG_PM_ENABLE)
            dump_pm_diagnostics();
#endif
            retainedState.advance_wake_cycle();
        } else {
            schedule_next_wake_and_sleep(deviceListener);
        }
    }
}

extern "C" void app_main(void) {
    mainLoop();
}
