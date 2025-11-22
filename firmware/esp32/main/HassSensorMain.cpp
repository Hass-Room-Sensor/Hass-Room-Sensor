#include "defs/DeviceDefs.hpp"
#include "devices/AbstractDeviceEventListener.hpp"
#include "devices/DeviceEventListenerFactory.hpp"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "sensors/AbstractScd41.hpp"
#include "sensors/Battery.hpp"
#include "sensors/IBattery.hpp"
#include "soc/gpio_num.h"
#include "zigbee/ZDevice.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_MOCK
#include "sensors/Scd41Mock.hpp"
#else
#include "sensors/Scd41.hpp"
#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_MOCK
namespace {
/**
 * Log tag.
 **/
const char* TAG = "hassSensor";
} // namespace

void mainLoop() {
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    ESP_LOGI(TAG, "Starting HASS environment sensor version %d.%d.%d", CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MAJOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_MINOR, CONFIG_HASS_ENVIRONMENT_SENSOR_VERSION_PATCH);

    // Initialize the flash:
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Initialize device
    std::shared_ptr<devices::AbstractDeviceEventListener> deviceListener = devices::create_device_event_listener();
    assert(deviceListener);
    deviceListener->init();
    zigbee::ZDevice::get_instance()->set_device_listener(deviceListener);

    // Initialize the SCD41 sensor:
    std::unique_ptr<sensors::AbstractScd41> scd41{nullptr};

#ifdef CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_MOCK
    scd41 = std::make_unique<sensors::Scd41Mock>();
#else
    scd41 = std::make_unique<sensors::Scd41>(HASS_SENSOR_SCD4X_SDA_GPIO, HASS_SENSOR_SCD4X_SCL_GPIO);
#endif // CONFIG_HASS_ENVIRONMENT_SENSOR_SCD41_MOCK

    if (!scd41->init()) {
        deviceListener->indicate_error();
        ESP_LOGE(TAG, "Initializing SCD41 failed. Rebooting...");
        esp_restart();
    }

    // Initialize battery ADC
    sensors::Battery battery;
    if (!battery.init()) {
        deviceListener->indicate_error();
        ESP_LOGE(TAG, "Initializing Battery failed. Rebooting...");
        esp_restart();
    }

    // Setup ZigBee device with initial measurements:
    std::optional<sensors::measurement_t> measurement = std::nullopt;
    do {
        if (scd41->get_data_ready_status()) {
            measurement = scd41->read_measurement();
            if (measurement) {
                ESP_LOGI(TAG, "Initial measurement: %d ppm, %.2lf °C, %.2lf %%", measurement->co2, measurement->temp, measurement->hum);
                zigbee::ZDevice::get_instance()->init(measurement->temp, measurement->hum, measurement->co2);
            }
        } else {
            ESP_LOGI(TAG, "Waiting for initial measurements. SCD41 is not ready yet. Sleeping for a second before rechecking...");
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
    } while (!measurement);

    ESP_LOGI(TAG, "Everything initialized.");

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK && (state == ESP_OTA_IMG_PENDING_VERIFY)) {
        // Sanity checks passed; we accept this image
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
        ESP_LOGI(TAG, "Marked current partition as OK and active to avoid rolling back to the old version.");
    }


    // Main loop:
    while (true) {
        ESP_LOGI(TAG, "Data ready. Reading...");
        measurement = scd41->read_measurement();
        if (measurement) {
            ESP_LOGI(TAG, "[Measurement]: %d ppm, %.2lf °C, %.2lf %%", measurement->co2, measurement->temp, measurement->hum);
            zigbee::ZDevice::get_instance()->update_temp(measurement->temp);
            zigbee::ZDevice::get_instance()->update_hum(measurement->hum);
            zigbee::ZDevice::get_instance()->update_co2(measurement->co2);
        }

        const std::optional<int> battery_mV = battery.read_milli_volt();
        if (battery_mV) {
            ESP_LOGI(TAG, "[Measurement]: %d mV", *battery_mV);
            // Convert millivolts to percentage (0-100). Use a simple linear mapping
            // and clamp to [0,100].
            constexpr int MV_MIN = 3000; // 0%
            constexpr int MV_MAX = 4200; // 100%
            int pct = (*battery_mV - MV_MIN) * 100 / (MV_MAX - MV_MIN);
            if (pct < 0) {
                pct = 0;
            }
            if (pct > 100) {
                pct = 100;
            }

            uint8_t batteryPercentage = static_cast<uint8_t>(pct);
            uint16_t batteryMv = static_cast<uint16_t>(*battery_mV);
            zigbee::ZDevice::get_instance()->update_battery(batteryPercentage, batteryMv);
        } else {
            ESP_LOGW(TAG, "Failed to read battery measurement.");
        }

        std::this_thread::sleep_for(std::chrono::seconds(30)); // Looks like we can update values via ZigBee every 30 seconds anyway. Ref: https://github.com/espressif/esp-zigbee-sdk/issues/65
    }
}

/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(void) {
    mainLoop();
}
