#include "app/RetainedState.hpp"

#include "esp_log.h"

namespace app {
namespace {
const char* TAG = "RetainedState";
} // namespace

RetainedState::RetainedState() : storedState_(&state()) {
    const uint32_t wakeupCauses = esp_sleep_get_wakeup_causes();
    const bool resumedFromDeepSleep = (wakeupCauses & (1UL << static_cast<uint32_t>(ESP_SLEEP_WAKEUP_TIMER))) != 0U;

    // Reject RTC contents that were never initialized by this firmware instance or belong to an
    // incompatible retained-state layout from an earlier build.
    if (storedState_->version != VERSION) {
        *storedState_ = StoredState{
                .version = VERSION,
                .boot_count = 1,
                .wake_cycle = 0,
                .last_battery_report_cycle = 0,
                .initial_report_pending = true,
        };
        ESP_LOGI(TAG, "Initialized retained state storage.");
        return;
    }

    if (resumedFromDeepSleep) {
        storedState_->wake_cycle += 1;
    } else {
        storedState_->boot_count += 1;
        storedState_->initial_report_pending = true;
        ESP_LOGI(TAG, "Detected a non-deep-sleep restart. A fresh startup report will be sent.");
    }
}

void RetainedState::reset() {
    state() = StoredState{};
    ESP_LOGI(TAG, "Cleared retained state storage.");
}

RetainedState::StoredState& RetainedState::state() {
    // RTC memory survives deep sleep, which makes it a good fit for "report only on change"
    // bookkeeping without paying the cost and wear of an NVS write every five minutes.
    static RTC_DATA_ATTR StoredState retainedState{};
    return retainedState;
}

bool RetainedState::is_initial_startup() const {
    return storedState_->initial_report_pending;
}

bool RetainedState::should_report_environment(const models::QuantizedEnvironmentalReadings& readings) const {
    if (storedState_->initial_report_pending) {
        return true;
    }

    // Temperature and humidity are refreshed every wake cycle so downstream consumers such as
    // Home Assistant can observe a new sample even if the quantized value itself did not move.
    if (readings.temperature_centi_celsius || readings.humidity_centi_percent) {
        return true;
    }

    if (readings.temperature_centi_celsius && (!storedState_->has_temperature || storedState_->temperature_centi_celsius != *readings.temperature_centi_celsius)) {
        return true;
    }
    if (readings.humidity_centi_percent && (!storedState_->has_humidity || storedState_->humidity_centi_percent != *readings.humidity_centi_percent)) {
        return true;
    }
    if (readings.pressure_deci_kpa && (!storedState_->has_pressure || storedState_->pressure_deci_kpa != *readings.pressure_deci_kpa)) {
        return true;
    }
    if (readings.co2_ppm && (!storedState_->has_co2 || storedState_->co2_ppm != *readings.co2_ppm)) {
        return true;
    }

    return false;
}

bool RetainedState::should_report_battery(const models::QuantizedBatteryReading& reading) const {
    if (storedState_->initial_report_pending) {
        return true;
    }
    if (!storedState_->has_battery) {
        return true;
    }
    if (storedState_->battery_percentage != reading.percentage) {
        return true;
    }
    return (storedState_->wake_cycle - storedState_->last_battery_report_cycle) >= BATTERY_REPORT_INTERVAL_CYCLES;
}

void RetainedState::advance_wake_cycle() {
    storedState_->wake_cycle += 1;
}

void RetainedState::mark_environment_reported(const models::QuantizedEnvironmentalReadings& readings) {
    if (readings.temperature_centi_celsius) {
        storedState_->has_temperature = true;
        storedState_->temperature_centi_celsius = *readings.temperature_centi_celsius;
    }
    if (readings.humidity_centi_percent) {
        storedState_->has_humidity = true;
        storedState_->humidity_centi_percent = *readings.humidity_centi_percent;
    }
    if (readings.pressure_deci_kpa) {
        storedState_->has_pressure = true;
        storedState_->pressure_deci_kpa = *readings.pressure_deci_kpa;
    }
    if (readings.co2_ppm) {
        storedState_->has_co2 = true;
        storedState_->co2_ppm = *readings.co2_ppm;
    }
}

void RetainedState::mark_battery_reported(const models::QuantizedBatteryReading& reading) {
    storedState_->has_battery = true;
    storedState_->battery_millivolts = reading.millivolts;
    storedState_->battery_percentage = reading.percentage;
    storedState_->last_battery_report_cycle = storedState_->wake_cycle;
}

void RetainedState::mark_startup_report_completed() {
    storedState_->initial_report_pending = false;
}
} // namespace app
