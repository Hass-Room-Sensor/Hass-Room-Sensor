#pragma once

#include "esp_sleep.h"
#include "models/SensorModels.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace app {
/**
 * Stores the last successfully published values across deep-sleep wake-ups.
 */
class RetainedState {
  public:
    /**
     * Deep-sleep interval between wake cycles.
     */
    static constexpr std::chrono::minutes WAKE_INTERVAL{5};

    /**
     * Number of five-minute wake cycles in roughly one day.
     */
    static constexpr uint32_t BATTERY_REPORT_INTERVAL_CYCLES = 24U * 60U / WAKE_INTERVAL.count();

    /**
     * Loads or initializes the RTC-retained state for the current boot.
     */
    RetainedState();

    /** Clears all RTC-retained sensor and reporting state. */
    static void reset();

    /**
     * Returns true until the first successful publish after power-on or reset.
     */
    [[nodiscard]] bool is_initial_startup() const;
    /**
     * Returns true when the current wake cycle should publish environmental data.
     *
     * Temperature and humidity should be reported on every wake so Home Assistant receives a fresh
     * sample timestamp even when the measurement backend is mocked or the values are stable.
     */
    [[nodiscard]] bool should_report_environment(const models::QuantizedEnvironmentalReadings& readings) const;
    /**
     * Returns true on initial startup, when the battery percentage changed, or once per day.
     */
    [[nodiscard]] bool should_report_battery(const models::QuantizedBatteryReading& reading) const;
    /**
     * Advances the retained wake-cycle counter for long-lived runtime modes such as light sleep.
     */
    void advance_wake_cycle();

    /**
     * Persists the last successfully published environmental values in RTC memory.
     */
    void mark_environment_reported(const models::QuantizedEnvironmentalReadings& readings);
    /**
     * Persists the last successfully published battery values in RTC memory.
     */
    void mark_battery_reported(const models::QuantizedBatteryReading& reading);
    /**
     * Marks the initial startup report as completed.
     */
    void mark_startup_report_completed();

  private:
    /**
     * RTC-retained values that survive deep sleep but reset after a power cycle or flash/boot event.
     *
     * The first field is a layout signature. On boot the firmware checks that signature before trusting
     * the remaining bytes. That protects the code from interpreting uninitialized RTC memory or stale
     * bytes from an older firmware layout as valid sensor history.
     */
    struct StoredState {
        /**
         * Layout signature written after initialization.
         * Indicates if the data format contained in this state is still valid.
         * Will change in case the data format changed so all stored will be reinitialized.
         */
        uint32_t version{};
        /** Number of non-deep-sleep boots observed by the firmware. */
        uint32_t boot_count{};
        /** Number of completed five-minute wake cycles since retained state initialization. */
        uint32_t wake_cycle{};
        /** Wake-cycle counter recorded when the last battery report was sent successfully. */
        uint32_t last_battery_report_cycle{};
        /** True until the first successful report after a cold boot or non-deep-sleep restart. */
        bool initial_report_pending{true};

        /** True once a temperature value has been published successfully. */
        bool has_temperature{false};
        /** Last published temperature in 0.01 degrees Celsius. */
        int16_t temperature_centi_celsius{};

        /** True once a humidity value has been published successfully. */
        bool has_humidity{false};
        /** Last published humidity in 0.01 percent RH. */
        uint16_t humidity_centi_percent{};

        /** True once a pressure value has been published successfully. */
        bool has_pressure{false};
        /** Last published pressure in 0.1 kPa. */
        int16_t pressure_deci_kpa{};

        /** True once a CO2 value has been published successfully. */
        bool has_co2{false};
        /** Last published CO2 concentration in ppm. */
        uint16_t co2_ppm{};

        /** True once a battery value has been published successfully. */
        bool has_battery{false};
        /** Last published battery voltage in millivolts. */
        uint16_t battery_millivolts{};
        /** Last published battery percentage. */
        uint8_t battery_percentage{};
    } __attribute__((aligned(64)));

    /**
     * Returns the singleton RTC-retained backing store.
     *
     * The returned object lives in RTC slow memory via `RTC_DATA_ATTR`, so it survives ESP deep sleep
     * and is reused by the next wake cycle until a cold boot, flash/reset event, or validation failure
     * causes reinitialization.
     */
    static StoredState& state();
    /**
     * Stored-state layout version.
     *
     * Any sufficiently unique constant would work here, but it should stay stable as long as the
     * persisted `StoredState` layout and semantics remain compatible.
     *
     * Change this value intentionally when the retained-state format changes in a way that makes old
     * RTC contents unsafe to reuse. The practical effect is that the next boot will treat RTC memory as
     * uninitialized, discard the previous retained values, and start with a fresh startup report.
     */
    static constexpr uint32_t VERSION = 0x42424242U;

    /** Pointer to the RTC-retained backing store used by this wake cycle. */
    StoredState* storedState_{nullptr};
};
} // namespace app
