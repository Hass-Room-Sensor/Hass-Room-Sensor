#include "sensors/Battery.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <cassert>
#include <optional>
#include <vector>

namespace sensors {
namespace {
/**
 * Battery divider input on the XIAO ESP32-C6 carrier.
 *
 * The KiCad netlist connects `U_Bat_ADC` to XIAO pin `D2`, and ESP-IDF documents that ESP32-C6
 * `GPIO2` is exposed as `ADC1_CH2`. Using ADC unit 2 on this target is invalid and aborts during
 * driver initialization, so both the ADC unit and the channel metadata must point at ADC1.
 *
 * Sources:
 * - `ecad/HassRoomSensor.kicad_sch`
 * - ESP-IDF GPIO reference: `docs/en/api-reference/peripherals/gpio/esp32c6.inc`
 * - ESP-IDF ADC channel map: `components/soc/esp32c6/include/soc/adc_channel.h`
 */
constexpr adc_unit_t BATTERY_ADC_UNIT = ADC_UNIT_1;
constexpr adc_channel_t BATTERY_ADC_CHANNEL = ADC_CHANNEL_2;
} // namespace

Battery::Battery()
        : channels(std::vector<espp::AdcConfig>{{.unit = BATTERY_ADC_UNIT, .channel = BATTERY_ADC_CHANNEL, .attenuation = ADC_ATTEN_DB_12}}),
          adc(espp::OneshotAdc({
                  .unit = BATTERY_ADC_UNIT,
                  .channels = channels,
          })) {}

bool Battery::init() {
    // TODO calibrate
    return true;
}

std::optional<int> Battery::read_milli_volt() {
    // We expect exactly one channel being configured
    assert(channels.size() == 1);

    // The maximum amount of attempts we try to read the battery ADC value before we give up.
    constexpr size_t maxReadAttempts = 64;
    // The amount of values measured to compute an average value from.
    constexpr int maxValidMeasurementCount = 8;

    // Try reading up to 'maxReadAttempts' times until we have up to 'expectedAverageCount' values.
    // Then calculate the average. Discard any 0 or std::nullopt during that process.
    int measurementSum = 0;
    int validMeasurementCount = 0;
    size_t attempts = 0;
    for (; attempts < maxReadAttempts && validMeasurementCount < maxValidMeasurementCount; attempts++) {
        const std::optional<int> measurement = adc.read_mv(channels[0]);
        if (measurement && *measurement > 0) {
            measurementSum += *measurement;
            validMeasurementCount++;
        }
    }

    if (validMeasurementCount > 0) {
        ESP_LOGD(TAG, "Reading %d valid battery measurement took %lu attempts.", validMeasurementCount, attempts);

        // We read the following data for calibration form the adc
        // Real Voltage -> ADC Value
        // 3700mV -> 2000
        // 2500mV -> 1350
        // 1750mV -> 950
        //
        // This results in the following formula for converting an ADC sample to mV:
        return static_cast<int>((static_cast<double>(measurementSum) / validMeasurementCount) * 1.85);
    }
    ESP_LOGE(TAG, "Failed to read valid battery measurements after %lu attempts.", attempts);
    return std::nullopt;
}

} // namespace sensors
