#include "sensors/Battery.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <cassert>
#include <optional>
#include <vector>

namespace sensors {
Battery::Battery()
        : channels(std::vector<espp::AdcConfig>{{.unit = ADC_UNIT_2, .channel = ADC_CHANNEL_2, .attenuation = ADC_ATTEN_DB_12}}), adc(espp::OneshotAdc({
                                                                                                                                          .unit = ADC_UNIT_1,
                                                                                                                                          .channels = channels,
                                                                                                                                  })) {}

bool Battery::init() {
    // TODO calibrate
    return true;
}

// 3.7V -> 2000
// 2.5V -> 1350
// 1.75V -> 950

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