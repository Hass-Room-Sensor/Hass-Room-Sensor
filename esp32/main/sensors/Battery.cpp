#include "sensors/Battery.hpp"
#include "esp_adc/adc_oneshot.h"
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
    const std::optional<int> measurement = adc.read_mv(channels[0]);

    if (!measurement) {
        return std::nullopt;
    }

    // We read the following data for calibration form the adc
    // Real Voltage -> ADC Value
    // 3700mV -> 2000
    // 2500mV -> 1350
    // 1750mV -> 950
    //
    // This results in the following formula for converting an ADC sample to mV:
    return *measurement * 1.85;
}

} // namespace sensors