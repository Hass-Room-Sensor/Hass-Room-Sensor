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

std::optional<int> Battery::read_measurement() {
    // We expect exactly one channel being configured
    assert(channels.size() == 1);
    return adc.read_mv(channels[0]);
}

} // namespace sensors