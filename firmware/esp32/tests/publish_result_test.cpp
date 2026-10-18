#include "zigbee/PublishRequest.hpp"
#include <cassert>

/** Verifies that incomplete or failed report groups cannot complete startup reporting. */
int main() {
    zigbee::PublishRequest request{};
    request.environmental = models::QuantizedEnvironmentalReadings{.temperature_centi_celsius = 2150};
    assert(!zigbee::PublishResult{}.complete_for(request));
    assert((zigbee::PublishResult{.environmental = true}.complete_for(request)));
    request.battery = models::QuantizedBatteryReading{.millivolts = 3700, .percentage = 58};
    assert(!(zigbee::PublishResult{.environmental = true}.complete_for(request)));
    assert(!(zigbee::PublishResult{.battery = true}.complete_for(request)));
    assert((zigbee::PublishResult{.environmental = true, .battery = true}.complete_for(request)));
}
