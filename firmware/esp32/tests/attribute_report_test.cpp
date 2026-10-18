#include "zigbee/AttributeReport.hpp"
#include <array>
#include <cassert>
#include <cstdint>
#include <span>

/** Checks report encoding and rejects confirmations for different or truncated transmissions. */
int main() {
    const zigbee::AttributeReport temperature{7, 0, 0x29, int16_t{-125}};
    const std::array<uint8_t, 8> expected{0x18, 7, 0x0a, 0, 0, 0x29, 0x83, 0xff};
    assert(temperature.matches(expected));
    assert(!temperature.matches(std::span{expected}.first(7)));
    auto stale = expected;
    stale.at(1) = 6;
    assert(!temperature.matches(stale));
    auto otherAttribute = expected;
    otherAttribute.at(3) = 1;
    assert(!temperature.matches(otherAttribute));
    const zigbee::AttributeReport battery{8, 0x21, 0x20, uint8_t{100}};
    const std::array<uint8_t, 7> expectedBattery{0x18, 8, 0x0a, 0x21, 0, 0x20, 100};
    assert(battery.matches(expectedBattery));
    const zigbee::AttributeReport co2{9, 0, 0x39, 1.0F};
    const std::array<uint8_t, 10> expectedCo2{0x18, 9, 0x0a, 0, 0, 0x39, 0, 0, 0x80, 0x3f};
    assert(co2.matches(expectedCo2));
}
