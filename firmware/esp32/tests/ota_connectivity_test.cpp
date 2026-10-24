#include "zigbee/ZigbeeDeviceState.hpp"
#include <array>
#include <cassert>
#include <utility>

/** Verifies that entering OTA preserves both connected and disconnected network states. */
int main() {
    using zigbee::ZigbeeDeviceState;
    bool connected = true;
    for (const auto [state, expected] : std::array{std::pair{ZigbeeDeviceState::OTA, true}, std::pair{ZigbeeDeviceState::CONNECTED, true}, std::pair{ZigbeeDeviceState::CONNECTING, false}, std::pair{ZigbeeDeviceState::OTA, false}}) {
        if (const auto update = zigbee::connection_update(state)) {
            connected = *update;
        }
        assert(connected == expected);
    }
    assert(!connected);
    assert(!zigbee::connection_update(ZigbeeDeviceState::OTA));
    assert(zigbee::connection_update(ZigbeeDeviceState::SETUP) == false);
}
