#pragma once

#include <cstdint>
#include <optional>

namespace zigbee {
enum class ZigbeeDeviceState : uint8_t {
    /**
     * The state when the device is factory reset and not yet associated with any ZigBee network yet.
     * Entering sleep is not possible in this state.
     **/
    SETUP,
    /**
     * The device is associated to a ZigBee network, but is not connected right now.
     * The device actively tries to join the network again.
     * Entering sleep is not possible in this state.
     **/
    CONNECTING,
    /**
     * The device is associated with a ZigBee network and connected to it.
     * The device can go to sleep.
     **/
    CONNECTED,
    /**
     * A OTA (Over The Air) update process is running.
     * The device can not go to sleep.
     **/
    OTA
};

/** Returns a connectivity update for network states; OTA changes activity without changing connectivity. */
[[nodiscard]] constexpr std::optional<bool> connection_update(ZigbeeDeviceState state) {
    if (state == ZigbeeDeviceState::OTA) {
        return std::nullopt;
    }
    return state == ZigbeeDeviceState::CONNECTED;
}
} // namespace zigbee
