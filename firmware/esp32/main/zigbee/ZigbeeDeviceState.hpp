#pragma once

#include <cstdint>

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
} // namespace zigbee
