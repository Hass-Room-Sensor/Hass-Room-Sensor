#pragma once

#include "models/SensorModels.hpp"
#include <optional>

namespace zigbee {
/**
 * Requested readings for one measurement cycle.
 */
struct PublishRequest {
    /** Environmental attributes to publish during the current wake session. */
    std::optional<models::QuantizedEnvironmentalReadings> environmental{};
    /** Battery attributes to publish during the current wake session. */
    std::optional<models::QuantizedBatteryReading> battery{};
};

/** Confirmed report groups; failed or timed-out groups must remain eligible for retry. */
struct PublishResult {
    /** True when every requested environmental attribute was acknowledged at the APS layer. */
    bool environmental{false};
    /** True when both battery attributes were acknowledged at the APS layer. */
    bool battery{false};

    /** Returns true only when every group requested by this cycle was confirmed. */
    [[nodiscard]] bool complete_for(const PublishRequest& request) const {
        return (!request.environmental || environmental) && (!request.battery || battery);
    }
};

} // namespace zigbee
