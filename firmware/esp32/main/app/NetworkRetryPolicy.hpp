#pragma once

#include <array>
#include <chrono>
#include <cstdint>

namespace app {
/** Selects Zigbee connection-attempt timeouts and outage retry backoff. */
class NetworkRetryPolicy {
  public:
    /** Maximum first connection attempt for a factory-new device. */
    static constexpr std::chrono::seconds factory_new_attempt{std::chrono::minutes{5}};
    /** Maximum connection attempt for a device that has already been commissioned. */
    static constexpr std::chrono::seconds commissioned_attempt{std::chrono::seconds{30}};

    /** Returns the connection-attempt timeout appropriate to the device's commissioning state. */
    [[nodiscard]] static constexpr std::chrono::seconds attempt_timeout(bool factoryNew) {
        return factoryNew ? factory_new_attempt : commissioned_attempt;
    }

    /** Returns exponential retry backoff, capped at fifteen minutes. */
    [[nodiscard]] static constexpr std::chrono::minutes retry_delay(uint8_t failedAttempts) {
        constexpr std::array delays{
                std::chrono::minutes{1}, std::chrono::minutes{2}, std::chrono::minutes{4}, std::chrono::minutes{8}, std::chrono::minutes{15},
        };
        return delays[failedAttempts < delays.size() ? failedAttempts : delays.size() - 1];
    }
};
} // namespace app
