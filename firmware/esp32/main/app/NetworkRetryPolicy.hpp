#pragma once

#include <array>
#include <chrono>
#include <cstdint>

namespace app {
class NetworkRetryPolicy {
  public:
    static constexpr std::chrono::seconds factory_new_attempt{std::chrono::minutes{5}};
    static constexpr std::chrono::seconds commissioned_attempt{std::chrono::seconds{30}};

    [[nodiscard]] static constexpr std::chrono::seconds attempt_timeout(bool factoryNew) {
        return factoryNew ? factory_new_attempt : commissioned_attempt;
    }

    [[nodiscard]] static constexpr std::chrono::minutes retry_delay(uint8_t failedAttempts) {
        constexpr std::array delays{
                std::chrono::minutes{1}, std::chrono::minutes{2}, std::chrono::minutes{4}, std::chrono::minutes{8}, std::chrono::minutes{15},
        };
        return delays[failedAttempts < delays.size() ? failedAttempts : delays.size() - 1];
    }
};
} // namespace app
