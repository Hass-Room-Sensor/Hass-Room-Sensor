#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace zigbee {
/** One standard ZCL Report Attributes frame, retained until its acknowledged APS transmission completes. */
class AttributeReport {
  public:
    /** Encodes a scalar attribute in little-endian ZCL format, independently of reporting thresholds. */
    template <class Value>
    AttributeReport(uint8_t sequence, uint16_t attribute, uint8_t type, Value value) : size_(6 + sizeof(Value)) {
        static_assert(std::is_arithmetic_v<Value> && sizeof(Value) <= 4);
        // Global command, server to client, no default response; APS acknowledgment is requested separately.
        frame_ = {0x18, sequence, 0x0a, static_cast<uint8_t>(attribute), static_cast<uint8_t>(attribute >> 8), type};
        auto bytes = std::bit_cast<std::array<uint8_t, sizeof(Value)>>(value);
        if constexpr (std::endian::native == std::endian::big) {
            std::ranges::reverse(bytes);
        }
        std::ranges::copy(bytes, std::span{frame_}.subspan(6).begin());
    }

    /** Returns the outgoing ASDU buffer required by the APS API. */
    [[nodiscard]] std::span<uint8_t> bytes() {
        return {frame_.data(), size_};
    }

    /** Matches the full confirmed ASDU, including transaction sequence, attribute, type, and value. */
    [[nodiscard]] bool matches(std::span<const uint8_t> confirmed) const {
        return std::ranges::equal(confirmed, std::span{frame_.data(), size_});
    }

  private:
    std::array<uint8_t, 10> frame_{};
    size_t size_;
};
} // namespace zigbee
