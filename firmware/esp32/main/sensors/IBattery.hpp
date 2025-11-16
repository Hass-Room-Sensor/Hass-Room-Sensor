#pragma once

#include <optional>

namespace sensors {
class IBattery {
  public:
    IBattery() = default;
    IBattery(IBattery&&) = default;
    IBattery(const IBattery&) = default;
    IBattery& operator=(IBattery&&) = default;
    IBattery& operator=(const IBattery&) = default;
    virtual ~IBattery() = default;

    [[nodiscard]] virtual bool init() = 0;

    [[nodiscard]] virtual std::optional<int> read_milli_volt() = 0;
};

} // namespace sensors
