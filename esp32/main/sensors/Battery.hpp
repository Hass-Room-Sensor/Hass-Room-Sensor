#pragma once

#include "oneshot_adc.hpp"
#include "sensors/IBattery.hpp"
#include <optional>

namespace sensors {
class Battery : public IBattery {
  private:
    std::vector<espp::AdcConfig> channels;
    espp::OneshotAdc adc;

  public:
    Battery();
    Battery(Battery&&) = default;
    Battery(const Battery&) = default;
    Battery& operator=(Battery&&) = default;
    Battery& operator=(const Battery&) = default;
    ~Battery() override = default;

    [[nodiscard]] bool init() override;

    [[nodiscard]] std::optional<int> read_measurement() override;
};

} // namespace sensors