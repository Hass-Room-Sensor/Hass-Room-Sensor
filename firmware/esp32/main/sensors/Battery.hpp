#pragma once

#include "oneshot_adc.hpp"
#include "sensors/IBattery.hpp"
#include <optional>

namespace sensors {
class Battery : public IBattery {
  private:
    std::vector<espp::AdcConfig> channels;
    espp::OneshotAdc adc;

    /**
     * Battery log tag.
     **/
    const char* TAG = "Battery";

  public:
    Battery();
    Battery(Battery&&) = default;
    Battery(const Battery&) = default;
    Battery& operator=(Battery&&) = default;
    Battery& operator=(const Battery&) = default;
    ~Battery() override = default;

    [[nodiscard]] bool init() override;

    /**
     * Tries to read a fixed amount of ADC measurements and calculates the average from it.
     * Then the value gets converted to mV.
     * If reading fails after a set amount of attempts std::nullopt will be returned.
     **/
    [[nodiscard]] std::optional<int> read_milli_volt() override;
};

} // namespace sensors
