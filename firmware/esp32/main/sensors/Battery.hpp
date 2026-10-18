#pragma once

#include "oneshot_adc.hpp"
#include "sensors/IBattery.hpp"
#include <optional>

namespace sensors {
class Battery : public IBattery {
  private:
    /** Single ADC channel wired to the battery divider. */
    std::vector<espp::AdcConfig> channels;
    /** Oneshot ADC helper that samples the battery divider during the current wake cycle. */
    espp::OneshotAdc adc;

    /**
     * Battery log tag.
     **/
    const char* TAG = "Battery";

  public:
    /**
     * Creates the ADC reader for the battery divider input.
     *
     * On the XIAO ESP32-C6 target the PCB routes `U_Bat_ADC` to XIAO pin `D2`, which maps to
     * ESP32-C6 `GPIO2 / ADC1_CH2`. ESP32-C6 exposes this channel on ADC1, so the driver must open
     * ADC unit 1 and sample channel 2 from that unit.
     */
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
