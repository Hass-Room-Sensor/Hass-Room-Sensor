#include "sensors/Bme690Mock.hpp"

namespace sensors {
bool Bme690Mock::init() {
    return true;
}

bool Bme690Mock::probe_device() {
    return true;
}

std::optional<bme690_measurement_t> Bme690Mock::read_measurement() {
    return std::make_optional<bme690_measurement_t>(bme690_measurement_t{
            .temp_celsius = 23.5,
            .humidity_percent = 48.0,
            .pressure_hpa = 1013.2,
    });
}
} // namespace sensors
