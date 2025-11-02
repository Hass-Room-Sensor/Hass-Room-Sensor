#include "sensors/Scd41Mock.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace sensors {

bool Scd41Mock::init() const {
    return true;
}

std::optional<measurement_t> Scd41Mock::read_measurement() const {
    return measurement_t{42, 24.42, 42.24};
}

bool Scd41Mock::get_data_ready_status() const {
    return true;
}

bool Scd41Mock::start_periodic_measurement() const {
    return true;
}

bool Scd41Mock::stop_periodic_measurement() const {
    return true;
}

bool Scd41Mock::perform_factory_reset() const {
    return true;
}

bool Scd41Mock::reinit() const {
    return true;
}

bool Scd41Mock::perform_self_test() const {
    return true;
}

uint64_t Scd41Mock::get_serial_number() const {
    return 42;
}

void Scd41Mock::set_temperature_offset(double /*offset*/) const {}

double Scd41Mock::get_temperature_offset() const {
    return 0.42;
}

void Scd41Mock::set_sensor_altitude(uint16_t /*altitude*/) const {}

uint16_t Scd41Mock::get_sensor_altitude() const {
    return 500;
}

void Scd41Mock::persist_settings() const {}

bool Scd41Mock::probe_device() const {
    return true;
}
} // namespace sensors