#pragma once

#include "bme69x.h"
#include "driver/i2c_master.h"
#include "sensors/AbstractBme690.hpp"
#include "soc/gpio_num.h"

#include <chrono>

namespace sensors {
/**
 * Bosch BME690 wrapper using the official Bosch SensorAPI.
 *
 * Datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme690-ds001.pdf
 *
 * Sensor API:
 * https://github.com/boschsensortec/BME690_SensorAPI
 */
class Bme690 : public AbstractBme690 {
  public:
    /** Creates the BME690 wrapper on the given I2C pins. */
    Bme690(gpio_num_t sda, gpio_num_t scl);
    Bme690(Bme690&&) = default;
    Bme690(const Bme690&) = delete;
    Bme690& operator=(Bme690&&) = default;
    Bme690& operator=(const Bme690&) = delete;
    ~Bme690() override;

    /** Initializes the Bosch SensorAPI and configures forced-mode measurements. */
    [[nodiscard]] bool init() override;
    /** Probes both supported BME690 I2C addresses and attaches the first device that answers. */
    [[nodiscard]] bool probe_device() override;
    /** Performs one forced-mode measurement. */
    [[nodiscard]] std::optional<bme690_measurement_t> read_measurement() override;

  private:
    struct InterfaceContext {
        /** Handle used by the Bosch SensorAPI register read/write callbacks. */
        i2c_master_dev_handle_t dev{};
    };

    static constexpr uint8_t PRIMARY_ADDRESS = BME69X_I2C_ADDR_LOW;
    static constexpr uint8_t SECONDARY_ADDRESS = BME69X_I2C_ADDR_HIGH;
    static constexpr std::chrono::milliseconds BUS_TIMEOUT{250};
    /** Extra delay added after the Bosch-calculated measurement time. */
    static constexpr std::chrono::microseconds MEASUREMENT_GUARD_TIME{1000};

    static const char* TAG;

    /** I2C SDA pin used by the sensor bus. */
    gpio_num_t sda_;
    /** I2C SCL pin used by the sensor bus. */
    gpio_num_t scl_;
    /** Currently attached I2C address. */
    uint8_t deviceAddress_{PRIMARY_ADDRESS};

    /** IDF I2C bus configuration. */
    i2c_master_bus_config_t busConfig_{};
    /** IDF I2C bus handle. */
    i2c_master_bus_handle_t bus_{};
    /** IDF I2C device configuration. */
    i2c_device_config_t deviceConfig_{};
    /** IDF I2C device handle for the active BME690. */
    i2c_master_dev_handle_t device_{};
    /** Bosch callback context that wraps the IDF device handle. */
    InterfaceContext interfaceContext_{};

    /** Bosch SensorAPI device descriptor. */
    bme69x_dev sensor_{};
    /** Environmental oversampling/filter configuration. */
    bme69x_conf sensorConfig_{};
    /** Gas-heater configuration, left disabled for this project. */
    bme69x_heatr_conf heaterConfig_{};
    /** True once the sensor has been successfully initialized. */
    bool initialized_{false};

    /** Creates the shared I2C master bus. */
    void create_bus();
    /** Releases the I2C device and bus handles. */
    void destroy_bus() noexcept;
    /** Attaches the Bosch interface to a specific I2C address. */
    [[nodiscard]] bool attach_device(uint8_t deviceAddress);

    /** Bosch SensorAPI register-read callback. */
    [[nodiscard]] static BME69X_INTF_RET_TYPE read_register(uint8_t regAddress, uint8_t* registerData, uint32_t length, void* interfacePointer);
    /** Bosch SensorAPI register-write callback. */
    [[nodiscard]] static BME69X_INTF_RET_TYPE write_register(uint8_t regAddress, const uint8_t* registerData, uint32_t length, void* interfacePointer);
    /** Bosch SensorAPI delay callback. */
    static void delay_microseconds(uint32_t period, void* interfacePointer);
};
} // namespace sensors
