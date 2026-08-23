cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(hass_sensor_srcs "HassSensorMain.cpp"
                     "app/RetainedState.hpp"
                     "app/RetainedState.cpp"
                     "defs/DeviceDefs.hpp"
                     "models/SensorModels.hpp"
                     "sensors/AbstractBme690.hpp"
                     "sensors/Bme690.hpp"
                     "sensors/Bme690.cpp"
                     "sensors/Bme690Mock.hpp"
                     "sensors/Bme690Mock.cpp"
                     "sensors/Scd41.hpp"
                     "sensors/Scd41.cpp"
                     "sensors/Scd41Mock.hpp"
                     "sensors/Scd41Mock.cpp"
                     "sensors/AbstractScd41.hpp"
                     "sensors/AbstractScd41.cpp"
                     "sensors/GpioInput.cpp"
                     "sensors/GpioInput.hpp"
                     "sensors/IBattery.hpp"
                     "sensors/Battery.hpp"
                     "sensors/Battery.cpp"
                     "devices/AbstractDeviceEventListener.hpp"
                     "devices/DeviceEventListenerFactory.hpp"
                     "devices/DeviceEventListenerFactory.cpp"
                     "devices/DebugRgbLedDevice.hpp"
                     "devices/DebugRgbLedDevice.cpp"
                     "devices/Esp32H2DevKitDevice.hpp"
                     "devices/Esp32H2DevKitDevice.cpp"
                     "devices/Esp32C6DevKitDevice.hpp"
                     "devices/Esp32C6DevKitDevice.cpp"
                     "devices/SeedStudioXiaoEspC6Device.hpp"
                     "devices/SeedStudioXiaoEspC6Device.cpp"
                     "actuators/RgbLed.hpp"
                     "actuators/RgbLed.cpp"
                     "actuators/Led.hpp"
                     "actuators/Led.cpp"
                     "zigbee/ZigbeeDeviceState.hpp"
                     "zigbee/ZDevice.hpp"
                     "zigbee/ZDevice.cpp")

set(hass_sensor_include_dirs ".")

if(NOT CMAKE_SCRIPT_MODE_FILE)
    include(FetchContent)

    cmake_policy(PUSH)
    cmake_policy(SET CMP0169 OLD)

    # Bosch BME690 SensorAPI
    FetchContent_Declare(
        bme690_sensorapi
        GIT_REPOSITORY https://github.com/boschsensortec/BME690_SensorAPI.git
        GIT_TAG v1.1.0
        GIT_PROGRESS TRUE
    )

    FetchContent_GetProperties(bme690_sensorapi)
    if(NOT bme690_sensorapi_POPULATED)
        FetchContent_Populate(bme690_sensorapi)
    endif()

    cmake_policy(POP)

    list(APPEND hass_sensor_srcs "${bme690_sensorapi_SOURCE_DIR}/bme69x.c")
    list(APPEND hass_sensor_include_dirs "${bme690_sensorapi_SOURCE_DIR}")
endif()
idf_component_register(SRCS ${hass_sensor_srcs}
                       INCLUDE_DIRS ${hass_sensor_include_dirs}
                       PRIV_REQUIRES driver esp_driver_i2c esp_driver_ledc nvs_flash app_update esp_adc)
