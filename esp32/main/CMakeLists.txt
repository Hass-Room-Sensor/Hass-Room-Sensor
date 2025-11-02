cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "HassSensorMain.cpp"
                            "defs/DeviceDefs.hpp"
                            "sensors/Scd41.hpp"
                            "sensors/Scd41.cpp"
                            "sensors/Scd41Mock.hpp"
                            "sensors/Scd41Mock.cpp"
                            "sensors/AbstractScd41.hpp"
                            "sensors/AbstractScd41.cpp"
                            "sensors/GpioInput.cpp"
                            "sensors/GpioInput.hpp"
                            "actuators/RgbLed.hpp"
                            "actuators/RgbLed.cpp"
                            "actuators/Led.hpp"
                            "actuators/Led.cpp"
                            "zigbee/ZDevice.hpp"
                            "zigbee/ZDevice.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver nvs_flash app_update)
