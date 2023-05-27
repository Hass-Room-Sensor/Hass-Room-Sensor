cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "HassSensorMain.cpp"
                            "sensors/Scd41.hpp"
                            "sensors/Scd41.cpp"
                            "actuators/RgbLed.cpp"
                            "actuators/RgbLed.cpp"
                       INCLUDE_DIRS ".")
