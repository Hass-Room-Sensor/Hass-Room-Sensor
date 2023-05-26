cmake_minimum_required(VERSION 3.16)

idf_component_register(SRCS "HassSensorMain.cpp"
                            "sensors/Scd41.hpp"
                            "sensors/Scd41.cpp"
                            "actuators/RgbLed.cpp"
                            "actuators/RgbLed.cpp"
                       INCLUDE_DIRS ".")
