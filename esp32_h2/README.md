# ESP32 H2 Code

This directory contains all code for reading the [SCD41 CO2 Sensor](https://sensirion.com/products/catalog/SCD41/) and pushing the data to [Home Assistant](https://www.home-assistant.io/).

The goal is to support pushing data to Home Assistant via multiple different interfaces.

| Interface       | Status | Notes |
|-----------------|--------|-------|
| MQTT            | TODO   |       |
| REST            | TODO   |       |
| ZigBee          | Supported | Temperature and Humidity |
| Thread (Matter) | TODO   |       |

## ESP32 Pinout

| GPIO | Action |
| ---- | ------ |
| 12   | SCD41 SDA |
| 22   | SCD41 SCL |
| 1    | Zigbee factory reset if high |
| 3    | ZigBee device power mode. low - DC, high - battery |

## Building

### ESP-IDF

For building the [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/get-started/linux-macos-setup.html#get-started-prerequisites) is required.
Follow the following guide to install the standard toolchain: https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/get-started/linux-macos-setup.html#get-started-prerequisites

### Visual Studio Code

Once the ESP-IDF is installed open this directory with Visual Studio Code.
By default, there are already tasks configured for building, flashing, and monitoring.
Execute them and you are ready to go!

## Misc

### ESP32-H2-DevKitM-1

Documentation: https://espressif-docs.readthedocs-hosted.com/projects/esp-dev-kits/en/latest/esp32h2/esp32-h2-devkitm-1/user_guide.html
