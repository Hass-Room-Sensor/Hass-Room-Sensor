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

For building the [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/get-started/linux-macos-setup.html#get-started-prerequisites) version `5.1` is required.
Follow the following guide to install the standard toolchain: https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/get-started/linux-macos-setup.html#get-started-prerequisites

### Visual Studio Code

Once the ESP-IDF is installed open this directory with Visual Studio Code.
By default, there are already tasks configured for building, flashing, and monitoring.
Execute them and you are ready to go!

## Misc

### ESP32-H2-DevKitM-1

Documentation: https://espressif-docs.readthedocs-hosted.com/projects/esp-dev-kits/en/latest/esp32h2/esp32-h2-devkitm-1/user_guide.html

```bash
# Build
cd esp32_h2/
idf.py set-target esp32h2
idf.py build

# Flash (change the ttyUSB0 according to where you plug in your ESP)
idf.py -p /dev/ttyUSB0 flash
```

### ESP32-C6-mini-1

The following commands show how to build and flash via JTAG.

```bash
# Build
cd esp32_h2/
idf.py set-target esp32c6
idf.py build

# Flash
openocd -f board/esp32c6-builtin.cfg -c "program_esp build/bootloader/bootloader.bin 0x0 verify exit"
openocd -f board/esp32c6-builtin.cfg -c "program_esp build/partition_table/partition-table.bin 0x8000 verify exit"
openocd -f board/esp32c6-builtin.cfg -c "program_esp build/hass_sensor.bin 0x10000 verify exit"
```