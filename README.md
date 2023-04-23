# HASS_EnviromentSensor
Home Assistant (ESPHome) Enviroment Sensor

## Hardware Platform

### ESP32-H2
Most recent 32-bit RISC-V chip from Espressif Systems: https://www.espressif.com/en/products/socs/esp32-h2

Lead time is quite high for a dev-board right now. They will arrive roughly in August (https://www.mouser.de/c/?q=ESP32-H2).

### Features
* IEEE 802.15.4
* BT/BLE 5.0
* Thread
* Zigbee

## Software Platform
There are multiple ways we are able to realize this project from the software side.

### ESP32 Matter
[Espressif Systems](https://www.espressif.com/) is involved in the development of the Matter specification.
This results in them providing a bunch of examples: https://github.com/project-chip/connectedhomeip/tree/master/examples

They are written in C/C++ and build on top of the [ESP-IDF](https://github.com/espressif/esp-idf) which is more work to set up. Also the examples require a **large** toolset with a lot of different requirements for them to build successfully.
**But** once the setup is done, using it is absolutely straight forward!

The device then can be integrated either via [Zigbee](https://www.home-assistant.io/integrations/zha/) or [Thread](https://www.home-assistant.io/integrations/thread/) in HASS.

#### Advantages
* Low power and direct control
* We decide if we want [Zigbee](https://www.home-assistant.io/integrations/zha/), [Thread](https://www.home-assistant.io/integrations/thread/) (or REST) as interface.

#### Disadvantages
* Complex Setup

### ESPHome
No Matter support yet ([tracking issue](https://github.com/esphome/feature-requests/issues/1430)).

#### Advantages
* Easy setup

#### Disadvantages
* Potentially high power consumption?
* No Matter support yet

### Tasmota
Alternative to ESPHome Tasmota exists. Right now they already have basic Matter support ([tracking issue](https://github.com/arendst/Tasmota/discussions/17872))

#### Advantages
* Easy setup

#### Disadvantages
* Potentially high power consumption?
* No "official" Matter support yet
