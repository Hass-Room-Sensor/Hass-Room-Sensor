# ESP32 H2 Code

This directory contains all code for reading the [SCD41 CO2 Sensor](https://sensirion.com/products/catalog/SCD41/) and pushing the data to [Home Assistant](https://www.home-assistant.io/).

The goal is to support pushing data to Home Assistant via multiple different interfaces.

| Interface       | Status | Notes |
|-----------------|--------|-------|
| MQTT            | TODO   |       |
| REST            | TODO   |       |
| ZigBee          | Supported | Temperature, Humidity, CO2, OTA |
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

# Select a debug or release build
export IDF_SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.debug"
# export IDF_SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.release"

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

# Select a debug or release build
export IDF_SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.debug"
# export IDF_SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.release"

idf.py set-target esp32c6
idf.py build

# Flash
openocd -f board/esp32c6-builtin.cfg -c "program_esp build/bootloader/bootloader.bin 0x0 verify exit"
openocd -f board/esp32c6-builtin.cfg -c "program_esp build/partition_table/partition-table.bin 0x8000 verify exit"
openocd -f board/esp32c6-builtin.cfg -c "program_esp build/hass_sensor.bin 0x10000 verify exit"
```

## OTA

This guide details how to build an Over The Air (OTA) update for use within HomeAssistant.

### 1. Build The Binary

```bash
# Clone the repository and change into it
git clone https://github.com/electronics4fun/HASS_EnviromentSensor.git
cd HASS_EnviromentSensor/esp32_h2

# Select a debug or release build
export IDF_SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.debug"
# export IDF_SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.release"

# Build
idf.py build

# See the output
ls -la build/hass_sensor.bin
```

### 2. Install Dependencies

Converting a binary to a valid ZigBee OTA update file requires a few further steps and tools.

```bash
# Install python image builder dependencies
pip install zigpy-cli zigpy

# Get a copy of the tool that takes care of converting your binary to an OTA binary
wget https://raw.githubusercontent.com/espressif/esp-zigbee-sdk/refs/heads/main/tools/image_builder_tool/image_builder_tool.py
```

### 3. Convert To OTA

Not that we have all dependencies, we can convert our binary to an OTA update file.
More about this process is available here: https://docs.espressif.com/projects/esp-zigbee-sdk/en/latest/esp32/user-guide/zcl_ota_upgrade.html#generate-an-ota-upgrade-image-for-the-esp-platform

```bash
# Prepare the output directory
mkdir ota && cd ota
# Copy the build binary
cp ../build/hass_sensor.bin hass_sensor.ota

# Build the OTA image
# Replace the value of the following properties accordingly:
# '--version 0x2' replace with the actual of your build image. Needs to be higher than the currently installed one on your ESP32.
python ../image_builder_tool.py --create hass_sensor.ota --manuf-id 0x0 --image-type 0x0 --version 0x2 --tag-file hass_sensor.ota
```

### 4. Prepare for HomeAssistant

The next step of making it usable is to create the required metadata for HomeAssistant.
More about this is available here: https://github.com/zigpy/zigpy/wiki/OTA-Information-for-Manufacturers#creation

```bash
# Replace 'https://hass.uwpx.org/local/ota' with where you are going to upload your OTA binary to.
# Usually it's '/config/www/ota' which would map to the path above.
zigpy ota generate-index --ota-url-root="https://hass.uwpx.org/local/ota" hass_sensor.ota > ota.json

# Since the output is not perfect or directly usable by HomeAssistant, we need to modify the JSON a bit.
file_size=$(stat -c %s "hass_sensor.ota")
# Add the missing 'file_size' property and wrap it into 'firmwares'.
cat ota.json | jq --argjson size "$file_size" '{firmwares: [ .[] | .file_size = $size ]}' > index.json
```

### 5. HomeAssistant Configuration

Take the `index.json` and `hass_sensor.ota` and place them in the following locations in HomeAssistant.
I recommend using the [Visual Studio Code Server](https://github.com/hassio-addons/addon-vscode) for that.

* `hass_sensor.ota` -> `/config/www/ota/hass_sensor.ota`
* `index.json` -> `/config/www/ota/index.json`

As a next step we have to tell HomeAssistant to load this update data source for our devices.
For this open `/config/configuration.yaml` and add the following:

```yaml
# ZigBee OTA docs
# Docs: https://www.home-assistant.io/integrations/zha/#ota-firmware-updates
# https://github.com/zigpy/zigpy/wiki/OTA-Configuration
zha:
  zigpy_config:
    ota:
      extra_providers:
        - type: zigpy_remote # The `zigpy_remote` provider type requires a URL that points to an OTA index file
          url: https://hass.uwpx.org/local/ota/index.json
      otau_directory: /config/www/ota
```

Now do a **full** restart of HomeAssistant. A config reload is not enough. Else ZHA might cache OTA properties.

### 6. Triggering A Update

You can manually inform your device a new OTA update might be available. For this follow this guide: https://github.com/zigpy/zigpy/wiki/OTA-Information-for-Manufacturers#testing

In case it was successful and the new firmware version is actually higher then the previous one, your HomeAssistant should show a new update being available for your device. Trigger the update and wait a long time. This is a slow process.
