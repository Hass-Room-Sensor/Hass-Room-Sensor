# ESP32 Code

This directory contains all code for reading and publishing sensor values to [Home Assistant](https://www.home-assistant.io/).

The goal is to support pushing data to Home Assistant via multiple different interfaces.

| Interface       | Status    | Notes                           |
|-----------------|-----------|---------------------------------|
| WIFI (MQTT)     | TODO      |                                 |
| REST            | TODO      |                                 |
| ZigBee          | Supported | Temperature, Humidity, CO2, OTA |
| Thread (Matter) | TODO      |                                 |

Besides multiple protocols, multiple devices are supported.

| Device                    | Link |
|---------------------------|------|
| seed studio XIAO ESPC6    | https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/ |
| Official ESP32 H2 Dev Kit | https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32h2/index.html |
| Official ESP32 C6 Dev Kit | https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/index.html |

## Device Specifics

### seed studio XIAO ESPC6

![LED behavior](docs/seed_studio_xiao_esp32_c6_sensor_board.svg)

### Official ESP32 H2 Dev Kit

#### Pinout

| GPIO | Action |
| ---- | ------ |
| 12   | SCD41 SDA |
| 22   | SCD41 SCL |
| 1    | Zigbee factory reset if high |
| 3    | ZigBee device power mode. low - DC, high - battery |

### Official ESP32 C6 Dev Kit

#### Pinout

| GPIO | Action |
| ---- | ------ |
| 12   | SCD41 SDA |
| 22   | SCD41 SCL |
| 1    | Zigbee factory reset if high |
| 3    | ZigBee device power mode. low - DC, high - battery |

## Building

### ESP-IDF

For building the [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/get-started/linux-macos-setup.html#get-started-prerequisites) version `5.5.1` is required.
Follow the following guide to install the standard toolchain: https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/get-started/linux-macos-setup.html#get-started-prerequisites

### Terminal

```bash
# Ensure you are inside this directory
cd firmware/esp32/

# Select a debug or release build
export IDF_SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.debug;sdkconfig.device"
# export IDF_SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.release"

# Select the underlying EPS hardware.
# For the 'seed studio XIAO ESPC6' or 'Official ESP32 C6 Dev Kit':
export ESP_HARDWARE=esp32c6
# For the 'Official ESP32 H2 Dev Kit':
# export ESP_HARDWARE=esp32h6

# Select which device you actually want to compile for
cat > sdkconfig.device <<'EOF'
CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_SEED_STUDIO_XIAO_ESPC6=y
CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_H2_DEV_KIT=n
CONFIG_HASS_ENVIRONMENT_SENSOR_DEVICE_TARGET_ESP32_C6_DEV_KIT=n
EOF

# Ensure defaults are applied fresh (avoid reusing a previous sdkconfig)
rm -f sdkconfig sdkconfig.old

idf.py set-target $ESP_HARDWARE
idf.py build

# Flash (change the ttyUSB0 according to where you plug in your ESP)
idf.py -p /dev/ttyUSB0 flash
```

### Visual Studio Code

Once the ESP-IDF is installed open this directory with Visual Studio Code.
By default, there are already tasks configured for building, flashing, and monitoring.
Execute them and you are ready to go!

## Consuming Prebuilt OTA Images

We provide a set of OTA images and update mirrors for all the supported the devices above.

### HomeAssistant Configuration

To tell Home Assistant to look for additional update sources for your ZigBee devices add the following to the `/config/configuration.yaml`.

```yaml
# ZigBee OTA docs
# Docs: https://www.home-assistant.io/integrations/zha/#ota-firmware-updates
# https://github.com/zigpy/zigpy/wiki/OTA-Configuration
zha:
  zigpy_config:
    ota:
      extra_providers:
        - type: zigpy_remote # The `zigpy_remote` provider type requires a URL that points to an OTA index file
          # This will give you a stable feed of updates. For unstable but more frequent updates replace 'main' with 'develop'.
          url: https://raw.githubusercontent.com/Hass-Room-Sensor/Hass-Room-Sensor/refs/heads/main/firmware/esp32/ota/index.json
      otau_directory: /config/www/ota
```

Now do a **full** restart of HomeAssistant. A config reload is not enough. Else ZHA might cache OTA properties.

### Triggering An Update

You can manually inform your device a new OTA update might be available. For this follow this guide: https://github.com/zigpy/zigpy/wiki/OTA-Information-for-Manufacturers#testing

In case it was successful and the new firmware version is actually higher then the previous one, your HomeAssistant should show a new update being available for your device. Trigger the update and wait a long time. This is a slow process.

## Building Your Own OTA Images

This guide details how to manually build an Over The Air (OTA) update for use within Home Assistant.

### 1. Build The Binary

```bash
# Clone the repository and change into it
git clone https://github.com/Hass-Room-Sensor/Hass-Room-Sensor.git
cd Hass-Room-Sensor/firmware/esp32

# Follow the steps from above for building for the correct device.
# Once done, continue here. And check if the output binary exists.
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
# Replace 'https://hass.example.com/local/ota' with where you are going to upload your OTA binary to.
# Usually it's '/config/www/ota' which would map to the path above.
zigpy ota generate-index --ota-url-root="https://hass.example.com/local/ota" hass_sensor.ota > ota.json

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
          url: https://hass.example.com/local/ota/index.json
      otau_directory: /config/www/ota
```

Now do a **full** restart of HomeAssistant. A config reload is not enough. Else ZHA might cache OTA properties.

### 6. Triggering An Update

You can manually inform your device a new OTA update might be available. For this follow this guide: https://github.com/zigpy/zigpy/wiki/OTA-Information-for-Manufacturers#testing

In case it was successful and the new firmware version is actually higher then the previous one, your HomeAssistant should show a new update being available for your device. Trigger the update and wait a long time. This is a slow process.
