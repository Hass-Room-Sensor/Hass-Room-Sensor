# HASS Environment Sensor
Home Assistant (ESPHome) Environment Sensor

## Docs
* [Datasheet ESP32-C6-MINI-1](https://www.espressif.com/sites/default/files/documentation/esp32-c6-mini-1_mini-1u_datasheet_en.pdf)
* [ESP-Prog JTAG Pinout](https://www.circuitstate.com/pinouts/espressif-esp-prog-esp32-jtag-debug-probe-pinout-diagram/)

## JTAG Connection

1. Setup the `esp_idf`: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html#get-started-set-up-tools
2. Run `/esp/esp-idf/export.sh` if you installed the `esp_idf` inside the default location.
3. Now `openocd --version` should print `[...]v0.12.0-esp32-20230419[...]`. The `esp32` part is the important one here. If it does not contain `esp32` inside the version string, then you most likely still have installed another version of `openocd`. Uninstall the other version and try again.
4. Install the `udev` rules on Linux for JTAG debugging: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/jtag-debugging/configure-other-jtag.html 

### USB

5. Connect the JTAG debugger. If connected successfully, the green light will turn on the board.
6. Run `openocd -f board/esp32c6-builtin.cfg` to connect to it. There are a bunch of other EPS32 devices supported. You can get a list by running `ls -l $OPENOCD_SCRIPTS/board | grep esp32`. We use the builtin `D+` and `D-` contacts for it.

### 8-Pin

5. Connect the JTAG debugger. If connected successfully, the green light will turn on the board.
6. Run `openocd -f interface/ftdi/esp32_devkitj_v1.cfg -f board/esp32c6-builtin.cfg` to connect to it. There are a bunch of other EPS32 devices supported. You can get a list by running `ls -l $OPENOCD_SCRIPTS/board | grep esp32`. We use the `ftdi` variant since our JTAG debugger has an FTDI on it.
7. Fails to connect to ESP32. `TODO FIX`


## Flashing and Building

For this please take a look at [`esp32_h2/README.md`](esp32_h2/README.md).