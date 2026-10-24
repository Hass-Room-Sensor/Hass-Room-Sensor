# Maintenance and power profiles

The XIAO ESP32-C6 uses GPIO9 for BOOT. While the application is awake, a debounced short press opens a five-minute maintenance window. A continuous ten-second hold performs a factory reset. A BOOT input already held as the application starts is ignored until it is released. A cold boot or physical RESET opens a 60-second window; an automatic timer wake skips it.

The battery profile uses automatic light sleep and peripheral power-down. GPIO9 cannot wake this profile after peripheral power-down, so use RESET for a temporary awake window. For firmware recovery, hold BOOT, press and release RESET, then release BOOT to enter the ESP32-C6 ROM downloader. ROM download does not run the application reset gesture and does not erase Zigbee credentials. Use the stable `/dev/serial/by-id/` path for flashing.

Build profiles are composed after the board fragment:

```sh
export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.debug;sdkconfig.device;sdkconfig.profile.development"
```

The development profile keeps USB Serial/JTAG awake while a host is connected. That makes flashing and console access reliable and changes measured current. For battery-profile measurements, use the optimized build and battery fragment:

```sh
export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.release;sdkconfig.device;sdkconfig.profile.battery"
```

The battery profile permits normal USB sleep behavior and enables supported C6 peripheral power-down. USB power measurements still include the board's charging path; they are not battery-current measurements. Compare old and new firmware with the same supply, charging state, cable, USB profile, Zigbee parent, and measurement cadence.

The current board has no SCD41 fitted. SCD41 is disabled by the board configuration, so BME690 supplies temperature, humidity, and pressure. The CO2 cluster remains present for compatibility, but its value is not updated or refreshed without a valid SCD41 sample. Home Assistant can therefore retain the last value it received; that retained value is not a current measurement. Mock SCD41 readings are available only through an explicit test build configuration.

## Identify acceptance

On a healthy joined mesh, the normal idle poll interval is three seconds and Identify should start within five seconds. Test 20 requests at varied points in the poll interval and record each successful visible start time. Record delivery losses separately; an unavailable parent or coordinator is outside this latency target. Parent aging and command retention are separate from the end device's poll interval.

## Current power evidence

The exploratory AVHzY C3 traces were USB-powered and averaged 2.012 mA over six minutes and 1.885 mA over 60.5 seconds. They did not establish battery-side current, charging state, or confirmed light-sleep residency. The board's 8.2 kΩ / 10 kΩ battery divider draws about 203 µA continuously at 3.7 V; reducing ADC sampling frequency does not remove that hardware current.
