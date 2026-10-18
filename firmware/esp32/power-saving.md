# Power consumption and validation

The firmware is intended to keep the Zigbee session joined and use automatic ESP32-C6 light sleep between work. Environmental samples are scheduled every five minutes; the default Zigbee long-poll interval is three seconds so a healthy mesh can deliver an Identify request within five seconds. Deep sleep is reserved for bounded outage recovery because it tears down the joined Zigbee session.

## Current board and reporting behavior

The XIAO ESP32-C6 board has a BME690 and no SCD41. The board configuration disables SCD41 probing, while keeping the existing cluster list and Home Assistant entities. When a real SCD41 is configured and returns a valid sample, its temperature, humidity, and CO₂ take precedence; BME690 continues to supply pressure. If SCD41 is disabled, absent, or fails for a cycle, current-cycle BME690 temperature and humidity remain available and no CO₂ value is supplied. Firmware leaves the CO₂ attribute untouched and disables its automatic reports; a value retained in Home Assistant is the last received value, not a fresh measurement. A newly paired entity may show `unknown` until it receives a valid SCD41 value. The mock SCD41 is a test-only build choice.

Battery voltage is sampled at startup and hourly. Firmware reports the battery percentage at startup, after a change of at least two percentage points, and at a 24-hour heartbeat. OTA image discovery is intended to run daily. These schedules reduce ADC and radio work; they do not remove current drawn by the board's battery divider.

During connected idle and light sleep, the LEDs should be off. Setup and connection recovery use the board LEDs as status indicators. An Identify request blinks the status LED for the requested duration and then turns it off. A short BOOT press opens a maintenance window; a continuous 10-second BOOT hold requests a factory reset. See [maintenance and power profiles](docs/maintenance-and-power-profiles.md) for USB and reset behavior.

## Zigbee outage recovery

Normal operation keeps the sleepy Zigbee session joined, so Identify can be delivered during the three-second long-poll interval. If the node loses its parent or coordinator, it allows one automatic recovery attempt of up to 30 seconds while commissioned, or up to five minutes while factory new. An explicit BOOT maintenance window can keep it awake after that attempt. Otherwise it turns the LEDs off and deep-sleeps between retries at 1, 2, 4, 8, then 15 minutes, preserving Zigbee credentials and the retry step in RTC memory. A successful reconnection resets the backoff. Identify cannot be received during outage deep sleep; the five-second latency target applies only while the node is joined to a healthy mesh. This retry policy is compiled, but its low-current intervals and no-reset reconnection still need hardware validation.

## What the AVHzY measurements establish

The exploratory AVHzY CT-3 measurements were made with USB power. Earlier traces averaged 2.012 mA over six minutes and 1.885 mA over 60.5 seconds at about 5.161 V. They did not establish charging state or battery-side current. A 40-second joined-idle trace on 2026-10-18 averaged 2.450 mA (2.178–2.782 mA) at 5.161 V. Four stale buffered records near 35 mA were excluded because their timestamps preceded the meter timer reset; the remaining 41 one-second samples are retained at [ct3-joined-idle-20261018.csv](docs/measurements/ct3-joined-idle-20261018.csv). This short USB trace is not a matched firmware comparison and does not establish battery current or sleep residency.

On 2026-10-18, the XIAO C6 battery and development profiles and explicit SCD41 mock image compiled with ESP-IDF 6.1; the battery sdkconfig enables peripheral power-down, while the development sdkconfig keeps USB Serial/JTAG awake. ESP32-H2 and C6 development-kit targets also compiled in isolated build directories. The battery image has since been read back and hash-matched on the board, booted, and confirmed to publish BME690 readings through HASS. Its boot log also reports `ledc: create retention module failed, power domain can't turn off`; successful light-sleep residency and peripheral power-down therefore remain unverified.

Use the meter with the voltage field first, because `avhzy-c3.py` currently writes the values in voltage/current order even when the requested header order differs:

```sh
python ./avhzy-c3.py read -t 1000 -g voltage current
```

USB current is useful for repeatable firmware comparisons but includes the board's USB and charging path. A USB-host connection can also affect USB Serial/JTAG sleep behavior. Use the documented battery profile for measurements and disconnect the host when the test does not require it. Do not interpret USB input current as battery current or estimate battery life from it.

For a useful comparison, record at least ten minutes for both known-good and candidate firmware with the same supply, charge state, cable, profile, Zigbee parent, and sensor schedule. Include average and peak current, voltage, meter sampling interval, whether the initial maintenance window expired, and any observed sleep residency. Keep the five-minute BME690 sample interval in the record so measurement and radio peaks can be compared fairly.

After the device was re-added on 2026-10-18, Home Assistant reported temperature and humidity at 18:13 and again at 18:18; pressure last reported at 18:13 while its value stayed at 967 hPa. CO2 remained `unknown`, with its last report at the initial 18:08 pairing report. Later, after flashing the IDF 6.1 battery profile, Home Assistant reported BME690 temperature/humidity at 19:01, humidity again at 19:06, temperature at 19:11, and humidity at 19:16 (60.29% RH). Pressure stayed at 968 hPa and had not been reported since 18:56; CO2 remained `unknown` since 18:08. This confirms fresh changed BME690 values and suppression of missing-source CO2, but also shows that unchanged pressure is not refreshed within the intended 300-second maximum. Repeated temperature/humidity reports while the node remained joined show the sensor/radio path survives recurring idle intervals; direct light-sleep residency is still pending PM diagnostics. Recovery from an absent SCD41 to a valid reading cannot be tested on this board without installing one.

After the IDF 6.1 battery-profile app was flashed at `0x10000` and the board reset, the same seven Home Assistant entities remained available without re-pairing. At 19:01 local time on 2026-10-18, HASS reported 20.62 °C and 57.62% RH from the BME690; pressure was 968 hPa, last reported at 18:56. CO2 remained `unknown`, with `last_reported` still 18:08. These readings are plausible room conditions and confirm the physical BME690 path is active in the flashed profile. The unchanged battery state and voltage still reflect the earlier USB-powered report and are not validated battery measurements.

At 19:12:10 local time, an Identify press was accepted by Home Assistant. The user observed the LEDs blinking within five seconds and confirmed they returned to the idle-off state. This validates one successful response on the joined device, not the required 20 trials across different poll phases; command loss and worst-case latency remain uncharacterized.

The battery profile currently requests peripheral power-down during light sleep. Boot reports two LEDC retention-allocation warnings for the three board LEDs. IDF 6.1 source inspection suggests the first channel allocates the shared retention module and later channels log because they request the same module again; treat this as an explanation of the warnings, not proof that the power domain actually shuts down. An opt-in diagnostics build is available by combining `sdkconfig.profile.pm-diagnostics` with the normal battery profile. Its `CONFIG_PM_PROFILING` option enables lock-held timing and light-sleep counts/residency in `esp_pm_dump_locks()`, which now prints after startup and after each scheduled idle interval. The fragment selects USB Serial/JTAG as the primary console so the `stdout` diagnostics reach the attached ESP port. Profiling and the attached USB console add overhead, so use this build to establish sleep residency only, then return to the unprofiled battery image for power measurements.

## Remaining power limits and checks

The board's 8.2 kΩ and 10 kΩ battery divider draws continuously from the battery. At 3.7 V, its current is:

```text
3.7 V / (8.2 kΩ + 10 kΩ) ≈ 203 µA
```

Reducing ADC sampling frequency saves conversion energy but does not remove this roughly 203 µA hardware load. A future board revision could use a switched divider or higher resistor values with a capacitor near the ADC; higher values need physical ADC-accuracy validation.

The reported 100% battery state came from a USB-powered session and is not a calibrated battery measurement. Validate the ADC millivolts and divider conversion against a measured battery voltage before using the displayed percentage for runtime estimates. Actual battery capacity, charging state, and battery-side current remain unknown.
