#pragma once

#include <cstdint>

#define RTC_DATA_ATTR
inline constexpr int ESP_SLEEP_WAKEUP_TIMER = 4;

/** Simulates a cold boot for retained reporting policy tests. */
inline uint32_t esp_sleep_get_wakeup_causes() {
    return 0;
}
