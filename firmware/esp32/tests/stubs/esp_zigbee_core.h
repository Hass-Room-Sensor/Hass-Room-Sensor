#pragma once

#include <cstdint>
#include <limits>

/** Minimal SDK lock declarations used only by the host RAII regression test. */
inline constexpr uint32_t portMAX_DELAY = std::numeric_limits<uint32_t>::max();
bool esp_zb_lock_acquire(uint32_t timeout);
void esp_zb_lock_release();
