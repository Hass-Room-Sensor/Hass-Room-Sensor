#pragma once

#include "esp_zigbee_core.h"
#include <cstdlib>

namespace zigbee {
/** Holds the SDK lock for calls made outside Zigbee callbacks; never hold it while waiting for callbacks. */
class StackLock {
  public:
    /** Waits until this task owns the Zigbee stack lock. */
    StackLock() {
        if (!esp_zb_lock_acquire(portMAX_DELAY)) {
            std::abort();
        }
    }
    StackLock(const StackLock&) = delete;
    StackLock& operator=(const StackLock&) = delete;
    StackLock(StackLock&&) = delete;
    StackLock& operator=(StackLock&&) = delete;
    /** Releases the stack lock on every scope exit. */
    ~StackLock() {
        esp_zb_lock_release();
    }
};
} // namespace zigbee
