#include "zigbee/StackLock.hpp"
#include <cassert>
#include <mutex>
#include <thread>

namespace {
std::mutex stackMutex;
unsigned calls{0};

/** Exercises destruction on an early return, as happens when queueing a report fails. */
void early_return() {
    const zigbee::StackLock lock;
    ++calls;
}
} // namespace

/** Simulates exclusive SDK ownership across application tasks. */
bool esp_zb_lock_acquire(uint32_t timeout) {
    assert(timeout == portMAX_DELAY);
    stackMutex.lock();
    return true;
}

/** Releases the simulated SDK ownership. */
void esp_zb_lock_release() {
    stackMutex.unlock();
}

/** Verifies early-return cleanup and serialized SDK calls from concurrent tasks. */
int main() {
    early_return();
    std::jthread first([] {
        for (unsigned index = 0; index < 1000; ++index) {
            early_return();
        }
    });
    std::jthread second([] {
        for (unsigned index = 0; index < 1000; ++index) {
            early_return();
        }
    });
    first.join();
    second.join();
    assert(calls == 2001);
}
