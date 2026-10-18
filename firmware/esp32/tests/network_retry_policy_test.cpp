#include "app/NetworkRetryPolicy.hpp"

#include <cassert>
#include <chrono>

using namespace std::chrono_literals;

/** Exercises the network retry policy behavior. */
int main() {
    using app::NetworkRetryPolicy;
    assert(NetworkRetryPolicy::factory_new_attempt == 5min);
    assert(NetworkRetryPolicy::commissioned_attempt == 30s);
    assert(NetworkRetryPolicy::attempt_timeout(true) == 5min);
    assert(NetworkRetryPolicy::attempt_timeout(false) == 30s);
    assert(NetworkRetryPolicy::retry_delay(0) == 1min);
    assert(NetworkRetryPolicy::retry_delay(1) == 2min);
    assert(NetworkRetryPolicy::retry_delay(2) == 4min);
    assert(NetworkRetryPolicy::retry_delay(3) == 8min);
    assert(NetworkRetryPolicy::retry_delay(4) == 15min);
    assert(NetworkRetryPolicy::retry_delay(100) == 15min);
}
