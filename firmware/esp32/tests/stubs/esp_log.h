#pragma once

/** Accepts firmware log arguments without producing output in host policy tests. */
template <class... Args>
inline void host_test_log(const char*, const char*, Args&&...) {}

#define ESP_LOGI(...) host_test_log(__VA_ARGS__)
