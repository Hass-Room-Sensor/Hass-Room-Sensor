#pragma once

#include <memory>

namespace devices {
class AbstractDeviceEventListener;

std::shared_ptr<AbstractDeviceEventListener> create_device_event_listener();
} // namespace devices
