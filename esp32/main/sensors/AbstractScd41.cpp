#include "sensors/AbstractScd41.hpp"

namespace sensors {
bool AbstractScd41::try_recover() const {
    return perform_factory_reset() && reinit() && perform_self_test();
}
} // namespace sensors