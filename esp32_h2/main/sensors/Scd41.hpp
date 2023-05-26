#pragma once

namespace sensors {
class Scd41 {
 public:
    Scd41() = default;
    Scd41(Scd41&&) = default;
    Scd41(const Scd41&) = default;
    Scd41& operator=(Scd41&&) = default;
    Scd41& operator=(const Scd41&) = default;
    ~Scd41() = default;

 private:
    void init();
};
}  // namespace sensors