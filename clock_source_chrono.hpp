#ifndef CLOCK_SOURCE_CHRONO_HPP__
#define CLOCK_SOURCE_CHRONO_HPP__

#include "ptp.hpp"
#include <chrono>

namespace PTP {
namespace ClockSource {

struct MonotonicClockSource {
    Timestamp128 now() {
        auto time_since_epoch = std::chrono::steady_clock::now().time_since_epoch();
        auto count = time_since_epoch.count();
        return static_cast<Timestamp128>(count);
    }
};

struct AdjustableClockSource {
    TimeDiff offset;

    AdjustableClockSource() : offset(0) {}
    AdjustableClockSource(const TimeDiff& offset) : offset(offset) {}
    Timestamp128 now() {
        MonotonicClockSource source;
        return source.now() + this->offset;
    }
    void add_offset(const TimeDiff& offset) {
        this->offset = this->offset + offset;
    }
};

}; // namespace ClockSource
}; // namespace PTP

#endif //CLOCK_SOURCE_CHRONO_HPP__