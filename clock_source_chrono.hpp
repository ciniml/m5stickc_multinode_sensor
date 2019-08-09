#ifndef CLOCK_SOURCE_CHRONO_HPP__
#define CLOCK_SOURCE_CHRONO_HPP__

#include "ptp.hpp"
#include <chrono>

namespace PTP {
namespace ClockSource {

struct MonotonicClockSource {
    Timestamp now() {
        auto time_since_epoch = std::chrono::steady_clock::now().time_since_epoch();
        auto count = time_since_epoch.count();
        Timestamp timestamp;
        timestamp.seconds_field = count / 1000000000ul;
        timestamp.nanoseconds_field = count % 1000000000ul;
        return timestamp;
    }
};

}
};

#endif //CLOCK_SOURCE_CHRONO_HPP__