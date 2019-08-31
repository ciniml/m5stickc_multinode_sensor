#ifndef CLOCK_SOURCE_HPP__
#define CLOCK_SOURCE_HPP__

#include <chrono>
#include <cstdint>

typedef std::uint64_t Timestamp;
typedef std::int64_t TimeDiff;


struct MonotonicClockSource {
    Timestamp now() {
        auto time_since_epoch = std::chrono::steady_clock::now().time_since_epoch();
        auto count = time_since_epoch.count();
        return static_cast<Timestamp>(count);
    }
};

struct AdjustableClockSource {
    TimeDiff offset;

    AdjustableClockSource() : offset(0) {}
    AdjustableClockSource(const TimeDiff& offset) : offset(offset) {}
    Timestamp now() {
        MonotonicClockSource source;
        return source.now() + this->offset;
    }
    void add_offset(const TimeDiff& offset) {
        this->offset = this->offset + offset;
    }
};

#endif // CLOCK_SOURCE_HPP__