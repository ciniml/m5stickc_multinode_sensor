#ifndef CLOCK_SOURCE_HPP__
#define CLOCK_SOURCE_HPP__

#include <chrono>
#include <cstdint>

typedef std::chrono::microseconds Timestamp;
typedef std::chrono::microseconds TimeDifference;

struct IClockSource {
    virtual Timestamp now() = 0;    
};

struct MonotonicClockSource : public IClockSource {
    static MonotonicClockSource instance;
    virtual Timestamp now() override {
        return std::chrono::duration_cast<Timestamp>(std::chrono::steady_clock::now().time_since_epoch());
    }
};

struct AdjustableClockSource : public IClockSource {
    TimeDifference offset;

    AdjustableClockSource() : offset(TimeDifference::zero()) {}
    AdjustableClockSource(TimeDifference offset) : offset(offset) {}
    virtual Timestamp now() override {
        MonotonicClockSource source;
        return source.now() + this->offset;
    }
    void add_offset(TimeDifference offset) {
        this->offset += offset;
    }
};


#endif // CLOCK_SOURCE_HPP__