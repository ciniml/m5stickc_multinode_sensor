#include "ptp.hpp"
#include "transport_udp.hpp"
#include "clock_source_chrono.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <errno.h>
#include <cstdio>

#if 0
struct Logger {
    template <typename... Args> static void info(const char* fmt, Args... args) {
        std::fprintf(stderr, fmt, args... );
    }
    template <typename... Args> static void error(const char* fmt, Args... args) {
        std::fprintf(stderr, fmt, args... );
    }
};
#else 
using Logger = PTP::NullLogger;
#endif

typedef PTP::ClockSource::AdjustableClockSource ClockSource;
typedef PTP::Transport::UDPTransport Transport;
typedef PTP::SlaveClock<Transport, ClockSource, Logger> SlaveClock;

int main(int argc, char* argv[]) {
    ClockSource clock_source(PTP::TimeDiff(-10000000));
    Transport transport(10000);
    SlaveClock slave(transport, clock_source);

    transport.bind(20000);

    while(true) {
        if( slave.update() ) {
            auto now = std::chrono::steady_clock::now().time_since_epoch();
            auto now_adjusted = std::chrono::nanoseconds(static_cast<uint64_t>(clock_source.now().raw));
            auto diff = now - now_adjusted;
            std::cout << "Diff: " << diff.count() << std::endl;
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } 
    return 0;
}