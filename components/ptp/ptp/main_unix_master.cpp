#include "ptp.hpp"
#include "transport_udp.hpp"
#include "clock_source_chrono.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <thread>
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

typedef PTP::ClockSource::MonotonicClockSource ClockSource;
typedef PTP::Transport::UDPTransport Transport;
typedef PTP::MasterClock<Transport, ClockSource, 1, Logger> MasterClock;

int main(int argc, char* argv[]) {
    ClockSource clock_source;
    Transport transport(20000);
    MasterClock master(transport, clock_source);

    transport.bind(10000);

    struct sockaddr_in slave_address;
    slave_address.sin_family = AF_INET;
    slave_address.sin_port = htons(20000);
    inet_aton("127.0.0.1", &slave_address.sin_addr);

    master.add_slave(slave_address);

    while(true) {
        master.update();
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } 
    return 0;
}