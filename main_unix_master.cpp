#include "ptp.hpp"
#include "transport_udp.hpp"
#include "clock_source_chrono.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <thread>

typedef PTP::ClockSource::MonotonicClockSource ClockSource;
typedef PTP::Transport::UDPTransport Transport;
typedef PTP::MasterClock<Transport, ClockSource, 1> MasterClock;

int main(int argc, char* argv[]) {
    ClockSource clock_source;
    Transport transport;
    MasterClock master(transport, clock_source);

    struct sockaddr_in slave_address;
    slave_address.sin_family = AF_INET;
    slave_address.sin_port = htons(2000);
    inet_aton("127.0.0.1", &slave_address.sin_addr);

    master.add_slave(slave_address);

    while(true) {
        master.update();
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    return 0;
}