#ifndef TRANSPORT_UDP_HPP__
#define TRANSPORT_UDP_HPP__

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>

#include <cstdint>
#include <cstdlib>

namespace PTP {
namespace Transport {

class UDPTransport {
private:
    int fd;

public:
    struct AddressType {
        struct sockaddr_in address;
        AddressType() = default;
        AddressType(const struct sockaddr_in& address) : address(address) {}
        bool operator==(const AddressType& rhs) {
            return this->address.sin_addr.s_addr == rhs.address.sin_addr.s_addr && this->address.sin_port == rhs.address.sin_port;
        }
        operator struct sockaddr_in() const {
            return this->address;
        }
    };
    UDPTransport() : fd(0) {
        this->fd = socket(AF_INET, SOCK_DGRAM, 0);
        int dummy;
        ioctl(this->fd, FIONBIO, &dummy);
    }

    int send_packet(const AddressType& address, const void* data, std::size_t length) {
        return sendto(this->fd, data, length, 0, reinterpret_cast<const struct sockaddr*>(&address), sizeof(address));
    }
    int receive_packet(AddressType& address, void* data, std::size_t length) {
        socklen_t address_len = sizeof(address);
        return recvfrom(this->fd, data, length, 0, reinterpret_cast<struct sockaddr*>(&address), &address_len);
    }
};

};
};

#endif //TRANSPORT_UDP_HPP__