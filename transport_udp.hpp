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
    int send_port;

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

    UDPTransport(int send_port) : fd(0), send_port(send_port) {
        this->fd = socket(AF_INET, SOCK_DGRAM, 0);
        int dummy;
        ioctl(this->fd, FIONBIO, &dummy);
    }

    void bind(int port) {
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;
        ::bind(this->fd, (struct sockaddr *)&addr, (::socklen_t)sizeof(addr));
    }

    int send_packet(const AddressType& address, const void* data, std::size_t length) {
        AddressType address_ = address;
        address_.address.sin_port = htons(this->send_port);
        int rc = sendto(this->fd, data, length, 0, reinterpret_cast<const struct sockaddr*>(&address_), sizeof(address_));
        if( rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) ) {
            return 0;
        }
        else {
            return rc;
        }
    }
    int receive_packet(AddressType& address, void* data, std::size_t length) {
        socklen_t address_len = sizeof(address);
        int rc = recvfrom(this->fd, data, length, 0, reinterpret_cast<struct sockaddr*>(&address), &address_len);
        if( rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) ) {
            return 0;
        }
        else {
            return rc;
        }
    }
};

};
};

#endif //TRANSPORT_UDP_HPP__