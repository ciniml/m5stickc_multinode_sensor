#ifndef SENSOR_COMMUNICATION_HPP__
#define SENSOR_COMMUNICATION_HPP__

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <memory>
#include <vector>
#include <chrono>
#include <numeric>

#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_event_loop.h>
#include <rom/crc.h>

#include <lwip/udp.h>

#include <freertos_util.hpp>
#include <result.hpp>
#include <timer.hpp>
#include <ringbuffer.hpp>
#include <vector3.hpp>

#include "espnow_packet.hpp"
#include <clock_source.hpp>

struct PBufPacket
{
    std::unique_ptr<pbuf, decltype(&pbuf_free)> buffer;
    SensorNodeAddress address;
    std::uint16_t port;
    Timestamp timestamp;
    

    PBufPacket() : PBufPacket(nullptr) {}
    PBufPacket(pbuf* buffer) : buffer(buffer, &pbuf_free), timestamp(Timestamp::zero()) {}
    PBufPacket(pbuf* buffer, const SensorNodeAddress& address, std::uint16_t port, Timestamp timestamp) : buffer(buffer, &pbuf_free), address(address), port(port), timestamp(timestamp) {}
    PBufPacket(const SensorNodePacket& packet) : PBufPacket(nullptr)
    {
        auto size = packet.total_size();
        this->buffer.reset( pbuf_alloc(PBUF_TRANSPORT, size, PBUF_RAM) );
        if( this->buffer ) {
            memcpy(this->buffer.get()->payload, &packet, size);
        }
    }
    PBufPacket(PBufPacket&& obj) : buffer(std::move(obj.buffer)), address(obj.address), port(obj.port), timestamp(obj.timestamp) {}

    pbuf* release() { return this->buffer.release(); }
    operator SensorNodePacket&() { return *reinterpret_cast<SensorNodePacket*>(this->buffer->payload); }
};


struct SensorCommunication
{
    static constexpr const char* TAG = "SENSORCOMM";

    struct ReceivedPBuf
    {
        pbuf* p;
        SensorNodeAddress address;
        std::uint16_t port;
        Timestamp timestamp;
    };

    SensorNodeAddress remote_address;
    std::uint16_t port;
    std::unique_ptr<udp_pcb, decltype(&udp_remove)> pcb;
    freertos::WaitQueue<ReceivedPBuf, 16> received_packet_queue;
    IClockSource& clock;

    static std::unique_ptr<pbuf, decltype(&pbuf_free)> to_unique_ptr(pbuf* p) {
        return std::unique_ptr<pbuf, decltype(&pbuf_free)>(p, &pbuf_free);
    }

    static void udp_recv_handler(void* arg, udp_pcb* pcb, pbuf* p, const ip_addr_t* addr, std::uint16_t port)
    {
        ESP_LOGD(TAG, "Packet received from " IPSTR " port %d", IP2STR(*addr), port);
        auto this_ = reinterpret_cast<SensorCommunication*>(arg);
        if( this_ != nullptr && this_->port == port ) {
            if( this_->received_packet_queue.send({p, SensorNodeAddress(addr), port, this_->clock.now()} , freertos::Ticks::zero()) ) {
                return;
            }
        }
        pbuf_free(p);
    }

    SensorCommunication() : pcb(nullptr, &udp_remove), clock(MonotonicClockSource::instance)
    {
    }
    Result<void, esp_err_t> start(const SensorNodeAddress& address, std::uint16_t port)
    {
        this->remote_address = address;
        this->port = port;

        this->pcb.reset(udp_new());
        if( !this->pcb ) {
            ESP_LOGE(TAG, "failed to initialize UDP");
            return failure(ESP_FAIL);
        }
        auto bind_result = udp_bind(this->pcb.get(), IP4_ADDR_ANY, port);
        if( bind_result != err_enum_t::ERR_OK ) {
            ESP_LOGE(TAG, "failed to bind to the UDP port");
            return failure(ESP_FAIL);
        }

        udp_recv(this->pcb.get(), &udp_recv_handler, this);
        ESP_LOGI(TAG, "SensorCommunication started address:" IPSTR " port:%d", IP2STR(address.ip_address), port);
        
        return success();
    }

    Result<void, bool> send(PBufPacket packet, const SensorNodeAddress& remote_address) {
        auto result = udp_sendto(this->pcb.get(), packet.buffer.get(), remote_address, this->port);
        if( result == ERR_OK ) {
            return success();
        }
        return failure(false);
    }
    Result<void, bool> send(PBufPacket packet) {
        auto result = udp_sendto(this->pcb.get(), packet.buffer.get(), this->remote_address, this->port);
        if( result == ERR_OK ) {
            return success();
        }
        return failure(false);
    }
    Result<PBufPacket, bool> receive(freertos::Ticks wait_ticks) {
        ReceivedPBuf received_pbuf;
        if( !this->received_packet_queue.receive(received_pbuf, wait_ticks) ) {
            return failure(false);
        }
        return success<PBufPacket>(std::move(PBufPacket(received_pbuf.p, received_pbuf.address, received_pbuf.port, received_pbuf.timestamp)));
    }
};

#endif //SENSOR_COMMUNICATION_HPP__