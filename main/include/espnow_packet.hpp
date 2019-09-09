#ifndef ESPNOW_PACKET_HPP__
#define ESPNOW_PACKET_HPP__

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <memory>
#include <vector>
#include <chrono>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>

#include <lwip/udp.h>

#include <freertos_util.hpp>
#include <result.hpp>
#include <timer.hpp>
#include <ringbuffer.hpp>
#include <vector3.hpp>


#define IP2STR(a) (((a).u_addr.ip4.addr) & 0xff),(((a).u_addr.ip4.addr >> 8) & 0xff),(((a).u_addr.ip4.addr >> 16) & 0xff),(((a).u_addr.ip4.addr >> 24))
#define IPSTR "%d.%d.%d.%d"


struct SensorNodeAddress
{
    static SensorNodeAddress broadcast() { return SensorNodeAddress(IP4_ADDR_BROADCAST); }

    ip_addr_t ip_address;
    
    SensorNodeAddress() = default;
    SensorNodeAddress(const SensorNodeAddress& obj) : ip_address(obj.ip_address) {};
    SensorNodeAddress(const ip_addr_t& ip_address) : ip_address(ip_address) {}
    SensorNodeAddress(const ip_addr_t* ip_address) : ip_address(*ip_address) {}
    SensorNodeAddress(const ip4_addr_t& ip_address) {
        memset(&this->ip_address, 0, sizeof(this->ip_address));
        this->ip_address.type = IPADDR_TYPE_V4;
        this->ip_address.u_addr.ip4 = ip_address;
    }
    SensorNodeAddress(const ip4_addr_t* ip_address) : SensorNodeAddress(*ip_address) {}

    bool equals(const ip_addr_t& ip_address) const 
    {
        return ip_addr_cmp(&this->ip_address, &ip_address); 
    }

    bool operator==(const SensorNodeAddress& rhs) const { return  this->equals(rhs.ip_address); }
    bool operator!=(const SensorNodeAddress& rhs) const { return !this->equals(rhs.ip_address); }

    operator const ip_addr_t*() const { return &this->ip_address; }
    operator const ip_addr_t() const { return this->ip_address; }
};


namespace std {
template<> struct hash<SensorNodeAddress>
{
    size_t operator()(const SensorNodeAddress& obj) const {
        return obj.ip_address.type == IPADDR_TYPE_V4
            ? obj.ip_address.u_addr.ip4.addr
            : (obj.ip_address.u_addr.ip6.addr[0] ^ obj.ip_address.u_addr.ip6.addr[1] ^ obj.ip_address.u_addr.ip6.addr[2] ^ obj.ip_address.u_addr.ip6.addr[3]);
    }
};
};

enum class SensorNodePacketType : std::uint8_t
{
    Discovery,
    DiscoveryResponse,
    ConnectionRequest,
    ConnectionResponse,
    NotifyDelay,
    DelayResponse,
    MeasurementRequest,
    MeasurementResponse,
    SensorData,
    SyncRequest,
    SyncResponse,
};


struct __attribute__((packed)) SensorNodePacket
{
    static constexpr std::uint16_t MAGIC = 0xBEEF;
    static constexpr std::size_t HEADER_SIZE = 16;
    static constexpr std::size_t MAX_SENSOR_DATA_SAMPLES = 20;
    std::uint16_t magic;
    SensorNodePacketType type;
    std::uint8_t reserved;
    std::uint16_t size;
    std::uint16_t crc;
    std::uint64_t timestamp;
    union 
    {
        struct __attribute__((packed))
        {
            std::uint8_t mac_address[6];
            std::uint8_t name_length;
            char name[32];
        } discovery_response;
        struct __attribute__((packed))
        {
            std::uint64_t target_time;
        } measurement_request;
        struct __attribute__((packed))
        {
            std::uint8_t target_mac[6];
        } sync_request;
        struct __attribute__((packed))
        {
            std::uint8_t number_of_samples;
            std::uint8_t reserved[7];
            std::uint64_t timestamp;
            struct 
            {
                float acc[3];
                float gyro[3];
                float mag[3];
            } samples[MAX_SENSOR_DATA_SAMPLES];
            template<typename Timestamp> void set_timestamp(Timestamp timestamp) { this->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(timestamp).count(); }
        } sensor_data;
    } body;

    void copy(const SensorNodePacket& packet)
    {
        memcpy(this, &packet, HEADER_SIZE);
        memcpy(&this->body, &packet.body, packet.size);
    }
    std::chrono::microseconds timestamp_typed() const 
    {
        return std::chrono::microseconds(this->timestamp);
    }
    template<typename Timestamp>
    void set_timestamp(const Timestamp& timestamp)
    {
        this->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(timestamp).count();
    }
    Result<std::size_t, bool> get_size_from_type() const 
    {
        switch(this->type) {
        case SensorNodePacketType::Discovery:  return success<std::size_t>(0);
        case SensorNodePacketType::DiscoveryResponse:  return success(sizeof(this->body.discovery_response));
        case SensorNodePacketType::ConnectionRequest:  return success<std::size_t>(0);
        case SensorNodePacketType::ConnectionResponse:  return success(sizeof(this->body.discovery_response));
        case SensorNodePacketType::SyncRequest:  return success<std::size_t>(sizeof(this->body.sync_request));
        case SensorNodePacketType::SyncResponse:  return success<std::size_t>(0);
        case SensorNodePacketType::NotifyDelay:  return success<std::size_t>(0);
        case SensorNodePacketType::DelayResponse:  return success<std::size_t>(0);
        case SensorNodePacketType::MeasurementRequest:  return success(sizeof(this->body.measurement_request));
        case SensorNodePacketType::MeasurementResponse:  return success<std::size_t>(0);
        case SensorNodePacketType::SensorData: return success(sizeof(this->body.sensor_data));
        default: return failure(false);
        }
    }
    void set_size_and_crc()
    {
        this->magic = MAGIC;
        this->size = this->get_size_from_type().unwrap_or(0);
        this->crc  = static_cast<std::uint16_t>(~crc16_le(0xffff, reinterpret_cast<const std::uint8_t*>(&this->body), this->size));
    }

    std::size_t total_size() const { return this->size + HEADER_SIZE; }
    enum class ValidateError
    {
        OK = 0,
        MagicMismatch,
        InvalidType,
        UnexpectedSize,
        CRCMismatch,
    };
    Result<void, ValidateError> validate() const 
    {
        if( this->magic != MAGIC ) {
            return failure(ValidateError::MagicMismatch);
        }
        auto expected_size = this->get_size_from_type();
        if( !expected_size ) {
            return failure(ValidateError::InvalidType);
        }
        if( this->size != expected_size.value ) {
            return failure(ValidateError::UnexpectedSize);
        }
        //auto crc  = static_cast<std::uint16_t>(~crc16_le(0xffff, reinterpret_cast<const std::uint8_t*>(&this->body), expected_size.value));
        //if( this->crc != crc ) {
        //    ESP_LOGE("PACKET", "CRC error expected=%04x, actual=%04x", crc, this->crc);
        //    return failure(ValidateError::CRCMismatch);
        //}

        return success();
    }
};

#endif //ESPNOW_PACKET_HPP__