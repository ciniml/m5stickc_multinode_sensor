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
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>
#include <driver/i2c.h>
#include <rom/crc.h>

#include <freertos_util.hpp>
#include <result.hpp>
#include <timer.hpp>
#include <ringbuffer.hpp>
#include <vector3.hpp>

enum class ESPNowPacketType : std::uint8_t
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
};

struct __attribute__((packed)) ESPNowAddress
{
    std::uint8_t values[ESP_NOW_ETH_ALEN];

    ESPNowAddress() = default;
    ESPNowAddress(const std::uint8_t* values)
    {
        memcpy(this->values, values, ESP_NOW_ETH_ALEN);
    }
    operator const std::uint8_t*() const
    {
        return this->values;
    }
    int compare(const ESPNowAddress& rhs) const
    {
        return memcmp(this->values, rhs.values, ESP_NOW_ETH_ALEN);
    }
    bool operator<(const ESPNowAddress& rhs) const
    {
        return this->compare(rhs) < 0;
    }
    bool operator<=(const ESPNowAddress& rhs) const
    {
        return this->compare(rhs) <= 0;
    }
    bool operator>(const ESPNowAddress& rhs) const
    {
        return this->compare(rhs) > 0;
    }
    bool operator>=(const ESPNowAddress& rhs) const
    {
        return this->compare(rhs) >= 0;
    }
    bool operator==(const ESPNowAddress& rhs) const
    {
        return this->compare(rhs) == 0;
    }
    bool operator!=(const ESPNowAddress& rhs) const
    {
        return !this->operator==(rhs);
    }

    void copy_from(const void* values)
    {
        memcpy(this->values, values, ESP_NOW_ETH_ALEN);
    }
    void copy_to(void* values) const
    {
        memcpy(values, this->values, ESP_NOW_ETH_ALEN);
    }
};

struct __attribute__((packed)) ESPNowPacket
{
    static constexpr std::uint16_t MAGIC = 0xBEEF;
    static constexpr std::size_t HEADER_SIZE = 16;
    std::uint16_t magic;
    ESPNowPacketType type;
    std::uint8_t reserved;
    std::uint8_t sequence;
    std::uint8_t size;
    std::uint16_t crc;
    std::uint64_t timestamp;
    union 
    {
        struct
        {
            std::uint64_t target_time;
        } measurement_request;
        struct 
        {
            std::uint8_t number_of_samples;
            std::uint8_t reserved[3];
            std::uint64_t timestamp;
            struct 
            {
                float acc[3];
                float gyro[3];
                float mag[3];
            } samples[4];
            template<typename Timestamp> void set_timestamp(Timestamp timestamp) { this->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(timestamp).count(); }
        } sensor_data;
    } body;

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
        case ESPNowPacketType::Discovery:  return success<std::size_t>(0);
        case ESPNowPacketType::DiscoveryResponse:  return success<std::size_t>(0);
        case ESPNowPacketType::ConnectionRequest:  return success<std::size_t>(0);
        case ESPNowPacketType::ConnectionResponse:  return success<std::size_t>(0);
        case ESPNowPacketType::NotifyDelay:  return success<std::size_t>(0);
        case ESPNowPacketType::DelayResponse:  return success<std::size_t>(0);
        case ESPNowPacketType::MeasurementRequest:  return success(sizeof(this->body.measurement_request));
        case ESPNowPacketType::MeasurementResponse:  return success<std::size_t>(0);
        case ESPNowPacketType::SensorData: return success(sizeof(this->body.sensor_data));
        default: return failure(false);
        }
    }
    void set_size_and_crc()
    {
        this->size = this->get_size_from_type().unwrap_or(0);
        this->crc  = crc16_le(0, reinterpret_cast<const std::uint8_t*>(&this->body), this->size);
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
        auto crc  = crc16_le(0, reinterpret_cast<const std::uint8_t*>(&this->body), expected_size.value);
        if( this->crc != crc ) {
            return failure(ValidateError::CRCMismatch);
        }

        return success();
    }
};

enum class ESPNowCallbackEventType
{
    SendEvent,
    ReceiveEvent,
};
struct ESPNowCallbackEvent
{
    static constexpr std::size_t FIXED_BUFFER_SIZE = sizeof(ESPNowPacket);
    ESPNowCallbackEventType type;
    std::uint8_t  buffer[FIXED_BUFFER_SIZE];
    ESPNowAddress  address;
    std::size_t   length;
    std::chrono::microseconds timestamp;

    ESPNowCallbackEvent() : length(0) {}

    template<typename Timestamp>
    ESPNowCallbackEvent(ESPNowCallbackEventType type, const ESPNowAddress& address, const std::uint8_t* data, std::size_t length, Timestamp timestamp) 
         : type(type), address(address), length(0), timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(timestamp))
    {
        if( length > FIXED_BUFFER_SIZE ) {
            return;
        }
        memcpy(this->buffer, data, length);
        this->length = length;
    }


    ESPNowCallbackEvent(const ESPNowCallbackEvent& obj) : type(obj.type), length(obj.length)
    {
        this->address.copy_from(obj.address);
        memcpy(this->buffer, obj.buffer, obj.length);
    }

    ESPNowCallbackEvent& operator= (ESPNowCallbackEvent&& obj)
    {
        this->type = obj.type;
        this->length = obj.length;
        this->address = obj.address;
        memcpy(this->buffer, obj.buffer, obj.length);
        return *this;
    }
};

#endif //ESPNOW_PACKET_HPP__