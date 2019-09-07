#ifndef SENSOR_NODE_HPP__
#define SENSOR_NODE_HPP__

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <memory>
#include <vector>
#include <chrono>
#include <numeric>

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

#include "espnow_packet.hpp"
#include <clock_source.hpp>
#include <sensor_communication.hpp>

struct SensorNode : public freertos::StaticTask<8192, SensorNode>
{
    static constexpr const char* TAG = "SENSOR";

    enum class State
    {
        Idle,
        Connecting,
        Syncing,
        Connected,
    };

    freertos::WaitQueue<esp_now_send_status_t, 1> send_event_queue;
    RingBuffer<std::chrono::microseconds, 16, true> send_complete_elapsed;
    std::chrono::microseconds average_send_complete_elapsed;
    std::chrono::microseconds max_send_complete_elapsed;

    freertos::Mutex running_lock;
    bool running;
    State state;
    SensorNodeAddress address;
    std::uint16_t port;
    std::uint8_t mac_address[6];
    char name[32];

    std::chrono::microseconds connection_request_timestamp;
    std::chrono::microseconds connection_request_received_time;
    std::chrono::microseconds connection_response_send_time;
    std::chrono::microseconds notify_delay_timestamp;
    std::chrono::microseconds last_send_time;
    std::chrono::microseconds time_offset;
    std::chrono::microseconds measurement_target_time;
    freertos::WaitEvent* measurement_requested_event;
    SensorCommunication communication;
    SensorNodePacket sensor_data_packet;

    static std::chrono::microseconds get_timestamp() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
    }
    
    SensorNode() : running(false)
                 , state(State::Idle) 
                 , connection_request_timestamp(0)
                 , connection_request_received_time(0)
                 , connection_response_send_time(0)
                 , notify_delay_timestamp(0)
                 , last_send_time(0)
                 , time_offset(0)
                 , measurement_target_time(0)
                 , measurement_requested_event(nullptr)
                {}

    bool is_connected() const { return this->state == State::Connected; }
    std::chrono::microseconds get_adjusted_timestamp() const
    {
        return get_timestamp() + this->time_offset;
    }
    std::chrono::microseconds get_measurement_target_time() const
    {
        return this->measurement_target_time;
    }

    Result<void, esp_err_t> start(const SensorNodeAddress& address, std::uint16_t port, const std::uint8_t* mac_address, const char* name, freertos::WaitEvent* measurement_requested_event)
    {
        ESP_LOGI(TAG, "Starting Sensor node receiver addr:" IPSTR " port:%d", IP2STR(address.ip_address), port);
        memcpy(this->mac_address, mac_address, 6);
        strcpy(reinterpret_cast<char*>(this->name), name);
        this->address = address;
        this->port = port;
        {
            auto result = this->communication.start(this->address, this->port);
            if( !result ) {
                ESP_LOGE(TAG, "failed to start UDP communication");
                return failure(ESP_FAIL);
            }
        }
        {
            this->running = true;
            auto result = SensorNode::SelfType::start("SENSOR", 2, APP_CPU_NUM);
            if( !result ) {
                ESP_LOGE(TAG, "failed to start sensor task");
                this->running = false;
                esp_now_unregister_recv_cb();
                esp_now_unregister_send_cb();
                return failure(ESP_FAIL);
            }
        }
        this->measurement_requested_event = measurement_requested_event;
        this->running_lock.lock();
        return success();
    }

    void stop()
    {
        this->running = false;
    }

    Result<void, esp_err_t> send_packet(const SensorNodePacket& packet, const SensorNodeAddress* address = nullptr, freertos::Ticks wait_ticks = freertos::to_ticks(std::chrono::milliseconds(1000))) 
    {
        auto start_time = get_timestamp();
        auto result = this->communication.send(PBufPacket(packet), address != nullptr ? *address : this->address);
        if( !result ) {
            return failure(ESP_FAIL);
        }
        auto elapsed = get_timestamp() - start_time;
        this->send_complete_elapsed.queue(elapsed);
        auto max_it = std::max_element(this->send_complete_elapsed.begin(), this->send_complete_elapsed.end());
        if( max_it != this->send_complete_elapsed.end() ) {
            this->max_send_complete_elapsed = *max_it;
        }
        this->average_send_complete_elapsed = std::accumulate(this->send_complete_elapsed.begin(), this->send_complete_elapsed.end(), std::chrono::microseconds::zero()) / 16;
        return success();

    }
    void discovery_received(const PBufPacket& raw_packet, const SensorNodePacket&)
    {
        ESP_LOGI(TAG, "Discovery from " IPSTR " received", IP2STR(raw_packet.address.ip_address));
        SensorNodePacket packet;
        packet.magic = SensorNodePacket::MAGIC;
        packet.type = SensorNodePacketType::DiscoveryResponse;
        packet.set_timestamp(this->get_adjusted_timestamp());
        packet.reserved = 0;
        memcpy(packet.body.discovery_response.mac_address, this->mac_address, 6);
        strcpy(packet.body.discovery_response.name, this->name);
        packet.body.discovery_response.name_length = strlen(this->name);
        packet.set_size_and_crc();
        auto result = this->send_packet(packet, &raw_packet.address);
        if( !result ) {
            ESP_LOGE(TAG, "send error - %x", result.error_code);
        }
        else {
            ESP_LOGI(TAG, "send ok");
        }
    }

    
    void connection_request_received(const PBufPacket& raw_packet, const SensorNodePacket& packet)
    {
        ESP_LOGI(TAG, "Connection request from " IPSTR " received, timestamp=%llu", IP2STR(raw_packet.address.ip_address), packet.timestamp);
        this->connection_request_timestamp = packet.timestamp_typed();
        this->connection_request_received_time = raw_packet.timestamp + this->time_offset;
        this->address = raw_packet.address;
        this->state = State::Connecting;
        this->last_send_time = std::chrono::microseconds::zero();
    }
    void send_connection_response()
    {
        auto timestamp = this->get_adjusted_timestamp();
        if( timestamp - this->last_send_time  < std::chrono::seconds(2)) {
            return;
        }
        this->last_send_time = timestamp;
        SensorNodePacket packet;
        packet.magic = SensorNodePacket::MAGIC;
        packet.type = SensorNodePacketType::ConnectionResponse;
        packet.set_timestamp(this->connection_response_send_time = this->get_adjusted_timestamp());
        packet.reserved = 0;
        memcpy(packet.body.discovery_response.mac_address, this->mac_address, 6);
        strcpy(packet.body.discovery_response.name, this->name);
        packet.body.discovery_response.name_length = strlen(this->name);
        packet.set_size_and_crc();
        auto result = this->send_packet(packet);
        if( !result ) {
            ESP_LOGE(TAG, "send connection response packet - %x", result.error_code);
        }
        ESP_LOGI(TAG, "Sent connection response, timestamp=%llu", packet.timestamp);
    }
    void notify_delay_received(const PBufPacket& raw_packet, const SensorNodePacket& packet)
    {
        ESP_LOGI(TAG, "Notify delay from " IPSTR " received, timestamp=%llu", IP2STR(raw_packet.address.ip_address), packet.timestamp);
        this->notify_delay_timestamp = packet.timestamp_typed() + this->time_offset;
        this->state = State::Syncing;
        this->last_send_time = std::chrono::microseconds::zero();

        this->time_offset = -((this->connection_request_received_time - this->connection_request_timestamp) - (this->notify_delay_timestamp - this->connection_response_send_time)) / 2;
        ESP_LOGI(TAG, "Receiver and node clock offset = %lld", this->time_offset.count());
    }
    void send_delay_response()
    {
        auto timestamp = this->get_adjusted_timestamp();
        if( timestamp - this->last_send_time  < std::chrono::seconds(2)) {
            return;
        }
        this->last_send_time = timestamp;
        SensorNodePacket packet;
        packet.magic = SensorNodePacket::MAGIC;
        packet.type = SensorNodePacketType::DelayResponse;
        packet.set_timestamp(this->get_adjusted_timestamp());
        packet.reserved = 0;
        memcpy(packet.body.discovery_response.mac_address, this->mac_address, 6);
        strcpy(packet.body.discovery_response.name, this->name);
        packet.body.discovery_response.name_length = strlen(this->name);
        packet.set_size_and_crc();
        auto result = this->send_packet(packet);
        if( !result ) {
            ESP_LOGE(TAG, "send connection response packet - %x", result.error_code);
        }
        else {
            this->state = State::Connected;
        }
    }
    void measurement_request_received(const PBufPacket& raw_packet, const SensorNodePacket& packet_received)
    {
        ESP_LOGI(TAG, "Measurement request from " IPSTR " received", IP2STR(raw_packet.address.ip_address));
        this->measurement_target_time = std::chrono::microseconds(packet_received.body.measurement_request.target_time);
        
        SensorNodePacket packet;
        packet.magic = SensorNodePacket::MAGIC;
        packet.type = SensorNodePacketType::MeasurementResponse;
        packet.set_timestamp(this->get_adjusted_timestamp());
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = this->send_packet(packet);
        if( !result ) {
            ESP_LOGE(TAG, "send measurement response packet - %x", result.error_code);
        }

        if( this->measurement_requested_event != nullptr ) {
            this->measurement_requested_event->set();
        }
    }

    void send_sensor_data(std::chrono::microseconds timestamp, const Vector3F& acc, const Vector3F& gyro, const Vector3F& mag)
    {
        SensorNodePacket& packet = this->sensor_data_packet;

        if( packet.body.sensor_data.number_of_samples == 0 ) {
            packet.body.sensor_data.set_timestamp(timestamp);
        }
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].acc , acc .items.data(), sizeof(float)*3);
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].gyro, gyro.items.data(), sizeof(float)*3);
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].mag , mag .items.data(), sizeof(float)*3);
        packet.body.sensor_data.number_of_samples++;
        //ESP_LOGI(TAG, "sensor data: %0.2f, %0.2f, %0.2f", acc.x_const(), acc.y_const(), acc.z_const());

        if( packet.body.sensor_data.number_of_samples == SensorNodePacket::MAX_SENSOR_DATA_SAMPLES ) {
            packet.magic = SensorNodePacket::MAGIC;

            packet.type = SensorNodePacketType::SensorData;
            packet.set_timestamp(this->get_adjusted_timestamp());
            packet.reserved = 0;
            packet.set_size_and_crc();
            auto result = this->send_packet(packet);
            if( !result ) {
                ESP_LOGE(TAG, "send sensor data packet - %x", result.error_code);
            }
            packet.body.sensor_data.number_of_samples = 0;
        }
    }
    
    void operator() ()
    {
        while(this->running)
        {
            auto result = this->communication.receive(freertos::to_ticks(std::chrono::milliseconds(10)));
            if( result ) {
                auto& packet = static_cast<SensorNodePacket&>(result.value);
                auto validation_result = packet.validate();
                if( !validation_result ) {
                    ESP_LOGE(TAG, "invalid packet - %d", static_cast<int>(validation_result.error_code));
                }
                else {
                    switch(packet.type)
                    {
                    case SensorNodePacketType::Discovery:
                        this->discovery_received(result.value, packet);
                        break;
                    case SensorNodePacketType::ConnectionRequest:
                        this->connection_request_received(result.value, packet);
                        break;
                    case SensorNodePacketType::NotifyDelay:
                        this->notify_delay_received(result.value, packet);
                        break;
                    case SensorNodePacketType::MeasurementRequest:
                        this->measurement_request_received(result.value, packet);
                        break;
                    default:
                        break;
                    }
                }
            }
            if( this->state == State::Connecting ) {
                this->send_connection_response();
            }
            if( this->state == State::Syncing ) {
                this->send_delay_response();
            }
        }
        this->running_lock.release();
    }

    ~SensorNode()
    {
    }
};

#endif //SENSOR_NODE_HPP__