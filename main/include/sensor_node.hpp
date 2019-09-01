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

    freertos::WaitQueue<ESPNowCallbackEvent, 16> event_queue;
    freertos::WaitQueue<esp_now_send_status_t, 1> send_event_queue;
    RingBuffer<std::chrono::microseconds, 16, true> send_complete_elapsed;
    std::chrono::microseconds average_send_complete_elapsed;
    std::chrono::microseconds max_send_complete_elapsed;

    freertos::Mutex running_lock;
    volatile bool running;
    volatile State state;
    ESPNowAddress receiver_address;

    std::chrono::microseconds connection_request_timestamp;
    std::chrono::microseconds connection_request_received_time;
    std::chrono::microseconds connection_response_send_time;
    std::chrono::microseconds notify_delay_timestamp;
    std::chrono::microseconds last_send_time;
    std::chrono::microseconds time_offset;
    std::chrono::microseconds measurement_target_time;
    freertos::WaitEvent* measurement_requested_event;
    ESPNowPacket sensor_data_packet;

    static SensorNode* instance;

    static std::chrono::microseconds get_timestamp() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
    }

    static void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        if( instance != nullptr )  {
            instance->send_event_queue.send(status);
        }    
    }
    static void receive_callback(const uint8_t *mac_addr, const uint8_t *data, int len)
    {
        if( instance != nullptr )  {
            instance->event_queue.send(ESPNowCallbackEvent(ESPNowCallbackEventType::ReceiveEvent, ESPNowAddress(mac_addr), data, len, get_timestamp()));
        }    
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

    Result<void, esp_err_t> start(freertos::WaitEvent* measurement_requested_event)
    {
        {
            esp_err_t result = ESP_OK;

            result = esp_now_register_send_cb(&send_callback);
            if( result != ESP_OK ) {
                ESP_LOGE(TAG, "failed to register send cb - %x", result);
                return failure(result);
            }
            result = esp_now_register_recv_cb(&receive_callback);
            if( result != ESP_OK ) {
                ESP_LOGE(TAG, "failed to register recv cb - %x", result);
                esp_now_unregister_send_cb();
                return failure(result);
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
        instance = this;
        this->running_lock.lock();
        return success();
    }

    void stop()
    {
        this->running = false;
        auto guard = freertos::lock(this->running_lock);
        if( instance == this ) {
            esp_now_unregister_send_cb();            
            esp_now_unregister_recv_cb();
            instance = nullptr;
        }
    }

    Result<void, esp_err_t> send_packet(const ESPNowAddress& address, const ESPNowPacket& packet, freertos::Ticks wait_ticks = freertos::to_ticks(std::chrono::milliseconds(1000))) 
    {
        auto start_time = get_timestamp();
        this->send_event_queue.reset();
        auto result = esp_now_send(address, reinterpret_cast<const std::uint8_t*>(&packet), packet.total_size());
        if( result != ESP_OK ) {
            return failure(result);
        }
        esp_now_send_status_t status;
        auto wait_result = this->send_event_queue.receive(status, wait_ticks);
        if( !wait_result ) {
            return failure(ESP_ERR_TIMEOUT);
        }
        if( status != esp_now_send_status_t::ESP_NOW_SEND_SUCCESS ) {
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
    void discovery_received(const ESPNowCallbackEvent& event, const ESPNowPacket&)
    {
        // Add peer list
        if( !esp_now_is_peer_exist(event.address) ) {
            esp_now_peer_info_t peer;
            memset(&peer, 0, sizeof(peer));
            peer.channel = 6;
            peer.ifidx = ESP_IF_WIFI_AP;
            peer.encrypt = false;
            event.address.copy_to(peer.peer_addr);
            auto result = esp_now_add_peer(&peer);
            if( result != ESP_OK ) {
                ESP_LOGE(TAG, "failed to add peer " MACSTR " - %x", MAC2STR(event.address), result);
            }
        }
        ESPNowPacket packet;
        packet.magic = ESPNowPacket::MAGIC;
        packet.sequence = 0;
        packet.type = ESPNowPacketType::DiscoveryResponse;
        packet.set_timestamp(this->get_adjusted_timestamp());
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = this->send_packet(event.address, packet);
        if( !result ) {
            ESP_LOGE(TAG, "send error - %x", result.error_code);
        }
        else {
            ESP_LOGI(TAG, "send ok");
        }
    }

    
    void connection_request_received(const ESPNowCallbackEvent& event, const ESPNowPacket& packet)
    {
        ESP_LOGI(TAG, "Connection request from " MACSTR " received", MAC2STR(event.address.values));
        this->connection_request_timestamp = packet.timestamp_typed();
        this->connection_request_received_time = event.timestamp + this->time_offset;
        this->state = State::Connecting;
        this->last_send_time = std::chrono::microseconds::zero();
        this->receiver_address = event.address;
        // Add peer list
        if( !esp_now_is_peer_exist(event.address) ) {
            esp_now_peer_info_t peer;
            memset(&peer, 0, sizeof(peer));
            peer.channel = 6;
            peer.ifidx = ESP_IF_WIFI_AP;
            peer.encrypt = false;
            event.address.copy_to(peer.peer_addr);
            auto result = esp_now_add_peer(&peer);
            if( result != ESP_OK ) {
                ESP_LOGE(TAG, "failed to add peer " MACSTR " - %x", MAC2STR(event.address), result);
            }
        }
    }
    void send_connection_response()
    {
        auto timestamp = this->get_adjusted_timestamp();
        if( timestamp - this->last_send_time  < std::chrono::seconds(2)) {
            return;
        }
        this->last_send_time = timestamp;
        ESPNowPacket packet;
        packet.magic = ESPNowPacket::MAGIC;
        packet.sequence = 0;
        packet.type = ESPNowPacketType::ConnectionResponse;
        packet.set_timestamp(this->connection_response_send_time = this->get_adjusted_timestamp());
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = this->send_packet(this->receiver_address, packet);
        if( !result ) {
            ESP_LOGE(TAG, "send connection response packet - %x", result.error_code);
        }
    }
    void notify_delay_received(const ESPNowCallbackEvent& event, const ESPNowPacket& packet)
    {
        ESP_LOGI(TAG, "Notify delay from " MACSTR " received", MAC2STR(event.address.values));
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
        ESPNowPacket packet;
        packet.magic = ESPNowPacket::MAGIC;
        packet.sequence = 0;
        packet.type = ESPNowPacketType::DelayResponse;
        packet.set_timestamp(this->get_adjusted_timestamp());
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = this->send_packet(this->receiver_address, packet);
        if( !result ) {
            ESP_LOGE(TAG, "send connection response packet - %x", result.error_code);
        }
        else {
            this->state = State::Connected;
        }
    }
    void measurement_request_received(const ESPNowCallbackEvent& event, const ESPNowPacket& packet_received)
    {
        ESP_LOGI(TAG, "Measurement request from " MACSTR " received", MAC2STR(event.address.values));
        
        ESPNowPacket packet;
        packet.magic = ESPNowPacket::MAGIC;
        packet.sequence = 0;
        packet.type = ESPNowPacketType::MeasurementResponse;
        packet.set_timestamp(this->get_adjusted_timestamp());
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = this->send_packet(this->receiver_address, packet);
        if( !result ) {
            ESP_LOGE(TAG, "send measurement response packet - %x", result.error_code);
        }

        this->measurement_target_time = std::chrono::microseconds(packet.body.measurement_request.target_time);
        if( this->measurement_requested_event != nullptr ) {
            this->measurement_requested_event->set();
        }
    }

    void send_sensor_data(std::chrono::microseconds timestamp, const Vector3F& acc, const Vector3F& gyro, const Vector3F& mag)
    {
        ESPNowPacket& packet = this->sensor_data_packet;

        if( packet.body.sensor_data.number_of_samples == 0 ) {
            packet.body.sensor_data.set_timestamp(timestamp);
        }
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].acc , acc .items.data(), sizeof(float)*3);
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].gyro, gyro.items.data(), sizeof(float)*3);
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].mag , mag .items.data(), sizeof(float)*3);
        packet.body.sensor_data.number_of_samples++;
        //ESP_LOGI(TAG, "sensor data: %0.2f, %0.2f, %0.2f", acc.x_const(), acc.y_const(), acc.z_const());

        if( packet.body.sensor_data.number_of_samples == 4 ) {
            packet.magic = ESPNowPacket::MAGIC;
            packet.sequence = 0;
            packet.type = ESPNowPacketType::SensorData;
            packet.set_timestamp(this->get_adjusted_timestamp());
            packet.reserved = 0;
            packet.set_size_and_crc();
            auto result = this->send_packet(this->receiver_address, packet);
            if( !result ) {
                ESP_LOGE(TAG, "send sensor data packet - %x", result.error_code);
            }
            packet.body.sensor_data.number_of_samples = 0;
        }
    }
    
    void operator() ()
    {
        ESPNowCallbackEvent event;
        while(this->running)
        {
            auto result = this->event_queue.receive(event);
            if( result ) {
                if( event.type == ESPNowCallbackEventType::ReceiveEvent ) {
                    auto& packet = *reinterpret_cast<ESPNowPacket*>(event.buffer);
                    auto validation_result = packet.validate();
                    if( !validation_result ) {
                        ESP_LOGE(TAG, "invalid packet - %d", static_cast<int>(validation_result.error_code));
                    }
                    else {
                        switch(packet.type)
                        {
                        case ESPNowPacketType::Discovery:
                            this->discovery_received(event, packet);
                            break;
                        case ESPNowPacketType::ConnectionRequest:
                            this->connection_request_received(event, packet);
                            break;
                        case ESPNowPacketType::NotifyDelay:
                            this->notify_delay_received(event, packet);
                            break;
                        case ESPNowPacketType::MeasurementResponse:
                            this->measurement_request_received(event, packet);
                            break;
                        default:
                            break;
                        }
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
        if( instance == this ) {
            esp_now_unregister_send_cb();            
            esp_now_unregister_recv_cb();
            instance = nullptr;
        }
    }
};

SensorNode* SensorNode::instance = nullptr;

#endif //SENSOR_NODE_HPP__