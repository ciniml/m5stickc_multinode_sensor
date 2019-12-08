#ifndef SENSOR_NODE_HPP__
#define SENSOR_NODE_HPP__

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <memory>
#include <vector>
#include <chrono>
#include <numeric>
#include <algorithm>

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

struct BatterySensorData
{
    float voltage;
    float charge_current;
    float discharge_current;
};

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
    std::uint32_t sensor_data_packet_index;

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

    enum class NodeSyncState
    {
        Idle,
        // Node sync master state
        SendingSyncRequest,
        WaitingSyncResponse,
        SendingNotifyDelay,
        WaitingDelayResponse,
        DelayResponseReceived,
        // Node sync slave state
        SendingSyncResponse,
        WaitingNotifyDelay,
        SendingDelayResponse,
    };
    struct NodeSyncContext {
        NodeSyncState state;
        std::uint8_t mac_address[6];
        std::chrono::microseconds last_send_time;
        std::chrono::microseconds sync_request_timestamp;
        std::chrono::microseconds sync_request_received_timestamp;
        std::chrono::microseconds sync_response_timestamp;
        std::chrono::microseconds notify_delay_timestamp;
        NodeSyncContext() : state(NodeSyncState::Idle) {}
    };
    NodeSyncContext node_sync_context;

    static std::chrono::microseconds get_timestamp() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
    }
    
    SensorNode() : running(false)
                 , state(State::Idle)
                 , sensor_data_packet_index(0)
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

    Result<void, esp_err_t> ensure_esp_now_peer_exist(const std::uint8_t* mac_address)
    {
        if( !esp_now_is_peer_exist(mac_address) ) {
            esp_now_peer_info_t peer;
            memset(&peer, 0, sizeof(peer));
            peer.channel = 1;
            peer.ifidx = ESP_IF_WIFI_STA;
            memcpy(peer.peer_addr, mac_address, 6);
            auto result = esp_now_add_peer(&peer);
            if( result != ESP_OK ) {
                return failure(result);
            }
        }
        return success();
    }
    void process_esp_now_packet(const std::uint8_t* mac_addr, const std::uint8_t* data , int data_len)
    {
        auto timestamp = get_timestamp();
        auto adjusted_timestamp = this->get_adjusted_timestamp();
        ESP_LOGD(TAG, "ESP-NOW received from " MACSTR, MAC2STR(mac_addr));

        if( data_len >= SensorNodePacket::HEADER_SIZE ) {
            auto packet = reinterpret_cast<const SensorNodePacket*>(data);
            auto validate_result = packet->validate();
            if( !validate_result ) {
                ESP_LOGI(TAG, "ESP-NOW invalid packet %d", static_cast<std::uint8_t>(validate_result.error_code));
                return;
            }
            ESP_LOGD(TAG, "ESP-NOW received %d from " MACSTR, static_cast<std::uint8_t>(packet->type), MAC2STR(mac_addr));

            this->ensure_esp_now_peer_exist(mac_address);
            
            switch(packet->type) {
                case SensorNodePacketType::SyncRequest: {
                    if( this->node_sync_context.state == NodeSyncState::Idle ) {
                        this->node_sync_context.sync_request_timestamp  = std::chrono::microseconds(packet->timestamp);
                        this->node_sync_context.sync_request_received_timestamp = timestamp;
                        memcpy(this->node_sync_context.mac_address, mac_addr, 6);
                        this->node_sync_context.state = NodeSyncState::SendingSyncResponse;
                    }
                    break;
                }
                case SensorNodePacketType::NotifyDelay: {
                    if( this->node_sync_context.state == NodeSyncState::WaitingNotifyDelay ) {
                        this->node_sync_context.notify_delay_timestamp = std::chrono::microseconds(packet->timestamp);
                        this->node_sync_context.state = NodeSyncState::SendingDelayResponse;
                    }
                    break;
                }
                case SensorNodePacketType::SyncResponse: {
                    if( this->node_sync_context.state == NodeSyncState::WaitingSyncResponse ) {
                        this->node_sync_context.sync_response_timestamp = adjusted_timestamp;
                        this->node_sync_context.state = NodeSyncState::SendingNotifyDelay;
                    }
                    break;
                }
                case SensorNodePacketType::DelayResponse: {
                    if( this->node_sync_context.state == NodeSyncState::WaitingDelayResponse ) {
                        if( memcmp(mac_addr, this->node_sync_context.mac_address, 6) == 0 ) {
                            this->node_sync_context.state = NodeSyncState::DelayResponseReceived;
                        }
                    }
                    break;
                }
                
                default:
                    break;
            }
        }
    }

    void process_node_sync()
    {
        this->ensure_esp_now_peer_exist(this->node_sync_context.mac_address);

        switch(this->node_sync_context.state) {
            case NodeSyncState::SendingSyncRequest: {
                SensorNodePacket packet;
                packet.type = SensorNodePacketType::SyncRequest;
                packet.set_timestamp(this->get_adjusted_timestamp());
                packet.set_size_and_crc();
                auto result = esp_now_send(this->node_sync_context.mac_address, reinterpret_cast<const std::uint8_t*>(&packet), packet.total_size());
                if( result == ESP_OK ) {
                    ESP_LOGI(TAG, "Send SyncRequest to " MACSTR, MAC2STR(this->node_sync_context.mac_address));
                    this->last_send_time = get_timestamp();
                    this->node_sync_context.state = NodeSyncState::WaitingSyncResponse;
                }
                break;
            }
            case NodeSyncState::WaitingSyncResponse: {
                if( get_timestamp() - this->last_send_time >= std::chrono::seconds(2) ) {
                    this->node_sync_context.state = NodeSyncState::SendingSyncRequest;
                }
                break;
            }
            case NodeSyncState::SendingSyncResponse: {
                this->node_sync_context.sync_response_timestamp = get_timestamp();
                SensorNodePacket packet;
                packet.type = SensorNodePacketType::SyncResponse;
                packet.set_timestamp(this->node_sync_context.sync_response_timestamp);
                packet.set_size_and_crc();
                auto result = esp_now_send(this->node_sync_context.mac_address, reinterpret_cast<const std::uint8_t*>(&packet), packet.total_size());
                if( result == ESP_OK ) {
                    ESP_LOGI(TAG, "Send SyncResponse to " MACSTR, MAC2STR(this->node_sync_context.mac_address));
                    this->last_send_time = get_timestamp();
                    this->node_sync_context.state = NodeSyncState::WaitingNotifyDelay;
                }
                break;
            }
            case NodeSyncState::WaitingNotifyDelay: {
                if( get_timestamp() - this->last_send_time >= std::chrono::seconds(2) ) {
                    this->node_sync_context.state = NodeSyncState::SendingSyncResponse;
                }
                break;
            }
            case NodeSyncState::SendingNotifyDelay: {
                SensorNodePacket packet;
                packet.type = SensorNodePacketType::NotifyDelay;
                packet.set_timestamp(this->node_sync_context.sync_response_timestamp);
                packet.set_size_and_crc();
                auto result = esp_now_send(this->node_sync_context.mac_address, reinterpret_cast<const std::uint8_t*>(&packet), packet.total_size());
                if( result == ESP_OK ) {
                    ESP_LOGI(TAG, "Send NotifyDelay to " MACSTR, MAC2STR(this->node_sync_context.mac_address));
                    this->last_send_time = get_timestamp();
                    this->node_sync_context.state = NodeSyncState::WaitingDelayResponse;
                }
                break;
            }
            case NodeSyncState::WaitingDelayResponse: {
                if( get_timestamp() - this->last_send_time >= std::chrono::seconds(2) ) {
                    this->node_sync_context.state = NodeSyncState::SendingNotifyDelay;
                }
                break;
            }
            case NodeSyncState::SendingDelayResponse: {
                SensorNodePacket packet;
                packet.type = SensorNodePacketType::DelayResponse;
                packet.set_timestamp(get_timestamp());
                packet.set_size_and_crc();
                auto result = esp_now_send(this->node_sync_context.mac_address, reinterpret_cast<const std::uint8_t*>(&packet), packet.total_size());
                if( result == ESP_OK ) {
                    // Calculate offset 
                    this->time_offset = -((this->node_sync_context.sync_request_received_timestamp - this->node_sync_context.sync_request_timestamp) - (this->node_sync_context.notify_delay_timestamp - this->node_sync_context.sync_response_timestamp)) / 2;
                    ESP_LOGI(TAG, "Send DelayResponse to " MACSTR, MAC2STR(this->node_sync_context.mac_address));
                    ESP_LOGI(TAG, "Node sync offset=%lld", this->time_offset.count());
                    this->last_send_time = get_timestamp();
                    this->node_sync_context.state = NodeSyncState::Idle;
                } 
                break;
            }
            case NodeSyncState::DelayResponseReceived: {
                SensorNodePacket packet;
                packet.type = SensorNodePacketType::SyncResponse;
                packet.set_timestamp(get_timestamp());
                packet.set_size_and_crc();
                auto result = this->send_packet(packet);
                if( result ) {
                    this->node_sync_context.state = NodeSyncState::Idle;
                }
                break;
            }
            
            default:
                break;
        }
    }
    Result<void, esp_err_t> start(const SensorNodeAddress& address, std::uint16_t port, const std::uint8_t* mac_address, const char* name, freertos::WaitEvent* measurement_requested_event)
    {
        ESP_LOGI(TAG, "Starting Sensor node addr:" IPSTR " port:%d", IP2STR(address.ip_address), port);
        memcpy(this->mac_address, mac_address, 6);
        strncpy(reinterpret_cast<char*>(this->name), name, sizeof(this->name)-1);
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

    void sync_request_received(const PBufPacket& raw_packet, const SensorNodePacket& packet_received)
    {
        ESP_LOGI(TAG, "Sync request from " IPSTR " received", IP2STR(raw_packet.address.ip_address));
        
        if( this->node_sync_context.state == NodeSyncState::Idle ) {
            memcpy(this->node_sync_context.mac_address, packet_received.body.sync_request.target_mac, 6);
            this->node_sync_context.state = NodeSyncState::SendingSyncRequest;
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

    void send_sensor_data(std::chrono::microseconds timestamp, const Vector3F& acc, const Vector3F& gyro, const Vector3F& mag, const BatterySensorData& battery)
    {
        SensorNodePacket& packet = this->sensor_data_packet;

        if( packet.body.sensor_data.number_of_samples == 0 ) {
            packet.body.sensor_data.set_timestamp(timestamp + this->time_offset);
        }
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].acc , acc .items.data(), sizeof(float)*3);
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].gyro, gyro.items.data(), sizeof(float)*3);
        memcpy(packet.body.sensor_data.samples[packet.body.sensor_data.number_of_samples].mag , mag .items.data(), sizeof(float)*3);
        packet.body.sensor_data.number_of_samples++;
        //ESP_LOGI(TAG, "sensor data: %0.2f, %0.2f, %0.2f", acc.x_const(), acc.y_const(), acc.z_const());

        if( packet.body.sensor_data.number_of_samples == SensorNodePacket::MAX_SENSOR_DATA_SAMPLES ) {
            packet.magic = SensorNodePacket::MAGIC;
            packet.body.sensor_data.battery_voltage = battery.voltage;
            packet.body.sensor_data.battery_charge_current    = battery.charge_current;
            packet.body.sensor_data.battery_discharge_current = battery.discharge_current;
            packet.body.sensor_data.packet_index = this->sensor_data_packet_index++;
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
                    case SensorNodePacketType::SyncRequest:
                        this->sync_request_received(result.value, packet);
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

            // Process node sync
            this->process_node_sync();
        }
        this->running_lock.release();
    }

    ~SensorNode()
    {
    }
};

#endif //SENSOR_NODE_HPP__