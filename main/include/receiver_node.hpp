#ifndef RECEIVER_NODE_HPP__
#define RECEIVER_NODE_HPP__

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <memory>
#include <vector>
#include <chrono>
#include <map>

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


struct IMUData
{
    Vector3F acc;
    Vector3F gyro;
    Vector3F mag;

    std::uint16_t max_fifo_usage;
    std:chrono::microseconds timestamp;
};

template<typename TDerived>
struct ReceiverNode : public freertos::StaticTask<8192, ReceiverNode<TDerived> >
{
    static constexpr const char* TAG = "RECEIVER";
    typedef ReceiverNode<TDerived> ReceiverNodeType;
    //static_assert(std::is_assignable<TDerived*, SelfType*>::value, "TDerived must inherit ReceivedNode<TDerived>" );

    struct ConnectedSensorNode
    {
        enum class State
        {
            NotConnected,
            Connecting,
            Syncing,
            Synced,
            WaitingMeasurementResponse,
            Measuring,
        };
        ESPNowAddress address;
        std::chrono::microseconds last_time;
        std::chrono::microseconds connection_response_time;
        State state;

        ConnectedSensorNode() : address(), last_time(0), connection_response_time(0), state(State::NotConnected) {}
        ConnectedSensorNode(const ESPNowAddress& address) : address(address), last_time(0), state(State::NotConnected) {}
    };

    enum class State
    {
        Idle,
        Connecting,
        Connected,
        RequestingMeasurement,
        Measuring,
    };

    freertos::WaitQueue<ESPNowCallbackEvent, 16> event_queue;
    freertos::Mutex running_lock;
    volatile bool running;
    volatile bool discovery_enabled;
    volatile State state;
    std::chrono::microseconds last_discovery_sent;
    std::map<ESPNowAddress, ConnectedSensorNode> sensor_nodes;

    std::chrono::microseconds measurement_start_time;
    
    freertos::WaitQueue<IMUData, 32> imu_queue;

    static ReceiverNodeType* instance;

    static std::chrono::microseconds get_timestamp() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
    }

    static void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        if( instance != nullptr )  {
            instance->event_queue.send(ESPNowCallbackEvent(ESPNowCallbackEventType::SendEvent, ESPNowAddress(mac_addr), reinterpret_cast<const std::uint8_t*>(&status), sizeof(status), get_timestamp()));
        }    
    }
    static void receive_callback(const uint8_t *mac_addr, const uint8_t *data, int len)
    {
        if( instance != nullptr )  {
            instance->event_queue.send(ESPNowCallbackEvent(ESPNowCallbackEventType::ReceiveEvent, ESPNowAddress(mac_addr), data, len, get_timestamp()));
        }    
    }

    ReceiverNode() : running(false)
                   , discovery_enabled(false)
                   , state(State::Idle)
                   , last_discovery_sent(0)
                   , measurement_start_time(0)
    {
    }

    TDerived& derived() { return static_cast<TDerived&>(*this); }

    bool is_discovery_enabled() const { return this->discovery_enabled; }

    void enable_discovery()
    {
        this->discovery_enabled = true;
    }
    void disable_discovery()
    {
        this->discovery_enabled = false;
    }
    
    template<typename Typestamp>
    void begin_measurement(Timestamp timestamp)
    {
        this->measurement_start_time = std::chrono::duration_cast<decltype(this->measurement_start_time)>(timestamp);
        this->state = State::RequestingMeasurement;
    }

    Result<void, esp_err_t> start()
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
            auto result = ReceiverNode::SelfType::start("SENSOR", 2, APP_CPU_NUM);
            if( !result ) {
                ESP_LOGE(TAG, "failed to start sensor task");
                this->running = false;
                esp_now_unregister_recv_cb();
                esp_now_unregister_send_cb();
                return failure(ESP_FAIL);
            }
        }
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

    Result<void, esp_err_t> add_node(const ESPNowAddress& address)
    {
        esp_now_peer_info_t peer;
        memset(&peer, 0, sizeof(peer));
        peer.channel = 6;
        peer.ifidx = ESP_IF_WIFI_AP;
        peer.encrypt = false;
        memcpy(peer.peer_addr, address.values, ESP_NOW_ETH_ALEN);
        auto result = esp_now_add_peer(&peer);
        if( result != ESP_OK ) {
            ESP_LOGE(TAG, "failed to add node " MACSTR " - %x", MAC2STR(address.values), result);
            return failure(result);
        }
        this->sensor_nodes.emplace(address, ConnectedSensorNode(address));
        return success();
    }

    void discovery_response_received(const ESPNowCallbackEvent& event)
    {
        ESP_LOGI(TAG, "Sensor node " MACSTR " found", MAC2STR(event.address.values));
        if( this->discovery_enabled ) {
            this->derived().on_discover(event.address);
        }
    }

    void connect_sensor_node(ConnectedSensorNode& node)
    {
        auto timestamp = get_timestamp();
        if( timestamp - node.last_time < std::chrono::seconds(1) ) {
            return;
        }
        node.last_time = timestamp;

        ESPNowPacket packet;
        packet.magic = ESPNowPacket::MAGIC;
        packet.sequence = 0;
        packet.type = ESPNowPacketType::ConnectionRequest;
        packet.set_timestamp(get_timestamp());
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = esp_now_send(node.address.values, reinterpret_cast<std::uint8_t*>(&packet), packet.total_size());
        if( result != ESP_OK ) {
            ESP_LOGE(TAG, "send connect packet - %x", result);
        }
    }

    void connection_response_received(const ESPNowCallbackEvent& event)
    {
        ESP_LOGI(TAG, "Connection response " MACSTR " received", MAC2STR(event.address.values));
        auto it = this->sensor_nodes.find(event.address);
        if( it == this->sensor_nodes.end() ) {
            return;
        }
        it->second.connection_response_time = event.timestamp;
        it->second.state = ConnectedSensorNode::State::Syncing;
    }
    
    void sync_sensor_node(ConnectedSensorNode& node)
    {
        auto timestamp = get_timestamp();
        if( timestamp - node.last_time < std::chrono::seconds(1) ) {
            return;
        }
        node.last_time = timestamp;

        ESPNowPacket packet;
        packet.magic = ESPNowPacket::MAGIC;
        packet.sequence = 0;
        packet.type = ESPNowPacketType::NotifyDelay;
        packet.set_timestamp(node.connection_response_time);
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = esp_now_send(node.address.values, reinterpret_cast<std::uint8_t*>(&packet), packet.total_size());
        if( result != ESP_OK ) {
            ESP_LOGE(TAG, "send connect packet - %x", result);
        }
    }

    void delay_response_received(const ESPNowCallbackEvent& event)
    {
        ESP_LOGI(TAG, "Delay response " MACSTR " received", MAC2STR(event.address.values));
        auto it = this->sensor_nodes.find(event.address);
        if( it == this->sensor_nodes.end() ) {
            return;
        }
        it->second.state = ConnectedSensorNode::State::Synced;
    }

    void send_measurement_request(ConnectedSensorNode& node)
    {
        ESPNowPacket packet;
        packet.magic = ESPNowPacket::MAGIC;
        packet.sequence = 0;
        packet.type = ESPNowPacketType::MeasurementRequest;
        packet.set_timestamp(get_timestamp());
        packet.reserved = 0;
        packet.body.measurement_request.target_time = this->measurement_start_time.count();
        packet.set_size_and_crc();
        auto result = esp_now_send(node.address.values, reinterpret_cast<std::uint8_t*>(&packet), packet.total_size());
        if( result != ESP_OK ) {
            ESP_LOGE(TAG, "send connect packet - %x", result);
        }
        else {
            node.state = ConnectedSensorNode::State::WaitingMeasurementResponse;
        }
    }

    void measurement_response_received(const ESPNowCallbackEvent& event)
    {
        ESP_LOGI(TAG, "Measurement response " MACSTR " received", MAC2STR(event.address.values));
        auto it = this->sensor_nodes.find(event.address);
        if( it == this->sensor_nodes.end() ) {
            return;
        }
        it->second.state = ConnectedSensorNode::State::Measuring;
    }

    void sensor_data_received(const ESPNowCallbackEvent& event, const ESPNowPacket& packet)
    {
        ESP_LOGI(TAG, "Sensor data " MACSTR " received", MAC2STR(event.address.values));
        auto it = this->sensor_nodes.find(event.address);
        if( it == this->sensor_nodes.end() ) {
            return;
        }
        
        auto number_of_samples = packet.body.sensor_data.number_of_samples;
        ESP_LOGI(TAG, "Number of samples = %d", number_of_samples);
        for(std::uint8_t i = 0; i < number_of_samples; i++ ) {
            const auto& sample = packet.body.sensor_data.samples[i];
            IMUData imu_data;
            imu_data.acc  = Vector3F(sample.acc);
            imu_data.gyro = Vector3F(sample.gyro);
            imu_data.mag  = Vector3F(sample.mag);
            this->imu_queue.send(imu_data, freertos::to_ticks(std::chrono::milliseconds(1)));
        }
    }
    void begin_connect()
    {
        this->state = State::Connecting;
    }
    bool is_connected() const
    {
        return this->state >= State::Connected;
    }

    void operator() ()
    {
        ESPNowCallbackEvent event;
        while(this->running)
        {
            auto result = this->event_queue.receive(event, freertos::to_ticks(std::chrono::milliseconds(10)));
            if( result ) {
                if( event.type == ESPNowCallbackEventType::ReceiveEvent ) {
                    auto& packet = *reinterpret_cast<ESPNowPacket*>(event.buffer);
                    auto validation_result = packet.validate();
                    if( !validation_result ) {
                        ESP_LOGE(TAG, "invalid packet - %d", static_cast<int>(validation_result.error_code));
                    }
                    else {
                        ESP_LOGI(TAG, "packet %d received from " MACSTR, static_cast<int>(packet.type), MAC2STR(event.address));
                        switch(packet.type)
                        {
                        case ESPNowPacketType::DiscoveryResponse:
                            this->discovery_response_received(event);
                            break;
                        case ESPNowPacketType::ConnectionResponse:
                            this->connection_response_received(event);
                            break;
                        case ESPNowPacketType::DelayResponse:
                            this->delay_response_received(event);
                            break;
                        case ESPNowPacketType::MeasurementResponse:
                            this->measurement_response_received(event);
                            break;
                        case ESPNowPacketType::SensorData:
                            this->sensor_data_received(event, packet);
                        default:
                            break;
                        }
                    }
                }
            }
            if( this->discovery_enabled && get_timestamp() - this->last_discovery_sent >= std::chrono::seconds(1) )  {
                this->last_discovery_sent = get_timestamp();
                ESPNowPacket packet;
                packet.magic = ESPNowPacket::MAGIC;
                packet.sequence = 0;
                packet.type = ESPNowPacketType::Discovery;
                packet.set_timestamp(this->last_discovery_sent);
                packet.reserved = 0;
                packet.set_size_and_crc();
                std::uint8_t mac_broadcast[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
                auto result = esp_now_send(mac_broadcast, reinterpret_cast<std::uint8_t*>(&packet), packet.total_size());
                if( result != ESP_OK ) {
                    ESP_LOGE(TAG, "send error - %x", result);
                }
            }
            switch(this->state) {
            case State::Connecting: {
                bool all_nodes_connected = true;
                for( auto& pair : this->sensor_nodes ) {
                    auto& node = pair.second;
                    switch(node.state) {
                    case ConnectedSensorNode::State::NotConnected:
                    case ConnectedSensorNode::State::Connecting:
                        this->connect_sensor_node(node);
                        break;
                    case ConnectedSensorNode::State::Syncing:
                        this->sync_sensor_node(node);
                        break;
                    case ConnectedSensorNode::State::Synced:
                        break;
                    default:
                        break;
                    }

                    if( node.state != ConnectedSensorNode::State::Synced ) {
                        all_nodes_connected = false;
                    }
                }

                if( all_nodes_connected ) {
                    this->state = State::Connected;
                }
                break;
            }
            case State::RequestingMeasurement: {
                bool all_nodes_responded = true;
                for( auto& pair : this->sensor_nodes ) {
                    auto& node = pair.second;
                    switch(node.state) {
                    case ConnectedSensorNode::State::Synced:
                        this->send_measurement_request(node);
                        break;
                    case ConnectedSensorNode::State::WaitingMeasurementResponse:
                        break;
                    case ConnectedSensorNode::State::Measuring:
                        break;
                    default:
                        break;
                    }

                    if( node.state != ConnectedSensorNode::State::Measuring ) {
                        all_nodes_responded = false;
                    }
                }

                if( all_nodes_responded ) {
                    this->state = State::Measuring;
                }
                break;
            }
            default:
                break;
            }
        }
        this->running_lock.release();
    }

    ~ReceiverNode()
    {
        if( instance == this ) {
            esp_now_unregister_send_cb();            
            esp_now_unregister_recv_cb();
            instance = nullptr;
        }
    }
};


#endif //RECEIVER_NODE_HPP__