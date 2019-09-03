#ifndef RECEIVER_NODE_HPP__
#define RECEIVER_NODE_HPP__

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <memory>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <numeric>

#include <esp_err.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event_legacy.h>
#include <dhcpserver/dhcpserver.h>

#include <freertos_util.hpp>
#include <result.hpp>
#include <timer.hpp>
#include <ringbuffer.hpp>
#include <vector3.hpp>

#include "espnow_packet.hpp"
#include <sensor_communication.hpp>

struct IMUData
{
    Vector3F acc;
    Vector3F gyro;
    Vector3F mag;
    std::chrono::microseconds timestamp;
};

struct SensorNodeIMUData
{
    SensorNodeAddress address;
    IMUData data;
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
        SensorNodeAddress address;
        std::chrono::microseconds last_time;
        std::chrono::microseconds connection_response_time;
        State state;
        freertos::WaitQueue<IMUData, 32> imu_queue;
        freertos::InterlockedValue<std::uint64_t> total_number_of_samples_received;

        ConnectedSensorNode() : address(), last_time(0), connection_response_time(0), state(State::NotConnected) {}

        std::uint64_t get_total_number_of_samples_received()
        {
            return this->total_number_of_samples_received;
        }
    };

    static constexpr std::size_t MAX_SENSOR_NODES = 3;
    std::array<ConnectedSensorNode, MAX_SENSOR_NODES> sensor_nodes_body;

    enum class State
    {
        Idle,
        Connecting,
        Connected,
        RequestingMeasurement,
        Measuring,
    };

    freertos::Mutex running_lock;
    bool running;
    bool discovery_enabled;
    State state;
    std::chrono::microseconds last_discovery_sent;
    std::unordered_map<SensorNodeAddress, ConnectedSensorNode&> sensor_nodes;
    std::uint16_t port;
    SensorCommunication communication;

    std::chrono::microseconds measurement_start_time;

    // Communication Statistics 
    RingBuffer<std::chrono::microseconds, 16, true> send_complete_elapsed;
    std::chrono::microseconds average_send_complete_elapsed;
    std::chrono::microseconds max_send_complete_elapsed;

    static std::chrono::microseconds get_timestamp() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
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
    
    template<typename Timestamp>
    void begin_measurement(Timestamp timestamp)
    {
        this->measurement_start_time = std::chrono::duration_cast<decltype(this->measurement_start_time)>(timestamp);
        this->state = State::RequestingMeasurement;
    }

    Result<void, esp_err_t> start(std::uint16_t port)
    {
        this->port = port;
        RESULT_TRY(this->communication.start(IP4_ADDR_BROADCAST, this->port));
        {
            this->running = true;
            auto result = ReceiverNode::SelfType::start("SENSOR", 3, APP_CPU_NUM);
            if( !result ) {
                ESP_LOGE(TAG, "failed to start sensor task");
                this->running = false;
            }
        }
        
        this->running_lock.lock();
        return success();
    }

    void stop()
    {
        this->running = false;
    }

    Result<void, esp_err_t> add_node(const SensorNodeAddress& address)
    {
        if( this->sensor_nodes.size() >= this->sensor_nodes_body.size() ) {
            ESP_LOGE(TAG, "failed to add node %08x - too many nodes.", address.ip_address.u_addr.ip4.addr);            
            return failure(ESP_FAIL);
        }
        auto& sensor_node = this->sensor_nodes_body[this->sensor_nodes.size()];
        sensor_node.address = address;
        this->sensor_nodes.emplace(address, sensor_node);
        return success();
    }

    Result<void, esp_err_t> broadcast_packet(const SensorNodePacket& packet) 
    {
        auto start_time = get_timestamp();
        auto result = this->communication.send(PBufPacket(packet));
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

    Result<void, esp_err_t> send_packet(ConnectedSensorNode& sensor_node, const SensorNodePacket& packet) 
    {
        auto start_time = get_timestamp();
        auto result = this->communication.send(PBufPacket(packet), sensor_node.address);
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

    void discovery_response_received(const PBufPacket& packet)
    {
        // ESP_LOGI(TAG, "Sensor node " IPSTR " found", IP2STR(packet.address.ip_address));
        // if( this->discovery_enabled ) {
        //     this->derived().on_discover(packet.address);
        // }
    }

    void handle_system_event(const system_event_t& event)
    {
        switch(event.event_id) {
            case SYSTEM_EVENT_AP_STAIPASSIGNED: {
                this->derived().on_discover(SensorNodeAddress(event.event_info.ap_staipassigned.ip));
                break;
            }
            default:
                break;
        }
    }

    void connect_sensor_node(ConnectedSensorNode& node)
    {
        auto timestamp = get_timestamp();
        if( timestamp - node.last_time < std::chrono::seconds(1) ) {
            return;
        }
        node.last_time = timestamp;

        SensorNodePacket packet;
        packet.magic = SensorNodePacket::MAGIC;
        packet.type = SensorNodePacketType::ConnectionRequest;
        packet.set_timestamp(get_timestamp());
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = this->send_packet(node, packet);
        if( !result ) {
            ESP_LOGE(TAG, "send connect packet - %x", result.error_code);
        }
    }

    void connection_response_received(const PBufPacket& packet)
    {
        ESP_LOGI(TAG, "Connection response " IPSTR " received", IP2STR(packet.address.ip_address));
        auto it = this->sensor_nodes.find(packet.address);
        if( it == this->sensor_nodes.end() ) {
            ESP_LOGE(TAG, "Connection response from unknown sensor node " IPSTR, IP2STR(packet.address.ip_address));
            return;
        }
        it->second.connection_response_time = packet.timestamp;
        it->second.state = ConnectedSensorNode::State::Syncing;
    }
    
    void sync_sensor_node(ConnectedSensorNode& node)
    {
        auto timestamp = get_timestamp();
        if( timestamp - node.last_time < std::chrono::seconds(1) ) {
            return;
        }
        node.last_time = timestamp;

        SensorNodePacket packet;
        packet.magic = SensorNodePacket::MAGIC;
        packet.type = SensorNodePacketType::NotifyDelay;
        packet.set_timestamp(node.connection_response_time);
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = this->send_packet(node, packet);
        if( !result ) {
            ESP_LOGE(TAG, "send notify delay packet - %x", result.error_code);
        }
    }

    void delay_response_received(const PBufPacket& packet)
    {
        ESP_LOGI(TAG, "Delay response " IPSTR " received", IP2STR(packet.address.ip_address));
        auto it = this->sensor_nodes.find(packet.address);
        if( it == this->sensor_nodes.end() ) {
            ESP_LOGE(TAG, "Delay response from unknown sensor node " IPSTR, IP2STR(packet.address.ip_address));
            return;
        }
        it->second.state = ConnectedSensorNode::State::Synced;
    }

    void send_measurement_request(ConnectedSensorNode& node)
    {
        SensorNodePacket packet;
        packet.magic = SensorNodePacket::MAGIC;
        packet.type = SensorNodePacketType::MeasurementRequest;
        packet.set_timestamp(get_timestamp());
        packet.reserved = 0;
        packet.body.measurement_request.target_time = this->measurement_start_time.count();
        packet.set_size_and_crc();
        auto result = this->send_packet(node, packet);
        if( !result ) {
            ESP_LOGE(TAG, "send measurement request packet - %x", result.error_code);
        }
        else {
            node.state = ConnectedSensorNode::State::WaitingMeasurementResponse;
        }
    }

    void measurement_response_received(const PBufPacket& packet)
    {
        ESP_LOGI(TAG, "Measurement response " IPSTR " received", IP2STR(packet.address.ip_address));
        auto it = this->sensor_nodes.find(packet.address);
        if( it == this->sensor_nodes.end() ) {
            return;
        }
        it->second.state = ConnectedSensorNode::State::Measuring;
    }

    void sensor_data_received(const PBufPacket& raw_packet, const SensorNodePacket& packet)
    {
        ESP_LOGV(TAG, "Sensor data " IPSTR " received", IP2STR(raw_packet.address.ip_address));
        auto it = this->sensor_nodes.find(raw_packet.address);
        if( it == this->sensor_nodes.end() ) {
            return;
        }
        
        auto number_of_samples = packet.body.sensor_data.number_of_samples;
        ESP_LOGV(TAG, "Number of samples = %d", number_of_samples);
        for(std::uint8_t i = 0; i < number_of_samples; i++ ) {
            const auto& sample = packet.body.sensor_data.samples[i];
            IMUData imu_data;
            imu_data.acc  = Vector3F(sample.acc);
            imu_data.gyro = Vector3F(sample.gyro);
            imu_data.mag  = Vector3F(sample.mag);
            imu_data.timestamp = std::chrono::microseconds(packet.body.sensor_data.timestamp);
            it->second.imu_queue.send(imu_data, freertos::to_ticks(std::chrono::milliseconds(1)));
        }
        it->second.total_number_of_samples_received += number_of_samples;
    }
    void begin_connect()
    {
        this->state = State::Connecting;
    }
    bool is_connected() const
    {
        return this->state >= State::Connected;
    }
    
    Result<SensorNodeIMUData, bool> get_sensor_data()
    {
        for(auto& pair : this->sensor_nodes ) {
            IMUData data;
            if(pair.second.imu_queue.receive(data, freertos::Ticks::zero())) {
                SensorNodeIMUData node_data = {
                    pair.first,
                    data,
                };
                return success<SensorNodeIMUData>(node_data);
            }
        }
        return failure(false);
    }

    void operator() ()
    {
        while(this->running)
        {
            while(auto result = this->communication.receive(freertos::to_ticks(std::chrono::milliseconds(10)))) {
                auto& packet = static_cast<SensorNodePacket&>(result.value);
                auto validation_result = packet.validate();
                if( !validation_result ) {
                    ESP_LOGE(TAG, "invalid packet - %d", static_cast<int>(validation_result.error_code));
                }
                else {
                    ESP_LOGV(TAG, "packet %d received from " IPSTR, static_cast<int>(packet.type), IP2STR(result.value.address.ip_address));
                    switch(packet.type)
                    {
                    case SensorNodePacketType::DiscoveryResponse:
                        this->discovery_response_received(result.value);
                        break;
                    case SensorNodePacketType::ConnectionResponse:
                        this->connection_response_received(result.value);
                        break;
                    case SensorNodePacketType::DelayResponse:
                        this->delay_response_received(result.value);
                        break;
                    case SensorNodePacketType::MeasurementResponse:
                        this->measurement_response_received(result.value);
                        break;
                    case SensorNodePacketType::SensorData:
                        this->sensor_data_received(result.value, packet);
                    default:
                        break;
                    }
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
    }
};


#endif //RECEIVER_NODE_HPP__