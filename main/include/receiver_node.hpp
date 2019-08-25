#ifndef RECEIVER_NODE_HPP__
#define RECEIVER_NODE_HPP__

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

#include "espnow_packet.hpp"

struct ReceiverNode : public freertos::StaticTask<8192, ReceiverNode>
{
    static constexpr const char* TAG = "RECEIVER";
    static constexpr std::size_t MAX_SENSOR_NODES = 10;
    ESPNowAddress sensor_nodes[MAX_SENSOR_NODES];
    std::size_t sensor_node_count;

    freertos::WaitQueue<ESPNowCallbackEvent, 16> event_queue;
    freertos::Mutex running_lock;
    volatile bool running;
    volatile bool connected;

    static ReceiverNode* instance;

    static std::uint64_t get_timestamp() 
    {
        return std::chrono::steady_clock::now().time_since_epoch().count();
    }

    static void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        if( instance != nullptr )  {
            instance->event_queue.send(ESPNowCallbackEvent(ESPNowCallbackEventType::SendEvent, mac_addr, reinterpret_cast<const std::uint8_t*>(&status), sizeof(status)));
        }    
    }
    static void receive_callback(const uint8_t *mac_addr, const uint8_t *data, int len)
    {
        if( instance != nullptr )  {
            instance->event_queue.send(ESPNowCallbackEvent(ESPNowCallbackEventType::ReceiveEvent, mac_addr, data, len));
        }    
    }

    ReceiverNode() : sensor_node_count(0), running(false), connected(false) {}

    bool is_connected() const { return this->connected; }

    bool add_sensor_node(const ESPNowAddress& sensor_node)
    {
        if( this->sensor_node_count == MAX_SENSOR_NODES ) {
            return DNS_FALLBACK_SERVER_INDEX;
        }
        for(std::size_t i = 0; i < this->sensor_node_count; i++ ) {
            if( this->sensor_nodes[i] == sensor_node ) {
                return false;
            }
        }
        this->sensor_nodes[this->sensor_node_count] = sensor_node;
        this->sensor_node_count++;
        return true;
    }
    std::size_t get_sensor_node_count() const { return this->sensor_node_count; }
    ESPNowAddress get_sensor_node(std::size_t index) const
    {
        if( index >= this->sensor_node_count ) {
            return ESPNowAddress();
        }
        return this->sensor_nodes[index];
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
            auto result = ReceiverNode::SelfType::start("SENSOR", 3, APP_CPU_NUM);
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

    void discovery_response_received(const ESPNowCallbackEvent& event)
    {
        ESP_LOGI(TAG, "Sensor node " MACSTR " found", MAC2STR(event.address));
        this->add_sensor_node(event.address);
    }

    void operator() ()
    {

        ESPNowCallbackEvent event;
        while(this->running)
        {
            auto result = this->event_queue.receive(event, 0);
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
                        default:
                            break;
                        }
                    }
                }
            }
            else {
                ESPNowPacket packet;
                packet.magic = ESPNowPacket::MAGIC;
                packet.sequence = 0;
                packet.type = ESPNowPacketType::Discovery;
                packet.timestamp = get_timestamp();
                packet.reserved = 0;
                packet.set_size_and_crc();
                std::uint8_t mac_broadcast[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
                auto result = esp_now_send(mac_broadcast, reinterpret_cast<std::uint8_t*>(&packet), packet.total_size());
                if( result != ESP_OK ) {
                    ESP_LOGE(TAG, "send error - %x", result);
                }
                else {
                    freertos::Task::delay_ms(100);
                }
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

ReceiverNode* ReceiverNode::instance = nullptr;

#endif //RECEIVER_NODE_HPP__