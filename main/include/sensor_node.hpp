#ifndef SENSOR_NODE_HPP__
#define SENSOR_NODE_HPP__

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

struct SensorNode : public freertos::StaticTask<8192, SensorNode>
{
    static constexpr const char* TAG = "SENSOR";

    freertos::WaitQueue<ESPNowCallbackEvent, 16> event_queue;
    freertos::Mutex running_lock;
    volatile bool running;
    volatile bool connected;

    static SensorNode* instance;

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

    SensorNode() : running(false), connected(false) {}

    bool is_connected() const { return this->connected; }

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
            auto result = SensorNode::SelfType::start("SENSOR", 3, APP_CPU_NUM);
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

    void discovery_received(const ESPNowCallbackEvent& event)
    {
        // Add peer list
        if( !esp_now_is_peer_exist(event.address) ) {
            esp_now_peer_info_t peer;
            memset(&peer, 0, sizeof(peer));
            peer.channel = 6;
            peer.ifidx = ESP_IF_WIFI_AP;
            peer.encrypt = false;
            memcpy(peer.peer_addr, event.address, ESP_NOW_ETH_ALEN);
            auto result = esp_now_add_peer(&peer);
            if( result != ESP_OK ) {
                ESP_LOGE(TAG, "failed to add peer " MACSTR " - %x", MAC2STR(event.address), result);
            }
        }
        ESPNowPacket packet;
        packet.magic = ESPNowPacket::MAGIC;
        packet.sequence = 0;
        packet.type = ESPNowPacketType::DiscoveryResponse;
        packet.timestamp = get_timestamp();
        packet.reserved = 0;
        packet.set_size_and_crc();
        auto result = esp_now_send(event.address, reinterpret_cast<std::uint8_t*>(&packet), packet.total_size());
        if( result != ESP_OK ) {
            ESP_LOGE(TAG, "send error - %x", result);
        }
        else {
            ESP_LOGI(TAG, "send ok");
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
                            this->discovery_received(event);
                            break;
                        default:
                            break;
                        }
                    }
                }
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