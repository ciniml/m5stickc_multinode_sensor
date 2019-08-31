#ifndef BUTTON_HPP__
#define BUTTON_HPP__

#include <cstdint>
#include <type_traits>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <driver/gpio.h>

#include "freertos_util.hpp"
#include "result.hpp"
#include "timer.hpp"

class Button
{
public:
    static constexpr const char* TAG = "BUTTON";

    enum class Position : std::uint8_t
    {
        A = 0,
        B = 1,
    };

    enum class EventType : std::uint8_t
    {
        Pushed,
        Released,
        Dummy,
    };

    struct Event
    {
        EventType type;
        Position position;
    };

    static constexpr std::uint8_t GPIO_BUTTON_A = 37;
    static constexpr std::uint8_t GPIO_BUTTON_B = 39;
    static constexpr std::uint64_t gpio_button_bit(std::uint8_t button) { return static_cast<std::uint64_t>(1) << button; }
    static constexpr std::uint64_t gpio_button_bit(Position button) { return static_cast<std::uint64_t>(1) << static_cast<std::uint8_t>(button); }

private:
    struct UpdateTimer : public Timer<UpdateTimer>
    {
        volatile std::uint8_t button_state;
        freertos::WaitQueue<Event, 16> queue;

        UpdateTimer() : button_state(0) {}

        void operator() ()
        {
            std::uint8_t new_state = 0;
            new_state |= gpio_get_level(static_cast<gpio_num_t>(GPIO_BUTTON_A)) << static_cast<std::uint8_t>(Position::A);
            new_state |= gpio_get_level(static_cast<gpio_num_t>(GPIO_BUTTON_B)) << static_cast<std::uint8_t>(Position::B);

            auto changed = this->button_state ^ new_state;
            for( std::uint8_t button = 0, button_bit = 1; button < 2; button++, button_bit <<= 1 ) {
                if( (changed & button_bit) != 0 ) {
                    Event event;
                    event.position = static_cast<Position>(button);
                    event.type = (new_state & button_bit) == 0 ? EventType::Pushed : EventType::Released;
                    this->queue.send(event, freertos::Ticks::zero());
                }
            }

            this->button_state = new_state;
        }
    };

    UpdateTimer update_timer;

public:
    Result<void, esp_err_t> initialize()
    {
        gpio_config_t config;
        config.intr_type = gpio_int_type_t::GPIO_INTR_DISABLE;
        config.mode = gpio_mode_t::GPIO_MODE_INPUT;
        config.pin_bit_mask = gpio_button_bit(GPIO_BUTTON_A) | gpio_button_bit(GPIO_BUTTON_B);
        config.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
        config.pull_up_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
        auto result = gpio_config(&config);
        if( result != ESP_OK ) {
            ESP_LOGE(TAG, "init failed - %x", result);
            return failure(result);
        }

        RESULT_TRY(this->update_timer.start(std::chrono::microseconds(1000)));
        
        return success();
    }

    void clear_events()
    {
        this->update_timer.queue.reset();
    }

    Result<Event, bool> read_event(freertos::Ticks wait_ticks = freertos::Ticks(0))
    {
        Event event;
        auto result = this->update_timer.queue.receive(event, wait_ticks);
        if( !result ) {
            return failure(false);
        }

        return success<Event>(event);
    }

    bool is_pressed(Position position) const 
    {
        return (this->update_timer.button_state & gpio_button_bit(position)) == 0;
    }
};

#endif //BUTTON_HPP__