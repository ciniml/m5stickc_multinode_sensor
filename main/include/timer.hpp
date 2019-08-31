#ifndef TIMER_HPP__
#define TIMER_HPP__

#include <cstdint>
#include <chrono>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>

template<typename TimerType>
struct Timer
{
    typedef Timer<TimerType> SelfType;

    esp_timer_handle_t handle;

    static void timer_proc_wrapper(void* parameters)
    {
        auto this_ = static_cast<TimerType*>(reinterpret_cast<SelfType*>(parameters));
        (*this_)();
    }
    Timer() : handle(nullptr) {}

    Result<bool, esp_err_t> start(std::chrono::microseconds period) noexcept
    {
        esp_timer_create_args_t args;
        args.name = "Timer";
        args.callback = &Timer::timer_proc_wrapper;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;

        auto result = esp_timer_create(&args, &this->handle);
        if( result != ESP_OK ) {
            return failure(result);
        }
        
        result = esp_timer_start_periodic(this->handle, period.count());
        if( result != ESP_OK ) {
            return failure(result);
        }
        return success(true);
    }
};


#endif //TIMER_HPP__