#ifndef TIMER_HPP__
#define TIMER_HPP__

#include <cstdint>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>

template<typename TimerFunc>
struct Timer
{
    typedef Timer<TimerFunc> SelfType;

    esp_timer_handle_t handle;
    TimerFunc timer_func;

    static void timer_proc_wrapper(void* parameters)
    {
        auto this_ = reinterpret_cast<SelfType*>(parameters);
        this_->timer_func();
    }
    Timer(TimerFunc&& timer_func) : timer_func(std::forward<TimerFunc>(timer_func)) {}

    Result<bool, esp_err_t> start(uint64_t period_us) noexcept
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
        
        result = esp_timer_start_periodic(this->handle, period_us);
        if( result != ESP_OK ) {
            return failure(result);
        }
        return success(true);
    }
};


#endif //TIMER_HPP__