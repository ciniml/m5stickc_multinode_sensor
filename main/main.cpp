#include <cstdint>
#include <type_traits>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <driver/i2c.h>

#include <M5Display.h>
#include <AXP192.h>

#include "freertos_util.hpp"


// Result -------------------------

template<typename T>
struct Success
{
    T value;
    
    Success(const T& value) : value(value) {}
    Success(T&& value) : value(std::forward<T>(value)) {}
};

template<> struct Success<void>{};


template<typename E>
struct Failure
{
    E error_code;
    
    Failure(const E& error_code) : error_code(error_code) {}
};

template<typename T, typename E, E SuccessCode = static_cast<E>(0)>
struct Result
{
    typedef Success<T> SuccessType;
    typedef Failure<E> FailureType;

    T value;
    bool is_success;
    E error_code;

    Result() = default;
    Result(const SuccessType& success) : value(success.value), is_success(true), error_code() {}
    Result(SuccessType&& success) : value(std::move(success.value)), is_success(true), error_code() {}
    Result(const FailureType& failure) : value(), is_success(false), error_code(failure.error_code) {}

    operator bool() const { return this->is_success; }

    template<typename Func>
    typename std::result_of<Func(T&&)>::type then(Func&& func)
    {
        if( this->is_success ) {
            return func(std::move(this->value));
        }
        else {
            return typename std::result_of<Func(T&&)>::type::FailureType(this->error_code);
        }
    }

};

template<typename E, E SuccessCode>
struct Result<void, E, SuccessCode>
{
    typedef Success<void> SuccessType;
    typedef Failure<E> FailureType;

    bool is_success;
    E error_code;

    Result() : is_success(true), error_code(SuccessCode) {};
    Result(const SuccessType&) : is_success(true), error_code(SuccessCode) {};
    Result(const FailureType& failure) : is_success(false), error_code(failure.error_code) {}
    Result(E error_code) : is_success(error_code == SuccessCode), error_code(error_code) {}
    operator bool() const { return this->is_success; }

    template<typename Func>
    typename std::result_of<Func()>::type then(Func&& func)
    {
        if( this->is_success ) {
            return func();
        }
        else {
            return typename std::result_of<Func()>::type::FailureType(this->error_code);
        }
    }
};

static Success<void> success(void) { return Success<void>(); }
template<typename T> Success<T> success(const T& value) { return Success<T>(value); }
template<typename T> Success<T> success(T&& value) { return Success<T>(std::forward<T>(value)); }
template<typename T, typename E, E SuccessCode> Success<T> success(Result<T, E, SuccessCode>&& success_result) { return Success<T>(std::move(success_result.value)); }
template<typename E> Failure<E> failure(const E& error_code) { return Failure<E>(error_code); }
template<typename T, typename E, E SuccessCode> Failure<E> failure(const Result<T, E, SuccessCode>& failure_result) { return Failure<E>(failure_result.error_code); }

// RingBuffer

template<typename TItem, std::size_t Capacity>
class RingBuffer
{
private:
    typedef RingBuffer<TItem, Capacity> SelfType;
    static constexpr std::size_t Mask = Capacity - 1;
    static constexpr std::size_t AdvanceMask = (Capacity<<1) - 1;
    static_assert((Mask & Capacity) == 0, "Capacity must be power of two.");

    std::array<TItem, Capacity> buffer;
    std::size_t read_index;
    std::size_t write_index;

public:
    RingBuffer() : read_index(0), write_index(0) {}

    bool is_empty() const { return this->read_index == this->write_index; }
    bool is_full() const { return (this->read_index ^ this->write_index) == Capacity; }

    bool queue(const TItem& item)
    {
        if( this->is_full() ) {
            return false;
        }
        this->buffer[this->write_index & Mask] = item;
        this->write_index = (this->write_index + 1) & AdvanceMask;
        return true;
    }

    Result<TItem, bool> dequeue()
    {
        if( this->is_empty() ) {
            return failure(false);
        }

        auto value = success<TItem>(this->buffer[this->read_index & Mask]);
        this->read_index = (this->read_index + 1) & AdvanceMask;
        return std::move(value);
    }

    void reset() 
    {
        while(!this->is_empty()) {
            this->dequeue();
        }
    }
};

static M5Display lcd;

static freertos::Mutex i2c_mutex;


struct Task
{
    TaskHandle_t handle;
    bool own_handle;
    
    Task(TaskHandle_t handle) : handle(handle), own_handle(false) {}
    Task(TaskHandle_t handle, bool own_handle) : handle(handle), own_handle(own_handle) {}
    Task(const Task& task) : handle(task.handle), own_handle(false) {}
    Task(Task&& task) : handle(task.handle), own_handle(task.own_handle) 
    {
        task.own_handle = false;
    }

    ~Task() {
        if( this->own_handle && this->handle != nullptr ) {
            vTaskDelete(this->handle);
            this->handle = nullptr;
            this->own_handle = false;
        }
    }

    struct NotifyFromISRResut
    {
        bool success;
        bool higher_priority_task_woken;
    };

    operator TaskHandle_t() const { return this->handle; }
    operator bool() const { return this->handle != nullptr; }

    bool notify(std::uint32_t value, eNotifyAction action)
    {
        return xTaskNotify(this->handle, value, action) == pdPASS;
    }
    Result<bool, bool> notify_from_isr(std::uint32_t value, eNotifyAction action)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if( xTaskNotifyFromISR(this->handle, value, action, &higher_priority_task_woken) == pdTRUE ) {
            return success(higher_priority_task_woken == pdTRUE);
        }
        else {
            return failure(true);
        }
    }

    static Task current()
    {
        return Task(xTaskGetCurrentTaskHandle());
    }

    static Result<std::uint32_t, bool> notify_wait(std::uint32_t bits_to_clear_on_entry, std::uint32_t bits_to_clear_on_exit, TickType_t ticks_to_wait)
    {
        std::uint32_t notification_value = 0;
        auto result = xTaskNotifyWait(bits_to_clear_on_entry, bits_to_clear_on_exit, &notification_value, ticks_to_wait);
        if( result != pdPASS ) {
            return failure(true);
        }
        else {
            return success<std::uint32_t>(notification_value);
        }
    }
};


template<std::uint32_t StackSize, typename TaskFunc>
struct StaticTask 
{
    typedef StaticTask<StackSize, TaskFunc> SelfType;

    StackType_t stack[StackSize];
    TaskHandle_t handle;
    StaticTask_t context;
    TaskFunc task_func;

    static void task_proc_wrapper(void* parameters) 
    {
        auto this_ = reinterpret_cast<SelfType*>(parameters);
        this_->task_func();
    }

    StaticTask(TaskFunc&& task_func) : handle(nullptr), task_func(std::forward<TaskFunc>(task_func)) {}

    operator bool() const { return this->handle != nullptr; }

    bool start(const char* name, UBaseType_t priority, BaseType_t core_id) 
    {
        this->handle = xTaskCreateStaticPinnedToCore(&StaticTask::task_proc_wrapper, name, StackSize, this, priority, this->stack, &this->context, core_id);
        return this->handle != nullptr;
    }

    Task task() const { return Task(this->handle); }
};

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


class I2CCommandLink
{
private:
    i2c_cmd_handle_t handle;
public:
    I2CCommandLink() : handle(nullptr)
    {
        this->handle = i2c_cmd_link_create();
    }
    I2CCommandLink(const I2CCommandLink&) = delete;
    I2CCommandLink(I2CCommandLink&& rv) : handle(rv.handle) { rv.handle = nullptr; }
    
    bool valid() const { return this->handle != nullptr; }
    operator bool() const { return this->valid(); }
    ~I2CCommandLink()
    {
        if( this->valid() ) {
            i2c_cmd_link_delete(this->handle);
            this->handle = nullptr;
        }
    }

    operator i2c_cmd_handle_t() const { return this->handle; }

    Result<void, esp_err_t> start() {
        auto result = i2c_master_start(this->handle);
        if( result != ESP_OK ) {
            return failure(result);
        }
        return success();
    }
    Result<void, esp_err_t> stop() {
        auto result = i2c_master_stop(this->handle);
        if( result != ESP_OK ) {
            return failure(result);
        }
        return success();
    }
    Result<void, esp_err_t> write_byte(std::uint8_t data, bool ack_en) {
        auto result = i2c_master_write_byte(this->handle, data, ack_en);
        if( result != ESP_OK ) {
            return failure(result);
        }
        return success();
    }
    Result<void, esp_err_t> read_byte(std::uint8_t& buffer, i2c_ack_type_t ack_type)
    {
        auto result = i2c_master_read_byte(this->handle, &buffer, ack_type);
        if( result != ESP_OK ) {
            return failure(result);
        }
        return success();
    }
    Result<void, esp_err_t> write(const std::uint8_t* data, std::size_t data_length, bool ack_en)
    {
        auto result = i2c_master_write(this->handle, const_cast<std::uint8_t*>(data), data_length, ack_en);
        if( result != ESP_OK ) {
            return failure(result);
        }
        return success();
    }
    Result<void, esp_err_t> read(std::uint8_t* buffer, std::size_t buffer_length, i2c_ack_type_t ack_type)
    {
        auto result = i2c_master_read(this->handle, buffer, buffer_length, ack_type);
        if( result != ESP_OK ) {
            return failure(result);
        }
        return success();
    }

    Result<void, esp_err_t> write_register(std::uint8_t device_address, std::uint8_t register_address, const std::uint8_t* data, std::size_t data_length)
    {
        auto result = this->start();
        if( !result.is_success ) return result;
        result = this->write_byte((device_address << 1) | 0, true); // SLA+W
        if( !result.is_success ) return result;
        result = this->write_byte(register_address, true);
        if( !result.is_success ) return result;
        result = this->write(data, data_length, true);
        if( !result.is_success ) return result;
        result = this->stop();
        if( !result.is_success ) return result;   
        return success();
    }

    Result<void, esp_err_t> read_register(std::uint8_t device_address, std::uint8_t register_address, std::uint8_t* buffer, std::size_t buffer_length, i2c_ack_type_t ack_type)
    {
        auto result = this->start();
        if( !result.is_success ) return result;
        result = this->write_byte((device_address << 1) | 0, true); // SLA+W
        if( !result.is_success ) return result;
        result = this->write_byte(register_address, true);
        if( !result.is_success ) return result;
        result = this->start();        // Repeated start
        if( !result.is_success ) return result;
        result = this->write_byte((device_address << 1) | 1, true); // SLA+R
        if( !result.is_success ) return result;
        result = this->read(buffer, buffer_length, ack_type);
        if( !result.is_success ) return result;
        result = this->stop();
        if( !result.is_success ) return result;   
        return success();
    }
};

class I2CMaster
{
private:
    i2c_port_t port;
    bool initialized;
    freertos::Mutex mutex;

public:
    I2CMaster(i2c_port_t port) : port(port), initialized(false) {}

    Result<void, esp_err_t> initialize(const i2c_config_t& config) 
    {
        if( this->initialized ) {
            return failure(ESP_FAIL);
        }

        auto guard = freertos::lock(this->mutex);

        if( this->initialized ) {
            return failure(ESP_FAIL);
        }
        if( config.mode != I2C_MODE_MASTER ) {
            return failure(ESP_ERR_INVALID_ARG);
        }

        auto result = i2c_driver_install(this->port, I2C_MODE_MASTER, 0, 0, 0);
        if( result != ESP_OK ) {
            return failure(result);
        }

        result = i2c_param_config(this->port, &config);
        if( result != ESP_OK ) {
            i2c_driver_delete(this->port);
            return failure(result);
        }

        this->initialized = true;
        return success();
    }

    ~I2CMaster()
    {
        auto guard = freertos::lock(this->mutex);

        if( this->initialized ) {
            i2c_driver_delete(this->port);
            this->initialized = false;
        }
    }

    i2c_port_t get_port() const { return this->port; }

    Result<void, esp_err_t> execute(const I2CCommandLink& commands, TickType_t ticks_to_wait)
    {
        auto guard = freertos::lock(this->mutex, ticks_to_wait);
        if( !guard ) {
            return failure(ESP_ERR_TIMEOUT);
        }

        auto result = i2c_master_cmd_begin(this->port, commands, ticks_to_wait);
        if( result != ESP_OK ) {
            return failure(result);
        }

        return success();
    }

    Result<std::uint8_t, esp_err_t> read_single_register(std::uint8_t device_address, std::uint8_t register_address, TickType_t wait_ticks)
    {
        I2CCommandLink commands;
        if( !commands ) {
            return failure(ESP_ERR_NO_MEM);
        }
        std::uint8_t buffer;
        auto result = commands.read_register(device_address, register_address, &buffer, 1, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
        if( !result ) return failure(result);

        result = this->execute(commands, wait_ticks);
        if( !result ) return failure(result);

        return success<std::uint8_t>(buffer);
    }
    Result<void, esp_err_t> write_single_register(std::uint8_t device_address, std::uint8_t register_address, std::uint8_t value, TickType_t wait_ticks)
    {
        I2CCommandLink commands;
        if( !commands ) {
            return failure(ESP_ERR_NO_MEM);
        }
        auto result = commands.write_register(device_address, register_address, &value, 1);
        if( !result ) return failure(result);

        result = this->execute(commands, wait_ticks);
        if( !result ) return failure(result);

        return success();
    }
};


template<typename TElement>
struct Vector3
{
    std::array<TElement, 3> items;

    Vector3() = default;
    Vector3(const TElement p) : items(p) {}
    Vector3(TElement x, TElement y, TElement z) : items({x, y, z}) {}

    TElement& x() { return this->items[0]; }
    TElement& y() { return this->items[1]; }
    TElement& z() { return this->items[2]; }

    TElement* x_pointer() { return this->items.data() + 0; }
    TElement* y_pointer() { return this->items.data() + 1; }
    TElement* z_pointer() { return this->items.data() + 2; }

    TElement x_const() const { return this->items.at(0); }
    TElement y_const() const { return this->items.at(1); }
    TElement z_const() const { return this->items.at(2); }

    Vector3<TElement>& operator+=(const Vector3& rhs) {
        for(std::size_t i = 0; i < this->items.size(); i++) { this->items[i] += rhs.items[i]; }
        return *this;
    }
    Vector3<TElement>& operator-=(const Vector3& rhs) {
        for(std::size_t i = 0; i < this->items.size(); i++) { this->items[i] -= rhs.items[i]; }
        return *this;
    }
    Vector3<TElement>& operator*=(TElement rhs) {
        for(std::size_t i = 0; i < this->items.size(); i++) { this->items[i] *= rhs; }
        return *this;
    }
    Vector3<TElement>& operator/=(TElement rhs) {
        for(std::size_t i = 0; i < this->items.size(); i++) { this->items[i] /= rhs; }
        return *this;
    }
    
    template<typename OtherElement>
    Vector3<OtherElement> operator*(OtherElement rhs) {
        return Vector3<OtherElement>(this->items[0] * rhs,
                                     this->items[1] * rhs,
                                     this->items[2] * rhs);
    }
    template<typename OtherElement>
    Vector3<OtherElement> operator/(OtherElement rhs) {
        return Vector3<OtherElement>(this->items[0] / rhs,
                                     this->items[1] / rhs,
                                     this->items[2] / rhs);
    }
};

typedef Vector3<float> Vector3F;
typedef Vector3<int16_t> Vector3I16;

class IMU
{
private:
    I2CMaster& i2c;
    std::uint8_t address;
    
    Vector3I16 acceleration_offset;
    
    RingBuffer<Vector3I16, 64> acceleration_queue;
    RingBuffer<Vector3I16, 128> angular_velocity_queue;

    static constexpr std::uint8_t SH200Q_REG_CHIP_ID = 0x30;
    static constexpr std::uint8_t SH200Q_REG_ACC_CONFIG = 0x0E;
    static constexpr std::uint8_t SH200Q_REG_GYRO_CONFIG = 0x0F;
    static constexpr std::uint8_t SH200Q_REG_GYRO_CONFIG_1 = 0x11;
    static constexpr std::uint8_t SH200Q_REG_FIFO_CONFIG = 0x12;
    static constexpr std::uint8_t SH200Q_REG_ACC_DATA_FORMAT = 0x16;
    static constexpr std::uint8_t SH200Q_REG_GYRO_RANGE = 0x2B;
    static constexpr std::uint8_t SH200Q_REG_ACC_FIFO_STATUS = 0x2E;
    static constexpr std::uint8_t SH200Q_REG_GYRO_FIFO_STATUS = 0x2F;
    static constexpr std::uint8_t SH200Q_REG_ADC_RESET = 0xC2;
    static constexpr std::uint8_t SH200Q_REG_PLL_RESET = 0xBA;
    static constexpr std::uint8_t SH200Q_CHIP_ID = 0x18;

    static constexpr TickType_t DEFAULT_REG_TIMEOUT = pdMS_TO_TICKS(10);

    static constexpr float ACCELEROMETER_RESOLUTION = 8.0/32768.0;
    static constexpr float GYRO_RESOLUTION = 2000.0/32768.0;

    #define TAG_IMU "IMU"

    Result<std::uint8_t, esp_err_t> read_single_register(std::uint8_t register_address, TickType_t wait_ticks=DEFAULT_REG_TIMEOUT)
    {
        return this->i2c.read_single_register(this->address, register_address, wait_ticks);
    }
    Result<void, esp_err_t> write_single_register(std::uint8_t register_address, std::uint8_t value, TickType_t wait_ticks=DEFAULT_REG_TIMEOUT)
    {
        return this->i2c.write_single_register(this->address, register_address, value, wait_ticks);
    }
public:
    IMU(I2CMaster& i2c, std::uint8_t address) : i2c(i2c), address(address), acceleration_offset(0, 0, 0) {}

    Result<void, esp_err_t> reset()
    {
        I2CCommandLink commands;
        if( !commands ) {
            return failure(ESP_ERR_NO_MEM);
        }

        // Check chip ID
        ESP_LOGI(TAG_IMU, "Checking chip id...");
        auto chip_id = this->read_single_register(SH200Q_REG_CHIP_ID);
        if( !chip_id ) return failure(chip_id);
        ESP_LOGI(TAG_IMU, "Chip ID: %02x", chip_id.value);
        if( chip_id.value != SH200Q_CHIP_ID ) {
            return failure(ESP_ERR_INVALID_RESPONSE);
        }

        ESP_LOGI(TAG_IMU, "Configuring registers...");
        // Configure registers
        {
            auto result = this->write_single_register(SH200Q_REG_ACC_CONFIG, 0x91); // HPF disabled, internal clock, ODR=256Hz, filter enabled
            if( !result ) return failure(result);
            result = this->write_single_register(SH200Q_REG_GYRO_CONFIG, 0x03); // HPF enabled, ODR=500Hz, filter enabled
            if( !result ) return failure(result);
            result = this->write_single_register(SH200Q_REG_GYRO_CONFIG_1, 0x13);   // data from HPF, DLPF=3
            if( !result ) return failure(result);
            result = this->write_single_register(SH200Q_REG_FIFO_CONFIG, 0x80); // Stream mode
            if( !result ) return failure(result);
            result = this->write_single_register(SH200Q_REG_ACC_DATA_FORMAT, 0x01); // Acc full scale = 8[G]
            if( !result ) return failure(result);
            result = this->write_single_register(SH200Q_REG_GYRO_RANGE, 0x01);  // Gyro full scale = 1000[DPS]
            if( !result ) return failure(result);
        }
        
        ESP_LOGI(TAG_IMU, "Resetting PLL and ADC...");

        // Reset PLL
        {
            auto reg_value = this->read_single_register(SH200Q_REG_PLL_RESET);
            if( !reg_value ) return failure(reg_value);
            auto write_result = this->write_single_register(SH200Q_REG_PLL_RESET, reg_value.value & ~0x01);
            if( !write_result ) return failure(write_result);
        }

        // Reset ADC
        {
            auto reg_value = this->read_single_register(SH200Q_REG_ADC_RESET);
            if( !reg_value ) return failure(reg_value);
            auto write_result = this->write_single_register(SH200Q_REG_ADC_RESET, reg_value.value | 0x10);
            if( !write_result ) return failure(write_result);
            write_result = this->write_single_register(SH200Q_REG_ADC_RESET, reg_value.value & ~0x10);
            if( !write_result ) return failure(write_result);
        }   

        ESP_LOGI(TAG_IMU, "Initialized");

        this->acceleration_queue.reset();
        this->angular_velocity_queue.reset();

        return success();
    }

    Result<void, esp_err_t> update()
    {
        // Read FIFO status registers to get how many samples in the acc/gyro FIFO.
        std::uint8_t fifo_status[2];
        {
            I2CCommandLink commands;
            if( !commands ) {
                return failure(ESP_ERR_NO_MEM);
            }
            auto result = commands.read_register(this->address, SH200Q_REG_ACC_FIFO_STATUS, fifo_status, 2, i2c_ack_type_t::I2C_MASTER_LAST_NACK)
                .then([&commands, this](){ return this->i2c.execute(commands, DEFAULT_REG_TIMEOUT); });
            if( !result ) {
                return failure(result);
            }
        }
        std::uint8_t acc_count = fifo_status[0] & 0x3f;;
        std::uint8_t gyro_count = fifo_status[1] & 0x3f;
        ESP_LOGI(TAG_IMU, "FIFO status: %02x, %02x", acc_count, gyro_count);

        // Read acc/gyro sensor data.
        std::uint8_t buffer[2*6*32];

        {
            std::uint8_t acc_remaining  = acc_count;
            std::uint8_t gyro_remaining = gyro_count;

            std::uint8_t* buffer_ptr = buffer;
            while(acc_remaining > 0 && gyro_remaining > 0) {
                I2CCommandLink commands;
                if( !commands ) {
                    return failure(ESP_ERR_NO_MEM);
                }
                std::uint8_t register_address = acc_remaining > 0 ? 0x00 : 0x06;
                std::uint8_t read_length = (acc_remaining > 0 ? 6 : 0) + (gyro_remaining > 0 ? 6 : 0);

                auto result = commands.read_register(this->address, register_address, buffer_ptr, read_length, i2c_ack_type_t::I2C_MASTER_LAST_NACK)
                    .then([&commands, this]() { return this->i2c.execute(commands, DEFAULT_REG_TIMEOUT); });
                if( !result ) {
                    return failure(result);
                }

                buffer_ptr += read_length;
                acc_remaining = acc_remaining > 0 ? acc_remaining - 1 : 0;
                gyro_remaining = gyro_remaining > 0 ? gyro_remaining - 1 : 0;
            }
        }

        // Parse contents of the buffer and put them into queues.
        {
            std::uint8_t acc_remaining  = acc_count;
            std::uint8_t gyro_remaining = gyro_count;

            std::uint8_t* buffer_ptr = buffer;
            while(acc_remaining > 0 && gyro_remaining > 0) {
                if( acc_remaining > 0 ) {
                    auto acceleration = Vector3I16(
                        static_cast<int16_t>(buffer_ptr[0] | (static_cast<uint16_t>(buffer_ptr[1]) << 8)),
                        static_cast<int16_t>(buffer_ptr[2] | (static_cast<uint16_t>(buffer_ptr[3]) << 8)),
                        static_cast<int16_t>(buffer_ptr[4] | (static_cast<uint16_t>(buffer_ptr[5]) << 8)));
                    acceleration += this->acceleration_offset;
                    this->acceleration_queue.queue(acceleration);
                    buffer_ptr += 6;
                    acc_remaining--;   
                }
                if( gyro_remaining > 0 ) {
                    auto gyro = Vector3I16(
                        static_cast<int16_t>(buffer_ptr[0] | (static_cast<uint16_t>(buffer_ptr[1]) << 8)),
                        static_cast<int16_t>(buffer_ptr[2] | (static_cast<uint16_t>(buffer_ptr[3]) << 8)),
                        static_cast<int16_t>(buffer_ptr[4] | (static_cast<uint16_t>(buffer_ptr[5]) << 8)));
                    this->angular_velocity_queue.queue(gyro);
                    buffer_ptr += 6;
                    gyro_remaining--;
                }
            }
        }
        return success();
    }

    Result<Vector3F, bool> get_acceleration()
    {
        auto result = this->acceleration_queue.dequeue();
        if( !result ) {
            return failure(false);
        }
        return success<Vector3F>(result.value * ACCELEROMETER_RESOLUTION);
    }

    Result<Vector3F, bool> get_angular_velocity()
    {
        auto result = this->angular_velocity_queue.dequeue();
        if( !result ) {
            return failure(false);
        }
        return success<Vector3F>(result.value * GYRO_RESOLUTION);
    }
};

struct IMUData
{
    Vector3F acc;
    Vector3F gyro;
    Vector3F mag;
};

static I2CMaster i2c(I2C_NUM_1);

static freertos::WaitQueue<IMUData, 10> imu_queue;
static void imu_task_proc();
static StaticTask<8192, decltype(&imu_task_proc)> imu_task(&imu_task_proc);
#define TAG_IMU "IMU"
static void imu_task_proc() 
{
    static IMU imu(i2c, 0x6C);
    {
        ESP_LOGI(TAG_IMU, "Initializing IMU sensor");
        // Initialize IMU
        auto result = imu.reset();
        if( !result ) {
            ESP_LOGE(TAG_IMU, "Failed to initialize IMU - %08x", result.error_code);
            return;
        }
    }
    auto task = Task::current();
    while( Task::notify_wait(0, 1, portMAX_DELAY).is_success ) {
        ESP_LOGV(TAG_IMU, "IMU sensor measurement begin");
        IMUData imu_data;
        imu_data.acc = Vector3F(0, 0, 0);
        imu_data.gyro = Vector3F(0, 0, 0);
        int acc_count = 0;
        int gyro_count = 0;

        // Read IMU sensor data via I2C
        auto result = imu.update();
        if( result ) {
            while(auto acc = imu.get_acceleration()) {
                imu_data.acc += acc.value;
                acc_count++;
            }
            while(auto gyro = imu.get_angular_velocity()) {
                imu_data.gyro += gyro.value;
                gyro_count++;
            }
            if( acc_count > 0 ) {
                imu_data.acc /= acc_count;
            }
            if( gyro_count > 0 ) {
                imu_data.gyro /= gyro_count;
            }

            ESP_LOGV(TAG_IMU, "IMU sensor measurement end. acc_count=%d, gyro_count=%d acc=%f,%f,%f gyro=%f,%f,%f"
                , acc_count
                , gyro_count
                , imu_data.acc.x_const()
                , imu_data.acc.y_const()
                , imu_data.acc.z_const()
                , imu_data.gyro.x_const()
                , imu_data.gyro.y_const()
                , imu_data.gyro.z_const()
            );
            imu_queue.send(imu_data);
        }
        else {
            ESP_LOGE(TAG_IMU, "IMU update failed - %x", result.error_code);
        }
    }
}

// Wakeup IMU task periodically
static void imu_timer_proc();
static Timer<decltype(&imu_timer_proc)> imu_timer(&imu_timer_proc);
static void imu_timer_proc()
{
    if( imu_task ) {
        imu_task.task().notify(1, eNotifyAction::eSetBits);
    }
}


class PMU
{
private:
    I2CMaster& i2c;
    std::uint8_t address;
    
    static constexpr std::uint8_t AXP192_REG_EXTEN_DCDC2_OUTPUT_CONFIG = 0x10;
    static constexpr std::uint8_t AXP192_REG_LDO2_LDO3_OUTPUT_VOLTAGE = 0x28;
    static constexpr std::uint8_t AXP192_REG_ADC_ENABLE_1 = 0x28;
    static constexpr std::uint8_t AXP192_REG_CHARGING_CONFIG = 0x33;
    static constexpr std::uint8_t AXP192_REG_DCDC_OPERATING_MODE = 0x80;
    static constexpr std::uint8_t AXP192_REG_COULOMB_COUNTER = 0xB8;
    static constexpr std::uint8_t AXP192_REG_OUTPUT_ENABLE = 0x12;
    static constexpr std::uint8_t AXP192_REG_PEK_CONFIG = 0x36;
    static constexpr std::uint8_t AXP192_REG_GPIO0_FUNCTION = 0x90;
    static constexpr std::uint8_t AXP192_REG_VBUS_IPSOUT_PATH_CONFIG = 030;
    static constexpr std::uint8_t AXP192_REG_VHTF_CHARGE_THRESHOLD = 0x39;
    static constexpr std::uint8_t AXP192_REG_BACKUP_BATTERY_CHARGING = 0x35;
    

    static constexpr TickType_t DEFAULT_REG_TIMEOUT = pdMS_TO_TICKS(10);

    #define TAG_PMU "PMU"

    Result<std::uint8_t, esp_err_t> read_single_register(std::uint8_t register_address, TickType_t wait_ticks=DEFAULT_REG_TIMEOUT)
    {
        return this->i2c.read_single_register(this->address, register_address, wait_ticks);
    }
    Result<void, esp_err_t> write_single_register(std::uint8_t register_address, std::uint8_t value, TickType_t wait_ticks=DEFAULT_REG_TIMEOUT)
    {
        return this->i2c.write_single_register(this->address, register_address, value, wait_ticks);
    }
public:
    PMU(I2CMaster& i2c, std::uint8_t address) : i2c(i2c), address(address) {}

    Result<void, esp_err_t> reset()
    {
        ESP_LOGD(TAG_PMU, "Configuring registers...");
        // Configure registers
        {
            auto result = this->write_single_register(AXP192_REG_EXTEN_DCDC2_OUTPUT_CONFIG, 0xff);
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_LDO2_LDO3_OUTPUT_VOLTAGE, 0xcc);   // LDO2 = LED 3.0V, LDO3 = TFT 3.0V
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_ADC_ENABLE_1, 0xff);   // Enable all ADCs.
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_CHARGING_CONFIG, 0xc1);   // Enable charging, 190mA, 4.2V, 90%
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_COULOMB_COUNTER, 0x80);   // Enable coulomb counter (battery energy counter)
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_OUTPUT_ENABLE, 0x4d);   // Enable EXTEN, LDO3, LDO2, DC-DC1 output
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_OUTPUT_ENABLE, 0x4d);   // Enable EXTEN, LDO3, LDO2, DC-DC1 output
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_PEK_CONFIG, 0x4d);   // Shutdown 4s, Shutdown with button press longer than 4s, 
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_GPIO0_FUNCTION, 0x02);   // GPIO0 = Low Noise LDO, 
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_VBUS_IPSOUT_PATH_CONFIG, 0xe0); // No use N_VBUSen, Limit Vbus voltage = 4.4V, Limit Vbus current = 500mA
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_VHTF_CHARGE_THRESHOLD, 0xfc); // 
            if( !result ) return failure(result);
            result = this->write_single_register(AXP192_REG_BACKUP_BATTERY_CHARGING, 0xa2); // Enable charging backup battery, 3.0V, 200uA 
            if( !result ) return failure(result);
        }
        
        ESP_LOGI(TAG_PMU, "Initialized");

        return success();
    }
};

static PMU pmu(i2c, 0x34);

#define TAG "MAIN"
extern "C" void app_main(void)
{
    // Initialie i2c
    {
        i2c_config_t i2c_config;
        i2c_config.mode = I2C_MODE_MASTER;
        i2c_config.sda_io_num = GPIO_NUM_21;
        i2c_config.sda_pullup_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
        i2c_config.scl_io_num = GPIO_NUM_22;
        i2c_config.scl_pullup_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
        i2c_config.master.clk_speed = 400000;
        auto result = i2c.initialize(i2c_config);
        if( !result ) ESP_ERROR_CHECK(result.error_code);
    }

    // Do I2C Scan
    {
        ESP_LOGI(TAG, "Scanning I2C Bus begin...");
        for(std::uint8_t device_address = 0; device_address <= 0x7f; device_address++ ) {
            I2CCommandLink commands;
            if( !commands ) continue;

            auto result = commands.start()
                .then([&commands, device_address](){ return commands.write_byte((device_address << 1) | 0, true); })
                .then([&commands](){ return commands.stop(); })
                .then([&commands](){ return i2c.execute(commands, pdMS_TO_TICKS(50)); });
            if( result.is_success ) {
                ESP_LOGI(TAG, "Address %02x: OK", device_address);
            }
            else if( result.error_code == ESP_ERR_TIMEOUT ) {
                ESP_LOGI(TAG, "Address %02x: Timed out", device_address);
            }
        }
        ESP_LOGI(TAG, "Scanning I2C end");

    }

    // Initialize PMU
    {
        auto result = pmu.reset();
        if( !result ) {
            ESP_LOGE(TAG, "Failed to initialize PMU - %x", result.error_code);
        }
    }

    lcd.begin();

    // Clear IMU queue
    imu_queue.reset();

    // Initialize IMU task
    imu_task.start("IMU", 3, APP_CPU_NUM);

    // Start IMU timer
    imu_timer.start(1000000ul);

    IMUData imu_data;
    while(imu_queue.receive(imu_data)) {
        ESP_LOGI(TAG, "IMU data: acc=%f,%f,%f gyro=%f,%f,%f"
            , imu_data.acc.x_const()
            , imu_data.acc.y_const()
            , imu_data.acc.z_const()
            , imu_data.gyro.x_const()
            , imu_data.gyro.y_const()
            , imu_data.gyro.z_const()
        );
    }
}
