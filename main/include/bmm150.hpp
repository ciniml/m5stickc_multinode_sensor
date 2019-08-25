#ifndef BMM150_HPP__
#define BMM150_HPP__

#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_log.h>

#include <bmm150.h>

#include "freertos_util.hpp"
#include "i2c.hpp"
#include "vector3.hpp"
#include "ringbuffer.hpp"

class BMM150
{
private:
    I2CMaster& i2c;
    std::uint8_t address;
    bmm150_dev device;

    static constexpr TickType_t DEFAULT_REG_TIMEOUT = pdMS_TO_TICKS(10);

    static constexpr const char* TAG = "BMM150";

    Result<std::uint8_t, esp_err_t> read_single_register(std::uint8_t register_address, TickType_t wait_ticks=DEFAULT_REG_TIMEOUT)
    {
        return this->i2c.read_single_register(this->address, register_address, wait_ticks);
    }
    Result<void, esp_err_t> write_single_register(std::uint8_t register_address, std::uint8_t value, TickType_t wait_ticks=DEFAULT_REG_TIMEOUT)
    {
        return this->i2c.write_single_register(this->address, register_address, value, wait_ticks);
    }

    static std::int8_t bmm150_read(struct bmm150_dev* dev, std::uint8_t reg_addr, std::uint8_t* read_data, std::uint16_t len)
    {
        auto this_ = reinterpret_cast<BMM150*>(dev->context);
        auto result = this_->i2c.read_register(dev->dev_id, reg_addr, read_data, len, DEFAULT_REG_TIMEOUT);
        if( !result ) {
            ESP_LOGE(TAG, "Failed to read from I2C dev=0x%02x, reg=0x%02x, size=%d - %08x", dev->dev_id, reg_addr, len, result.error_code);
        }
        return result ? 0 : -128;
    }
    static std::int8_t bmm150_write(struct bmm150_dev* dev, std::uint8_t reg_addr, std::uint8_t* write_data, std::uint16_t len)
    {
        auto this_ = reinterpret_cast<BMM150*>(dev->context);
        auto result = this_->i2c.write_register(dev->dev_id, reg_addr, write_data, len, DEFAULT_REG_TIMEOUT);
        if( !result ) {
            ESP_LOGE(TAG, "Failed to write from I2C dev=0x%02x, reg=0x%02x, size=%d - %08x", dev->dev_id, reg_addr, len, result.error_code);
        }
        return result ? 0 : -128;
    }
    static void bmm150_delay_ms(struct bmm150_dev* dev, std::uint32_t delay_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
public:
    BMM150(I2CMaster& i2c, std::uint8_t address) : i2c(i2c), address(address) {}

    Result<void, std::int8_t> reset()
    {
        this->device.dev_id = this->address;
        this->device.intf = BMM150_I2C_INTF;
        this->device.read = &bmm150_read;
        this->device.write = &bmm150_write;
        this->device.delay_ms = &bmm150_delay_ms;
        this->device.context = this;

        auto result = bmm150_init(&this->device);
        if( result < 0 ) {
            ESP_LOGE(TAG, "Failed to initialize BMM150 - %d", result);
            return failure(result);
        }
        
        this->device.settings.pwr_mode = BMM150_NORMAL_MODE;
        this->device.settings.preset_mode = BMM150_PRESETMODE_REGULAR;
        result = bmm150_set_op_mode(&this->device);
        if( result < 0 ) {
            ESP_LOGE(TAG, "Failed to set op mode of BMM150 - %d", result);
            return failure(result);
        }
        result = bmm150_set_presetmode(&this->device);
        if( result < 0 ) {
            ESP_LOGE(TAG, "Failed to set preset mode of BMM150 - %d", result);
            return failure(result);
        }

        return success();
    }

    Result<Vector3F, std::int8_t> read()
    {
        auto result = bmm150_read_mag_data(&this->device);
        if( result < 0 ) {
            return failure(result);
        }

        return success(Vector3F( this->device.data.x
                        ,this->device.data.y
                        ,this->device.data.z
        ));
    }
};


#endif //BMM150_HPP__