#ifndef I2C_HPP__
#define I2C_HPP__

#include <cstdint>
#include <type_traits>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <driver/i2c.h>

#include "freertos_util.hpp"
#include "result.hpp"

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

        auto result = i2c_param_config(this->port, &config);
        if( result != ESP_OK ) {
            return failure(result);
        }

        result = i2c_driver_install(this->port, I2C_MODE_MASTER, 0, 0, 0);
        if( result != ESP_OK ) {
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
    Result<void, esp_err_t> read_register(std::uint8_t device_address, std::uint8_t register_address, std::uint8_t* buffer, std::size_t length, TickType_t wait_ticks)
    {
        I2CCommandLink commands;
        if( !commands ) {
            return failure(ESP_ERR_NO_MEM);
        }
        auto result = commands.read_register(device_address, register_address, buffer, length, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
        if( !result ) return failure(result);

        result = this->execute(commands, wait_ticks);
        if( !result ) return failure(result);

        return success();
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
    Result<void, esp_err_t> write_register(std::uint8_t device_address, std::uint8_t register_address, const std::uint8_t* value, std::size_t length, TickType_t wait_ticks)
    {
        I2CCommandLink commands;
        if( !commands ) {
            return failure(ESP_ERR_NO_MEM);
        }
        auto result = commands.write_register(device_address, register_address, value, length);
        if( !result ) return failure(result);

        result = this->execute(commands, wait_ticks);
        if( !result ) return failure(result);

        return success();
    }
};


#endif //I2C_HPP__