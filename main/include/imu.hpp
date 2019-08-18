#ifndef IMU_HPP__
#define IMU_HPP__

#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_log.h>

#include "freertos_util.hpp"
#include "i2c.hpp"
#include "vector3.hpp"
#include "ringbuffer.hpp"

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


#endif //IMU_HPP__