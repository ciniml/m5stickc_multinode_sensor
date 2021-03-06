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
public:
    enum class GyroFullScale
    {
        DPS_250,
        DPS_500,
        DPS_1000,
        DPS_2000,
    };
    enum class AccelerometerFullScale
    {
        G_2,
        G_4,
        G_8,
        G_16,
    };
    struct SensorData
    {
        Vector3F acc;
        Vector3F gyro;
    };
private:
    I2CMaster& i2c;
    std::uint8_t address;
    
    AccelerometerFullScale accel_fullscale;
    GyroFullScale gyro_fullscale;
    
    struct RawSensorData
    {
        Vector3I16 acc;
        Vector3I16 gyro;
    };

    RingBuffer<RawSensorData, 64> sensor_data_queue;
    RingBuffer<std::uint16_t, 16, true> fifo_counts;
    
    static constexpr std::uint8_t REG_SMPLRT_DIV = 25;
    static constexpr std::uint8_t REG_CONFIG = 26;
    static constexpr std::uint8_t REG_GYRO_CONFIG = 27;
    static constexpr std::uint8_t REG_ACCEL_CONFIG = 28;
    static constexpr std::uint8_t REG_ACCEL_CONFIG2 = 29;
    static constexpr std::uint8_t REG_FIFO_EN = 35;
    static constexpr std::uint8_t REG_INT_PIN_CFG = 55;
    static constexpr std::uint8_t REG_INT_ENABLE = 56;
    static constexpr std::uint8_t REG_WM_INT_STATUS = 57;
    static constexpr std::uint8_t REG_INT_STATUS = 58;
    static constexpr std::uint8_t REG_FIFO_WM_TH1 = 96;
    static constexpr std::uint8_t REG_FIFO_WM_TH2 = 97;
    static constexpr std::uint8_t REG_SIGNAL_PATH_RESET = 104;
    static constexpr std::uint8_t REG_ACCEL_INTEL_CTRL = 105;
    static constexpr std::uint8_t REG_USER_CTRL = 106;
    static constexpr std::uint8_t REG_PWR_MGMT_1 = 107;
    static constexpr std::uint8_t REG_PWR_MGMT_2 = 108;
    static constexpr std::uint8_t REG_FIFO_COUNTH = 114;
    static constexpr std::uint8_t REG_FIFO_COUNTL = 115;
    static constexpr std::uint8_t REG_FIFO_R_W = 116;
    static constexpr std::uint8_t REG_WHO_AM_I = 117;

    static constexpr std::uint8_t DEVICE_ID = 0x19;
    static constexpr std::uint8_t FIFO_THRESHOLD = 20;

    static constexpr freertos::Ticks DEFAULT_REG_TIMEOUT = freertos::to_ticks(std::chrono::microseconds(10));

    static constexpr const char* TAG = "IMU";

    Result<std::uint8_t, esp_err_t> read_single_register(std::uint8_t register_address, freertos::Ticks wait_ticks=DEFAULT_REG_TIMEOUT)
    {
        return this->i2c.read_single_register(this->address, register_address, wait_ticks);
    }
    Result<void, esp_err_t> write_single_register(std::uint8_t register_address, std::uint8_t value, freertos::Ticks wait_ticks=DEFAULT_REG_TIMEOUT)
    {
        return this->i2c.write_single_register(this->address, register_address, value, wait_ticks);
    }
public:
    


    IMU(I2CMaster& i2c, std::uint8_t address) : i2c(i2c), address(address), accel_fullscale(AccelerometerFullScale::G_16), gyro_fullscale(GyroFullScale::DPS_2000) {}

    float get_gyro_fullscale_value() const 
    {
        switch(this->gyro_fullscale)
        {
            case GyroFullScale::DPS_250: return 250.0f;
            case GyroFullScale::DPS_500: return 500.0f;
            case GyroFullScale::DPS_1000: return 1000.0f;
            case GyroFullScale::DPS_2000: return 2000.0f;
            default: return 0;
        }
    }
    GyroFullScale get_gyro_fullscale() const { return this->gyro_fullscale; }
    Result<void, esp_err_t> set_gyro_fullscale(GyroFullScale fullscale)
    {
        std::uint8_t value = 0;
        switch(fullscale)
        {
            case GyroFullScale::DPS_250: value = 0b00; break;
            case GyroFullScale::DPS_500: value = 0b01; break;
            case GyroFullScale::DPS_1000: value = 0b10; break;
            case GyroFullScale::DPS_2000: value = 0b11; break;
            default: return failure(ESP_ERR_INVALID_ARG);
        }

        RESULT_TRY(this->write_single_register(REG_GYRO_CONFIG, value << 3));
        this->gyro_fullscale = fullscale;

        return success();
    }

    float get_accelerometer_fullscale_value() const 
    {
        switch(this->accel_fullscale)
        {
            case AccelerometerFullScale::G_2: return 2.0f;
            case AccelerometerFullScale::G_4: return 4.0f;
            case AccelerometerFullScale::G_8: return 8.0f;
            case AccelerometerFullScale::G_16: return 16.0f;
            default: return 0;
        }
    }
    AccelerometerFullScale get_accelerometer_fullscale() const { return this->accel_fullscale; }
    Result<void, esp_err_t> set_accelerometer_fullscale(AccelerometerFullScale fullscale)
    {
        std::uint8_t value = 0;
        switch(fullscale)
        {
            case AccelerometerFullScale::G_2: value = 0b00; break;
            case AccelerometerFullScale::G_4: value = 0b01; break;
            case AccelerometerFullScale::G_8: value = 0b10; break;
            case AccelerometerFullScale::G_16: value = 0b11; break;
            default: return failure(ESP_ERR_INVALID_ARG);
        }

        RESULT_TRY(this->write_single_register(REG_ACCEL_CONFIG, value << 3));
        this->accel_fullscale = fullscale;

        return success();
    }

    Result<void, esp_err_t> reset()
    {
        // Check chip ID
        ESP_LOGI(TAG, "Checking device id...");
        auto device_id = this->read_single_register(REG_WHO_AM_I);
        if( !device_id ) return failure(device_id);
        ESP_LOGI(TAG, "Device ID: %02x", device_id.value);
        if( device_id.value != DEVICE_ID ) {
            return failure(ESP_ERR_INVALID_RESPONSE);
        }

        ESP_LOGI(TAG, "Resetting Device...");
        RESULT_TRY(this->write_single_register(REG_PWR_MGMT_1, 0x80));
        vTaskDelay(pdMS_TO_TICKS(100));
        for(std::uint32_t trial = 0; ; trial++ ) {
            auto result = this->read_single_register(REG_PWR_MGMT_1);
            if( !result ) {
                return failure(result);
            }
            if( (result.value & 0x80) == 0 ) {
                break;
            }
            if( trial >= 20 ) {
                ESP_LOGE(TAG, "Reset timed out.");
                return failure(ESP_ERR_TIMEOUT);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Configure registers
        ESP_LOGI(TAG, "Configuring Device...");
        {
            // Configure power management
            RESULT_TRY( this->write_single_register(REG_PWR_MGMT_1, 0x01) );    // CKSEL[2:0] = 0b001
            
            // Set full scale 
            RESULT_TRY( this->set_accelerometer_fullscale(AccelerometerFullScale::G_16) );
            RESULT_TRY( this->set_gyro_fullscale(GyroFullScale::DPS_2000) );

            // Set filter and ODR
            RESULT_TRY( this->write_single_register(REG_ACCEL_CONFIG2, 0) );    // DEC2_CFG=0, ACCEL_FCHOICE_0=0, A_DLPF_CFG=0
            RESULT_TRY( this->write_single_register(REG_SMPLRT_DIV, 4) );       // SMPLRT_DIV=4 
            // Set DLPF config and FIFO mode
            RESULT_TRY( this->write_single_register(REG_CONFIG, 0x01) );    // FIFO_MODE=0, DLPF_CFG=1
            // Enable gyro and accerometer FIFO
            RESULT_TRY( this->write_single_register(REG_FIFO_EN, 0x18) );   // GYRO_FIFO_EN=1, ACCEL_FIFO_EN=1
            // Configure interrupt
            RESULT_TRY( this->write_single_register(REG_INT_PIN_CFG, 0xA0));
        }
        
        ESP_LOGI(TAG, "Initialized");

        this->sensor_data_queue.reset();

        return success();
    }

    Result<void, esp_err_t> start()
    {
        // Enable FIFO
        RESULT_TRY( this->write_single_register(REG_USER_CTRL, 0x40) );   // FIFO_EN=1
        // Set water mark threshold to FIFO_THRESHOLD samples
        const std::uint16_t threshold = 2*7*FIFO_THRESHOLD;
        const std::uint8_t threshold_buffer[2] = {threshold >> 8, threshold & 0xff};
        RESULT_TRY( this->i2c.write_register(this->address, REG_FIFO_WM_TH1, threshold_buffer, 2, DEFAULT_REG_TIMEOUT));

        return success();
    }

    Result<bool, esp_err_t> check_interrupt_status()
    {
        auto result = this->read_single_register(REG_WM_INT_STATUS, DEFAULT_REG_TIMEOUT);
        if( !result ) {
            return failure(result);
        }
        return success( (result.value & 0x40) != 0 );   // Check if FIFO_WM_INT bit is set.
    }

    Result<void, esp_err_t> update()
    {
        // Read acc/gyro sensor data.
        std::uint8_t buffer[2*7*FIFO_THRESHOLD];

        // Read FIFO status register to get how many samples in the FIFO.
        std::uint16_t fifo_count = 0;
        {
            RESULT_TRY( this->i2c.read_register(this->address, REG_FIFO_COUNTH, buffer, 2, DEFAULT_REG_TIMEOUT) );
            fifo_count = (static_cast<std::uint16_t>(buffer[0]) << 8) | (static_cast<std::uint16_t>(buffer[1]) & 0xff);
        }
        std::uint16_t bytes_remaining = fifo_count - (fifo_count % (2*7));

        while(bytes_remaining > 0) {
            I2CCommandLink commands;
            if( !commands ) {
                return failure(ESP_ERR_NO_MEM);
            }
            std::uint16_t bytes_to_read = bytes_remaining > sizeof(buffer) ? sizeof(buffer) : bytes_remaining;

            RESULT_TRY(commands.read_register(this->address, REG_FIFO_R_W, buffer, bytes_to_read, i2c_ack_type_t::I2C_MASTER_LAST_NACK));
            RESULT_TRY(this->i2c.execute(commands, DEFAULT_REG_TIMEOUT*10));

            for(std::uint16_t index = 0; index < bytes_to_read; index += 2*7) {   // According to the FIFO_R_W register read value, 0xff indicated that FIFO is empty.
                RawSensorData sensor_data;
                sensor_data.acc = Vector3I16(
                        static_cast<int16_t>((static_cast<uint16_t>(buffer[index+0+0]) << 8) | buffer[index+0+1]),
                        static_cast<int16_t>((static_cast<uint16_t>(buffer[index+0+2]) << 8) | buffer[index+0+3]),
                        static_cast<int16_t>((static_cast<uint16_t>(buffer[index+0+4]) << 8) | buffer[index+0+5]));
                sensor_data.gyro = Vector3I16(
                        static_cast<int16_t>((static_cast<uint16_t>(buffer[index+8+0]) << 8) | buffer[index+8+1]),
                        static_cast<int16_t>((static_cast<uint16_t>(buffer[index+8+2]) << 8) | buffer[index+8+3]),
                        static_cast<int16_t>((static_cast<uint16_t>(buffer[index+8+4]) << 8) | buffer[index+8+5]));
                this->sensor_data_queue.queue(sensor_data);
            }
            bytes_remaining -= bytes_to_read;
            // for(std::uint16_t i = 0; i < sizeof(buffer); i += 16) {
            //     char line[16*3+1];
            //     char* p = line;
            //     for(std::uint16_t j = 0; j < 16 && i+j < sizeof(buffer); j++, p+=3) {
            //         sprintf(p, "%02x,", buffer[i+j]);
            //     }
            //     *p = 0;
            //     ESP_LOGI(TAG, "buffer: %04x: %s", i, line);
            // }
        }
        return success();
    }

    void clear_sensor_data()
    {
        this->sensor_data_queue.reset();
    }

    Result<SensorData, bool> get_sensor_data()
    {
        auto result = this->sensor_data_queue.dequeue();
        if( !result ) {
            return failure(false);
        }
        SensorData sensor_data = {
            result.value.acc * (this->get_accelerometer_fullscale_value()/32768.0f),
            result.value.gyro * (this->get_gyro_fullscale_value()/32768.0f),
        };
        return success<SensorData>(sensor_data);
    }

    std::uint16_t get_max_fifo_usage()
    {
        std::uint8_t usage = 0;
        for(const auto& value : this->fifo_counts ) {
            if( value > usage ) {
                usage = value;
            }
        }
        return usage;
    }
};

constexpr freertos::Ticks IMU::DEFAULT_REG_TIMEOUT;

#endif //IMU_HPP__