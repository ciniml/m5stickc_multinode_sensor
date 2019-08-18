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

#include <freertos_util.hpp>
#include <result.hpp>
#include <i2c.hpp>
#include <timer.hpp>
#include <ringbuffer.hpp>
#include <vector3.hpp>
#include <imu.hpp>
#include <pmu.hpp>

static M5Display lcd;
static I2CMaster i2c(I2C_NUM_1);
static PMU pmu(i2c, 0x34);

struct IMUData
{
    Vector3F acc;
    Vector3F gyro;
    Vector3F mag;
};

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
    lcd.setRotation(3);

    // Clear IMU queue
    imu_queue.reset();

    // Initialize IMU task
    imu_task.start("IMU", 3, APP_CPU_NUM);

    // Start IMU timer
    imu_timer.start(33333ul); // 33ms

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
        lcd.fillScreen(0);
        lcd.setCursor(0, 0);
        lcd.printf("acc : %0.2f, %0.2f, %0.2f\n"
            , imu_data.acc.x_const()
            , imu_data.acc.y_const()
            , imu_data.acc.z_const()
        );
        lcd.printf("gyro: %0.2f, %0.2f, %0.2f\n"
            , imu_data.gyro.x_const()
            , imu_data.gyro.y_const()
            , imu_data.gyro.z_const()
        );
    }
}
