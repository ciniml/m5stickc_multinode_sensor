#include <cstdint>
#include <cstring>
#include <type_traits>
#include <memory>
#include <vector>
#include <chrono>
#include <map>

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

#include <M5Display.h>

#include <freertos_util.hpp>
#include <result.hpp>
#include <i2c.hpp>
#include <timer.hpp>
#include <ringbuffer.hpp>
#include <vector3.hpp>
#include <imu_mpu6886.hpp>
#include <bmm150.hpp>
#include <pmu.hpp>
#include <button.hpp>
#include <espnow_packet.hpp>
#include <sensor_node.hpp>
#include <receiver_node.hpp>

static M5Display lcd;

static I2CMaster i2c_internal(I2C_NUM_1);
static I2CMaster i2c_external(I2C_NUM_0);

static PMU pmu(i2c_internal, 0x34);

class IMUTask : public freertos::StaticTask<8192, IMUTask>
{
private:
    static constexpr const char* TAG = "IMUTASK";
    static constexpr std::uint32_t NOTIFY_BIT_TIMER = 0x0001;
    static constexpr std::uint32_t NOTIFY_BIT_CLEAR_ERROR = 0x0002;
    static constexpr std::uint32_t NOTIFY_BIT_START_MEASUREMENT = 0x0004;

    freertos::WaitQueue<IMUData, 256> imu_queue;
    enum class State
    {
        NotStarted,
        Initializing,
        Active,
        Error,
    };
    State state;
    IMU imu;
    BMM150 magnetometer;

    static std::chrono::microseconds get_timestamp() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
    }

    struct UpdateTimer : Timer<UpdateTimer>
    {
        IMUTask& task;
        bool enabled;
        UpdateTimer(IMUTask& task) : task(task), enabled(true) {}
        
        void operator() () 
        {
            if( this->task && this->enabled ) {
                this->task.task().notify(NOTIFY_BIT_TIMER, eNotifyAction::eSetBits);
            }
        }

        void enable() { this->enabled = true; }
        void disable() { this->enabled = false; }
    };
    struct MagnetometerTimer : Timer<MagnetometerTimer>
    {
        IMUTask& task;
        freertos::InterlockedValue<Vector3F> value;

        MagnetometerTimer(IMUTask& task) : task(task) {}
        
        void operator() () 
        {
            auto result = task.magnetometer.read();
            if( !result ) {
                ESP_LOGE(TAG, "Failed to read magnetometer");
                return;
            }

            this->value.set_value(result.value);
        }
    };

    
    UpdateTimer update_timer;
    MagnetometerTimer mag_timer;
public:
    IMUTask(I2CMaster& internal_i2c, I2CMaster& external_i2c) 
        : state(State::NotStarted)
        , imu(i2c_internal, 0x68)
        , magnetometer(external_i2c, 0x10)
        , update_timer(*this)
        , mag_timer(*this) 
    {
    }

    void clear_error() 
    {
        this->task().notify(NOTIFY_BIT_CLEAR_ERROR, eNotifyAction::eSetBits);
    }

    void resume() 
    {
        this->update_timer.enable();
    }
    
    void pause() 
    {
        this->update_timer.disable();
    }

    bool start()
    {
        if( !IMUTask::SelfType::start("IMU", 3, APP_CPU_NUM) ) {
            ESP_LOGE(TAG, "Failed to start IMU task");
            return false;
        }

        {
            auto result = this->update_timer.start(std::chrono::microseconds(10000ul));
            if( !result ) {
                ESP_LOGE(TAG, "Failed to start IMU task timer - %x", result.error_code);
                return false;
            }
        }
        {
            auto result = this->mag_timer.start(std::chrono::microseconds(100000ul));
            if( !result ) {
                ESP_LOGE(TAG, "Failed to start magnetometer timer - %x", result.error_code);
                return false;
            }
        }
        return true;
    }

    void start_measurement()
    {
        this->task().notify(NOTIFY_BIT_START_MEASUREMENT, eNotifyAction::eSetBits);
    }

    void operator() ()
    {
        while(true)
        {
            freertos::Task::notify_wait(0, NOTIFY_BIT_START_MEASUREMENT, freertos::MAX_DELAY);

            if( this->state == State::Error ) {
                freertos::Task::notify_wait(0, NOTIFY_BIT_CLEAR_ERROR, freertos::MAX_DELAY);
            }
            std::uint8_t retry = 0;
            const std::uint8_t max_retry = 10;
            this->state = State::Initializing;
            
            for(retry = 0; retry < max_retry; retry++) {
                {
                    ESP_LOGI(TAG, "Initializing IMU sensor");
                    // Initialize IMU
                    auto result = imu.reset();
                    if( !result ) {
                        ESP_LOGE(TAG, "Failed to initialize IMU - %08x", result.error_code);
                        freertos::Task::delay_ms(10);
                        continue;
                    }
                }
                {
                    ESP_LOGI(TAG, "Initializing magnetometer sensor");
                    auto result = magnetometer.reset();
                    if( !result ) {
                        ESP_LOGE(TAG, "Failed to initialize magnetometer - %08x", result.error_code);
                        freertos::Task::delay_ms(10);
                        continue;
                    }
                }
                break;
            }
            if( retry == max_retry ) {
                state = State::Error;
                continue;
            }

            ESP_LOGI(TAG, "IMU initialization completed.");
            this->state = State::Active;

            while( freertos::Task::notify_wait(0, NOTIFY_BIT_TIMER, freertos::MAX_DELAY).is_success ) {
                ESP_LOGV(TAG, "IMU sensor measurement begin");

                // Read IMU sensor data via I2C
                auto result = imu.update();
                if( result ) {
                    IMUData imu_data;
                    //imu_data.max_fifo_usage  = imu.get_max_fifo_usage();

                    while(auto result = imu.get_sensor_data())
                    {
                        imu_data.acc = result.value.acc;
                        imu_data.gyro = result.value.gyro;
                        imu_data.mag = this->mag_timer.value.get_value();
                        imu_data.timestamp = get_timestamp();
                        ESP_LOGV(TAG, "IMU sensor measurement end. acc=%f,%f,%f gyro=%f,%f,%f mag=%f,%f,%f"
                            , imu_data.acc.x_const()
                            , imu_data.acc.y_const()
                            , imu_data.acc.z_const()
                            , imu_data.gyro.x_const()
                            , imu_data.gyro.y_const()
                            , imu_data.gyro.z_const()
                            , imu_data.mag.x_const()
                            , imu_data.mag.y_const()
                            , imu_data.mag.z_const()
                        );
                        imu_queue.send(imu_data);
                    }
                }
                else {
                    ESP_LOGE(TAG, "IMU update failed - %x", result.error_code);
                }
            }
        }
    }

    Result<IMUData, bool> receive()
    {
        IMUData imu_data;
        if( !this->imu_queue.receive(imu_data) ) {
            return failure(false);
        }
        return success<IMUData>(imu_data);
    }
};

// Main Task
static constexpr const char* TAG = "MAIN";

static IMUTask imu_task(i2c_internal, i2c_external);
static Button buttons;
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

enum class MainState
{
    Initializing,
    ModeSelecting,
    SensorConnecting,
    SensorConnected,
    ReceiverConnecting,
    ReceiverConnected,
    Testing,
    FatalError,
};

static volatile MainState main_state = MainState::Initializing;
static char error_message[128] = {0,};
static void fatal_error(const char* message)
{
    strncpy(error_message, message, sizeof(error_message));
    main_state = MainState::FatalError;
}
template<typename... Args>
static void fatal_error(const char* format, const Args&... args)
{
    snprintf(error_message, sizeof(error_message), format, args...);
    main_state = MainState::FatalError;
}
static bool check_fatal(esp_err_t err, const char* message)
{
    if( err != ESP_OK ) {
        fatal_error("[%08x] %s", err, message);
        ESP_LOGE(TAG, "[%08x] %s", err, message);
    }
    return err != ESP_OK;
}
static bool check_fatal(const Result<void, esp_err_t>& result, const char* message)
{
    if( result ) {
        check_fatal(result.error_code, message);
    }
    return result;
}

static void do_fatal_error()
{
    lcd.fillScreen(lcd.color565(192, 0, 0));
    lcd.setCursor(0, 0);
    lcd.setTextColor(lcd.color565(255, 255, 255));
    lcd.print(error_message);
    while(true)
    {
        freertos::Task::delay_ms(100);
    }
}

static esp_err_t example_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_START:
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "WiFi started");
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void do_initializing()
{
    lcd.fillScreen(lcd.color565(255, 255, 255));
    lcd.setCursor(0, 0);
    lcd.setTextColor(0);
    lcd.print("Initializing...");

    esp_err_t result = ESP_OK;

    // Initialize NVS
    result = nvs_flash_init();
    if( result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if( check_fatal(nvs_flash_erase(), "failed to erase NVS") ) return;
        result = nvs_flash_init();
    }
    if( check_fatal(result, "failed to initialize NVS") ) return;
    
    // Initialize event handler
    if( check_fatal(esp_event_loop_init(example_event_handler, nullptr), "failed to initialize event handler") ) return;

    // Initialize Wi-Fi
    tcpip_adapter_init();
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    if( check_fatal(esp_wifi_init(&wifi_cfg), "failed to initialize Wi-Fi (init)") ) return;
    if( check_fatal(esp_wifi_set_storage(WIFI_STORAGE_RAM), "failed to initialize Wi-Fi (storage)") ) return;
    if( check_fatal(esp_wifi_set_mode(WIFI_MODE_AP), "failed to initialize Wi-Fi (ESPNOW)") ) return;
    if( check_fatal(esp_wifi_start(), "failed to initialize Wi-Fi (start)") ) return;
    if( check_fatal(esp_wifi_set_channel(6, wifi_second_chan_t::WIFI_SECOND_CHAN_NONE), "failed to initialize Wi-Fi (set channel)") ) return;
    if( check_fatal(esp_wifi_set_protocol(ESP_IF_WIFI_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR), "failed to initialize Wi-Fi (protocol)") ) return;

    // Initialize ESP-NOW
    if( check_fatal(esp_now_init(), "ESP-NOW init") ) return;

    esp_now_peer_info_t peer;
    std::memset(&peer, 0, sizeof(peer));
    peer.channel = 6;
    peer.ifidx = ESP_IF_WIFI_AP;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    if( check_fatal(esp_now_add_peer(&peer), "ESP-NOW add broadcast peer") ) return;

    main_state = MainState::ModeSelecting;
    buttons.clear_events();
}

static void do_modeselecting()
{
    static std::uint8_t mode = 0;

    lcd.fillScreen(0);
    lcd.setCursor(0, 0);
    lcd.setTextColor(lcd.color565(255, 255, 255));
    lcd.print("Select Mode: ");
    if( mode == 0 ) {
        lcd.println("Sensor");
    }
    else {
        lcd.println("Receiver");
    }

    while( auto event = buttons.read_event(freertos::to_ticks(std::chrono::milliseconds(10))) ) {
        if( event.value.type == Button::EventType::Pushed ) {
            if( event.value.position == Button::Position::B ) {
                mode = mode ^ 1;
            }
            else {
                if( mode == 0 ) {
                    main_state = MainState::SensorConnecting;
                }
                else {
                    main_state = MainState::ReceiverConnecting;
                }
            }
        }
    }
}

static SensorNode sensor_node;
static freertos::WaitEvent sensor_node_received_event;
static void do_sensor_connecting()
{
    static std::uint8_t mode = 0;

    lcd.fillScreen(0);
    lcd.setCursor(0, 0);
    lcd.setTextColor(lcd.color565(255, 255, 255));
    
    lcd.println("Starting sensor node...");
    check_fatal(sensor_node.start(&sensor_node_received_event), "failed to start sensor node");

    if(! imu_task.start() ) {
        fatal_error("Failed to start IMU task");
    }

    lcd.println("Waiting connection...");
    while(!sensor_node.is_connected()) {
        freertos::Task::delay_ms(50);
    }

    lcd.println("Waiting until target time...");
    while( sensor_node.measurement_target_time - sensor_node.get_adjusted_timestamp() >= std::chrono::seconds(2) ) {
        auto timestamp = sensor_node.get_adjusted_timestamp();
        lcd.fillScreen(0);
        lcd.setCursor(0, 0);
        lcd.setTextColor(lcd.color565(255, 255, 255));
        lcd.printf("%llu, %llu, %lld", timestamp.count(), SensorNode::get_timestamp().count(), sensor_node.time_offset.count());
        freertos::Task::delay_ms(1000);
    }
    while( sensor_node.measurement_target_time > sensor_node.get_adjusted_timestamp() ); // Spin wait

    imu_task.start_measurement();

    main_state = MainState::SensorConnected;
}

static void do_sensor_connected()
{
    lcd.fillScreen(0);
    lcd.setCursor(0, 0);
    lcd.setTextColor(lcd.color565(255, 255, 255));
    auto last_frame_updated = sensor_node.get_timestamp();

    while( auto result = imu_task.receive() ) {
        const auto& data = result.value;
        sensor_node.send_sensor_data(data.timestamp + sensor_node.time_offset, data.acc, data.gyro, data.mag);
        auto now = sensor_node.get_timestamp();
        if( now - last_frame_updated >= std::chrono::milliseconds(50) ) {
            last_frame_updated = now;
            lcd.fillScreen(0);
            lcd.setCursor(0, 0);
            lcd.printf("max time: %0.2f [ms]\n", sensor_node.max_send_complete_elapsed.count()/1000.0f);
            lcd.printf("avg time: %0.2f [ms]\n", sensor_node.average_send_complete_elapsed.count()/1000.0f);
        }
    }
}


struct MyReceiverNode : public ReceiverNode<MyReceiverNode>
{
    freertos::Mutex discovered_devices_lock;
    std::map<ESPNowAddress, bool> discovered_devices;

    void add_device(const ESPNowAddress& address)
    {
        auto guard = freertos::lock(this->discovered_devices_lock);
        if( this->discovered_devices.find(address) == this->discovered_devices.end() ) {
            this->discovered_devices[address] = false;
        }
    }

    void on_discover(const ESPNowAddress& address)
    {
        this->add_device(address);
    }
};
template<> ReceiverNode<MyReceiverNode>* ReceiverNode<MyReceiverNode>::instance = nullptr;
static MyReceiverNode receiver_node;
static void do_receiver_connecting()
{
    static std::uint8_t mode = 0;

    lcd.fillScreen(0);
    lcd.setCursor(0, 0);
    lcd.setTextColor(lcd.color565(255, 255, 255));
    
    lcd.println("Starting receiver node...");
    check_fatal(receiver_node.start(), "failed to start receiver node");

    buttons.clear_events();

    receiver_node.enable_discovery();

    bool discovery_completed = false;
    while(!discovery_completed) {
        while( auto event = buttons.read_event(freertos::to_ticks(std::chrono::milliseconds(50))) ) {
            if( event.value.type == Button::EventType::Pushed ) {
                if( event.value.position == Button::Position::A ) {
                    auto guard = freertos::lock(receiver_node.discovered_devices_lock);
                    if( receiver_node.discovered_devices.size() > 0 ) {
                        discovery_completed = true;
                        break;
                    }
                }
            }
        }
        // Update device list
        lcd.fillScreen(0);
        lcd.setCursor(0, 0);
        lcd.setTextColor(lcd.color565(255, 255, 255));
        {
            auto guard = freertos::lock(receiver_node.discovered_devices_lock);
            for(const auto& pair : receiver_node.discovered_devices) {
                lcd.printf(" " MACSTR "\n", MAC2STR(pair.first.values));
            }
        }
    }

    receiver_node.disable_discovery();
    {
        auto guard = freertos::lock(receiver_node.discovered_devices_lock);
        for(const auto& node : receiver_node.discovered_devices) {
            receiver_node.add_node(node.first);
        }
    }
    receiver_node.begin_connect();

    lcd.fillScreen(0);
    lcd.setCursor(0, 0);
    lcd.setTextColor(lcd.color565(255, 255, 255));
    lcd.println("Connecting to the sensor nodes...");
    while(!receiver_node.is_connected()) {
        freertos::Task::delay_ms(50);
    }
    main_state = MainState::ReceiverConnected;
}

static void do_receiver_connected()
{
    auto timestamp = MyReceiverNode::get_timestamp();
    SensorNodeIMUData imu_data;
    bool has_data = false;
    while(auto result = receiver_node.get_sensor_data()) {
        imu_data = result.value;
        ESP_LOGV(TAG, "sensor data(" MACSTR "): %0.2f, %0.2f, %0.2f"
            , MAC2STR(imu_data.address.values)
            , imu_data.data.acc.x_const()
            , imu_data.data.acc.y_const()
            , imu_data.data.acc.z_const()
        );
        has_data = true;
    }
    if( has_data ) {
        lcd.fillScreen(0);
        lcd.setCursor(0, 0);
        lcd.setTextColor(lcd.color565(255, 255, 255));
        lcd.printf("%llu\n", timestamp.count());
        lcd.printf("%llu\n", imu_data.data.timestamp.count());
        lcd.printf("node"  MACSTR "\n", MAC2STR(imu_data.address.values));
        lcd.printf("acc: %0.1f, %0.1f, %0.1f\n"
            , imu_data.data.acc.x_const()
            , imu_data.data.acc.y_const()
            , imu_data.data.acc.z_const()
        );
        lcd.printf("gyr: %0.1f, %0.1f, %0.1f\n"
            , imu_data.data.gyro.x_const()
            , imu_data.data.gyro.y_const()
            , imu_data.data.gyro.z_const()
        );
        lcd.printf("mag: %0.1f, %0.1f, %0.1f\n"
            , imu_data.data.mag.x_const()
            , imu_data.data.mag.y_const()
            , imu_data.data.mag.z_const()
        );
    }
    //ESP_LOGI(TAG, "samples received: %llu", receiver_node.get_total_number_of_samples_received());
    freertos::Task::delay_ms(50);
}


static void do_testing()
{
    auto result = imu_task.receive();
    if( !result ) {
        return;
    }

    IMUData imu_data = result.value;
    // ESP_LOGI(TAG, "IMU data: acc=%f,%f,%f gyro=%f,%f,%f"
    //     , imu_data.acc.x_const()
    //     , imu_data.acc.y_const()
    //     , imu_data.acc.z_const()
    //     , imu_data.gyro.x_const()
    //     , imu_data.gyro.y_const()
    //     , imu_data.gyro.z_const()
    // );
    lcd.fillScreen(0);
    lcd.setCursor(0, 0);
    lcd.setTextColor(lcd.color565(255, 255, 255));
    lcd.printf("acc: %0.1f, %0.1f, %0.1f\n"
        , imu_data.acc.x_const()
        , imu_data.acc.y_const()
        , imu_data.acc.z_const()
    );
    lcd.printf("gyr: %0.1f, %0.1f, %0.1f\n"
        , imu_data.gyro.x_const()
        , imu_data.gyro.y_const()
        , imu_data.gyro.z_const()
    );
    lcd.printf("mag: %0.1f, %0.1f, %0.1f\n"
        , imu_data.mag.x_const()
        , imu_data.mag.y_const()
        , imu_data.mag.z_const()
    );
    // lcd.printf("fifo: %d\n"
    //     , imu_data.max_fifo_usage
    // );

    if( buttons.is_pressed(Button::Position::B) ) {
        {
            auto result = pmu.get_coulomb_data();
            if( result ) {
                lcd.printf("charge: %f.2[C]\n", result.value);
            }
            else {
                lcd.printf("charge: error\n");
            }
        }
        {
            PMU::ADCRegisters regs;
            auto result = pmu.read_adc_registers(regs);
            if( result ) {
                auto voltage = (regs.battery_voltage_h8 << 4) | (regs.battery_voltage_l4 & 0x0f);
                lcd.printf("voltage: %03x\n", voltage);
            }
            else {
                lcd.printf("voltage: error\n");
            }
        }
    }
}

#define TAG "MAIN"
extern "C" void app_main(void)
{
    // Initialie i2c_internal
    {
        i2c_config_t i2c_config;
        i2c_config.mode = I2C_MODE_MASTER;
        i2c_config.sda_io_num = GPIO_NUM_21;
        i2c_config.sda_pullup_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
        i2c_config.scl_io_num = GPIO_NUM_22;
        i2c_config.scl_pullup_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
        i2c_config.master.clk_speed = 400000;
        auto result = i2c_internal.initialize(i2c_config);
        if( !result ) ESP_ERROR_CHECK(result.error_code);
    }
    // Initialie i2c_external
    {
        i2c_config_t i2c_config;
        i2c_config.mode = I2C_MODE_MASTER;
        i2c_config.sda_io_num = GPIO_NUM_0;
        i2c_config.sda_pullup_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
        i2c_config.scl_io_num = GPIO_NUM_26;
        i2c_config.scl_pullup_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
        i2c_config.master.clk_speed = 400000;
        auto result = i2c_external.initialize(i2c_config);
        if( !result ) ESP_ERROR_CHECK(result.error_code);
    }

    // Do I2C Scan
    // {
    //     ESP_LOGI(TAG, "Scanning I2C Bus begin...");
    //     for(std::uint8_t device_address = 0; device_address <= 0x7f; device_address++ ) {
    //         I2CCommandLink commands;
    //         if( !commands ) continue;

    //         auto result = commands.start()
    //             .then([&commands, device_address](){ return commands.write_byte((device_address << 1) | 0, true); })
    //             .then([&commands](){ return commands.stop(); })
    //             .then([&commands](){ return i2c_external.execute(commands, pdMS_TO_TICKS(50)); });
    //         if( result.is_success ) {
    //             ESP_LOGI(TAG, "Address %02x: OK", device_address);
    //         }
    //         else if( result.error_code == ESP_ERR_TIMEOUT ) {
    //             ESP_LOGI(TAG, "Address %02x: Timed out", device_address);
    //         }
    //     }
    //     ESP_LOGI(TAG, "Scanning I2C end");
    // }

    // Initialize PMU
    {
        auto result = pmu.reset();
        if( !result ) {
            ESP_LOGE(TAG, "Failed to initialize PMU - %x", result.error_code);
        }
    }

    // Initialize buttons
    {
        auto result = buttons.initialize();
        if( !result ) {
            ESP_LOGE(TAG, "Failed to initialize buttons - %x", result.error_code);
        }
    }

    // Initializing LCD
    lcd.begin();
    lcd.setRotation(3);
    
    while(true) {
        switch(main_state) {
        case MainState::Initializing:
            do_initializing();
            break;
        
        case MainState::FatalError:
            do_fatal_error();
            break;
        
        case MainState::ModeSelecting:
            do_modeselecting();
            break;

        case MainState::SensorConnecting:
            do_sensor_connecting();
            break;

        case MainState::SensorConnected:
            do_sensor_connected();
            break;
        
        case MainState::ReceiverConnecting:
            do_receiver_connecting();
            break;
        
        case MainState::ReceiverConnected:
            do_receiver_connected();
            break;
        
        case MainState::Testing:
            do_testing();
            break;
        default:
            fatal_error("Unknown state - %d", main_state);
            break;
        }
    }
}
