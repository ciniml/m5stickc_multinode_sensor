#ifndef PMU_HPP__
#define PMU_HPP__

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

class PMU
{
private:
    static constexpr const char* TAG = "PMU";

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
        ESP_LOGD(TAG, "Configuring registers...");
        // Configure registers
        {
            RESULT_TRY(this->write_single_register(AXP192_REG_EXTEN_DCDC2_OUTPUT_CONFIG, 0xff));// Enable all DC-DC2 outputs
            RESULT_TRY(this->write_single_register(AXP192_REG_LDO2_LDO3_OUTPUT_VOLTAGE, 0xcc)); // LDO2 = LED 3.0V, LDO3 = TFT 3.0V
            RESULT_TRY(this->write_single_register(AXP192_REG_ADC_ENABLE_1, 0xff));             // Enable all ADCs.
            RESULT_TRY(this->write_single_register(AXP192_REG_CHARGING_CONFIG, 0xc1));          // Enable charging, 190mA, 4.2V, 90%
            RESULT_TRY(this->write_single_register(AXP192_REG_COULOMB_COUNTER, 0x80));          // Enable coulomb counter (battery energy counter)
            RESULT_TRY(this->write_single_register(AXP192_REG_OUTPUT_ENABLE, 0x4d));            // Enable EXTEN, LDO3, LDO2, DC-DC1 output
            RESULT_TRY(this->write_single_register(AXP192_REG_OUTPUT_ENABLE, 0x4d));            // Enable EXTEN, LDO3, LDO2, DC-DC1 output
            RESULT_TRY(this->write_single_register(AXP192_REG_PEK_CONFIG, 0x4d));               // Shutdown 4s, Shutdown with button press longer than 4s, 
            RESULT_TRY(this->write_single_register(AXP192_REG_GPIO0_FUNCTION, 0x02));           // GPIO0 = Low Noise LDO, 
            RESULT_TRY(this->write_single_register(AXP192_REG_VBUS_IPSOUT_PATH_CONFIG, 0xe0));  // No use N_VBUSen, Limit Vbus voltage = 4.4V, Limit Vbus current = 500mA
            RESULT_TRY(this->write_single_register(AXP192_REG_VHTF_CHARGE_THRESHOLD, 0xfc));    // 
            RESULT_TRY(this->write_single_register(AXP192_REG_BACKUP_BATTERY_CHARGING, 0xa2));  // Enable charging backup battery, 3.0V, 200uA 
        }
        
        ESP_LOGI(TAG, "Initialized");

        return success();
    }
};


#endif //PMU_HPP__