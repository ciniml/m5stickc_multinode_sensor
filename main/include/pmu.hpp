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
    
    std::uint8_t screen_brightness;

    static constexpr std::uint8_t AXP192_REG_EXTEN_DCDC2_OUTPUT_CONFIG = 0x10;
    static constexpr std::uint8_t AXP192_REG_OUTPUT_ENABLE = 0x12;
    static constexpr std::uint8_t AXP192_REG_LDO2_LDO3_OUTPUT_VOLTAGE = 0x28;
    static constexpr std::uint8_t AXP192_REG_ADC_ENABLE_1 = 0x82;
    static constexpr std::uint8_t AXP192_REG_VOFF_SHUTDOWN_VOLTAGE_CONFIG = 0x31;
    static constexpr std::uint8_t AXP192_REG_CHARGING_CONFIG = 0x33;
    static constexpr std::uint8_t AXP192_REG_DCDC_OPERATING_MODE = 0x80;
    static constexpr std::uint8_t AXP192_REG_COULOMB_COUNTER_CHARGE    = 0xB0;
    static constexpr std::uint8_t AXP192_REG_COULOMB_COUNTER_DISCHARGE = 0xB4;
    static constexpr std::uint8_t AXP192_REG_COULOMB_COUNTER_CONTROL = 0xB8;
    static constexpr std::uint8_t AXP192_REG_ADC_RATE_TS_CONTROL = 0x84;
    static constexpr std::uint8_t AXP192_REG_PEK_CONFIG = 0x36;
    static constexpr std::uint8_t AXP192_REG_GPIO0_FUNCTION = 0x90;
    static constexpr std::uint8_t AXP192_REG_VBUS_IPSOUT_PATH_CONFIG = 030;
    static constexpr std::uint8_t AXP192_REG_VHTF_CHARGE_THRESHOLD = 0x39;
    static constexpr std::uint8_t AXP192_REG_BACKUP_BATTERY_CHARGING = 0x35;
    static constexpr std::uint8_t AXP192_REG_ADC_BASE = 0x56;

    static constexpr std::uint8_t TFT_OUTPUT_VALUE = 0x0c;  // TFT Output Voltage = 1.8 + 0.1*12 = 3.0[V]

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
    struct ADCRegisters
    {
        std::uint8_t acin_voltage_h8;           // 0x56
        std::uint8_t acin_voltage_l4;
        std::uint8_t acin_current_h8;
        std::uint8_t acin_current_l4;
        std::uint8_t vbus_voltage_h8;
        std::uint8_t vbus_voltage_l4;
        std::uint8_t vbus_current_h8;
        std::uint8_t vbus_current_l4;
        std::uint8_t internal_tempreature_h8;
        std::uint8_t internal_tempreature_l4;
        std::uint8_t reserved0[2];
        std::uint8_t ts_input_h8;               // 0x62
        std::uint8_t ts_input_l4;
        std::uint8_t gpio0_voltage_h8;
        std::uint8_t gpio0_voltage_l4;
        std::uint8_t gpio1_voltage_h8;
        std::uint8_t gpio1_voltage_l4;
        std::uint8_t gpio2_voltage_h8;
        std::uint8_t gpio2_voltage_l4;
        std::uint8_t gpio3_voltage_h8;
        std::uint8_t gpio3_voltage_l4;
        std::uint8_t reserved1[4];
        std::uint8_t battery_instant_voltage_h8; // 0x70
        std::uint8_t battery_instant_voltage_m8;
        std::uint8_t battery_instant_voltage_l8;
        std::uint8_t reserved2[5];
        std::uint8_t battery_voltage_h8;        // 0x78
        std::uint8_t battery_voltage_l4;
        std::uint8_t battery_charge_current_h8;
        std::uint8_t battery_charge_current_l5;
        std::uint8_t battery_discharge_current_h8;
        std::uint8_t battery_discharge_current_l5;
        std::uint8_t aps_voltage_h8;
        std::uint8_t aps_voltage_l4;
    };

    static constexpr std::uint8_t MAX_SCREEN_BRIGHTNESS = 0xc;

    PMU(I2CMaster& i2c, std::uint8_t address) : i2c(i2c), address(address), screen_brightness(MAX_SCREEN_BRIGHTNESS) {}

    std::uint8_t get_screen_brightness() const { return this->screen_brightness; }
    Result<void, esp_err_t> set_screen_brightness(std::uint8_t brightness) 
    {
        if( brightness > MAX_SCREEN_BRIGHTNESS ) {
            brightness = MAX_SCREEN_BRIGHTNESS;
        }

        RESULT_TRY(this->write_single_register(AXP192_REG_LDO2_LDO3_OUTPUT_VOLTAGE, TFT_OUTPUT_VALUE | (brightness << 4)));
        this->screen_brightness = brightness;
        return success();
    }
    
    Result<void, esp_err_t> enable_power_outputs()
    {
        RESULT_TRY(this->write_single_register(AXP192_REG_OUTPUT_ENABLE, 0x4d));            // Enable EXTEN, LDO3, LDO2, DC-DC1 output
        return success();
    }

    Result<void, esp_err_t> disable_power_outputs()
    {
        RESULT_TRY(this->write_single_register(AXP192_REG_OUTPUT_ENABLE, 0x01));            // Enable DC-DC1 output only
        return success();
    }

    Result<void, esp_err_t> enable_coulomb_counter()
    {
        return this->write_single_register(AXP192_REG_COULOMB_COUNTER_CONTROL, 0x80);
    }

    Result<void, esp_err_t> disable_coulomb_counter()
    {
        return this->write_single_register(AXP192_REG_COULOMB_COUNTER_CONTROL, 0x00);
    }

    Result<void, esp_err_t> stop_coulomb_counter()
    {
        return this->write_single_register(AXP192_REG_COULOMB_COUNTER_CONTROL, 0xc0);
    }

    Result<void, esp_err_t> clear_coulomb_counter()
    {
        return this->write_single_register(AXP192_REG_COULOMB_COUNTER_CONTROL, 0xa0);
    }

    Result<std::uint32_t, esp_err_t> get_coulomb_charge_counter_raw()
    {
        uint8_t buffer[4];
        RESULT_TRY(this->i2c.read_register(this->address, AXP192_REG_COULOMB_COUNTER_CHARGE, buffer, 4, DEFAULT_REG_TIMEOUT));

        return success(  (static_cast<std::uint32_t>(buffer[3]) << 24)
                       | (static_cast<std::uint32_t>(buffer[2]) << 16)
                       | (static_cast<std::uint32_t>(buffer[1]) <<  8)
                       | (static_cast<std::uint32_t>(buffer[0]) <<  0)
        );
    }
    Result<std::uint32_t, esp_err_t> get_coulomb_discharge_counter_raw()
    {
        uint8_t buffer[4];
        RESULT_TRY(this->i2c.read_register(this->address, AXP192_REG_COULOMB_COUNTER_DISCHARGE, buffer, 4, DEFAULT_REG_TIMEOUT));

        return success(  (static_cast<std::uint32_t>(buffer[3]) << 24)
                       | (static_cast<std::uint32_t>(buffer[2]) << 16)
                       | (static_cast<std::uint32_t>(buffer[1]) <<  8)
                       | (static_cast<std::uint32_t>(buffer[0]) <<  0)
        );
    }
    
    Result<float, esp_err_t> get_coulomb_data()
    {
        uint8_t buffer[8];
        uint8_t adc_rate;

        RESULT_TRY( this->i2c.read_register(this->address, AXP192_REG_ADC_RATE_TS_CONTROL, &adc_rate, 1, i2c_ack_type_t::I2C_MASTER_LAST_NACK) );
        RESULT_TRY( this->i2c.read_register(this->address, AXP192_REG_COULOMB_COUNTER_CHARGE, buffer, 8, i2c_ack_type_t::I2C_MASTER_LAST_NACK) );

        std::int64_t charge = (static_cast<std::uint32_t>(buffer[0]) << 24)
                             | (static_cast<std::uint32_t>(buffer[1]) << 16)
                             | (static_cast<std::uint32_t>(buffer[2]) <<  8)
                             | (static_cast<std::uint32_t>(buffer[3]) <<  0);
        std::int64_t discharge = (static_cast<std::uint32_t>(buffer[4]) << 24)
                                | (static_cast<std::uint32_t>(buffer[5]) << 16)
                                | (static_cast<std::uint32_t>(buffer[6]) <<  8)
                                | (static_cast<std::uint32_t>(buffer[7]) <<  0);
        std::uint8_t adc_sample_rate_exp = adc_rate >> 6;
        std::uint8_t adc_sample_rate = 25 << adc_sample_rate_exp;

        return success( 65536 * 0.5f * (charge - discharge) / (3600.0f * adc_sample_rate) );
    }

    Result<void, esp_err_t> reset()
    {
        ESP_LOGD(TAG, "Configuring registers...");
        // Configure registers
        {
            RESULT_TRY(this->write_single_register(AXP192_REG_EXTEN_DCDC2_OUTPUT_CONFIG, 0xff));// Enable all DC-DC2 outputs
            RESULT_TRY(this->set_screen_brightness(MAX_SCREEN_BRIGHTNESS));                     // Initialize LDO2/LDO3 output.                               
            RESULT_TRY(this->write_single_register(AXP192_REG_ADC_ENABLE_1, 0xff));             // Enable all ADCs.
            RESULT_TRY(this->write_single_register(AXP192_REG_CHARGING_CONFIG, 0xc1));          // Enable charging, 190mA, 4.2V, 90%
            RESULT_TRY(this->enable_coulomb_counter());                                         // Enable coulomb counter (battery energy counter)
            RESULT_TRY(this->enable_power_outputs());                                           // Enable EXTEN, LDO3, LDO2, DC-DC1 output
            RESULT_TRY(this->write_single_register(AXP192_REG_PEK_CONFIG, 0x4d));               // Shutdown 4s, Shutdown with button press longer than 4s, 
            RESULT_TRY(this->write_single_register(AXP192_REG_GPIO0_FUNCTION, 0x02));           // GPIO0 = Low Noise LDO, 
            RESULT_TRY(this->write_single_register(AXP192_REG_VBUS_IPSOUT_PATH_CONFIG, 0xe0));  // No use N_VBUSen, Limit Vbus voltage = 4.4V, Limit Vbus current = 500mA
            RESULT_TRY(this->write_single_register(AXP192_REG_VHTF_CHARGE_THRESHOLD, 0xfc));    // 
            RESULT_TRY(this->write_single_register(AXP192_REG_BACKUP_BATTERY_CHARGING, 0xa2));  // Enable charging backup battery, 3.0V, 200uA 
        }
        
        ESP_LOGI(TAG, "Initialized");

        return success();
    }

    Result<void, esp_err_t> read_adc_registers(ADCRegisters& registers)
    {
        RESULT_TRY(this->i2c.read_register(this->address, AXP192_REG_ADC_BASE, reinterpret_cast<std::uint8_t*>(&registers), sizeof(ADCRegisters), DEFAULT_REG_TIMEOUT) );
        return success();
    }

    Result<void, esp_err_t> enable_wakeup_button()
    {
        auto value = this->read_single_register(AXP192_REG_VOFF_SHUTDOWN_VOLTAGE_CONFIG);
        if( !value ) {
            return failure(value);
        }
        RESULT_TRY(this->write_single_register(AXP192_REG_VOFF_SHUTDOWN_VOLTAGE_CONFIG, value.value | 0x08));
        return success();
    }
    Result<void, esp_err_t> disable_wakeup_button()
    {
        auto value = this->read_single_register(AXP192_REG_VOFF_SHUTDOWN_VOLTAGE_CONFIG);
        if( !value ) {
            return failure(value);
        }
        RESULT_TRY(this->write_single_register(AXP192_REG_VOFF_SHUTDOWN_VOLTAGE_CONFIG, value.value & ~0x08));
        return success();
    }
};


#endif //PMU_HPP__