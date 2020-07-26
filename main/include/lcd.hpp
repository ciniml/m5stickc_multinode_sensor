#ifndef LCD_HPP__

#include <LovyanGFX.hpp>


struct LGFX_Config {
    static constexpr spi_host_device_t spi_host = VSPI_HOST;
    static constexpr int dma_channel = 1;
    static constexpr int spi_mosi = 15;
    static constexpr int spi_miso = 14;
    static constexpr int spi_sclk = 13;
};

typedef lgfx::LGFX_SPI<LGFX_Config> LCD;
typedef lgfx::Panel_M5StickC Panel;

#endif // LCD_HPP__