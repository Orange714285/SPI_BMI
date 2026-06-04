#pragma once
#include <gpiod.h>
class SPI_BMI055_Protocol {
  private:
    const char *const m_chip_path = "/dev/gpiochip0";
    struct gpiod_chip *m_chip = nullptr;
    struct gpiod_line_settings *m_line_settings_GPIO_MODE_IPU = nullptr;
    struct gpiod_line_settings *m_line_settings_GPIO_MODE_OUT_PP = nullptr;
    struct gpiod_line_settings *m_line_settings_GPIO_ACCEL_INTERRUPT = nullptr;

    struct gpiod_line_config *m_line_config_BMI055 = nullptr;
    struct gpiod_line_config *m_line_config_acc_interrupt_BMI055 = nullptr;

    struct gpiod_line_request *m_line_request_BMI055 = nullptr;
  public:
    struct gpiod_line_request *m_line_request_acc_interrupt_BMI055 = nullptr;
  private:
    const unsigned int m_GPIO_CS_GYRO   = 7;
    const unsigned int m_GPIO_CS_ACCEL  = 8;
    const unsigned int m_GPIO_SPI_MISO  = 9;
    const unsigned int m_GPIO_SPI_MOSI  = 10;
    const unsigned int m_GPIO_SPI_CLK   = 11;
    const unsigned int m_GPIO_INT_GYRO  = 16;
    const unsigned int m_GPIO_INT_ACCEL = 17;

  public:
    SPI_BMI055_Protocol() = default;
    ~SPI_BMI055_Protocol() = default;
    bool spi_init();
    bool spi_write_cs_gyro(int line_value);
    bool spi_write_cs_accel(int line_value);
    int  spi_read_spi_miso(void);
    bool spi_write_spi_mosi(int line_value);
    bool spi_write_spi_clk(int line_value);

    bool spi_accel_start(void);
    bool spi_stop(void);
    bool spi_swap_byte(uint8_t byte_send, uint8_t &byte_receive);
    bool spi_swap_byte(uint8_t byte_send);
  
};
