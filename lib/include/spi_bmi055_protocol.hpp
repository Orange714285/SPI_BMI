#pragma once
#include <gpiod.h>
#include <vector>
#include <cstdint>

class SPI_BMI055_Protocol {
  private:
    // 硬件 SPI 文件描述符
    int m_fd_acc = -1;
    int m_fd_gyr = -1;
    uint32_t m_spi_speed = 8000000; // 8MHz SPI 频率

    // 事务缓存数据结构
    enum class Device { NONE, ACCEL, GYRO };
    Device m_active_device = Device::NONE;
    std::vector<uint8_t> m_tx_buf;
    std::vector<uint8_t*> m_rx_ptrs;

    const char *const m_chip_path = "/dev/gpiochip0";
    struct gpiod_chip *m_chip = nullptr;
    struct gpiod_line_settings *m_line_settings_GPIO_ACCEL_INTERRUPT = nullptr;
    struct gpiod_line_settings *m_line_settings_GPIO_GYRO_INTERRUPT  = nullptr;

    struct gpiod_line_config *m_line_config_acc_interrupt_BMI055 = nullptr;
    struct gpiod_line_config *m_line_config_gyr_interrupt_BMI055 = nullptr;

  public:
    struct gpiod_line_request *m_line_request_acc_interrupt_BMI055 = nullptr;
    struct gpiod_line_request *m_line_request_gyr_interrupt_BMI055 = nullptr;
  private:
    const unsigned int m_GPIO_INT_GYRO  = 16;
    const unsigned int m_GPIO_INT_ACCEL = 17;

  public:
    SPI_BMI055_Protocol() = default;
    ~SPI_BMI055_Protocol();
    bool spi_init();

    // 保持兼容的接口声明，但内部实现将精简
    bool spi_write_cs_gyro(int line_value);
    bool spi_write_cs_accel(int line_value);
    int  spi_read_spi_miso(void);
    bool spi_write_spi_mosi(int line_value);
    bool spi_write_spi_clk(int line_value);

    bool spi_accel_start();
    bool spi_gyro_start();
    bool spi_stop();
    bool spi_swap_byte(uint8_t byte_send, uint8_t &byte_receive);
    bool spi_swap_byte(uint8_t byte_send);
  
};
