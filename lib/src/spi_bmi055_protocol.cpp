#include "spi_bmi055_protocol.hpp"
#include <gpiod.h>
#include <iostream>
#include <chrono>   
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

bool SPI_BMI055_Protocol::spi_init()
{
    const unsigned int acc_interrupt_pins[] = {m_GPIO_INT_ACCEL};
    const unsigned int gyr_interrupt_pins[] = {m_GPIO_INT_GYRO};

    // ============================== 硬件 SPI 初始化 ===============================//
    // 打开设备文件
    m_fd_acc = open("/dev/spidev0.0", O_RDWR);
    if (m_fd_acc < 0) {
        std::cerr << "[ERROR] Failed to open /dev/spidev0.0!" << std::endl;
        goto error;
    }
    m_fd_gyr = open("/dev/spidev0.1", O_RDWR);
    if (m_fd_gyr < 0) {
        std::cerr << "[ERROR] Failed to open /dev/spidev0.1!" << std::endl;
        goto error;
    }

    // 配置 SPI 参数 (SPI_MODE_0, 8 bits per word, 8MHz)
    {
        uint8_t mode = SPI_MODE_0;
        uint8_t bits = 8;
        uint32_t speed = m_spi_speed;

        if (ioctl(m_fd_acc, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(m_fd_gyr, SPI_IOC_WR_MODE, &mode) < 0) {
            std::cerr << "[ERROR] SPI_IOC_WR_MODE failed!" << std::endl;
            goto error;
        }
        if (ioctl(m_fd_acc, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(m_fd_gyr, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
            std::cerr << "[ERROR] SPI_IOC_WR_BITS_PER_WORD failed!" << std::endl;
            goto error;
        }
        if (ioctl(m_fd_acc, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 || ioctl(m_fd_gyr, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            std::cerr << "[ERROR] SPI_IOC_WR_MAX_SPEED_HZ failed!" << std::endl;
            goto error;
        }
    }

    // ============================== libgpiod 中断引脚初始化 ===============================//
    // 打开芯片
    m_chip = gpiod_chip_open(m_chip_path);
    if (!m_chip) {
        std::cerr << "[ERROR] Failed to open chip!" << std::endl;
        goto error;
    }

    // 线路设置 中断
    m_line_settings_GPIO_ACCEL_INTERRUPT = gpiod_line_settings_new();
    if (!m_line_settings_GPIO_ACCEL_INTERRUPT) 
    {
        std::cerr << "[ERROR] Failed to get line settings GPIO_ACCEL_INTERRUPT!" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_direction(m_line_settings_GPIO_ACCEL_INTERRUPT, GPIOD_LINE_DIRECTION_INPUT) < 0) 
    {
        std::cerr << "[ERROR] Failed to set line settings GPIO_ACCEL_INTERRUPT direction!" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_bias(m_line_settings_GPIO_ACCEL_INTERRUPT, GPIOD_LINE_BIAS_PULL_DOWN) < 0) 
    {
        std::cerr << "[ERROR] Failed to set line settings GPIO_ACCEL_INTERRUPT bias!" << std::endl;
        goto error;
    }
    if(gpiod_line_settings_set_edge_detection(m_line_settings_GPIO_ACCEL_INTERRUPT,GPIOD_LINE_EDGE_RISING)<0)
    {
        std::cerr << "[ERROR] Failed to set line settings GPIO_ACCEL_INTERRUPT edge detection!" << std::endl;
        goto error;
    }

    // 线路设置 GYRO中断
    m_line_settings_GPIO_GYRO_INTERRUPT = gpiod_line_settings_new();
    if (!m_line_settings_GPIO_GYRO_INTERRUPT)
    {
        std::cerr << "[ERROR] Failed to get line settings GPIO_GYRO_INTERRUPT!" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_direction(m_line_settings_GPIO_GYRO_INTERRUPT, GPIOD_LINE_DIRECTION_INPUT) < 0)
    {
        std::cerr << "[ERROR] Failed to set line settings GPIO_GYRO_INTERRUPT direction!" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_bias(m_line_settings_GPIO_GYRO_INTERRUPT, GPIOD_LINE_BIAS_PULL_DOWN) < 0)
    {
        std::cerr << "[ERROR] Failed to set line settings GPIO_GYRO_INTERRUPT bias!" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_edge_detection(m_line_settings_GPIO_GYRO_INTERRUPT, GPIOD_LINE_EDGE_RISING) < 0)
    {
        std::cerr << "[ERROR] Failed to set line settings GPIO_GYRO_INTERRUPT edge detection!" << std::endl;
        goto error;
    }

    // 线路配置
    m_line_config_acc_interrupt_BMI055 = gpiod_line_config_new();
    if (gpiod_line_config_add_line_settings(m_line_config_acc_interrupt_BMI055,acc_interrupt_pins,1,m_line_settings_GPIO_ACCEL_INTERRUPT)<0)
    {
        std::cerr << "[ERROR] Failed to add acc_interrupt line settings!" << std::endl;
        goto error;
    }
    m_line_config_gyr_interrupt_BMI055 = gpiod_line_config_new();
    if (gpiod_line_config_add_line_settings(m_line_config_gyr_interrupt_BMI055,gyr_interrupt_pins,1,m_line_settings_GPIO_GYRO_INTERRUPT)<0)
    {
        std::cerr << "[ERROR] Failed to add gyr_interrupt line settings!" << std::endl;
        goto error;
    }

    // 线路请求
    m_line_request_acc_interrupt_BMI055 = gpiod_chip_request_lines(m_chip,nullptr,m_line_config_acc_interrupt_BMI055);
    if (!m_line_request_acc_interrupt_BMI055) {
        std::cerr << "[ERROR] Failed to get line request acc_interrupt!" << std::endl;
        goto error;
    }
    m_line_request_gyr_interrupt_BMI055 = gpiod_chip_request_lines(m_chip,nullptr,m_line_config_gyr_interrupt_BMI055);
    if (!m_line_request_gyr_interrupt_BMI055) {
        std::cerr << "[ERROR] Failed to get line request gyr_interrupt!" << std::endl;
        goto error;
    }

    return true;

error:
    if (m_fd_acc >= 0) { close(m_fd_acc); m_fd_acc = -1; }
    if (m_fd_gyr >= 0) { close(m_fd_gyr); m_fd_gyr = -1; }

    if (m_line_request_acc_interrupt_BMI055) {
        gpiod_line_request_release(m_line_request_acc_interrupt_BMI055);
        m_line_request_acc_interrupt_BMI055 = nullptr;
    }
    if (m_line_request_gyr_interrupt_BMI055) {
        gpiod_line_request_release(m_line_request_gyr_interrupt_BMI055);
        m_line_request_gyr_interrupt_BMI055 = nullptr;
    }

    if (m_line_config_acc_interrupt_BMI055) {
        gpiod_line_config_free(m_line_config_acc_interrupt_BMI055);
        m_line_config_acc_interrupt_BMI055 = nullptr;
    }
    if (m_line_config_gyr_interrupt_BMI055) {
        gpiod_line_config_free(m_line_config_gyr_interrupt_BMI055);
        m_line_config_gyr_interrupt_BMI055 = nullptr;
    }

    if  (m_line_settings_GPIO_ACCEL_INTERRUPT) {
        gpiod_line_settings_free(m_line_settings_GPIO_ACCEL_INTERRUPT);
        m_line_settings_GPIO_ACCEL_INTERRUPT = nullptr;
    }
    if  (m_line_settings_GPIO_GYRO_INTERRUPT) {
        gpiod_line_settings_free(m_line_settings_GPIO_GYRO_INTERRUPT);
        m_line_settings_GPIO_GYRO_INTERRUPT = nullptr;
    }

    if (m_chip) {
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
    }
    return false;
}

bool SPI_BMI055_Protocol::spi_write_cs_gyro(int) { return true; }
bool SPI_BMI055_Protocol::spi_write_cs_accel(int) { return true; }
int  SPI_BMI055_Protocol::spi_read_spi_miso(void) { return 0; }
bool SPI_BMI055_Protocol::spi_write_spi_mosi(int) { return true; }
bool SPI_BMI055_Protocol::spi_write_spi_clk(int) { return true; }

bool SPI_BMI055_Protocol::spi_accel_start(void) {
    m_active_device = Device::ACCEL;
    m_tx_buf.clear();
    m_rx_ptrs.clear();
    return true;
}

bool SPI_BMI055_Protocol::spi_gyro_start(void) {
    m_active_device = Device::GYRO;
    m_tx_buf.clear();
    m_rx_ptrs.clear();
    return true;
}

bool SPI_BMI055_Protocol::spi_swap_byte(uint8_t byte_send, uint8_t &byte_receive)
{
    m_tx_buf.push_back(byte_send);
    m_rx_ptrs.push_back(&byte_receive);
    return true;
}

bool SPI_BMI055_Protocol::spi_swap_byte(uint8_t byte_send)
{
    m_tx_buf.push_back(byte_send);
    m_rx_ptrs.push_back(nullptr);
    return true;
}

bool SPI_BMI055_Protocol::spi_stop(void) {
    if (m_active_device == Device::NONE || m_tx_buf.empty()) {
        m_active_device = Device::NONE;
        m_tx_buf.clear();
        m_rx_ptrs.clear();
        return true;
    }

    int fd = (m_active_device == Device::ACCEL) ? m_fd_acc : m_fd_gyr;
    if (fd < 0) {
        std::cerr << "[ERROR] spi_stop: Active device FD is invalid!" << std::endl;
        m_active_device = Device::NONE;
        m_tx_buf.clear();
        m_rx_ptrs.clear();
        return false;
    }

    std::vector<uint8_t> rx_buf(m_tx_buf.size(), 0);

    struct spi_ioc_transfer tr;
    std::memset(&tr, 0, sizeof(tr));
    tr.tx_buf = reinterpret_cast<unsigned long>(m_tx_buf.data());
    tr.rx_buf = reinterpret_cast<unsigned long>(rx_buf.data());
    tr.len = m_tx_buf.size();
    tr.speed_hz = m_spi_speed;
    tr.bits_per_word = 8;
    tr.delay_usecs = 0;
    tr.cs_change = 0;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        std::cerr << "[ERROR] SPI transaction ioctl failed! device: "
                  << ((m_active_device == Device::ACCEL) ? "ACCEL" : "GYRO") << std::endl;
        m_active_device = Device::NONE;
        m_tx_buf.clear();
        m_rx_ptrs.clear();
        return false;
    }

    // 将收到的数据写回到调用者传入的引用中
    for (size_t i = 0; i < m_tx_buf.size(); ++i) {
        if (m_rx_ptrs[i] != nullptr) {
            *m_rx_ptrs[i] = rx_buf[i];
        }
    }

    m_active_device = Device::NONE;
    m_tx_buf.clear();
    m_rx_ptrs.clear();
    return true;
}

SPI_BMI055_Protocol::~SPI_BMI055_Protocol()
{
    if (m_fd_acc >= 0) {
        close(m_fd_acc);
        m_fd_acc = -1;
    }
    if (m_fd_gyr >= 0) {
        close(m_fd_gyr);
        m_fd_gyr = -1;
    }

    if (m_line_request_acc_interrupt_BMI055) {
        gpiod_line_request_release(m_line_request_acc_interrupt_BMI055);
        m_line_request_acc_interrupt_BMI055 = nullptr;
    }
    if (m_line_request_gyr_interrupt_BMI055) {
        gpiod_line_request_release(m_line_request_gyr_interrupt_BMI055);
        m_line_request_gyr_interrupt_BMI055 = nullptr;
    }

    if (m_line_config_acc_interrupt_BMI055) {
        gpiod_line_config_free(m_line_config_acc_interrupt_BMI055);
        m_line_config_acc_interrupt_BMI055 = nullptr;
    }
    if (m_line_config_gyr_interrupt_BMI055) {
        gpiod_line_config_free(m_line_config_gyr_interrupt_BMI055);
        m_line_config_gyr_interrupt_BMI055 = nullptr;
    }

    if  (m_line_settings_GPIO_ACCEL_INTERRUPT) {
        gpiod_line_settings_free(m_line_settings_GPIO_ACCEL_INTERRUPT);
        m_line_settings_GPIO_ACCEL_INTERRUPT = nullptr;
    }
    if  (m_line_settings_GPIO_GYRO_INTERRUPT) {
        gpiod_line_settings_free(m_line_settings_GPIO_GYRO_INTERRUPT);
        m_line_settings_GPIO_GYRO_INTERRUPT = nullptr;
    }

    if (m_chip) {
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
    }
    std::cerr << "[INFO] 释放libgpiod硬件和SPI描述符资源" << std::endl;
}