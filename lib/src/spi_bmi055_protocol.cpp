#include "spi_bmi055_protocol.hpp"
#include <gpiod.h>
#include <iostream>
#include <chrono>   
#include <thread>
bool SPI_BMI055_Protocol::spi_init()
{
    const unsigned int output_pins[] = {m_GPIO_CS_GYRO, m_GPIO_CS_ACCEL, m_GPIO_SPI_MOSI, m_GPIO_SPI_CLK};
    const unsigned int input_pins[] = {m_GPIO_SPI_MISO};
    const unsigned int acc_interrupt_pins[] = {m_GPIO_INT_ACCEL};

    // ============================== libgpiod 引脚初始化 ===============================//
    // 打开芯片
    m_chip = gpiod_chip_open(m_chip_path);
    if (!m_chip) {
        std::cout << "Failed to open chip " << std::endl;
        goto error;
    }

    // 线路设置 上拉输入
    m_line_settings_GPIO_MODE_IPU = gpiod_line_settings_new();
    if (!m_line_settings_GPIO_MODE_IPU) {
        std::cout << "Failed to get line settings GPIO MODE IPU" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_direction(m_line_settings_GPIO_MODE_IPU, GPIOD_LINE_DIRECTION_INPUT) < 0) {
        std::cout << "Failed to set line settings GPIO_MODE_IPU direction!" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_bias(m_line_settings_GPIO_MODE_IPU, GPIOD_LINE_BIAS_PULL_UP) < 0) {
        std::cout << "Failed to set line settings GPIO_MODE_IPU bias" << std::endl;
        goto error;
    }

    // 线路设置 推挽输出
    m_line_settings_GPIO_MODE_OUT_PP = gpiod_line_settings_new();
    if (!m_line_settings_GPIO_MODE_OUT_PP) {
        std::cout << "Failed to get line settings GPIO_MODE_OUTPUT_PP" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_direction(m_line_settings_GPIO_MODE_OUT_PP, GPIOD_LINE_DIRECTION_OUTPUT) < 0) {
        std::cout << "Failed to set line settings GPIO_MODE_OUTPUT_PP direction" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_drive(m_line_settings_GPIO_MODE_OUT_PP, GPIOD_LINE_DRIVE_PUSH_PULL) < 0) {
        std::cout << "Failed to set line settings GPIO_MODE_OUTPUT_PP drive" << std::endl;
        goto error;
    }
    // 线路设置 中断
    m_line_settings_GPIO_ACCEL_INTERRUPT = gpiod_line_settings_new();
    if (!m_line_settings_GPIO_ACCEL_INTERRUPT) 
    {
        std::cout << "Failed to get line settings GPIO_ACCEL_INTERRUPT" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_direction(m_line_settings_GPIO_ACCEL_INTERRUPT, GPIOD_LINE_DIRECTION_INPUT) < 0) 
    {
        std::cout << "Failed to set line settings GPIO_ACCEL_INTERRUPT direction!" << std::endl;
        goto error;
    }
    if (gpiod_line_settings_set_bias(m_line_settings_GPIO_ACCEL_INTERRUPT, GPIOD_LINE_BIAS_PULL_UP) < 0) 
    {
        std::cout << "Failed to set line settings GPIO_MODE_IPU bias" << std::endl;
        goto error;
    }
    if(gpiod_line_settings_set_edge_detection(m_line_settings_GPIO_ACCEL_INTERRUPT,GPIOD_LINE_EDGE_BOTH)<0)
    {
        std::cerr<< "Failed to set line settings GPIO_MODE_ACCEL_INTERRUPT edge detection" ;
        goto error;
    }

    // 线路配置
    m_line_config_BMI055 = gpiod_line_config_new();
    if (gpiod_line_config_add_line_settings(m_line_config_BMI055, output_pins, 4, m_line_settings_GPIO_MODE_OUT_PP) <
        0) {
        std::cout << "Failed to add output line settings" << std::endl;
        goto error;
    }
    if (gpiod_line_config_add_line_settings(m_line_config_BMI055, input_pins, 1, m_line_settings_GPIO_MODE_IPU) < 0) {
        std::cout << "Failed to add input line settings" << std::endl;
        goto error;
    }
    m_line_config_acc_interrupt_BMI055 = gpiod_line_config_new();
    if (gpiod_line_config_add_line_settings(m_line_config_acc_interrupt_BMI055,acc_interrupt_pins,1,m_line_settings_GPIO_ACCEL_INTERRUPT)<0)
    {
        std::cerr << "Failed to add acc_interrupt line settings" <<std::endl;
        goto error;
    }

    // 线路请求
    m_line_request_BMI055 = gpiod_chip_request_lines(m_chip, nullptr, m_line_config_BMI055);
    if (!m_line_request_BMI055) {
        std::cout << "Faild to get line request " << std::endl;
        goto error;
    }
    m_line_request_acc_interrupt_BMI055 = gpiod_chip_request_lines(m_chip,nullptr,m_line_config_acc_interrupt_BMI055);
    {
        std::cerr << "Failed to get line request acc_interrupt " << std::endl;
        goto error;
    }

    // 置初始化默认电平
    if(!spi_write_cs_gyro(1))
    {
        std::cerr << "Failed to init spi ! Failed to write cs gyro !" << std::endl;
        goto error;
    }
    if(!spi_write_cs_accel(1))
    {
        std::cerr << "Failed to init spi ! Failed to write cs accel !" << std::endl;
        goto error;
    }
    if(!spi_write_spi_clk(0))
    {
        std::cerr << "Failed to init spi ! Failed to write clk !" << std::endl;
        goto error;
    }

    

    return true;

error:

    if (m_line_request_BMI055)
        gpiod_line_request_release(m_line_request_BMI055);
    if (m_line_request_acc_interrupt_BMI055)
        gpiod_line_request_release(m_line_request_acc_interrupt_BMI055);

    if (m_line_config_BMI055)
        gpiod_line_config_free(m_line_config_BMI055);
    if (m_line_config_acc_interrupt_BMI055)
        gpiod_line_config_free(m_line_config_acc_interrupt_BMI055);


    if (m_line_settings_GPIO_MODE_IPU)
        gpiod_line_settings_free(m_line_settings_GPIO_MODE_IPU);
    if (m_line_settings_GPIO_MODE_OUT_PP)
        gpiod_line_settings_free(m_line_settings_GPIO_MODE_OUT_PP);
    if  (m_line_settings_GPIO_ACCEL_INTERRUPT)
        gpiod_line_settings_free(m_line_settings_GPIO_ACCEL_INTERRUPT);

    if (m_chip)
        gpiod_chip_close(m_chip);
    return false;
}

bool SPI_BMI055_Protocol::spi_write_cs_gyro(int line_value) {
    if (line_value == 0) {
        if (gpiod_line_request_set_value(m_line_request_BMI055, m_GPIO_CS_GYRO, GPIOD_LINE_VALUE_INACTIVE) < 0) {
            std::cout << "[ERROR] Failed to write cs gyro , failed to set line request value! " << std::endl;
            return false;
        } else
            return true;

    } else if (line_value == 1) {
        if (gpiod_line_request_set_value(m_line_request_BMI055, m_GPIO_CS_GYRO, GPIOD_LINE_VALUE_ACTIVE) < 0) {
            std::cout << "[ERROR] Failed to write cs gyro , failed to set line request value! " << std::endl;
            return false;
        } else
            return true;

    } else {
        std::cout << "[ERROR] Failed to write cs gyro , Illegal level ! " << std::endl;
        return false;
    }
}

bool SPI_BMI055_Protocol::spi_write_cs_accel(int line_value) {
    if (line_value == 0) {
        if (gpiod_line_request_set_value(m_line_request_BMI055, m_GPIO_CS_ACCEL, GPIOD_LINE_VALUE_INACTIVE) < 0) {
            std::cout << "[ERROR] Failed to write cs accel , failed to set line request value! " << std::endl;
            return false;
        } else
            return true;

    } else if (line_value == 1) {
        if (gpiod_line_request_set_value(m_line_request_BMI055, m_GPIO_CS_ACCEL, GPIOD_LINE_VALUE_ACTIVE) < 0) {
            std::cout << "[ERROR] Failed to write cs accel , failed to set line request value! " << std::endl;
            return false;
        } else
            return true;

    } else {
        std::cout << "[ERROR] Failed to write cs accel , Illegal level ! " << std::endl;
        return false;
    }
}

bool SPI_BMI055_Protocol::spi_write_spi_mosi(int line_value) {
    if (line_value == 0) 
    {
        if (gpiod_line_request_set_value(m_line_request_BMI055, m_GPIO_SPI_MOSI, GPIOD_LINE_VALUE_INACTIVE) < 0) 
        {
            std::cout << "[ERROR] Failed to write spi mosi , failed to set line request value! " << std::endl;
            return false;
        } 
        else
            return true;

    } 
    else if (line_value == 1) {
        if (gpiod_line_request_set_value(m_line_request_BMI055, m_GPIO_SPI_MOSI, GPIOD_LINE_VALUE_ACTIVE) < 0) 
        {
            std::cout << "[ERROR] Failed to write spi mosi , failed to set line request value! " << std::endl;
            return false;
        } 
        else
            return true;

    } 
    else 
    {
        std::cout << "[ERROR] Failed to write spi mosi , Illegal level ! " << std::endl;
        return false;
    }
}

bool SPI_BMI055_Protocol::spi_write_spi_clk(int line_value) {
    if (line_value == 0) {
        if (gpiod_line_request_set_value(m_line_request_BMI055, m_GPIO_SPI_CLK, GPIOD_LINE_VALUE_INACTIVE) < 0) {
            std::cout << "[ERROR] Failed to write spi clk , failed to set line request value! " << std::endl;
            return false;
        } else
            return true;

    } else if (line_value == 1) {
        if (gpiod_line_request_set_value(m_line_request_BMI055, m_GPIO_SPI_CLK, GPIOD_LINE_VALUE_ACTIVE) < 0) {
            std::cout << "[ERROR] Failed to write cs gyro , failed to set line request value! " << std::endl;
            return false;
        } else
            return true;
    } else {
        std::cout << "[ERROR] Failed to write cs gyro , Illegal level ! " << std::endl;
        return false;
    }
}

int SPI_BMI055_Protocol::spi_read_spi_miso(void) {
    int nRet = gpiod_line_request_get_value(m_line_request_BMI055, m_GPIO_SPI_MISO);
    if (nRet == GPIOD_LINE_VALUE_ACTIVE) {
        return 1;
    } else if (nRet == GPIOD_LINE_VALUE_INACTIVE) {
        return 0;
    } else {
        return -1;
    }
}

bool SPI_BMI055_Protocol::spi_accel_start(void) {
    if (!spi_write_spi_clk(0))
    {
        std::cerr << "[ERROR] Failed to spi accel start! Failed to write spi clk!" << std::endl;
        return false;
    }

    if (!spi_write_cs_gyro(1)) 
    {
        std::cerr << "[ERROR] Failed to spi accel start! Failed to write cs gyro!" << std::endl;
        return false;
    }

    if (!spi_write_cs_accel(0)) 
    {
        std::cerr << "[ERROR] Failed to spi accel start! Failed to write cs accel!" << std::endl;
        return false;
    }


    return true;
}

bool SPI_BMI055_Protocol::spi_stop(void) {
    if (!spi_write_cs_accel(1)) {
        std::cout << "[ERROR] Failed to spi start! Failed to write cs accel!" << std::endl;
        return false;
    }

    if (!spi_write_cs_gyro(1)) {
        std::cout << "[ERROR] Failed to spi start! Failed to write cs gyro!" << std::endl;
        return false;
    }

    if (!spi_write_spi_clk(0))
    {
        std::cerr << "[ERROR] Failed to spi accel start! Failed to write spi clk!" << std::endl;
        return false;
    }
    return true;
}

bool SPI_BMI055_Protocol::spi_swap_byte(uint8_t byte_send,uint8_t &byte_receive) // SPI Mode 0
{

    byte_receive = 0x00;
    for (int i = 0; i <= 7; i++) {
        int bit_send = (byte_send & (0x80 >> i)) ? 1 : 0;
        if (!spi_write_spi_mosi(bit_send))  
        {
            std::cerr << "[ERROR] Write SPI MOSI failed! " << std::endl;
            return false;
        }

        if (!spi_write_spi_clk(1)) 
        {
            std::cerr << "[ERROR] Write SPI CLK failed! " << std::endl;
            return false;   
        }
        
        int bit_receive = spi_read_spi_miso();
        if (bit_receive == 1) {
            byte_receive |= 0x80 >> i;
        } 
        else if (bit_receive < 0) 
        {
            std::cerr << "[ERROR] Read SPI MISO failed! " << std::endl;
            return false;
        }

        if (!spi_write_spi_clk(0)) 
        {
            std::cerr << "[ERROR] Write SPI CLK failed! " << std::endl;
            return false;
        }
    }

    return true;
}
