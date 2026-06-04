#include <iostream>

#include "bmi055_driver.hpp"
#include "bmi055_driver_def.hpp"

bool BMI055::BMI055_init()
{
    if(!m_spi.spi_init())
    {
        std::cerr << "[ERROR] SPI init Failed!" << std::endl;
        return false;
    }
    return true;
}
bool BMI055::acc_set_data_output_unfiltered()
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read acc chip id failed! spi start failed!" <<std::endl;
        return false;
    }

    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_HBW,dummy))
    {
        std::cerr << "[ERROR] Set acc unfiltered data failed! spi swap byte failed! (1)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x80,dummy))
    {
        std::cerr << "[ERROR] Set acc unfiltered data failed! spi swap byte failed! (2)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Set acc unfiltered data failed!  spi stop failed! " <<std::endl;
        return false;
    }
}

bool BMI055::acc_read_chip_id(uint8_t &acc_chip_id)
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read acc chip id failed! spi start failed!" <<std::endl;
        return false;
    }
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_BGW_CHIPID|0x80,dummy))
    {
        std::cerr << "[ERROR] Read acc chip id failed! spi swap byte failed! (1)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_chip_id))
    {
        std::cerr << "[ERROR] Read acc chip id failed! spi swap byte  failed! (2)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read acc chip id failed! spi stop failed! " <<std::endl;
        return false;
    }
    return true;
}

bool BMI055::acc_get_accd_x_mg(uint8_t &acc_accd_x_lsb,uint8_t &acc_accd_x_msb)
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read accd_x failed! spi start failed!" <<std::endl;
        return false;
    }
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_X_LSB|0x80,dummy))
    {
        std::cerr << "[ERROR] Read accd_x failed! spi swap byte failed! (1)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_x_lsb))
    {
        std::cerr << "[ERROR] Read accd_x failed! spi swap byte  failed! (2)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_x_msb))
    {
        std::cerr << "[ERROR] Read accd_x failed! spi swap byte  failed! (3)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read accd_x failed! spi stop failed! " <<std::endl;
        return false;
    }
    m_acc_accd_x_mg = acc_get_mg(acc_accd_x_lsb,acc_accd_x_msb);
    return true;
}

bool BMI055::acc_get_accd_y_mg(uint8_t &acc_accd_y_lsb,uint8_t &acc_accd_y_msb)
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read accd_y failed! spi start failed!" <<std::endl;
        return false;
    }
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_Y_LSB|0x80,dummy))
    {
        std::cerr << "[ERROR] Read accd_y failed! spi swap byte failed! (1)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_y_lsb))
    {
        std::cerr << "[ERROR] Read accd_y failed! spi swap byte  failed! (2)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_y_msb))
    {
        std::cerr << "[ERROR] Read accd_y failed! spi swap byte  failed! (3)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read accd_y failed! spi stop failed! " <<std::endl;
        return false;
    }
    m_acc_accd_y_mg = acc_get_mg(acc_accd_y_lsb,acc_accd_y_msb);
    return true;
}

bool BMI055::acc_get_accd_z_mg(uint8_t &acc_accd_z_lsb,uint8_t &acc_accd_z_msb)
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read accd_z failed! spi start failed!" <<std::endl;
        return false;
    }
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_Z_LSB|0x80,dummy))
    {
        std::cerr << "[ERROR] Read accd_z failed! spi swap byte failed! (1)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_z_lsb))
    {
        std::cerr << "[ERROR] Read accd_z failed! spi swap byte  failed! (2)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_z_msb))
    {
        std::cerr << "[ERROR] Read accd_z failed! spi swap byte  failed! (3)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read accd_z failed! spi stop failed! " <<std::endl;
        return false;
    }
    m_acc_accd_z_mg = acc_get_mg(acc_accd_z_lsb,acc_accd_z_msb);
    return true;
}

float BMI055::acc_get_mg(uint8_t lsb, uint8_t msb)
{
    uint16_t raw_12bit = ((uint16_t)msb << 4) | (lsb >> 4);
    int16_t acc_raw;
    if (raw_12bit & 0x0800)  
        acc_raw = (int16_t)(raw_12bit - 4096);
    else
        acc_raw = (int16_t)raw_12bit;
    float acc_mg = acc_raw * 0.98f;
    return acc_mg;
}

