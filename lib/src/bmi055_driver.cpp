#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include "bmi055_driver.hpp"
#include "bmi055_driver_def.hpp"

bool BMI055::BMI055_init()
{
    // spi 初始化
    if (!m_spi.spi_init())
    {
        std::cerr << "[ERROR] SPI init failed!" << std::endl;
        return false;
    }
    // 加速度计 陀螺仪自检
    if (!acc_self_test())
	{
		std::cerr << "[ERROR] BMI055 acc self test failed!" << std::endl;
		return false;
	}
    if (!gyr_self_test())
    {
        std::cerr << "[ERROR] BMI055 gyro self test failed!" << std::endl;
        return false;
    }

    // 配置加速度计
    if (!acc_set_data_output_unfiltered())
    {
        std::cerr << "[ERROR] Set acc data output unfiltered failed!" << std::endl;
        return false;
    }
    if (!acc_config_drdy())
    {
        std::cerr << "[ERROR] Config acc drdy failed!" << std::endl;
        return false;
    }
    if (!acc_set_measuring_range())
    {
        std::cerr << "[ERROR] set_measuring_range failed!" << std::endl;
    }
    if (!acc_set_data_refresh_frequency())
    {
        std::cerr << "[ERROR] set acc data refresh frequency failed!" << std::endl;
    }
    // 配置陀螺仪
    if (!gyr_set_data_output_unfiltered())
    {
        std::cerr << "[ERROR] Set gyr data output unfiltered failed!" << std::endl;
        return false;
    }
    if (!gyr_config_drdy())
    {
        std::cerr << "[ERROR] Config gyr drdy failed!" << std::endl;
        return false;
    }
    // if (!gyr_measure_zero_bias())
    // {
    //     std::cerr << "[ERROR] BMI055 gyr measure zero bias failed!" << std::endl;
    //     return false;
    // }
    if (!gyr_set_data_refresh_frequency())
    {
        std::cerr << "[ERROR] set gyr data refresh frequency failed!" << std::endl;
    }

    return true;
}

bool BMI055::acc_config_drdy()
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Config acc drdy failed! Spi start failed!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_INT_OUT_CTRL))
    {
        std::cerr << "[ERROR] Config acc drdy failed! spi swap byte failed! (ACC_INT_OUT_CTRL)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x05))
    {
        std::cerr << "[ERROR] Config acc drdy failed! spi swap byte failed! (0x05)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Config acc drdy failed! Spi stop failed!" << std::endl;
        return false;
    }

    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Config acc drdy failed! Spi start failed!" << std::endl;
        return false;
    }   
    if (!m_spi.spi_swap_byte(ACC_INT_MAP_1))
    {
        std::cerr << "[ERROR] Config acc drdy failed! spi swap byte failed! (ACC_INT_MAP_1)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x01))
    {   
        std::cerr << "[ERROR] Config acc drdy failed! spi swap byte failed! (0x01)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Config acc drdy failed! Spi stop failed!" << std::endl;
        return false;
    }

    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Config acc drdy failed! Spi start failed!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_INT_EN_1))
    {
        std::cerr << "[ERROR] Config acc drdy failed! spi swap byte failed! (ACC_INT_EN_1)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x10))
    {
        std::cerr << "[ERROR] Config acc drdy failed! spi swap byte failed! (0x10)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Config acc drdy failed! Spi stop failed!" << std::endl;
        return false;
    }
    return true;
}

bool BMI055::gyr_config_drdy()
{
    // Step 1: INT3 push-pull, non-latched (GYR_INT_RST_LATCH = 0x00)
    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Config gyr drdy failed! Spi start failed!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_INT_RST_LATCH))
    {
        std::cerr << "[ERROR] Config gyr drdy failed! spi swap byte failed! (GYR_INT_RST_LATCH)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x00))
    {
        std::cerr << "[ERROR] Config gyr drdy failed! spi swap byte failed! (0x00)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Config gyr drdy failed! Spi stop failed!" << std::endl;
        return false;
    }

    // Step 2: map data interrupt to INT3 (GYR_INT_MAP_2[7] = 1)
    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Config gyr drdy failed! Spi start failed!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_INT_MAP_1))
    {
        std::cerr << "[ERROR] Config gyr drdy failed! spi swap byte failed! (GYR_INT_MAP_2)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x01))
    {
        std::cerr << "[ERROR] Config gyr drdy failed! spi swap byte failed! (0x80)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Config gyr drdy failed! Spi stop failed!" << std::endl;
        return false;
    }


    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Config gyr drdy failed! Spi start failed!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_INT_EN_1))
    {
        std::cerr << "[ERROR] Config gyr drdy failed! spi swap byte failed! (GYR_INT_EN_1)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x01))
    {
        std::cerr << "[ERROR] Config gyr drdy failed! spi swap byte failed! (0x01)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Config gyr drdy failed! Spi stop failed!" << std::endl;
        return false;
    }
    // Step 3: enable new data interrupt (GYR_INT_EN_0[7] = 1)
    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Config gyr drdy failed! Spi start failed!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_INT_EN_0))
    {
        std::cerr << "[ERROR] Config gyr drdy failed! spi swap byte failed! (GYR_INT_EN_0)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x80))
    {
        std::cerr << "[ERROR] Config gyr drdy failed! spi swap byte failed! (0x80)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Config gyr drdy failed! Spi stop failed!" << std::endl;
        return false;
    }

    return true;
}

bool BMI055::acc_wait_for_new_info()
{
    int nRet = gpiod_line_request_wait_edge_events(m_spi.m_line_request_acc_interrupt_BMI055,1000000000);
    if(nRet == 0)
    {
        std::cerr << "[WARNING] Time out! Failed to get new acc_info " << std::endl; 
        return false;
    }
    else if (nRet==-1)
    {
        std::cerr << "[ERROR] Error occurred! Failed to get new acc_info " << std::endl;
        return false;
    }
    else if (nRet == 1)
    {
        return true;
    }
    else 
    {
        std::cerr << "[ERROR] Unknown error! Failed to get new acc_info " << std::endl;
        return false;
    }
}

bool BMI055::gyr_wait_for_new_info()
{
    int nRet = gpiod_line_request_wait_edge_events(m_spi.m_line_request_gyr_interrupt_BMI055, 1000000000);
    if (nRet == 0)
    {
        std::cerr << "[WARNING] Time out! Failed to get new gyr_info " << std::endl;
        return false;
    }
    else if (nRet == -1)
    {
        std::cerr << "[ERROR] Error occurred! Failed to get new gyr_info " << std::endl;
        return false;
    }
    else if (nRet == 1)
    {
        return true;
    }
    else
    {
        std::cerr << "[ERROR] Unknown error! Failed to get new gyr_info " << std::endl;
        return false;
    }
}

bool BMI055::acc_set_data_refresh_frequency()
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Set acc data refresh frequency failed! spi start failed!" << std::endl;
        return false;
    }
    
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_PMU_BW,dummy))
    {
        std::cerr << "[ERROR] Set acc data refresh frequency failed! spi swap byte failed! (ACC_PMU_BW)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x0F,dummy))
    {
        std::cerr << "[ERROR] Set acc data refresh frequency failed! spi swap byte failed! (ACC_PMU_BW)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Set acc data refresh frequency failed! spi stop failed!" <<std::endl;
        return false;
    }
    return true;
}

bool BMI055::gyr_set_data_refresh_frequency()
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Set acc data refresh frequency failed! spi start failed!" << std::endl;
        return false;
    }
    
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(GYR_BW,dummy))
    {
        std::cerr << "[ERROR] Set acc data refresh frequency failed! spi swap byte failed! (ACC_PMU_BW)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x80,dummy))
    {
        std::cerr << "[ERROR] Set acc data refresh frequency failed! spi swap byte failed! (ACC_PMU_BW)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Set acc data refresh frequency failed! spi stop failed!" <<std::endl;
        return false;
    }
    return true;
}

bool BMI055::acc_set_measuring_range()
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Set acc measuring range failed! spi start failed!" << std::endl;
        return false;
    }
    
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_HBW,dummy))
    {
        std::cerr << "[ERROR] Set acc measuring range failed! spi swap byte failed! (ACC_ACCD_HBW)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x03,dummy))
    {
        std::cerr << "[ERROR] Set acc measuring range failed! spi swap byte failed! (ACC_ACCD_HBW)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Set acc measuring range failed! spi stop failed!" <<std::endl;
        return false;
    }
    return true;
}

bool BMI055::acc_set_data_output_unfiltered()
{
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Set acc unfiltered data failed! spi start failed!" << std::endl;
        return false;
    }

    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_HBW,dummy))
    {
        std::cerr << "[ERROR] Set acc unfiltered data failed! spi swap byte failed! (ACC_ACCD_HBW)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x80,dummy))
    {
        std::cerr << "[ERROR] Set acc unfiltered data failed! spi swap byte failed! (0x80)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Set acc unfiltered data failed! spi stop failed!" <<std::endl;
        return false;
    }
    return true;
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
        std::cerr << "[ERROR] Read acc chip id failed! spi swap byte failed! (ACC_BGW_CHIPID|0x80)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_chip_id))
    {
        std::cerr << "[ERROR] Read acc chip id failed! spi swap byte failed! (read chip_id)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read acc chip id failed! spi stop failed!" <<std::endl;
        return false;
    }
    return true;
}

bool BMI055::acc_get_accd_x_mg()
{
    uint8_t acc_accd_x_lsb,acc_accd_x_msb;
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read accd_x failed! spi start failed!" <<std::endl;
        return false;
    }
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_X_LSB|0x80,dummy))
    {
        std::cerr << "[ERROR] Read accd_x failed! spi swap byte failed! (ACC_ACCD_X_LSB|0x80)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_x_lsb))
    {
        std::cerr << "[ERROR] Read accd_x failed! spi swap byte failed! (read accd_x_lsb)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_x_msb))
    {
        std::cerr << "[ERROR] Read accd_x failed! spi swap byte failed! (read accd_x_msb)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read accd_x failed! spi stop failed!" <<std::endl;
        return false;
    }
    m_acc_imu_accd_x_mg = acc_get_mg(acc_accd_x_lsb,acc_accd_x_msb);
    return true;
}

bool BMI055::acc_get_accd_y_mg()
{
    uint8_t acc_accd_y_lsb,acc_accd_y_msb;
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read accd_y failed! spi start failed!" <<std::endl;
        return false;
    }
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_Y_LSB|0x80,dummy))
    {
        std::cerr << "[ERROR] Read accd_y failed! spi swap byte failed! (ACC_ACCD_Y_LSB|0x80)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_y_lsb))
    {
        std::cerr << "[ERROR] Read accd_y failed! spi swap byte failed! (read accd_y_lsb)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_y_msb))
    {
        std::cerr << "[ERROR] Read accd_y failed! spi swap byte failed! (read accd_y_msb)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read accd_y failed! spi stop failed!" <<std::endl;
        return false;
    }
    m_acc_imu_accd_y_mg = acc_get_mg(acc_accd_y_lsb,acc_accd_y_msb);
    return true;
}

bool BMI055::acc_get_accd_z_mg()
{
    uint8_t acc_accd_z_lsb,acc_accd_z_msb;
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read accd_z failed! spi start failed!" <<std::endl;
        return false;
    }
    uint8_t dummy = ACC_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(ACC_ACCD_Z_LSB|0x80,dummy))
    {
        std::cerr << "[ERROR] Read accd_z failed! spi swap byte failed! (ACC_ACCD_Z_LSB|0x80)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_z_lsb))
    {
        std::cerr << "[ERROR] Read accd_z failed! spi swap byte failed! (read accd_z_lsb)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE,acc_accd_z_msb))
    {
        std::cerr << "[ERROR] Read accd_z failed! spi swap byte failed! (read accd_z_msb)" <<std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read accd_z failed! spi stop failed!" <<std::endl;
        return false;
    }
    m_acc_imu_accd_z_mg = acc_get_mg(acc_accd_z_lsb,acc_accd_z_msb);
    return true;
}

bool BMI055::acc_get_accd_all_mg()
{
    uint8_t acc_accd_x_lsb, acc_accd_x_msb;
    uint8_t acc_accd_y_lsb, acc_accd_y_msb;
    uint8_t acc_accd_z_lsb, acc_accd_z_msb;

    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] Read accd_all failed! spi start failed!" << std::endl;
        return false;
    }

    uint8_t dummy = ACC_DUMMY_BYTE;

    // Step 1: send starting register address ACC_ACCD_X_LSB with read bit (0x80) set
    if (!m_spi.spi_swap_byte(ACC_ACCD_X_LSB | 0x80, dummy))
    {
        std::cerr << "[ERROR] Read accd_all failed! spi swap byte failed! (ACC_ACCD_X_LSB|0x80)" << std::endl;
        return false;
    }

    // Step 2: burst read 6 bytes consecutively (auto-increment from 0x02 to 0x07)
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE, acc_accd_x_lsb))
    {
        std::cerr << "[ERROR] Read accd_all failed! spi swap byte failed! (read accd_x_lsb)" << std::endl;
        return false;       
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE, acc_accd_x_msb))
    {
        std::cerr << "[ERROR] Read accd_all failed! spi swap byte failed! (read accd_x_msb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE, acc_accd_y_lsb))
    {
        std::cerr << "[ERROR] Read accd_all failed! spi swap byte failed! (read accd_y_lsb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE, acc_accd_y_msb))
    {
        std::cerr << "[ERROR] Read accd_all failed! spi swap byte failed! (read accd_y_msb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE, acc_accd_z_lsb))
    {
        std::cerr << "[ERROR] Read accd_all failed! spi swap byte failed! (read accd_z_lsb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(ACC_DUMMY_BYTE, acc_accd_z_msb))
    {
        std::cerr << "[ERROR] Read accd_all failed! spi swap byte failed! (read accd_z_msb)" << std::endl;
        return false;
    }

    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read accd_all failed! spi stop failed!" << std::endl;
        return false;
    }

    // Step 3: convert raw data to mg
    m_acc_imu_accd_x_mg = acc_get_mg(acc_accd_x_lsb, acc_accd_x_msb);
    m_acc_imu_accd_y_mg = acc_get_mg(acc_accd_y_lsb, acc_accd_y_msb);
    m_acc_imu_accd_z_mg = acc_get_mg(acc_accd_z_lsb, acc_accd_z_msb);

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

bool BMI055::acc_self_test()
{
    /* ===== Step 1: set range to ±8g (required by datasheet) ===== */
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] acc_self_test: spi_accel_start failed when setting range to 8g!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x0F))  // ACC_PMU_RANGE
    {
        std::cerr << "[ERROR] acc_self_test: spi_swap_byte failed when setting range to 8g!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x01))  // ±8g (0x01 for 8g on register 0x0F)
    {
        std::cerr << "[ERROR] acc_self_test: spi_swap_byte failed when writing 8g value!" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] acc_self_test: spi_stop failed after setting range to 8g!" << std::endl;
        return false;
    }

    /* ===== Step 2: per-axis positive/negative self-test =====*/
    const uint8_t cmd[3][2] = {
        {0x11, 0x15},  // X-axis: neg=0x11, pos=0x15
        {0x12, 0x16},  // Y-axis: neg=0x12, pos=0x16
        {0x13, 0x17},  // Z-axis: neg=0x13, pos=0x17
    };
    const float   threshold_mg[3] = {800.0f, 800.0f, 400.0f};
    const char*   axis_name[3]     = {"X", "Y", "Z"};
    const char*   sign_name[2]     = {"neg", "pos"};
    float         val[3][2];  // [axis][sign]

    for (int axis = 0; axis < 3; axis++)
    {
        for (int sign = 0; sign < 2; sign++)
        {
            // enable self-test for this axis & sign
            if (!m_spi.spi_accel_start())
            {
                std::cerr << "[ERROR] acc_self_test: spi_accel_start failed enabling "
                          << axis_name[axis] << " " << sign_name[sign] << "!" << std::endl;
                return false;
            }
            if (!m_spi.spi_swap_byte(0x32))  // ACC_PMU_SELF_TEST
            {
                std::cerr << "[ERROR] acc_self_test: spi_swap_byte failed sending 0x32 for "
                          << axis_name[axis] << " " << sign_name[sign] << "!" << std::endl;
                return false;
            }
            if (!m_spi.spi_swap_byte(cmd[axis][sign]))
            {
                std::cerr << "[ERROR] acc_self_test: spi_swap_byte failed writing 0x"
                          << std::hex << (int)cmd[axis][sign] << std::dec << " for "
                          << axis_name[axis] << " " << sign_name[sign] << "!" << std::endl;
                return false;
            }
            if (!m_spi.spi_stop())
            {
                std::cerr << "[ERROR] acc_self_test: spi_stop failed after enabling "
                          << axis_name[axis] << " " << sign_name[sign] << "!" << std::endl;
                return false;
            }

            // wait >= 50ms for deflection to settle (datasheet requirement)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // read acceleration data
            if (!acc_get_accd_x_mg() || !acc_get_accd_y_mg() || !acc_get_accd_z_mg())
            {
                std::cerr << "[ERROR] acc_self_test: read acc failed for "
                          << axis_name[axis] << " " << sign_name[sign] << "!" << std::endl;
                return false;
            }
            val[axis][sign] = (axis == 0) ? m_acc_imu_accd_x_mg :
                              (axis == 1) ? m_acc_imu_accd_y_mg :
                                            m_acc_imu_accd_z_mg;

            // disable self-test (write 0x00 to 0x32)
            if (!m_spi.spi_accel_start())
            {
                std::cerr << "[ERROR] acc_self_test: spi_accel_start failed disabling "
                          << axis_name[axis] << " " << sign_name[sign] << "!" << std::endl;
                return false;
            }
            if (!m_spi.spi_swap_byte(0x32))
            {
                std::cerr << "[ERROR] acc_self_test: spi_swap_byte failed sending 0x32 to disable!" << std::endl;
                return false;
            }
            if (!m_spi.spi_swap_byte(0x00))  // disable
            {
                std::cerr << "[ERROR] acc_self_test: spi_swap_byte failed writing 0x00 to disable!" << std::endl;
                return false;
            }
            if (!m_spi.spi_stop())
            {
                std::cerr << "[ERROR] acc_self_test: spi_stop failed after disabling self-test!" << std::endl;
                return false;
            }
        }

        // check pos-neg difference against threshold
        float diff = std::fabs(val[axis][1] - val[axis][0]);
        if (diff < threshold_mg[axis])
        {
            std::cerr << "[ERROR] acc_self_test: " << axis_name[axis]
                      << "-axis FAILED! diff=" << diff
                      << " mg < " << threshold_mg[axis] << " mg" << std::endl;
            return false;
        }
        std::cout << "[INFO] acc_self_test: " << axis_name[axis]
                  << "-axis PASSED, diff=" << diff << " mg" << std::endl;
    }

    /* ===== Step 3: soft reset to restore normal operation ===== */
    if (!m_spi.spi_accel_start())
    {
        std::cerr << "[ERROR] acc_self_test: spi_accel_start failed doing soft reset!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x14))  // ACC_BGW_SOFTRESET
    {
        std::cerr << "[ERROR] acc_self_test: spi_swap_byte failed sending soft reset!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0xB6))  // soft reset key
    {
        std::cerr << "[ERROR] acc_self_test: spi_swap_byte failed writing soft reset key!" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] acc_self_test: spi_stop failed after soft reset!" << std::endl;
        return false;
    }

    std::cout << "[INFO] acc_self_test: ALL AXES PASSED!" << std::endl;
    return true;
}

bool BMI055::gyr_self_test()
{
    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] gyr_self_test: spi gyro start failed!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_BIST|0x80))
    {
        std::cerr << "[ERROR] gyr_self_test: spi swap byte failed! (GYR_BIST)" << std::endl;
        return false;
    }
    uint8_t receive = 0x00;
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE,receive))
    {
        std::cerr << "[ERROR] gyr_self_test: spi swap byte failed! (read GYR_BIST)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] gyr_self_test: spi stop failed!" << std::endl;
        return false;
    }
    if (!((receive >> 4) == 0x01))
    {
        std::cout << std::hex << static_cast<int>(receive) << std::dec << std::endl;      
        std::cerr << "[ERROR] gyr_self_test: sensor function has something wrong!" << std::endl;
        return false;
    }
    std::cout << "[INFO] gyr_self_test: PASSED!" << std::endl;
    return true;
}

bool BMI055::gyr_measure_zero_bias()
{
    const int sample_count = 500;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float sum_sq_x = 0.0f, sum_sq_y = 0.0f, sum_sq_z = 0.0f;
    float min_x = 0.0f, max_x = 0.0f;
    float min_y = 0.0f, max_y = 0.0f;
    float min_z = 0.0f, max_z = 0.0f;

    m_gyr_bias_x_dps = 0.0f;
    m_gyr_bias_y_dps = 0.0f;
    m_gyr_bias_z_dps = 0.0f;

    std::cerr << "[INFO] gyr_measure_zero_bias: collecting " << sample_count
              << " samples, please keep device stationary..." << std::endl;

    for (int i = 0; i < sample_count; i++)
    {
        if (!gyr_wait_for_new_info())
        {
            std::cerr << "[ERROR] gyr_measure_zero_bias: wait for new info failed at sample "
                      << i << "!" << std::endl;
            return false;
        }
        if (!gyr_get_rate_all_dps())
        {
            std::cerr << "[ERROR] gyr_measure_zero_bias: read rates failed at sample "
                      << i << "!" << std::endl;
            return false;
        }

        float rx = m_gyr_rate_x_dps;
        float ry = m_gyr_rate_y_dps;
        float rz = m_gyr_rate_z_dps;

        sum_x += rx;
        sum_y += ry;
        sum_z += rz;
        sum_sq_x += rx * rx;
        sum_sq_y += ry * ry;
        sum_sq_z += rz * rz;

        if (i == 0) {
            min_x = max_x = rx;
            min_y = max_y = ry;
            min_z = max_z = rz;
        } else {
            if (rx < min_x) min_x = rx;  if (rx > max_x) max_x = rx;
            if (ry < min_y) min_y = ry;  if (ry > max_y) max_y = ry;
            if (rz < min_z) min_z = rz;  if (rz > max_z) max_z = rz;
        }
    }

    m_gyr_bias_x_dps = sum_x / sample_count;
    m_gyr_bias_y_dps = sum_y / sample_count;
    m_gyr_bias_z_dps = sum_z / sample_count;

    float std_x = std::sqrt(sum_sq_x / sample_count - m_gyr_bias_x_dps * m_gyr_bias_x_dps);
    float std_y = std::sqrt(sum_sq_y / sample_count - m_gyr_bias_y_dps * m_gyr_bias_y_dps);
    float std_z = std::sqrt(sum_sq_z / sample_count - m_gyr_bias_z_dps * m_gyr_bias_z_dps);

    std::cerr << "[INFO] gyr zero bias measured (dps):" << std::endl;
    std::cerr << "      bias:    X=" << m_gyr_bias_x_dps
              << "  Y=" << m_gyr_bias_y_dps
              << "  Z=" << m_gyr_bias_z_dps << std::endl;
    std::cerr << "      std:     X=" << std_x
              << "  Y=" << std_y
              << "  Z=" << std_z << std::endl;
    std::cerr << "      max-min: X=[" << min_x << ", " << max_x << "]"
              << "  Y=[" << min_y << ", " << max_y << "]"
              << "  Z=[" << min_z << ", " << max_z << "]" << std::endl;
    return true;
}

bool BMI055::gyr_set_data_output_unfiltered()
{
    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Set gyr unfiltered data failed! spi start failed!" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_RATE_HBW))
    {
        std::cerr << "[ERROR] Set gyr unfiltered data failed! spi swap byte failed! (GYR_RATE_HBW)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(0x80))
    {
        std::cerr << "[ERROR] Set gyr unfiltered data failed! spi swap byte failed! (0x80)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Set gyr unfiltered data failed! spi stop failed!" << std::endl;
        return false;
    }
    return true;    
}

float BMI055::gyr_get_deg_per_s(uint8_t lsb, uint8_t msb)
{
    int16_t raw = ((int16_t)msb << 8) | lsb;
    float rate_dps = raw / 16.4f;
    return rate_dps;
}

bool BMI055::gyr_get_rate_x_deg_per_s()
{
    uint8_t gyr_rate_x_lsb, gyr_rate_x_msb;
    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Read gyr_rate_x failed! spi start failed!" << std::endl;
        return false;
    }
    uint8_t dummy = GYR_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(GYR_RATE_X_LSB | 0x80, dummy))
    {
        std::cerr << "[ERROR] Read gyr_rate_x failed! spi swap byte failed! (GYR_RATE_X_LSB|0x80)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_x_lsb))
    {
        std::cerr << "[ERROR] Read gyr_rate_x failed! spi swap byte failed! (read gyr_rate_x_lsb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_x_msb))
    {
        std::cerr << "[ERROR] Read gyr_rate_x failed! spi swap byte failed! (read gyr_rate_x_msb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read gyr_rate_x failed! spi stop failed!" << std::endl;
        return false;
    }
    float raw_x = gyr_get_deg_per_s(gyr_rate_x_lsb, gyr_rate_x_msb);
    m_gyr_rate_x_raw_dps = raw_x;
    m_gyr_rate_x_dps = raw_x - m_gyr_bias_x_dps;
    return true;
}

bool BMI055::gyr_get_rate_y_deg_per_s()
{
    uint8_t gyr_rate_y_lsb, gyr_rate_y_msb;
    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Read gyr_rate_y failed! spi start failed!" << std::endl;
        return false;
    }
    uint8_t dummy = GYR_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(GYR_RATE_Y_LSB | 0x80, dummy))
    {
        std::cerr << "[ERROR] Read gyr_rate_y failed! spi swap byte failed! (GYR_RATE_Y_LSB|0x80)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_y_lsb))
    {
        std::cerr << "[ERROR] Read gyr_rate_y failed! spi swap byte failed! (read gyr_rate_y_lsb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_y_msb))
    {
        std::cerr << "[ERROR] Read gyr_rate_y failed! spi swap byte failed! (read gyr_rate_y_msb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read gyr_rate_y failed! spi stop failed!" << std::endl;
        return false;
    }
    float raw_y = gyr_get_deg_per_s(gyr_rate_y_lsb, gyr_rate_y_msb);
    m_gyr_rate_y_raw_dps = raw_y;
    m_gyr_rate_y_dps = raw_y - m_gyr_bias_y_dps;
    return true;
}

bool BMI055::gyr_get_rate_z_deg_per_s()
{
    uint8_t gyr_rate_z_lsb, gyr_rate_z_msb;
    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Read gyr_rate_z failed! spi start failed!" << std::endl;
        return false;
    }
    uint8_t dummy = GYR_DUMMY_BYTE;
    if (!m_spi.spi_swap_byte(GYR_RATE_Z_LSB | 0x80, dummy))
    {
        std::cerr << "[ERROR] Read gyr_rate_z failed! spi swap byte failed! (GYR_RATE_Z_LSB|0x80)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_z_lsb))
    {
        std::cerr << "[ERROR] Read gyr_rate_z failed! spi swap byte failed! (read gyr_rate_z_lsb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_z_msb))
    {
        std::cerr << "[ERROR] Read gyr_rate_z failed! spi swap byte failed! (read gyr_rate_z_msb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read gyr_rate_z failed! spi stop failed!" << std::endl;
        return false;
    }
    float raw_z = gyr_get_deg_per_s(gyr_rate_z_lsb, gyr_rate_z_msb);
    m_gyr_rate_z_raw_dps = raw_z;
    m_gyr_rate_z_dps = raw_z - m_gyr_bias_z_dps;
    return true;
}

bool BMI055::gyr_get_rate_all_dps()
{
    uint8_t gyr_rate_x_lsb, gyr_rate_x_msb;
    uint8_t gyr_rate_y_lsb, gyr_rate_y_msb;
    uint8_t gyr_rate_z_lsb, gyr_rate_z_msb;

    if (!m_spi.spi_gyro_start())
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi start failed!" << std::endl;
        return false;
    }

    uint8_t dummy = GYR_DUMMY_BYTE;

    if (!m_spi.spi_swap_byte(GYR_RATE_X_LSB | 0x80, dummy))
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi swap byte failed! (GYR_RATE_X_LSB|0x80)" << std::endl;
        return false;
    }

    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_x_lsb))
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi swap byte failed! (read gyr_rate_x_lsb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_x_msb))
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi swap byte failed! (read gyr_rate_x_msb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_y_lsb))
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi swap byte failed! (read gyr_rate_y_lsb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_y_msb))
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi swap byte failed! (read gyr_rate_y_msb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_z_lsb))
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi swap byte failed! (read gyr_rate_z_lsb)" << std::endl;
        return false;
    }
    if (!m_spi.spi_swap_byte(GYR_DUMMY_BYTE, gyr_rate_z_msb))
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi swap byte failed! (read gyr_rate_z_msb)" << std::endl;
        return false;
    }

    if (!m_spi.spi_stop())
    {
        std::cerr << "[ERROR] Read gyr_rate_all failed! spi stop failed!" << std::endl;
        return false;
    }

    float raw_x = gyr_get_deg_per_s(gyr_rate_x_lsb, gyr_rate_x_msb);
    float raw_y = gyr_get_deg_per_s(gyr_rate_y_lsb, gyr_rate_y_msb);
    float raw_z = gyr_get_deg_per_s(gyr_rate_z_lsb, gyr_rate_z_msb);
    m_gyr_rate_x_raw_dps = raw_x;
    m_gyr_rate_y_raw_dps = raw_y;
    m_gyr_rate_z_raw_dps = raw_z;
    m_gyr_rate_x_dps = raw_x - m_gyr_bias_x_dps;
    m_gyr_rate_y_dps = raw_y - m_gyr_bias_y_dps;
    m_gyr_rate_z_dps = raw_z - m_gyr_bias_z_dps;

    return true;
}
