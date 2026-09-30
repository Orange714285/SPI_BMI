#pragma once

#include <cstdint>

#include <Eigen/Core>

#include "spi_bmi055_protocol.hpp"

class BMI055
{
public:
    // 生命周期
    bool BMI055_init();
    bool BMI055_stop();

    // 加速度计配置与状态
    bool acc_read_chip_id(uint8_t& acc_chip_id);
    bool acc_self_test();
    bool acc_config_drdy();
    bool acc_set_data_output_unfiltered();
    bool acc_set_measuring_range();
    bool acc_set_data_refresh_frequency();
    bool acc_wait_for_new_info();

    // 加速度计数据读取，单位为 mg
    bool acc_get_accd_x_mg();
    bool acc_get_accd_y_mg();
    bool acc_get_accd_z_mg();
    bool acc_get_accd_all_mg();
    float acc_get_mg(uint8_t lsb, uint8_t msb);

    // 陀螺仪配置与状态
    bool gyr_self_test();
    bool gyr_config_drdy();
    bool gyr_set_data_output_unfiltered();
    bool gyr_set_data_refresh_frequency();
    bool gyr_wait_for_new_info();
    bool gyr_measure_zero_bias();

    // 陀螺仪数据读取，单位为度每秒（dps）
    bool gyr_get_rate_x_deg_per_s();
    bool gyr_get_rate_y_deg_per_s();
    bool gyr_get_rate_z_deg_per_s();
    bool gyr_get_rate_all_dps();
    float gyr_get_deg_per_s(uint8_t lsb, uint8_t msb);

    // 三轴测量结果，分量顺序为 X、Y、Z
    Eigen::Vector3f m_acc_imu_accd_mg = Eigen::Vector3f::Zero();
    Eigen::Vector3f m_gyr_rate_raw_dps = Eigen::Vector3f::Zero();
    Eigen::Vector3f m_gyr_rate_dps = Eigen::Vector3f::Zero();
    Eigen::Vector3f m_gyr_bias_dps = Eigen::Vector3f::Zero();
    Eigen::Matrix3f m_acc_calibration_matrix = Eigen::Matrix3f::Zero();
    Eigen::Vector3f m_acc_calibration_bias = Eigen::Vector3f::Zero();
    // 运行状态与底层通信
    int index = 0;
    SPI_BMI055_Protocol m_spi;
};
