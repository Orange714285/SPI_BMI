#pragma once
#include "spi_bmi055_protocol.hpp"
class BMI055 
{
  public:
    SPI_BMI055_Protocol m_spi;

    int index = 0;

    bool BMI055_init();
    bool BMI055_stop();
    bool acc_config_drdy();
    bool gyr_config_drdy();

    bool acc_read_chip_id(uint8_t &acc_chip_id);
    bool acc_self_test();    
    bool acc_wait_for_new_info();
    bool gyr_wait_for_new_info();
    bool acc_set_data_output_unfiltered();
    float acc_get_mg(uint8_t lsb, uint8_t msb);
    bool acc_get_accd_x_mg(); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_get_accd_y_mg(); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_get_accd_z_mg(); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_get_accd_all_mg(); // 将读数转化单位为mg后直接写入accd_x_mg
    float m_acc_imu_accd_x_mg,m_acc_imu_accd_y_mg,m_acc_imu_accd_z_mg;
    float m_gyr_rate_x_dps,m_gyr_rate_y_dps,m_gyr_rate_z_dps;
  
    bool gyr_set_data_output_unfiltered();
    bool gyr_get_rate_x_deg_per_s(); //
    bool gyr_get_rate_y_deg_per_s(); // 
    bool gyr_get_rate_z_deg_per_s(); // 将读数转化单位为mg后直接写入accd_x_mg
    float gyr_get_deg_per_s(uint8_t lsb, uint8_t msb);
    bool gyr_get_rate_all_deg_per_s(); // 将读数转化单位为mg后直接写入accd_x_mg


};
