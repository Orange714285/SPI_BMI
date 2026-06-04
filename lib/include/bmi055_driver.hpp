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

    bool acc_read_chip_id(uint8_t &acc_chip_id);
    bool acc_self_test();    
    bool acc_wait_for_new_info();
    bool acc_set_data_output_unfiltered();
    float acc_get_mg(uint8_t lsb, uint8_t msb);
    bool acc_get_accd_x_mg(); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_get_accd_y_mg(); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_get_accd_z_mg(); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_get_accd_all_mg(); // 将读数转化单位为mg后直接写入accd_x_mg


    float m_acc_accd_x_mg,m_acc_accd_y_mg,m_acc_accd_z_mg;

};
