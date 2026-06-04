#pragma once
#include "spi_bmi055_protocol.hpp"
class BMI055 
{
  public:
    SPI_BMI055_Protocol m_spi;
    //===========读取寄存器==========//
    bool BMI055_init();
    bool BMI055_stop();
    bool acc_read_chip_id(uint8_t &acc_chip_id);
    bool acc_write_unfilter_accd;
    bool acc_get_accd_x_mg(uint8_t &acc_accd_x_lsb,uint8_t &acc_accd_x_msb); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_get_accd_y_mg(uint8_t &acc_accd_y_lsb,uint8_t &acc_accd_y_msb); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_get_accd_z_mg(uint8_t &acc_accd_z_lsb,uint8_t &acc_accd_z_msb); // 将读数转化单位为mg后直接写入accd_x_mg
    bool acc_set_data_output_unfiltered();
    float acc_get_mg(uint8_t lsb, uint8_t msb);

    int m_acc_accd_x_mg,m_acc_accd_y_mg,m_acc_accd_z_mg;

};
