#include <iostream>
#include "spi_bmi055_protocol.hpp"
#include "bmi055_driver.hpp"
#include "attitude_algorithm.hpp"
#include <chrono>
#include <thread>
int main() 
{
	BMI055 bmi055;
	if (!bmi055.BMI055_init())
	{
		std::cerr << "[ERROR] BMI055 init failed!" << std::endl;
		return 0;
	}
	std::cerr << "[INFO] BMI055 init successed!" << std::endl;

	uint8_t acc_chip_id;
	if (!bmi055.acc_read_chip_id(acc_chip_id))
	{
		std::cerr << "[ERROR] BMI055 read acc_chip_id failed!" << std::endl;
		return 0;		
	}
	std::cout << "The Chip ID is 0x" << std::hex << (int)acc_chip_id << std::dec << std::endl;
	while (1)
	{
		if (!bmi055.acc_wait_for_new_info())
		{
			std::cerr << "[ERROR] acc_wait_for_new_info failed! Exiting loop." << std::endl;
			return 0;
		}
		if (!bmi055.acc_get_accd_x_mg())
		{
			std::cerr << "[ERROR] acc_get_accd_x_mg failed! Exiting loop." << std::endl;
			return 0;		
		}
		if (!bmi055.acc_get_accd_y_mg())
		{
			std::cerr << "[ERROR] acc_get_accd_y_mg failed! Exiting loop." << std::endl;
			return 0;		
		}
		if (!bmi055.acc_get_accd_z_mg())
		{
			std::cerr << "[ERROR] acc_get_accd_z_mg failed! Exiting loop." << std::endl;
			return 0;		
		}
		std::cout << "============================" << std::endl;
		std::cout << "[INFO] accd_x_mg:" << bmi055.m_acc_accd_x_mg << std::endl;
		std::cout << "[INFO] accd_y_mg:" << bmi055.m_acc_accd_y_mg << std::endl;
		std::cout << "[INFO] accd_z_mg:" << bmi055.m_acc_accd_z_mg << std::endl;
		
	}
	return 0;
    
}
