#include <iostream>
#include "spi_bmi055_protocol.hpp"
#include "bmi055_driver.hpp"
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
	std::cout << "The Chip ID is 0x" << std::hex << (int)acc_chip_id << std::endl;
	while (1)
	{
		
	}
	
	uint8_t acc_accd_x_lsb,acc_accd_x_msb;
	if (!bmi055.acc_get_accd_x_mg(acc_accd_x_lsb,acc_accd_x_msb))
	{
		std::cerr << "[ERROR] BMI055 read acc_chip_id failed!" << std::endl;
		return 0;		
	}
	
	return 0;
    
}
