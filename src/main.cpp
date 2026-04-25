#include <iostream>
#include "spi_bmi055_protocol.hpp"
#include "bmi055_driver.hpp"
int main() 
{
	SPI_BMI055_Protocol SPI_test;
	SPI_test.spi_init();
	std::cout << "Hello,World!" <<std::endl;
	return 0;
    
}
