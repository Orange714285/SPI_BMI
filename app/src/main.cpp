#include <iostream>
#include "spi_bmi055_protocol.hpp"
#include "bmi055_driver.hpp"
#include "attitude_algorithm.hpp"
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

std::atomic<bool> g_running{true};

void signal_handler(int signum)
{
    g_running.store(false);
}

int main() 
{
	BMI055 bmi055;
	if (!bmi055.BMI055_init())
	{
		std::cerr << "[ERROR] BMI055 init failed!" << std::endl;
		return 0;
	}
	std::cerr << "[INFO] BMI055 init successed!" << std::endl;

	signal(SIGINT, signal_handler);
	std::cout << "\033[2J\033[H";

	while (g_running)
	{	
		bmi055.index ++;
		if (!bmi055.acc_wait_for_new_info())
		{
			std::cerr << "[ERROR] acc_wait_for_new_info failed! Exiting loop." << std::endl;
			return 0;
		}
		if (!bmi055.acc_get_accd_all_mg())
		{
			std::cerr << "[ERROR] acc_get_accd_mg failed! Exiting loop." << std::endl;
			return 0;		
		}

		AttitudeAlgorithmer attitude_algorithmer(bmi055.m_acc_accd_x_mg,bmi055.m_acc_accd_y_mg,bmi055.m_acc_accd_z_mg);
		std::cout << "\033[H\033[J";
		std::cout << "============================" << std::endl;
		std::cout << "[INFO] accd_x_mg:" << bmi055.m_acc_accd_x_mg << std::endl;
		std::cout << "[INFO] accd_y_mg:" << bmi055.m_acc_accd_y_mg << std::endl;
		std::cout << "[INFO] accd_z_mg:" << bmi055.m_acc_accd_z_mg << std::endl;
		std::cout << "[INFO] pitch:"  << attitude_algorithmer.m_pitch << std::endl;		
		std::cout << "[INFO] roll:"   << attitude_algorithmer.m_roll  << std::endl;
		std::cout << "[INFO] index:"  << bmi055.index <<std::endl;
		std::cout << attitude_algorithmer.m_FRD_gravity_vec <<std::endl;
		std::cout << std::flush;
	}
	std::cout << "[INFO] Received SIGINT, exiting cleanly." << std::endl;
	return 0;
    
}
