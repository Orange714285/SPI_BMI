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

		AccAttitudeAlgorithmer acc_attitude_algorithmer(bmi055.m_acc_imu_accd_x_mg,bmi055.m_acc_imu_accd_y_mg,bmi055.m_acc_imu_accd_z_mg);
		std::cout << "\033[H\033[J";
		std::cout << "============================" << std::endl;
		std::cout << "[INFO] imu_accd_x_mg:" << bmi055.m_acc_imu_accd_x_mg << std::endl;
		std::cout << "[INFO] imu_accd_y_mg:" << bmi055.m_acc_imu_accd_y_mg << std::endl;
		std::cout << "[INFO] imu_accd_z_mg:" << bmi055.m_acc_imu_accd_z_mg << std::endl;
		std::cout << "[INFO] frd_accd_x_mg:" << acc_attitude_algorithmer.m_frd_acc_x << std::endl;
		std::cout << "[INFO] frd_accd_y_mg:" << acc_attitude_algorithmer.m_frd_acc_y << std::endl;
		std::cout << "[INFO] frd_accd_z_mg:" << acc_attitude_algorithmer.m_frd_acc_z << std::endl;
		std::cout << "[INFO] pitch:"  << acc_attitude_algorithmer.m_pitch << std::endl;		
		std::cout << "[INFO] roll:"   << acc_attitude_algorithmer.m_roll  << std::endl;
		std::cout << "[INFO] index:"  << bmi055.index <<std::endl;
		std::cout << "============================" << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << std::flush;
		if (acc_attitude_algorithmer.m_frd_acc_x < -900)
		break;
	}
	
	std::cout << "[INFO] I have been Shooted! I am flying!" << std::endl;

	while (g_running)
	{	
		bmi055.index ++;
		if (!bmi055.gyr_wait_for_new_info())
		{
			std::cerr << "[ERROR] gyr_wait_for_new_info failed! Exiting loop." << std::endl;
			return 0;
		}
		if (!bmi055.gyr_get_rate_all_deg_per_s())
		{
			std::cerr << "[ERROR] gyr_get_rate_all_deg_per_s failed! Exiting loop." << std::endl;
			return 0;		
		}

		std::cout << "\033[H\033[J";
		std::cout << "============================" << std::endl;
		std::cout << "[INFO] gyr_rate_x_dps:" << bmi055.m_gyr_rate_x_dps << std::endl;
		std::cout << "[INFO] gyr_rate_y_dps:" << bmi055.m_gyr_rate_y_dps << std::endl;
		std::cout << "[INFO] gyr_rate_z_dps:" << bmi055.m_gyr_rate_z_dps << std::endl;
		std::cout << "[INFO] index:"  << bmi055.index <<std::endl;
		std::cout << "============================" << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << std::flush;
	}

	std::cout << "[INFO] Received SIGINT, exiting cleanly." << std::endl;
	return 0;
	    
}
