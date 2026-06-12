#include <iostream>
#include "spi_bmi055_protocol.hpp"
#include "bmi055_driver.hpp"
#include "attitude_algorithm.hpp"
#include "capture.hpp"
#include "data_type.hpp"
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

std::atomic<bool> g_running{true};
int dart_state = 0;
void signal_handler(int signum)
{
    g_running.store(false);
}

int main() 
{
    BMI055 bmi055;
    AccAttitudeAlgorithmer acc_attitude_algorithmer;
    GyroAttitudeAlgorithmer gyro_attitude_algorithmer;
    CarData dart_data;
    Capture capturer;

    if (!capturer.init())
    {
        std::cerr << "[ERROR] Capturer init failed! " << std::endl;
        capturer.finish();
        return -1;
    }
    if (!bmi055.BMI055_init())
    {
        std::cerr << "[ERROR] BMI055 init failed!" << std::endl;
        return -1;
    }
    std::cerr << "[INFO] BMI055 init successed!" << std::endl;

    signal(SIGINT, signal_handler);
    std::cout << "\033[2J\033[H";

    enum State { PRE_FLIGHT, FLIGHT };
    State state = PRE_FLIGHT;
    while (g_running)
    {
        bmi055.index++;

        // ==================== 数据采集 ====================
        if (state == PRE_FLIGHT)
        {
            // 着陆态：以加速度计为时钟源
            if (!bmi055.acc_wait_for_new_info())
            {
                std::cerr << "[ERROR] acc_wait_for_new_info failed!" << std::endl;
                return 0;
            }
            if (!bmi055.acc_get_accd_all_mg())
            {
                std::cerr << "[ERROR] acc_get_accd_mg failed!" << std::endl;
                return 0;
            }
            if (!bmi055.gyr_get_rate_all_dps())
            {
                std::cerr << "[ERROR] gyro get all rate dps failed!" << std::endl;
                return 0;
        // ==================
            }
        }
        else // FLIGHT
        {
            // 飞行态：以陀螺仪为时钟源
            if (!bmi055.gyr_wait_for_new_info())
            {
                std::cerr << "[ERROR] gyr_wait_for_new_info failed!" << std::endl;
                return 0;
            }
            if (!bmi055.gyr_get_rate_all_dps())
            {
                std::cerr << "[ERROR] gyr_get_rate_all_deg_per_s failed!" << std::endl;
                return 0;
            }
            if (!bmi055.acc_get_accd_all_mg())
            {
                std::cerr << "[ERROR] acc_get_accd_mg failed!" << std::endl;
                return 0;
            }
        }
		// ================ imu -> frd 转系 ================
        acc_attitude_algorithmer.transform_coordinate(
                bmi055.m_acc_imu_accd_x_mg,
                bmi055.m_acc_imu_accd_y_mg,
                bmi055.m_acc_imu_accd_z_mg);
        gyro_attitude_algorithmer.transform_coordinate(
                bmi055.m_gyr_rate_x_dps,
                bmi055.m_gyr_rate_y_dps,
                bmi055.m_gyr_rate_z_dps);
		// ==================== 姿态解算 ====================
        if (state == PRE_FLIGHT)
        {
            acc_attitude_algorithmer.algorithmer();
        }
        else
        {
            gyro_attitude_algorithmer.algorithm(
                acc_attitude_algorithmer.m_roll,
                acc_attitude_algorithmer.m_pitch);
        }

        // ==================== 状态切换 ====================
        if (state == PRE_FLIGHT && acc_attitude_algorithmer.m_frd_acc_x < -900)
        {
            std::cout << "[INFO] I have been Shooted! I am flying!" << std::endl;
            state = FLIGHT;
        }

        // ==================== 统一输出 ====================
        float roll  = (state == PRE_FLIGHT) ? acc_attitude_algorithmer.m_roll  : gyro_attitude_algorithmer.m_roll;
        float pitch = (state == PRE_FLIGHT) ? acc_attitude_algorithmer.m_pitch : gyro_attitude_algorithmer.m_pitch;
        float yaw   = (state == PRE_FLIGHT) ? 0.0f : gyro_attitude_algorithmer.m_yaw;

        dart_data.data_update(
            acc_attitude_algorithmer.m_frd_acc_x,
            acc_attitude_algorithmer.m_frd_acc_y,
            acc_attitude_algorithmer.m_frd_acc_z,
            gyro_attitude_algorithmer.m_frd_gyro_x,
            gyro_attitude_algorithmer.m_frd_gyro_y,
            gyro_attitude_algorithmer.m_frd_gyro_z,
            roll, pitch, yaw,
            bmi055.index);
        capturer.update(dart_data);
        dart_data.print(state == PRE_FLIGHT ? "PRE_FLIGHT" : "FLIGHT");
        

    }
    capturer.finish();
    std::cout << "[INFO] Received SIGINT, exiting cleanly." << std::endl;
    return 0;
}
