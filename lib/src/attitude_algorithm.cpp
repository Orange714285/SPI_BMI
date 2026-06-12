#include "attitude_algorithm.hpp"
#include <cmath>
AccAttitudeAlgorithmer::AccAttitudeAlgorithmer()
{
    m_IMU_to_FRD_matrix << 0 , -1,  0,
                           1 ,  0,  0,
                           0 ,  0,  1; 
}
void AccAttitudeAlgorithmer::transform_coordinate(float acc_x, float acc_y, float acc_z)
{
    m_imu_acc_x = acc_x;
    m_imu_acc_y = acc_y;
    m_imu_acc_z = acc_z;

    m_IMU_acc_vec << m_imu_acc_x,
                     m_imu_acc_y,
                     m_imu_acc_z;

    m_FRD_acc_vec = m_IMU_to_FRD_matrix.inverse() * m_IMU_acc_vec;
    m_acc_g = sqrt(m_imu_acc_x * m_imu_acc_x + m_imu_acc_y * m_imu_acc_y + m_imu_acc_z * m_imu_acc_z);

    m_frd_acc_x = m_FRD_acc_vec[0];
    m_frd_acc_y = m_FRD_acc_vec[1];
    m_frd_acc_z = m_FRD_acc_vec[2];
}

void AccAttitudeAlgorithmer::algorithmer()
{
    m_pitch = atan2(m_FRD_acc_vec[0],
              sqrt(m_FRD_acc_vec[1] * m_FRD_acc_vec[1] + m_FRD_acc_vec[2] * m_FRD_acc_vec[2])) * 180 / M_PI;
    m_roll  = atan2(-m_FRD_acc_vec[1], -m_FRD_acc_vec[2]) * 180 / M_PI;
}

GyroAttitudeAlgorithmer::GyroAttitudeAlgorithmer()
{
    m_IMU_to_FRD_matrix << 0 , -1,  0,
                           1 ,  0,  0,
                           0 ,  0,  1; 
    m_initialized = false;
}
void GyroAttitudeAlgorithmer::transform_coordinate(float gyro_x, float gyro_y, float gyro_z)
{
    // ---- 存储原始 IMU 陀螺仪数据 (deg/s) ----
    m_imu_gyro_x = gyro_x;
    m_imu_gyro_y = gyro_y;
    m_imu_gyro_z = gyro_z;

    m_IMU_gyro_vec << m_imu_gyro_x,
                      m_imu_gyro_y,
                      m_imu_gyro_z;

    // ---- 将角速度从 IMU 坐标系转到 FRD 机体系 ----
    m_FRD_gyro_vec = m_IMU_to_FRD_matrix.inverse() * m_IMU_gyro_vec;
    m_frd_gyro_x = m_FRD_gyro_vec[0];
    m_frd_gyro_y = m_FRD_gyro_vec[1];
    m_frd_gyro_z = m_FRD_gyro_vec[2];

    // 记录陀螺数据的时刻，供 algorithm() 初始化 m_last_time 使用
    m_gyro_data_time = std::chrono::steady_clock::now();
}

void GyroAttitudeAlgorithmer::algorithm(float roll, float pitch)
{
    //  yaw 初始化为 0
    if (!m_initialized)
    {
        // 使用最近一次陀螺数据的时间戳，而非 now()，
        // 确保第一个 FLIGHT 积分步长的 dt 是准确的
        m_last_time = m_gyro_data_time;
        m_initialized = 1;
        // 初始欧拉角
        m_yaw   = 0.0f;
        m_pitch = pitch;
        m_roll  = roll;
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    m_dt_us = std::chrono::duration_cast<std::chrono::microseconds>(now - m_last_time);
    m_last_time = now;

    // dt 上限保护：若 dt 超过 50ms (如传感器丢帧或状态切换延迟)，截断到 50ms
    // 防止欧拉角积分因异常大的 dt 而爆炸
    constexpr long kMaxDtUs = 50000;
    long dt_us_actual = m_dt_us.count();
    if (dt_us_actual > kMaxDtUs)
    {
        dt_us_actual = kMaxDtUs;
        m_dt_us = std::chrono::microseconds(kMaxDtUs);
    }

    float dtheta_x = m_frd_gyro_x * dt_us_actual / 1000000.0f;
    float dtheta_y = m_frd_gyro_y * dt_us_actual / 1000000.0f;
    float dtheta_z = m_frd_gyro_z * dt_us_actual / 1000000.0f;

    m_roll  += dtheta_x;
    m_pitch += dtheta_y;
    m_yaw   += dtheta_z;

}