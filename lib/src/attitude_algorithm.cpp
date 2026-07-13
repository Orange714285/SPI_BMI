#include "attitude_algorithm.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
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
void AccAttitudeAlgorithmer::print_attitude_comparison()
{
    // ANSI 转义：清屏并回到左上角，原地刷新无滚动
    std::cout << "\033[H\033[J";
    std::cout << "╔═══════════════╦══════════╦══════════╦══════════╗" << std::endl;
    std::cout << "║               ║  Roll    ║  Pitch   ║   Yaw    ║" << std::endl;
    std::cout << "╠═══════════════╬══════════╬══════════╬══════════╣" << std::endl;
    printf("║ 原始伪角积分   ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n", m_roll, m_pitch, 0.0f);
    std::cout << "╚═══════════════╩══════════╩══════════╩══════════╝" << std::endl;
    std::cout << std::flush;
}

GyroAttitudeAlgorithmer::GyroAttitudeAlgorithmer()
{
    m_IMU_to_FRD_matrix << 0 , -1,  0,
                           1 ,  0,  0,
                           0 ,  0,  1; 
    m_IMU_to_FRD_matrix_inv = m_IMU_to_FRD_matrix.inverse();
    m_initialized = false;

    // Kalman:
    m_FRD_gyro_vec_pri.setZero();
    m_FRD_gyro_vec_pos.setZero();
    m_control_mat << 1,0,0,0,0,0,
                     0,1,0,0,0,0,
                     0,0,1,0,0,0;
    // 初始协方差：设为较大值，表示初始状态估计不可靠，让滤波器快速收敛
    m_state_covariance_mat_pos = Eigen::Matrix<float, 6, 6>::Identity() * 100.0f;

    // 原始测量值积分欧拉角
    m_Euler_roll = m_Euler_pitch = m_Euler_yaw = 0.0f;
    m_diff_roll = m_diff_pitch = m_diff_yaw = 0.0f;

    // 四元数
    m_q.setIdentity();

}
void GyroAttitudeAlgorithmer::transform_coordinate_and_kalman_filter(float gyro_x, float gyro_y, float gyro_z)
{
    // ---- 存储原始 IMU 陀螺仪数据 (deg/s) ----
    m_imu_gyro_x = gyro_x;
    m_imu_gyro_y = gyro_y;
    m_imu_gyro_z = gyro_z;

    m_IMU_gyro_vec << m_imu_gyro_x,
                      m_imu_gyro_y,
                      m_imu_gyro_z;

    // ---- 将角速度从 IMU 坐标系转到 FRD 机体系（使用预计算逆矩阵）----
    m_FRD_gyro_vec_measure = m_IMU_to_FRD_matrix_inv * m_IMU_gyro_vec;

    // 记录陀螺数据的时刻，供 algorithm() 初始化 m_last_time 使用
    m_gyro_data_time = std::chrono::steady_clock::now();

    // // ---- Kalman 滤波器 ----
    // constexpr float kProcessNoiseK   = 10.0f;   // 过程噪声幅度
    // constexpr float kProcessNoiseKdt = 2.0f;    // 过程噪声时间尺度
    // constexpr float kMeasureNoiseK   = 0.5f;    // 测量噪声幅度 (deg/s)

    // // 更新状态向量先验值
    // m_state_transition_mat << 1, 0, 0, m_dt_s, 0, 0,
    //                          0, 1, 0, 0, m_dt_s, 0,
    //                          0, 0, 1, 0, 0, m_dt_s,       
    //                          0, 0, 0, 1, 0, 0,
    //                          0, 0, 0, 0, 1, 0,
    //                          0, 0, 0, 0, 0, 1;
    // m_FRD_gyro_vec_pri = m_state_transition_mat * m_FRD_gyro_vec_pos;

    // // 更新状态向量协方差矩阵先验值
    // get_process_noise_covariance_mat(kProcessNoiseK, kProcessNoiseKdt);
    // m_state_covariance_mat_pri = m_state_transition_mat * m_state_covariance_mat_pos
    //                              * m_state_transition_mat.transpose() + m_process_noise_covariance_mat;
    // get_measure_noise_covariance_mat(kMeasureNoiseK);

    // // 更新卡尔曼增益
    // m_kalman_gain = m_state_covariance_mat_pri * m_control_mat.transpose()
    //                 * ((m_control_mat * m_state_covariance_mat_pri 
    //                     * m_control_mat.transpose() + m_measure_noise_covariance_mat).inverse());
    // // 更新预测状态向量后验值 
    // m_FRD_gyro_vec_pos = m_FRD_gyro_vec_pri + m_kalman_gain * (m_FRD_gyro_vec_measure - m_control_mat * m_FRD_gyro_vec_pri);  

    // // 更新状态向量协方差矩阵后验值
    // m_state_covariance_mat_pos = (m_identify - m_kalman_gain * m_control_mat) * m_state_covariance_mat_pri;

    // m_frd_gyro_x = m_FRD_gyro_vec_pos[0];
    // m_frd_gyro_y = m_FRD_gyro_vec_pos[1];
    // m_frd_gyro_z = m_FRD_gyro_vec_pos[2];
    

}

void GyroAttitudeAlgorithmer::algorithm(float roll, float pitch)
{
    // 角度转弧度常数
    constexpr float deg2rad = M_PI / 180.0f;
    constexpr float rad2deg = 180.0f / M_PI;

    // 1. 初始化
    if (!m_initialized)
    {
        m_last_time = m_gyro_data_time;
        m_initialized = 1;
        
        m_yaw   = 0.0f;
        m_pitch = pitch;
        m_roll  = roll;
        
        m_Euler_roll  = roll;
        m_Euler_pitch = pitch;
        m_Euler_yaw   = 0.0f;
        m_diff_roll = m_diff_pitch = m_diff_yaw = 0.0f;

        m_q = Eigen::AngleAxisf(0.0f,            Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(pitch * deg2rad, Eigen::Vector3f::UnitY())        
            * Eigen::AngleAxisf(roll * deg2rad,  Eigen::Vector3f::UnitX());
        return;
    }
    
    // 2. dt 及 dt 保护
    auto now = std::chrono::steady_clock::now();
    m_dt_us = std::chrono::duration_cast<std::chrono::microseconds>(now - m_last_time);
    m_last_time = now;
    constexpr std::chrono::microseconds kMaxDt(50000);
    m_dt_s = m_dt_us.count() / 1000000.0f;
    if (m_dt_us > kMaxDt)
    {
        m_dt_us = kMaxDt;
        m_dt_s  = kMaxDt.count() / 1000000.0f;
    }

    // 3. 四元数积分（体轴角速度，内旋右乘，利用指数映射更新）
    float dtheta_x_rad = m_FRD_gyro_vec_measure[0] * m_dt_s * deg2rad;
    float dtheta_y_rad = m_FRD_gyro_vec_measure[1] * m_dt_s * deg2rad;
    float dtheta_z_rad = m_FRD_gyro_vec_measure[2] * m_dt_s * deg2rad;
    
    Eigen::Vector3f dtheta_vec(dtheta_x_rad, dtheta_y_rad, dtheta_z_rad);
    float dtheta_mag = dtheta_vec.norm();

    Eigen::Quaternionf dq;
    if (dtheta_mag > 1e-6f) {
        dq = Eigen::AngleAxisf(dtheta_mag, dtheta_vec / dtheta_mag);
    } else {
        dq = Eigen::Quaternionf(1.0f, dtheta_x_rad * 0.5f, dtheta_y_rad * 0.5f, dtheta_z_rad * 0.5f);
    }

    m_q = m_q * dq;
    m_q.normalize();

    // 4. 手动解析提取 Z-Y-X 欧拉角
    Eigen::Matrix3f R = m_q.toRotationMatrix();

    float roll_rad, pitch_rad, yaw_rad;

    // R(2, 0) = -sin(pitch)
    float sin_pitch = std::max(-1.0f, std::min(1.0f, -R(2, 0)));
    pitch_rad = std::asin(sin_pitch); // 范围 [-pi/2, pi/2]

    // 规避万向节死锁（Pitch = +-90度）的奇异点
    if (std::abs(R(2, 0)) < 0.99999f)
    {
        // 正常情况
        roll_rad = std::atan2(R(2, 1), R(2, 2)); // 范围 [-pi, pi]
        yaw_rad  = std::atan2(R(1, 0), R(0, 0)); // 范围 [-pi, pi]
    }
    else
    {
        // 奇异发生时，Roll 和 Yaw 处于同一直线上，无法单独解算，此时令 Roll = 0 辅助解算
        roll_rad = 0.0f;
        yaw_rad  = std::atan2(-R(0, 1), R(1, 1));
    }

    // 5. 转换为角度输出
    m_quat_roll  = roll_rad  * rad2deg;
    m_quat_pitch = pitch_rad * rad2deg;
    m_quat_yaw   = yaw_rad   * rad2deg;


    // ---- 原始测量值角速度直接积分 ----
    m_Euler_roll  += m_FRD_gyro_vec_measure[0] * m_dt_s;
    m_Euler_pitch += m_FRD_gyro_vec_measure[1] * m_dt_s;
    m_Euler_yaw   += m_FRD_gyro_vec_measure[2] * m_dt_s;
}

void GyroAttitudeAlgorithmer::print_attitude_comparison()
{
    // ANSI 转义：清屏并回到左上角，原地刷新无滚动
    std::cout << "\033[H\033[J";
    std::cout << "╔═════════════════════╦══════════╦══════════╦══════════╗" << std::endl;
    std::cout << "║ 内旋 Yaw Roll Pitch  ║  Roll    ║  Pitch   ║   Yaw    ║" << std::endl;
    std::cout << "╠═════════════════════╬══════════╬══════════╬══════════╣" << std::endl;
    printf(      "║ 原始伪角积分          ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n", m_Euler_roll, m_Euler_pitch, m_Euler_yaw);
    printf(      "║ 四元数积分            ║%+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n", m_quat_roll, m_quat_pitch, m_quat_yaw);
    printf(      "║ 差值                 ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n",
           m_roll - m_quat_roll, m_pitch - m_quat_pitch, m_yaw - m_quat_yaw);
    printf(      "║ 四元数 w             ║  %7.4f  ║  %7.4f  ║  %7.4f  ║\n", m_q.w(), m_q.w(), m_q.w());
    std::cout << "╚═════════════════════╩══════════╩══════════╩══════════╝" << std::endl;
    std::cout<<std::endl;

    std::cout << std::flush;
}

void GyroAttitudeAlgorithmer::get_process_noise_covariance_mat(float k, float k_dt) {
    m_Q_coefficient = k;
    float kdt = m_dt_s * k_dt;
    float a = k * 0.25 * kdt * kdt * kdt * kdt;
    float b = k * 0.5 * kdt * kdt * kdt;
    float c = k * kdt * kdt;
    m_process_noise_covariance_mat << a, 0, 0, b, 0, 0,
                                      0, a, 0, 0, b, 0,
                                      0, 0, a, 0, 0, b,
                                      b, 0, 0, c, 0, 0,
                                      0, b, 0, 0, c, 0,
                                      0, 0, b, 0, 0, c;
}

void GyroAttitudeAlgorithmer::get_measure_noise_covariance_mat(float k) {
    m_R_coefficient = k;
    m_measure_noise_covariance_mat << k, 0, 0,
                                      0, k, 0,
                                      0, 0, k;
}