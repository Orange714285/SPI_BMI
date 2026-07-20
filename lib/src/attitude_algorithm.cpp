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

    // 初始协方差：设为较大值，表示初始状态估计不可靠，让滤波器快速收敛
    m_state_covariance_mat_pos = Eigen::Matrix<float, 6, 6>::Identity() * 100.0f;

    // 原始测量值积分欧拉角
    m_Euler_roll = m_Euler_pitch = m_Euler_yaw = 0.0f;
    m_diff_roll = m_diff_pitch = m_diff_yaw = 0.0f;

    // 四元数
    m_mekf_q_attitude_pri.setIdentity();

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

    // ---- 将角速度从 IMU 坐标系转到 FRD 机体系（使用预计算逆矩阵）----
    m_measure_angular_rate_vec = m_IMU_to_FRD_matrix_inv * m_IMU_gyro_vec;

    // 记录陀螺数据的时刻，供 algorithm() 初始化 m_last_time 使用
    m_gyro_data_time = std::chrono::steady_clock::now();
}

void GyroAttitudeAlgorithmer::algorithm(float roll, float pitch)
{
    // 角度与弧度转换常数
    constexpr float deg2rad = M_PI / 180.0f;
    constexpr float rad2deg = 180.0f / M_PI;
    
    // 1. 初始化
    if (!m_initialized)
    {
        m_last_time = m_gyro_data_time;
        m_initialized = true;
        
        m_yaw   = 0.0f;
        m_pitch = pitch;
        m_roll  = roll;
        
        m_Euler_roll  = roll;
        m_Euler_pitch = pitch;
        m_Euler_yaw   = 0.0f;
        m_diff_roll = m_diff_pitch = m_diff_yaw = 0.0f;

        // 根据初始欧拉角（角度）生成初始姿态四元数
        m_mekf_q_attitude_pri = Eigen::AngleAxisf(0.0f,             Eigen::Vector3f::UnitZ())
                              * Eigen::AngleAxisf(pitch * deg2rad,  Eigen::Vector3f::UnitY())        
                              * Eigen::AngleAxisf(roll * deg2rad,   Eigen::Vector3f::UnitX());
        
        m_mekf_q_attitude_last_pos = m_mekf_q_attitude_pri;
        
        // 估计状态初始化：滤波器内部计算统一使用 rad/s
        m_mekf_estimated_angular_rate_vec = m_measure_angular_rate_vec * deg2rad;
        
        // 构造名义状态向量 (7维)
        m_mekf_normal_state << m_mekf_q_attitude_pri.w(), m_mekf_q_attitude_pri.x(), m_mekf_q_attitude_pri.y(), m_mekf_q_attitude_pri.z(),
                               m_mekf_estimated_angular_rate_vec[0], m_mekf_estimated_angular_rate_vec[1], m_mekf_estimated_angular_rate_vec[2];
        
        m_mekf_error_state_pri.setZero();
        
        // 初始化误差协方差矩阵 P_0 (6x6)
        m_mekf_covariance_matrix_pos_last.setZero();
        m_mekf_covariance_matrix_pos_last.diagonal() << 3e-4f, 3e-4f, 3e-4f, 3.707e-3f, 3.309e-3f,4.498e-3f;
        
        // 观测噪声协方差矩阵 R (3x3)，采用静态标定值
        m_mekf_measurement_noise_covariance_matrix << 3.707e-3f, 0.0f,       0.0f,
                                                      0.0f,       3.309e-3f, 0.0f,
                                                      0.0f,       0.0f,       4.498e-3f;
        
        // 观测矩阵 H (3x6)
        m_mekf_observation_matrix.setZero();
        m_mekf_observation_matrix.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
        
        return;
    }

    // 姿态传播必须使用相邻两帧陀螺数据的时间差，而不是 algorithm() 的执行时刻。
    m_dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
        m_gyro_data_time - m_last_time);
    m_last_time = m_gyro_data_time;
    m_dt_s = m_dt_us.count() / 1000000.0f;

    // 防止重复数据或数据时间戳异常。
    if (m_dt_s <= 1e-6f || m_dt_s > 0.005f) {
        m_dt_s = 0.005f; // 默认设为 200Hz 对应步长
    }

    // 转换为弧度单位，用于滤波解算
    m_mekf_measure_angular_rate_vec = m_measure_angular_rate_vec * deg2rad;

    // ==================== 2. 预测阶段 （Prediction） ====================
    
    // 角速度先验估计（匀速假设模型）
    Eigen::Vector3f omega_minus = m_mekf_estimated_angular_rate_vec; 

    // 旋转向量 delta_theta = omega * dt，使用四元数指数映射传播名义姿态。
    const Eigen::Vector3f rotation_vector = omega_minus * m_dt_s;
    m_mekf_theta_temp = rotation_vector.norm();

    Eigen::Quaternionf dq;
    if (m_mekf_theta_temp > 1e-6f) {
        dq = Eigen::Quaternionf(
            Eigen::AngleAxisf(m_mekf_theta_temp,
                              rotation_vector / m_mekf_theta_temp));
    } else {
        dq = Eigen::Quaternionf(1.0f,
                                0.5f * rotation_vector[0],
                                0.5f * rotation_vector[1],
                                0.5f * rotation_vector[2]);
        dq.normalize();
    }
    m_mekf_q_attitude_pri = m_mekf_q_attitude_last_pos * dq;
    m_mekf_q_attitude_pri.normalize();

    // 动态构造离散过程噪声 Q。假设每个采样周期内随机角加速度保持不变：
    // delta_theta = 0.5 * alpha * dt^2, delta_omega = alpha * dt。
    const float sigma_theta_int = 1.722579e-4f; // 额外姿态积分噪声 (rad/sqrt(s))
    const float sigma_alpha = 10.0f;            // 角加速度扰动标准差 (rad/s^2)
    const float dt2 = m_dt_s * m_dt_s;
    const float dt3 = dt2 * m_dt_s;
    const float dt4 = dt2 * dt2;
    const float var_alpha = sigma_alpha * sigma_alpha;
    const Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();

    m_mekf_process_noise_covariance_matrix.setZero();
    m_mekf_process_noise_covariance_matrix.block<3, 3>(0, 0) =
        (0.25f * var_alpha * dt4
         + sigma_theta_int * sigma_theta_int * m_dt_s) * I3;
    m_mekf_process_noise_covariance_matrix.block<3, 3>(0, 3) =
        0.5f * var_alpha * dt3 * I3;
    m_mekf_process_noise_covariance_matrix.block<3, 3>(3, 0) =
        0.5f * var_alpha * dt3 * I3;
    m_mekf_process_noise_covariance_matrix.block<3, 3>(3, 3) =
        var_alpha * dt2 * I3;

    // 状态转移矩阵 F (反对称矩阵采用当前先验估计 omega_minus)
    m_mekf_angular_rate_cross_matrix << 
         0.0f,           -omega_minus[2],  omega_minus[1],
         omega_minus[2],  0.0f,           -omega_minus[0],
        -omega_minus[1],  omega_minus[0],  0.0f;
    
    m_mekf_state_transition_matrix.setIdentity();
    m_mekf_state_transition_matrix.block<3, 3>(0, 0) -= m_mekf_angular_rate_cross_matrix * m_dt_s;
    m_mekf_state_transition_matrix.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * m_dt_s;
    
    // 协方差预测
    m_mekf_covariance_matrix_pri = m_mekf_state_transition_matrix * m_mekf_covariance_matrix_pos_last * m_mekf_state_transition_matrix.transpose() 
                                 + m_mekf_process_noise_covariance_matrix;

    // ==================== 3. 观测更新 (Measurement Update) ====================
    
    // 计算观测残差 
    Eigen::Vector3f r_k = m_mekf_measure_angular_rate_vec - omega_minus;

    // 计算卡尔曼增益
    Eigen::Matrix3f S = m_mekf_observation_matrix * m_mekf_covariance_matrix_pri * m_mekf_observation_matrix.transpose() 
                      + m_mekf_measurement_noise_covariance_matrix;
    m_mekf_kalman_gain = m_mekf_covariance_matrix_pri * m_mekf_observation_matrix.transpose() * S.inverse();

    // ==================== 4. 误差注入与状态重置 ====================
    
    // 计算后验误差状态估计
    m_mekf_error_state_pos = m_mekf_kalman_gain * r_k;
    Eigen::Vector3f delta_theta = m_mekf_error_state_pos.head<3>();
    Eigen::Vector3f delta_omega = m_mekf_error_state_pos.tail<3>();

    // 注入姿态误差：q_pos = q_pri * delta_q
    Eigen::Quaternionf dq_err(1.0f, 0.5f * delta_theta[0], 0.5f * delta_theta[1], 0.5f * delta_theta[2]);
    m_mekf_q_attitude_pos = m_mekf_q_attitude_pri * dq_err;
    m_mekf_q_attitude_pos.normalize();

    // 注入角速度误差并获取最终的最优滤波后角速度
    m_mekf_estimated_angular_rate_vec = omega_minus + delta_omega;

    // Joseph 形式更新后验协方差，减少浮点误差导致的非对称和非正定。
    const Eigen::Matrix<float, 6, 6> I_KH =
        Eigen::Matrix<float, 6, 6>::Identity()
        - m_mekf_kalman_gain * m_mekf_observation_matrix;
    m_mekf_covariance_matrix_pos =
        I_KH * m_mekf_covariance_matrix_pri * I_KH.transpose()
        + m_mekf_kalman_gain
              * m_mekf_measurement_noise_covariance_matrix
              * m_mekf_kalman_gain.transpose();

    // 误差注入改变了名义姿态的局部切空间，使用 reset Jacobian 将协方差变换到新切空间。
    Eigen::Matrix3f delta_theta_cross;
    delta_theta_cross <<
         0.0f,            -delta_theta[2],  delta_theta[1],
         delta_theta[2],   0.0f,            -delta_theta[0],
        -delta_theta[1],   delta_theta[0],   0.0f;

    Eigen::Matrix<float, 6, 6> reset_jacobian =
        Eigen::Matrix<float, 6, 6>::Identity();
    reset_jacobian.block<3, 3>(0, 0) -= 0.5f * delta_theta_cross;
    m_mekf_covariance_matrix_pos =
        reset_jacobian
        * m_mekf_covariance_matrix_pos
        * reset_jacobian.transpose();

    // 消除可能的浮点非对称，并显式完成误差状态重置。
    const Eigen::Matrix<float, 6, 6> covariance_symmetric =
        0.5f * (m_mekf_covariance_matrix_pos
                + m_mekf_covariance_matrix_pos.transpose());
    m_mekf_covariance_matrix_pos = covariance_symmetric;
    m_mekf_error_state_pri.setZero();
    m_mekf_error_state_pos.setZero();

    // 为下一帧循环保存后验状态
    m_mekf_q_attitude_last_pos = m_mekf_q_attitude_pos;
    m_mekf_covariance_matrix_pos_last = m_mekf_covariance_matrix_pos;

    // 导出滤波后的平滑机体系角速度（转换回 deg/s 输出给上层业务）
    m_frd_gyro_x = m_mekf_estimated_angular_rate_vec[0] * rad2deg;
    m_frd_gyro_y = m_mekf_estimated_angular_rate_vec[1] * rad2deg;
    m_frd_gyro_z = m_mekf_estimated_angular_rate_vec[2] * rad2deg;

    // ==================== 5. 手动解析提取 Z-Y-X 欧拉角 ====================
    
    Eigen::Matrix3f R_mat = m_mekf_q_attitude_pos.toRotationMatrix();

    float roll_rad, pitch_rad, yaw_rad;

    // R(2, 0) = -sin(pitch)
    float sin_pitch = std::max(-1.0f, std::min(1.0f, -R_mat(2, 0)));
    pitch_rad = std::asin(sin_pitch); // 范围 [-pi/2, pi/2]

    // 规避万向节死锁（Pitch = +-90度）的奇异点
    if (std::abs(R_mat(2, 0)) < 0.99999f)
    {
        roll_rad = std::atan2(R_mat(2, 1), R_mat(2, 2)); // 范围 [-pi, pi]
        yaw_rad  = std::atan2(R_mat(1, 0), R_mat(0, 0)); // 范围 [-pi, pi]
    }
    else
    {
        roll_rad = 0.0f;
        yaw_rad  = std::atan2(-R_mat(0, 1), R_mat(1, 1));
    }

    // 转换为角度输出
    m_quat_roll  = roll_rad  * rad2deg;
    m_quat_pitch = pitch_rad * rad2deg;
    m_quat_yaw   = yaw_rad   * rad2deg;
    
    m_roll = m_quat_roll;
    m_pitch = m_quat_pitch;
    m_yaw = m_quat_yaw;


    // ---- 用于对照参考的原始测量值直接积分量 ----
    m_Euler_roll  += m_measure_angular_rate_vec[0] * m_dt_s;
    m_Euler_pitch += m_measure_angular_rate_vec[1] * m_dt_s;
    m_Euler_yaw   += m_measure_angular_rate_vec[2] * m_dt_s;
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
    printf(      "║ 四元数 w             ║  %7.4f  ║  %7.4f  ║  %7.4f  ║\n", m_mekf_q_attitude_pri.w(), m_mekf_q_attitude_pri.w(), m_mekf_q_attitude_pri.w());
    std::cout << "╚═════════════════════╩══════════╩══════════╩══════════╝" << std::endl;
    std::cout<<std::endl;

    std::cout << std::flush;
}
