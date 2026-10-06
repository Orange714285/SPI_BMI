#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

// 合并后的姿态解算器：
//  - 加速度计通道：发射前用于 roll/pitch 初始姿态估计（algorithmer()）。
//  - 陀螺仪通道：方案 B，三维角速度 Kalman 滤波 + 四元数梯形积分（initialize()/update()）。
// kalman_filter() 一次完成两路的 IMU->FRD 转系与三维随机游走 Kalman 滤波，
// 类内不再保存 IMU 系原始数据。
class AttitudeAlgorithmer
{
public:
    AttitudeAlgorithmer();
    ~AttitudeAlgorithmer() = default;

    // IMU -> FRD 转系，并对加速度、角速度分别执行三维随机游走 Kalman 滤波。
    void kalman_filter(
        float acc_x, float acc_y, float acc_z,
        float gyro_x, float gyro_y, float gyro_z);
    void initialize(float roll, float pitch);
    void algorithmer();
    void update();

    float raw_frd_acc_x() const { return m_measure_acc_vec.x(); }
    float raw_frd_acc_y() const { return m_measure_acc_vec.y(); }
    float raw_frd_acc_z() const { return m_measure_acc_vec.z(); }

    float raw_frd_gyro_x() const { return m_measure_angular_rate_vec.x(); }
    float raw_frd_gyro_y() const { return m_measure_angular_rate_vec.y(); }
    float raw_frd_gyro_z() const { return m_measure_angular_rate_vec.z(); }

    // 加速度计通道（发射前姿态）。
    Eigen::Vector3f m_frd_acc = Eigen::Vector3f::Zero();   // FRD, mg（KF 后）
    float m_acc_roll = 0.0f;
    float m_acc_pitch = 0.0f;

    // 陀螺仪通道（飞行姿态）。
    Eigen::Vector3f m_frd_gyro = Eigen::Vector3f::Zero();  // FRD, deg/s（KF 后）
    float m_gyro_roll = 0.0f;
    float m_gyro_pitch = 0.0f;
    float m_gyro_yaw = 0.0f;
    float m_dt_s = 0.0f;

private:
    void quaternion_to_euler(
        const Eigen::Quaternionf& attitude,
        float& roll_deg, float& pitch_deg, float& yaw_deg) const;
    Eigen::Quaternionf quaternion_from_rotation_vector(
        const Eigen::Vector3f& rotation_vector) const;

    Eigen::Matrix3f m_frd_from_imu;

    // ---- 加速度计通道 ----
    Eigen::Vector3f m_measure_acc_vec;      // FRD, mg（滤波前）
    Eigen::Vector3f m_acc_estimate;
    Eigen::Matrix3f m_acc_covariance;
    Eigen::Matrix3f m_acc_process_noise;
    Eigen::Matrix3f m_acc_measurement_noise;
    bool m_acc_initialized = false;
    std::chrono::steady_clock::time_point m_last_acc_time{};

    // ---- 陀螺仪通道 ----
    Eigen::Vector3f m_measure_angular_rate_vec; // FRD, deg/s（滤波前）
    bool m_rate_initialized = false;
    bool m_initialized = false;
    std::chrono::steady_clock::time_point m_gyro_data_time{};
    std::chrono::steady_clock::time_point m_last_time{};
    Eigen::Quaternionf m_filtered_attitude;
    Eigen::Vector3f m_rate_estimate;
    Eigen::Vector3f m_previous_rate_estimate;
    Eigen::Matrix3f m_rate_covariance;
    Eigen::Matrix3f m_rate_process_noise;
    Eigen::Matrix3f m_rate_measurement_noise;
};
