#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

// 加速度计仅用于发射前的 roll/pitch 初始姿态估计。
class AccAttitudeAlgorithmer
{
public:
    AccAttitudeAlgorithmer();
    ~AccAttitudeAlgorithmer() = default;

    void transform_coordinate(float acc_x, float acc_y, float acc_z);
    void algorithmer();
    void print_attitude_comparison();

    float m_frd_acc_x = 0.0f;
    float m_frd_acc_y = 0.0f;
    float m_frd_acc_z = 0.0f;
    float m_pitch = 0.0f;
    float m_roll = 0.0f;

private:
    Eigen::Matrix3f m_frd_from_imu;
    Eigen::Vector3f m_imu_acc;
    Eigen::Vector3f m_frd_acc;
    float m_acc_magnitude = 0.0f;
};

// 方案 B：三维角速度 Kalman 滤波 + 四元数梯形积分。
// 内部角速度单位为 rad/s，对外输入和输出单位为 deg/s。
class GyroAttitudeAlgorithmer
{
public:
    GyroAttitudeAlgorithmer();
    ~GyroAttitudeAlgorithmer() = default;

    void transform_coordinate(float gyro_x, float gyro_y, float gyro_z);
    void algorithm(float roll, float pitch);
    void print_attitude_comparison();

    float raw_frd_gyro_x() const { return m_measure_angular_rate_vec.x(); }
    float raw_frd_gyro_y() const { return m_measure_angular_rate_vec.y(); }
    float raw_frd_gyro_z() const { return m_measure_angular_rate_vec.z(); }

    float m_imu_gyro_x = 0.0f;
    float m_imu_gyro_y = 0.0f;
    float m_imu_gyro_z = 0.0f;

    float m_frd_gyro_x = 0.0f;
    float m_frd_gyro_y = 0.0f;
    float m_frd_gyro_z = 0.0f;

    float m_roll = 0.0f;
    float m_pitch = 0.0f;
    float m_yaw = 0.0f;

    // KF 前原始角速度通过独立四元数积分得到的 ZYX 姿态。
    float m_Euler_roll = 0.0f;
    float m_Euler_pitch = 0.0f;
    float m_Euler_yaw = 0.0f;
    float m_diff_roll = 0.0f;
    float m_diff_pitch = 0.0f;
    float m_diff_yaw = 0.0f;
    float m_quat_roll = 0.0f;
    float m_quat_pitch = 0.0f;
    float m_quat_yaw = 0.0f;

    std::chrono::microseconds m_dt_us{0};
    float m_dt_s = 0.0f;

private:
    void initialize_attitude(float roll_deg, float pitch_deg);
    void update_euler_outputs();
    void quaternion_to_euler(
        const Eigen::Quaternionf& attitude,
        float& roll_deg,
        float& pitch_deg,
        float& yaw_deg) const;
    Eigen::Quaternionf quaternion_from_rotation_vector(
        const Eigen::Vector3f& rotation_vector) const;

    Eigen::Matrix3f m_frd_from_imu;
    Eigen::Vector3f m_imu_gyro;
    Eigen::Vector3f m_measure_angular_rate_vec; // FRD, deg/s

    bool m_initialized = false;
    std::chrono::steady_clock::time_point m_gyro_data_time{};
    std::chrono::steady_clock::time_point m_last_time{};

    Eigen::Quaternionf m_filtered_attitude;
    Eigen::Quaternionf m_raw_attitude;

    // 三维角速度 Kalman 状态，单位 rad/s。
    Eigen::Vector3f m_rate_estimate;
    Eigen::Vector3f m_previous_rate_estimate;
    Eigen::Vector3f m_previous_raw_rate;
    Eigen::Matrix3f m_rate_covariance;
    Eigen::Matrix3f m_rate_process_noise;
    Eigen::Matrix3f m_rate_measurement_noise;
    Eigen::Matrix3f m_rate_kalman_gain;
};
