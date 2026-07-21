#include "attitude_algorithm.hpp"
#include "config.hpp"
#include "tools/buzzer.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace {
constexpr float kDeg2Rad = static_cast<float>(M_PI / 180.0);
constexpr float kRad2Deg = static_cast<float>(180.0 / M_PI);
constexpr float kNominalDt = 0.005f;
constexpr float kMinDt = 1.0e-6f;
constexpr float kMaxDt = 0.020f;

float wrap_degrees(float angle)
{
    return std::remainder(angle, 360.0f);
}

}

AccAttitudeAlgorithmer::AccAttitudeAlgorithmer()
{
    // 与原实现保持一致：最终使用 FRD <- IMU 的逆矩阵。
    const Eigen::Matrix3f imu_basis_matrix = [] {
        Eigen::Matrix3f matrix;
        matrix << 0.0f, -1.0f, 0.0f,
                   1.0f,  0.0f, 0.0f,
                   0.0f,  0.0f, 1.0f;
        return matrix;
    }();
    m_frd_from_imu = imu_basis_matrix.inverse();
}

void AccAttitudeAlgorithmer::transform_coordinate(
    float acc_x, float acc_y, float acc_z)
{
    m_imu_acc << acc_x, acc_y, acc_z;
    m_frd_acc = m_frd_from_imu * m_imu_acc;
    m_acc_magnitude = m_imu_acc.norm();

    m_frd_acc_x = m_frd_acc.x();
    m_frd_acc_y = m_frd_acc.y();
    m_frd_acc_z = m_frd_acc.z();
}

void AccAttitudeAlgorithmer::algorithmer()
{
    m_pitch = std::atan2(
                  m_frd_acc.x(),
                  std::hypot(m_frd_acc.y(), m_frd_acc.z()))
              * kRad2Deg;
    m_roll = std::atan2(-m_frd_acc.y(), -m_frd_acc.z()) * kRad2Deg;
}

void AccAttitudeAlgorithmer::print_attitude_comparison()
{
    std::cout << "\033[H\033[J";
    std::cout << "╔═══════════════╦══════════╦══════════╦══════════╗\n"
              << "║               ║  Roll    ║  Pitch   ║   Yaw    ║\n"
              << "╠═══════════════╬══════════╬══════════╬══════════╣\n";
    std::printf("║ 加速度计姿态   ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n",
                m_roll, m_pitch, 0.0f);
    std::cout << "╚═══════════════╩══════════╩══════════╩══════════╝\n"
              << std::flush;
}

GyroAttitudeAlgorithmer::GyroAttitudeAlgorithmer()
    : m_filtered_attitude(Eigen::Quaternionf::Identity()),
      m_raw_attitude(Eigen::Quaternionf::Identity()),
      m_rate_estimate(Eigen::Vector3f::Zero()),
      m_previous_rate_estimate(Eigen::Vector3f::Zero()),
      m_previous_raw_rate(Eigen::Vector3f::Zero()),
      m_rate_covariance(Eigen::Matrix3f::Zero()),
      m_rate_process_noise(Eigen::Matrix3f::Zero()),
      m_rate_measurement_noise(Eigen::Matrix3f::Zero()),
      m_rate_kalman_gain(Eigen::Matrix3f::Zero())
{
    const Eigen::Matrix3f imu_basis_matrix = [] {
        Eigen::Matrix3f matrix;
        matrix << 0.0f, -1.0f, 0.0f,
                   1.0f,  0.0f, 0.0f,
                   0.0f,  0.0f, 1.0f;
        return matrix;
    }();
    m_frd_from_imu = imu_basis_matrix.inverse();

    m_rate_covariance.diagonal() <<
        config::GYRO_KF_INITIAL_VARIANCE_X,
        config::GYRO_KF_INITIAL_VARIANCE_Y,
        config::GYRO_KF_INITIAL_VARIANCE_Z;

    m_rate_measurement_noise.diagonal() <<
        config::GYRO_KF_MEASUREMENT_VARIANCE_X,
        config::GYRO_KF_MEASUREMENT_VARIANCE_Y,
        config::GYRO_KF_MEASUREMENT_VARIANCE_Z;
}

void GyroAttitudeAlgorithmer::transform_coordinate(
    float gyro_x, float gyro_y, float gyro_z)
{
    m_imu_gyro_x = gyro_x;
    m_imu_gyro_y = gyro_y;
    m_imu_gyro_z = gyro_z;
    m_imu_gyro << gyro_x, gyro_y, gyro_z;

    m_measure_angular_rate_vec = m_frd_from_imu * m_imu_gyro;
    m_gyro_data_time = std::chrono::steady_clock::now();
}

void GyroAttitudeAlgorithmer::initialize_attitude(
    float roll_deg, float pitch_deg)
{
    m_filtered_attitude =
        Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ())
      * Eigen::AngleAxisf(pitch_deg * kDeg2Rad,
                          Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(roll_deg * kDeg2Rad,
                          Eigen::Vector3f::UnitX());
    m_filtered_attitude.normalize();
    m_raw_attitude = m_filtered_attitude;

    const Eigen::Vector3f initial_measurement =
        m_measure_angular_rate_vec * kDeg2Rad;
    m_rate_estimate = initial_measurement;
    m_previous_rate_estimate = m_rate_estimate;
    m_previous_raw_rate = initial_measurement;

    m_rate_covariance.setZero();
    m_rate_covariance.diagonal() <<
        config::GYRO_KF_INITIAL_VARIANCE_X,
        config::GYRO_KF_INITIAL_VARIANCE_Y,
        config::GYRO_KF_INITIAL_VARIANCE_Z;
    m_last_time = m_gyro_data_time;
    m_initialized = true;

    m_roll = roll_deg;
    m_pitch = pitch_deg;
    m_yaw = 0.0f;
    m_quat_roll = roll_deg;
    m_quat_pitch = pitch_deg;
    m_quat_yaw = 0.0f;
    m_Euler_roll = roll_deg;
    m_Euler_pitch = pitch_deg;
    m_Euler_yaw = 0.0f;
    m_diff_roll = m_diff_pitch = m_diff_yaw = 0.0f;
}

void GyroAttitudeAlgorithmer::algorithm(float roll, float pitch)
{
    if (!m_initialized) {
        initialize_attitude(roll, pitch);
        return;
    }

    m_dt_us = std::chrono::duration_cast<std::chrono::microseconds>(m_gyro_data_time - m_last_time);
    m_last_time = m_gyro_data_time;
    m_dt_s = m_dt_us.count() / 1000000.0f;

    if (m_dt_s <= kMinDt) {
        return;
    }
    if (m_dt_s > kMaxDt) {
        m_dt_s = kNominalDt;
    }

    const float dt2 = m_dt_s * m_dt_s;
    m_rate_process_noise.setZero();
    m_rate_process_noise.diagonal() <<
        config::GYRO_KF_ANGULAR_ACCEL_STD_X
            * config::GYRO_KF_ANGULAR_ACCEL_STD_X * dt2,
        config::GYRO_KF_ANGULAR_ACCEL_STD_Y
            * config::GYRO_KF_ANGULAR_ACCEL_STD_Y * dt2,
        config::GYRO_KF_ANGULAR_ACCEL_STD_Z
            * config::GYRO_KF_ANGULAR_ACCEL_STD_Z * dt2;

    const Eigen::Vector3f measurement =
        m_measure_angular_rate_vec * kDeg2Rad;
    // KF 公式 1：状态预测（随机游走模型）。
    const Eigen::Vector3f prediction = m_rate_estimate;

    // KF 公式 2：协方差预测。
    //   P_k^- = F * P_(k-1)^+ * F^T + Q，F = I 时退化为 P^+ + Q。
    const Eigen::Matrix3f covariance_prediction =
        m_rate_covariance + m_rate_process_noise;

    // KF 公式 3：量测创新及其协方差。
    //   r_k = z_k - H * omega_k^-，H = I；S_k = H*P_k^-*H^T + R。
    const Eigen::Vector3f innovation = measurement - prediction;
    const Eigen::Matrix3f innovation_covariance =
        covariance_prediction + m_rate_measurement_noise;

    // KF 公式 4：Kalman 增益。
    //   K_k = P_k^- * H^T * S_k^-1，H = I。
    // 当前按需求使用显式 3x3 逆矩阵；S 的数值有效性由有限值检查保护。
    if (innovation_covariance.allFinite()) {
        m_rate_kalman_gain = covariance_prediction
            * innovation_covariance.inverse();

        // KF 公式 5：后验状态更新。
        //   omega_k^+ = omega_k^- + K_k * r_k。
        m_rate_estimate = prediction
            + m_rate_kalman_gain * innovation;

        // Joseph 形式的后验协方差更新：
        //   P_k^+ = (I-KH)P_k^-(I-KH)^T + K*R*K^T。
        const Eigen::Matrix3f I_K =
            Eigen::Matrix3f::Identity() - m_rate_kalman_gain;
        m_rate_covariance =
            I_K * covariance_prediction * I_K.transpose()
            + m_rate_kalman_gain * m_rate_measurement_noise
                * m_rate_kalman_gain.transpose();
        m_rate_covariance = 0.5f
            * (m_rate_covariance + m_rate_covariance.transpose());
    } else {
        m_rate_estimate = prediction;
        m_rate_covariance = covariance_prediction;
    }

    const Eigen::Vector3f average_rate =
        0.5f * (m_previous_rate_estimate + m_rate_estimate);
    const Eigen::Quaternionf filtered_dq =
        quaternion_from_rotation_vector(average_rate * m_dt_s);
    m_filtered_attitude = m_filtered_attitude * filtered_dq;
    m_filtered_attitude.normalize();
    m_previous_rate_estimate = m_rate_estimate;

    // 对照通道：使用 KF 前原始角速度做同样的梯形四元数积分。
    const Eigen::Vector3f raw_average_rate =
        0.5f * (m_previous_raw_rate + measurement);
    const Eigen::Quaternionf raw_dq =
        quaternion_from_rotation_vector(raw_average_rate * m_dt_s);
    m_raw_attitude = m_raw_attitude * raw_dq;
    m_raw_attitude.normalize();
    m_previous_raw_rate = measurement;

    m_frd_gyro_x = m_rate_estimate.x() * kRad2Deg;
    m_frd_gyro_y = m_rate_estimate.y() * kRad2Deg;
    m_frd_gyro_z = m_rate_estimate.z() * kRad2Deg;

    update_euler_outputs();
    m_diff_roll = wrap_degrees(m_roll - m_Euler_roll);
    m_diff_pitch = wrap_degrees(m_pitch - m_Euler_pitch);
    m_diff_yaw = wrap_degrees(m_yaw - m_Euler_yaw);
}

Eigen::Quaternionf GyroAttitudeAlgorithmer::quaternion_from_rotation_vector(
    const Eigen::Vector3f& rotation_vector) const
{
    const float angle = rotation_vector.norm();
    if (angle > 1.0e-6f) {
        return Eigen::Quaternionf(
            Eigen::AngleAxisf(angle, rotation_vector / angle));
    }

    Eigen::Quaternionf dq(
        1.0f,
        0.5f * rotation_vector.x(),
        0.5f * rotation_vector.y(),
        0.5f * rotation_vector.z());
    dq.normalize();
    return dq;
}

void GyroAttitudeAlgorithmer::update_euler_outputs()
{
    quaternion_to_euler(
        m_filtered_attitude, m_roll, m_pitch, m_yaw);
    quaternion_to_euler(
        m_raw_attitude, m_Euler_roll, m_Euler_pitch, m_Euler_yaw);

    // 保留原有 quat_* 字段作为滤波后姿态的兼容镜像。
    m_quat_roll = m_roll;
    m_quat_pitch = m_pitch;
    m_quat_yaw = m_yaw;
}

void GyroAttitudeAlgorithmer::quaternion_to_euler(
    const Eigen::Quaternionf& attitude,
    float& roll_deg,
    float& pitch_deg,
    float& yaw_deg) const
{
    const Eigen::Matrix3f rotation = attitude.toRotationMatrix();
    const float sin_pitch = std::clamp(
        -rotation(2, 0), -1.0f, 1.0f);
    const float pitch_rad = std::asin(sin_pitch);

    float roll_rad = 0.0f;
    float yaw_rad = 0.0f;
    if (std::abs(rotation(2, 0)) < 0.99999f) {
        roll_rad = std::atan2(rotation(2, 1), rotation(2, 2));
        yaw_rad = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        yaw_rad = std::atan2(-rotation(0, 1), rotation(1, 1));
    }

    roll_deg = roll_rad * kRad2Deg;
    pitch_deg = pitch_rad * kRad2Deg;
    yaw_deg = yaw_rad * kRad2Deg;
}

void GyroAttitudeAlgorithmer::print_attitude_comparison()
{
    std::cout << "\033[H\033[J";
    std::cout << "╔═════════════════════╦══════════╦══════════╦══════════╗\n"
              << "║ 方案B姿态输出       ║  Roll    ║  Pitch   ║   Yaw    ║\n"
              << "╠═════════════════════╬══════════╬══════════╬══════════╣\n";
    std::printf("║ 角速度KF+四元数      ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n",
                m_roll, m_pitch, m_yaw);
    std::printf("║ KF前角速度+四元数   ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n",
                m_Euler_roll, m_Euler_pitch, m_Euler_yaw);
    std::printf("║ 差值                 ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n",
                m_diff_roll, m_diff_pitch, m_diff_yaw);
    std::cout << "╚═════════════════════╩══════════╩══════════╩══════════╝\n"
              << std::flush;
}
