#include "attitude_algorithm.hpp"
#include "config.hpp"
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

Eigen::Matrix3f frd_from_imu_matrix()
{
    Eigen::Matrix3f matrix;
    matrix << 0.0f,   1.0f, 0.0f,
              -1.0f,   0.0f, 0.0f,
              0.0f,   0.0f, 1.0f;
    return matrix;
}

}

AccAttitudeAlgorithmer::AccAttitudeAlgorithmer()
    : m_acc_estimate(Eigen::Vector3f::Zero()),
      m_acc_covariance(Eigen::Matrix3f::Zero()),
      m_acc_process_noise(Eigen::Matrix3f::Zero()),
      m_acc_measurement_noise(Eigen::Matrix3f::Zero())
{
    m_frd_from_imu = frd_from_imu_matrix();
    m_acc_covariance.diagonal() <<
        config::ACC_KF_INITIAL_VARIANCE_X,
        config::ACC_KF_INITIAL_VARIANCE_Y,
        config::ACC_KF_INITIAL_VARIANCE_Z;
    m_acc_measurement_noise.diagonal() <<
        config::ACC_KF_MEASUREMENT_VARIANCE_X,
        config::ACC_KF_MEASUREMENT_VARIANCE_Y,
        config::ACC_KF_MEASUREMENT_VARIANCE_Z;
}

void AccAttitudeAlgorithmer::transform_coordinate(
    float acc_x, float acc_y, float acc_z)
{
    // 将加速度计数据转换到 FRD 坐标系。
    m_imu_acc << acc_x, acc_y, acc_z;
    m_measure_acc_vec = m_frd_from_imu * m_imu_acc;
    const auto now = std::chrono::steady_clock::now();

    // 第一帧直接建立 Kalman 初始状态。
    if (!m_acc_initialized) {
        m_acc_estimate = m_measure_acc_vec;
        m_last_acc_time = now;
        m_acc_initialized = true;
    } else {
        // 使用与陀螺仪相同的三维随机游走 Kalman 滤波。
        float dt = std::chrono::duration<float>(now - m_last_acc_time).count();
        m_last_acc_time = now;
        if (dt <= kMinDt) return;
        if (dt > kMaxDt) dt = kNominalDt;
        const float dt2 = dt * dt;
        m_acc_process_noise.setZero();
        m_acc_process_noise.diagonal() <<
            config::ACC_KF_JERK_STD_X * config::ACC_KF_JERK_STD_X * dt2,
            config::ACC_KF_JERK_STD_Y * config::ACC_KF_JERK_STD_Y * dt2,
            config::ACC_KF_JERK_STD_Z * config::ACC_KF_JERK_STD_Z * dt2;

        const Eigen::Vector3f prediction = m_acc_estimate;
        const Eigen::Matrix3f covariance_prediction =
            m_acc_covariance + m_acc_process_noise;
        const Eigen::Vector3f innovation = m_measure_acc_vec - prediction;
        const Eigen::Matrix3f innovation_covariance =
            covariance_prediction + m_acc_measurement_noise;
        if (innovation_covariance.allFinite()) {
            const Eigen::Matrix3f kalman_gain = covariance_prediction
                * innovation_covariance.inverse();
            m_acc_estimate = prediction + kalman_gain * innovation;

            const Eigen::Matrix3f I_K = Eigen::Matrix3f::Identity() - kalman_gain;
            m_acc_covariance =
                I_K * covariance_prediction * I_K.transpose()
                + kalman_gain * m_acc_measurement_noise * kalman_gain.transpose();
            m_acc_covariance = 0.5f
                * (m_acc_covariance + m_acc_covariance.transpose());
        } else {
            m_acc_estimate = prediction;
            m_acc_covariance = covariance_prediction;
        }
    }

    // 输出 Kalman 滤波后的 FRD 三轴加速度。
    m_frd_acc_x = m_acc_estimate.x();
    m_frd_acc_y = m_acc_estimate.y();
    m_frd_acc_z = m_acc_estimate.z();
}

void AccAttitudeAlgorithmer::algorithmer()
{
    // 分别用滤波前和滤波后的加速度解算姿态，供 MCAP 对照。
    m_raw_pitch = std::atan2(
                      m_measure_acc_vec.x(),
                      std::hypot(m_measure_acc_vec.y(), m_measure_acc_vec.z()))
                  * kRad2Deg;
    m_raw_roll = std::atan2(
                     -m_measure_acc_vec.y(), -m_measure_acc_vec.z())
                 * kRad2Deg;
    m_pitch = std::atan2(
                  m_acc_estimate.x(),
                  std::hypot(m_acc_estimate.y(), m_acc_estimate.z()))
              * kRad2Deg;
    m_roll = std::atan2(-m_acc_estimate.y(), -m_acc_estimate.z()) * kRad2Deg;
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
      m_rate_measurement_noise(Eigen::Matrix3f::Zero())
{
    m_frd_from_imu = frd_from_imu_matrix();

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
    // 将陀螺仪数据转换到 FRD 坐标系。
    m_imu_gyro << gyro_x, gyro_y, gyro_z;
    m_measure_angular_rate_vec = m_frd_from_imu * m_imu_gyro;
    m_gyro_data_time = std::chrono::steady_clock::now();

    const Eigen::Vector3f measurement =
        m_measure_angular_rate_vec * kDeg2Rad;

    // 第一帧直接建立 Kalman 初始状态。
    if (!m_rate_initialized) {
        m_rate_estimate = measurement;
        m_previous_rate_estimate = measurement;
        m_previous_raw_rate = measurement;
        m_last_time = m_gyro_data_time;
        m_frd_gyro_x = m_measure_angular_rate_vec.x();
        m_frd_gyro_y = m_measure_angular_rate_vec.y();
        m_frd_gyro_z = m_measure_angular_rate_vec.z();
        m_rate_initialized = true;
        return;
    }

    // 计算本次角速度采样周期。
    const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
        m_gyro_data_time - m_last_time);
    m_last_time = m_gyro_data_time;
    m_dt_s = dt_us.count() / 1000000.0f;
    if (m_dt_s <= kMinDt) return;
    if (m_dt_s > kMaxDt) m_dt_s = kNominalDt;

    // 更新 Kalman 过程噪声。
    const float dt2 = m_dt_s * m_dt_s;
    m_rate_process_noise.setZero();
    m_rate_process_noise.diagonal() <<
        config::GYRO_KF_ANGULAR_ACCEL_STD_X
            * config::GYRO_KF_ANGULAR_ACCEL_STD_X * dt2,
        config::GYRO_KF_ANGULAR_ACCEL_STD_Y
            * config::GYRO_KF_ANGULAR_ACCEL_STD_Y * dt2,
        config::GYRO_KF_ANGULAR_ACCEL_STD_Z
            * config::GYRO_KF_ANGULAR_ACCEL_STD_Z * dt2;

    // 对 FRD 三轴角速度进行 Kalman 滤波。
    const Eigen::Vector3f prediction = m_rate_estimate;
    const Eigen::Matrix3f covariance_prediction =
        m_rate_covariance + m_rate_process_noise;
    const Eigen::Vector3f innovation = measurement - prediction;
    const Eigen::Matrix3f innovation_covariance =
        covariance_prediction + m_rate_measurement_noise;

    if (innovation_covariance.allFinite()) {
        const Eigen::Matrix3f kalman_gain = covariance_prediction
            * innovation_covariance.inverse();
        m_rate_estimate = prediction + kalman_gain * innovation;

        const Eigen::Matrix3f I_K =
            Eigen::Matrix3f::Identity() - kalman_gain;
        m_rate_covariance =
            I_K * covariance_prediction * I_K.transpose()
            + kalman_gain * m_rate_measurement_noise
                * kalman_gain.transpose();
        m_rate_covariance = 0.5f
            * (m_rate_covariance + m_rate_covariance.transpose());
    } else {
        m_rate_estimate = prediction;
        m_rate_covariance = covariance_prediction;
    }

    // 输出 Kalman 滤波后的 FRD 三轴角速度。
    m_frd_gyro_x = m_rate_estimate.x() * kRad2Deg;
    m_frd_gyro_y = m_rate_estimate.y() * kRad2Deg;
    m_frd_gyro_z = m_rate_estimate.z() * kRad2Deg;
}

void GyroAttitudeAlgorithmer::initialize(
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

    const Eigen::Vector3f initial_measurement = m_measure_angular_rate_vec * kDeg2Rad;
    m_previous_rate_estimate = m_rate_estimate;
    m_previous_raw_rate = initial_measurement;
    m_roll = roll_deg;
    m_pitch = pitch_deg;
    m_yaw = 0.0f;
    m_unfiltered_roll = roll_deg;
    m_unfiltered_pitch = pitch_deg;
    m_unfiltered_yaw = 0.0f;
    m_wrong_roll = roll_deg;
    m_wrong_pitch = pitch_deg;
    m_wrong_yaw = 0.0f;
    m_initialized = true;
}

void GyroAttitudeAlgorithmer::update()
{
    if (!m_initialized) return;
    if (m_dt_s <= kMinDt) return;

    const Eigen::Vector3f measurement =
        m_measure_angular_rate_vec * kDeg2Rad;

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

    // 错误对照：把机体系角速度直接当作欧拉角速度分别积分。
    m_wrong_roll = wrap_degrees(m_wrong_roll + m_frd_gyro_x * m_dt_s);
    m_wrong_pitch = wrap_degrees(m_wrong_pitch + m_frd_gyro_y * m_dt_s);
    m_wrong_yaw = wrap_degrees(m_wrong_yaw + m_frd_gyro_z * m_dt_s);

    quaternion_to_euler(
        m_filtered_attitude, m_roll, m_pitch, m_yaw);
    quaternion_to_euler(
        m_raw_attitude,
        m_unfiltered_roll,
        m_unfiltered_pitch,
        m_unfiltered_yaw);
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
    const float diff_roll = wrap_degrees(m_roll - m_unfiltered_roll);
    const float diff_pitch = wrap_degrees(m_pitch - m_unfiltered_pitch);
    const float diff_yaw = wrap_degrees(m_yaw - m_unfiltered_yaw);
    std::cout << "\033[H\033[J";
    std::cout << "╔═════════════════════╦══════════╦══════════╦══════════╗\n"
              << "║ 方案B姿态输出       ║  Roll    ║  Pitch   ║   Yaw    ║\n"
              << "╠═════════════════════╬══════════╬══════════╬══════════╣\n";
    std::printf("║ 角速度KF+四元数      ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n",
                m_roll, m_pitch, m_yaw);
    std::printf("║ KF前角速度+四元数   ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n",
                m_unfiltered_roll, m_unfiltered_pitch, m_unfiltered_yaw);
    std::printf("║ 差值                 ║ %+7.2f° ║ %+7.2f° ║ %+7.2f° ║\n",
                diff_roll, diff_pitch, diff_yaw);
    std::cout << "╚═════════════════════╩══════════╩══════════╩══════════╝\n"
              << std::flush;
}
