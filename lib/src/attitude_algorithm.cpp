#include "attitude_algorithm.hpp"
#include "config.hpp"
#include <algorithm>
#include <cmath>

namespace {
constexpr float kDeg2Rad = static_cast<float>(M_PI / 180.0);
constexpr float kRad2Deg = static_cast<float>(180.0 / M_PI);
constexpr float kNominalDt = 0.005f;
constexpr float kMaxDt = 0.020f;

Eigen::Matrix3f frd_from_imu_matrix()
{
    Eigen::Matrix3f matrix;
    matrix << 0.0f,   1.0f, 0.0f,
              -1.0f,   0.0f, 0.0f,
              0.0f,   0.0f, 1.0f;
    return matrix;
}

}

AttitudeAlgorithmer::AttitudeAlgorithmer()
    : m_acc_estimate(Eigen::Vector3f::Zero()),
      m_acc_covariance(Eigen::Matrix3f::Zero()),
      m_acc_process_noise(Eigen::Matrix3f::Zero()),
      m_acc_measurement_noise(Eigen::Matrix3f::Zero()),
      m_filtered_attitude(Eigen::Quaternionf::Identity()),
      m_rate_estimate(Eigen::Vector3f::Zero()),
      m_previous_rate_estimate(Eigen::Vector3f::Zero()),
      m_rate_covariance(Eigen::Matrix3f::Zero()),
      m_rate_process_noise(Eigen::Matrix3f::Zero()),
      m_rate_measurement_noise(Eigen::Matrix3f::Zero())
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

    m_rate_covariance.diagonal() <<
        config::GYRO_KF_INITIAL_VARIANCE_X,
        config::GYRO_KF_INITIAL_VARIANCE_Y,
        config::GYRO_KF_INITIAL_VARIANCE_Z;
    m_rate_measurement_noise.diagonal() <<
        config::GYRO_KF_MEASUREMENT_VARIANCE_X,
        config::GYRO_KF_MEASUREMENT_VARIANCE_Y,
        config::GYRO_KF_MEASUREMENT_VARIANCE_Z;
}

void AttitudeAlgorithmer::kalman_filter(
    float acc_x, float acc_y, float acc_z,
    float gyro_x, float gyro_y, float gyro_z)
{
    // IMU -> FRD 转系（不再保留 IMU 系原始数据）。
    const Eigen::Vector3f acc_frd = m_frd_from_imu * Eigen::Vector3f(acc_x, acc_y, acc_z);
    const Eigen::Vector3f gyro_frd = m_frd_from_imu * Eigen::Vector3f(gyro_x, gyro_y, gyro_z);
    m_measure_acc_vec = acc_frd;
    m_measure_angular_rate_vec = gyro_frd;

    const auto now = std::chrono::steady_clock::now();

    // ---- 加速度计三维随机游走 Kalman 滤波 ----
    if (!m_acc_initialized) {
        // 第一帧直接建立 Kalman 初始状态。
        m_acc_estimate = acc_frd;
        m_last_acc_time = now;
        m_acc_initialized = true;
    } else {
        const float dt =
            std::chrono::duration<float>(now - m_last_acc_time).count();
        m_last_acc_time = now;
        const float dt2 = dt * dt;
        m_acc_process_noise.setZero();
        m_acc_process_noise.diagonal() <<
            config::ACC_KF_JERK_STD_X * config::ACC_KF_JERK_STD_X * dt2,
            config::ACC_KF_JERK_STD_Y * config::ACC_KF_JERK_STD_Y * dt2,
            config::ACC_KF_JERK_STD_Z * config::ACC_KF_JERK_STD_Z * dt2;

        const Eigen::Vector3f prediction = m_acc_estimate;
        const Eigen::Matrix3f covariance_prediction = m_acc_covariance + m_acc_process_noise;
        const Eigen::Vector3f innovation = acc_frd - prediction;
        const Eigen::Matrix3f innovation_covariance = covariance_prediction + m_acc_measurement_noise;
        const Eigen::Matrix3f kalman_gain = covariance_prediction * innovation_covariance.inverse();
        m_acc_estimate = prediction + kalman_gain * innovation;

        const Eigen::Matrix3f I_K = Eigen::Matrix3f::Identity() - kalman_gain;
        m_acc_covariance =
            I_K * covariance_prediction * I_K.transpose()
            + kalman_gain * m_acc_measurement_noise * kalman_gain.transpose();
        m_acc_covariance =
            0.5f * (m_acc_covariance + m_acc_covariance.transpose());
    }

    // 输出 Kalman 滤波后的 FRD 三轴加速度。
    m_frd_acc = m_acc_estimate;

    // ---- 角速度三维随机游走 Kalman 滤波 ----
    const Eigen::Vector3f measurement = gyro_frd * kDeg2Rad;
    m_gyro_data_time = now;
    if (!m_rate_initialized) {
        // 第一帧直接建立 Kalman 初始状态。
        m_rate_estimate = measurement;
        m_previous_rate_estimate = measurement;
        m_last_time = m_gyro_data_time;
        m_frd_gyro = gyro_frd;
        m_rate_initialized = true;
        return;
    }

    // 计算本次角速度采样周期。
    const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
        m_gyro_data_time - m_last_time);
    m_last_time = m_gyro_data_time;
    m_dt_s = dt_us.count() / 1000000.0f;
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
    const Eigen::Matrix3f kalman_gain =
        covariance_prediction * innovation_covariance.inverse();
    m_rate_estimate = prediction + kalman_gain * innovation;

    const Eigen::Matrix3f I_K =
        Eigen::Matrix3f::Identity() - kalman_gain;
    m_rate_covariance =
        I_K * covariance_prediction * I_K.transpose()
        + kalman_gain * m_rate_measurement_noise * kalman_gain.transpose();
    m_rate_covariance =
        0.5f * (m_rate_covariance + m_rate_covariance.transpose());

    // 输出 Kalman 滤波后的 FRD 三轴角速度（deg/s）。
    m_frd_gyro = m_rate_estimate * kRad2Deg;
}

void AttitudeAlgorithmer::algorithmer()
{
    // 用 Kalman 滤波后的加速度解算发射前姿态。
    m_acc_pitch = std::atan2(
                      m_acc_estimate.x(),
                      std::hypot(m_acc_estimate.y(), m_acc_estimate.z()))
                  * kRad2Deg;
    m_acc_roll = std::atan2(-m_acc_estimate.y(), -m_acc_estimate.z()) * kRad2Deg;
}

void AttitudeAlgorithmer::initialize(
    float roll_deg, float pitch_deg)
{
    m_filtered_attitude =
        Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ())
      * Eigen::AngleAxisf(pitch_deg * kDeg2Rad,
                          Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(roll_deg * kDeg2Rad,
                          Eigen::Vector3f::UnitX());
    m_filtered_attitude.normalize();

    m_previous_rate_estimate = m_rate_estimate;
    m_gyro_roll = roll_deg;
    m_gyro_pitch = pitch_deg;
    m_gyro_yaw = 0.0f;
    m_initialized = true;
}

void AttitudeAlgorithmer::update()
{
    if (!m_initialized) return;

    const Eigen::Vector3f average_rate =
        0.5f * (m_previous_rate_estimate + m_rate_estimate);
    const Eigen::Quaternionf filtered_dq =
        quaternion_from_rotation_vector(average_rate * m_dt_s);
    m_filtered_attitude = m_filtered_attitude * filtered_dq;
    m_filtered_attitude.normalize();
    m_previous_rate_estimate = m_rate_estimate;

    quaternion_to_euler(
        m_filtered_attitude, m_gyro_roll, m_gyro_pitch, m_gyro_yaw);
}

Eigen::Quaternionf AttitudeAlgorithmer::quaternion_from_rotation_vector(
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

void AttitudeAlgorithmer::quaternion_to_euler(
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
