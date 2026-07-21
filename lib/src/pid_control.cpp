#include "pid_control.hpp"

#include "config.hpp"

#include <algorithm>
#include <cmath>

CascadedPIDController::CascadedPIDController() = default;

bool CascadedPIDController::initialize_servos(
    int left_upper_us, int right_upper_us,
    int left_lower_us, int right_lower_us)
{
    // 保存四个舵机中位并立即归中。
    center_us_ = {
        left_upper_us, right_upper_us, left_lower_us, right_lower_us,
    };
    return servo_driver_.write_pulsewidths(center_us_);
}

float CascadedPIDController::calculate_pid(
    int index, float error, float dt,
    const std::array<float, 2>& kp,
    const std::array<float, 2>& ki,
    const std::array<float, 2>& kd,
    std::array<float, 2>& integral,
    std::array<float, 2>& last_error)
{
    // 计算单轴 PID 输出。
    integral[index] += error * dt;
    const float derivative = (error - last_error[index]) / dt;
    last_error[index] = error;
    return kp[index] * error
        + ki[index] * integral[index]
        + kd[index] * derivative;
}

bool CascadedPIDController::control(
    float target_x_rate, float target_z_rate,
    float target_roll, float target_yaw,
    float gyro_x, float gyro_z,
    float roll, float yaw,
    float dt)
{
    // 检查控制周期。
    if (dt <= 0.0f) return false;

    // 读取姿态环和角速度环 PID 参数。
    const std::array<float, 2> angle_kp{{
        config::ROLL_ANGLE_KP, config::YAW_ANGLE_KP}};
    const std::array<float, 2> angle_ki{{
        config::ROLL_ANGLE_KI, config::YAW_ANGLE_KI}};
    const std::array<float, 2> angle_kd{{
        config::ROLL_ANGLE_KD, config::YAW_ANGLE_KD}};
    const std::array<float, 2> rate_kp{{config::X_RATE_KP, config::Z_RATE_KP}};
    const std::array<float, 2> rate_ki{{config::X_RATE_KI, config::Z_RATE_KI}};
    const std::array<float, 2> rate_kd{{config::X_RATE_KD, config::Z_RATE_KD}};

    // 计算 roll 和 yaw 姿态误差。
    const float roll_error = std::remainder(target_roll - roll, 360.0f);
    const float yaw_error = std::remainder(target_yaw - yaw, 360.0f);

    // 姿态外环输出 x 和 z 轴目标角速度。
    const float x_rate_command = target_x_rate + calculate_pid(
        0, roll_error, dt, angle_kp, angle_ki, angle_kd,
        angle_integral_, angle_last_error_);
    const float z_rate_command = target_z_rate + calculate_pid(
        1, yaw_error, dt, angle_kp, angle_ki, angle_kd,
        angle_integral_, angle_last_error_);

    // 角速度内环输出 roll 和 yaw 控制量。
    const float roll_output = calculate_pid(
        0, x_rate_command - gyro_x, dt, rate_kp, rate_ki, rate_kd,
        rate_integral_, rate_last_error_);
    const float yaw_output = calculate_pid(
        1, z_rate_command - gyro_z, dt, rate_kp, rate_ki, rate_kd,
        rate_integral_, rate_last_error_);

    // 将三轴控制量混控为四个 X 舵的脉宽增量。
    const std::array<float, 4> servo_delta_us{{
        roll_output + yaw_output,
        roll_output - yaw_output,
        roll_output - yaw_output,
        roll_output + yaw_output,
    }};

    // 叠加舵机中位并输出四路 PWM。
    std::array<int, 4> pulse_us{};
    for (int i = 0; i < 4; ++i) {
        const float delta = std::clamp(
            servo_delta_us[i], -config::SERVO_MAX_DELTA_US,
            config::SERVO_MAX_DELTA_US);
        pulse_us[i] = center_us_[i] + static_cast<int>(std::lround(delta));
    }
    return servo_driver_.write_pulsewidths(pulse_us);
}
