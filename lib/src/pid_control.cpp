#include "pid_control.hpp"

#include "config.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>

CascadedPIDController::CascadedPIDController() = default;

bool CascadedPIDController::initialize_servos(
    int left_upper_us, int right_upper_us,
    int right_lower_us, int left_lower_us)
{
    // 保存四个舵机中位并立即归中。
    center_us_ = {
        left_upper_us, right_upper_us, right_lower_us, left_lower_us,
    };
    std::array<int,4UL> left_us , right_us;
    for (int i = 0 ; i <=  3 ; i++)
    {
        left_us[i] = center_us_[i] - 500;
        right_us[i] = center_us_[i] + 500;
    }

    if (!servo_driver_.write_pulsewidths(center_us_))
    {
        std::cerr << "[Error] Servo Self_test_wrong!" <<std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!servo_driver_.write_pulsewidths(left_us))
    {
        std::cerr << "[Error] Servo Self_test_wrong!" <<std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!servo_driver_.write_pulsewidths(right_us))
    {
        std::cerr << "[Error] Servo Self_test_wrong!" <<std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!servo_driver_.write_pulsewidths(center_us_))
    {
        std::cerr << "[Error] Servo Self_test_wrong!" <<std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}    
        

    


float CascadedPIDController::calculate_pid(
    int index, float error, float dt,
    const std::array<float, 3>& kp,
    const std::array<float, 3>& ki,
    const std::array<float, 3>& kd,
    std::array<float, 3>& integral,
    std::array<float, 3>& last_error)
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
    float target_x_rate, float target_y_rate, float target_z_rate,
    float target_roll, float target_pitch, float target_yaw,
    float gyro_x, float gyro_y, float gyro_z,
    float roll, float pitch, float yaw,
    float dt)
{
    // 读取姿态环和角速度环 PID 参数。索引：0=roll，1=yaw，2=pitch。
    const std::array<float, 3> angle_kp{{config::ROLL_ANGLE_KP, config::YAW_ANGLE_KP,
                                         config::PITCH_ANGLE_KP}};
    const std::array<float, 3> angle_ki{{config::ROLL_ANGLE_KI, config::YAW_ANGLE_KI,
                                         config::PITCH_ANGLE_KI}};
    const std::array<float, 3> angle_kd{{config::ROLL_ANGLE_KD, config::YAW_ANGLE_KD,
                                         config::PITCH_ANGLE_KD}};
    const std::array<float, 3> rate_kp{{config::X_RATE_KP, config::Z_RATE_KP,
                                        config::Y_RATE_KP}};
    const std::array<float, 3> rate_ki{{config::X_RATE_KI, config::Z_RATE_KI,
                                        config::Y_RATE_KI}};
    const std::array<float, 3> rate_kd{{config::X_RATE_KD, config::Z_RATE_KD,
                                        config::Y_RATE_KD}};

    // 计算 roll / pitch / yaw 姿态误差。
    // 注意 pitch 的符号约定：FRD 系绕 +y（右）正转 = 抬头，与 ZYX 欧拉 pitch 正方向一致，
    // 故与 roll/yaw 一样直接用 目标-实测，无需取反。
    const float roll_error = std::remainder(target_roll - roll, 360.0f);
    const float yaw_error = std::remainder(target_yaw - yaw, 360.0f);
    const float pitch_error = std::remainder(target_pitch - pitch, 360.0f);

    // 姿态外环输出三轴目标角速度。
    const float roll_outer_output = calculate_pid(
        0, roll_error, dt, angle_kp, angle_ki, angle_kd,
        angle_integral_, angle_last_error_);
    const float yaw_outer_output = calculate_pid(
        1, yaw_error, dt, angle_kp, angle_ki, angle_kd,
        angle_integral_, angle_last_error_);
    const float pitch_outer_output = calculate_pid(
        2, pitch_error, dt, angle_kp, angle_ki, angle_kd,
        angle_integral_, angle_last_error_);
    pid_outer_outputs_ = {roll_outer_output, yaw_outer_output, pitch_outer_output};
    const float x_rate_command = target_x_rate + roll_outer_output;
    const float z_rate_command = target_z_rate + yaw_outer_output;
    const float y_rate_command = target_y_rate + pitch_outer_output;

    // 角速度内环输出三轴控制量。
    const float roll_output = calculate_pid(
        0, x_rate_command - gyro_x, dt, rate_kp, rate_ki, rate_kd,
        rate_integral_, rate_last_error_);
    const float yaw_output = calculate_pid(
        1, z_rate_command - gyro_z, dt, rate_kp, rate_ki, rate_kd,
        rate_integral_, rate_last_error_);
    const float pitch_output = config::PITCH_CHANNEL_SIGN * calculate_pid(
        2, y_rate_command - gyro_y, dt, rate_kp, rate_ki, rate_kd,
        rate_integral_, rate_last_error_);
    pid_inner_outputs_ = {roll_output, yaw_output, pitch_output};

    // 将三轴控制量混控为四个 X 舵的脉宽增量。
    // roll 为共模，yaw 为「上组 vs 下组」差动，pitch 为「左组 vs 右组」差动。
    // pitch_output = 0 时与只控 roll/yaw 时完全一致。
    const std::array<float, 4> servo_delta_us{{
        - roll_output + yaw_output + pitch_output,   // 左上
        - roll_output + yaw_output - pitch_output,   // 右上
        - roll_output - yaw_output - pitch_output,   // 右下
        - roll_output - yaw_output + pitch_output,   // 左下
    }};

    // 叠加舵机中位并输出四路 PWM。
    std::array<int, 4> pulse_us{};
    for (int i = 0; i < 4; ++i) {
        const float delta = std::clamp(
            servo_delta_us[i], -config::SERVO_MAX_DELTA_US,
            config::SERVO_MAX_DELTA_US);
        servo_deltas_us_[i] = delta;
        pulse_us[i] = center_us_[i] + static_cast<int>(std::lround(delta));
    }
    return servo_driver_.write_pulsewidths(pulse_us);
}
