#pragma once

#include "servo_driver.hpp"

#include <array>

class CascadedPIDController {
public:
    CascadedPIDController();

    // 顺序：左上、右上、左下、右下。
    bool initialize_servos(int left_upper_us, int right_upper_us,
                           int right_lower_us, int left_lower_us);

    // 目标顺序：x/z 角速度，roll/yaw 姿态；不控制 pitch。
    bool control(float target_x_rate, float target_z_rate,
                 float target_roll, float target_yaw,
                 float gyro_x, float gyro_z,
                 float roll, float yaw,
                 float dt);
    PigpioServoDriver servo_driver_;

private:
    float calculate_pid(int index, float error, float dt,
                        const std::array<float, 2>& kp,
                        const std::array<float, 2>& ki,
                        const std::array<float, 2>& kd,
                        std::array<float, 2>& integral,
                        std::array<float, 2>& last_error);

    std::array<int, 4> center_us_{{1500, 1500, 1500, 1500}};
    std::array<float, 2> angle_integral_{};
    std::array<float, 2> angle_last_error_{};
    std::array<float, 2> rate_integral_{};
    std::array<float, 2> rate_last_error_{};
};
