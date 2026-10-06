#pragma once

#include "servo_driver.hpp"

#include <array>

class CascadedPIDController {
public:
    CascadedPIDController();

    // 顺序：左上、右上、右下、左下。
    bool initialize_servos(int left_upper_us, int right_upper_us,
                           int right_lower_us, int left_lower_us);

    // 目标顺序：x/z 角速度，roll/yaw 姿态；不控制 pitch。
    bool control(float target_x_rate, float target_z_rate,
                 float target_roll, float target_yaw,
                 float gyro_x, float gyro_z,
                 float roll, float yaw,
                 float dt);

    // 顺序均为 roll/yaw；外环单位 deg/s，内环单位 us。
    const std::array<float, 2>& pid_outer_outputs() const {
        return pid_outer_outputs_;
    }
    const std::array<float, 2>& pid_inner_outputs() const {
        return pid_inner_outputs_;
    }
    // 四舵实际采用的脉宽增量（经过限幅），顺序与舵机驱动一致，单位 us。
    const std::array<float, 4>& servo_deltas_us() const {
        return servo_deltas_us_;
    }
    PigpioServoDriver servo_driver_;
    std::array<int, 4> center_us_{{1500, 1500, 1500, 1500}};
private:
    float calculate_pid(int index, float error, float dt,
                        const std::array<float, 2>& kp,
                        const std::array<float, 2>& ki,
                        const std::array<float, 2>& kd,
                        std::array<float, 2>& integral,
                        std::array<float, 2>& last_error);

    std::array<float, 2> angle_integral_{};
    std::array<float, 2> angle_last_error_{};
    std::array<float, 2> rate_integral_{};
    std::array<float, 2> rate_last_error_{};
    std::array<float, 2> pid_outer_outputs_{};
    std::array<float, 2> pid_inner_outputs_{};
    std::array<float, 4> servo_deltas_us_{};
};
