#pragma once

#include "servo_driver.hpp"

#include <array>

class CascadedPIDController {
public:
    CascadedPIDController();

    // 顺序：左上、右上、右下、左下。
    bool initialize_servos(int left_upper_us, int right_upper_us,
                           int right_lower_us, int left_lower_us);

    // 目标顺序：x/y/z 角速度前馈（deg/s），roll/pitch/yaw 姿态目标（deg）。
    bool control(float target_x_rate, float target_y_rate, float target_z_rate,
                 float target_roll, float target_pitch, float target_yaw,
                 float gyro_x, float gyro_y, float gyro_z,
                 float roll, float pitch, float yaw,
                 float dt);

    // 顺序均为 roll/yaw/pitch（索引 0/1/2）；外环单位 deg/s，内环单位 us。
    const std::array<float, 3>& pid_outer_outputs() const {
        return pid_outer_outputs_;
    }
    const std::array<float, 3>& pid_inner_outputs() const {
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
                        const std::array<float, 3>& kp,
                        const std::array<float, 3>& ki,
                        const std::array<float, 3>& kd,
                        std::array<float, 3>& integral,
                        std::array<float, 3>& last_error);

    // 索引：0=roll(x)，1=yaw(z)，2=pitch(y)。
    std::array<float, 3> angle_integral_{};
    std::array<float, 3> angle_last_error_{};
    std::array<float, 3> rate_integral_{};
    std::array<float, 3> rate_last_error_{};
    std::array<float, 3> pid_outer_outputs_{};
    std::array<float, 3> pid_inner_outputs_{};
    std::array<float, 4> servo_deltas_us_{};
};
