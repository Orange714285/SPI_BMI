#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <libcamera/libcamera.h>

struct VisionData
{
    int m_target_pixel_x{0};
    int m_target_pixel_y{0};
    int m_target_status{0};
    int m_frame_dt_ms{0};
    int m_video_fps{0};
};

enum class FlyingState : int
{
    PRE_FLIGHT = 0,
    FLIGHT = 1,
    FLIGHT_END = 2,
};

// Shared cross-thread state. The instance is owned by main() and passed to
// components that need access to it; the pointers are therefore non-owning.
struct SharedState
{
    std::atomic<bool> running{true};
    std::atomic<VisionData> vision_data{};
    std::atomic<FlyingState> flying_state{FlyingState::PRE_FLIGHT};
    std::atomic<long long> flight_duration_ms{0};
};

struct FrameData
{
    libcamera::FrameBuffer::Plane plane;
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int stride = 0;
    uint64_t sequence = 0;
    bool valid = false;

    FrameData():plane{libcamera::SharedFD(-1),0,0}{}
};

class CarData
{
public:
    int state = 0;
    float acc_raw_frd_x_mg = 0.0f;
    float acc_raw_frd_y_mg = 0.0f;
    float acc_raw_frd_z_mg = 0.0f;
    float acc_filtered_frd_x_mg = 0.0f;
    float acc_filtered_frd_y_mg = 0.0f;
    float acc_filtered_frd_z_mg = 0.0f;
    float gyro_raw_frd_x_dps = 0.0f;
    float gyro_raw_frd_y_dps = 0.0f;
    float gyro_raw_frd_z_dps = 0.0f;
    float gyro_filtered_frd_x_dps = 0.0f;
    float gyro_filtered_frd_y_dps = 0.0f;
    float gyro_filtered_frd_z_dps = 0.0f;
    float attitude_roll_deg = 0.0f;
    float attitude_yaw_deg = 0.0f;
    float attitude_pitch_deg = 0.0f;
    float pid_outer_roll = 0.0f;
    float pid_outer_yaw = 0.0f;
    float pid_inner_roll = 0.0f;
    float pid_inner_yaw = 0.0f;
    float servo_left_upper = 0.0f;
    float servo_right_upper = 0.0f;
    float servo_right_lower = 0.0f;
    float servo_left_lower = 0.0f;
    float pid_inner_pitch = 0.0f;
    float pitch_target_deg = 0.0f;
    float pid_outer_pitch = 0.0f;

    void data_update(
        int flying_state,
        float acc_raw_x, float acc_raw_y, float acc_raw_z,
        float acc_filtered_x, float acc_filtered_y, float acc_filtered_z,
        float gyro_raw_x, float gyro_raw_y, float gyro_raw_z,
        float gyro_filtered_x, float gyro_filtered_y, float gyro_filtered_z,
        float roll, float yaw, float pitch,
        float outer_roll, float outer_yaw,
        float inner_roll, float inner_yaw,
        float left_upper, float right_upper,
        float right_lower, float left_lower,
        float inner_pitch,
        float pitch_target,
        float outer_pitch)
    {
        state = flying_state;
        acc_raw_frd_x_mg = acc_raw_x;
        acc_raw_frd_y_mg = acc_raw_y;
        acc_raw_frd_z_mg = acc_raw_z;
        acc_filtered_frd_x_mg = acc_filtered_x;
        acc_filtered_frd_y_mg = acc_filtered_y;
        acc_filtered_frd_z_mg = acc_filtered_z;
        gyro_raw_frd_x_dps = gyro_raw_x;
        gyro_raw_frd_y_dps = gyro_raw_y;
        gyro_raw_frd_z_dps = gyro_raw_z;
        gyro_filtered_frd_x_dps = gyro_filtered_x;
        gyro_filtered_frd_y_dps = gyro_filtered_y;
        gyro_filtered_frd_z_dps = gyro_filtered_z;
        attitude_roll_deg = roll;
        attitude_yaw_deg = yaw;
        attitude_pitch_deg = pitch;
        pid_outer_roll = outer_roll;
        pid_outer_yaw = outer_yaw;
        pid_inner_roll = inner_roll;
        pid_inner_yaw = inner_yaw;
        servo_left_upper = left_upper;
        servo_right_upper = right_upper;
        servo_right_lower = right_lower;
        servo_left_lower = left_lower;
        pid_inner_pitch = inner_pitch;
        pitch_target_deg = pitch_target;
        pid_outer_pitch = outer_pitch;
    }

    /** @brief 输出姿态角 roll / yaw / pitch（度），以及三轴原始角速度（dps）与三轴原始加速度（mg） */
    void print_attitude() const
    {
        // 三轴原始量跟随姿态同周期打印：每周期只做一次 printf（一次 write），保持一行一周期。
        std::printf("roll: %+7.2f deg | yaw: %+7.2f deg | pitch: %+7.2f deg | "
                    "gyro_raw: %+8.2f %+8.2f %+8.2f dps | "
                    "acc_raw: %+8.1f %+8.1f %+8.1f mg\n",
                    attitude_roll_deg, attitude_yaw_deg, attitude_pitch_deg,
                    gyro_raw_frd_x_dps, gyro_raw_frd_y_dps, gyro_raw_frd_z_dps,
                    acc_raw_frd_x_mg, acc_raw_frd_y_mg, acc_raw_frd_z_mg);
    }
};
