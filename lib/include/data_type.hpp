#pragma once

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
    int imu_fps = 0;
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
    float attitude_unfiltered_roll_deg = 0.0f;
    float attitude_unfiltered_yaw_deg = 0.0f;
    float attitude_unfiltered_pitch_deg = 0.0f;
    float attitude_direct_integral_roll_deg = 0.0f;
    float attitude_direct_integral_yaw_deg = 0.0f;
    float attitude_direct_integral_pitch_deg = 0.0f;

    void data_update(
        int flying_state, int current_imu_fps,
        float acc_raw_x, float acc_raw_y, float acc_raw_z,
        float acc_filtered_x, float acc_filtered_y, float acc_filtered_z,
        float gyro_raw_x, float gyro_raw_y, float gyro_raw_z,
        float gyro_filtered_x, float gyro_filtered_y, float gyro_filtered_z,
        float roll, float yaw, float pitch,
        float unfiltered_roll, float unfiltered_yaw, float unfiltered_pitch,
        float direct_roll, float direct_yaw, float direct_pitch)
    {
        state = flying_state;
        imu_fps = current_imu_fps;
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
        attitude_unfiltered_roll_deg = unfiltered_roll;
        attitude_unfiltered_yaw_deg = unfiltered_yaw;
        attitude_unfiltered_pitch_deg = unfiltered_pitch;
        attitude_direct_integral_roll_deg = direct_roll;
        attitude_direct_integral_yaw_deg = direct_yaw;
        attitude_direct_integral_pitch_deg = direct_pitch;
    }

    /** @brief 输出当前姿态角 roll / yaw / pitch（单位：度） */
    void print_attitude() const
    {
        std::printf("roll: %+7.2f deg | yaw: %+7.2f deg | pitch: %+7.2f deg\n",
                    attitude_roll_deg, attitude_yaw_deg, attitude_pitch_deg);
    }
};
