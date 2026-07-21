# pragma once
#include <iostream>
#include <string_view>
#include <cstdint>
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
    float acc_frd_x_mg;
    float acc_frd_y_mg;
    float acc_frd_z_mg;
    float gyro_frd_x_dps;         // 滤波后的角速度 (Kalman 输出)
    float gyro_frd_y_dps;
    float gyro_frd_z_dps;
    float gyro_raw_frd_x_dps;     // 原始角速度 (Kalman 输入，FRD 机体系)
    float gyro_raw_frd_y_dps;
    float gyro_raw_frd_z_dps;
    float euler_roll;     // 方案B输出的四元数 ZYX 欧拉角
    float euler_yaw;
    float euler_pitch;
    float roll_raw;       // KF 前原始角速度的四元数积分姿态
    float pitch_raw;
    float yaw_raw;
    float diff_roll;      // 差值 = 滤波后 - 原始
    float diff_pitch;
    float diff_yaw;
    float quat_roll;          // 四元数积分提取的欧拉角 (内旋 roll-yaw-pitch)
    float quat_yaw;
    float quat_pitch;
    int IMU_data_index;
    int IMU_fps;
    int m_cpu_usage;
    int m_state;

    void data_update(float acc_x, float acc_y, float acc_z,
                   float gyro_x, float gyro_y, float gyro_z,
                   float gyro_raw_x, float gyro_raw_y, float gyro_raw_z,
                   float r, float p, float y,
                   float r_raw, float p_raw, float y_raw,
                   float r_diff, float p_diff, float y_diff,
                   float r_q, float p_q, float y_q,
                   int idx, int cpu_usage, int imu_fps,
                   int state)
    {
        acc_frd_x_mg  = acc_x;
        acc_frd_y_mg  = acc_y;
        acc_frd_z_mg  = acc_z;
        gyro_frd_x_dps = gyro_x;
        gyro_frd_y_dps = gyro_y;
        gyro_frd_z_dps = gyro_z;
        gyro_raw_frd_x_dps = gyro_raw_x;
        gyro_raw_frd_y_dps = gyro_raw_y;
        gyro_raw_frd_z_dps = gyro_raw_z;
        euler_roll  = r;
        euler_pitch = p;
        euler_yaw   = y;
        roll_raw   = r_raw;
        pitch_raw  = p_raw;
        yaw_raw    = y_raw;
        diff_roll  = r_diff;
        diff_pitch = p_diff;
        diff_yaw   = y_diff;
        quat_roll  = r_q;
        quat_pitch = p_q;
        quat_yaw   = y_q;
        IMU_data_index = idx;
        m_cpu_usage = cpu_usage;
        IMU_fps = imu_fps;
        m_state = state;
    }

    void print(std::string_view state_label) const
    {
        std::cout << "\033[H\033[J";
        std::cout << "============================" << std::endl;
        std::cout << "  [" << state_label << "]" << std::endl;
        std::cout << "============================" << std::endl;
        std::cout << "[INFO] acc_frd_x_mg: "  << acc_frd_x_mg  << std::endl;
        std::cout << "[INFO] acc_frd_y_mg: "  << acc_frd_y_mg  << std::endl;
        std::cout << "[INFO] acc_frd_z_mg: "  << acc_frd_z_mg  << std::endl;
        std::cout << "[INFO] gyr_frd_x_dps: " << gyro_frd_x_dps << std::endl;
        std::cout << "[INFO] gyr_frd_y_dps: " << gyro_frd_y_dps << std::endl;
        std::cout << "[INFO] gyr_frd_z_dps: " << gyro_frd_z_dps << std::endl;
        std::cout << "[INFO] euler_roll:  " << euler_roll  << std::endl;
        std::cout << "[INFO] euler_pitch: " << euler_pitch << std::endl;
        std::cout << "[INFO] euler_yaw:   " << euler_yaw   << std::endl;
        std::cout << "[INFO] index: " << IMU_data_index << std::endl;
        std::cout << "============================" << std::endl;
        std::cout << std::flush;
    }
};
