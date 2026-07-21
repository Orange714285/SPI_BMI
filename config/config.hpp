#pragma once
namespace config {

// 外环输出 deg/s，内环输出舵机脉宽增量 us。
inline constexpr float ROLL_ANGLE_KP = 3.0f;
inline constexpr float ROLL_ANGLE_KI = 0.0f;
inline constexpr float ROLL_ANGLE_KD = 0.0f;
inline constexpr float YAW_ANGLE_KP = 2.0f;
inline constexpr float YAW_ANGLE_KI = 0.0f;
inline constexpr float YAW_ANGLE_KD = 0.0f;
inline constexpr float X_RATE_KP = 0.5f;
inline constexpr float X_RATE_KI = 0.0f;
inline constexpr float X_RATE_KD = 0.0f;
inline constexpr float Z_RATE_KP = 0.5f;
inline constexpr float Z_RATE_KI = 0.0f;
inline constexpr float Z_RATE_KD = 0.0f;
inline constexpr float SERVO_MAX_DELTA_US = 200.0f;

// 发射检测与发射前姿态锁存周期。
inline constexpr float LAUNCH_ACCEL_X_MG = -1900.0f;
inline constexpr int PREFLIGHT_ATTITUDE_LATCH_INTERVAL_MS = 300;

// 第一控制阶段：发射后 0~1200 ms。
inline constexpr int STAGE_ONE_END_MS = 1200;
inline constexpr float STAGE_ONE_TARGET_X_RATE = 0.0f;
inline constexpr float STAGE_ONE_TARGET_Z_RATE = 0.0f;
inline constexpr float STAGE_ONE_TARGET_ROLL = 0.0f;
inline constexpr float STAGE_ONE_TARGET_YAW = 0.0f;

// 第二控制阶段：发射 1200 ms 后。
inline constexpr float STAGE_TWO_TARGET_X_RATE = 0.0f;
inline constexpr float STAGE_TWO_TARGET_Z_RATE = 0.0f;
inline constexpr float STAGE_TWO_TARGET_ROLL = 0.0f;
inline constexpr float STAGE_TWO_TARGET_YAW = 0.0f;


// 相机参数 (OV5647)
inline constexpr int   CAM_WIDTH             = 640;
inline constexpr int   CAM_HEIGHT            = 480;
inline constexpr int   CAM_CROP_WIDTH        = 1280;
inline constexpr int   CAM_CROP_HEIGHT       = 960;
inline constexpr int   CAM_CROP_X            = 1000;
inline constexpr int   CAM_CROP_Y            = 752;
inline constexpr int   CAM_EXPOSURE_TIME_US  = 500;
inline constexpr float CAM_BRIGHTNESS        = 0.0f;
inline constexpr int   CAM_FPS               = 200;
inline constexpr int   CAM_COLOUR_TEMP       = 6100;

// 检测参数 (HSV 阈值 / ROI / 圆形度)
inline constexpr int    HSV_H_LOW                     = 50;
inline constexpr int    HSV_H_HIGH                    = 80;
inline constexpr int    HSV_S_LOW                     = 22;
inline constexpr int    HSV_S_HIGH                    = 255;
inline constexpr int    HSV_V_LOW                     = 90;
inline constexpr int    HSV_V_HIGH                    = 255;
inline constexpr int    ROI_WIDTH                     = 200;
inline constexpr int    ROI_HEIGHT                    = 200;
inline constexpr double BEST_CIRCULARITY_STANDARD     = 0.60;
inline constexpr int DIFF_THRESHOLD = 80;

// 陀螺仪三维 Kalman 滤波参数
// 观测噪声方差 R，单位 (rad/s)^2。
// 来源：bag/2026_7_20_17_26_51.mcap 静止数据，25887 帧，
// 原始 FRD 角速度的样本方差（ddof=1）。
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_X = 3.72952e-3f;
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_Y = 3.26236e-3f;
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_Z = 4.47215e-3f;

// 初始角速度协方差 P0，单位 (rad/s)^2。
// 默认与单帧观测噪声方差相同。
inline constexpr float GYRO_KF_INITIAL_VARIANCE_X =
    GYRO_KF_MEASUREMENT_VARIANCE_X;
inline constexpr float GYRO_KF_INITIAL_VARIANCE_Y =
    GYRO_KF_MEASUREMENT_VARIANCE_Y;
inline constexpr float GYRO_KF_INITIAL_VARIANCE_Z =
    GYRO_KF_MEASUREMENT_VARIANCE_Z;

// 过程模型的角加速度扰动标准差，单位 rad/s^2。
// Q_k = diag(sigma_alpha^2 * dt^2)。数值越大跟踪越快、平滑越弱。
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_X = 30.0f;
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_Y = 30.0f;
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_Z = 30.0f;

// 图像流传输参数
inline constexpr const char* STREAM_HOST        = "192.168.44.29";
inline constexpr int         STREAM_PORT        = 8080;
inline constexpr int         JPEG_QUALITY       = 70;

}  
