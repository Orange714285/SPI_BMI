#pragma once
namespace config {

// 外环输出 deg/s，内环输出舵机脉宽增量 us。
inline constexpr float ROLL_ANGLE_KP = 2.0f;
inline constexpr float ROLL_ANGLE_KI = 0.0f;
inline constexpr float ROLL_ANGLE_KD = 0.0f;
inline constexpr float YAW_ANGLE_KP = 2.0f;
inline constexpr float YAW_ANGLE_KI = 0.0f;
inline constexpr float YAW_ANGLE_KD = 0.0f;
inline constexpr float X_RATE_KP = 3.0f;
inline constexpr float X_RATE_KI = 0.0f;
inline constexpr float X_RATE_KD = 0.0f;
inline constexpr float Z_RATE_KP = 3.0f;
inline constexpr float Z_RATE_KI = 0.0f;
inline constexpr float Z_RATE_KD = 0.0f;
inline constexpr float SERVO_MAX_DELTA_US = 200.0f;
inline constexpr int SERVO_PWM_FREQUENCY_HZ = 320;
inline constexpr int SERVO_PWM_PERIOD_US = 3125;
inline constexpr int SERVO_MIN_PULSE_US = 800;
inline constexpr int SERVO_MAX_PULSE_US = 2200;

// 舵机中位（μs）：顺序与 initialize_servos 参数一致：左上、右上、右下、左下。
inline constexpr int SERVO_CENTER_LEFT_UPPER_US = 1500;
inline constexpr int SERVO_CENTER_RIGHT_UPPER_US = 1550;
inline constexpr int SERVO_CENTER_RIGHT_LOWER_US = 1500;
inline constexpr int SERVO_CENTER_LEFT_LOWER_US = 1430;

// 发射检测与发射前姿态锁存周期。
inline constexpr float LAUNCH_ACCEL_X_MG = -3800.0f;
// 发射加速度阈值需持续的时间（毫秒），防瞬间噪声误触发。
inline constexpr int LAUNCH_ACCEL_CONFIRM_MS = 20;
inline constexpr int PREFLIGHT_ATTITUDE_LATCH_INTERVAL_MS = 500;

// 第一控制阶段：发射后 0~1200 ms。
inline constexpr int STAGE_ONE_END_MS = 1200;
inline constexpr float STAGE_ONE_TARGET_X_RATE = 5.0f;
inline constexpr float STAGE_ONE_TARGET_Z_RATE = 5.0f;
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

// 静止数据 56129 帧测得的三轴加速度观测噪声方差，单位 mg^2。
inline constexpr float ACC_KF_MEASUREMENT_VARIANCE_X = 18.58767f;
inline constexpr float ACC_KF_MEASUREMENT_VARIANCE_Y = 15.90837f;
inline constexpr float ACC_KF_MEASUREMENT_VARIANCE_Z = 27.18441f;
inline constexpr float ACC_KF_INITIAL_VARIANCE_X = ACC_KF_MEASUREMENT_VARIANCE_X;
inline constexpr float ACC_KF_INITIAL_VARIANCE_Y = ACC_KF_MEASUREMENT_VARIANCE_Y;
inline constexpr float ACC_KF_INITIAL_VARIANCE_Z = ACC_KF_MEASUREMENT_VARIANCE_Z;
inline constexpr float ACC_KF_JERK_STD_X = 3500.0f;
inline constexpr float ACC_KF_JERK_STD_Y = 3500.0f;
inline constexpr float ACC_KF_JERK_STD_Z = 3500.0f;

// 陀螺仪三维 Kalman 滤波参数
// 观测噪声方差 R，单位 (rad/s)^2。
// 来源：bag/2026_7_20_17_26_51.mcap 静止数据，25887 帧，
// 原始 FRD 角速度的样本方差（ddof=1）。
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_X = 3.72952e-3f;
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_Y = 3.26236e-3f;
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_Z = 4.47215e-3f;

// 初始角速度协方差 P0，单位 (rad/s)^2。
// 默认与单帧观测噪声方差相同。
inline constexpr float GYRO_KF_INITIAL_VARIANCE_X = GYRO_KF_MEASUREMENT_VARIANCE_X;
inline constexpr float GYRO_KF_INITIAL_VARIANCE_Y = GYRO_KF_MEASUREMENT_VARIANCE_Y;
inline constexpr float GYRO_KF_INITIAL_VARIANCE_Z = GYRO_KF_MEASUREMENT_VARIANCE_Z;

// 过程模型的角加速度扰动标准差，单位 rad/s^2。
// Q_k = diag(sigma_alpha^2 * dt^2)。数值越大跟踪越快、平滑越弱。
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_X = 40.0f;
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_Y = 40.0f;
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_Z = 40.0f;

// 图像流传输参数
inline constexpr const char* STREAM_HOST        = "192.168.44.29";
inline constexpr int         STREAM_PORT        = 8080;
inline constexpr int         JPEG_QUALITY       = 70;

// MCAP 自动上传目标。
inline constexpr const char* MCAP_UPLOAD_USER = "orange";
inline constexpr const char* MCAP_UPLOAD_HOST = "10.42.0.1";
inline constexpr const char* MCAP_UPLOAD_DIRECTORY =
    "/home/orange/pi-workspace/DartControl/bag";

}  
