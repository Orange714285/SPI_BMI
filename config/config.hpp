#pragma once
namespace config {

// ==== 1. 姿态控制增益（外环 deg->deg/s，内环 dps->舵机脉宽增量 us）====


// --- Roll 双环 ---
inline constexpr float ROLL_ANGLE_KP = 10.0f;
inline constexpr float ROLL_ANGLE_KI = 1.0f;
inline constexpr float ROLL_ANGLE_KD = 0.0f;
inline constexpr float X_RATE_KP = 1.5f;
inline constexpr float X_RATE_KI = 1.0f;
inline constexpr float X_RATE_KD = 0.0f;

// --- Yaw 双环 ---
inline constexpr float YAW_ANGLE_KP = 10.0f;
inline constexpr float YAW_ANGLE_KI = 6.0f;
inline constexpr float YAW_ANGLE_KD = 0.0f;
inline constexpr float Z_RATE_KP = 3.0f;
inline constexpr float Z_RATE_KI = 2.0f;
inline constexpr float Z_RATE_KD = 0.0f;

// --- Pitch 双环 ---
// 增益沿用 2026/10/7 已验证的取值，尚未针对新剖面重调。
// 首飞重点看 pid_inner_pitch 峰值是否逼近 SERVO_MAX_DELTA_US。
inline constexpr float PITCH_ANGLE_KP = 5.0f;
inline constexpr float PITCH_ANGLE_KI = 0.0f;
inline constexpr float PITCH_ANGLE_KD = 0.0f;
inline constexpr float Y_RATE_KP = 6.0f;
inline constexpr float Y_RATE_KI = 0.0f;
inline constexpr float Y_RATE_KD = 0.0f;
// pitch 通道极性：台架验证若发现方向相反，置 -1.0f 即可，无需改代码。
inline constexpr float PITCH_CHANNEL_SIGN = 1.0f;

// ==== 2. 舵机与混控 ====


inline constexpr float SERVO_MAX_DELTA_US = 800.0f;
inline constexpr int SERVO_PWM_FREQUENCY_HZ = 320;
inline constexpr int SERVO_PWM_PERIOD_US = 3125;
inline constexpr int SERVO_MIN_PULSE_US = 700;
inline constexpr int SERVO_MAX_PULSE_US = 2300;

// 舵机中位（us）：顺序与 initialize_servos 参数一致 —— 左上、右上、右下、左下。
inline constexpr int SERVO_CENTER_LEFT_UPPER_US = 1405;
inline constexpr int SERVO_CENTER_RIGHT_UPPER_US = 1430;
inline constexpr int SERVO_CENTER_RIGHT_LOWER_US = 1540;
inline constexpr int SERVO_CENTER_LEFT_LOWER_US = 1510;

// ==== 3. 飞行时序：发射 -> 分级目标 -> 延迟开控 -> Pitch 剖面 ====


// --- 发射检测与姿态初始化 ---
inline constexpr float LAUNCH_ACCEL_X_MG = 3800.0f;
// 发射加速度阈值需持续的时间（毫秒），防瞬间噪声误触发。
inline constexpr int LAUNCH_ACCEL_CONFIRM_MS = 20;
// 发射瞬间用于初始化姿态积分的标称发射姿态（deg）。
// 发射架几何固定，且发射前加速度计被推力/振动污染、锁存值不可靠，故直接给定。
inline constexpr float LAUNCH_INITIAL_ROLL_DEG = 0.0f;
inline constexpr float LAUNCH_INITIAL_PITCH_DEG = 30.0f;

// --- 分级控制目标 ---
// 第一阶段：发射后 0 ~ STAGE_ONE_END_MS。
inline constexpr int STAGE_ONE_END_MS = 1200;
inline constexpr float STAGE_ONE_TARGET_X_RATE = 0.0f;
inline constexpr float STAGE_ONE_TARGET_Z_RATE = 0.0f;
inline constexpr float STAGE_ONE_TARGET_ROLL = 0.0f;
inline constexpr float STAGE_ONE_TARGET_YAW = 0.0f;

// 第二阶段：发射 STAGE_ONE_END_MS 之后。偏航目标设为 15 度，用于转向测试。
inline constexpr float STAGE_TWO_TARGET_X_RATE = 0.0f;
inline constexpr float STAGE_TWO_TARGET_Z_RATE = 0.0f;
inline constexpr float STAGE_TWO_TARGET_ROLL = 0.0f;
inline constexpr float STAGE_TWO_TARGET_YAW = 15.0f;
// 转向斜坡速率（dps）：目标角以该速率从 STAGE_ONE 爬到 STAGE_TWO，爬升期间兼作角速度前馈。
// 用斜坡而非阶跃：阶跃会激励 roll 交叉耦合（phi_dot 里含 r*tan(theta) 项）。
inline constexpr float STAGE_TWO_YAW_RATE_DPS = 150.0f;

// --- 延迟开控 ---
// 出膛后约 50ms 内陀螺被推力/导轨振动污染（实测滤波角速度出现 ±300dps 非物理尖峰），
// 此时闭环只会打出错误且饱和的舵面指令，故延迟到离架后再开控。
// 延迟期间 pitch_target_deg 仍照常计算并记录，四舵保持中位不动。
inline constexpr int CONTROL_START_DELAY_MS = 50;

// --- Pitch 剖面 ---
// 发射后 0 ~ PITCH_PROFILE_DURATION_MS 内，目标俯仰角由 START 线性变到 END，
// 该斜率同时作为内环的角速度前馈。
// 取自 13 发「pitch 增益为 0」自由弹道的最小二乘拟合：pitch = 30.00 - 38.16*t（RMS 1.50 度）。
// START 锚定在 LAUNCH_INITIAL_PITCH_DEG：t=0 时目标误差为 0，不会凭空产生发射瞬间的舵指令。
// DURATION 固定 1635 ms，故 END = 30.00 - 38.16 * 1.635 = -32.4 度。
inline constexpr float PITCH_PROFILE_START_DEG = 30.0f;
inline constexpr float PITCH_PROFILE_END_DEG = -32.4f;
inline constexpr int PITCH_PROFILE_DURATION_MS = 1635;

// ==== 4. 记录与回传 ====


// 发射确认前保留在内存中的 CarData 时长，撞击判定后回写进 MCAP。
// 只需覆盖发射检测（确认 20 ms）与释放瞬态，200 ms 已足够。
inline constexpr int CAR_DATA_PRE_RECORD_MS = 200;

// MCAP 自动上传目标。
inline constexpr const char* MCAP_UPLOAD_USER = "orange";
inline constexpr const char* MCAP_UPLOAD_HOST = "10.42.0.1";
inline constexpr const char* MCAP_UPLOAD_DIRECTORY = "/home/orange/pi-workspace/DartControl/bag";
// 回传成功后是否删除树莓派本地文件（防止 bag/ 逐发累积撑爆存储）。
// 只有 scp 返回 0（传输成功）才删；失败保留原文件以便重传。
inline constexpr bool MCAP_DELETE_LOCAL_AFTER_UPLOAD = true;

// ==== 5. 诊断：数据健康判定 ====


// 控制循环的节拍由 CPU 吞吐决定，故该发数据是否可用取决于运行期间主频与帧率是否稳定；
// 结论写在 config 快照的 [PI_HEALTH] 行里。
// 启动暖机期：这之前的样本不参与统计 —— 程序启动阶段设备初始化、CPU 策略爬坡、
// FrameCounter 首个统计窗口不完整，那几秒的数字会伪装成「降频/掉帧」。
inline constexpr int HEALTH_WARMUP_MS = 5000;
// SoC 温度超过该值即判为可疑（摄氏度）。
inline constexpr double HEALTH_TEMP_WARN_C = 75.0;
// 帧率离散度 (max-min)/max 超过该百分比即判为可疑。
inline constexpr double HEALTH_FPS_SPREAD_WARN_PCT = 10.0;

// ==== 6. 传感器 Kalman 滤波 ====


// --- 加速度计 ---
// 观测噪声方差 R：静止 56129 帧测得，单位 mg^2。
inline constexpr float ACC_KF_MEASUREMENT_VARIANCE_X = 18.58767f;
inline constexpr float ACC_KF_MEASUREMENT_VARIANCE_Y = 15.90837f;
inline constexpr float ACC_KF_MEASUREMENT_VARIANCE_Z = 27.18441f;
// 初始协方差 P0。
inline constexpr float ACC_KF_INITIAL_VARIANCE_X = ACC_KF_MEASUREMENT_VARIANCE_X;
inline constexpr float ACC_KF_INITIAL_VARIANCE_Y = ACC_KF_MEASUREMENT_VARIANCE_Y;
inline constexpr float ACC_KF_INITIAL_VARIANCE_Z = ACC_KF_MEASUREMENT_VARIANCE_Z;
// 过程模型的角加速度扰动标准差。
inline constexpr float ACC_KF_JERK_STD_X = 3500.0f;
inline constexpr float ACC_KF_JERK_STD_Y = 3500.0f;
inline constexpr float ACC_KF_JERK_STD_Z = 3500.0f;

// --- 陀螺仪 ---
// 观测噪声方差 R：静止 25887 帧测得，单位 (rad/s)^2。
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_X = 3.72952e-3f;
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_Y = 3.26236e-3f;
inline constexpr float GYRO_KF_MEASUREMENT_VARIANCE_Z = 4.47215e-3f;
// 初始协方差 P0，默认与单帧观测噪声方差相同。
inline constexpr float GYRO_KF_INITIAL_VARIANCE_X = GYRO_KF_MEASUREMENT_VARIANCE_X;
inline constexpr float GYRO_KF_INITIAL_VARIANCE_Y = GYRO_KF_MEASUREMENT_VARIANCE_Y;
inline constexpr float GYRO_KF_INITIAL_VARIANCE_Z = GYRO_KF_MEASUREMENT_VARIANCE_Z;
// 过程模型的角加速度扰动标准差，单位 rad/s^2；Q_k = diag(sigma_alpha^2 * dt^2)。
// 数值越大跟踪越快、平滑越弱。
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_X = 40.0f;
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_Y = 40.0f;
inline constexpr float GYRO_KF_ANGULAR_ACCEL_STD_Z = 40.0f;

// ==== 7. 视觉：相机 / 检测 / 图像流 ====


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

// 图像流传输参数
inline constexpr const char* STREAM_HOST        = "192.168.44.29";
inline constexpr int         STREAM_PORT        = 8080;
inline constexpr int         JPEG_QUALITY       = 70;

}
