#include <iostream>
#include "spi_bmi055_protocol.hpp"
#include "bmi055_driver.hpp"
#include "attitude_algorithm.hpp"
#include "capture.hpp"
#include "data_type.hpp"
#include "camera.hpp"
#include "detector.hpp"
#include "image_streamer.hpp"
#include "pid_control.hpp"
#include "config.hpp"
#include "tools/buzzer.hpp"

#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <tools/cpu_monitor.hpp>
#include <tools/frame_counter.hpp>

std::atomic<bool> g_running{true};

std::atomic<VisionData> g_vision_data;

// 飞行状态：0=PRE_FLIGHT, 1=FLIGHT；control 线程写入，vision 线程读取
std::atomic<int> g_flying_state{0};

void signal_handler(int)
{
    g_running.store(false);
}
void dart_control(Capturer& capturer);
void dart_vision(Capturer& capturer);
int main() 
{
    Buzzer buzzer;
    buzzer.start(10);
    // 创建唯一 Capturer，电控和视频写入同一个 .mcap 文件
    Capturer capturer;
    if (!capturer.init())
    {
        std::cerr << "[ERROR] Capturer init failed!" << std::endl;
        capturer.finish();
        return -1;
    }

    std::thread dart_control_thread(dart_control, std::ref(capturer));
    std::thread dart_vision_thread(dart_vision, std::ref(capturer));
    dart_control_thread.join();
    dart_vision_thread.join();
    capturer.finish();
    std::cout << "[INFO] Exiting cleanly." << std::endl;
}

void dart_control(Capturer& capturer)
{   
    Buzzer buzzer;
    BMI055 bmi055;
    AccAttitudeAlgorithmer acc_attitude_algorithmer;
    GyroAttitudeAlgorithmer gyro_attitude_algorithmer;
    CarData dart_data;
    CascadedPIDController attitude_controller;
    float launch_roll = 0.0f;
    float launch_pitch = 0.0f;
    auto last_attitude_latch_timer = std::chrono::steady_clock::now();

    if (!bmi055.BMI055_init())
    {
        std::cerr << "[ERROR] BMI055 init failed!" << std::endl;
        return ;
    }
    std::cerr << "[INFO] BMI055 init successed!" << std::endl;

    // 初始化四个舵机并归中。
    if (!attitude_controller.initialize_servos(1500, 1500, 1500, 1500)) {
        std::cerr << "[ERROR] servo initialization failed!" << std::endl;
        return;
    }

    signal(SIGINT, signal_handler);

    // 使用全局 g_flying_state (0=PRE_FLIGHT, 1=FLIGHT)，供 vision 线程可见
    g_flying_state.store(0);  // 初始为 PRE_FLIGHT
    CpuMonitor::sample();   // 建立基准

    auto flying_start_timer = std::chrono::steady_clock::now();
    while (g_running)
    {
        bmi055.index++;

        // ==================== 数据采集 ====================
        if (g_flying_state.load() == 0)  // PRE_FLIGHT
        {
            // 着陆态：以加速度计为时钟源
            if (!bmi055.acc_wait_for_new_info())
            {
                std::cerr << "[ERROR] acc_wait_for_new_info failed!" << std::endl;
                return ;
            }
            if (!bmi055.acc_get_accd_all_mg())
            {
                std::cerr << "[ERROR] acc_get_accd_mg failed!" << std::endl;
                return ;
            }
            if (!bmi055.gyr_get_rate_all_dps())
            {
                std::cerr << "[ERROR] gyro get all rate dps failed!" << std::endl;
                return ;
        // ==================
            }
        }
        else // FLIGHT
        {
            // 飞行态：以陀螺仪为时钟源
            if (!bmi055.gyr_wait_for_new_info())
            {
                std::cerr << "[ERROR] gyr_wait_for_new_info failed!" << std::endl;
                return ;
            }
            if (!bmi055.gyr_get_rate_all_dps())
            {
                std::cerr << "[ERROR] gyr_get_rate_all_deg_per_s failed!" << std::endl;
                return ;
            }
            if (!bmi055.acc_get_accd_all_mg())
            {
                std::cerr << "[ERROR] acc_get_accd_mg failed!" << std::endl;
                return ;
            }
        }
			// ================ imu -> frd 转系 ================
        acc_attitude_algorithmer.transform_coordinate(
                bmi055.m_acc_imu_accd_x_mg,
                bmi055.m_acc_imu_accd_y_mg,
                bmi055.m_acc_imu_accd_z_mg);
        gyro_attitude_algorithmer.transform_coordinate(
                bmi055.m_gyr_rate_x_dps,
                bmi055.m_gyr_rate_y_dps,
                bmi055.m_gyr_rate_z_dps);
        // ==================== 姿态解算与状态切换 ====================
        if (g_flying_state.load() == 0)  // PRE_FLIGHT
        {
            if (acc_attitude_algorithmer.m_frd_acc_x < config::LAUNCH_ACCEL_X_MG)
            {
                // 发射帧使用最后保存的姿态初始化陀螺仪积分。
                gyro_attitude_algorithmer.algorithm(launch_roll, launch_pitch);
                flying_start_timer = std::chrono::steady_clock::now();
                g_flying_state.store(1);
                std::cout << "I am flying!" << std::endl;
            }
            else
            {
                // 发射前每隔 300 ms 保存一次加速度计姿态。
                acc_attitude_algorithmer.algorithmer();
                const auto now = std::chrono::steady_clock::now();
                if (now - last_attitude_latch_timer
                    >= std::chrono::milliseconds(
                        config::PREFLIGHT_ATTITUDE_LATCH_INTERVAL_MS))
                {
                    launch_roll = acc_attitude_algorithmer.m_roll;
                    launch_pitch = acc_attitude_algorithmer.m_pitch;
                    last_attitude_latch_timer = now;
                }
            }
        }
        else
        {
            // 每个陀螺仪周期更新飞行姿态。
            gyro_attitude_algorithmer.algorithm(
                launch_roll,
                launch_pitch);
        }

        // ==================== 统一姿态 ====================
        int flying_state = g_flying_state.load();
        float roll  = (flying_state == 0) ? acc_attitude_algorithmer.m_roll  : gyro_attitude_algorithmer.m_roll;
        float pitch = (flying_state == 0) ? acc_attitude_algorithmer.m_pitch : gyro_attitude_algorithmer.m_pitch;
        float yaw   = (flying_state == 0) ? 0.0f : gyro_attitude_algorithmer.m_yaw;

        if (flying_state == 1)
        {
            // 输入目标和实测姿态，执行一次双环 PID 控制。
            auto flying_process_timer = std::chrono::steady_clock::now();
            auto flying_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(flying_process_timer - flying_start_timer);
            const bool stage_one =
                flying_duration_ms.count() < config::STAGE_ONE_END_MS;
            const float target_x_rate = stage_one
                ? config::STAGE_ONE_TARGET_X_RATE : config::STAGE_TWO_TARGET_X_RATE;
            const float target_z_rate = stage_one
                ? config::STAGE_ONE_TARGET_Z_RATE : config::STAGE_TWO_TARGET_Z_RATE;
            const float target_roll = stage_one
                ? config::STAGE_ONE_TARGET_ROLL : config::STAGE_TWO_TARGET_ROLL;
            const float target_yaw = stage_one
                ? config::STAGE_ONE_TARGET_YAW : config::STAGE_TWO_TARGET_YAW;

            attitude_controller.control(
                target_x_rate,
                target_z_rate,
                target_roll,
                target_yaw,
                gyro_attitude_algorithmer.m_frd_gyro_x,
                gyro_attitude_algorithmer.m_frd_gyro_z,
                roll,
                yaw,
                gyro_attitude_algorithmer.m_dt_s);

        }

        CpuMonitor::sample();
        FrameCounter::tick();
        dart_data.data_update(
            acc_attitude_algorithmer.m_frd_acc_x,
            acc_attitude_algorithmer.m_frd_acc_y,
            acc_attitude_algorithmer.m_frd_acc_z,
            gyro_attitude_algorithmer.m_frd_gyro_x,       // 滤波后 (Kalman 输出)
            gyro_attitude_algorithmer.m_frd_gyro_y,
            gyro_attitude_algorithmer.m_frd_gyro_z,
            gyro_attitude_algorithmer.raw_frd_gyro_x(),   // 原始测量 (Kalman 输入)
            gyro_attitude_algorithmer.raw_frd_gyro_y(),
            gyro_attitude_algorithmer.raw_frd_gyro_z(),
            roll, pitch, yaw,                              // 方案B：KF角速度+四元数姿态
            gyro_attitude_algorithmer.m_Euler_roll,        // KF前角速度的四元数姿态
            gyro_attitude_algorithmer.m_Euler_pitch,
            gyro_attitude_algorithmer.m_Euler_yaw,
            gyro_attitude_algorithmer.m_diff_roll,          // 差值 (滤波后 - 原始)
            gyro_attitude_algorithmer.m_diff_pitch,
            gyro_attitude_algorithmer.m_diff_yaw,
            gyro_attitude_algorithmer.m_quat_roll,           // 四元数积分欧拉角
            gyro_attitude_algorithmer.m_quat_pitch,
            gyro_attitude_algorithmer.m_quat_yaw,
            static_cast<int>(bmi055.index), static_cast<int>(CpuMonitor::usage()), static_cast<int>(FrameCounter::fps()), 
            flying_state);
        
        if (flying_state == 1)  // FLIGHT
        {
            capturer.update_car_data(dart_data);
            // gyro_attitude_algorithmer.print_attitude_comparison();
        }    
        else 
        {
            capturer.update_car_data(dart_data);
            // acc_attitude_algorithmer.print_attitude_comparison();
        }

        if (acc_attitude_algorithmer.m_frd_acc_x > 1900 && flying_state == 1)
        {
            g_running.store(0);
            std::cerr << "[INFO] Flying Ending " << std::endl;
        }

    }
    std::cout << "[INFO] dart_control thread received SIGINT, exiting cleanly." << std::endl;
    return ;
}
void dart_vision(Capturer& capturer)
{
    signal(SIGINT, signal_handler);
    Camera ov5647;
    Detector detector;
    if (!ov5647.start())
    {
        ov5647.stop();
        std::cerr << "[ERROR] Camera init failed!" << std::endl;
        return ;
    }
    int index = 0;
    while (g_running.load())
    {
        cv::Mat frame = ov5647.wait_and_get_latest_frame();
        index ++;
        if (index%2 ==0)
        {
            // capturer.write_video_frame(frame, 100);
        }
        detector.detect_and_draw_lights(frame);

        VideoFrameCounter::tick();  // 每检测一帧 tick
        VisionData vd = detector.vision_data();
        vd.m_video_fps = VideoFrameCounter::fps();
        g_vision_data.store(vd);    // 保留：供 control 线程读取目标坐标

        if (!g_running.load())
            break;
        if (frame.empty())
        {
            std::cout << "Frame empty!" << std::endl;
            break;
        }
        // ── 视觉检测数据写入 MCAP（vision 线程独立写入）──
        capturer.write_vision_data(vd.m_target_pixel_x, vd.m_target_pixel_y,
                                   vd.m_target_status, vd.m_frame_dt_ms, vd.m_video_fps);
    }

    ov5647.stop();
    std::cout << "[INFO] dart_vision thread exiting cleanly." << std::endl;
    return ;
}
