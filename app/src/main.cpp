#include <iostream>
#include "spi_bmi055_protocol.hpp"
#include "bmi055_driver.hpp"
#include "attitude_algorithm.hpp"
#include "capture.hpp"
#include "mcap_uploader.hpp"
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
std::atomic<int> g_flying_state{0};// 飞行状态：0=PRE_FLIGHT，1=FLIGHT，2=FLIGHT_END。
std::atomic<long long> g_flight_duration_ms{0};// 当前飞行时长（毫秒），由 dart_control 每循环更新，全程序可读；未飞行时为 0。

void signal_handler(int)
{
    g_running.store(false);
}
void dart_control(Capturer& capturer);
void dart_vision(Capturer& caspturer);
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
    upload_mcap(capturer.mcap_path());
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
    if (!attitude_controller.initialize_servos(
            config::SERVO_CENTER_LEFT_UPPER_US,
            config::SERVO_CENTER_RIGHT_UPPER_US,
            config::SERVO_CENTER_RIGHT_LOWER_US,
            config::SERVO_CENTER_LEFT_LOWER_US)) {
        std::cerr << "[ERROR] servo initialization failed!" << std::endl;
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "[INFO] servo initialization sucessed!" << std::endl;
    signal(SIGINT, signal_handler);

    // 使用全局 g_flying_state (0=PRE_FLIGHT, 1=FLIGHT)，供 vision 线程可见
    g_flying_state.store(0);  // 初始为 PRE_FLIGHT
    CpuMonitor::sample();   // 建立基准

    auto flying_start_timer = std::chrono::steady_clock::now();
    // 发射加速度阈值持续时间确认：条件连续成立 LAUNCH_ACCEL_CONFIRM_MS 才触发。
    auto launch_cond_start = std::chrono::steady_clock::now();
    bool launch_cond_active = false;
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
        // ======================================================
		// ==================== imu -> frd 转系 ==================
        // ======================================================

        acc_attitude_algorithmer.transform_coordinate(
                bmi055.m_acc_imu_accd_x_mg,
                bmi055.m_acc_imu_accd_y_mg,
                bmi055.m_acc_imu_accd_z_mg);
        gyro_attitude_algorithmer.transform_coordinate(
                bmi055.m_gyr_rate_x_dps,
                bmi055.m_gyr_rate_y_dps,
                bmi055.m_gyr_rate_z_dps);
        // 每帧同时计算滤波前、后的加速度计姿态用于记录对照。
        acc_attitude_algorithmer.algorithmer();

        // ========================================================
        // =================== 姿态解算与状态切换 ====================
        // ========================================================

        if (g_flying_state.load() == 0)  // PRE_FLIGHT
        {
            if (acc_attitude_algorithmer.m_frd_acc_x > -config::LAUNCH_ACCEL_X_MG)
            {
                // 加速度超阈值需连续持续 LAUNCH_ACCEL_CONFIRM_MS 才判定发射。
                const auto now = std::chrono::steady_clock::now();
                if (!launch_cond_active)
                {
                    launch_cond_active = true;   // 条件首次成立，记录起点
                    launch_cond_start = now;
                }
                if (now - launch_cond_start
                    >= std::chrono::milliseconds(config::LAUNCH_ACCEL_CONFIRM_MS))
                {
                    // 发射帧使用最后保存的姿态初始化陀螺仪积分。
                    gyro_attitude_algorithmer.initialize(launch_roll, launch_pitch);
                    flying_start_timer = now;
                    g_flight_duration_ms.store(0);  // 发射瞬间归零计时
                    g_flying_state.store(1);
                    std::cout << "I am flying!" << std::endl;
                }
            }
            else
            {
                launch_cond_active = false;  // 条件中断，重新计时

                // 发射前每隔 300 ms 保存一次加速度计姿态。
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
            gyro_attitude_algorithmer.update();
        }

        // ==================== 统一姿态 ====================
        int flying_state = g_flying_state.load();
        float roll  = (flying_state == 0) ? acc_attitude_algorithmer.m_roll  : gyro_attitude_algorithmer.m_roll;
        float pitch = (flying_state == 0) ? acc_attitude_algorithmer.m_pitch : gyro_attitude_algorithmer.m_pitch;
        float yaw   = (flying_state == 0) ? 0.0f : gyro_attitude_algorithmer.m_yaw;
        float unfiltered_roll = (flying_state == 0)
            ? acc_attitude_algorithmer.m_raw_roll
            : gyro_attitude_algorithmer.m_unfiltered_roll;
        float unfiltered_pitch = (flying_state == 0)
            ? acc_attitude_algorithmer.m_raw_pitch
            : gyro_attitude_algorithmer.m_unfiltered_pitch;
        float unfiltered_yaw = (flying_state == 0)
            ? 0.0f : gyro_attitude_algorithmer.m_unfiltered_yaw;
        
        // 检测到撞击后，将最后一帧状态记录为 2。
        if (flying_state == 1
            && acc_attitude_algorithmer.m_frd_acc_x < config::LAUNCH_ACCEL_X_MG
            && g_flight_duration_ms.load()>1000)
        {
            flying_state = 2;
            g_flying_state.store(2);
            std::cerr << "[INFO] Flying Ending " << std::endl;
        }

        // 控制
        if (flying_state == 1)
        {
            // 输入目标和实测姿态，执行一次双环 PID 控制。
            auto flying_process_timer = std::chrono::steady_clock::now();
            auto flying_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(flying_process_timer - flying_start_timer);
            g_flight_duration_ms.store(flying_duration_ms.count());  // 更新全局飞行时长
            const bool stage_one =
                g_flight_duration_ms.load() < config::STAGE_ONE_END_MS;
            const float target_x_rate = stage_one ? config::STAGE_ONE_TARGET_X_RATE : config::STAGE_TWO_TARGET_X_RATE;
            const float target_z_rate = stage_one ? config::STAGE_ONE_TARGET_Z_RATE : config::STAGE_TWO_TARGET_Z_RATE;
            const float target_roll = stage_one ? config::STAGE_ONE_TARGET_ROLL : config::STAGE_TWO_TARGET_ROLL;
            const float target_yaw = stage_one ? config::STAGE_ONE_TARGET_YAW : config::STAGE_TWO_TARGET_YAW;

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
        if (flying_state == 1)
        dart_data.data_update(
            flying_state,
            static_cast<int>(FrameCounter::fps()),
            acc_attitude_algorithmer.raw_frd_acc_x(),
            acc_attitude_algorithmer.raw_frd_acc_y(),
            acc_attitude_algorithmer.raw_frd_acc_z(),
            acc_attitude_algorithmer.m_frd_acc_x,
            acc_attitude_algorithmer.m_frd_acc_y,
            acc_attitude_algorithmer.m_frd_acc_z,
            gyro_attitude_algorithmer.raw_frd_gyro_x(),
            gyro_attitude_algorithmer.raw_frd_gyro_y(),
            gyro_attitude_algorithmer.raw_frd_gyro_z(),
            gyro_attitude_algorithmer.m_frd_gyro_x,
            gyro_attitude_algorithmer.m_frd_gyro_y,
            gyro_attitude_algorithmer.m_frd_gyro_z,
            roll, yaw, pitch,
            unfiltered_roll,
            unfiltered_yaw,
            unfiltered_pitch,
            gyro_attitude_algorithmer.m_wrong_roll,
            gyro_attitude_algorithmer.m_wrong_yaw,
            gyro_attitude_algorithmer.m_wrong_pitch);

        capturer.update_car_data(dart_data);
        dart_data.print_attitude();
        if (flying_state == 2) g_running.store(false);


    }
    const std::array<int, 4> ending_pose{2300,2300,2300,2300};
    attitude_controller.servo_driver_.write_pulsewidths(ending_pose);
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
