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
#include "pi_health_monitor.hpp"
#include "tools/buzzer.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
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

    try
    {
        Capturer capturer;
        pi_health::append_state_to(capturer.config_snapshot_path(), "start");

        std::thread dart_control_thread(dart_control, std::ref(capturer));
        // std::thread dart_vision_thread(dart_vision, std::ref(capturer));
        dart_control_thread.join();
        // dart_vision_thread.join();
        capturer.finish();
        // 飞行结束后的运行状态（线程已 join），先落盘再上传。
        pi_health::append_state_to(capturer.config_snapshot_path(), "end");
        upload_mcap({capturer.mcap_path(), capturer.config_snapshot_path()});
    }
    catch (const std::exception& error)
    {
        std::cerr << "[ERROR] " << error.what() << std::endl;
        return -1;
    }

    std::cout << "[INFO] Exiting cleanly." << std::endl;
    return 0;
}

void dart_control(Capturer& capturer)
{   
    Buzzer buzzer;
    BMI055 bmi055;
    if (!bmi055.BMI055_init())
    {
        std::cerr << "[ERROR] BMI055 init failed!" << std::endl;
        g_running.store(false);
        return ;
    }
    std::cerr << "[INFO] BMI055 init successed!" << std::endl;

    CascadedPIDController attitude_controller;
    if (!attitude_controller.initialize_servos(
            config::SERVO_CENTER_LEFT_UPPER_US,
            config::SERVO_CENTER_RIGHT_UPPER_US,
            config::SERVO_CENTER_RIGHT_LOWER_US,
            config::SERVO_CENTER_LEFT_LOWER_US)) {
        std::cerr << "[ERROR] servo initialization failed!" << std::endl;
        g_running.store(false);
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
    struct PreRecordedCarData
    {
        std::chrono::steady_clock::time_point captured_at;
        uint64_t timestamp_ns;
        CarData data;
    };
    std::deque<PreRecordedCarData> car_data_pre_record;

    AttitudeAlgorithmer attitude_algorithmer;
    CarData dart_data;
    while (g_running)
    {
        bmi055.index++;
        // ======================================================
        // ====================== 数据采集 ====================
        // ======================================================

        if (g_flying_state.load() == 0)  // PRE_FLIGHT
        {
            // 着陆态：以加速度计为时钟源
            if (!bmi055.acc_wait_for_new_info())
            {
                std::cerr << "[ERROR] acc_wait_for_new_info failed!" << std::endl;
                g_running.store(false);
                return ;
            }
            if (!bmi055.acc_get_accd_all_mg())
            {
                std::cerr << "[ERROR] acc_get_accd_mg failed!" << std::endl;
                g_running.store(false);
                return ;
            }
            if (!bmi055.gyr_get_rate_all_dps())
            {
                std::cerr << "[ERROR] gyro get all rate dps failed!" << std::endl;
                g_running.store(false);
                return ;
            }
        }
        else // FLIGHT
        {
            // 飞行态：以陀螺仪为时钟源
            if (!bmi055.gyr_wait_for_new_info())
            {
                std::cerr << "[ERROR] gyr_wait_for_new_info failed!" << std::endl;
                g_running.store(false);
                return ;
            }
            if (!bmi055.gyr_get_rate_all_dps())
            {
                std::cerr << "[ERROR] gyr_get_rate_all_deg_per_s failed!" << std::endl;
                g_running.store(false);
                return ;
            }
            if (!bmi055.acc_get_accd_all_mg())
            {
                std::cerr << "[ERROR] acc_get_accd_mg failed!" << std::endl;
                g_running.store(false);
                return ;
            }
        }
        // ======================================================
		// ==================== imu -> frd 转系 ==================
        // ======================================================

        attitude_algorithmer.kalman_filter(
                bmi055.m_acc_imu_accd_mg.x(),
                bmi055.m_acc_imu_accd_mg.y(),
                bmi055.m_acc_imu_accd_mg.z(),
                bmi055.m_gyr_rate_dps.x(),
                bmi055.m_gyr_rate_dps.y(),
                bmi055.m_gyr_rate_dps.z());
        // 每帧同时计算滤波前、后的加速度计姿态用于记录对照。
        attitude_algorithmer.algorithmer();

        // ========================================================
        // =========================== 状态切换 ====================
        // ========================================================

        if (g_flying_state.load() == 0)  // PRE_FLIGHT
        {
            if (attitude_algorithmer.m_frd_acc.x() > config::LAUNCH_ACCEL_X_MG)
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
                    // 发射帧用固定的标称发射姿态初始化姿态积分（见 config）。
                    attitude_algorithmer.initialize(
                        config::LAUNCH_INITIAL_ROLL_DEG,
                        config::LAUNCH_INITIAL_PITCH_DEG);
                    flying_start_timer = now;
                    g_flight_duration_ms.store(0);  // 发射瞬间归零计时
                    g_flying_state.store(1);
                }
            }
            else
            {
                launch_cond_active = false;  // 条件中断，重新计时
            }
        }
        else
        {
            // 每个陀螺仪周期更新飞行姿态。
            attitude_algorithmer.update();
        }

        // ==================== 统一姿态 ====================
        int flying_state = g_flying_state.load();
        float roll  = (flying_state == 0) ? attitude_algorithmer.m_acc_roll  : attitude_algorithmer.m_gyro_roll;
        float pitch = (flying_state == 0) ? attitude_algorithmer.m_acc_pitch : attitude_algorithmer.m_gyro_pitch;
        float yaw   = (flying_state == 0) ? 0.0f : attitude_algorithmer.m_gyro_yaw;
        

        // 本帧的 pitch 目标（发射前无目标，保持 0），供控制与记录共用。
        float pitch_target_deg = 0.0f;

        //==================================================
        //====================== 控制 =======================
        //==================================================
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

            // Pitch 剖面：发射后 PITCH_PROFILE_DURATION_MS 内，目标俯仰由 START 线性变到 END。
            // 剖面斜率直接作为 y 轴角速度前馈，外环只需修正偏差。
            constexpr float kPitchProfileSlopeDps =
                (config::PITCH_PROFILE_END_DEG - config::PITCH_PROFILE_START_DEG)
                * 1000.0f / static_cast<float>(config::PITCH_PROFILE_DURATION_MS);
            const float pitch_profile_t =
                static_cast<float>(g_flight_duration_ms.load())
                / static_cast<float>(config::PITCH_PROFILE_DURATION_MS);
            pitch_target_deg = config::PITCH_PROFILE_START_DEG
                + (config::PITCH_PROFILE_END_DEG - config::PITCH_PROFILE_START_DEG)
                  * pitch_profile_t;

            // 发射后前 CONTROL_START_DELAY_MS 内不开控：弹体还在镖架上/刚出膛，
            // 陀螺被推力与振动污染（滤波角速度会出现 ±300dps 非物理尖峰），
            // 闭环只会打出错误且饱和的舵面指令。延迟到离架后再开控。
            if (g_flight_duration_ms.load() >= config::CONTROL_START_DELAY_MS)
            {
                attitude_controller.control(
                    target_x_rate,
                    kPitchProfileSlopeDps,
                    target_z_rate,
                    target_roll,
                    pitch_target_deg,
                    target_yaw,
                    attitude_algorithmer.m_frd_gyro.x(),
                    attitude_algorithmer.m_frd_gyro.y(),
                    attitude_algorithmer.m_frd_gyro.z(),
                    roll,
                    pitch,
                    yaw,
                    attitude_algorithmer.m_dt_s);
            }

        }

        /// ==========================================================
        /// =================== 控制数据记录 ==============================
        /// =========================================================
        CpuMonitor::sample();
        FrameCounter::tick();
        pi_health::sample();   // 运行中的主频/帧率（1 Hz，sysfs 文件读）
        dart_data.data_update(
            flying_state,
            attitude_algorithmer.raw_frd_acc_x(),
            attitude_algorithmer.raw_frd_acc_y(),
            attitude_algorithmer.raw_frd_acc_z(),
            attitude_algorithmer.m_frd_acc.x(),
            attitude_algorithmer.m_frd_acc.y(),
            attitude_algorithmer.m_frd_acc.z(),
            attitude_algorithmer.raw_frd_gyro_x(),
            attitude_algorithmer.raw_frd_gyro_y(),
            attitude_algorithmer.raw_frd_gyro_z(),
            attitude_algorithmer.m_frd_gyro.x(),
            attitude_algorithmer.m_frd_gyro.y(),
            attitude_algorithmer.m_frd_gyro.z(),
            roll, yaw, pitch,
            attitude_controller.pid_outer_outputs()[0],
            attitude_controller.pid_outer_outputs()[1],
            attitude_controller.pid_inner_outputs()[0],
            attitude_controller.pid_inner_outputs()[1],
            attitude_controller.servo_deltas_us()[0],
            attitude_controller.servo_deltas_us()[1],
            attitude_controller.servo_deltas_us()[2],
            attitude_controller.servo_deltas_us()[3],
            attitude_controller.pid_inner_outputs()[2],
            pitch_target_deg,
            attitude_controller.pid_outer_outputs()[2],
            pi_health::cpu_clock_mhz(),
            pi_health::cpu_temp_c());

        const auto sample_steady_time = std::chrono::steady_clock::now();
        const auto sample_system_time = std::chrono::system_clock::now();
        const uint64_t sample_timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                sample_system_time.time_since_epoch()).count());

        if (flying_state == 0)
        {
            // PRE_FLIGHT 阶段只保存在内存中，并持续淘汰 500 ms 以前的数据。
            car_data_pre_record.push_back(
                {sample_steady_time, sample_timestamp_ns, dart_data});
            const auto oldest_allowed = sample_steady_time
                - std::chrono::milliseconds(config::CAR_DATA_PRE_RECORD_MS);
            while (!car_data_pre_record.empty()
                   && car_data_pre_record.front().captured_at < oldest_allowed)
            {
                car_data_pre_record.pop_front();
            }
        }
        else if (flying_state == 1)
        {
            // 飞行期间只写当前帧，避免预录回写阻塞实时控制循环。
            capturer.update_car_data(dart_data, sample_timestamp_ns);
        }

        // 检测到撞击后切换为结束状态；
        if (flying_state == 1
            && attitude_algorithmer.m_frd_acc.x() < -config::LAUNCH_ACCEL_X_MG
            && g_flight_duration_ms.load()>1000)
        {
            std::array<int, 4> ending_pose{2200,2200,2200,2200};
            attitude_controller.servo_driver_.write_pulsewidths(ending_pose);
            flying_state = 2;
            g_flying_state.store(2);
            std::cerr << "[INFO] Flying Ending " << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            // 控制已经结束，此时再回写发射前 500 ms 数据，不影响飞行实时性。
            // 使用原始采样时间戳，读取端可按 logTime 离谱恢复时间顺序。
            for (const auto& sample : car_data_pre_record)
            {
                capturer.update_car_data(sample.data, sample.timestamp_ns);
            }
            car_data_pre_record.clear();
            g_running.store(false);
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
        g_running.store(false);
        return ;
    }
    int index = 0;
    while (g_running.load())
    {
        cv::Mat frame = ov5647.wait_and_get_latest_frame();
        if (frame.empty())
        {
            std::cout << "Frame empty!" << std::endl;
            g_running.store(false);
            break;
        }
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
        // ── 视觉检测数据写入 MCAP（vision 线程独立写入）──
        capturer.write_vision_data(vd.m_target_pixel_x, vd.m_target_pixel_y,
                                   vd.m_target_status, vd.m_frame_dt_ms, vd.m_video_fps);
    }

    ov5647.stop();
    std::cout << "[INFO] dart_vision thread exiting cleanly." << std::endl;
    return ;
}
