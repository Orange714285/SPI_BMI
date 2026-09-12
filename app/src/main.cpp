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
#include <cstdint>
#include <deque>
#include <thread>
#include <csignal>
#include <atomic>
#include <tools/cpu_monitor.hpp>
#include <tools/frame_counter.hpp>

namespace
{
SharedState* g_signal_state = nullptr;
}

void signal_handler(int)
{
    if (g_signal_state != nullptr)
        g_signal_state->running.store(false);
}
void dart_control(Capturer& capturer, SharedState* state);
void dart_vision(Capturer& capturer, SharedState* state);
int main() 
{
    SharedState shared_state;
    g_signal_state = &shared_state;

    Buzzer buzzer;
    buzzer.start(1);

    try
    {
        // 构造时完成初始化；电控和视频写入同一个 .mcap 文件。
        Capturer capturer;
        std::thread dart_control_thread(dart_control, std::ref(capturer), &shared_state);
        // std::thread dart_vision_thread(dart_vision, std::ref(capturer), &shared_state);
        dart_control_thread.join();
        // dart_vision_thread.join();
        capturer.finish();
        upload_mcap(capturer.mcap_path());
    }
    catch (const std::exception& error)
    {
        std::cerr << "[ERROR] " << error.what() << std::endl;
        return -1;
    }

    std::cout << "[INFO] Exiting cleanly." << std::endl;
    return 0;
}

void dart_control(Capturer& capturer, SharedState* state)
{   
    Buzzer buzzer;
    BMI055 bmi055;
    if (!bmi055.BMI055_init())
    {
        std::cerr << "[ERROR] BMI055 init failed!" << std::endl;
        state->running.store(false);
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
        state->running.store(false);
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "[INFO] servo initialization sucessed!" << std::endl;
    signal(SIGINT, signal_handler);

    state->flying_state.store(FlyingState::PRE_FLIGHT);
    CpuMonitor::sample();   // 建立基准
    auto flying_start_timer = std::chrono::steady_clock::now();

    // 发射前姿态锁存：周期性保存最近一次稳定的 roll/pitch，供发射时初始化陀螺仪积分。
    float launch_roll = 0.0f;
    float launch_pitch = 0.0f;
    auto last_attitude_latch_timer = std::chrono::steady_clock::now();

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

    AccAttitudeAlgorithmer acc_attitude_algorithmer;
    GyroAttitudeAlgorithmer gyro_attitude_algorithmer;
    CarData dart_data;
    while (state->running.load())
    {
        bmi055.index++;
        // ======================================================
        // ====================== 数据采集 ====================
        // ======================================================

        if (state->flying_state.load() == FlyingState::PRE_FLIGHT)
        {
            // 着陆态：以加速度计为时钟源
            if (!bmi055.acc_wait_for_new_info())
            {
                std::cerr << "[ERROR] acc_wait_for_new_info failed!" << std::endl;
                state->running.store(false);
                return ;
            }
            if (!bmi055.acc_get_accd_all_mg())
            {
                std::cerr << "[ERROR] acc_get_accd_mg failed!" << std::endl;
                state->running.store(false);
                return ;
            }
            if (!bmi055.gyr_get_rate_all_dps())
            {
                std::cerr << "[ERROR] gyro get all rate dps failed!" << std::endl;
                state->running.store(false);
                return ;
            }
        }
        else // FLIGHT
        {
            // 飞行态：以陀螺仪为时钟源
            if (!bmi055.gyr_wait_for_new_info())
            {
                std::cerr << "[ERROR] gyr_wait_for_new_info failed!" << std::endl;
                state->running.store(false);
                return ;
            }
            if (!bmi055.gyr_get_rate_all_dps())
            {
                std::cerr << "[ERROR] gyr_get_rate_all_deg_per_s failed!" << std::endl;
                state->running.store(false);
                return ;
            }
            if (!bmi055.acc_get_accd_all_mg())
            {
                std::cerr << "[ERROR] acc_get_accd_mg failed!" << std::endl;
                state->running.store(false);
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
        // =========================== 状态切换 ====================
        // ========================================================

        if (state->flying_state.load() == FlyingState::PRE_FLIGHT)
        {
            if (acc_attitude_algorithmer.m_frd_acc_x > config::LAUNCH_ACCEL_X_MG)
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
                    state->flight_duration_ms.store(0);  // 发射瞬间归零计时
                    state->flying_state.store(FlyingState::FLIGHT);
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
        FlyingState flying_state = state->flying_state.load();
        float roll  = (flying_state == FlyingState::PRE_FLIGHT) ? acc_attitude_algorithmer.m_roll  : gyro_attitude_algorithmer.m_roll;
        float pitch = (flying_state == FlyingState::PRE_FLIGHT) ? acc_attitude_algorithmer.m_pitch : gyro_attitude_algorithmer.m_pitch;
        float yaw   = (flying_state == FlyingState::PRE_FLIGHT) ? 0.0f : gyro_attitude_algorithmer.m_yaw;
        float unfiltered_roll = (flying_state == FlyingState::PRE_FLIGHT)
            ? acc_attitude_algorithmer.m_raw_roll
            : gyro_attitude_algorithmer.m_unfiltered_roll;
        float unfiltered_pitch = (flying_state == FlyingState::PRE_FLIGHT)
            ? acc_attitude_algorithmer.m_raw_pitch
            : gyro_attitude_algorithmer.m_unfiltered_pitch;
        float unfiltered_yaw = (flying_state == FlyingState::PRE_FLIGHT)
            ? 0.0f : gyro_attitude_algorithmer.m_unfiltered_yaw;
        

        //==================================================
        //====================== 控制 =======================
        //==================================================
        if (flying_state == FlyingState::FLIGHT)
        {
            // 输入目标和实测姿态，执行一次双环 PID 控制。
            auto flying_process_timer = std::chrono::steady_clock::now();
            auto flying_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(flying_process_timer - flying_start_timer);
            state->flight_duration_ms.store(flying_duration_ms.count());
            const bool stage_one =
                state->flight_duration_ms.load() < config::STAGE_ONE_END_MS;
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

        /// ==========================================================
        /// =================== 数据记录 ==============================
        /// =========================================================
        CpuMonitor::sample();
        FrameCounter::tick();
        dart_data.data_update(
            static_cast<int>(flying_state),
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
            gyro_attitude_algorithmer.m_wrong_pitch,
            attitude_controller.pid_outer_outputs()[0],
            attitude_controller.pid_outer_outputs()[1],
            attitude_controller.pid_inner_outputs()[0],
            attitude_controller.pid_inner_outputs()[1],
            attitude_controller.servo_deltas_us()[0],
            attitude_controller.servo_deltas_us()[1],
            attitude_controller.servo_deltas_us()[2],
            attitude_controller.servo_deltas_us()[3]);
        dart_data.print_attitude();

        const auto sample_steady_time = std::chrono::steady_clock::now();
        const auto sample_system_time = std::chrono::system_clock::now();
        const uint64_t sample_timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                sample_system_time.time_since_epoch()).count());

        if (flying_state == FlyingState::PRE_FLIGHT)
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
        else if (flying_state == FlyingState::FLIGHT)
        {
            // 飞行期间只写当前帧，避免预录回写阻塞实时控制循环。
            capturer.update_car_data(dart_data, sample_timestamp_ns);
        }

        // 检测到撞击后切换为结束状态；
        if (flying_state == FlyingState::FLIGHT
            && acc_attitude_algorithmer.m_frd_acc_x < -config::LAUNCH_ACCEL_X_MG
            && state->flight_duration_ms.load() > 1000)
        {
            std::array<int, 4> ending_pose{2200,2200,2200,2200};
            attitude_controller.servo_driver_.write_pulsewidths(ending_pose);
            flying_state = FlyingState::FLIGHT_END;
            state->flying_state.store(FlyingState::FLIGHT_END);
            std::cerr << "[INFO] Flying Ending " << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            // 控制已经结束，此时再回写发射前 500 ms 数据，不影响飞行实时性。
            // 使用原始采样时间戳，读取端可按 logTime 离谱恢复时间顺序。
            for (const auto& sample : car_data_pre_record)
            {
                capturer.update_car_data(sample.data, sample.timestamp_ns);
            }
            car_data_pre_record.clear();
            state->running.store(false);
        }


    }
    std::cout << "[INFO] dart_control thread received SIGINT, exiting cleanly." << std::endl;
    return ;
}
void dart_vision(Capturer& capturer, SharedState* state)
{
    signal(SIGINT, signal_handler);
    Camera ov5647;
    Detector detector;
    if (!ov5647.start())
    {
        ov5647.stop();
        std::cerr << "[ERROR] Camera init failed!" << std::endl;
        state->running.store(false);
        return ;
    }
    int index = 0;
    while (state->running.load())
    {
        cv::Mat frame = ov5647.wait_and_get_latest_frame();
        if (frame.empty())
        {
            std::cout << "Frame empty!" << std::endl;
            state->running.store(false);
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
        state->vision_data.store(vd);

        if (!state->running.load())
            break;
        // ── 视觉检测数据写入 MCAP（vision 线程独立写入）──
        capturer.write_vision_data(vd.m_target_pixel_x, vd.m_target_pixel_y,
                                   vd.m_target_status, vd.m_frame_dt_ms, vd.m_video_fps);
    }

    ov5647.stop();
    std::cout << "[INFO] dart_vision thread exiting cleanly." << std::endl;
    return ;
}
