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
#include <cstdio>
#include <deque>
#include <fstream>
#include <iomanip>
#include <thread>
#include <csignal>
#include <atomic>
#include <tools/cpu_monitor.hpp>
#include <tools/frame_counter.hpp>

namespace {

// 执行一条 shell 命令并返回其首行输出；失败返回空串。
// 注意：内部用 popen（fork+exec），只允许在没有其他线程运行时调用
// （本程序里仅在控制线程启动前、以及 join 之后调用），避免多线程 fork 死锁。
std::string run_shell_command(const std::string& command)
{
    std::string result;
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr)
    {
        return result;
    }
    char buffer[256] = {};
    if (std::fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        result = buffer;
        while (!result.empty()
               && (result.back() == '\n' || result.back() == '\r'))
        {
            result.pop_back();
        }
    }
    pclose(pipe);
    return result;
}

std::string run_vcgencmd(const char* args)
{
    return run_shell_command(std::string("vcgencmd ") + args + " 2>/dev/null");
}

// 5V 输入轨电压：Pi 4B / Pi 5 的 PMIC 自带 ADC，可直接读出 EXT5V_V，
// 无需任何外部硬件。不支持该命令的型号会返回空串，不影响其余字段。
std::string read_5v_rail()
{
    const std::string line = run_shell_command(
        "vcgencmd pmic_read_adc 2>/dev/null | grep -m1 EXT5V_V");
    const std::size_t pos = line.find('=');
    return pos == std::string::npos ? std::string() : line.substr(pos + 1);
}

// 运行中的健康采样：ARM 主频 / SoC 温度 / 循环帧率，1 Hz。
// 为什么必须在循环内采样：进程启动时（循环未跑）与线程 join 后（循环已停）CPU 都是空闲的，
// 那时读到的是空闲值，不代表工作点——所以由 sample_runtime_state_periodically() 在循环里按 1 Hz 采。
// 用 sysfs 文件读而不是 vcgencmd：后者要 fork/exec，不能在实时循环里用。
// 记「最低~最高」而不是末值：整场运行的跨度才能反映工作点是否变化过。
// 暖机期（HEALTH_WARMUP_MS）内的样本只更新「当前值」，不参与最低/最高统计。
int g_arm_freq_min_khz = 0;
int g_arm_freq_max_khz = 0;
double g_cpu_temp_min_c = 0.0;
double g_cpu_temp_max_c = 0.0;
double g_loop_fps_min = 0.0;
double g_loop_fps_max = 0.0;
// 本帧对应的树莓派状态，随每帧写入 mcap，供 Foxglove 直接画曲线。
double g_cpu_clock_mhz_now = 0.0;
double g_cpu_temp_c_now = 0.0;
// SoC 的硬件满频（kHz）。只在循环开始读一次，作为「主频是否偏低」的判定标准值。
// 不能靠「中途有没有变化」来判断：单次飞行只有 1.6 s 左右，慢速劣化根本来不及在飞行内显现。
int g_cpu_max_freq_khz = 0;

int read_arm_freq_khz()
{
    std::ifstream in("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
    int khz = 0;
    if (in)
    {
        in >> khz;
    }
    return khz;
}

// SoC 硬件满频（cpuinfo_max_freq，单位 kHz）。以它为标准值判断当前主频是否偏低。
int read_cpu_max_freq_khz()
{
    std::ifstream in("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    int khz = 0;
    if (in)
    {
        in >> khz;
    }
    return khz;
}

// /sys/class/thermal/thermal_zone0/temp 的单位是毫摄氏度。
double read_cpu_temp_c()
{
    std::ifstream in("/sys/class/thermal/thermal_zone0/temp");
    long millideg = 0;
    if (in)
    {
        in >> millideg;
    }
    return millideg > 0 ? static_cast<double>(millideg) / 1000.0 : 0.0;
}

void sample_runtime_state_periodically()
{
    static auto last_sample = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_sample < std::chrono::seconds(1))
    {
        return;
    }
    last_sample = now;

    // 这两项任何时刻都要更新：它们随每帧写进 mcap，画曲线用。
    g_cpu_clock_mhz_now = read_arm_freq_khz() / 1000.0;
    g_cpu_temp_c_now = read_cpu_temp_c();

    // 满频是恒定值，整个进程只读一次，作为主频判定的标准值。
    if (g_cpu_max_freq_khz <= 0)
    {
        g_cpu_max_freq_khz = read_cpu_max_freq_khz();
    }

    // 暖机期内的数字不可信（设备初始化 + CPU 策略爬坡 + FrameCounter 首个窗口不完整），
    // 只更新上面的「当前值」，不计入下面用于判定的统计量。
    // 起点取本函数第一次被调用的时刻（≈ 循环开始），而不是进程启动时刻——后者定义在文件后面，
    // 且会把设备初始化那一段也划进暖机期之外。
    static const auto warmup_origin = now;
    if (now - warmup_origin < std::chrono::milliseconds(config::HEALTH_WARMUP_MS))
    {
        return;
    }

    const double fps = FrameCounter::fps();
    if (fps > 0.0)
    {
        if (g_loop_fps_min == 0.0 || fps < g_loop_fps_min)
        {
            g_loop_fps_min = fps;
        }
        if (fps > g_loop_fps_max)
        {
            g_loop_fps_max = fps;
        }
    }

    const double temp_c = g_cpu_temp_c_now;
    if (temp_c > 0.0)
    {
        if (g_cpu_temp_min_c == 0.0 || temp_c < g_cpu_temp_min_c)
        {
            g_cpu_temp_min_c = temp_c;
        }
        if (temp_c > g_cpu_temp_max_c)
        {
            g_cpu_temp_max_c = temp_c;
        }
    }

    const int khz = read_arm_freq_khz();
    if (khz <= 0)
    {
        return;
    }
    if (g_arm_freq_min_khz == 0 || khz < g_arm_freq_min_khz)
    {
        g_arm_freq_min_khz = khz;
    }
    if (khz > g_arm_freq_max_khz)
    {
        g_arm_freq_max_khz = khz;
    }
}

// 把树莓派运行状态追加到 config 快照末尾。
// 本程序的控制循环节拍由 CPU 吞吐决定（DRDY 积压会被排空丢弃），
// 因此 5V 轨电压 / 节流标志 / ARM 主频 / SoC 温度 / 循环帧率是「帧率为何变慢」的直接证据。
// 循环帧率取自 FrameCounter 最近 1 s 窗口，即使不发射也能读到（发射前同样在跑循环）。
// tag=="end" 时额外写一行 [PI_HEALTH]，把「这一发数据能不能用」直接判出来。
void append_pi_state(const std::string& snapshot_path, const char* tag)
{
    std::ofstream out(snapshot_path, std::ios::app);
    if (!out)
    {
        return;
    }

    const std::string throttled = run_vcgencmd("get_throttled");
    out << "[PI_STATE " << tag << "] 5V=" << read_5v_rail()
        << " | " << throttled
        << " | " << run_vcgencmd("measure_clock arm")
        << " | " << run_vcgencmd("measure_temp")
        << " | loop_fps=" << static_cast<long long>(g_loop_fps_min)
        << "~" << static_cast<long long>(g_loop_fps_max)
        << " | loop_arm_freq_khz=" << g_arm_freq_min_khz << "~" << g_arm_freq_max_khz << "\n";

    // 统计量要等运行结束才完整，所以判定行只在 end 时写。
    if (std::string(tag) != "end")
    {
        return;
    }

    // 帧率离散度：(max-min)/max。暖机期已排除，所以这里的区间只反映真实工作点变化。
    const double fps_spread_pct = g_loop_fps_max > 0.0
        ? 100.0 * (g_loop_fps_max - g_loop_fps_min) / g_loop_fps_max
        : 0.0;
    // 与硬件满频这个标准值比较，而不是看「有没有中途变化」——
    // 单次飞行只有 1.6 s 左右，全程稳在低位同样不可用。
    const bool clock_dipped = g_cpu_max_freq_khz > 0
        ? (g_arm_freq_min_khz > 0 && g_arm_freq_min_khz < g_cpu_max_freq_khz)
        : (g_arm_freq_min_khz > 0 && g_arm_freq_min_khz < g_arm_freq_max_khz);
    const bool temp_high = g_cpu_temp_max_c >= config::HEALTH_TEMP_WARN_C;
    const bool fps_wide = fps_spread_pct >= config::HEALTH_FPS_SPREAD_WARN_PCT;

    // 从 "throttled=0x..." 解出位掩码。
    unsigned long throttled_flags = 0;
    const std::size_t eq = throttled.find('=');
    if (eq != std::string::npos)
    {
        std::sscanf(throttled.c_str() + eq + 1, "%lx", &throttled_flags);
    }
    // bit1/2/3 = 当前主频被压 / 正在降频 / 软温度限；bit16/17/18/19 = 曾发生过。
    constexpr unsigned long kThrottleMask = 0x000E0000UL | 0x0000000EUL;
    const bool throttled_hit = (throttled_flags & kThrottleMask) != 0UL;

    std::string verdict = "健康";
    std::string reasons;
    if (throttled_hit)
    {
        verdict = "不可用";
        reasons += " 触发节流";
    }
    else
    {
        if (clock_dipped)
        {
            reasons += " 主频中途低于满频";
        }
        if (temp_high)
        {
            reasons += " 温度超过阈值";
        }
        if (fps_wide)
        {
            reasons += " 帧率离散过大";
        }
        if (!reasons.empty())
        {
            verdict = "可疑";
        }
    }

    out << "[PI_HEALTH] 判定=" << verdict
        << " | 主频=" << g_arm_freq_min_khz / 1000 << "~" << g_arm_freq_max_khz / 1000 << "MHz"
        << "(满频" << g_cpu_max_freq_khz / 1000 << "MHz)"
        << " | 温度=" << g_cpu_temp_min_c << "~" << g_cpu_temp_max_c << "C"
        << " | 帧率=" << static_cast<long long>(g_loop_fps_min) << "~"
        << static_cast<long long>(g_loop_fps_max) << "Hz(离散"
        << std::fixed << std::setprecision(1) << fps_spread_pct << "%)"
        << " | " << throttled
        << " |" << (reasons.empty() ? std::string(" 无异常") : reasons) << "\n";
}

}   // namespace

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
        // Capturer 在构造函数中完成初始化。
        Capturer capturer;
        // 进程启动时的树莓派运行状态（此刻尚无其他线程，popen 安全）。
        append_pi_state(capturer.config_snapshot_path(), "start");

        std::thread dart_control_thread(dart_control, std::ref(capturer));
        // std::thread dart_vision_thread(dart_vision, std::ref(capturer));
        dart_control_thread.join();
        // dart_vision_thread.join();
        capturer.finish();
        // 飞行结束后的运行状态（线程已 join），先落盘再上传。
        append_pi_state(capturer.config_snapshot_path(), "end");
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
        sample_runtime_state_periodically();   // 运行中的主频/帧率（1 Hz，sysfs 文件读）
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
            g_cpu_clock_mhz_now,
            g_cpu_temp_c_now);

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
