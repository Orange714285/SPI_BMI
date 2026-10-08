#include "pi_health_monitor.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <string>

#include "config.hpp"
#include "tools/frame_counter.hpp"

namespace
{

// 执行一条 shell 命令并返回其首行输出；失败返回空串。
// 注意：内部用 popen（fork+exec），只允许在没有其他线程运行时调用
// （本文件里仅在控制线程启动前、以及 join 之后调用），避免多线程 fork 死锁。
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
// 那时读到的是空闲值，不代表工作点——所以由 sample() 在循环里按 1 Hz 采。
// 用 sysfs 文件读而不是 vcgencmd：后者要 fork/exec，不能在实时循环里用。
// 记「最低~最高」而不是末值：整场运行的跨度才能反映工作点是否变化过。
// 暖机期（HEALTH_WARMUP_MS）内的样本只更新「当前值」，不参与最低/最高统计。
int arm_freq_min_khz = 0;
int arm_freq_max_khz = 0;
double cpu_temp_min_c = 0.0;
double cpu_temp_max_c = 0.0;
double loop_fps_min = 0.0;
double loop_fps_max = 0.0;
// 本帧对应的树莓派状态，随每帧写入 mcap，供 Foxglove 直接画曲线。
double cpu_clock_mhz_now = 0.0;
double cpu_temp_c_now = 0.0;
// SoC 的硬件满频（kHz）。只在循环开始读一次，作为「主频是否偏低」的判定标准值。
// 不能靠「中途有没有变化」来判断：单次飞行只有 1.6 s 左右，慢速劣化根本来不及在飞行内显现。
int cpu_max_freq_khz = 0;

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

}   // namespace

namespace pi_health
{

void sample()
{
    static auto last_sample = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_sample < std::chrono::seconds(1))
    {
        return;
    }
    last_sample = now;

    // 这两项任何时刻都要更新：它们随每帧写进 mcap，画曲线用。
    cpu_clock_mhz_now = read_arm_freq_khz() / 1000.0;
    cpu_temp_c_now = read_cpu_temp_c();

    // 满频是恒定值，整个进程只读一次，作为主频判定的标准值。
    if (cpu_max_freq_khz <= 0)
    {
        cpu_max_freq_khz = read_cpu_max_freq_khz();
    }

    // 暖机期内的数字不可信（设备初始化 + CPU 策略爬坡 + FrameCounter 首个窗口不完整），
    // 只更新上面的「当前值」，不计入下面用于判定的统计量。
    // 起点取本函数第一次被调用的时刻（≈ 循环开始），而不是进程启动时刻——
    // 否则会把设备初始化那一段也划进暖机期之外。
    static const auto warmup_origin = now;
    if (now - warmup_origin < std::chrono::milliseconds(config::HEALTH_WARMUP_MS))
    {
        return;
    }

    const double fps = FrameCounter::fps();
    if (fps > 0.0)
    {
        if (loop_fps_min == 0.0 || fps < loop_fps_min)
        {
            loop_fps_min = fps;
        }
        if (fps > loop_fps_max)
        {
            loop_fps_max = fps;
        }
    }

    const double temp_c = cpu_temp_c_now;
    if (temp_c > 0.0)
    {
        if (cpu_temp_min_c == 0.0 || temp_c < cpu_temp_min_c)
        {
            cpu_temp_min_c = temp_c;
        }
        if (temp_c > cpu_temp_max_c)
        {
            cpu_temp_max_c = temp_c;
        }
    }

    const int khz = read_arm_freq_khz();
    if (khz <= 0)
    {
        return;
    }
    if (arm_freq_min_khz == 0 || khz < arm_freq_min_khz)
    {
        arm_freq_min_khz = khz;
    }
    if (khz > arm_freq_max_khz)
    {
        arm_freq_max_khz = khz;
    }
}

// 把树莓派运行状态追加到 config 快照末尾。
// 本程序的控制循环节拍由 CPU 吞吐决定（DRDY 积压会被排空丢弃），
// 因此 5V 轨电压 / 节流标志 / ARM 主频 / SoC 温度 / 循环帧率是「帧率为何变慢」的直接证据。
// 循环帧率取自 FrameCounter 最近 1 s 窗口，即使不发射也能读到（发射前同样在跑循环）。
// tag=="end" 时额外写一行 [PI_HEALTH]，把「这一发数据能不能用」直接判出来。
void append_state_to(const std::string& snapshot_path, const char* tag)
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
        << " | loop_fps=" << static_cast<long long>(loop_fps_min)
        << "~" << static_cast<long long>(loop_fps_max)
        << " | loop_arm_freq_khz=" << arm_freq_min_khz << "~" << arm_freq_max_khz << "\n";

    // 统计量要等运行结束才完整，所以判定行只在 end 时写。
    if (std::string(tag) != "end")
    {
        return;
    }

    // 帧率离散度：(max-min)/max。暖机期已排除，所以这里的区间只反映真实工作点变化。
    const double fps_spread_pct = loop_fps_max > 0.0
        ? 100.0 * (loop_fps_max - loop_fps_min) / loop_fps_max
        : 0.0;
    // 与硬件满频这个标准值比较，而不是看「有没有中途变化」——
    // 单次飞行只有 1.6 s 左右，全程稳在低位同样不可用。
    const bool clock_dipped = cpu_max_freq_khz > 0
        ? (arm_freq_min_khz > 0 && arm_freq_min_khz < cpu_max_freq_khz)
        : (arm_freq_min_khz > 0 && arm_freq_min_khz < arm_freq_max_khz);
    const bool temp_high = cpu_temp_max_c >= config::HEALTH_TEMP_WARN_C;
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
        << " | 主频=" << arm_freq_min_khz / 1000 << "~" << arm_freq_max_khz / 1000 << "MHz"
        << "(满频" << cpu_max_freq_khz / 1000 << "MHz)"
        << " | 温度=" << cpu_temp_min_c << "~" << cpu_temp_max_c << "C"
        << " | 帧率=" << static_cast<long long>(loop_fps_min) << "~"
        << static_cast<long long>(loop_fps_max) << "Hz(离散"
        << std::fixed << std::setprecision(1) << fps_spread_pct << "%)"
        << " | " << throttled
        << " |" << (reasons.empty() ? std::string(" 无异常") : reasons) << "\n";
}

double cpu_clock_mhz()
{
    return cpu_clock_mhz_now;
}

double cpu_temp_c()
{
    return cpu_temp_c_now;
}

}   // namespace pi_health
