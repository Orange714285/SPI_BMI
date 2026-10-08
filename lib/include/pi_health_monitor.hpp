#pragma once

#include <string>

// 树莓派运行状态监控：ARM 主频 / SoC 温度 / 节流标志 / 循环帧率。
//
// 控制循环的节拍由 CPU 吞吐决定（DRDY 积压会被排空丢弃），所以「这一发数据能不能用」
// 取决于运行期间主频与帧率是否稳定。本模块负责在控制循环内采样，并把结论写进 config 快照。
//
// 用法：
//   pi_health::sample();                          // 控制循环内每帧调用（内部 1 Hz 限流）
//   pi_health::cpu_clock_mhz();                   // 随每帧写进 mcap，供 Foxglove 画曲线
//   pi_health::cpu_temp_c();
//   pi_health::append_state_to(path, "start");    // 进程启动时
//   pi_health::append_state_to(path, "end");      // 退出时（同时写一行 [PI_HEALTH] 判定）
namespace pi_health
{

// 采样一次 ARM 主频 / SoC 温度 / 循环帧率。
//
// 内部按 1 Hz 限流，并用 sysfs 文件读而不是 vcgencmd（后者要 fork/exec，不能在实时循环里用）。
// 启动暖机期（config::HEALTH_WARMUP_MS）内的样本只更新「当前值」，不计入最低/最高统计——
// 那几秒设备正在初始化、CPU 策略正在爬坡、FrameCounter 首个窗口不完整，会伪装成降频/掉帧。
void sample();

// 把 [PI_STATE <tag>] 追加到 config 快照末尾。
//
// tag == "end" 时额外追加一行 [PI_HEALTH]，把「这一发数据能不能用」直接判出来
// （健康 / 可疑 / 不可用），供事后筛选可信的飞行数据。
void append_state_to(const std::string& snapshot_path, const char* tag);

// 本帧对应的树莓派状态，由 sample() 每秒刷新一次。
double cpu_clock_mhz();
double cpu_temp_c();

}   // namespace pi_health
