#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <thread>

class PigpioServoDriver {
public:
    // 顺序固定为：左上、右上、左下、右下。
    static constexpr std::array<unsigned, 4> GPIO_PINS{{12, 13, 18, 27}};

    PigpioServoDriver();
    ~PigpioServoDriver();
    PigpioServoDriver(const PigpioServoDriver&) = delete;
    PigpioServoDriver& operator=(const PigpioServoDriver&) = delete;

    bool is_ready() const { return ready_; }
    bool write_pulsewidths(const std::array<int, 4>& pulsewidth_us);

private:
    void pwm_loop();

    bool ready_ = false;
    std::atomic<bool> running_{false};
    std::atomic<bool> has_command_{false};
    std::array<int, 4> latest_pulsewidth_us_{{1500, 1500, 1500, 1500}};
    std::mutex pulsewidth_mutex_;
    std::thread pwm_thread_;
};
