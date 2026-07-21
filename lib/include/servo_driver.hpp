#pragma once

#include <array>

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
    bool ready_ = false;
};
