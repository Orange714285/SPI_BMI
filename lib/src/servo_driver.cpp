#include "servo_driver.hpp"

#include <pigpio.h>

#include <algorithm>
#include <iostream>

PigpioServoDriver::PigpioServoDriver()
{
    // 初始化 pigpio 库。
    ready_ = gpioInitialise() >= 0;
    if (!ready_) {
        std::cerr << "[ERROR] pigpio initialization failed" << std::endl;
    }
}

PigpioServoDriver::~PigpioServoDriver()
{
    // 停止四路 PWM 并释放 pigpio。
    if (!ready_) return;
    for (const unsigned pin : GPIO_PINS) gpioServo(pin, 0);
    gpioTerminate();
}

bool PigpioServoDriver::write_pulsewidths(
    const std::array<int, 4>& pulsewidth_us)
{
    // 依次输出左上、右上、左下、右下舵机脉宽。
    if (!ready_) return false;
    bool success = true;
    for (std::size_t i = 0; i < GPIO_PINS.size(); ++i) {
        const int pulse = std::clamp(pulsewidth_us[i], 500, 2500);
        if (gpioServo(GPIO_PINS[i], pulse) != 0) success = false;
    }
    return success;
}
