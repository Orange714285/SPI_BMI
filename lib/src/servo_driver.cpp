#include "servo_driver.hpp"

#include "config.hpp"

#include <pigpio.h>

#include <algorithm>
#include <chrono>
#include <iostream>

PigpioServoDriver::PigpioServoDriver()
{
    // 初始化 pigpio，并将四路软件 PWM 配置为 320 Hz。
    ready_ = gpioInitialise() >= 0;
    if (!ready_) {
        std::cerr << "[ERROR] pigpio initialization failed" << std::endl;
        return;
    }

    for (const unsigned pin : GPIO_PINS) {
        const int frequency = gpioSetPWMfrequency(
            pin, config::SERVO_PWM_FREQUENCY_HZ);
        const int range = gpioSetPWMrange(pin, config::SERVO_PWM_PERIOD_US);
        if (frequency != config::SERVO_PWM_FREQUENCY_HZ || range < 0) {
            std::cerr << "[ERROR] failed to configure 320 Hz PWM on GPIO "
                      << pin << std::endl;
            ready_ = false;
            gpioTerminate();
            return;
        }
    }

    running_.store(true);
    pwm_thread_ = std::thread(&PigpioServoDriver::pwm_loop, this);
}

PigpioServoDriver::~PigpioServoDriver()
{
    // 停止异步 PWM 线程、关闭四路输出并释放 pigpio。
    if (!ready_) return;
    running_.store(false);
    if (pwm_thread_.joinable()) pwm_thread_.join();
    for (const unsigned pin : GPIO_PINS) gpioPWM(pin, 0);
    gpioTerminate();
}

bool PigpioServoDriver::write_pulsewidths(
    const std::array<int, 4>& pulsewidth_us)
{
    // 主控制线程只更新最新目标脉宽，不直接操作 GPIO。
    if (!ready_) return false;
    std::lock_guard<std::mutex> lock(pulsewidth_mutex_);
    for (std::size_t i = 0; i < GPIO_PINS.size(); ++i) {
        latest_pulsewidth_us_[i] = std::clamp(
            pulsewidth_us[i],
            config::SERVO_MIN_PULSE_US,
            config::SERVO_MAX_PULSE_US);
    }
    has_command_.store(true);
    return true;
}

void PigpioServoDriver::pwm_loop()
{
    // 独立线程固定以 320 Hz 读取最新脉宽并更新四路 PWM。
    const auto period = std::chrono::microseconds(
        config::SERVO_PWM_PERIOD_US);
    auto next_update = std::chrono::steady_clock::now();

    while (running_.load()) {
        if (has_command_.load()) {
            std::array<int, 4> pulsewidth_us{};
            {
                std::lock_guard<std::mutex> lock(pulsewidth_mutex_);
                pulsewidth_us = latest_pulsewidth_us_;
            }

            for (std::size_t i = 0; i < GPIO_PINS.size(); ++i) {
                gpioPWM(GPIO_PINS[i], pulsewidth_us[i]);
            }
        }

        next_update += period;
        std::this_thread::sleep_until(next_update);
    }
}
