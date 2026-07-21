#pragma once

#include <gpiod.h>
#include <chrono>
#include <thread>
#include <cstdint>

/**
 * @brief 蜂鸣器控制类
 *
 * 基于 libgpiod + 软件 PWM，可接任意 GPIO。
 *
 * 用法：
 *   Buzzer buzzer(4, 1000);     // GPIO4, 1kHz
 *   buzzer.beep(50, 500);       // 50% 占空比响 500ms（阻塞）
 */
class Buzzer
{
public:
    Buzzer(int pin = 4, int freq = 1000)
        : m_pin(pin), m_freq(freq)
    {
        if (s_refcount == 0)
        {
            s_chip = gpiod_chip_open("/dev/gpiochip0");

            gpiod_line_settings *settings = gpiod_line_settings_new();
            gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
            gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

            gpiod_line_config *config = gpiod_line_config_new();
            const unsigned int offsets[] = {static_cast<unsigned int>(m_pin)};
            gpiod_line_config_add_line_settings(config, offsets, 1, settings);

            s_request = gpiod_chip_request_lines(s_chip, nullptr, config);

            gpiod_line_settings_free(settings);
            gpiod_line_config_free(config);
        }
        s_refcount++;
    }

    ~Buzzer()
    {
        s_refcount--;
        if (s_refcount == 0)
        {
            gpiod_line_request_release(s_request);
            s_request = nullptr;
            gpiod_chip_close(s_chip);
            s_chip = nullptr;
        }
    }

    // 不允许拷贝，避免引用计数混乱
    Buzzer(const Buzzer&)            = delete;
    Buzzer& operator=(const Buzzer&) = delete;

    /** @brief 以指定 PWM 占空比鸣叫指定时间（阻塞）
     *  @param duty_cycle  占空比 0~100
     *  @param duration_ms 持续时间（毫秒）
     */
    void beep(int duty_cycle, int duration_ms)
    {
        auto end = std::chrono::steady_clock::now()
                 + std::chrono::milliseconds(duration_ms);

        int period_us = 1000000 / m_freq;
        int on_us  = period_us * duty_cycle / 100;
        int off_us = period_us - on_us;

        while (std::chrono::steady_clock::now() < end)
        {
            gpiod_line_request_set_value(s_request, m_pin, GPIOD_LINE_VALUE_ACTIVE);
            spin_wait(on_us);
            gpiod_line_request_set_value(s_request, m_pin, GPIOD_LINE_VALUE_INACTIVE);
            spin_wait(off_us);
        }
    }

    /** @brief 音调从低到高响三声，每声 0.2s（阻塞）
     *  @param duty_cycle  占空比 0~100，默认 50
     */
    void start(int duty_cycle = 50)
    {
        const int tones[] = {1000, 2000, 3000};

        for (int i = 0; i < 3; ++i)
        {
            int freq_backup = m_freq;
            m_freq = tones[i];
            beep(duty_cycle, 200);
            m_freq = freq_backup;

            if (i < 2)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

private:
    static void spin_wait(int us)
    {
        auto end = std::chrono::steady_clock::now()
                 + std::chrono::microseconds(us);
        while (std::chrono::steady_clock::now() < end)
            ;
    }

    int m_pin;
    int m_freq;

    inline static int               s_refcount = 0;
    inline static gpiod_chip        *s_chip     = nullptr;
    inline static gpiod_line_request *s_request = nullptr;
};
