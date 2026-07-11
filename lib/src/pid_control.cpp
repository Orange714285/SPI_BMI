#include "pid_control.hpp"
float Pid_controler::PID_Calculate(PID_t *pid, float target, float feedback, float dt) {
    pid->error = target - feedback;
    
    // 积分项
    pid->integral += pid->error * dt;
    // 积分限幅（防止积分饱和）
    if (pid->integral > pid->i_limit) pid->integral = pid->i_limit;
    else if (pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;
    
    // 微分项
    float derivative = (pid->error - pid->last_error) / dt;
    pid->last_error = pid->error;
    
    // 计算输出
    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    
    // 输出限幅
    if (pid->output > pid->out_limit) pid->output = pid->out_limit;
    else if (pid->output < -pid->out_limit) pid->output = -pid->out_limit;
    
    return pid->output;
}

