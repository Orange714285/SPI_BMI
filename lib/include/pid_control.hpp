typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float i_limit;      // 积分限幅
    float output;
    float out_limit;    // 输出限幅
} PID_t;

class Pid_controler
{
    PID_t roll_angle_pid,  roll_rate_pid;
    PID_t pitch_angle_pid, pitch_rate_pid;
    PID_t yaw_angle_pid,   yaw_rate_pid;
    float PID_Calculate(PID_t *pid, float target, float feedback, float dt);
    void Control_Loop(float dt);
};
