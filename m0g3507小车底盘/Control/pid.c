/**
 * @file pid.c
 * @brief 通用位置式 PID 控制器实现。
 */
#include "pid.h"

/* 将数值限制在 -limit～+limit，防止 PID 输出或积分无限增大。 */
static float clamp_value(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/* 保存 PID 参数，同时清空积分项和历史误差。 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_limit = output_limit;
}

/* 清空 PID 历史状态，避免上一次控制残留影响下一次启动。 */
void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

/* 执行一次位置式 PID：误差 = 目标值 - 测量值。 */
float PID_Calculate(PID_Controller *pid, float target, float measurement)
{
    float error = target - measurement;
    float derivative = error - pid->previous_error;
    pid->integral = clamp_value(pid->integral + error, pid->output_limit);
    pid->previous_error = error;
    return clamp_value(pid->kp * error + pid->ki * pid->integral + pid->kd * derivative,
        pid->output_limit);
}
