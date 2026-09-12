/**
 * @file pid.h
 * @brief PID 控制器数据结构与接口声明。
 */
#ifndef PID_H
#define PID_H

/* 位置式 PID 控制器运行状态。 */
typedef struct {
    float kp;             /* 比例系数。 */
    float ki;             /* 积分系数。 */
    float kd;             /* 微分系数。 */
    float integral;       /* 已累计的积分项。 */
    float previous_error; /* 上一次误差，用于计算微分。 */
    float output_limit;   /* 输出最大绝对值。 */
} PID_Controller;

/* 初始化 PID 参数并清空历史状态。 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_limit);
/* 清空积分和上一次误差，切换控制模式时应调用。 */
void PID_Reset(PID_Controller *pid);
/* 根据目标值和测量值计算一次 PID 输出。 */
float PID_Calculate(PID_Controller *pid, float target, float measurement);

#endif