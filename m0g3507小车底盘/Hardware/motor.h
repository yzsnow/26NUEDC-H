/**
 * @file motor.h
 * @brief 电机驱动模块对外接口。
 */
#ifndef MOTOR_H
#define MOTOR_H

/* 初始化电机模块，默认保持停车。 */
void Motor_Init(void);
/* 设置左右电机 PWM；正值前进，负值后退，0 为停止。 */
void Motor_Set(float left_pwm, float right_pwm);
/* 左右电机立即停止。 */
void Motor_Stop(void);

/**
 * @brief 获取最近一次实际下发的左电机PWM命令。
 * @return 限幅后的有符号PWM，正值表示前进。
 * @note 用于双主控惯性前馈遥测，不应作为新的电机闭环反馈量。
 */
float Motor_GetLeftCommandPwm(void);

/**
 * @brief 获取最近一次实际下发的右电机PWM命令。
 * @return 限幅后的有符号PWM，正值表示前进。
 * @note 用于双主控惯性前馈遥测，不应作为新的电机闭环反馈量。
 */
float Motor_GetRightCommandPwm(void);

#endif
