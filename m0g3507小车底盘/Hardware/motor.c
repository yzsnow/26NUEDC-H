/**
 * @file motor.c
 * @brief TB6612 左右电机方向控制与 PWM 输出。
 */
#include "motor.h"
#include "ti_msp_dl_config.h"
#include "vehicle_config.h"

static float last_left_command_pwm;
static float last_right_command_pwm;

/* 限制 PWM 范围，保护定时器比较值和电机驱动。 */
static float clamp_pwm(float pwm)
{
    if (pwm > MOTOR_PWM_MAXIMUM_DUTY_VALUE) return MOTOR_PWM_MAXIMUM_DUTY_VALUE;
    if (pwm < -MOTOR_PWM_MAXIMUM_DUTY_VALUE) return -MOTOR_PWM_MAXIMUM_DUTY_VALUE;
    return pwm;
}

/* 设置左电机方向与 PWM；负值表示反转。 */
static void set_left(float pwm)
{
    pwm = clamp_pwm(pwm);
    if (pwm >= 0.0f) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_LEFT_IN1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_LEFT_IN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_LEFT_IN1_PIN);
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_LEFT_IN2_PIN);
        pwm = -pwm;
    }
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, (uint32_t)pwm, DL_TIMER_CC_0_INDEX);
}

/* 设置右电机方向与 PWM；负值表示反转。 */
static void set_right(float pwm)
{
    pwm = clamp_pwm(pwm);
    if (pwm >= 0.0f) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_RIGHT_IN1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_RIGHT_IN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_RIGHT_IN1_PIN);
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_RIGHT_IN2_PIN);
        pwm = -pwm;
    }
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, (uint32_t)pwm, DL_TIMER_CC_1_INDEX);
}

/* 上电后先停车，防止初始化期间电机误动作。 */
void Motor_Init(void)
{
    Motor_Stop();
}

/* 独立设置左右电机速度，用于直行和差速转向。 */
void Motor_Set(float left_pwm, float right_pwm)
{
    left_pwm = clamp_pwm(left_pwm);
    right_pwm = clamp_pwm(right_pwm);
    last_left_command_pwm = left_pwm;
    last_right_command_pwm = right_pwm;
    set_left(left_pwm);
    set_right(right_pwm);
}

/* PWM 清零并将方向输入置为制动状态。 */
void Motor_Stop(void)
{
    last_left_command_pwm = 0.0f;
    last_right_command_pwm = 0.0f;
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 0, DL_TIMER_CC_1_INDEX);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN |
        MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN);
}

float Motor_GetLeftCommandPwm(void)
{
    return last_left_command_pwm;
}

float Motor_GetRightCommandPwm(void)
{
    return last_right_command_pwm;
}
