/**
 * @file encoder.c
 * @brief 左右电机正交编码器计数与行驶距离计算。
 */
#include "encoder.h"
#include "ti_msp_dl_config.h"
#include "vehicle_config.h"

static volatile int32_t left_encoder_count;
static volatile int32_t right_encoder_count;

/* 清零编码器计数，并开启 GPIOA/GPIOB 中断。 */
void Encoder_Init(void)
{
    Encoder_Reset();
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/* 关中断后清零计数，避免中断同时修改造成数据不一致。 */
void Encoder_Reset(void)
{
    __disable_irq();
    left_encoder_count = 0;
    right_encoder_count = 0;
    __enable_irq();
}

/* 返回左编码器累计计数。 */
int32_t Encoder_GetLeftCount(void) { return left_encoder_count; }
/* 返回右编码器累计计数。 */
int32_t Encoder_GetRightCount(void) { return right_encoder_count; }

/* 使用左右轮计数绝对值平均数换算行驶距离。 */
float Encoder_GetAverageDistanceMm(void)
{
    float left_count_magnitude = (float)(left_encoder_count < 0 ? -left_encoder_count : left_encoder_count);
    float right_count_magnitude = (float)(right_encoder_count < 0 ? -right_encoder_count : right_encoder_count);
    return ((left_count_magnitude + right_count_magnitude) * 0.5f) *
        (DRIVE_WHEEL_CIRCUMFERENCE_MILLIMETERS / MOTOR_ENCODER_COUNTS_PER_WHEEL_REVOLUTION);
}

/* GPIO 组中断：根据正交编码器另一相电平判断计数方向。 */
void GROUP1_IRQHandler(void)
{
    uint32_t left_interrupt_status = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_LEFT_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN);
    uint32_t right_interrupt_status = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_RIGHT_PORT, ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);

    if ((left_interrupt_status & ENCODER_LEFT_A_PIN) != 0U) {
        left_encoder_count += DL_GPIO_readPins(ENCODER_LEFT_PORT, ENCODER_LEFT_B_PIN) ? 1 : -1;
    }
    if ((left_interrupt_status & ENCODER_LEFT_B_PIN) != 0U) {
        left_encoder_count += DL_GPIO_readPins(ENCODER_LEFT_PORT, ENCODER_LEFT_A_PIN) ? -1 : 1;
    }
    if ((right_interrupt_status & ENCODER_RIGHT_A_PIN) != 0U) {
        right_encoder_count += DL_GPIO_readPins(ENCODER_RIGHT_PORT, ENCODER_RIGHT_B_PIN) ? -1 : 1;
    }
    if ((right_interrupt_status & ENCODER_RIGHT_B_PIN) != 0U) {
        right_encoder_count += DL_GPIO_readPins(ENCODER_RIGHT_PORT, ENCODER_RIGHT_A_PIN) ? 1 : -1;
    }

    DL_GPIO_clearInterruptStatus(ENCODER_LEFT_PORT, left_interrupt_status);
    DL_GPIO_clearInterruptStatus(ENCODER_RIGHT_PORT, right_interrupt_status);
}
