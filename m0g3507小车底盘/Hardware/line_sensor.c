/**
 * @file line_sensor.c
 * @brief 感维八路灰度并行数字输出读取与加权位置误差计算。
 */
#include "line_sensor.h"
#include "ti_msp_dl_config.h"
#include "vehicle_config.h"

static const float line_sensor_position_weights[8] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f
};

/* 一次读取感维灰度模块的八路GPIO数字信号；当前硬件按低电平有效，不使用I2C。 */
void LineSensor_Read(uint8_t sensor_is_active[8])
{
    uint32_t sensor_pin_states = DL_GPIO_readPins(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS_MASK);
    sensor_is_active[0] = (sensor_pin_states & LINE_L4_PIN) ? 0U : 1U;
    sensor_is_active[1] = (sensor_pin_states & LINE_L3_PIN) ? 0U : 1U;
    sensor_is_active[2] = (sensor_pin_states & LINE_L2_PIN) ? 0U : 1U;
    sensor_is_active[3] = (sensor_pin_states & LINE_L1_PIN) ? 0U : 1U;
    sensor_is_active[4] = (sensor_pin_states & LINE_R1_PIN) ? 0U : 1U;
    sensor_is_active[5] = (sensor_pin_states & LINE_R2_PIN) ? 0U : 1U;
    sensor_is_active[6] = (sensor_pin_states & LINE_R3_PIN) ? 0U : 1U;
    sensor_is_active[7] = (sensor_pin_states & LINE_R4_PIN) ? 0U : 1U;
}

/* 用 -3.5～+3.5 权重计算线相对车体中心的位置误差。 */
bool LineSensor_CalculatePosition(const uint8_t sensor_is_active[8], float *line_position_error)
{
    float active_sensor_count = 0.0f;
    float weighted_position_sum = 0.0f;
    for (uint32_t sensor_index = 0U; sensor_index < 8U; sensor_index++) {
        active_sensor_count += sensor_is_active[sensor_index];
        weighted_position_sum += sensor_is_active[sensor_index] * line_sensor_position_weights[sensor_index];
    }
    if (active_sensor_count == 0.0f) return false;
    /*
     * 灰度板安装方向或八路排线顺序与程序定义相反时，只在这里统一反转误差，
     * 避免在PID和左右轮输出中重复修改方向。
     */
    *line_position_error = weighted_position_sum / active_sensor_count *
        LINE_TRACKING_ERROR_DIRECTION_SIGN;
    return true;
}

/* 同时返回八路原始状态和加权位置误差，供锐角判断使用。 */
bool LineSensor_ReadPosition(uint8_t sensor_is_active[8], float *line_position_error)
{
    LineSensor_Read(sensor_is_active);
    return LineSensor_CalculatePosition(sensor_is_active, line_position_error);
}

/* 只需要位置误差时使用的简化接口。 */
bool LineSensor_GetPosition(float *line_position_error)
{
    uint8_t sensor_is_active[8];
    return LineSensor_ReadPosition(sensor_is_active, line_position_error);
}
