/**
 * @file line_sensor.h
 * @brief 感维八路灰度并行数字输出读取与位置估计接口。
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/* 读取八路GPIO数字状态；数组顺序为L4、L3、L2、L1、R1、R2、R3、R4。 */
void LineSensor_Read(uint8_t sensor_is_active[8]);
/* 根据已读取的八路状态计算加权位置误差。 */
bool LineSensor_CalculatePosition(const uint8_t sensor_is_active[8], float *line_position_error);
/* 一次完成八路读取和位置误差计算。 */
bool LineSensor_ReadPosition(uint8_t sensor_is_active[8], float *line_position_error);
/* 只获取位置误差，不需要保留八路原始状态时使用。 */
bool LineSensor_GetPosition(float *line_position_error);

#endif
