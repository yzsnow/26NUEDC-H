/**
 * @file encoder.h
 * @brief 编码器模块对外接口。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* 清零计数并开启 GPIO 编码器中断。 */
void Encoder_Init(void);
/* 原子地清零左右编码器计数。 */
void Encoder_Reset(void);
/* 获取左编码器累计计数。 */
int32_t Encoder_GetLeftCount(void);
/* 获取右编码器累计计数。 */
int32_t Encoder_GetRightCount(void);
/* 根据左右计数绝对值的平均值计算行驶距离，单位 mm。 */
float Encoder_GetAverageDistanceMm(void);

#endif