/**
 * @file inertia_telemetry.h
 * @brief 向视觉控球主控发送底盘惯性前馈遥测数据。
 */
#ifndef INERTIA_TELEMETRY_H
#define INERTIA_TELEMETRY_H

#include <stdint.h>

#define INERTIA_TELEMETRY_TASK_NONE       0U
#define INERTIA_TELEMETRY_TASK2           2U
#define INERTIA_TELEMETRY_TASK4           4U
#define INERTIA_TELEMETRY_TASK5_6        56U

#define INERTIA_TELEMETRY_STATE_STANDBY   0U
#define INERTIA_TELEMETRY_STATE_RUNNING   1U
#define INERTIA_TELEMETRY_STATE_STOPPING  2U
#define INERTIA_TELEMETRY_STATE_SETTLING  3U

/**
 * @brief 初始化UART1惯性遥测发送状态和编码器估速器。
 * @note 必须在SYSCFG_DL_init()和SystemTime_Initialize()之后调用。
 */
void InertiaTelemetry_Initialize(void);

/**
 * @brief 按固定50Hz节拍生成并异步发送一帧底盘惯性数据。
 * @param task_id 当前任务编号，任务5/6统一使用56。
 * @param motion_state 待机、运行、软停或停车后稳定状态。
 * @note 应在主循环每周期调用；UART中断负责实际逐字节发送，不阻塞循迹采样。
 */
void InertiaTelemetry_Process(uint8_t task_id, uint8_t motion_state);

#endif
