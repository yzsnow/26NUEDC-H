/**
 * @file system_time.h
 * @brief 基于SysTick的毫秒时间基准，为比赛计时和非阻塞超时判断提供统一接口。
 */
#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdint.h>

/**
 * @brief 初始化1ms SysTick中断并清零系统时间。
 * @note 必须在SYSCFG_DL_init()完成系统时钟配置后调用。
 */
void SystemTime_Initialize(void);

/**
 * @brief 获取上电后累计毫秒数。
 * @return 32位毫秒计数，约49天回绕一次。
 */
uint32_t SystemTime_GetMilliseconds(void);

/**
 * @brief 判断从指定时刻起是否已经经过目标时间。
 * @param start_milliseconds 起始毫秒数。
 * @param interval_milliseconds 目标间隔。
 * @return 已达到间隔返回1，否则返回0。
 */
uint8_t SystemTime_HasElapsed(
    uint32_t start_milliseconds, uint32_t interval_milliseconds);

#endif
