/**
 * @file competition_control.h
 * @brief H题天猛星底盘比赛任务状态机对外接口。
 */
#ifndef COMPETITION_CONTROL_H
#define COMPETITION_CONTROL_H

/**
 * @brief 初始化比赛模式、显示状态和停车保护。
 * @note 必须在底层硬件、按键、运动控制和系统时间初始化后调用。
 */
void CompetitionControl_Initialize(void);

/**
 * @brief 执行一次比赛任务调度、按键处理和OLED刷新。
 * @note 主循环应约每5ms调用一次，运行中确认键或返回键会立即停车。
 */
void CompetitionControl_Process(void);

#endif
