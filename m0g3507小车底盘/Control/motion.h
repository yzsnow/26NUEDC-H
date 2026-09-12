/**
 * @file motion.h
 * @brief H题底盘循迹接口，仿照参考工程提供感维八路灰度分级PID、丢线保护和直道陀螺仪辅助。
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>

typedef enum {
    MOTION_LINE_TRACKING_STATUS_FOLLOWING = 0,
    MOTION_LINE_TRACKING_STATUS_SEARCHING,
    MOTION_LINE_TRACKING_STATUS_STOPPED_LINE_LOST
} MotionLineTrackingStatus;

typedef enum {
    MOTION_GYROSCOPE_STRAIGHT_STATUS_FOLLOWING = 0,
    MOTION_GYROSCOPE_STRAIGHT_STATUS_GYRO_UNAVAILABLE
} MotionGyroscopeStraightStatus;

/**
 * @brief 初始化循迹与航向辅助控制器。
 * @note 必须在电机、编码器和G356接收模块初始化后调用。
 */
void Motion_InitializeControllers(void);

/**
 * @brief 按指定直道和弯道速度档开始一次新的循迹任务。
 * @param base_motor_pwm 直线基础PWM。
 * @param curve_base_pwm 普通弯基础PWM上限。
 * @param sharp_turn_base_pwm 急弯基础PWM上限。
 * @param curve_hold_pwm 出弯短时保持的基础PWM上限。
 * @param task2_control_is_enabled true时使用任务2高速圆弧控制。
 * @param smooth_control_is_enabled true时使用任务5/6长车柔和圆弧控制。
 * @note 调用时会清空PID、丢线和直线航向锁定历史。
 */
void Motion_StartLineTracking(float base_motor_pwm, float curve_base_pwm,
    float sharp_turn_base_pwm, float curve_hold_pwm,
    bool task2_control_is_enabled, bool smooth_control_is_enabled);

/**
 * @brief 启动任务4陀螺仪定向直行。
 * @param base_motor_pwm 直行基础PWM。
 * @note 必须在G356已经收到有效数据后调用；启动时锁定当前Yaw为直行参考方向。
 */
void Motion_StartGyroscopeStraight(float base_motor_pwm);

/**
 * @brief 修改任务4陀螺仪直行基础PWM，用于接近B点时平滑减速。
 * @param base_motor_pwm 新的基础PWM。
 */
void Motion_SetGyroscopeStraightBasePwm(float base_motor_pwm);

/**
 * @brief 执行一次任务4陀螺仪定向直行闭环。
 * @return 正常直行或陀螺仪数据不可用。
 */
MotionGyroscopeStraightStatus Motion_RunGyroscopeStraightControlStep(void);

/**
 * @brief 运行中修改直线基础PWM，用于终点前减速。
 * @param base_motor_pwm 新的基础PWM。
 * @note 只修改巡航速度，不清空当前循迹状态。
 */
void Motion_SetLineTrackingBasePwm(float base_motor_pwm);

/**
 * @brief 设置任务5/6起步阶段的左右轮整体输出比例。
 * @param output_scale 输出比例，0表示保持停车，1表示完整输出循迹计算结果。
 * @note 只在柔和循迹模式中生效，使起步与终点软停使用互为镜像的五次S曲线。
 */
void Motion_SetLineTrackingStartupScale(float output_scale);

/**
 * @brief 锁存任务5/6识别横线前最后一次左右轮输出，准备线性软停车。
 * @note 调用后停止执行循迹PID，避免车身越过短横线后因探头状态变化再次转向。
 */
void Motion_StartLineTrackingSoftStop(void);

/**
 * @brief 按锁存轮速的相同比例输出任务5/6软停车命令。
 * @param remaining_ratio 剩余轮速比例，1保持横线前轮速，0降为零。
 * @note 必须先调用Motion_StartLineTrackingSoftStop，比例会自动限制在0～1。
 */
void Motion_RunLineTrackingSoftStop(float remaining_ratio);

/**
 * @brief 执行一次循迹闭环。
 * @return 当前循迹、短时找线或丢线停车状态。
 * @note 设计调用周期约5ms，周期变化过大会影响PID参数。
 */
MotionLineTrackingStatus Motion_RunLineTrackingControlStep(void);

/**
 * @brief 立即退出循迹或任务4陀螺仪直行，并制动停车。
 */
void Motion_StopLineTracking(void);

/**
 * @brief 查询当前是否正在使用陀螺仪直道航向辅助。
 * @return 已锁定直道航向返回true，否则返回false。
 */
bool Motion_IsStraightGyroscopeAssistActive(void);

/**
 * @brief 获取最近一次有效的循迹位置误差。
 * @return 八路加权后的相对位置误差。
 */
float Motion_GetLastLinePositionError(void);

#endif
