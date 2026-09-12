/**
 * @file motion.c
 * @brief H题底盘运动控制，仿照参考工程实现感维八路灰度分级循迹、丢线恢复和直道航向辅助。
 */
#include "motion.h"
#include "pid.h"
#include "motor.h"
#include "line_sensor.h"
#include "g356.h"
#include "vehicle_config.h"
#include <stdint.h>

static PID_Controller line_tracking_pid_controller;
static PID_Controller straight_heading_pid_controller;
static PID_Controller gyroscope_straight_pid_controller;
static float line_tracking_base_motor_pwm;
static float line_tracking_curve_base_pwm;
static float line_tracking_sharp_turn_base_pwm;
static float line_tracking_curve_hold_pwm;
static float line_tracking_current_base_pwm;
static float line_tracking_last_position_error;
static float line_tracking_filtered_position_error;
static float line_tracking_last_steering_correction;
static float line_tracking_error_max_change_per_step;
static float line_tracking_steering_max_change_per_step;
static float line_tracking_base_pwm_max_change_per_step;
static float line_tracking_motor_pwm_max_change_per_step;
static float line_tracking_minimum_forward_pwm;
static float line_tracking_arc_base_pwm;
static float line_tracking_arc_speed_reduction_start_error;
static float line_tracking_arc_full_speed_reduction_error;
static float line_tracking_center_error_deadband;
static float line_tracking_max_steering_pwm;
static float line_tracking_lost_line_outer_pwm;
static float line_tracking_lost_line_inner_pwm;
static uint16_t line_tracking_sensor_gap_hold_steps;
static uint16_t line_tracking_lost_line_max_search_steps;
static float straight_heading_reference_degrees;
static float gyroscope_straight_reference_degrees;
static float gyroscope_straight_target_base_pwm;
static float gyroscope_straight_current_base_pwm;
static float line_tracking_last_left_motor_pwm;
static float line_tracking_last_right_motor_pwm;
static float line_tracking_startup_output_scale;
static float line_tracking_soft_stop_left_motor_pwm;
static float line_tracking_soft_stop_right_motor_pwm;
static uint16_t line_tracking_lost_line_steps;
static uint16_t line_tracking_curve_speed_hold_steps;
static uint16_t straight_lock_confirmation_steps;
static uint16_t straight_gyro_stale_steps;
static uint32_t straight_gyro_last_valid_frame_count;
static uint16_t gyroscope_straight_stale_steps;
static uint32_t gyroscope_straight_last_valid_frame_count;
static bool line_tracking_has_last_direction;
static bool line_tracking_is_active;
static bool line_tracking_task2_control_is_enabled;
static bool line_tracking_smooth_control_is_enabled;
static bool straight_gyroscope_assist_is_active;
static bool gyroscope_straight_is_active;

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static float clamp_float(float value, float minimum_value, float maximum_value)
{
    if (value < minimum_value) return minimum_value;
    if (value > maximum_value) return maximum_value;
    return value;
}

/*
 * 限制相邻控制周期的变化量，避免外侧探头触发时误差和左右轮差速瞬间跳变。
 * 这比单纯降低PID增益更能保留直道纠偏能力，同时让半圆弯转向连续。
 */
static float approach_float(float current_value, float target_value,
    float maximum_change)
{
    if (target_value > current_value + maximum_change) {
        return current_value + maximum_change;
    }
    if (target_value < current_value - maximum_change) {
        return current_value - maximum_change;
    }
    return target_value;
}

/*
 * L2/L3或R2/R3出现时，基础速度下降和转向量增大会同时改变某一侧车轮PWM。
 * 对最终左右轮命令再做一次独立斜率限制，可保留正确纠偏方向，同时避免长车身被瞬时差速拉顿。
 */
static void drive_line_tracking_motors(float left_motor_pwm,
    float right_motor_pwm)
{
    /*
     * 任务5/6/7起步同时缩放基础速度与转向量，避免低速阶段先出现较大的左右差速。
     * 缩放作用在最终轮速目标上，与终点锁存左右轮后整体缩放的软停路径保持对称。
     */
    if (line_tracking_smooth_control_is_enabled) {
        left_motor_pwm *= line_tracking_startup_output_scale;
        right_motor_pwm *= line_tracking_startup_output_scale;
    }

    if (line_tracking_task2_control_is_enabled ||
        line_tracking_smooth_control_is_enabled) {
        left_motor_pwm = approach_float(line_tracking_last_left_motor_pwm,
            left_motor_pwm, line_tracking_motor_pwm_max_change_per_step);
        right_motor_pwm = approach_float(line_tracking_last_right_motor_pwm,
            right_motor_pwm, line_tracking_motor_pwm_max_change_per_step);
    }

    line_tracking_last_left_motor_pwm = left_motor_pwm;
    line_tracking_last_right_motor_pwm = right_motor_pwm;
    Motor_Set(left_motor_pwm, right_motor_pwm);
}

/*
 * 长车身在直线中心附近若对很小的灰度跳变持续纠偏，车尾会被放大成左右摆动。
 * 使用连续死区，在阈值边缘不产生输出跳变，只消除没有实际意义的小误差。
 */
static float apply_center_deadband(float value, float deadband)
{
    if (value > deadband) return value - deadband;
    if (value < -deadband) return value + deadband;
    return 0.0f;
}

/*
 * 胶囊形赛道只有直线和固定半径圆弧，不需要普通弯、急弯两级速度跳变。
 * 误差增大时从当前巡航PWM连续过渡到圆弧PWM，长轴距车进入和退出半圆更平顺。
 */
static float calculate_continuous_arc_base_pwm(float requested_base_pwm,
    float absolute_position_error)
{
    if (requested_base_pwm <= line_tracking_arc_base_pwm ||
        absolute_position_error <= line_tracking_arc_speed_reduction_start_error) {
        return requested_base_pwm;
    }
    if (absolute_position_error >= line_tracking_arc_full_speed_reduction_error) {
        return line_tracking_arc_base_pwm;
    }

    float speed_reduction_progress = (absolute_position_error -
        line_tracking_arc_speed_reduction_start_error) /
        (line_tracking_arc_full_speed_reduction_error -
            line_tracking_arc_speed_reduction_start_error);
    return requested_base_pwm -
        (requested_base_pwm - line_tracking_arc_base_pwm) * speed_reduction_progress;
}

/* 弯道中保证转向修正至少达到指定强度，同时保留PID计算出的方向。 */
static float apply_minimum_magnitude(float value, float minimum_magnitude,
    float direction)
{
    if (absolute_float(value) >= minimum_magnitude) return value;
    return direction < 0.0f ? -minimum_magnitude : minimum_magnitude;
}

static float normalize_heading_error_degrees(float angle_degrees)
{
    while (angle_degrees > 180.0f) angle_degrees -= 360.0f;
    while (angle_degrees < -180.0f) angle_degrees += 360.0f;
    return angle_degrees;
}

static bool left_outer_sensors_are_active(const uint8_t sensor_is_active[8])
{
    return sensor_is_active[0] != 0U || sensor_is_active[1] != 0U;
}

static bool right_outer_sensors_are_active(const uint8_t sensor_is_active[8])
{
    return sensor_is_active[6] != 0U || sensor_is_active[7] != 0U;
}

static uint8_t count_active_sensors(const uint8_t sensor_is_active[8])
{
    uint8_t active_count = 0U;
    for (uint8_t sensor_index = 0U; sensor_index < 8U; sensor_index++) {
        active_count += sensor_is_active[sensor_index];
    }
    return active_count;
}

/* 横向启停线会同时压住大多数探头，应视为居中而不是沿上次方向急转。 */
static float select_control_position_error(
    const uint8_t sensor_is_active[8], float weighted_position_error)
{
    if (count_active_sensors(sensor_is_active) >=
        COMPETITION_WIDE_LINE_MINIMUM_ACTIVE_SENSORS) {
        return 0.0f;
    }

    /*
     * 任务5/6没有锐角和直角，外侧探头只表示车身正在连续圆弧上偏离中心。
     * 直接保留八路加权误差，避免将圆弧误判为急弯并强制跳到最大误差。
     */
    if (line_tracking_task2_control_is_enabled ||
        line_tracking_smooth_control_is_enabled) {
        return weighted_position_error;
    }

    bool left_outer_is_active = left_outer_sensors_are_active(sensor_is_active);
    bool right_outer_is_active = right_outer_sensors_are_active(sensor_is_active);
    if (left_outer_is_active && !right_outer_is_active) {
        return -LINE_TRACKING_OUTER_SENSOR_FORCED_ERROR *
            LINE_TRACKING_ERROR_DIRECTION_SIGN;
    }
    if (right_outer_is_active && !left_outer_is_active) {
        return LINE_TRACKING_OUTER_SENSOR_FORCED_ERROR *
            LINE_TRACKING_ERROR_DIRECTION_SIGN;
    }
    if (left_outer_is_active && right_outer_is_active &&
        line_tracking_has_last_direction) {
        return line_tracking_last_position_error < 0.0f ?
            -LINE_TRACKING_OUTER_SENSOR_FORCED_ERROR :
            LINE_TRACKING_OUTER_SENSOR_FORCED_ERROR;
    }
    return weighted_position_error;
}

static void drive_lost_line_search(void)
{
    if (line_tracking_last_position_error < 0.0f) {
        drive_line_tracking_motors(line_tracking_lost_line_inner_pwm,
            line_tracking_lost_line_outer_pwm);
    } else {
        drive_line_tracking_motors(line_tracking_lost_line_outer_pwm,
            line_tracking_lost_line_inner_pwm);
    }
}

static void release_straight_gyroscope_assist(void)
{
    straight_gyroscope_assist_is_active = false;
    straight_lock_confirmation_steps = 0U;
    PID_Reset(&straight_heading_pid_controller);
}

/*
 * 只有“灰度误差基本居中且角速度很小”持续一段时间才锁定航向。
 * 半圆弯中角速度持续存在，因此会立即退出航向辅助，让灰度循迹独立完成转弯。
 */
static float calculate_straight_heading_correction(
    const uint8_t sensor_is_active[8], float line_position_error)
{
#if STRAIGHT_GYRO_ASSIST_ENABLE
    /*
     * 长车身从直线进入半圆时，航向环会短暂阻止车头自然转入圆弧，
     * 随后达到解锁阈值又突然退出，容易形成一次明显甩动。任务5/6因此只用
     * 经过死区和斜率限制的灰度闭环，避免两个控制环在相切位置互相对抗。
     */
    if (line_tracking_task2_control_is_enabled ||
        line_tracking_smooth_control_is_enabled) {
        release_straight_gyroscope_assist();
        return 0.0f;
    }

    uint32_t current_valid_frame_count = G356_GetValidFrameCount();
    if (current_valid_frame_count != straight_gyro_last_valid_frame_count) {
        straight_gyro_last_valid_frame_count = current_valid_frame_count;
        straight_gyro_stale_steps = 0U;
    } else if (straight_gyro_stale_steps < STRAIGHT_GYRO_STALE_MAXIMUM_STEPS) {
        straight_gyro_stale_steps++;
    }

    if (!G356_HasValidTelemetryData()) {
        release_straight_gyroscope_assist();
        return 0.0f;
    }
    if (straight_gyro_stale_steps >= STRAIGHT_GYRO_STALE_MAXIMUM_STEPS) {
        release_straight_gyroscope_assist();
        return 0.0f;
    }

    float yaw_rate_degrees_per_second =
        absolute_float(G356_GetYawRateDegreesPerSecond());
    bool outer_sensor_is_active = left_outer_sensors_are_active(sensor_is_active) ||
        right_outer_sensors_are_active(sensor_is_active);
    bool curve_is_detected = outer_sensor_is_active ||
        absolute_float(line_position_error) >= STRAIGHT_GYRO_UNLOCK_LINE_ERROR ||
        yaw_rate_degrees_per_second >= STRAIGHT_GYRO_UNLOCK_YAW_RATE_DEG_PER_S;

    if (curve_is_detected) {
        release_straight_gyroscope_assist();
        return 0.0f;
    }

    if (!straight_gyroscope_assist_is_active) {
        bool straight_is_stable =
            absolute_float(line_position_error) <= STRAIGHT_GYRO_LOCK_LINE_ERROR_MAX &&
            yaw_rate_degrees_per_second <= STRAIGHT_GYRO_LOCK_YAW_RATE_MAX_DEG_PER_S;
        if (!straight_is_stable) {
            straight_lock_confirmation_steps = 0U;
            return 0.0f;
        }
        if (straight_lock_confirmation_steps < STRAIGHT_GYRO_LOCK_CONFIRMATION_STEPS) {
            straight_lock_confirmation_steps++;
            return 0.0f;
        }
        straight_heading_reference_degrees = G356_GetAbsoluteYawDegrees();
        straight_gyroscope_assist_is_active = true;
        PID_Reset(&straight_heading_pid_controller);
    }

    float heading_error_degrees = normalize_heading_error_degrees(
        G356_GetAbsoluteYawDegrees() - straight_heading_reference_degrees);
    return PID_Calculate(&straight_heading_pid_controller,
        0.0f, heading_error_degrees);
#else
    (void)sensor_is_active;
    (void)line_position_error;
    return 0.0f;
#endif
}

void Motion_InitializeControllers(void)
{
    PID_Init(&line_tracking_pid_controller,
        LINE_TRACKING_PID_PROPORTIONAL_GAIN,
        LINE_TRACKING_PID_INTEGRAL_GAIN,
        LINE_TRACKING_PID_DERIVATIVE_GAIN,
        LINE_TRACKING_PID_OUTPUT_LIMIT);
    PID_Init(&straight_heading_pid_controller,
        STRAIGHT_GYRO_PID_PROPORTIONAL_GAIN,
        STRAIGHT_GYRO_PID_INTEGRAL_GAIN,
        STRAIGHT_GYRO_PID_DERIVATIVE_GAIN,
        STRAIGHT_GYRO_PID_OUTPUT_LIMIT);
    PID_Init(&gyroscope_straight_pid_controller,
        COMPETITION_TASK4_GYRO_PID_PROPORTIONAL_GAIN,
        COMPETITION_TASK4_GYRO_PID_INTEGRAL_GAIN,
        COMPETITION_TASK4_GYRO_PID_DERIVATIVE_GAIN,
        COMPETITION_TASK4_GYRO_PID_OUTPUT_LIMIT);
    line_tracking_base_motor_pwm = LINE_TRACKING_DRIVE_BASE_PWM;
    line_tracking_curve_base_pwm = LINE_TRACKING_CURVE_BASE_PWM;
    line_tracking_sharp_turn_base_pwm = LINE_TRACKING_SHARP_TURN_BASE_PWM;
    line_tracking_curve_hold_pwm = LINE_TRACKING_DENSE_CURVE_HOLD_PWM;
    line_tracking_current_base_pwm = LINE_TRACKING_DRIVE_BASE_PWM;
    line_tracking_last_position_error = 0.0f;
    line_tracking_filtered_position_error = 0.0f;
    line_tracking_last_steering_correction = 0.0f;
    line_tracking_error_max_change_per_step =
        LINE_TRACKING_ERROR_MAX_CHANGE_PER_STEP;
    line_tracking_steering_max_change_per_step =
        LINE_TRACKING_STEERING_MAX_CHANGE_PWM_PER_STEP;
    line_tracking_base_pwm_max_change_per_step = 0.0f;
    line_tracking_motor_pwm_max_change_per_step =
        MOTOR_PWM_MAXIMUM_DUTY_VALUE;
    line_tracking_minimum_forward_pwm = 0.0f;
    line_tracking_arc_base_pwm = LINE_TRACKING_CURVE_BASE_PWM;
    line_tracking_arc_speed_reduction_start_error =
        LINE_TRACKING_CURVE_ERROR_THRESHOLD;
    line_tracking_arc_full_speed_reduction_error =
        LINE_TRACKING_SHARP_TURN_ERROR_THRESHOLD;
    line_tracking_center_error_deadband = 0.0f;
    line_tracking_max_steering_pwm = LINE_TRACKING_PID_OUTPUT_LIMIT;
    line_tracking_lost_line_outer_pwm = LINE_TRACKING_LOST_LINE_OUTER_PWM;
    line_tracking_lost_line_inner_pwm = LINE_TRACKING_LOST_LINE_INNER_PWM;
    line_tracking_sensor_gap_hold_steps = 0U;
    line_tracking_lost_line_max_search_steps =
        LINE_TRACKING_LOST_LINE_MAX_SEARCH_STEPS;
    line_tracking_lost_line_steps = 0U;
    line_tracking_curve_speed_hold_steps = 0U;
    straight_lock_confirmation_steps = 0U;
    straight_gyro_stale_steps = 0U;
    straight_gyro_last_valid_frame_count = G356_GetValidFrameCount();
    line_tracking_has_last_direction = false;
    line_tracking_is_active = false;
    line_tracking_task2_control_is_enabled = false;
    line_tracking_smooth_control_is_enabled = false;
    straight_gyroscope_assist_is_active = false;
    gyroscope_straight_is_active = false;
    gyroscope_straight_reference_degrees = 0.0f;
    gyroscope_straight_target_base_pwm = 0.0f;
    gyroscope_straight_current_base_pwm = 0.0f;
    line_tracking_last_left_motor_pwm = 0.0f;
    line_tracking_last_right_motor_pwm = 0.0f;
    line_tracking_startup_output_scale = 1.0f;
    gyroscope_straight_stale_steps = 0U;
    gyroscope_straight_last_valid_frame_count = G356_GetValidFrameCount();
}

void Motion_StartLineTracking(float base_motor_pwm, float curve_base_pwm,
    float sharp_turn_base_pwm, float curve_hold_pwm,
    bool task2_control_is_enabled, bool smooth_control_is_enabled)
{
    if (task2_control_is_enabled) {
        PID_Init(&line_tracking_pid_controller,
            COMPETITION_TASK2_PID_PROPORTIONAL_GAIN,
            COMPETITION_TASK2_PID_INTEGRAL_GAIN,
            COMPETITION_TASK2_PID_DERIVATIVE_GAIN,
            COMPETITION_TASK2_PID_OUTPUT_LIMIT);
        line_tracking_error_max_change_per_step =
            COMPETITION_TASK2_ERROR_MAX_CHANGE_PER_STEP;
        line_tracking_steering_max_change_per_step =
            COMPETITION_TASK2_STEERING_MAX_CHANGE_PER_STEP;
        line_tracking_base_pwm_max_change_per_step =
            COMPETITION_TASK2_BASE_PWM_MAX_CHANGE_PER_STEP;
        line_tracking_motor_pwm_max_change_per_step =
            COMPETITION_TASK2_MOTOR_PWM_MAX_CHANGE_PER_STEP;
        line_tracking_minimum_forward_pwm =
            COMPETITION_TASK2_MINIMUM_FORWARD_PWM;
        line_tracking_arc_base_pwm = COMPETITION_TASK2_ARC_BASE_PWM;
        line_tracking_arc_speed_reduction_start_error =
            COMPETITION_TASK2_ARC_SPEED_REDUCTION_START_ERROR;
        line_tracking_arc_full_speed_reduction_error =
            COMPETITION_TASK2_ARC_FULL_SPEED_REDUCTION_ERROR;
        line_tracking_center_error_deadband =
            COMPETITION_TASK2_CENTER_ERROR_DEADBAND;
        line_tracking_max_steering_pwm = COMPETITION_TASK2_MAX_STEERING_PWM;
        line_tracking_lost_line_outer_pwm =
            COMPETITION_TASK2_LOST_LINE_OUTER_PWM;
        line_tracking_lost_line_inner_pwm =
            COMPETITION_TASK2_LOST_LINE_INNER_PWM;
        line_tracking_sensor_gap_hold_steps =
            COMPETITION_TASK2_SENSOR_GAP_HOLD_STEPS;
        line_tracking_lost_line_max_search_steps =
            COMPETITION_TASK2_LOST_LINE_MAX_SEARCH_STEPS;
    } else if (smooth_control_is_enabled) {
        PID_Init(&line_tracking_pid_controller,
            COMPETITION_TASK5_6_PID_PROPORTIONAL_GAIN,
            COMPETITION_TASK5_6_PID_INTEGRAL_GAIN,
            COMPETITION_TASK5_6_PID_DERIVATIVE_GAIN,
            COMPETITION_TASK5_6_PID_OUTPUT_LIMIT);
        line_tracking_error_max_change_per_step =
            COMPETITION_TASK5_6_ERROR_MAX_CHANGE_PER_STEP;
        line_tracking_steering_max_change_per_step =
            COMPETITION_TASK5_6_STEERING_MAX_CHANGE_PER_STEP;
        line_tracking_base_pwm_max_change_per_step =
            COMPETITION_TASK5_6_BASE_PWM_MAX_CHANGE_PER_STEP;
        line_tracking_motor_pwm_max_change_per_step =
            COMPETITION_TASK5_6_MOTOR_PWM_MAX_CHANGE_PER_STEP;
        line_tracking_minimum_forward_pwm =
            COMPETITION_TASK5_6_MINIMUM_FORWARD_PWM;
        line_tracking_arc_base_pwm = COMPETITION_TASK5_6_ARC_BASE_PWM;
        line_tracking_arc_speed_reduction_start_error =
            COMPETITION_TASK5_6_ARC_SPEED_REDUCTION_START_ERROR;
        line_tracking_arc_full_speed_reduction_error =
            COMPETITION_TASK5_6_ARC_FULL_SPEED_REDUCTION_ERROR;
        line_tracking_center_error_deadband =
            COMPETITION_TASK5_6_CENTER_ERROR_DEADBAND;
        line_tracking_max_steering_pwm = COMPETITION_TASK5_6_MAX_STEERING_PWM;
        line_tracking_lost_line_outer_pwm =
            COMPETITION_TASK5_6_LOST_LINE_OUTER_PWM;
        line_tracking_lost_line_inner_pwm =
            COMPETITION_TASK5_6_LOST_LINE_INNER_PWM;
        line_tracking_sensor_gap_hold_steps =
            COMPETITION_TASK5_6_SENSOR_GAP_HOLD_STEPS;
        line_tracking_lost_line_max_search_steps =
            COMPETITION_TASK5_6_LOST_LINE_MAX_SEARCH_STEPS;
    } else {
        PID_Init(&line_tracking_pid_controller,
            LINE_TRACKING_PID_PROPORTIONAL_GAIN,
            LINE_TRACKING_PID_INTEGRAL_GAIN,
            LINE_TRACKING_PID_DERIVATIVE_GAIN,
            LINE_TRACKING_PID_OUTPUT_LIMIT);
        line_tracking_error_max_change_per_step =
            LINE_TRACKING_ERROR_MAX_CHANGE_PER_STEP;
        line_tracking_steering_max_change_per_step =
            LINE_TRACKING_STEERING_MAX_CHANGE_PWM_PER_STEP;
        line_tracking_base_pwm_max_change_per_step = 0.0f;
        line_tracking_motor_pwm_max_change_per_step =
            MOTOR_PWM_MAXIMUM_DUTY_VALUE;
        line_tracking_minimum_forward_pwm = 0.0f;
        line_tracking_arc_base_pwm = LINE_TRACKING_CURVE_BASE_PWM;
        line_tracking_arc_speed_reduction_start_error =
            LINE_TRACKING_CURVE_ERROR_THRESHOLD;
        line_tracking_arc_full_speed_reduction_error =
            LINE_TRACKING_SHARP_TURN_ERROR_THRESHOLD;
        line_tracking_center_error_deadband = 0.0f;
        line_tracking_max_steering_pwm = LINE_TRACKING_PID_OUTPUT_LIMIT;
        line_tracking_lost_line_outer_pwm = LINE_TRACKING_LOST_LINE_OUTER_PWM;
        line_tracking_lost_line_inner_pwm = LINE_TRACKING_LOST_LINE_INNER_PWM;
        line_tracking_sensor_gap_hold_steps = 0U;
        line_tracking_lost_line_max_search_steps =
            LINE_TRACKING_LOST_LINE_MAX_SEARCH_STEPS;
    }
    PID_Reset(&straight_heading_pid_controller);
    line_tracking_base_motor_pwm = clamp_float(base_motor_pwm,
        0.0f, MOTOR_PWM_MAXIMUM_DUTY_VALUE);
    line_tracking_curve_base_pwm = clamp_float(curve_base_pwm,
        0.0f, line_tracking_base_motor_pwm);
    line_tracking_sharp_turn_base_pwm = clamp_float(sharp_turn_base_pwm,
        0.0f, line_tracking_curve_base_pwm);
    line_tracking_curve_hold_pwm = clamp_float(curve_hold_pwm,
        0.0f, line_tracking_base_motor_pwm);
    /*
     * 任务5/6从完整循迹目标开始计算，再由整体比例从零平滑释放实际左右轮输出。
     * 任务2仍保留原基础PWM斜率限制，不改变已经标定完成的高速起步行为。
     */
    line_tracking_current_base_pwm = task2_control_is_enabled ?
        0.0f : line_tracking_base_motor_pwm;
    line_tracking_last_position_error = 0.0f;
    line_tracking_filtered_position_error = 0.0f;
    line_tracking_last_steering_correction = 0.0f;
    line_tracking_last_left_motor_pwm = 0.0f;
    line_tracking_last_right_motor_pwm = 0.0f;
    line_tracking_startup_output_scale = smooth_control_is_enabled ? 0.0f : 1.0f;
    line_tracking_lost_line_steps = 0U;
    line_tracking_curve_speed_hold_steps = 0U;
    straight_lock_confirmation_steps = 0U;
    straight_gyro_stale_steps = 0U;
    straight_gyro_last_valid_frame_count = G356_GetValidFrameCount();
    line_tracking_has_last_direction = false;
    line_tracking_is_active = true;
    line_tracking_task2_control_is_enabled = task2_control_is_enabled;
    line_tracking_smooth_control_is_enabled = smooth_control_is_enabled;
    straight_gyroscope_assist_is_active = false;
    gyroscope_straight_is_active = false;
}

void Motion_StartGyroscopeStraight(float base_motor_pwm)
{
    Motion_StopLineTracking();
    PID_Init(&gyroscope_straight_pid_controller,
        COMPETITION_TASK4_GYRO_PID_PROPORTIONAL_GAIN,
        COMPETITION_TASK4_GYRO_PID_INTEGRAL_GAIN,
        COMPETITION_TASK4_GYRO_PID_DERIVATIVE_GAIN,
        COMPETITION_TASK4_GYRO_PID_OUTPUT_LIMIT);
    gyroscope_straight_target_base_pwm = clamp_float(base_motor_pwm,
        0.0f, MOTOR_PWM_MAXIMUM_DUTY_VALUE);
    gyroscope_straight_current_base_pwm = 0.0f;
    gyroscope_straight_stale_steps = 0U;
    gyroscope_straight_last_valid_frame_count = G356_GetValidFrameCount();
    if (!G356_HasValidTelemetryData()) {
        gyroscope_straight_is_active = false;
        return;
    }

    /* 启动瞬间锁定当前Yaw，任务4后续不读取灰度，只保持这一物理方向。 */
    gyroscope_straight_reference_degrees = G356_GetAbsoluteYawDegrees();
    gyroscope_straight_is_active = true;
}

void Motion_SetGyroscopeStraightBasePwm(float base_motor_pwm)
{
    gyroscope_straight_target_base_pwm = clamp_float(base_motor_pwm,
        0.0f, MOTOR_PWM_MAXIMUM_DUTY_VALUE);
}

MotionGyroscopeStraightStatus Motion_RunGyroscopeStraightControlStep(void)
{
    uint32_t current_valid_frame_count = G356_GetValidFrameCount();
    if (current_valid_frame_count != gyroscope_straight_last_valid_frame_count) {
        gyroscope_straight_last_valid_frame_count = current_valid_frame_count;
        gyroscope_straight_stale_steps = 0U;
    } else if (gyroscope_straight_stale_steps <
        COMPETITION_TASK4_GYRO_STALE_MAXIMUM_STEPS) {
        gyroscope_straight_stale_steps++;
    }

    if (!gyroscope_straight_is_active || !G356_HasValidTelemetryData() ||
        gyroscope_straight_stale_steps >=
            COMPETITION_TASK4_GYRO_STALE_MAXIMUM_STEPS) {
        gyroscope_straight_is_active = false;
        Motor_Stop();
        return MOTION_GYROSCOPE_STRAIGHT_STATUS_GYRO_UNAVAILABLE;
    }

    gyroscope_straight_current_base_pwm = approach_float(
        gyroscope_straight_current_base_pwm,
        gyroscope_straight_target_base_pwm,
        COMPETITION_TASK4_GYRO_BASE_PWM_MAX_CHANGE_PER_STEP);
    float heading_error_degrees = normalize_heading_error_degrees(
        G356_GetAbsoluteYawDegrees() - gyroscope_straight_reference_degrees);
    float steering_correction = PID_Calculate(&gyroscope_straight_pid_controller,
        0.0f, heading_error_degrees);
    /*
     * 实车左右电机和轮胎阻力不完全一致，仅靠无积分航向PD会留下固定向右偏差。
     * 在PD输出后增加很小的机械补偿，仍由陀螺仪负责动态纠偏，不改变目标航向。
     */
    /*
     * 线性软停接近0PWM时，固定机械补偿可能让一侧轮子的命令变成负值并产生反向冲击。
     * 因此航向修正上限同时受当前基础PWM约束，保证软停期间两侧车轮始终只前进或为零。
     */
    float gyroscope_steering_output_limit = clamp_float(
        gyroscope_straight_current_base_pwm, 0.0f,
        COMPETITION_TASK4_GYRO_PID_OUTPUT_LIMIT);
    steering_correction = clamp_float(steering_correction +
        COMPETITION_TASK4_GYRO_STEERING_TRIM_PWM,
        -gyroscope_steering_output_limit,
        gyroscope_steering_output_limit);
    Motor_Set(gyroscope_straight_current_base_pwm - steering_correction,
        gyroscope_straight_current_base_pwm + steering_correction);
    return MOTION_GYROSCOPE_STRAIGHT_STATUS_FOLLOWING;
}

void Motion_SetLineTrackingBasePwm(float base_motor_pwm)
{
    line_tracking_base_motor_pwm = clamp_float(base_motor_pwm,
        0.0f, MOTOR_PWM_MAXIMUM_DUTY_VALUE);
}

void Motion_SetLineTrackingStartupScale(float output_scale)
{
    if (!line_tracking_smooth_control_is_enabled) return;
    line_tracking_startup_output_scale = clamp_float(output_scale, 0.0f, 1.0f);
}

void Motion_StartLineTrackingSoftStop(void)
{
    /*
     * 横线只有5cm，确认后继续读取灰度可能很快变成全灭或偏向一侧。
     * 锁存确认前最后一次实际轮速并退出PID，可让车辆沿原轨迹只做纵向减速，不再突然找线或打舵。
     */
    line_tracking_soft_stop_left_motor_pwm =
        line_tracking_last_left_motor_pwm;
    line_tracking_soft_stop_right_motor_pwm =
        line_tracking_last_right_motor_pwm;
    release_straight_gyroscope_assist();
    line_tracking_is_active = false;
}

void Motion_RunLineTrackingSoftStop(float remaining_ratio)
{
    remaining_ratio = clamp_float(remaining_ratio, 0.0f, 1.0f);
    Motor_Set(line_tracking_soft_stop_left_motor_pwm * remaining_ratio,
        line_tracking_soft_stop_right_motor_pwm * remaining_ratio);
}

MotionLineTrackingStatus Motion_RunLineTrackingControlStep(void)
{
    uint8_t sensor_is_active[8];
    float line_position_error;

    if (!line_tracking_is_active) {
        return MOTION_LINE_TRACKING_STATUS_STOPPED_LINE_LOST;
    }

    if (!LineSensor_ReadPosition(sensor_is_active, &line_position_error)) {
        release_straight_gyroscope_assist();
        /*
         * 数字灰度从L2切到L3或R2切到R3时，探头间隙可能造成一两个周期全灭。
         * 任务2和任务5/6/7先保持上一周期左右轮输出，不重置PID也不进入大差速找线，
         * 只有全灭持续超过15～20ms才认为真正丢线，从而消除直线纠偏时的瞬时顿挫。
         */
        if ((line_tracking_task2_control_is_enabled ||
            line_tracking_smooth_control_is_enabled) &&
            line_tracking_has_last_direction &&
            line_tracking_lost_line_steps <
                line_tracking_sensor_gap_hold_steps) {
            Motor_Set(line_tracking_last_left_motor_pwm,
                line_tracking_last_right_motor_pwm);
            line_tracking_lost_line_steps++;
            return MOTION_LINE_TRACKING_STATUS_SEARCHING;
        }
        if (!line_tracking_has_last_direction ||
            line_tracking_lost_line_steps >=
                line_tracking_lost_line_max_search_steps) {
            Motion_StopLineTracking();
            return MOTION_LINE_TRACKING_STATUS_STOPPED_LINE_LOST;
        }
        PID_Reset(&line_tracking_pid_controller);
        drive_lost_line_search();
        line_tracking_lost_line_steps++;
        return MOTION_LINE_TRACKING_STATUS_SEARCHING;
    }

    line_tracking_lost_line_steps = 0U;
    line_position_error = select_control_position_error(
        sensor_is_active, line_position_error);
    line_tracking_filtered_position_error = approach_float(
        line_tracking_filtered_position_error, line_position_error,
        line_tracking_error_max_change_per_step);
    line_position_error = line_tracking_filtered_position_error;
    if (line_tracking_task2_control_is_enabled ||
        line_tracking_smooth_control_is_enabled) {
        line_position_error = apply_center_deadband(line_position_error,
            line_tracking_center_error_deadband);
    }
    if (absolute_float(line_position_error) >= LINE_TRACKING_DIRECTION_MEMORY_ERROR) {
        line_tracking_last_position_error = line_position_error;
        line_tracking_has_last_direction = true;
    }

    float absolute_position_error = absolute_float(line_position_error);
    float target_base_pwm = line_tracking_base_motor_pwm;
    float steering_correction = PID_Calculate(
        &line_tracking_pid_controller, 0.0f, line_position_error);
    float correction_direction = line_position_error < 0.0f ? 1.0f : -1.0f;

    if (line_tracking_task2_control_is_enabled ||
        line_tracking_smooth_control_is_enabled) {
        /*
         * 任务2和任务5/6/7只有直线和圆弧，按误差连续降速即可，不使用急弯最小打舵。
         * 这样长车身不会在圆弧入口突然甩头，圆弧出口也不会因保持档突然回正。
         */
        line_tracking_curve_speed_hold_steps = 0U;
        target_base_pwm = calculate_continuous_arc_base_pwm(
            target_base_pwm, absolute_position_error);
    } else {
        /* 普通循迹模式保留原有分级弯道策略。 */
        if (absolute_position_error >= LINE_TRACKING_SHARP_TURN_ERROR_THRESHOLD) {
            line_tracking_curve_speed_hold_steps = LINE_TRACKING_CURVE_SPEED_HOLD_STEPS;
            if (target_base_pwm > line_tracking_sharp_turn_base_pwm) {
                target_base_pwm = line_tracking_sharp_turn_base_pwm;
            }
            steering_correction = apply_minimum_magnitude(steering_correction,
                LINE_TRACKING_SHARP_TURN_MIN_CORRECTION_PWM, correction_direction);
        } else if (absolute_position_error >= LINE_TRACKING_CURVE_ERROR_THRESHOLD) {
            line_tracking_curve_speed_hold_steps = LINE_TRACKING_CURVE_SPEED_HOLD_STEPS;
            if (target_base_pwm > line_tracking_curve_base_pwm) {
                target_base_pwm = line_tracking_curve_base_pwm;
            }
            steering_correction = apply_minimum_magnitude(steering_correction,
                LINE_TRACKING_CURVE_MIN_CORRECTION_PWM, correction_direction);
        } else if (line_tracking_curve_speed_hold_steps > 0U) {
            line_tracking_curve_speed_hold_steps--;
            if (target_base_pwm > line_tracking_curve_hold_pwm) {
                target_base_pwm = line_tracking_curve_hold_pwm;
            }
        }
    }

    /*
     * 任务5/6的直道、弯道和终点速度都可能跨档变化，统一通过基础PWM斜坡过渡，
     * 避免车身前后顿挫；其他任务仍保持原有的即时速度响应。
     */
    if (line_tracking_task2_control_is_enabled ||
        line_tracking_smooth_control_is_enabled) {
        line_tracking_current_base_pwm = approach_float(
            line_tracking_current_base_pwm, target_base_pwm,
            line_tracking_base_pwm_max_change_per_step);
    } else {
        line_tracking_current_base_pwm = target_base_pwm;
    }
    float current_base_pwm = line_tracking_current_base_pwm;

    steering_correction += calculate_straight_heading_correction(
        sensor_is_active, line_position_error);
    if (line_tracking_task2_control_is_enabled ||
        line_tracking_smooth_control_is_enabled) {
        /*
         * 低速时转向量若超过基础PWM，内轮会突然反转并造成明显顿挫。
         * 保留少量同向PWM，通过连续差速完成转弯，而不是在弯中切换电机方向。
         */
        float steering_output_limit = current_base_pwm -
            line_tracking_minimum_forward_pwm;
        steering_output_limit = clamp_float(steering_output_limit,
            0.0f, line_tracking_max_steering_pwm);
        steering_correction = clamp_float(steering_correction,
            -steering_output_limit, steering_output_limit);
    } else {
        steering_correction = clamp_float(steering_correction,
            -LINE_TRACKING_PID_OUTPUT_LIMIT, LINE_TRACKING_PID_OUTPUT_LIMIT);
    }
    steering_correction = approach_float(
        line_tracking_last_steering_correction, steering_correction,
        line_tracking_steering_max_change_per_step);
    if (line_tracking_task2_control_is_enabled ||
        line_tracking_smooth_control_is_enabled) {
        float steering_output_limit = clamp_float(current_base_pwm -
            line_tracking_minimum_forward_pwm, 0.0f,
            line_tracking_max_steering_pwm);
        steering_correction = clamp_float(steering_correction,
            -steering_output_limit, steering_output_limit);
    }
    line_tracking_last_steering_correction = steering_correction;
    drive_line_tracking_motors(current_base_pwm - steering_correction,
        current_base_pwm + steering_correction);
    return MOTION_LINE_TRACKING_STATUS_FOLLOWING;
}

void Motion_StopLineTracking(void)
{
    line_tracking_is_active = false;
    line_tracking_lost_line_steps = 0U;
    line_tracking_curve_speed_hold_steps = 0U;
    line_tracking_has_last_direction = false;
    line_tracking_filtered_position_error = 0.0f;
    line_tracking_last_steering_correction = 0.0f;
    line_tracking_current_base_pwm = 0.0f;
    line_tracking_last_left_motor_pwm = 0.0f;
    line_tracking_last_right_motor_pwm = 0.0f;
    line_tracking_startup_output_scale = 1.0f;
    line_tracking_soft_stop_left_motor_pwm = 0.0f;
    line_tracking_soft_stop_right_motor_pwm = 0.0f;
    line_tracking_task2_control_is_enabled = false;
    line_tracking_smooth_control_is_enabled = false;
    gyroscope_straight_is_active = false;
    gyroscope_straight_target_base_pwm = 0.0f;
    gyroscope_straight_current_base_pwm = 0.0f;
    release_straight_gyroscope_assist();
    PID_Reset(&line_tracking_pid_controller);
    PID_Reset(&gyroscope_straight_pid_controller);
    Motor_Stop();
}

bool Motion_IsStraightGyroscopeAssistActive(void)
{
    return straight_gyroscope_assist_is_active || gyroscope_straight_is_active;
}

float Motion_GetLastLinePositionError(void)
{
    return line_tracking_last_position_error;
}
