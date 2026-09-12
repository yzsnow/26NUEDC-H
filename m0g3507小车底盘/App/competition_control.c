/**
 * @file competition_control.c
 * @brief H题底盘比赛状态机，实现AB停车、单圈起终点识别、计时显示和安全停车。
 *
 * 本模块只负责天猛星底盘。任务5与任务6在底盘侧使用同一套稳速单圈流程，
 * 任务7复用其柔和循迹参数但取消单圈终点限制；后续由地猛星控球板分别保持中心位置
 * 或赛前指定位置。
 */
#include "competition_control.h"
#include "vehicle_config.h"
#include "inertia_telemetry.h"
#include "system_time.h"
#include "menu_buttons.h"
#include "line_sensor.h"
#include "encoder.h"
#include "motion.h"
#include "motor.h"
#include "g356.h"
#include "oled.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    COMPETITION_MODE_TASK2_FAST_LAP = 0,
    COMPETITION_MODE_TASK4_GYROSCOPE_STRAIGHT,
    COMPETITION_MODE_TASK5_6_STEADY_LAP,
    COMPETITION_MODE_TASK7_INFINITE_LINE,
    COMPETITION_MODE_GYROSCOPE_STATUS,
    COMPETITION_MODE_COUNT
} CompetitionMode;

typedef enum {
    COMPETITION_STATE_READY = 0,
    COMPETITION_STATE_RUNNING,
    COMPETITION_STATE_FINISHED,
    COMPETITION_STATE_STOPPED_BY_USER,
    COMPETITION_STATE_LINE_LOST,
    COMPETITION_STATE_TIMEOUT,
    COMPETITION_STATE_DISTANCE_FAILSAFE,
    COMPETITION_STATE_GYROSCOPE_UNAVAILABLE,
    COMPETITION_STATE_DISTANCE_SETTING,
    COMPETITION_STATE_GYROSCOPE_STATUS
} CompetitionState;

static CompetitionMode selected_mode;
static CompetitionState competition_state;
static uint32_t run_start_milliseconds;
static uint32_t finish_milliseconds;
static uint32_t last_display_refresh_milliseconds;
static uint8_t finish_line_confirmation_steps;
static uint8_t finish_accumulated_sensor_mask;
static float finish_accumulation_start_distance_mm;
static bool finish_clockwise_arc_was_seen;
static uint32_t task4_speed_sample_milliseconds;
static uint32_t task4_soft_stop_start_milliseconds;
static float task4_speed_sample_distance_mm;
static float task4_estimated_speed_mm_per_second;
static float task4_soft_stop_start_pwm;
static bool task4_soft_stop_is_active;
static uint32_t task5_6_soft_stop_start_milliseconds;
static uint32_t task5_6_soft_stop_duration_milliseconds;
static bool task5_6_soft_stop_is_active;
static CompetitionState task5_6_soft_stop_finish_state;
static float task2_fixed_distance_stop_mm;
static float task4_target_distance_mm;
static float task5_6_fixed_distance_stop_mm;
static float task7_base_pwm;
static float distance_setting_original_mm;

static uint8_t count_active_sensors(const uint8_t sensor_is_active[8])
{
    uint8_t active_count = 0U;
    uint8_t sensor_index;

    for (sensor_index = 0U; sensor_index < 8U; sensor_index++) {
        active_count += sensor_is_active[sensor_index];
    }
    return active_count;
}

static uint8_t count_mask_bits(uint8_t sensor_mask)
{
    uint8_t active_count = 0U;

    while (sensor_mask != 0U) {
        active_count += sensor_mask & 1U;
        sensor_mask >>= 1U;
    }
    return active_count;
}

static bool selected_mode_supports_distance_setting(void)
{
    return selected_mode == COMPETITION_MODE_TASK2_FAST_LAP ||
        selected_mode == COMPETITION_MODE_TASK4_GYROSCOPE_STRAIGHT ||
        selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP ||
        selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE;
}

static float selected_mode_adjustable_distance_mm(void)
{
    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        return task2_fixed_distance_stop_mm;
    }
    if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        return task5_6_fixed_distance_stop_mm;
    }
    if (selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE) {
        return task7_base_pwm;
    }
    return task4_target_distance_mm;
}

/*
 * 任务2高速通过5cm横线时有效组合可能只持续一个主循环周期，因此扩大累计行程并单次锁存；
 * 任务5/6速度较低，继续保留两周期确认，避免末弧普通循迹波动被误判成终点横线。
 */
static float selected_mode_finish_exit_confirm_distance_mm(void)
{
    return selected_mode == COMPETITION_MODE_TASK2_FAST_LAP ?
        COMPETITION_TASK2_FINISH_EXIT_CONFIRM_DISTANCE_MM :
        COMPETITION_FINISH_EXIT_CONFIRM_DISTANCE_MM;
}

static float selected_mode_finish_accumulation_distance_mm(void)
{
    return selected_mode == COMPETITION_MODE_TASK2_FAST_LAP ?
        COMPETITION_TASK2_FINISH_SENSOR_ACCUMULATION_DISTANCE_MM :
        COMPETITION_FINISH_SENSOR_ACCUMULATION_DISTANCE_MM;
}

static uint8_t selected_mode_finish_confirmation_steps(void)
{
    return selected_mode == COMPETITION_MODE_TASK2_FAST_LAP ?
        COMPETITION_TASK2_FINISH_LINE_CONFIRMATION_STEPS :
        COMPETITION_FINISH_LINE_CONFIRMATION_STEPS;
}

/*
 * 顺时针末弧会先让R2～R4压线，出弯即进入终点横线。车身带角度通过5cm横线时，
 * 左右探头可能先后命中而不是同一周期同时命中，因此在100mm内累计探头掩码。
 * 只有左侧L4～L2和右侧R2～R4都被扫到，才认为横线横跨了正常纵向轨迹。
 */
static bool finish_line_is_detected(float distance_mm)
{
    uint8_t sensor_is_active[8];
    uint8_t active_sensor_mask = 0U;
    uint8_t active_count;
    bool left_cross_sensor_was_seen;
    bool right_cross_sensor_was_seen;
    float exit_confirm_distance_mm =
        selected_mode_finish_exit_confirm_distance_mm();
    float accumulation_distance_mm =
        selected_mode_finish_accumulation_distance_mm();
    uint8_t required_confirmation_steps =
        selected_mode_finish_confirmation_steps();

    LineSensor_Read(sensor_is_active);
    active_count = count_active_sensors(sensor_is_active);
    for (uint8_t sensor_index = 0U; sensor_index < 8U; sensor_index++) {
        if (sensor_is_active[sensor_index] != 0U) {
            active_sensor_mask |= (uint8_t)(1U << sensor_index);
        }
    }

    if ((active_sensor_mask &
        COMPETITION_FINISH_CLOCKWISE_ARC_SENSOR_MASK) != 0U) {
        finish_clockwise_arc_was_seen = true;
    }

    if (distance_mm - finish_accumulation_start_distance_mm >=
        accumulation_distance_mm) {
        finish_accumulation_start_distance_mm = distance_mm;
        finish_accumulated_sensor_mask = 0U;
    }
    finish_accumulated_sensor_mask |= active_sensor_mask;
    left_cross_sensor_was_seen = (finish_accumulated_sensor_mask &
        COMPETITION_FINISH_LEFT_CROSS_SENSOR_MASK) != 0U;
    right_cross_sensor_was_seen = (finish_accumulated_sensor_mask &
        COMPETITION_FINISH_RIGHT_CROSS_SENSOR_MASK) != 0U;

    if (distance_mm < exit_confirm_distance_mm ||
        active_count == 0U || !finish_clockwise_arc_was_seen ||
        !left_cross_sensor_was_seen || !right_cross_sensor_was_seen ||
        count_mask_bits(finish_accumulated_sensor_mask) <
            COMPETITION_FINISH_LINE_MINIMUM_ACTIVE_SENSORS) {
        finish_line_confirmation_steps = 0U;
        return false;
    }

    if (finish_line_confirmation_steps < required_confirmation_steps) {
        finish_line_confirmation_steps++;
    }
    return finish_line_confirmation_steps >= required_confirmation_steps;
}

static bool selected_mode_is_lap(void)
{
    return selected_mode == COMPETITION_MODE_TASK2_FAST_LAP ||
        selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP;
}

static bool selected_mode_uses_smooth_line_tracking(void)
{
    return selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP ||
        selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE;
}

static uint8_t selected_mode_inertia_task_id(void)
{
    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        return INERTIA_TELEMETRY_TASK2;
    }
    if (selected_mode == COMPETITION_MODE_TASK4_GYROSCOPE_STRAIGHT) {
        return INERTIA_TELEMETRY_TASK4;
    }
    if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        return INERTIA_TELEMETRY_TASK5_6;
    }
    if (selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE) {
        /* 任务7沿用任务5/6的循迹和控球端惯性前馈协议。 */
        return INERTIA_TELEMETRY_TASK5_6;
    }
    return INERTIA_TELEMETRY_TASK_NONE;
}

/*
 * 视觉控球端需要区分正常运动、主动软停和已经停车三个阶段：软停期间继续做反惯性前馈，
 * 完全停车后则利用SETTLING状态逐步撤销补偿，只保留视觉位置闭环，避免补偿残留推走小球。
 */
static uint8_t current_inertia_motion_state(void)
{
    if (competition_state == COMPETITION_STATE_RUNNING) {
        if (task4_soft_stop_is_active || task5_6_soft_stop_is_active) {
            return INERTIA_TELEMETRY_STATE_STOPPING;
        }
        return INERTIA_TELEMETRY_STATE_RUNNING;
    }
    if (competition_state == COMPETITION_STATE_READY ||
        competition_state == COMPETITION_STATE_DISTANCE_SETTING ||
        competition_state == COMPETITION_STATE_GYROSCOPE_STATUS) {
        return INERTIA_TELEMETRY_STATE_STANDBY;
    }
    return INERTIA_TELEMETRY_STATE_SETTLING;
}

static float selected_mode_base_pwm(void)
{
    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        return COMPETITION_TASK2_BASE_PWM;
    }
    if (selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE) {
        return task7_base_pwm;
    }
    return COMPETITION_LAP_STEADY_BASE_PWM;
}

static float selected_mode_curve_base_pwm(void)
{
    return selected_mode == COMPETITION_MODE_TASK2_FAST_LAP ?
        COMPETITION_TASK2_CURVE_BASE_PWM : LINE_TRACKING_CURVE_BASE_PWM;
}

static float selected_mode_sharp_turn_base_pwm(void)
{
    return selected_mode == COMPETITION_MODE_TASK2_FAST_LAP ?
        COMPETITION_TASK2_SHARP_TURN_BASE_PWM :
        LINE_TRACKING_SHARP_TURN_BASE_PWM;
}

static float selected_mode_curve_hold_pwm(void)
{
    return selected_mode == COMPETITION_MODE_TASK2_FAST_LAP ?
        COMPETITION_TASK2_CURVE_HOLD_PWM :
        LINE_TRACKING_DENSE_CURVE_HOLD_PWM;
}

static float selected_mode_approach_pwm(void)
{
    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        return COMPETITION_TASK2_APPROACH_BASE_PWM;
    }
    if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        return COMPETITION_TASK5_6_FINISH_BASE_PWM;
    }
    return LINE_TRACKING_APPROACH_BASE_PWM;
}

static float calculate_minimum_jerk_progress(float normalized_progress);
static void start_task5_6_finish_soft_stop(
    CompetitionState finish_state);

/*
 * 任务5/6最后650mm使用与任务4相同的五次S曲线从巡航PWM降到横线识别速度。
 * 曲线起点和终点的斜率、曲率都为零，避免进入减速段和接近横线时突然推拉小球。
 */
static float calculate_task5_6_finish_base_pwm(float distance_mm)
{
    float deceleration_start_mm = COMPETITION_LAP_THEORETICAL_LENGTH_MM -
        COMPETITION_TASK5_6_DECELERATION_DISTANCE_MM;
    float cruise_pwm = COMPETITION_LAP_STEADY_BASE_PWM;
    float finish_pwm = COMPETITION_TASK5_6_FINISH_BASE_PWM;

    if (distance_mm <= deceleration_start_mm) return cruise_pwm;
    if (distance_mm >= COMPETITION_LAP_THEORETICAL_LENGTH_MM) return finish_pwm;

    float deceleration_progress = (distance_mm - deceleration_start_mm) /
        COMPETITION_TASK5_6_DECELERATION_DISTANCE_MM;
    return cruise_pwm - (cruise_pwm - finish_pwm) *
        calculate_minimum_jerk_progress(deceleration_progress);
}

/* 任务5/6基础PWM只负责巡航和终点前距离减速，起步改由运动层整体缩放最终左右轮输出。 */
static float calculate_task5_6_requested_base_pwm(float distance_mm)
{
    return calculate_task5_6_finish_base_pwm(distance_mm);
}

/*
 * 五次最小冲击曲线6t^5-15t^4+10t^3在t=0和t=1处的一阶、二阶导数都为零。
 * 这使PWM变化率以及变化率的改变都能平滑接入起步、巡航和末速，避免半余弦曲线端点
 * 仍存在的加速度突变激起小球晃动；仅使用乘加运算也适合Cortex-M0+软浮点执行。
 */
static float calculate_minimum_jerk_progress(float normalized_progress)
{
    if (normalized_progress <= 0.0f) return 0.0f;
    if (normalized_progress >= 1.0f) return 1.0f;

    return normalized_progress * normalized_progress * normalized_progress *
        (10.0f + normalized_progress *
            (-15.0f + 6.0f * normalized_progress));
}

/*
 * 起步使用p(t)，停车使用1-p(t)，两条五次S曲线在数学上互为时间镜像。
 * 起步持续2400ms而停车持续1500ms，是为匹配200和124两种PWM幅值，使实际PWM变化率也接近对称。
 */
static void update_task5_6_startup_scale(void)
{
    uint32_t elapsed_milliseconds =
        SystemTime_GetMilliseconds() - run_start_milliseconds;

    if (elapsed_milliseconds >= COMPETITION_TASK5_6_START_RAMP_DURATION_MS) {
        Motion_SetLineTrackingStartupScale(1.0f);
        return;
    }

    Motion_SetLineTrackingStartupScale(calculate_minimum_jerk_progress(
        (float)elapsed_milliseconds /
            (float)COMPETITION_TASK5_6_START_RAMP_DURATION_MS));
}

/* 任务7只需要起步缓加，进入巡航后不再按距离或时间自动降速。 */
static void update_task7_startup_scale(void)
{
    uint32_t elapsed_milliseconds =
        SystemTime_GetMilliseconds() - run_start_milliseconds;

    if (elapsed_milliseconds >= COMPETITION_TASK7_START_RAMP_DURATION_MS) {
        Motion_SetLineTrackingStartupScale(1.0f);
        return;
    }

    Motion_SetLineTrackingStartupScale(calculate_minimum_jerk_progress(
        (float)elapsed_milliseconds /
            (float)COMPETITION_TASK7_START_RAMP_DURATION_MS));
}

/*
 * 任务4先用五次S曲线从起步PWM升到巡航PWM，中段保持巡航速度，最后550mm按同一曲线降速。
 * 曲线两端的PWM斜率和曲率都为零，可让长车身与小球在三个分段连接处平稳过渡。
 */
static float calculate_task4_base_pwm(float distance_mm)
{
    float progress;
    float deceleration_start_mm = task4_target_distance_mm -
        COMPETITION_TASK4_GYRO_DECELERATION_DISTANCE_MM;

    if (distance_mm < COMPETITION_TASK4_GYRO_ACCELERATION_DISTANCE_MM) {
        progress = distance_mm /
            COMPETITION_TASK4_GYRO_ACCELERATION_DISTANCE_MM;
        return COMPETITION_TASK4_GYRO_START_PWM +
            (COMPETITION_TASK4_GYRO_CRUISE_PWM -
                COMPETITION_TASK4_GYRO_START_PWM) *
            calculate_minimum_jerk_progress(progress);
    }
    if (distance_mm < deceleration_start_mm) {
        return COMPETITION_TASK4_GYRO_CRUISE_PWM;
    }
    if (distance_mm >= task4_target_distance_mm) {
        return COMPETITION_TASK4_GYRO_FINISH_PWM;
    }

    progress = (task4_target_distance_mm - distance_mm) /
        (task4_target_distance_mm - deceleration_start_mm);
    return COMPETITION_TASK4_GYRO_FINISH_PWM +
        (COMPETITION_TASK4_GYRO_CRUISE_PWM -
            COMPETITION_TASK4_GYRO_FINISH_PWM) *
        calculate_minimum_jerk_progress(progress);
}

/*
 * 每100ms用编码器距离增量估算任务4车速，并对新值做轻度低通。
 * 末端减速段速度变化较慢，保留少量历史值可抑制单个编码器计数带来的停车起点抖动。
 */
static void update_task4_speed_estimate(float distance_mm,
    uint32_t current_milliseconds)
{
    uint32_t sample_interval_milliseconds = (uint32_t)(current_milliseconds -
        task4_speed_sample_milliseconds);

    if (sample_interval_milliseconds <
        COMPETITION_TASK4_GYRO_SPEED_SAMPLE_INTERVAL_MS) {
        return;
    }

    float measured_speed_mm_per_second =
        (distance_mm - task4_speed_sample_distance_mm) * 1000.0f /
        (float)sample_interval_milliseconds;
    if (measured_speed_mm_per_second < 0.0f) {
        measured_speed_mm_per_second = 0.0f;
    }
    if (task4_estimated_speed_mm_per_second <= 0.0f) {
        task4_estimated_speed_mm_per_second = measured_speed_mm_per_second;
    } else {
        task4_estimated_speed_mm_per_second =
            task4_estimated_speed_mm_per_second * 0.3f +
            measured_speed_mm_per_second * 0.7f;
    }

    task4_speed_sample_milliseconds = current_milliseconds;
    task4_speed_sample_distance_mm = distance_mm;
}

/*
 * 线性降速时的平均速度约为初速的一半，因此用当前速度乘软停时间的一半估算提前量。
 * 电机进入低PWM死区后实际行程会小于理想三角速度模型，故减去标定补偿并进行上下限保护。
 */
static float calculate_task4_soft_stop_distance_mm(void)
{
    float soft_stop_distance_mm = task4_estimated_speed_mm_per_second *
        (float)COMPETITION_TASK4_GYRO_SOFT_STOP_DURATION_MS / 2000.0f -
        COMPETITION_TASK4_GYRO_SOFT_STOP_DISTANCE_TRIM_MM;

    if (soft_stop_distance_mm <
        COMPETITION_TASK4_GYRO_SOFT_STOP_DISTANCE_MIN_MM) {
        return COMPETITION_TASK4_GYRO_SOFT_STOP_DISTANCE_MIN_MM;
    }
    if (soft_stop_distance_mm >
        COMPETITION_TASK4_GYRO_SOFT_STOP_DISTANCE_MAX_MM) {
        return COMPETITION_TASK4_GYRO_SOFT_STOP_DISTANCE_MAX_MM;
    }
    return soft_stop_distance_mm;
}

static uint32_t selected_mode_timeout_milliseconds(void)
{
    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        return COMPETITION_TASK2_TIMEOUT_MS;
    }
    if (selected_mode == COMPETITION_MODE_TASK4_GYROSCOPE_STRAIGHT) {
        return COMPETITION_TASK4_TIMEOUT_MS;
    }
    return COMPETITION_TASK5_6_TIMEOUT_MS;
}

static void stop_with_state(CompetitionState new_state)
{
    /* 同一停止接口退出循迹和任务4陀螺仪直行，并立即令TB6612制动。 */
    Motion_StopLineTracking();
    competition_state = new_state;
    finish_milliseconds = SystemTime_GetMilliseconds();
}

static void start_selected_mode(void)
{
    Motor_Stop();
    if (selected_mode == COMPETITION_MODE_GYROSCOPE_STATUS) {
        Motion_StopLineTracking();
        competition_state = COMPETITION_STATE_GYROSCOPE_STATUS;
        return;
    }

    Encoder_Reset();
    finish_line_confirmation_steps = 0U;
    finish_accumulated_sensor_mask = 0U;
    finish_accumulation_start_distance_mm = 0.0f;
    finish_clockwise_arc_was_seen = false;
    run_start_milliseconds = SystemTime_GetMilliseconds();
    finish_milliseconds = run_start_milliseconds;
    task4_speed_sample_milliseconds = run_start_milliseconds;
    task4_soft_stop_start_milliseconds = run_start_milliseconds;
    task4_speed_sample_distance_mm = 0.0f;
    task4_estimated_speed_mm_per_second = 0.0f;
    task4_soft_stop_start_pwm = 0.0f;
    task4_soft_stop_is_active = false;
    task5_6_soft_stop_start_milliseconds = run_start_milliseconds;
    task5_6_soft_stop_duration_milliseconds = 0U;
    task5_6_soft_stop_is_active = false;
    task5_6_soft_stop_finish_state = COMPETITION_STATE_FINISHED;
    competition_state = COMPETITION_STATE_RUNNING;
    if (selected_mode == COMPETITION_MODE_TASK4_GYROSCOPE_STRAIGHT) {
        Motion_StartGyroscopeStraight(calculate_task4_base_pwm(0.0f));
    } else {
        Motion_StartLineTracking(selected_mode_base_pwm(),
            selected_mode_curve_base_pwm(),
            selected_mode_sharp_turn_base_pwm(),
            selected_mode_curve_hold_pwm(),
            selected_mode == COMPETITION_MODE_TASK2_FAST_LAP,
            selected_mode_uses_smooth_line_tracking());
    }
}

static uint32_t get_elapsed_milliseconds(void)
{
    uint32_t current_or_finish_milliseconds;
    current_or_finish_milliseconds =
        competition_state == COMPETITION_STATE_RUNNING ?
        SystemTime_GetMilliseconds() : finish_milliseconds;
    return current_or_finish_milliseconds - run_start_milliseconds;
}

static void draw_elapsed_time(uint8_t x_coordinate, uint8_t y_coordinate)
{
    uint32_t elapsed_milliseconds = get_elapsed_milliseconds();
    OLED_DrawUnsignedInteger(x_coordinate, y_coordinate,
        elapsed_milliseconds / 1000U);
    OLED_DrawCharacter((uint8_t)(x_coordinate + 24U), y_coordinate, '.');
    OLED_DrawUnsignedInteger((uint8_t)(x_coordinate + 30U), y_coordinate,
        (elapsed_milliseconds % 1000U) / 100U);
    OLED_DrawCharacter((uint8_t)(x_coordinate + 38U), y_coordinate, 's');
}

static void draw_mode_name(void)
{
    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        OLED_DrawString(0U, 0U, "T2 FAST LAP");
    } else if (selected_mode == COMPETITION_MODE_TASK4_GYROSCOPE_STRAIGHT) {
        OLED_DrawString(0U, 0U, "T4 A TO B");
    } else if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        OLED_DrawString(0U, 0U, "T5/6 STEADY");
    } else if (selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE) {
        OLED_DrawString(0U, 0U, "T7 INFINITE");
    } else {
        OLED_DrawString(0U, 0U, "GYRO STATUS");
    }
}

static void draw_state_name(void)
{
    switch (competition_state) {
        case COMPETITION_STATE_READY:
            OLED_DrawString(0U, 16U, "READY PRESS OK");
            break;
        case COMPETITION_STATE_RUNNING:
            OLED_DrawString(0U, 16U, "RUNNING");
            break;
        case COMPETITION_STATE_FINISHED:
            OLED_DrawString(0U, 16U, "FINISHED");
            break;
        case COMPETITION_STATE_STOPPED_BY_USER:
            OLED_DrawString(0U, 16U, "USER STOP");
            break;
        case COMPETITION_STATE_LINE_LOST:
            OLED_DrawString(0U, 16U, "LINE LOST");
            break;
        case COMPETITION_STATE_TIMEOUT:
            OLED_DrawString(0U, 16U, "TIMEOUT");
            break;
        case COMPETITION_STATE_DISTANCE_FAILSAFE:
            OLED_DrawString(0U, 16U, "DIST STOP");
            break;
        case COMPETITION_STATE_GYROSCOPE_UNAVAILABLE:
            OLED_DrawString(0U, 16U, "GYRO ERROR");
            break;
        default:
            OLED_DrawString(0U, 16U, "READY PRESS OK");
            break;
    }
}

/*
 * 诊断页同时显示接收状态、软件零点、绝对航向、归零后偏移、Z轴角速度和帧计数。
 * 字节数增长但有效帧不增长，通常表示接线已通但波特率或G356输出协议不匹配。
 */
static void draw_gyroscope_status_screen(void)
{
    OLED_ClearDisplayBuffer();
    OLED_DrawString(0U, 0U,
        G356_HasValidTelemetryData() ? "G:OK" : "G:NO");
    OLED_DrawString(30U, 0U,
        G356_HasYawZeroReference() ? "Z:OK" : "Z:NO");
    OLED_DrawString(60U, 0U, "OK=ZERO");

    OLED_DrawString(0U, 16U, "Y:");
    OLED_DrawSignedFloat(12U, 16U, G356_GetAbsoluteYawDegrees(), 1U);
    OLED_DrawString(66U, 16U, "O:");
    OLED_DrawSignedFloat(78U, 16U, G356_GetYawOffsetDegrees(), 1U);

    OLED_DrawString(0U, 32U, "W:");
    OLED_DrawSignedFloat(12U, 32U,
        G356_GetYawRateDegreesPerSecond(), 1U);
    OLED_DrawString(66U, 32U, "BK=EXIT");

    OLED_DrawString(0U, 48U, "B:");
    OLED_DrawUnsignedInteger(12U, 48U, G356_GetReceivedByteCount());
    OLED_DrawString(54U, 48U, "F:");
    OLED_DrawUnsignedInteger(66U, 48U, G356_GetValidFrameCount());
    OLED_DrawString(96U, 48U, "E:");
    OLED_DrawUnsignedInteger(108U, 48U, G356_GetInvalidFrameCount());
    OLED_RefreshDisplay();
}

/* 距离或速度设置只改RAM运行值，适合现场连续标定；重新上电后恢复配置文件默认值。 */
static void draw_distance_setting_screen(void)
{
    const char *setting_title;
    const char *value_label;
    const char *unit_label;
    const char *step_label;

    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        setting_title = "T2 DIST SET";
        value_label = "DIST:";
        unit_label = "mm";
        step_label = "UP/DN:10mm";
    } else if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        setting_title = "T5/6 SAFE SET";
        value_label = "DIST:";
        unit_label = "mm";
        step_label = "UP/DN:10mm";
    } else if (selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE) {
        setting_title = "T7 SPEED SET";
        value_label = "PWM:";
        unit_label = "PWM";
        step_label = "UP/DN:10PWM";
    } else {
        setting_title = "T4 DIST SET";
        value_label = "DIST:";
        unit_label = "mm";
        step_label = "UP/DN:10mm";
    }

    OLED_ClearDisplayBuffer();
    OLED_DrawString(0U, 0U, setting_title);
    OLED_DrawString(0U, 16U, value_label);
    OLED_DrawUnsignedInteger(30U, 16U,
        (uint32_t)(selected_mode_adjustable_distance_mm() + 0.5f));
    OLED_DrawString(60U, 16U, unit_label);
    OLED_DrawString(0U, 32U, step_label);
    OLED_DrawString(0U, 48U, "OK=SAVE BK=CANCEL");
    OLED_RefreshDisplay();
}

static void refresh_display(void)
{
    bool run_is_active = competition_state == COMPETITION_STATE_RUNNING;

    if (competition_state == COMPETITION_STATE_GYROSCOPE_STATUS) {
        draw_gyroscope_status_screen();
        return;
    }
    if (competition_state == COMPETITION_STATE_DISTANCE_SETTING) {
        draw_distance_setting_screen();
        return;
    }

    OLED_ClearDisplayBuffer();
    draw_mode_name();
    draw_state_name();
    if (!run_is_active && competition_state == COMPETITION_STATE_READY &&
        selected_mode_supports_distance_setting()) {
        OLED_DrawString(0U, 32U,
            selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE ?
                "PWM:" : "SET:");
        OLED_DrawUnsignedInteger(24U, 32U,
            (uint32_t)(selected_mode_adjustable_distance_mm() + 0.5f));
        OLED_DrawString(54U, 32U,
            selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE ?
                "PWM" : "mm");
        OLED_DrawString(0U, 48U, "OK=RUN BK=EDIT");
    } else {
        OLED_DrawString(0U, 32U, "T:");
        draw_elapsed_time(12U, 32U);
        OLED_DrawString(64U, 32U,
            Motion_IsStraightGyroscopeAssistActive() ? "GYRO" : "LINE");
        OLED_DrawString(0U, 48U, "D:");
        OLED_DrawUnsignedInteger(12U, 48U,
            (uint32_t)Encoder_GetAverageDistanceMm());
        OLED_DrawString(48U, 48U, "mm");
        OLED_DrawString(78U, 48U,
            G356_HasValidTelemetryData() ? "G:OK" : "G:NO");
    }
    if (run_is_active) {
        OLED_RefreshNextPage();
    } else {
        OLED_RefreshDisplay();
    }
}

static void process_task4_run(float distance_mm)
{
    MotionGyroscopeStraightStatus gyroscope_status;
    uint32_t current_milliseconds = SystemTime_GetMilliseconds();
    float target_base_pwm;

    if (SystemTime_HasElapsed(run_start_milliseconds,
        COMPETITION_TASK4_TIMEOUT_MS)) {
        stop_with_state(COMPETITION_STATE_TIMEOUT);
        return;
    }

    update_task4_speed_estimate(distance_mm, current_milliseconds);
    if (!task4_soft_stop_is_active &&
        distance_mm >= task4_target_distance_mm -
            calculate_task4_soft_stop_distance_mm()) {
        /*
         * 在预计剩余行程等于软停行程时锁存当前PWM，后续只按时间线性下降。
         * 这样编码器的细小跳变不会反复改变减速斜率，小球受到的纵向冲击更稳定。
         */
        task4_soft_stop_is_active = true;
        task4_soft_stop_start_milliseconds = current_milliseconds;
        task4_soft_stop_start_pwm = calculate_task4_base_pwm(distance_mm);
    }

    if (task4_soft_stop_is_active) {
        uint32_t soft_stop_elapsed_milliseconds =
            (uint32_t)(current_milliseconds -
                task4_soft_stop_start_milliseconds);
        if (soft_stop_elapsed_milliseconds >=
            COMPETITION_TASK4_GYRO_SOFT_STOP_DURATION_MS) {
            Motion_SetGyroscopeStraightBasePwm(0.0f);
            stop_with_state(COMPETITION_STATE_FINISHED);
            return;
        }
        target_base_pwm = task4_soft_stop_start_pwm *
            (float)(COMPETITION_TASK4_GYRO_SOFT_STOP_DURATION_MS -
                soft_stop_elapsed_milliseconds) /
            (float)COMPETITION_TASK4_GYRO_SOFT_STOP_DURATION_MS;
    } else {
        target_base_pwm = calculate_task4_base_pwm(distance_mm);
    }

    Motion_SetGyroscopeStraightBasePwm(target_base_pwm);
    gyroscope_status = Motion_RunGyroscopeStraightControlStep();
    if (gyroscope_status ==
        MOTION_GYROSCOPE_STRAIGHT_STATUS_GYRO_UNAVAILABLE) {
        stop_with_state(COMPETITION_STATE_GYROSCOPE_UNAVAILABLE);
    }
}

static void process_lap_run(float distance_mm)
{
    if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        Motion_SetLineTrackingBasePwm(
            calculate_task5_6_requested_base_pwm(distance_mm));
    } else if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP &&
        distance_mm >= COMPETITION_LAP_THEORETICAL_LENGTH_MM -
        COMPETITION_FINISH_APPROACH_DISTANCE_MM) {
        Motion_SetLineTrackingBasePwm(selected_mode_approach_pwm());
    }

    /*
     * 任务2的定长停车放在横线识别、循迹PID和超时判断之后执行。
     * 正常情况下先由黑横线停车；只有漏识别横线时才在可调距离处作为完成兜底。
     */
    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP &&
        distance_mm >= task2_fixed_distance_stop_mm) {
        stop_with_state(COMPETITION_STATE_FINISHED);
    } else if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP &&
        distance_mm >= task5_6_fixed_distance_stop_mm) {
        /*
         * 黑横线判断已在本周期更早执行；走到这里说明横线尚未确认。
         * 定长保险只复用原有软停入口，不改变起步曲线、停车曲线、停车时长或终点PWM。
         */
        start_task5_6_finish_soft_stop(
            COMPETITION_STATE_DISTANCE_FAILSAFE);
    }
}

static void adjust_selected_mode_distance(int8_t adjustment_direction)
{
    float adjustment_mm = (float)adjustment_direction *
        COMPETITION_DISTANCE_SETTING_STEP_MM;

    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        task2_fixed_distance_stop_mm += adjustment_mm;
        if (task2_fixed_distance_stop_mm <
            COMPETITION_TASK2_DISTANCE_SETTING_MINIMUM_MM) {
            task2_fixed_distance_stop_mm =
                COMPETITION_TASK2_DISTANCE_SETTING_MINIMUM_MM;
        } else if (task2_fixed_distance_stop_mm >
            COMPETITION_TASK2_DISTANCE_SETTING_MAXIMUM_MM) {
            task2_fixed_distance_stop_mm =
                COMPETITION_TASK2_DISTANCE_SETTING_MAXIMUM_MM;
        }
        return;
    }

    if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        task5_6_fixed_distance_stop_mm += adjustment_mm;
        if (task5_6_fixed_distance_stop_mm <
            COMPETITION_TASK5_6_DISTANCE_SETTING_MINIMUM_MM) {
            task5_6_fixed_distance_stop_mm =
                COMPETITION_TASK5_6_DISTANCE_SETTING_MINIMUM_MM;
        } else if (task5_6_fixed_distance_stop_mm >
            COMPETITION_TASK5_6_DISTANCE_SETTING_MAXIMUM_MM) {
            task5_6_fixed_distance_stop_mm =
                COMPETITION_TASK5_6_DISTANCE_SETTING_MAXIMUM_MM;
        }
        return;
    }

    if (selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE) {
        task7_base_pwm += (float)adjustment_direction *
            COMPETITION_TASK7_SPEED_SETTING_STEP_PWM;
        if (task7_base_pwm < COMPETITION_TASK7_SPEED_SETTING_MINIMUM_PWM) {
            task7_base_pwm = COMPETITION_TASK7_SPEED_SETTING_MINIMUM_PWM;
        } else if (task7_base_pwm > COMPETITION_TASK7_SPEED_SETTING_MAXIMUM_PWM) {
            task7_base_pwm = COMPETITION_TASK7_SPEED_SETTING_MAXIMUM_PWM;
        }
        return;
    }

    task4_target_distance_mm += adjustment_mm;
    if (task4_target_distance_mm <
        COMPETITION_TASK4_DISTANCE_SETTING_MINIMUM_MM) {
        task4_target_distance_mm =
            COMPETITION_TASK4_DISTANCE_SETTING_MINIMUM_MM;
    } else if (task4_target_distance_mm >
        COMPETITION_TASK4_DISTANCE_SETTING_MAXIMUM_MM) {
        task4_target_distance_mm =
            COMPETITION_TASK4_DISTANCE_SETTING_MAXIMUM_MM;
    }
}

static void restore_distance_setting_original_value(void)
{
    if (selected_mode == COMPETITION_MODE_TASK2_FAST_LAP) {
        task2_fixed_distance_stop_mm = distance_setting_original_mm;
    } else if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        task5_6_fixed_distance_stop_mm = distance_setting_original_mm;
    } else if (selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE) {
        task7_base_pwm = distance_setting_original_mm;
    } else {
        task4_target_distance_mm = distance_setting_original_mm;
    }
}

/*
 * 横线或定长保险触发后都进入同一条五次S曲线软停车，并将软停时间计入任务总用时。
 * 两种触发方式只改变最终显示状态，不改变停车PWM、曲线或时长；若触发较晚，则用30秒前
 * 的剩余时间压缩软停，确保不会因追求平滑而超时。
 */
static void start_task5_6_finish_soft_stop(
    CompetitionState finish_state)
{
    uint32_t current_milliseconds = SystemTime_GetMilliseconds();
    uint32_t elapsed_milliseconds = current_milliseconds -
        run_start_milliseconds;
    uint32_t available_milliseconds = elapsed_milliseconds <
        COMPETITION_TASK5_6_TIMEOUT_MS ?
        COMPETITION_TASK5_6_TIMEOUT_MS - elapsed_milliseconds : 0U;

    if (available_milliseconds >
        COMPETITION_TASK5_6_FINISH_TIME_MARGIN_MS) {
        available_milliseconds -=
            COMPETITION_TASK5_6_FINISH_TIME_MARGIN_MS;
    } else {
        available_milliseconds = 0U;
    }
    task5_6_soft_stop_duration_milliseconds =
        available_milliseconds <
            COMPETITION_TASK5_6_FINISH_SOFT_STOP_DURATION_MS ?
        available_milliseconds :
            COMPETITION_TASK5_6_FINISH_SOFT_STOP_DURATION_MS;
    task5_6_soft_stop_start_milliseconds = current_milliseconds;
    task5_6_soft_stop_is_active = true;
    task5_6_soft_stop_finish_state = finish_state;
    Motion_StartLineTrackingSoftStop();
    if (task5_6_soft_stop_duration_milliseconds == 0U) {
        /* 横线已经确认但30秒余量不足时，立即停车并按正常完成记录，绝不拖过限时。 */
        Motion_RunLineTrackingSoftStop(0.0f);
        task5_6_soft_stop_is_active = false;
        stop_with_state(task5_6_soft_stop_finish_state);
    }
}

static void process_task5_6_finish_soft_stop(void)
{
    uint32_t current_milliseconds = SystemTime_GetMilliseconds();
    uint32_t total_elapsed_milliseconds = current_milliseconds -
        run_start_milliseconds;
    uint32_t elapsed_milliseconds = current_milliseconds -
        task5_6_soft_stop_start_milliseconds;

    if (total_elapsed_milliseconds >=
            COMPETITION_TASK5_6_TIMEOUT_MS ||
        task5_6_soft_stop_duration_milliseconds == 0U ||
        elapsed_milliseconds >= task5_6_soft_stop_duration_milliseconds) {
        Motion_RunLineTrackingSoftStop(0.0f);
        task5_6_soft_stop_is_active = false;
        stop_with_state(task5_6_soft_stop_finish_state);
        return;
    }

    float stop_progress = (float)elapsed_milliseconds /
        (float)task5_6_soft_stop_duration_milliseconds;
    Motion_RunLineTrackingSoftStop(
        1.0f - calculate_minimum_jerk_progress(stop_progress));
}

/*
 * 横线停车必须先于本周期的循迹PID执行，否则确认终点后电机仍会收到一次新的驱动命令。
 * 仅在5832mm基准终点前的窗口内启用，避免任务2在前半圈圆弧中误判停车。
 */
static bool handle_finish_line_if_detected(float distance_mm)
{
    float detection_start_mm = COMPETITION_LAP_THEORETICAL_LENGTH_MM -
        COMPETITION_FINISH_DETECTION_WINDOW_MM;

    if (!selected_mode_is_lap() ||
        distance_mm < detection_start_mm) {
        return false;
    }
    /* 两种单圈任务都先等待真实横线；可调定长兜底在本周期PID之后处理。 */
    if (!finish_line_is_detected(distance_mm)) return false;

    if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        start_task5_6_finish_soft_stop(COMPETITION_STATE_FINISHED);
        return true;
    }
    stop_with_state(COMPETITION_STATE_FINISHED);
    return true;
}

static void process_running_state(void)
{
    MotionLineTrackingStatus tracking_status;
    float distance_mm;

    distance_mm = Encoder_GetAverageDistanceMm();

    if (selected_mode == COMPETITION_MODE_TASK4_GYROSCOPE_STRAIGHT) {
        process_task4_run(distance_mm);
        return;
    }

    if (task5_6_soft_stop_is_active) {
        process_task5_6_finish_soft_stop();
        return;
    }

    if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP &&
        SystemTime_HasElapsed(run_start_milliseconds,
            COMPETITION_TASK5_6_TIMEOUT_MS)) {
        /* 未识别横线时严格以30秒为硬上限，避免下一主循环周期才停车造成超时。 */
        stop_with_state(COMPETITION_STATE_TIMEOUT);
        return;
    }

    /* 返回A点横线具有最高停车优先级，确认后不再运行PID、超时或距离保护。 */
    if (handle_finish_line_if_detected(distance_mm)) return;

    if (selected_mode == COMPETITION_MODE_TASK5_6_STEADY_LAP) {
        update_task5_6_startup_scale();
    } else if (selected_mode == COMPETITION_MODE_TASK7_INFINITE_LINE) {
        update_task7_startup_scale();
    }

    tracking_status = Motion_RunLineTrackingControlStep();
    if (tracking_status == MOTION_LINE_TRACKING_STATUS_STOPPED_LINE_LOST) {
        stop_with_state(COMPETITION_STATE_LINE_LOST);
        return;
    }

    if (selected_mode != COMPETITION_MODE_TASK7_INFINITE_LINE &&
        SystemTime_HasElapsed(run_start_milliseconds,
            selected_mode_timeout_milliseconds())) {
        stop_with_state(COMPETITION_STATE_TIMEOUT);
        return;
    }

    process_lap_run(distance_mm);
}

static void process_button_event(MenuButtonEvent button_event)
{
    bool run_is_active = competition_state == COMPETITION_STATE_RUNNING;

    if (competition_state == COMPETITION_STATE_DISTANCE_SETTING) {
        if (button_event == MENU_BUTTON_EVENT_UP) {
            adjust_selected_mode_distance(1);
        } else if (button_event == MENU_BUTTON_EVENT_DOWN) {
            adjust_selected_mode_distance(-1);
        } else if (button_event == MENU_BUTTON_EVENT_CONFIRM) {
            competition_state = COMPETITION_STATE_READY;
        } else if (button_event == MENU_BUTTON_EVENT_BACK) {
            restore_distance_setting_original_value();
            competition_state = COMPETITION_STATE_READY;
        }
        return;
    }

    if (competition_state == COMPETITION_STATE_GYROSCOPE_STATUS) {
        if (button_event == MENU_BUTTON_EVENT_CONFIRM) {
            /* 软件归零只记录当前Yaw，不向模块写配置，避免比赛中误改G356参数。 */
            G356_SetCurrentYawAsZeroReference();
        } else if (button_event == MENU_BUTTON_EVENT_BACK) {
            competition_state = COMPETITION_STATE_READY;
        }
        return;
    }

    if (run_is_active) {
        if (button_event == MENU_BUTTON_EVENT_BACK ||
            button_event == MENU_BUTTON_EVENT_CONFIRM) {
            stop_with_state(COMPETITION_STATE_STOPPED_BY_USER);
        }
        return;
    }

    if (button_event == MENU_BUTTON_EVENT_UP) {
        selected_mode = selected_mode == 0U ?
            (CompetitionMode)(COMPETITION_MODE_COUNT - 1U) :
            (CompetitionMode)(selected_mode - 1U);
        competition_state = COMPETITION_STATE_READY;
    } else if (button_event == MENU_BUTTON_EVENT_DOWN) {
        selected_mode = (CompetitionMode)((selected_mode + 1U) %
            COMPETITION_MODE_COUNT);
        competition_state = COMPETITION_STATE_READY;
    } else if (button_event == MENU_BUTTON_EVENT_CONFIRM) {
        start_selected_mode();
    } else if (button_event == MENU_BUTTON_EVENT_BACK &&
        selected_mode_supports_distance_setting()) {
        distance_setting_original_mm = selected_mode_adjustable_distance_mm();
        competition_state = COMPETITION_STATE_DISTANCE_SETTING;
    }
}

void CompetitionControl_Initialize(void)
{
    selected_mode = COMPETITION_MODE_TASK2_FAST_LAP;
    competition_state = COMPETITION_STATE_READY;
    run_start_milliseconds = SystemTime_GetMilliseconds();
    finish_milliseconds = run_start_milliseconds;
    finish_line_confirmation_steps = 0U;
    finish_accumulated_sensor_mask = 0U;
    finish_accumulation_start_distance_mm = 0.0f;
    finish_clockwise_arc_was_seen = false;
    task4_speed_sample_milliseconds = run_start_milliseconds;
    task4_soft_stop_start_milliseconds = run_start_milliseconds;
    task4_speed_sample_distance_mm = 0.0f;
    task4_estimated_speed_mm_per_second = 0.0f;
    task4_soft_stop_start_pwm = 0.0f;
    task4_soft_stop_is_active = false;
    task5_6_soft_stop_start_milliseconds = run_start_milliseconds;
    task5_6_soft_stop_duration_milliseconds = 0U;
    task5_6_soft_stop_is_active = false;
    task5_6_soft_stop_finish_state = COMPETITION_STATE_FINISHED;
    task2_fixed_distance_stop_mm =
        COMPETITION_TASK2_FIXED_DISTANCE_STOP_MM;
    task4_target_distance_mm =
        COMPETITION_TASK4_GYRO_TARGET_DISTANCE_MM;
    task5_6_fixed_distance_stop_mm =
        COMPETITION_TASK5_6_FIXED_DISTANCE_STOP_MM;
    task7_base_pwm = COMPETITION_TASK7_DEFAULT_BASE_PWM;
    distance_setting_original_mm = task2_fixed_distance_stop_mm;
    last_display_refresh_milliseconds = run_start_milliseconds -
        COMPETITION_DISPLAY_REFRESH_MS;
    Motor_Stop();
    InertiaTelemetry_Initialize();
    refresh_display();
}

void CompetitionControl_Process(void)
{
    MenuButtonEvent button_event = MenuButtons_GetPressedEvent();
    uint32_t current_milliseconds;

    process_button_event(button_event);
    if (competition_state == COMPETITION_STATE_RUNNING) {
        process_running_state();
    }

    InertiaTelemetry_Process(selected_mode_inertia_task_id(),
        current_inertia_motion_state());

    current_milliseconds = SystemTime_GetMilliseconds();
    if ((uint32_t)(current_milliseconds - last_display_refresh_milliseconds) >=
        COMPETITION_DISPLAY_REFRESH_MS) {
        last_display_refresh_milliseconds = current_milliseconds;
        /*
         * 软件SPI刷新会短时阻塞主循环。单圈接近终点后暂停发送，避免高速经过横线时漏采样；
         * SysTick计时仍在中断中独立运行，停车后全屏刷新会显示真实最终用时。
         */
        if (!(competition_state == COMPETITION_STATE_RUNNING &&
            selected_mode_is_lap() &&
            Encoder_GetAverageDistanceMm() >=
                COMPETITION_FINISH_DISPLAY_SUSPEND_DISTANCE_MM)) {
            refresh_display();
        }
    }
}
