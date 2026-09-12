/**
 * @file vehicle_config.h
 * @brief H题天猛星底盘统一参数配置，集中保存循迹、停车、计时和陀螺仪辅助参数。
 *
 * 调车时只修改本文件。距离、传感器安装偏移和制动距离必须按实车重新标定，
 * 避免把机械差异散落到控制代码中。
 */
#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

/* ==================== 电机、车轮与编码器 ==================== */
#define MOTOR_PWM_MAXIMUM_DUTY_VALUE                    1000.0f
#define DRIVE_WHEEL_CIRCUMFERENCE_MILLIMETERS            204.0f
#define MOTOR_ENCODER_COUNTS_PER_WHEEL_REVOLUTION        730.0f

/* ==================== 感维八路灰度数字循迹 ==================== */
#define LINE_TRACKING_ERROR_DIRECTION_SIGN                -1.0f
#define LINE_TRACKING_PID_PROPORTIONAL_GAIN               55.0f
#define LINE_TRACKING_PID_INTEGRAL_GAIN                    0.0f
#define LINE_TRACKING_PID_DERIVATIVE_GAIN                 15.0f
#define LINE_TRACKING_PID_OUTPUT_LIMIT                   260.0f
#define LINE_TRACKING_DRIVE_BASE_PWM                     190.0f
#define LINE_TRACKING_SLOW_BASE_PWM                      125.0f
#define LINE_TRACKING_APPROACH_BASE_PWM                  105.0f
#define COMPETITION_TASK2_BASE_PWM                       400.0f
#define COMPETITION_TASK2_CURVE_BASE_PWM                 360.0f
#define COMPETITION_TASK2_SHARP_TURN_BASE_PWM            340.0f
#define COMPETITION_TASK2_CURVE_HOLD_PWM                 350.0f
#define COMPETITION_TASK2_APPROACH_BASE_PWM              250.0f
#define COMPETITION_LAP_STEADY_BASE_PWM                  200.0f

/* 任务2同样只跑直线和半圆弧，使用高速连续圆弧参数，避免分级急转导致丢线。 */
#define COMPETITION_TASK2_PID_PROPORTIONAL_GAIN           52.0f
#define COMPETITION_TASK2_PID_INTEGRAL_GAIN                0.0f
#define COMPETITION_TASK2_PID_DERIVATIVE_GAIN              8.0f
#define COMPETITION_TASK2_PID_OUTPUT_LIMIT                210.0f
#define COMPETITION_TASK2_ARC_BASE_PWM                    350.0f
#define COMPETITION_TASK2_ARC_SPEED_REDUCTION_START_ERROR   1.20f
#define COMPETITION_TASK2_ARC_FULL_SPEED_REDUCTION_ERROR    3.20f
#define COMPETITION_TASK2_CENTER_ERROR_DEADBAND              0.10f
#define COMPETITION_TASK2_MAX_STEERING_PWM                190.0f
#define COMPETITION_TASK2_ERROR_MAX_CHANGE_PER_STEP         0.25f
#define COMPETITION_TASK2_STEERING_MAX_CHANGE_PER_STEP      8.0f
#define COMPETITION_TASK2_BASE_PWM_MAX_CHANGE_PER_STEP       8.0f
#define COMPETITION_TASK2_MOTOR_PWM_MAX_CHANGE_PER_STEP       8.0f
#define COMPETITION_TASK2_MINIMUM_FORWARD_PWM              70.0f
#define COMPETITION_TASK2_LOST_LINE_OUTER_PWM             280.0f
#define COMPETITION_TASK2_LOST_LINE_INNER_PWM              80.0f
#define COMPETITION_TASK2_SENSOR_GAP_HOLD_STEPS              3U
#define COMPETITION_TASK2_LOST_LINE_MAX_SEARCH_STEPS       60U

/*
 * 任务5/6当前实测用时正好30秒。起步、终点前减速和横线确认后的停车都使用任务4同款五次S曲线，
 * 让PWM变化率及其斜率在首尾回到零，减小长车身带动小球前后晃动。
 * 为进一步放慢首尾过程，中段直线提高到200、圆弧提高到195补回时间；
 * 终点速度和650mm预减速不变，只利用稳定区提速，不在起步或停车阶段追求速度。
 * 终点完成以黑横线识别为准，5832mm只用于终点窗口和终点前预减速，不作为任务5/6完成条件。
 * 这些参数不影响任务2高速圈和任务4定长行驶，便于分别调车。
 */
#define COMPETITION_TASK5_6_PID_PROPORTIONAL_GAIN         45.0f
#define COMPETITION_TASK5_6_PID_INTEGRAL_GAIN              0.0f
#define COMPETITION_TASK5_6_PID_DERIVATIVE_GAIN            6.0f
#define COMPETITION_TASK5_6_PID_OUTPUT_LIMIT              260.0f
#define COMPETITION_TASK5_6_FINISH_BASE_PWM               124.0f
#define COMPETITION_TASK5_6_ARC_BASE_PWM                  195.0f
/*
 * 起步0→200使用2400ms，停车124→0使用1500ms；两段均采用同一五次S曲线，
 * PWM幅值与时间的比例接近，因此实际最大PWM变化率也基本对称。
 */
#define COMPETITION_TASK5_6_START_RAMP_DURATION_MS         2400U
#define COMPETITION_TASK5_6_FINISH_SOFT_STOP_DURATION_MS   1500U
#define COMPETITION_TASK5_6_FINISH_TIME_MARGIN_MS            20U
#define COMPETITION_TASK5_6_ARC_SPEED_REDUCTION_START_ERROR 1.20f
#define COMPETITION_TASK5_6_ARC_FULL_SPEED_REDUCTION_ERROR  3.20f
#define COMPETITION_TASK5_6_CENTER_ERROR_DEADBAND            0.15f
#define COMPETITION_TASK5_6_MAX_STEERING_PWM              110.0f
#define COMPETITION_TASK5_6_ERROR_MAX_CHANGE_PER_STEP       0.20f
#define COMPETITION_TASK5_6_STEERING_MAX_CHANGE_PER_STEP    4.0f
#define COMPETITION_TASK5_6_BASE_PWM_MAX_CHANGE_PER_STEP    2.0f
#define COMPETITION_TASK5_6_MOTOR_PWM_MAX_CHANGE_PER_STEP    4.0f
#define COMPETITION_TASK5_6_MINIMUM_FORWARD_PWM            25.0f
#define COMPETITION_TASK5_6_LOST_LINE_OUTER_PWM           170.0f
#define COMPETITION_TASK5_6_LOST_LINE_INNER_PWM            25.0f
#define COMPETITION_TASK5_6_SENSOR_GAP_HOLD_STEPS            4U
#define COMPETITION_TASK5_6_LOST_LINE_MAX_SEARCH_STEPS     50U

/*
 * 任务7在任务5/6柔和循迹的基础上取消终点、定长和30秒限制，只保留人工按键停车。
 * 速度是运行时PWM目标，可在待机页用外部按键按10PWM步进调整；起步继续使用五次S曲线。
 */
#define COMPETITION_TASK7_DEFAULT_BASE_PWM                 260.0f
#define COMPETITION_TASK7_SPEED_SETTING_MINIMUM_PWM        100.0f
#define COMPETITION_TASK7_SPEED_SETTING_MAXIMUM_PWM        300.0f
#define COMPETITION_TASK7_SPEED_SETTING_STEP_PWM            10.0f
#define COMPETITION_TASK7_START_RAMP_DURATION_MS            2400U

/*
 * 以下弯道参数仿照参考工程 E:\A-2026diansai\715test1 的普通循迹逻辑：
 * 外侧灰度优先判断转向，误差达到普通弯或急弯阈值后降低基础速度，
 * 同时保证足够的最小转向量，出弯后短时保持低速以抑制连续摆动。
 */
#define LINE_TRACKING_CURVE_ERROR_THRESHOLD                1.25f
#define LINE_TRACKING_CURVE_BASE_PWM                     165.0f
#define LINE_TRACKING_CURVE_MIN_CORRECTION_PWM            90.0f
#define LINE_TRACKING_SHARP_TURN_ERROR_THRESHOLD           2.40f
#define LINE_TRACKING_SHARP_TURN_BASE_PWM                125.0f
#define LINE_TRACKING_SHARP_TURN_MIN_CORRECTION_PWM      160.0f
#define LINE_TRACKING_DENSE_CURVE_HOLD_PWM               145.0f
#define LINE_TRACKING_CURVE_SPEED_HOLD_STEPS              30U
#define LINE_TRACKING_OUTER_SENSOR_FORCED_ERROR             3.50f
#define LINE_TRACKING_ERROR_MAX_CHANGE_PER_STEP             0.35f
#define LINE_TRACKING_STEERING_MAX_CHANGE_PWM_PER_STEP      12.0f
#define LINE_TRACKING_DIRECTION_MEMORY_ERROR                0.20f
#define LINE_TRACKING_LOST_LINE_OUTER_PWM                 170.0f
#define LINE_TRACKING_LOST_LINE_INNER_PWM                  20.0f
#define LINE_TRACKING_LOST_LINE_MAX_SEARCH_STEPS           20U

/*
 * 直道时灰度位置误差较小且车体角速度较低，才锁定当前航向作为短时参考。
 * 一旦进入半圆弯道，外侧探头、较大循迹误差或陀螺仪角速度会立即解除锁定，
 * 防止航向环与循迹环互相对抗。
 */
#define STRAIGHT_GYRO_ASSIST_ENABLE                         1U
#define STRAIGHT_GYRO_LOCK_LINE_ERROR_MAX                   0.75f
#define STRAIGHT_GYRO_UNLOCK_LINE_ERROR                     1.25f
#define STRAIGHT_GYRO_LOCK_YAW_RATE_MAX_DEG_PER_S            8.0f
#define STRAIGHT_GYRO_UNLOCK_YAW_RATE_DEG_PER_S             18.0f
#define STRAIGHT_GYRO_LOCK_CONFIRMATION_STEPS               20U
#define STRAIGHT_GYRO_STALE_MAXIMUM_STEPS                    40U
#define STRAIGHT_GYRO_PID_PROPORTIONAL_GAIN                  3.0f
#define STRAIGHT_GYRO_PID_INTEGRAL_GAIN                      0.0f
#define STRAIGHT_GYRO_PID_DERIVATIVE_GAIN                    0.8f
#define STRAIGHT_GYRO_PID_OUTPUT_LIMIT                      35.0f

/* ==================== 任务4陀螺仪定向直行 ==================== */
/*
 * 任务4按距离执行五次S曲线加减速。五次曲线在首尾同时将PWM变化率和变化率的斜率压到零，
 * 比半余弦曲线更能减小长车身起步、进入巡航和停车前的冲击，避免小球前后晃动。
 * 默认定长由COMPETITION_TASK4_GYRO_TARGET_DISTANCE_MM给出，也可在待机页用外部按键临时调整；
 * 加速段固定500mm，最后550mm减速起点会自动跟随运行时目标，600ms线性到0保持不变。
 * 起步由运动层按1PWM/周期从0线性升到120；末端先降到115，再用600ms线性降到0。
 * 只把巡航提高到250，为实测超时约0.3秒留出余量；首尾PWM、距离和软停时间均不改变。
 */
#define COMPETITION_TASK4_GYRO_START_PWM                   120.0f
#define COMPETITION_TASK4_GYRO_CRUISE_PWM                  250.0f
#define COMPETITION_TASK4_GYRO_FINISH_PWM                  115.0f
#define COMPETITION_TASK4_GYRO_ACCELERATION_DISTANCE_MM    500.0f
#ifndef COMPETITION_TASK4_GYRO_TARGET_DISTANCE_MM
#define COMPETITION_TASK4_GYRO_TARGET_DISTANCE_MM         1500.0f
#endif
#define COMPETITION_TASK4_DISTANCE_SETTING_MINIMUM_MM     1100.0f
#define COMPETITION_TASK4_DISTANCE_SETTING_MAXIMUM_MM     3000.0f
#define COMPETITION_TASK4_GYRO_DECELERATION_DISTANCE_MM    550.0f
#define COMPETITION_TASK4_GYRO_BASE_PWM_MAX_CHANGE_PER_STEP  1.0f
/*
 * 末端线性软停的理论行程约为“当前速度×软停时间÷2”。减去15mm用于补偿低PWM死区，
 * 再限制在4～32mm内，避免低速时过早停车，同时限制高速时明显越过B点。
 */
#define COMPETITION_TASK4_GYRO_SPEED_SAMPLE_INTERVAL_MS     100U
#define COMPETITION_TASK4_GYRO_SOFT_STOP_DURATION_MS        600U
#define COMPETITION_TASK4_GYRO_SOFT_STOP_DISTANCE_MIN_MM      4.0f
#define COMPETITION_TASK4_GYRO_SOFT_STOP_DISTANCE_MAX_MM     32.0f
#define COMPETITION_TASK4_GYRO_SOFT_STOP_DISTANCE_TRIM_MM    15.0f
#define COMPETITION_TASK4_GYRO_PID_PROPORTIONAL_GAIN         3.0f
#define COMPETITION_TASK4_GYRO_PID_INTEGRAL_GAIN             0.0f
#define COMPETITION_TASK4_GYRO_PID_DERIVATIVE_GAIN           0.8f
#define COMPETITION_TASK4_GYRO_PID_OUTPUT_LIMIT             40.0f
/*
 * 实车验证表明当前电机安装和输出映射下，正转向补偿会使车辆进一步向右。
 * 因此使用负值抵消固定右偏；先从-4开始，避免由+8直接反到-8造成过度左偏。
 */
#define COMPETITION_TASK4_GYRO_STEERING_TRIM_PWM            -4.0f
#define COMPETITION_TASK4_GYRO_STALE_MAXIMUM_STEPS          40U

/* ==================== H题赛道与停车标定 ==================== */
/* 以任务6实车定长结果5832mm作为终点识别和任务5/6预减速的圈长基准。 */
#define COMPETITION_LAP_THEORETICAL_LENGTH_MM             5832.0f
/*
 * 任务2优先识别黑横线，只有横线漏识别时才在该距离定长停车。这里给出上电默认值，
 * 待机页还可用外部按键临时调整，不会影响任务5/6固定使用的5832mm圈长基准。
 */
#ifndef COMPETITION_TASK2_FIXED_DISTANCE_STOP_MM
#define COMPETITION_TASK2_FIXED_DISTANCE_STOP_MM          5910.0f
#endif
#define COMPETITION_TASK2_DISTANCE_SETTING_MINIMUM_MM     5400.0f
#define COMPETITION_TASK2_DISTANCE_SETTING_MAXIMUM_MM     7000.0f
/*
 * 任务5/6仍以真实黑横线为第一停车条件；若横线漏识别，则在该可调距离启动原有五次S曲线软停。
 * 这里的数值表示“开始软停的位置”，不是电机完全归零后的最终编码器距离。
 */
#ifndef COMPETITION_TASK5_6_FIXED_DISTANCE_STOP_MM
#define COMPETITION_TASK5_6_FIXED_DISTANCE_STOP_MM \
    COMPETITION_LAP_THEORETICAL_LENGTH_MM
#endif
#define COMPETITION_TASK5_6_DISTANCE_SETTING_MINIMUM_MM   5400.0f
#define COMPETITION_TASK5_6_DISTANCE_SETTING_MAXIMUM_MM   7000.0f
#define COMPETITION_DISTANCE_SETTING_STEP_MM                10.0f
#define COMPETITION_FINISH_APPROACH_DISTANCE_MM            250.0f
#define COMPETITION_TASK5_6_DECELERATION_DISTANCE_MM       650.0f
#define COMPETITION_FINISH_DETECTION_WINDOW_MM              900.0f
#define COMPETITION_FINISH_DISPLAY_SUSPEND_DISTANCE_MM \
    (COMPETITION_LAP_THEORETICAL_LENGTH_MM - COMPETITION_FINISH_DETECTION_WINDOW_MM)
#define COMPETITION_FINISH_LINE_CONFIRM_WINDOW_MM           120.0f
#define COMPETITION_FINISH_EXIT_CONFIRM_DISTANCE_MM \
    (COMPETITION_LAP_THEORETICAL_LENGTH_MM - COMPETITION_FINISH_LINE_CONFIRM_WINDOW_MM)
#define COMPETITION_TASK2_FINISH_CONFIRM_WINDOW_MM           350.0f
#define COMPETITION_TASK2_FINISH_EXIT_CONFIRM_DISTANCE_MM \
    (COMPETITION_LAP_THEORETICAL_LENGTH_MM - \
        COMPETITION_TASK2_FINISH_CONFIRM_WINDOW_MM)

/*
 * 顺时针末段圆弧会先让R2～R4压线，出弯到终点时横线会继续扫到L4～L2。
 * 普通纵向轨迹可能同时落在L1/R1，因此停车组合刻意排除这两个中心探头；
 * 5cm短横线不要求5路同时有效，只在100mm内累计两侧非中心探头的扫线结果。
 * 掩码位0～7依次对应L4、L3、L2、L1、R1、R2、R3、R4。
 */
#define COMPETITION_FINISH_CLOCKWISE_ARC_SENSOR_MASK        0xE0U
#define COMPETITION_FINISH_LEFT_CROSS_SENSOR_MASK           0x07U
#define COMPETITION_FINISH_RIGHT_CROSS_SENSOR_MASK          0xE0U
#define COMPETITION_FINISH_SENSOR_ACCUMULATION_DISTANCE_MM   100.0f
#define COMPETITION_TASK2_FINISH_SENSOR_ACCUMULATION_DISTANCE_MM 150.0f
#define COMPETITION_FINISH_LINE_MINIMUM_ACTIVE_SENSORS        2U
#define COMPETITION_FINISH_LINE_CONFIRMATION_STEPS            2U
#define COMPETITION_TASK2_FINISH_LINE_CONFIRMATION_STEPS       1U
/* 该阈值仅供循迹控制识别“整排压线并保持居中”，不能与停车阈值共用。 */
#define COMPETITION_WIDE_LINE_MINIMUM_ACTIVE_SENSORS          6U

#define COMPETITION_TASK2_TIMEOUT_MS                       22000U
#define COMPETITION_TASK4_TIMEOUT_MS                       10000U
#define COMPETITION_TASK5_6_TIMEOUT_MS                     30000U
#define COMPETITION_DISPLAY_REFRESH_MS                       100U

/* ==================== 串口、显示与主循环 ==================== */
#define G356_UART_BAUD_RATE                               115200U
/* UART1用于向视觉控球主控发送50Hz底盘惯性前馈帧。 */
#define WIRELESS_UART_BAUD_RATE                           115200U
#define INERTIA_TELEMETRY_TRANSMIT_INTERVAL_MS              20U
#define MAIN_LOOP_DELAY_CPU_CYCLES                        160000U

#endif
