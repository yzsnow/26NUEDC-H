/**
 * @file inertia_telemetry.c
 * @brief 生成带CRC16的底盘惯性前馈帧，并通过UART1非阻塞发送给视觉控球主控。
 *
 * 直接只发送某一个加速度轴会依赖G356安装方向，还会混入重力和车架轻微倾斜。
 * 因此协议同时发送三轴加速度、横滚/俯仰、偏航角速度、编码器纵向运动和电机命令，
 * 由视觉主控结合实际安装方向选择前后、左右补偿量，并可用编码器量抑制陀螺仪震动。
 */
#include "inertia_telemetry.h"
#include "vehicle_config.h"
#include "system_time.h"
#include "encoder.h"
#include "g356.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define INERTIA_FRAME_HEADER_FIRST_BYTE                 0xA5U
#define INERTIA_FRAME_HEADER_SECOND_BYTE                0x5AU
#define INERTIA_FRAME_PROTOCOL_VERSION                  0x01U
#define INERTIA_FRAME_SIZE_BYTES                          32U
#define INERTIA_FRAME_PAYLOAD_SIZE_BYTES                  26U
#define INERTIA_FRAME_CRC_START_INDEX                      0U
#define INERTIA_FRAME_CRC_END_INDEX                       29U
#define INERTIA_ACCELERATION_SCALE_MILLI_G_PER_G       1000.0f
#define INERTIA_ANGLE_SCALE_CENTIDEGREE_PER_DEGREE      100.0f
#define INERTIA_PWM_SCALE_TENTHS_PER_PWM                  10.0f
#define INERTIA_GYROSCOPE_STALE_TIMEOUT_MS               100U

#define INERTIA_FLAG_GYROSCOPE_VALID                    0x01U
#define INERTIA_FLAG_ENCODER_VALID                      0x02U
#define INERTIA_FLAG_COMPENSATION_TASK                  0x04U
#define INERTIA_FLAG_MOTOR_MOVING                       0x08U

static volatile uint8_t transmit_frame[INERTIA_FRAME_SIZE_BYTES];
static volatile uint8_t transmit_index;
static volatile bool transmission_is_active;
static uint8_t frame_sequence;
static uint32_t last_transmit_milliseconds;
static uint32_t previous_speed_sample_milliseconds;
static uint32_t last_gyroscope_frame_milliseconds;
static uint32_t previous_gyroscope_frame_count;
static float previous_distance_mm;
static float filtered_speed_mm_per_second;
static float previous_filtered_speed_mm_per_second;
static float previous_forward_pwm;
static bool encoder_estimate_is_valid;
static uint8_t previous_task_id;

static int16_t clamp_float_to_int16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static void write_little_endian_int16(uint8_t *frame, uint8_t index,
    int16_t value)
{
    frame[index] = (uint8_t)((uint16_t)value & 0xFFU);
    frame[index + 1U] = (uint8_t)(((uint16_t)value >> 8U) & 0xFFU);
}

/* CRC16-Modbus覆盖帧头到最后一个数据字节，接收端可据此丢弃串口噪声或半帧。 */
static uint16_t calculate_crc16_modbus(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint8_t byte_index = 0U; byte_index < length; byte_index++) {
        crc ^= data[byte_index];
        for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/* 尽量填满UART FIFO，剩余字节由TX中断继续发送，避免主循环等待串口移位完成。 */
static void fill_uart_transmit_fifo(void)
{
    while (transmit_index < INERTIA_FRAME_SIZE_BYTES &&
        !DL_UART_Main_isTXFIFOFull(WIRELESS_UART_INST)) {
        DL_UART_Main_transmitData(WIRELESS_UART_INST,
            transmit_frame[transmit_index]);
        transmit_index++;
    }

    if (transmit_index >= INERTIA_FRAME_SIZE_BYTES) {
        transmission_is_active = false;
        DL_UART_Main_disableInterrupt(WIRELESS_UART_INST,
            DL_UART_MAIN_INTERRUPT_TX);
    }
}

static void reset_encoder_estimate(float distance_mm,
    uint32_t current_milliseconds)
{
    previous_distance_mm = distance_mm;
    previous_speed_sample_milliseconds = current_milliseconds;
    filtered_speed_mm_per_second = 0.0f;
    previous_filtered_speed_mm_per_second = 0.0f;
    encoder_estimate_is_valid = false;
}

/*
 * 编码器速度做轻度低通再求导，主要用于给视觉端提供稳定的前后惯性趋势；
 * G356三轴加速度响应更快，编码器量不替代它，只用于去除安装零偏和高频震动。
 */
static float update_encoder_motion_estimate(float distance_mm,
    uint32_t current_milliseconds, float *acceleration_mm_per_second_squared)
{
    uint32_t elapsed_milliseconds = current_milliseconds -
        previous_speed_sample_milliseconds;

    *acceleration_mm_per_second_squared = 0.0f;
    if (elapsed_milliseconds == 0U) return filtered_speed_mm_per_second;

    float measured_speed_mm_per_second =
        (distance_mm - previous_distance_mm) * 1000.0f /
        (float)elapsed_milliseconds;
    if (measured_speed_mm_per_second < 0.0f) {
        measured_speed_mm_per_second = 0.0f;
    }

    previous_filtered_speed_mm_per_second = filtered_speed_mm_per_second;
    if (!encoder_estimate_is_valid) {
        filtered_speed_mm_per_second = measured_speed_mm_per_second;
        encoder_estimate_is_valid = true;
    } else {
        filtered_speed_mm_per_second = filtered_speed_mm_per_second * 0.35f +
            measured_speed_mm_per_second * 0.65f;
        *acceleration_mm_per_second_squared =
            (filtered_speed_mm_per_second -
                previous_filtered_speed_mm_per_second) * 1000.0f /
            (float)elapsed_milliseconds;
    }

    previous_distance_mm = distance_mm;
    previous_speed_sample_milliseconds = current_milliseconds;
    return filtered_speed_mm_per_second;
}

static void start_frame_transmission(const uint8_t *frame)
{
    if (transmission_is_active) return;

    for (uint8_t byte_index = 0U;
        byte_index < INERTIA_FRAME_SIZE_BYTES; byte_index++) {
        transmit_frame[byte_index] = frame[byte_index];
    }
    transmit_index = 0U;
    transmission_is_active = true;
    fill_uart_transmit_fifo();
    if (transmission_is_active) {
        DL_UART_Main_enableInterrupt(WIRELESS_UART_INST,
            DL_UART_MAIN_INTERRUPT_TX);
    }
}

void InertiaTelemetry_Initialize(void)
{
    uint32_t current_milliseconds = SystemTime_GetMilliseconds();

    transmit_index = 0U;
    transmission_is_active = false;
    frame_sequence = 0U;
    last_transmit_milliseconds = current_milliseconds -
        INERTIA_TELEMETRY_TRANSMIT_INTERVAL_MS;
    previous_speed_sample_milliseconds = current_milliseconds;
    last_gyroscope_frame_milliseconds = current_milliseconds;
    previous_gyroscope_frame_count = G356_GetValidFrameCount();
    previous_distance_mm = Encoder_GetAverageDistanceMm();
    filtered_speed_mm_per_second = 0.0f;
    previous_filtered_speed_mm_per_second = 0.0f;
    previous_forward_pwm = 0.0f;
    encoder_estimate_is_valid = false;
    previous_task_id = INERTIA_TELEMETRY_TASK_NONE;
    DL_UART_Main_setTXFIFOThreshold(WIRELESS_UART_INST,
        DL_UART_MAIN_TX_FIFO_LEVEL_EMPTY);
    DL_UART_Main_disableInterrupt(WIRELESS_UART_INST,
        DL_UART_MAIN_INTERRUPT_TX);
    NVIC_ClearPendingIRQ(WIRELESS_UART_INT_IRQN);
    NVIC_EnableIRQ(WIRELESS_UART_INT_IRQN);
}

void InertiaTelemetry_Process(uint8_t task_id, uint8_t motion_state)
{
    uint8_t frame[INERTIA_FRAME_SIZE_BYTES] = {0U};
    G356_TelemetryData gyroscope_data = {0};
    uint32_t current_milliseconds = SystemTime_GetMilliseconds();
    uint32_t current_gyroscope_frame_count = G356_GetValidFrameCount();
    float distance_mm;
    float acceleration_mm_per_second_squared;
    float speed_mm_per_second;
    float left_pwm;
    float right_pwm;
    float forward_pwm;
    float turn_pwm;
    float forward_pwm_rate_per_second;
    uint8_t flags = 0U;
    uint16_t crc;

    if ((uint32_t)(current_milliseconds - last_transmit_milliseconds) <
        INERTIA_TELEMETRY_TRANSMIT_INTERVAL_MS || transmission_is_active) {
        return;
    }
    last_transmit_milliseconds = current_milliseconds;
    distance_mm = Encoder_GetAverageDistanceMm();

    if (task_id != previous_task_id ||
        distance_mm + 5.0f < previous_distance_mm) {
        reset_encoder_estimate(distance_mm, current_milliseconds);
    }
    speed_mm_per_second = update_encoder_motion_estimate(distance_mm,
        current_milliseconds, &acceleration_mm_per_second_squared);
    if (encoder_estimate_is_valid) flags |= INERTIA_FLAG_ENCODER_VALID;

    if (current_gyroscope_frame_count != previous_gyroscope_frame_count) {
        previous_gyroscope_frame_count = current_gyroscope_frame_count;
        last_gyroscope_frame_milliseconds = current_milliseconds;
    }
    if ((uint32_t)(current_milliseconds - last_gyroscope_frame_milliseconds) <=
        INERTIA_GYROSCOPE_STALE_TIMEOUT_MS &&
        G356_GetTelemetrySnapshot(&gyroscope_data)) {
        flags |= INERTIA_FLAG_GYROSCOPE_VALID;
    }

    if (task_id == INERTIA_TELEMETRY_TASK4 ||
        task_id == INERTIA_TELEMETRY_TASK5_6) {
        flags |= INERTIA_FLAG_COMPENSATION_TASK;
    }
    left_pwm = Motor_GetLeftCommandPwm();
    right_pwm = Motor_GetRightCommandPwm();
    forward_pwm = (left_pwm + right_pwm) * 0.5f;
    turn_pwm = (right_pwm - left_pwm) * 0.5f;
    if (left_pwm > 1.0f || left_pwm < -1.0f ||
        right_pwm > 1.0f || right_pwm < -1.0f) {
        flags |= INERTIA_FLAG_MOTOR_MOVING;
    }
    forward_pwm_rate_per_second = (forward_pwm - previous_forward_pwm) *
        1000.0f / (float)INERTIA_TELEMETRY_TRANSMIT_INTERVAL_MS;
    previous_forward_pwm = forward_pwm;

    frame[0] = INERTIA_FRAME_HEADER_FIRST_BYTE;
    frame[1] = INERTIA_FRAME_HEADER_SECOND_BYTE;
    frame[2] = INERTIA_FRAME_PROTOCOL_VERSION;
    frame[3] = INERTIA_FRAME_PAYLOAD_SIZE_BYTES;
    frame[4] = frame_sequence++;
    frame[5] = task_id;
    frame[6] = motion_state;
    frame[7] = flags;
    write_little_endian_int16(frame, 8U, clamp_float_to_int16(
        gyroscope_data.acceleration_x_g *
            INERTIA_ACCELERATION_SCALE_MILLI_G_PER_G));
    write_little_endian_int16(frame, 10U, clamp_float_to_int16(
        gyroscope_data.acceleration_y_g *
            INERTIA_ACCELERATION_SCALE_MILLI_G_PER_G));
    write_little_endian_int16(frame, 12U, clamp_float_to_int16(
        gyroscope_data.acceleration_z_g *
            INERTIA_ACCELERATION_SCALE_MILLI_G_PER_G));
    write_little_endian_int16(frame, 14U, clamp_float_to_int16(
        gyroscope_data.roll_degrees *
            INERTIA_ANGLE_SCALE_CENTIDEGREE_PER_DEGREE));
    write_little_endian_int16(frame, 16U, clamp_float_to_int16(
        gyroscope_data.pitch_degrees *
            INERTIA_ANGLE_SCALE_CENTIDEGREE_PER_DEGREE));
    write_little_endian_int16(frame, 18U, clamp_float_to_int16(
        gyroscope_data.angular_velocity_z_degrees_per_second *
            INERTIA_ANGLE_SCALE_CENTIDEGREE_PER_DEGREE));
    write_little_endian_int16(frame, 20U,
        clamp_float_to_int16(speed_mm_per_second));
    write_little_endian_int16(frame, 22U,
        clamp_float_to_int16(acceleration_mm_per_second_squared));
    write_little_endian_int16(frame, 24U, clamp_float_to_int16(
        forward_pwm * INERTIA_PWM_SCALE_TENTHS_PER_PWM));
    write_little_endian_int16(frame, 26U, clamp_float_to_int16(
        turn_pwm * INERTIA_PWM_SCALE_TENTHS_PER_PWM));
    write_little_endian_int16(frame, 28U,
        clamp_float_to_int16(forward_pwm_rate_per_second));
    crc = calculate_crc16_modbus(frame,
        INERTIA_FRAME_CRC_END_INDEX + 1U);
    frame[30] = (uint8_t)(crc & 0xFFU);
    frame[31] = (uint8_t)((crc >> 8U) & 0xFFU);

    previous_task_id = task_id;
    start_frame_transmission(frame);
}

/* UART1中断只搬运遥测字节；若视觉端误接了回传数据则直接清空，防止RX中断持续占用CPU。 */
void UART1_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index =
        DL_UART_Main_getPendingInterrupt(WIRELESS_UART_INST);

    if (interrupt_index == DL_UART_MAIN_IIDX_TX) {
        fill_uart_transmit_fifo();
    } else if (interrupt_index == DL_UART_MAIN_IIDX_RX) {
        while (!DL_UART_Main_isRXFIFOEmpty(WIRELESS_UART_INST)) {
            (void)DL_UART_Main_receiveData(WIRELESS_UART_INST);
        }
    }
}
