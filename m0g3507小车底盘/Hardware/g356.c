/**
 * @file g356.c
 * @brief JYTech G356 56 字节 UART 遥测帧解析和航向零点管理。
 *
 * 协议依据 G356 用户手册 V1.1：帧头 AA 55，Type=02，Length=36，
 * 字节 2 到 53 累加校验，字节 55 固定为 5A。UART 默认 115200、8N1。
 */
#include "g356.h"
#include "ti_msp_dl_config.h"
#include <string.h>

#define G356_FRAME_SIZE_BYTES                 56U
#define G356_FRAME_HEADER_FIRST_BYTE        0xAAU
#define G356_FRAME_HEADER_SECOND_BYTE       0x55U
#define G356_FRAME_TELEMETRY_TYPE           0x02U
#define G356_FRAME_PAYLOAD_LENGTH           0x36U
#define G356_FRAME_TAIL_BYTE                0x5AU
#define G356_ACCELERATION_COUNTS_PER_G      2048.0f
#define G356_ANGULAR_RATE_COUNTS_PER_DPS       8.2f
#define G356_TEMPERATURE_COUNTS_PER_DEGREE   100.0f

static volatile G356_TelemetryData latest_telemetry_data;
static volatile bool telemetry_data_is_valid;
static volatile float yaw_zero_reference_degrees;
static volatile bool yaw_zero_reference_is_valid;
static volatile uint32_t received_byte_count;
static volatile uint32_t valid_frame_count;
static volatile uint32_t invalid_frame_count;
static volatile uint32_t telemetry_update_sequence;
static uint8_t received_frame[G356_FRAME_SIZE_BYTES];
static uint8_t parser_state;
static uint8_t parser_index;

/* 显式按小端序解析，避免依赖未对齐结构体和编译器填充方式。 */
static int16_t parse_little_endian_int16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

/* G356 的欧拉角和原始六轴字段是 IEEE754 小端 float，使用 memcpy 避免未对齐访问。 */
static float parse_little_endian_float(const uint8_t *bytes)
{
    float parsed_value;
    uint8_t ordered_bytes[sizeof(float)];
    for (uint8_t byte_index = 0U; byte_index < sizeof(float); byte_index++) {
        ordered_bytes[byte_index] = bytes[byte_index];
    }
    memcpy(&parsed_value, ordered_bytes, sizeof(parsed_value));
    return parsed_value;
}

/* 将跨越正负180度边界的航向差转换为最短方向误差。 */
static float normalize_angle_to_180_degrees(float angle_degrees)
{
    while (angle_degrees > 180.0f) angle_degrees -= 360.0f;
    while (angle_degrees < -180.0f) angle_degrees += 360.0f;
    return angle_degrees;
}

/* 先检查固定字段和校验和，避免把串口噪声解析成异常浮点数送入控制环。 */
static bool received_frame_is_valid(const uint8_t *frame)
{
    if (frame[0] != G356_FRAME_HEADER_FIRST_BYTE ||
        frame[1] != G356_FRAME_HEADER_SECOND_BYTE ||
        frame[2] != G356_FRAME_TELEMETRY_TYPE ||
        frame[3] != G356_FRAME_PAYLOAD_LENGTH ||
        frame[55] != G356_FRAME_TAIL_BYTE) {
        return false;
    }

    uint8_t calculated_checksum = 0U;
    for (uint8_t byte_index = 2U; byte_index <= 53U; byte_index++) {
        calculated_checksum = (uint8_t)(calculated_checksum + frame[byte_index]);
    }
    return calculated_checksum == frame[54];
}

/* 一帧同时更新角度和角速度，保证正方形转弯外环与角速度内环使用同一时刻的数据。 */
static void update_telemetry_from_frame(const uint8_t *frame)
{
    /* 奇数表示UART中断正在写遥测字段，偶数表示一整帧已经写完。 */
    telemetry_update_sequence++;
    latest_telemetry_data.acceleration_x_g =
        (float)parse_little_endian_int16(&frame[4]) / G356_ACCELERATION_COUNTS_PER_G;
    latest_telemetry_data.acceleration_y_g =
        (float)parse_little_endian_int16(&frame[6]) / G356_ACCELERATION_COUNTS_PER_G;
    latest_telemetry_data.acceleration_z_g =
        (float)parse_little_endian_int16(&frame[8]) / G356_ACCELERATION_COUNTS_PER_G;
    latest_telemetry_data.angular_velocity_x_degrees_per_second =
        (float)parse_little_endian_int16(&frame[10]) / G356_ANGULAR_RATE_COUNTS_PER_DPS;
    latest_telemetry_data.angular_velocity_y_degrees_per_second =
        (float)parse_little_endian_int16(&frame[12]) / G356_ANGULAR_RATE_COUNTS_PER_DPS;
    latest_telemetry_data.angular_velocity_z_degrees_per_second =
        (float)parse_little_endian_int16(&frame[14]) / G356_ANGULAR_RATE_COUNTS_PER_DPS;
    latest_telemetry_data.roll_degrees = parse_little_endian_float(&frame[16]);
    latest_telemetry_data.pitch_degrees = parse_little_endian_float(&frame[20]);
    latest_telemetry_data.yaw_degrees = parse_little_endian_float(&frame[24]);
    latest_telemetry_data.temperature_degrees_celsius =
        (float)parse_little_endian_int16(&frame[28]) / G356_TEMPERATURE_COUNTS_PER_DEGREE;
    latest_telemetry_data.raw_acceleration_x_g = parse_little_endian_float(&frame[30]);
    latest_telemetry_data.raw_acceleration_y_g = parse_little_endian_float(&frame[34]);
    latest_telemetry_data.raw_acceleration_z_g = parse_little_endian_float(&frame[38]);
    latest_telemetry_data.raw_angular_velocity_x_degrees_per_second =
        parse_little_endian_float(&frame[42]);
    latest_telemetry_data.raw_angular_velocity_y_degrees_per_second =
        parse_little_endian_float(&frame[46]);
    latest_telemetry_data.raw_angular_velocity_z_degrees_per_second =
        parse_little_endian_float(&frame[50]);
    telemetry_update_sequence++;
    telemetry_data_is_valid = true;
    valid_frame_count++;
}

void G356_InitializeUartReceiver(void)
{
    memset((void *)&latest_telemetry_data, 0, sizeof(latest_telemetry_data));
    telemetry_data_is_valid = false;
    yaw_zero_reference_degrees = 0.0f;
    yaw_zero_reference_is_valid = false;
    received_byte_count = 0U;
    valid_frame_count = 0U;
    invalid_frame_count = 0U;
    telemetry_update_sequence = 0U;
    parser_state = 0U;
    parser_index = 0U;
    NVIC_ClearPendingIRQ(GYRO_UART_INT_IRQN);
    NVIC_EnableIRQ(GYRO_UART_INT_IRQN);
}

void G356_ParseReceivedByte(uint8_t received_byte)
{
    received_byte_count++;

    /* UART 没有片选信号，因此必须在连续字节流中寻找 AA 55，并在错误后重新同步。 */
    if (parser_state == 0U) {
        if (received_byte == G356_FRAME_HEADER_FIRST_BYTE) {
            received_frame[0] = received_byte;
            parser_state = 1U;
        }
        return;
    }

    if (parser_state == 1U) {
        if (received_byte == G356_FRAME_HEADER_SECOND_BYTE) {
            received_frame[1] = received_byte;
            parser_index = 2U;
            parser_state = 2U;
        } else if (received_byte == G356_FRAME_HEADER_FIRST_BYTE) {
            received_frame[0] = received_byte;
        } else {
            parser_state = 0U;
        }
        return;
    }

    received_frame[parser_index++] = received_byte;
    if (parser_index < G356_FRAME_SIZE_BYTES) return;

    parser_state = 0U;
    parser_index = 0U;
    if (!received_frame_is_valid(received_frame)) {
        invalid_frame_count++;
        return;
    }
    update_telemetry_from_frame(received_frame);
}

bool G356_HasValidTelemetryData(void)
{
    return telemetry_data_is_valid;
}

bool G356_GetTelemetrySnapshot(G356_TelemetryData *telemetry_data)
{
    uint32_t sequence_before_copy;
    uint32_t sequence_after_copy;

    if (telemetry_data == NULL || !telemetry_data_is_valid) return false;

    /*
     * 序列号为奇数表示中断正在更新字段；复制前后序列号必须相同且为偶数，
     * 这样即使100Hz新帧恰好到达，也不会把两帧拼成一个错误的惯性补偿样本。
     */
    while (true) {
        sequence_before_copy = telemetry_update_sequence;
        if ((sequence_before_copy & 1U) != 0U) continue;
        *telemetry_data = latest_telemetry_data;
        sequence_after_copy = telemetry_update_sequence;
        if (sequence_before_copy == sequence_after_copy &&
            (sequence_after_copy & 1U) == 0U) {
            return true;
        }
    }
}

uint32_t G356_GetReceivedByteCount(void)
{
    return received_byte_count;
}

uint32_t G356_GetValidFrameCount(void)
{
    return valid_frame_count;
}

uint32_t G356_GetInvalidFrameCount(void)
{
    return invalid_frame_count;
}

float G356_GetAbsoluteYawDegrees(void)
{
    return telemetry_data_is_valid ? latest_telemetry_data.yaw_degrees : 0.0f;
}

float G356_GetYawRateDegreesPerSecond(void)
{
    return telemetry_data_is_valid ?
        latest_telemetry_data.angular_velocity_z_degrees_per_second : 0.0f;
}

void G356_SetCurrentYawAsZeroReference(void)
{
    if (!telemetry_data_is_valid) return;
    yaw_zero_reference_degrees = latest_telemetry_data.yaw_degrees;
    yaw_zero_reference_is_valid = true;
}

bool G356_HasYawZeroReference(void)
{
    return yaw_zero_reference_is_valid;
}

float G356_GetYawOffsetDegrees(void)
{
    if (!telemetry_data_is_valid || !yaw_zero_reference_is_valid) return 0.0f;
    return normalize_angle_to_180_degrees(
        latest_telemetry_data.yaw_degrees - yaw_zero_reference_degrees);
}

/* UART3 中断只负责清空 FIFO 和解析数据，避免在100Hz遥测流中执行显示或日志操作。 */
void UART3_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(GYRO_UART_INST) != DL_UART_MAIN_IIDX_RX) return;
    while (!DL_UART_Main_isRXFIFOEmpty(GYRO_UART_INST)) {
        G356_ParseReceivedByte((uint8_t)DL_UART_Main_receiveData(GYRO_UART_INST));
    }
}
