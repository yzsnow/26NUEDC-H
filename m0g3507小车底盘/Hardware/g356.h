/**
 * @file g356.h
 * @brief JYTech G356 六轴姿态模块 UART 遥测解析与航向零点接口。
 */
#ifndef G356_H
#define G356_H

#include <stdbool.h>
#include <stdint.h>

/* G356 每帧同时包含加速度、角速度、欧拉角、温度和原始六轴数据。 */
typedef struct {
    float acceleration_x_g;
    float acceleration_y_g;
    float acceleration_z_g;
    float angular_velocity_x_degrees_per_second;
    float angular_velocity_y_degrees_per_second;
    float angular_velocity_z_degrees_per_second;
    float roll_degrees;
    float pitch_degrees;
    float yaw_degrees;
    float temperature_degrees_celsius;
    float raw_acceleration_x_g;
    float raw_acceleration_y_g;
    float raw_acceleration_z_g;
    float raw_angular_velocity_x_degrees_per_second;
    float raw_angular_velocity_y_degrees_per_second;
    float raw_angular_velocity_z_degrees_per_second;
} G356_TelemetryData;

/**
 * @brief 清空 G356 接收状态并开启 UART3 接收中断。
 * @note 必须在 SYSCFG_DL_init() 完成 UART3 初始化后调用。
 */
void G356_InitializeUartReceiver(void);

/**
 * @brief 向 G356 56 字节遥测帧状态机输入一个 UART 字节。
 * @param received_byte UART3 接收到的一个原始字节。
 * @note 通常只在 UART3_IRQHandler() 中调用，不应从主循环重复喂入。
 */
void G356_ParseReceivedByte(uint8_t received_byte);

/**
 * @brief 判断是否至少收到过一帧校验正确的 G356 遥测数据。
 * @return true 表示航向角和角速度均可使用，否则返回 false。
 */
bool G356_HasValidTelemetryData(void);

/**
 * @brief 原子语义地复制最近一帧完整六轴遥测数据。
 * @param telemetry_data 用于接收遥测快照的结构体指针。
 * @return 已有有效帧且复制成功返回true，否则返回false。
 * @note 主循环可调用；函数会检查帧计数，避免UART中断更新时读到混合数据。
 */
bool G356_GetTelemetrySnapshot(G356_TelemetryData *telemetry_data);

/**
 * @brief 获取 UART3 累计收到的字节数。
 * @return 自本次初始化以来收到的总字节数。
 */
uint32_t G356_GetReceivedByteCount(void);

/**
 * @brief 获取累计校验正确的遥测帧数量。
 * @return 自初始化以来成功解析的有效帧数量。
 */
uint32_t G356_GetValidFrameCount(void);

/**
 * @brief 获取格式或校验失败的完整帧数量。
 * @return 无效 G356 帧累计数量。
 */
uint32_t G356_GetInvalidFrameCount(void);

/**
 * @brief 获取 G356 当前输出的绝对 Yaw 航向角。
 * @return 航向角，单位为度；尚无有效帧时返回 0。
 */
float G356_GetAbsoluteYawDegrees(void);

/**
 * @brief 获取 G356 当前 Z 轴角速度。
 * @return Z 轴角速度，单位为度每秒；尚无有效帧时返回 0。
 */
float G356_GetYawRateDegreesPerSecond(void);

/**
 * @brief 将当前 G356 Yaw 记录为软件零点。
 * @note 只有已经收到有效遥测帧时才会更新零点。
 */
void G356_SetCurrentYawAsZeroReference(void);

/**
 * @brief 判断软件航向零点是否已经建立。
 * @return 已建立返回 true，否则返回 false。
 */
bool G356_HasYawZeroReference(void);

/**
 * @brief 获取相对软件零点的航向偏移。
 * @return 归一化到 -180 到 180 度的偏移角。
 */
float G356_GetYawOffsetDegrees(void);

#endif
