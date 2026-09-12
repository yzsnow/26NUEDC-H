/**
 * @file system_time.c
 * @brief MSPM0G3507 SysTick毫秒计时实现，保证OLED刷新不改变比赛计时结果。
 */
#include "system_time.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t system_milliseconds;

void SystemTime_Initialize(void)
{
    system_milliseconds = 0U;
    SysTick_Config(CPUCLK_FREQ / 1000U);
}

uint32_t SystemTime_GetMilliseconds(void)
{
    return system_milliseconds;
}

uint8_t SystemTime_HasElapsed(
    uint32_t start_milliseconds, uint32_t interval_milliseconds)
{
    return (uint32_t)(system_milliseconds - start_milliseconds) >=
        interval_milliseconds;
}

void SysTick_Handler(void)
{
    system_milliseconds++;
}
