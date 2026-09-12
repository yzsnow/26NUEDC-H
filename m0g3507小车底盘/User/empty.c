/**
 * @file empty.c
 * @brief H题天猛星底盘入口，初始化循迹、编码器、陀螺仪、计时和比赛状态机。
 */
#include "ti_msp_dl_config.h"
#include "vehicle_config.h"
#include "competition_control.h"
#include "system_time.h"
#include "motor.h"
#include "encoder.h"
#include "g356.h"
#include "oled.h"
#include "motion.h"
#include "menu_buttons.h"

int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();
    Encoder_Init();
    G356_InitializeUartReceiver();
    OLED_Initialize();
    MenuButtons_Initialize();
    Motion_InitializeControllers();
    SystemTime_Initialize();
    CompetitionControl_Initialize();

    while (1) {
        CompetitionControl_Process();
        delay_cycles(MAIN_LOOP_DELAY_CPU_CYCLES);
    }
}
