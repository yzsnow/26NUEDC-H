/**
 * @file menu_buttons.c
 * @brief 菜单按键读取、消抖和单次按下事件生成。
 */
#include "menu_buttons.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define ALL_MENU_BUTTON_BITS 0x0FU

static uint8_t stable_button_state;
static uint8_t previous_raw_button_state;
static uint8_t matching_sample_count;

/* 将四个低电平有效按键压缩为 4 位状态。 */
static uint8_t read_pressed_button_bits(void)
{
    uint8_t pressed_button_bits = 0U;
    if (DL_GPIO_readPins(MENU_BUTTON_PORT, MENU_BUTTON_UP_PIN) == 0U) pressed_button_bits |= 0x01U;
    if (DL_GPIO_readPins(MENU_BUTTON_PORT, MENU_BUTTON_DOWN_PIN) == 0U) pressed_button_bits |= 0x02U;
    if (DL_GPIO_readPins(BOARD_KEY_PORT, BOARD_KEY_PIN) == 0U) pressed_button_bits |= 0x04U;
    if (DL_GPIO_readPins(MENU_BUTTON_PORT, MENU_BUTTON_BACK_PIN) == 0U) pressed_button_bits |= 0x08U;
    return pressed_button_bits;
}

/* 记录初始状态，避免上电时把已按下按键误判为新事件。 */
void MenuButtons_Initialize(void)
{
    stable_button_state = read_pressed_button_bits();
    previous_raw_button_state = stable_button_state;
    matching_sample_count = 0U;
}

/* 连续三次采样一致才确认状态变化，并只返回新按下事件。 */
MenuButtonEvent MenuButtons_GetPressedEvent(void)
{
    uint8_t raw_button_state = read_pressed_button_bits();
    if (raw_button_state != previous_raw_button_state) {
        previous_raw_button_state = raw_button_state;
        matching_sample_count = 0U;
        return MENU_BUTTON_EVENT_NONE;
    }
    if (matching_sample_count < 2U) {
        matching_sample_count++;
        return MENU_BUTTON_EVENT_NONE;
    }
    if (raw_button_state == stable_button_state) return MENU_BUTTON_EVENT_NONE;

    uint8_t newly_pressed_bits = (uint8_t)(raw_button_state & (uint8_t)~stable_button_state & ALL_MENU_BUTTON_BITS);
    stable_button_state = raw_button_state;
    if ((newly_pressed_bits & 0x01U) != 0U) return MENU_BUTTON_EVENT_UP;
    if ((newly_pressed_bits & 0x02U) != 0U) return MENU_BUTTON_EVENT_DOWN;
    if ((newly_pressed_bits & 0x04U) != 0U) return MENU_BUTTON_EVENT_CONFIRM;
    if ((newly_pressed_bits & 0x08U) != 0U) return MENU_BUTTON_EVENT_BACK;
    return MENU_BUTTON_EVENT_NONE;
}
