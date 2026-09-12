/**
 * @file menu_buttons.h
 * @brief 菜单按键事件定义与接口声明。
 */
#ifndef MENU_BUTTONS_H
#define MENU_BUTTONS_H

/* 消抖后产生的单次按键事件。 */
typedef enum {
    MENU_BUTTON_EVENT_NONE = 0,
    MENU_BUTTON_EVENT_UP,
    MENU_BUTTON_EVENT_DOWN,
    MENU_BUTTON_EVENT_CONFIRM,
    MENU_BUTTON_EVENT_BACK
} MenuButtonEvent;

/* 保存开机时的按键状态，避免上电误触发。 */
void MenuButtons_Initialize(void);
/* 执行一次按键采样和消抖，只在新按下时返回事件。 */
MenuButtonEvent MenuButtons_GetPressedEvent(void);

#endif