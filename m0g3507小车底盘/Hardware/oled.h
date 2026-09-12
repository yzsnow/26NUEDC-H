/**
 * @file oled.h
 * @brief OLED 显示模块对外接口。
 */
#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/* 硬件复位 SSD1306、发送初始化命令并清屏。 */
void OLED_Initialize(void);
/* 只清空 RAM 中的屏幕缓冲区，调用刷新后才会显示。 */
void OLED_ClearDisplayBuffer(void);
/* 在缓冲区中设置一个像素。 */
void OLED_SetPixel(uint8_t x_coordinate, uint8_t y_coordinate, uint8_t pixel_is_on);
/* 在缓冲区中绘制一个 5×7 ASCII 字符。 */
void OLED_DrawCharacter(uint8_t x_coordinate, uint8_t y_coordinate, char character);
/* 在缓冲区中绘制字符串。 */
void OLED_DrawString(uint8_t x_coordinate, uint8_t y_coordinate, const char *text);
/* 在缓冲区中绘制无符号整数。 */
void OLED_DrawUnsignedInteger(uint8_t x_coordinate, uint8_t y_coordinate, uint32_t value);
/* 在缓冲区中绘制带正负号的小数。 */
void OLED_DrawSignedFloat(uint8_t x_coordinate, uint8_t y_coordinate, float value, uint8_t decimal_places);
/* 通过软件 SPI 将 1024 字节缓冲区发送到 OLED。 */
void OLED_RefreshDisplay(void);

/**
 * @brief 只刷新一页显存并自动轮换到下一页。
 * @note 运行控制期间周期调用可把一次全屏阻塞拆成八次短传输。
 */
void OLED_RefreshNextPage(void);

#endif
