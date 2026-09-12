/**
 * @file oled.c
 * @brief SSD1306 0.96 寸 OLED 软件 SPI 驱动、字符绘制和屏幕缓冲刷新。
 */
#include "oled.h"
#include "ti_msp_dl_config.h"
#include "vehicle_config.h"
#include <string.h>

#define OLED_WIDTH_PIXELS 128U
#define OLED_HEIGHT_PIXELS 64U
#define OLED_PAGE_COUNT 8U
#define FONT_CHARACTER_WIDTH 5U
#define FONT_CHARACTER_SPACING 1U

static uint8_t display_buffer[OLED_WIDTH_PIXELS * OLED_PAGE_COUNT];
static uint8_t next_refresh_page_index;

static const uint8_t digit_font[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E}
};

static const uint8_t uppercase_font[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
};

/* 软件 SPI 的短延时，保证 OLED 能可靠采样时钟。 */
static void oled_spi_delay(void)
{
    delay_cycles(8U);
}

/* 按高位在前方式，用 GPIO 模拟 SPI 发送一个字节。 */
static void oled_spi_write_byte(uint8_t data)
{
    for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++) {
        if ((data & 0x80U) != 0U) DL_GPIO_setPins(OLED_SPI_PORT, OLED_SPI_MOSI_PIN);
        else DL_GPIO_clearPins(OLED_SPI_PORT, OLED_SPI_MOSI_PIN);
        oled_spi_delay();
        DL_GPIO_setPins(OLED_SPI_PORT, OLED_SPI_CLOCK_PIN);
        oled_spi_delay();
        DL_GPIO_clearPins(OLED_SPI_PORT, OLED_SPI_CLOCK_PIN);
        data <<= 1;
    }
}

/* 拉低片选并设置 DC：true 为显存数据，false 为命令。 */
static void oled_begin_transfer(bool transfer_is_display_data)
{
    DL_GPIO_clearPins(OLED_SPI_PORT, OLED_CHIP_SELECT_PIN);
    if (transfer_is_display_data) DL_GPIO_setPins(OLED_SPI_PORT, OLED_DATA_COMMAND_PIN);
    else DL_GPIO_clearPins(OLED_SPI_PORT, OLED_DATA_COMMAND_PIN);
}

/* 拉高片选，结束本次软件 SPI 传输。 */
static void oled_end_transfer(void)
{
    DL_GPIO_setPins(OLED_SPI_PORT, OLED_CHIP_SELECT_PIN);
}

/* 向 SSD1306 发送一个控制命令。 */
static void oled_send_command(uint8_t command)
{
    oled_begin_transfer(false);
    oled_spi_write_byte(command);
    oled_end_transfer();
}
/* 查询 5×7 字体点阵；小写字母自动转换为大写。 */
static void get_font_columns(char character, uint8_t columns[5])
{
    memset(columns, 0, 5U);
    if (character >= 'a' && character <= 'z') character = (char)(character - ('a' - 'A'));
    if (character >= '0' && character <= '9') {
        memcpy(columns, digit_font[(uint8_t)(character - '0')], 5U);
        return;
    }
    if (character >= 'A' && character <= 'Z') {
        memcpy(columns, uppercase_font[(uint8_t)(character - 'A')], 5U);
        return;
    }
    switch (character) {
        case '-': columns[0]=0x08; columns[1]=0x08; columns[2]=0x08; columns[3]=0x08; columns[4]=0x08; break;
        case '+': columns[0]=0x08; columns[1]=0x08; columns[2]=0x3E; columns[3]=0x08; columns[4]=0x08; break;
        case '.': columns[1]=0x60; columns[2]=0x60; break;
        case ':': columns[1]=0x36; columns[2]=0x36; break;
        case '>': columns[0]=0x41; columns[1]=0x22; columns[2]=0x14; columns[3]=0x08; break;
        case '<': columns[1]=0x08; columns[2]=0x14; columns[3]=0x22; columns[4]=0x41; break;
        case '/': columns[0]=0x20; columns[1]=0x10; columns[2]=0x08; columns[3]=0x04; columns[4]=0x02; break;
        case '_': columns[0]=0x40; columns[1]=0x40; columns[2]=0x40; columns[3]=0x40; columns[4]=0x40; break;
        default: break;
    }
}

/* 复位并初始化 SSD1306，最后清空显示。 */
void OLED_Initialize(void)
{
    static const uint8_t initialization_commands[] = {
        0xAE,0x20,0x00,0xB0,0xC8,0x00,0x10,0x40,0x81,0x7F,0xA1,0xA6,
        0xA8,0x3F,0xA4,0xD3,0x00,0xD5,0x80,0xD9,0xF1,0xDA,0x12,0xDB,
        0x40,0x8D,0x14,0xAF
    };
    DL_GPIO_setPins(OLED_SPI_PORT, OLED_CHIP_SELECT_PIN);
    DL_GPIO_clearPins(OLED_SPI_PORT, OLED_SPI_CLOCK_PIN | OLED_SPI_MOSI_PIN | OLED_DATA_COMMAND_PIN);
    DL_GPIO_clearPins(OLED_SPI_PORT, OLED_RESET_PIN);
    delay_cycles(320000U);
    DL_GPIO_setPins(OLED_SPI_PORT, OLED_RESET_PIN);
    delay_cycles(3200000U);
    for (uint32_t command_index = 0U; command_index < sizeof(initialization_commands); command_index++) {
        oled_send_command(initialization_commands[command_index]);
    }
    OLED_ClearDisplayBuffer();
    next_refresh_page_index = 0U;
    OLED_RefreshDisplay();
}

/* 清空 MCU 内存中的 128×64 显示缓冲区。 */
void OLED_ClearDisplayBuffer(void)
{
    memset(display_buffer, 0, sizeof(display_buffer));
}

/* 设置缓冲区中的单个像素，越界坐标直接忽略。 */
void OLED_SetPixel(uint8_t x_coordinate, uint8_t y_coordinate, uint8_t pixel_is_on)
{
    if (x_coordinate >= OLED_WIDTH_PIXELS || y_coordinate >= OLED_HEIGHT_PIXELS) return;
    uint16_t buffer_index = (uint16_t)x_coordinate + ((uint16_t)(y_coordinate >> 3) * OLED_WIDTH_PIXELS);
    uint8_t pixel_mask = (uint8_t)(1U << (y_coordinate & 7U));
    if (pixel_is_on != 0U) display_buffer[buffer_index] |= pixel_mask;
    else display_buffer[buffer_index] &= (uint8_t)~pixel_mask;
}

/* 将一个 5×7 字符绘制到显示缓冲区。 */
void OLED_DrawCharacter(uint8_t x_coordinate, uint8_t y_coordinate, char character)
{
    uint8_t font_columns[5];
    get_font_columns(character, font_columns);
    for (uint8_t column_index = 0U; column_index < FONT_CHARACTER_WIDTH; column_index++) {
        for (uint8_t row_index = 0U; row_index < 7U; row_index++) {
            OLED_SetPixel((uint8_t)(x_coordinate + column_index), (uint8_t)(y_coordinate + row_index),
                (uint8_t)((font_columns[column_index] >> row_index) & 0x01U));
        }
    }
}

/* 连续绘制字符串，到屏幕右边界时自动停止。 */
void OLED_DrawString(uint8_t x_coordinate, uint8_t y_coordinate, const char *text)
{
    while (*text != '\0' && x_coordinate <= (OLED_WIDTH_PIXELS - FONT_CHARACTER_WIDTH)) {
        OLED_DrawCharacter(x_coordinate, y_coordinate, *text++);
        x_coordinate = (uint8_t)(x_coordinate + FONT_CHARACTER_WIDTH + FONT_CHARACTER_SPACING);
    }
}

/* 不使用 sprintf，直接把无符号整数绘制到缓冲区。 */
void OLED_DrawUnsignedInteger(uint8_t x_coordinate, uint8_t y_coordinate, uint32_t value)
{
    char digits[11];
    uint8_t digit_count = 0U;
    do {
        digits[digit_count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && digit_count < sizeof(digits));
    while (digit_count > 0U) {
        digit_count--;
        OLED_DrawCharacter(x_coordinate, y_coordinate, digits[digit_count]);
        x_coordinate = (uint8_t)(x_coordinate + FONT_CHARACTER_WIDTH + FONT_CHARACTER_SPACING);
    }
}

/* 绘制带符号浮点数，保留指定小数位数。 */
void OLED_DrawSignedFloat(uint8_t x_coordinate, uint8_t y_coordinate, float value, uint8_t decimal_places)
{
    if (value < 0.0f) {
        OLED_DrawCharacter(x_coordinate, y_coordinate, '-');
        value = -value;
    } else {
        OLED_DrawCharacter(x_coordinate, y_coordinate, '+');
    }
    x_coordinate = (uint8_t)(x_coordinate + FONT_CHARACTER_WIDTH + FONT_CHARACTER_SPACING);
    uint32_t scale = 1U;
    for (uint8_t decimal_index = 0U; decimal_index < decimal_places; decimal_index++) scale *= 10U;
    uint32_t scaled_value = (uint32_t)(value * (float)scale + 0.5f);
    OLED_DrawUnsignedInteger(x_coordinate, y_coordinate, scaled_value / scale);
    uint32_t integer_value = scaled_value / scale;
    do {
        x_coordinate = (uint8_t)(x_coordinate + FONT_CHARACTER_WIDTH + FONT_CHARACTER_SPACING);
        integer_value /= 10U;
    } while (integer_value != 0U);
    if (decimal_places == 0U) return;
    OLED_DrawCharacter(x_coordinate, y_coordinate, '.');
    x_coordinate = (uint8_t)(x_coordinate + FONT_CHARACTER_WIDTH + FONT_CHARACTER_SPACING);
    uint32_t fractional_value = scaled_value % scale;
    for (uint8_t decimal_index = decimal_places; decimal_index > 0U; decimal_index--) {
        uint32_t divisor = 1U;
        for (uint8_t power_index = 1U; power_index < decimal_index; power_index++) divisor *= 10U;
        OLED_DrawCharacter(x_coordinate, y_coordinate, (char)('0' + ((fractional_value / divisor) % 10U)));
        x_coordinate = (uint8_t)(x_coordinate + FONT_CHARACTER_WIDTH + FONT_CHARACTER_SPACING);
    }
}

/* 按 8 页发送完整显存；电机循迹运行期间不调用，避免干扰乱码。 */
void OLED_RefreshDisplay(void)
{
    for (uint8_t page_index = 0U; page_index < OLED_PAGE_COUNT; page_index++) {
        oled_send_command((uint8_t)(0xB0U + page_index));
        oled_send_command(0x00U);
        oled_send_command(0x10U);
        oled_begin_transfer(true);
        for (uint8_t x_coordinate = 0U; x_coordinate < OLED_WIDTH_PIXELS; x_coordinate++) {
            oled_spi_write_byte(display_buffer[(uint16_t)page_index * OLED_WIDTH_PIXELS + x_coordinate]);
        }
        oled_end_transfer();
    }
    next_refresh_page_index = 0U;
}

void OLED_RefreshNextPage(void)
{
    uint8_t page_index = next_refresh_page_index;
    oled_send_command((uint8_t)(0xB0U + page_index));
    oled_send_command(0x00U);
    oled_send_command(0x10U);
    oled_begin_transfer(true);
    for (uint8_t x_coordinate = 0U;
        x_coordinate < OLED_WIDTH_PIXELS; x_coordinate++) {
        oled_spi_write_byte(
            display_buffer[(uint16_t)page_index * OLED_WIDTH_PIXELS + x_coordinate]);
    }
    oled_end_transfer();
    next_refresh_page_index = (uint8_t)((page_index + 1U) % OLED_PAGE_COUNT);
}
