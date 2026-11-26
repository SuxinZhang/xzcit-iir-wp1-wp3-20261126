/**
 * @file bsp_oled.c
 * @brief Implementation of 4-wire software SPI SSD1306 OLED status BSP driver.
 *        四线软件 SPI SSD1306 OLED 状态显示屏板级驱动实现。
 */

#include "oled/bsp_oled.h"

#include "main.h"

#define OLED_CMD_MODE  0u
#define OLED_DATA_MODE 1u
#define OLED_COLS      128u
#define OLED_PAGES     8u

static uint8_t s_gram[OLED_PAGES][OLED_COLS];

/* Minimal 5x7 ASCII font subset (' ' through 'Z' and digits/symbols) for status display.
 * 用于状态显示的精简 5x7 ASCII 字库（覆盖空格、数字、符号及大写字母）。 */
static const uint8_t s_font_digits[12][5] = {
    {0x3Eu, 0x51u, 0x49u, 0x45u, 0x3Eu}, /* 0 */
    {0x00u, 0x42u, 0x7Fu, 0x40u, 0x00u}, /* 1 */
    {0x42u, 0x61u, 0x51u, 0x49u, 0x46u}, /* 2 */
    {0x21u, 0x41u, 0x45u, 0x4Bu, 0x31u}, /* 3 */
    {0x18u, 0x14u, 0x12u, 0x7Fu, 0x10u}, /* 4 */
    {0x27u, 0x45u, 0x45u, 0x45u, 0x39u}, /* 5 */
    {0x3Cu, 0x4Au, 0x49u, 0x49u, 0x30u}, /* 6 */
    {0x01u, 0x71u, 0x09u, 0x05u, 0x03u}, /* 7 */
    {0x36u, 0x49u, 0x49u, 0x49u, 0x36u}, /* 8 */
    {0x06u, 0x49u, 0x49u, 0x29u, 0x1Eu}, /* 9 */
    {0x00u, 0x60u, 0x60u, 0x00u, 0x00u}, /* . (index 10) */
    {0x08u, 0x08u, 0x08u, 0x08u, 0x08u}, /* - (index 11) */
};

static void oled_write_byte(uint8_t byte, uint8_t mode)
{
    HAL_GPIO_WritePin(IIR_OLED_DC_GPIO_Port,
                      IIR_OLED_DC_Pin,
                      (mode == OLED_DATA_MODE) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    for (uint8_t bit = 0u; bit < 8u; ++bit) {
        HAL_GPIO_WritePin(IIR_OLED_SCL_GPIO_Port, IIR_OLED_SCL_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IIR_OLED_SDA_GPIO_Port,
                          IIR_OLED_SDA_Pin,
                          ((byte & 0x80u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IIR_OLED_SCL_GPIO_Port, IIR_OLED_SCL_Pin, GPIO_PIN_SET);
        byte = (uint8_t)(byte << 1u);
    }

    HAL_GPIO_WritePin(IIR_OLED_SCL_GPIO_Port, IIR_OLED_SCL_Pin, GPIO_PIN_RESET);
}

static void oled_flush(void)
{
    for (uint8_t page = 0u; page < OLED_PAGES; ++page) {
        oled_write_byte((uint8_t)(0xB0u + page), OLED_CMD_MODE);
        oled_write_byte(0x00u, OLED_CMD_MODE);
        oled_write_byte(0x10u, OLED_CMD_MODE);
        for (uint8_t col = 0u; col < OLED_COLS; ++col) {
            oled_write_byte(s_gram[page][col], OLED_DATA_MODE);
        }
    }
}

static void oled_clear_buf(void)
{
    for (uint8_t page = 0u; page < OLED_PAGES; ++page) {
        for (uint8_t col = 0u; col < OLED_COLS; ++col) {
            s_gram[page][col] = 0u;
        }
    }
}

static void draw_glyph(uint8_t page, uint8_t col, const uint8_t glyph[5])
{
    if (page >= OLED_PAGES || (uint16_t)col + 5u > OLED_COLS) {
        return;
    }
    for (uint8_t i = 0u; i < 5u; ++i) {
        s_gram[page][col + i] = glyph[i];
    }
}

static uint8_t draw_uint(uint8_t page, uint8_t col, uint32_t value)
{
    uint8_t digits[10];
    uint8_t n = 0u;
    if (value == 0u) {
        digits[n++] = 0u;
    } else {
        while (value > 0u && n < 10u) {
            digits[n++] = (uint8_t)(value % 10u);
            value /= 10u;
        }
    }

    for (uint8_t i = 0u; i < n; ++i) {
        draw_glyph(page, col, s_font_digits[digits[n - 1u - i]]);
        col = (uint8_t)(col + 6u);
    }
    return col;
}

void BSP_Oled_Init(void)
{
    /* Pulse hardware reset pin during startup before FreeRTOS scheduler starts.
     * 在启动阶段产生硬件复位脉冲以初始化 SSD1306 显示控制器。 */
    HAL_GPIO_WritePin(IIR_OLED_RST_GPIO_Port, IIR_OLED_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20u);
    HAL_GPIO_WritePin(IIR_OLED_RST_GPIO_Port, IIR_OLED_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10u);

    static const uint8_t init_seq[] = {
        0xAEu, 0xD5u, 0x80u, 0xA8u, 0x3Fu, 0xD3u, 0x00u, 0x40u,
        0x8Du, 0x14u, 0x20u, 0x02u, 0xA1u, 0xC8u, 0xDAu, 0x12u,
        0x81u, 0xCFu, 0xD9u, 0xF1u, 0xDBu, 0x40u, 0xA4u, 0xA6u, 0xAFu
    };
    for (uint32_t i = 0u; i < (uint32_t)sizeof(init_seq); ++i) {
        oled_write_byte(init_seq[i], OLED_CMD_MODE);
    }

    oled_clear_buf();
    oled_flush();
}

void BSP_Oled_ShowStatus(uint16_t battery_mv, bool enabled, int16_t vx_mm_s)
{
    oled_clear_buf();

    /* Page 0: battery voltage in XX.XV format (e.g. 24.5).
     * 第 0 页：以 XX.X 形式显示当前电池电压（例如 24.5）。 */
    const uint32_t volts = battery_mv / 1000u;
    const uint32_t tenths = (battery_mv % 1000u) / 100u;
    uint8_t col = draw_uint(0u, 4u, volts);
    draw_glyph(0u, col, s_font_digits[10]);
    col = (uint8_t)(col + 6u);
    (void)draw_uint(0u, col, tenths);

    /* Page 2: motion gate status (1 = armed, 0 = stopped).
     * 第 2 页：显示当前运动使能状态（1 = 允许运动，0 = 安全停机）。 */
    (void)draw_uint(2u, 4u, enabled ? 1u : 0u);

    /* Page 4: signed forward speed in mm/s.
     * 第 4 页：显示当前有符号前进线速度（单位 mm/s）。 */
    col = 4u;
    uint32_t speed_abs;
    if (vx_mm_s < 0) {
        draw_glyph(4u, col, s_font_digits[11]);
        col = (uint8_t)(col + 6u);
        speed_abs = (uint32_t)(-(int32_t)vx_mm_s);
    } else {
        speed_abs = (uint32_t)vx_mm_s;
    }
    (void)draw_uint(4u, col, speed_abs);

    oled_flush();
}
