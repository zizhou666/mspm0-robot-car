/*
================================================================================
128x64 OLED 软件 SPI 图形接口模块
================================================================================
【功能简介】
本文件声明 SSD1306/SH1106 兼容 OLED 的 GPIO 软件 SPI、帧缓冲、文字数字、
基础图形、旧工程兼容接口和非阻塞分片刷新接口。

================================================================================
【函数定义】
- OLED_GPIO_Init/OLED_Reset/OLED_Init：初始化 GPIO、复位并配置显示控制器。
- OLED_WriteCommand/OLED_WriteData/OLED_SetPosition：底层命令、数据和地址接口。
- OLED_DisplayOn/OLED_InvertDisplay/OLED_ShowStartupLogo：显示状态与启动图。
- OLED_ClearBuffer/OLED_Clear/OLED_Fill/OLED_ClearLine：帧缓冲清理与填充。
- OLED_ShowChar6x8/OLED_ShowString6x8：6x8 字符和字符串绘制。
- OLED_ShowChar8x16/OLED_ShowString8x16：8x16 字符和字符串绘制。
- OLED_ShowInt6x8/OLED_ShowFloat6x8：整数和浮点数绘制。
- OLED_DrawPixel/OLED_DrawLine：像素和任意直线绘制。
- OLED_DrawFastHLine/OLED_DrawFastVLine：快速水平/垂直线绘制。
- OLED_DrawRect/OLED_FillRect：矩形轮廓和填充绘制。
- OLED_DrawCircle/OLED_FillCircle：圆形轮廓和填充绘制。
- OLED_RequestRefresh/OLED_Process/OLED_IsBusy：非阻塞分片刷新状态机。
- OLED_RefreshBlocking/OLED_Refresh：同步刷新兼容接口。
- OLED_WrCmd/OLED_WrDat/OLED_Set_Pos/OLED_CLS：旧 OLED 底层兼容接口。
- LCD_Set_Pos/LCD_CLS/LCD_Fill/LCD_clear_L：旧 LCD 页面兼容接口。
- LCD_P6x8Char/LCD_P6x8Str/LCD_P8x16Char/LCD_P8x16Str：旧文字接口。
- write_6_8_number/write_6_8_number_f1/write_8_16_number：旧数字写入接口。
- display_6_8_number/display_6_8_number_pro/display_6_8_string：旧显示接口。
- Draw_Logo：把内置 LOGO 复制到帧缓冲。
- ssd1306_width/ssd1306_height/ssd1306_begin：尺寸和初始化兼容接口。
- ssd1306_command/ssd1306_data：底层命令和数据兼容接口。
- ssd1306_clear_display/ssd1306_display：清屏和刷新兼容接口。
- ssd1306_invert_display/ssd1306_dim：反显和对比度接口。
- ssd1306_stop_scroll/ssd1306_start_scroll_right/ssd1306_start_scroll_left：滚动接口。
- ssd1306_draw_pixel/ssd1306_draw_line：像素和直线兼容接口。
- ssd1306_draw_fast_hline/ssd1306_draw_fast_hline_internal：水平快速线接口。
- ssd1306_draw_fast_vline/ssd1306_draw_fast_vline_internal：垂直快速线接口。
- ssd1306_draw_rect/ssd1306_fill_rect/ssd1306_fill_screen：矩形和填充接口。
- ssd1306_draw_circle/ssd1306_fill_circle：圆形接口。
- ssd1306_draw_triangle/ssd1306_draw_bitmap：三角形和位图接口。
- draw_oled：请求一次帧缓冲刷新。

================================================================================
【使用说明】
1. 引脚来自 ti_msp_dl_config.h；不要在本文件硬编码端口号。
2. 正常 UI 使用 RequestRefresh + 高频 Process，避免在中断中整屏刷新。
3. 字符坐标 x 为像素列，page 为 0~7；绘图坐标使用 128x64 像素。
4. 列偏移、方向、亮度、SPI 延时和每次服务字节数在 user_config.h 中修改。
================================================================================
*/
#ifndef MSPM0G3507_OLED_H_
#define MSPM0G3507_OLED_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "user_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * MSPM0G3507 128x64 OLED driver
     * Interface: 4-wire software SPI plus reset (five GPIO outputs)
     * Controller: SSD1306 / SH1106-compatible module
     *
     * Pin ownership belongs to SysConfig.  The aliases below deliberately do
     * not contain fallback pin numbers, so a stale or incomplete generated
     * configuration fails at compile time instead of silently driving a wrong
     * board pin.
     */

#define OLED_WIDTH (128U)
#define OLED_HEIGHT (64U)
#define OLED_PAGE_COUNT (8U)

#ifndef OLED_CPU_CLOCK_HZ
#ifdef CPUCLK_FREQ
#define OLED_CPU_CLOCK_HZ (CPUCLK_FREQ)
#else
#define OLED_CPU_CLOCK_HZ (80000000UL)
#endif
#endif

#define OLED_SCL_PORT  OLED_PINS_SCL_PORT
#define OLED_SCL_PIN   OLED_PINS_SCL_PIN
#define OLED_SCL_IOMUX OLED_PINS_SCL_IOMUX

#define OLED_SDA_PORT  OLED_PINS_SDA_PORT
#define OLED_SDA_PIN   OLED_PINS_SDA_PIN
#define OLED_SDA_IOMUX OLED_PINS_SDA_IOMUX

#define OLED_RST_PORT  OLED_PINS_RST_PORT
#define OLED_RST_PIN   OLED_PINS_RST_PIN
#define OLED_RST_IOMUX OLED_PINS_RST_IOMUX

#define OLED_DC_PORT  OLED_PINS_DC_PORT
#define OLED_DC_PIN   OLED_PINS_DC_PIN
#define OLED_DC_IOMUX OLED_PINS_DC_IOMUX

#define OLED_CS_PORT  OLED_PINS_OLED_CS_PORT
#define OLED_CS_PIN   OLED_PINS_OLED_CS_PIN
#define OLED_CS_IOMUX OLED_PINS_OLED_CS_IOMUX

#define OLED_COLOR_BLACK (0U)
#define OLED_COLOR_WHITE (1U)
#define OLED_COLOR_INVERSE (2U)

#ifndef BLACK
#define BLACK OLED_COLOR_BLACK
#endif
#ifndef WHITE
#define WHITE OLED_COLOR_WHITE
#endif
#ifndef INVERSE
#define INVERSE OLED_COLOR_INVERSE
#endif

#define SSD1306_EXTERNALVCC (1U)
#define SSD1306_SWITCHCAPVCC (2U)

    /* Initialization and low-level access. */
    void OLED_GPIO_Init(void);
    void OLED_Reset(void);
    void OLED_Init(void);
    void OLED_WriteCommand(uint8_t command);
    void OLED_WriteData(uint8_t data);
    void OLED_SetPosition(uint8_t x, uint8_t page);
    void OLED_DisplayOn(uint8_t on);
    void OLED_InvertDisplay(uint8_t invert);
    void OLED_ShowStartupLogo(void);
    void OLED_ShowStartupLogoFrame(uint8_t frame, uint8_t progress_percent);

    /* Frame-buffer operations. Drawing functions take pixel coordinates. */
    void OLED_ClearBuffer(void);
    void OLED_RequestRefresh(void);
    void OLED_Process(void);
    uint8_t OLED_IsBusy(void);
    void OLED_RefreshBlocking(void);
    void OLED_Refresh(void);
    void OLED_Clear(void);
    void OLED_Fill(uint8_t pattern);
    void OLED_DrawPixel(int16_t x, int16_t y, uint8_t color);
    void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);
    void OLED_DrawFastHLine(int16_t x, int16_t y, int16_t width, uint8_t color);
    void OLED_DrawFastVLine(int16_t x, int16_t y, int16_t height, uint8_t color);
    void OLED_DrawRect(int16_t x, int16_t y, int16_t width, int16_t height, uint8_t color);
    void OLED_FillRect(int16_t x, int16_t y, int16_t width, int16_t height, uint8_t color);
    void OLED_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint8_t color);
    void OLED_FillCircle(int16_t x0, int16_t y0, int16_t radius, uint8_t color);

    /* Text coordinates: x is a pixel column; page is 0..7. */
    void OLED_ShowChar6x8(uint8_t x, uint8_t page, char character);
    void OLED_ShowString6x8(uint8_t x, uint8_t page, const char *text);
    void OLED_ShowChar8x16(uint8_t x, uint8_t page, char character);
    void OLED_ShowString8x16(uint8_t x, uint8_t page, const char *text);
    void OLED_ShowInt6x8(uint8_t x, uint8_t page, int32_t value);
    void OLED_ShowFloat6x8(uint8_t x, uint8_t page, float value, uint8_t decimals);
    void OLED_ClearLine(uint8_t x, uint8_t page);

    /* Compatibility interfaces retained from the source project. */
    void OLED_WrCmd(unsigned char data);
    void OLED_WrDat(unsigned char data);
    void OLED_Set_Pos(unsigned char x, unsigned char y);
    void OLED_CLS(void);
    void LCD_Set_Pos(unsigned char x, unsigned char y);
    void LCD_CLS(void);
    void LCD_Fill(unsigned char pattern);
    void LCD_clear_L(unsigned char x, unsigned char y);
    void LCD_P6x8Char(unsigned char x, unsigned char y, unsigned char ch);
    void LCD_P6x8Str(unsigned char x, unsigned char y, unsigned char ch[]);
    void LCD_P8x16Char(unsigned char x, unsigned char y, unsigned char ch);
    void LCD_P8x16Str(unsigned char x, unsigned char y, unsigned char ch[]);
    void write_6_8_number(unsigned char x, unsigned char y, float number);
    void write_6_8_number_f1(unsigned char x, unsigned char y, float number);
    void write_8_16_number(unsigned char x, unsigned char y, float number);
    void display_6_8_number(unsigned char x, unsigned char y, float number);
    void display_6_8_number_pro(unsigned char x, unsigned char y, float number);
    void display_6_8_string(unsigned char x, unsigned char y, char ch[]);
    void Draw_Logo(void);

    /* SSD1306 graphics compatibility used by the original ui.c. */
    int16_t ssd1306_width(void);
    int16_t ssd1306_height(void);
    void ssd1306_begin(uint8_t vccstate);
    void ssd1306_command(uint8_t command);
    void ssd1306_data(uint8_t data);
    void ssd1306_clear_display(void);
    void ssd1306_display(void);
    void ssd1306_invert_display(uint8_t invert);
    void ssd1306_dim(uint8_t dim);
    void ssd1306_stop_scroll(void);
    void ssd1306_start_scroll_right(uint8_t start, uint8_t stop);
    void ssd1306_start_scroll_left(uint8_t start, uint8_t stop);
    void ssd1306_draw_pixel(int16_t x, int16_t y, uint16_t color);
    void ssd1306_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
    void ssd1306_draw_fast_hline(int16_t x, int16_t y, int16_t width, uint16_t color);
    void ssd1306_draw_fast_hline_internal(int16_t x, int16_t y, int16_t width, uint16_t color);
    void ssd1306_draw_fast_vline(int16_t x, int16_t y, int16_t height, uint16_t color);
    void ssd1306_draw_fast_vline_internal(int16_t x, int16_t y, int16_t height, uint16_t color);
    void ssd1306_draw_rect(int16_t x, int16_t y, int16_t width, int16_t height, uint16_t color);
    void ssd1306_fill_rect(int16_t x, int16_t y, int16_t width, int16_t height, uint16_t color);
    void ssd1306_fill_screen(uint16_t color);
    void ssd1306_draw_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);
    void ssd1306_fill_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);
    void ssd1306_draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                               int16_t x2, int16_t y2, uint16_t color);
    void ssd1306_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap,
                             int16_t width, int16_t height, uint16_t color);
    void draw_oled(void);

#ifdef __cplusplus
}
#endif

#endif
