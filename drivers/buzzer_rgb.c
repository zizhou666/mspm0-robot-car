/*
================================================================================
蜂鸣器与 RGB 状态指示驱动模块
================================================================================
【功能简介】
本文件实现 PA27 蜂鸣器和 PB26/PB27/PB22 RGB 指示灯的非阻塞 GPIO 控制。
- PA27 为蜂鸣器，高电平发声、低电平关闭。
- PB26、PB27、PB22 分别为红、绿、蓝，高电平点亮、低电平熄灭。
- 初始化始终先关闭全部输出，避免系统启动时误鸣或误亮。
- 接口不使用动态内存、延时和总线访问，单次调用只执行少量 GPIO 操作。

================================================================================
【函数定义】
| 中文功能             | 函数名             | 作用 |
|----------------------|--------------------|------|
| 模块安全初始化       | BuzzerRGB_Init     | 关闭所有输出并复位软件状态 |
| 设置蜂鸣器状态       | Buzzer_Set         | 写 PA27 并更新状态 |
| 打开/关闭/翻转蜂鸣器 | Buzzer_On/Off/Toggle | 提供便于实时调用的快捷接口 |
| 查询蜂鸣器状态       | Buzzer_IsOn        | 返回当前软件状态 |
| 设置 RGB 三通道      | RGB_Set            | 把三个布尔值转换为颜色掩码 |
| 设置预定义颜色       | RGB_SetColor       | 写 PB26/PB27/PB22 并更新状态 |
| 关闭全部 RGB         | RGB_Off            | 选择 RGB_COLOR_OFF |
| 查询当前 RGB 颜色    | RGB_GetColor       | 返回当前颜色掩码 |

================================================================================
【使用说明】
1. 本文件依赖 ti_msp_dl_config.h 中由 main.syscfg 生成的引脚宏。
2. main.c 在 SYSCFG_DL_init() 后调用 BuzzerRGB_Init()，应用层随后可直接调用各接口。
3. 若要实现 100 ms 蜂鸣或周期闪灯，请在调度器中按时间调用开/关接口，不要在本驱动加延时。
4. RGB_COLOR_* 是位掩码，可使用预定义颜色，也可把有效通道组合后传给 RGB_SetColor。
================================================================================
*/
#include "buzzer_rgb.h"

#include "ti_msp_dl_config.h"

static bool g_buzzerEnabled;
static bool g_buzzerRequested;
static bool g_buzzerPulseActive;
static uint32_t g_buzzerPulseStartedMs;
static uint32_t g_buzzerPulseDurationMs;
static RGB_Color_t g_rgbColor;

static void Buzzer_ApplyOutput(void)
{
    bool enabled = g_buzzerRequested || g_buzzerPulseActive;

    if (enabled) {
        DL_GPIO_setPins(BUZZER_PIN_PORT, BUZZER_PIN_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(BUZZER_PIN_PORT, BUZZER_PIN_BUZZER_PIN);
    }
    g_buzzerEnabled = enabled;
}

void BuzzerRGB_Init(void)
{
    DL_GPIO_clearPins(BUZZER_PIN_PORT, BUZZER_PIN_BUZZER_PIN);
    DL_GPIO_clearPins(RGB_PINS_PORT,
                      RGB_PINS_RED_PIN |
                      RGB_PINS_GREEN_PIN |
                      RGB_PINS_BLUE_PIN);

    g_buzzerEnabled = false;
    g_buzzerRequested = false;
    g_buzzerPulseActive = false;
    g_buzzerPulseStartedMs = 0U;
    g_buzzerPulseDurationMs = 0U;
    g_rgbColor = RGB_COLOR_OFF;
}

void Buzzer_Set(bool enabled)
{
    g_buzzerRequested = enabled;
    Buzzer_ApplyOutput();
}

void Buzzer_On(void)
{
    Buzzer_Set(true);
}

void Buzzer_Off(void)
{
    Buzzer_Set(false);
}

void Buzzer_Toggle(void)
{
    Buzzer_Set(!g_buzzerRequested);
}

bool Buzzer_IsOn(void)
{
    return g_buzzerEnabled;
}

void Buzzer_Pulse(uint32_t nowMs, uint32_t durationMs)
{
    if (durationMs == 0U) {
        return;
    }

    g_buzzerPulseStartedMs = nowMs;
    g_buzzerPulseDurationMs = durationMs;
    g_buzzerPulseActive = true;
    Buzzer_ApplyOutput();
}

void Buzzer_Service(uint32_t nowMs)
{
    if (g_buzzerPulseActive &&
        ((uint32_t)(nowMs - g_buzzerPulseStartedMs) >=
         g_buzzerPulseDurationMs)) {
        g_buzzerPulseActive = false;
        Buzzer_ApplyOutput();
    }
}

void RGB_Set(bool redEnabled, bool greenEnabled, bool blueEnabled)
{
    uint32_t color = (uint32_t)RGB_COLOR_OFF;

    if (redEnabled) {
        color |= (uint32_t)RGB_COLOR_RED;
    }
    if (greenEnabled) {
        color |= (uint32_t)RGB_COLOR_GREEN;
    }
    if (blueEnabled) {
        color |= (uint32_t)RGB_COLOR_BLUE;
    }

    RGB_SetColor((RGB_Color_t)color);
}

void RGB_SetColor(RGB_Color_t color)
{
    uint32_t requested = ((uint32_t)color & (uint32_t)RGB_COLOR_WHITE);
    uint32_t enabledPins = 0U;
    const uint32_t allPins = RGB_PINS_RED_PIN |
                             RGB_PINS_GREEN_PIN |
                             RGB_PINS_BLUE_PIN;

    if ((requested & (uint32_t)RGB_COLOR_RED) != 0U) {
        enabledPins |= RGB_PINS_RED_PIN;
    }
    if ((requested & (uint32_t)RGB_COLOR_GREEN) != 0U) {
        enabledPins |= RGB_PINS_GREEN_PIN;
    }
    if ((requested & (uint32_t)RGB_COLOR_BLUE) != 0U) {
        enabledPins |= RGB_PINS_BLUE_PIN;
    }

    DL_GPIO_clearPins(RGB_PINS_PORT, allPins & ~enabledPins);
    if (enabledPins != 0U) {
        DL_GPIO_setPins(RGB_PINS_PORT, enabledPins);
    }

    g_rgbColor = (RGB_Color_t)requested;
}

void RGB_Off(void)
{
    RGB_SetColor(RGB_COLOR_OFF);
}

RGB_Color_t RGB_GetColor(void)
{
    return g_rgbColor;
}
