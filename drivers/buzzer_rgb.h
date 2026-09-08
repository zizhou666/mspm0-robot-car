/*
================================================================================
蜂鸣器与 RGB 状态指示接口模块
================================================================================
【功能简介】
本文件声明 PA27 蜂鸣器和 PB26/PB27/PB22 RGB 指示灯的统一控制接口。
- 蜂鸣器和三个颜色通道均为高电平有效，上电初始化后默认关闭。
- 支持蜂鸣器开、关、翻转和状态查询。
- 支持 RGB 三通道独立控制、常用颜色组合、熄灭和当前颜色查询。
- 所有控制函数只执行 GPIO 寄存器操作，不包含延时、循环等待或阻塞通信，
  可由主循环、周期任务或状态机实时调用。

================================================================================
【函数定义】
| 中文功能             | 函数名             | 作用 |
|----------------------|--------------------|------|
| 模块安全初始化       | BuzzerRGB_Init     | 再次关闭蜂鸣器和 RGB，并清零软件状态 |
| 设置蜂鸣器状态       | Buzzer_Set         | 根据布尔值立即打开或关闭蜂鸣器 |
| 打开蜂鸣器           | Buzzer_On          | 立即输出蜂鸣器有效电平 |
| 关闭蜂鸣器           | Buzzer_Off         | 立即输出蜂鸣器无效电平 |
| 翻转蜂鸣器状态       | Buzzer_Toggle      | 在开和关之间切换 |
| 查询蜂鸣器状态       | Buzzer_IsOn        | 返回驱动记录的当前状态 |
| 设置 RGB 三通道      | RGB_Set            | 分别指定红、绿、蓝是否点亮 |
| 设置预定义颜色       | RGB_SetColor       | 使用 RGB_Color_t 一次选择颜色组合 |
| 关闭全部 RGB         | RGB_Off            | 熄灭红、绿、蓝三个通道 |
| 查询当前 RGB 颜色    | RGB_GetColor       | 返回驱动记录的通道组合 |

================================================================================
【使用说明】
1. 必须先执行 SYSCFG_DL_init()，随后调用 BuzzerRGB_Init()；main.c 已完成此顺序。
2. 蜂鸣提示音的时长和 RGB 闪烁节拍应由系统毫秒时基或状态机控制，不要用阻塞延时。
3. 例如 RGB_SetColor(RGB_COLOR_GREEN) 点亮绿色，Buzzer_Set(true) 打开蜂鸣器。
4. 不要绕过本模块直接写对应 GPIO，否则 Buzzer_IsOn/RGB_GetColor 的软件状态会失真。
================================================================================
*/
#ifndef BUZZER_RGB_H_
#define BUZZER_RGB_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RGB_COLOR_OFF     = 0U,
    RGB_COLOR_RED     = (1U << 0),
    RGB_COLOR_GREEN   = (1U << 1),
    RGB_COLOR_BLUE    = (1U << 2),
    RGB_COLOR_YELLOW  = RGB_COLOR_RED | RGB_COLOR_GREEN,
    RGB_COLOR_MAGENTA = RGB_COLOR_RED | RGB_COLOR_BLUE,
    RGB_COLOR_CYAN    = RGB_COLOR_GREEN | RGB_COLOR_BLUE,
    RGB_COLOR_WHITE   = RGB_COLOR_RED | RGB_COLOR_GREEN | RGB_COLOR_BLUE
} RGB_Color_t;

void BuzzerRGB_Init(void);

void Buzzer_Set(bool enabled);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Toggle(void);
bool Buzzer_IsOn(void);
/* Non-blocking timed pulse overlaid on the normal buzzer request. */
void Buzzer_Pulse(uint32_t nowMs, uint32_t durationMs);
void Buzzer_Service(uint32_t nowMs);

void RGB_Set(bool redEnabled, bool greenEnabled, bool blueEnabled);
void RGB_SetColor(RGB_Color_t color);
void RGB_Off(void);
RGB_Color_t RGB_GetColor(void);

#endif /* BUZZER_RGB_H_ */
