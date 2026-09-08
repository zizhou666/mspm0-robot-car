/*
================================================================================
分页键与五向按键接口模块
================================================================================
【功能简介】
本文件定义 PA18 上一页、PB21 下一页及五向按键的编号和事件位，支持非阻塞
消抖、方向键长按连发，以及中心键短按和长按互斥判定。

================================================================================
【函数定义】
- Buttons_Init：读取初始电平并初始化所有按键消抖状态。
- Buttons_Scan10ms：周期扫描、消抖并产生短按/长按事件。
- Buttons_GetAndClearEvents：原子地取出并清空待处理事件位。
- Buttons_IsPressed：读取指定按键的稳定按下状态。
- Buttons_IsInitialized：查询按键模块是否初始化成功。

================================================================================
【使用说明】
1. 按键均为上拉输入、按下接地，扫描周期由 user_config.h 指定。
2. 主循环每 10 ms 调用一次 Buttons_Scan10ms，不要加入阻塞延时。
3. PA18/PB21 一次按下只产生一个翻页事件，五向方向键另有独立连发事件位。
4. 中心键长按不会再产生短按事件，也不会产生方向键连发事件。
================================================================================
*/
#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BUTTON_ID_PAGE_PREV = 0,
    BUTTON_ID_PAGE_NEXT,
    BUTTON_ID_UP,
    BUTTON_ID_DOWN,
    BUTTON_ID_LEFT,
    BUTTON_ID_RIGHT,
    BUTTON_ID_CENTER,
    BUTTON_ID_COUNT
} ButtonId;

typedef enum {
    BUTTON_EVENT_NONE         = 0U,
    BUTTON_EVENT_PAGE_PREV    = (1U << BUTTON_ID_PAGE_PREV),
    BUTTON_EVENT_PAGE_NEXT    = (1U << BUTTON_ID_PAGE_NEXT),
    BUTTON_EVENT_UP           = (1U << BUTTON_ID_UP),
    BUTTON_EVENT_DOWN         = (1U << BUTTON_ID_DOWN),
    BUTTON_EVENT_LEFT         = (1U << BUTTON_ID_LEFT),
    BUTTON_EVENT_RIGHT        = (1U << BUTTON_ID_RIGHT),
    BUTTON_EVENT_CENTER       = (1U << BUTTON_ID_CENTER),
    /* Keep bit 7 aligned with UI_EVENT_CENTER_LONG. */
    BUTTON_EVENT_CENTER_LONG  = (1U << 7),
    BUTTON_EVENT_UP_REPEAT    = (1U << 8),
    BUTTON_EVENT_DOWN_REPEAT  = (1U << 9),
    BUTTON_EVENT_LEFT_REPEAT  = (1U << 10),
    BUTTON_EVENT_RIGHT_REPEAT = (1U << 11)
} ButtonEvent;

bool Buttons_Init(void);

/*
 * Call once every 10 ms. Page keys queue only one press-edge event. Five-way
 * direction keys queue one edge event plus delayed periodic repeat events.
 * CENTER queues a short event on release or one long event at the configured
 * threshold, never both for the same press and never repeats.
 */
void Buttons_Scan10ms(void);
uint32_t Buttons_GetAndClearEvents(void);
bool Buttons_IsPressed(ButtonId id);
bool Buttons_IsInitialized(void);

#endif /* BUTTONS_H_ */
