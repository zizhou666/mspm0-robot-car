/*
================================================================================
分页键与五向按键扫描实现模块
================================================================================
【功能简介】
本文件按固定周期读取七个低电平有效按键，使用连续样本消抖。五向上、下、左、
右首次按下产生边沿事件，按住达到等待时间后产生固定周期连发事件；中心键在
释放时产生短按，达到长按阈值时产生保存事件，二者不会重复触发。

================================================================================
【函数定义】
- Buttons_ReadPressed：读取一个上拉、按下接地按键的即时状态。
- Buttons_Init：初始化各键稳定值、候选值、计数器和事件队列。
- Buttons_RepeatEventForId：把五向方向键编号转换为独立的连发事件位。
- Buttons_Scan10ms：执行消抖、按下边沿、方向键连发和中心键短/长按判定。
- Buttons_GetAndClearEvents：在临界区中取出并清空累计事件。
- Buttons_IsPressed：返回指定按键的消抖后稳定状态。
- Buttons_IsInitialized：返回模块初始化状态。

================================================================================
【使用说明】
1. 调用周期必须与 APP_BUTTON_SCAN_PERIOD_MS 一致。
2. PA18/PB21 只产生上一页/下一页事件；长按不会连续快速翻页。
3. 只有五向上/下/左/右产生连发位，第六页电机测试可忽略这些连发位。
4. 消抖、连发等待/间隔和中心键长按时间均在 user_config.h 中修改。
================================================================================
*/
#include "buttons.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

typedef struct {
    uint8_t stablePressed;
    uint8_t candidatePressed;
    uint8_t equalSamples;
    uint8_t longEventSent;
    uint16_t heldSamples;
    uint16_t repeatCountdown;
} ButtonDebounce;

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} ButtonPin;

static const ButtonPin g_buttonPins[BUTTON_ID_COUNT] = {
    {PAGE_PREV_PORT, PAGE_PREV_PREV_PIN},
    {PAGE_NEXT_PORT, PAGE_NEXT_NEXT_PIN},
    {KEY5_PORT, KEY5_UP_PIN},
    {KEY5_PORT, KEY5_DOWN_PIN},
    {KEY5_PORT, KEY5_LEFT_PIN},
    {KEY5_PORT, KEY5_RIGHT_PIN},
    {KEY5_PORT, KEY5_CENTER_PIN},
};

static ButtonDebounce g_buttons[BUTTON_ID_COUNT];
static uint32_t g_pendingEvents;
static bool g_initialized;

static uint8_t Buttons_ReadPressed(const ButtonPin *buttonPin)
{
    /* All page and five-way inputs use pull-ups and close to ground. */
    return ((DL_GPIO_readPins(buttonPin->port, buttonPin->pin) &
             buttonPin->pin) == 0U) ? 1U : 0U;
}

static uint32_t Buttons_RepeatEventForId(uint32_t id)
{
    switch ((ButtonId)id) {
        case BUTTON_ID_UP:
            return BUTTON_EVENT_UP_REPEAT;
        case BUTTON_ID_DOWN:
            return BUTTON_EVENT_DOWN_REPEAT;
        case BUTTON_ID_LEFT:
            return BUTTON_EVENT_LEFT_REPEAT;
        case BUTTON_ID_RIGHT:
            return BUTTON_EVENT_RIGHT_REPEAT;
        default:
            return BUTTON_EVENT_NONE;
    }
}

bool Buttons_Init(void)
{
    for (uint32_t i = 0U; i < (uint32_t) BUTTON_ID_COUNT; ++i) {
        uint8_t pressed = Buttons_ReadPressed(&g_buttonPins[i]);
        g_buttons[i].stablePressed    = pressed;
        g_buttons[i].candidatePressed = pressed;
        g_buttons[i].equalSamples     = APP_BUTTON_DEBOUNCE_SAMPLES;
        g_buttons[i].longEventSent    = 0U;
        g_buttons[i].heldSamples      = 0U;
        g_buttons[i].repeatCountdown  =
            ((pressed != 0U) &&
             (i >= (uint32_t)BUTTON_ID_UP) &&
             (i <= (uint32_t)BUTTON_ID_RIGHT)) ?
                APP_BUTTON_REPEAT_DELAY_SAMPLES : 0U;
    }
    g_pendingEvents = BUTTON_EVENT_NONE;
    g_initialized   = true;
    return true;
}

void Buttons_Scan10ms(void)
{
    if (!g_initialized) {
        return;
    }

    for (uint32_t i = 0U; i < (uint32_t) BUTTON_ID_COUNT; ++i) {
        uint8_t pressed = Buttons_ReadPressed(&g_buttonPins[i]);
        uint8_t stableChanged = 0U;
        ButtonDebounce *button = &g_buttons[i];

        if (pressed == button->candidatePressed) {
            if (button->equalSamples < APP_BUTTON_DEBOUNCE_SAMPLES) {
                ++button->equalSamples;
            }
        } else {
            button->candidatePressed = pressed;
            button->equalSamples     = 1U;
        }

        if ((button->equalSamples >= APP_BUTTON_DEBOUNCE_SAMPLES) &&
            (button->stablePressed != button->candidatePressed)) {
            button->stablePressed = button->candidatePressed;
            stableChanged = 1U;
            if (button->stablePressed != 0U) {
                button->heldSamples = 0U;
                button->longEventSent = 0U;
                button->repeatCountdown =
                    ((i >= (uint32_t)BUTTON_ID_UP) &&
                     (i <= (uint32_t)BUTTON_ID_RIGHT)) ?
                        APP_BUTTON_REPEAT_DELAY_SAMPLES : 0U;
                if (i != (uint32_t)BUTTON_ID_CENTER) {
                    /* The normal edge event is immediate; repeats use other bits. */
                    g_pendingEvents |= (1UL << i);
                }
            } else {
                if ((i == (uint32_t)BUTTON_ID_CENTER) &&
                    (button->longEventSent == 0U)) {
                    g_pendingEvents |= BUTTON_EVENT_CENTER;
                }
                button->heldSamples = 0U;
                button->longEventSent = 0U;
                button->repeatCountdown = 0U;
            }
        }

        if ((i >= (uint32_t)BUTTON_ID_UP) &&
            (i <= (uint32_t)BUTTON_ID_RIGHT) &&
            (button->stablePressed != 0U) &&
            (stableChanged == 0U)) {
            if (button->repeatCountdown > 0U) {
                --button->repeatCountdown;
            }
            if (button->repeatCountdown == 0U) {
                g_pendingEvents |= Buttons_RepeatEventForId(i);
                button->repeatCountdown = APP_BUTTON_REPEAT_PERIOD_SAMPLES;
            }
        } else if ((i == (uint32_t)BUTTON_ID_CENTER) &&
            (button->stablePressed != 0U) &&
            (button->longEventSent == 0U)) {
            if (button->heldSamples < UINT16_MAX) {
                ++button->heldSamples;
            }
            if (button->heldSamples >= APP_BUTTON_LONG_PRESS_SAMPLES) {
                button->longEventSent = 1U;
                g_pendingEvents |= BUTTON_EVENT_CENTER_LONG;
            }
        }
    }
}

uint32_t Buttons_GetAndClearEvents(void)
{
    uint32_t events = g_pendingEvents;
    g_pendingEvents = BUTTON_EVENT_NONE;
    return events;
}

bool Buttons_IsPressed(ButtonId id)
{
    if ((!g_initialized) || ((uint32_t) id >= (uint32_t) BUTTON_ID_COUNT)) {
        return false;
    }
    return (g_buttons[id].stablePressed != 0U);
}

bool Buttons_IsInitialized(void)
{
    return g_initialized;
}
