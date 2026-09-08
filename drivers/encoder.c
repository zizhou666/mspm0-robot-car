/*
================================================================================
左右轮 A/B 双相双边沿四倍频编码器实现模块
================================================================================
【功能简介】
本文件在 GPIOB 中断中分别累计左右编码器 A/B 双相双边沿脉冲，并用状态跳变判断
方向。主循环按实际采样周期原子截取增量，计算两轮速度、累计路程和车体里程。

================================================================================
【函数定义】
- Encoder_EnterCritical/Encoder_ExitCritical：保护多字节中断共享计数。
- Encoder_QuadratureStep：依据 A/B 上一状态和当前状态返回 +1/-1。
- Encoder_SaturatingStep：累计计数到 int32_t 边界时饱和而不溢出。
- Encoder_Init：清零左右累计/窗口计数并使能编码器 GPIO 中断。
- Encoder_HandleGpioInterruptIID：更新对应左或右 A/B 相的四倍频计数器。
- Encoder_SamplePeriodMs：截取窗口计数并换算速度、单轮路程和平均里程。
- Encoder_Sample10ms：按默认控制周期调用通用采样函数。
- Encoder_GetSnapshot：在临界区内复制完整编码器快照。
- Encoder_ResetCounts：清零全部位置、窗口、速度和路程状态。
- Encoder_IsInitialized：返回模块初始化标志。

================================================================================
【使用说明】
1. 左右计数通道不得交叉；当前左 A/B=PB5/PB7，右 A/B=PB4/PB6。
2. 中断中只计数，不进行浮点运算、OLED 刷屏或总线通信。
3. 每圈计数、轮径、左右极性在 user_config.h 中标定。
4. 本次注释和参数集中化不改变编码器计数、中断或速度计算逻辑。
================================================================================
*/
#include "encoder.h"

#include <limits.h>
#include <stddef.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

/* Written in GPIO ISR context and read/reset by Encoder_Sample10ms(). */
static volatile int32_t g_leftPositionCounts;
static volatile int32_t g_rightPositionCounts;
static volatile int32_t g_leftWindowCounts;
static volatile int32_t g_rightWindowCounts;
static volatile uint8_t g_leftPreviousState;
static volatile uint8_t g_rightPreviousState;

static EncoderSnapshot g_snapshot;
static volatile bool g_initialized;

static uint32_t Encoder_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Encoder_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

static uint8_t Encoder_StateFromAB(
    uint32_t pins, uint32_t aPin, uint32_t bPin)
{
    uint8_t state = 0U;

    if ((pins & aPin) != 0U) {
        state |= 2U;
    }
    if ((pins & bPin) != 0U) {
        state |= 1U;
    }

    return state;
}

static int32_t Encoder_QuadratureStep(
    uint8_t previousState, uint8_t currentState, int32_t polarity)
{
    static const int8_t stepTable[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    return ((int32_t)stepTable[((previousState & 3U) << 2U) |
                               (currentState & 3U)]) * polarity;
}

static int32_t Encoder_SaturatingStep(int32_t position, int32_t step)
{
    if ((step > 0) && (position < INT32_MAX)) {
        return position + 1;
    }
    if ((step < 0) && (position > INT32_MIN)) {
        return position - 1;
    }
    return position;
}

bool Encoder_Init(void)
{
    uint32_t pins = DL_GPIO_readPins(
        ENCODER_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN |
                      ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
    uint32_t primask = Encoder_EnterCritical();
    g_leftPositionCounts  = 0;
    g_rightPositionCounts = 0;
    g_leftWindowCounts    = 0;
    g_rightWindowCounts   = 0;
    g_leftPreviousState = Encoder_StateFromAB(
        pins, ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    g_rightPreviousState = Encoder_StateFromAB(
        pins, ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    g_snapshot.leftPositionCounts  = 0;
    g_snapshot.rightPositionCounts = 0;
    g_snapshot.leftDeltaCounts     = 0;
    g_snapshot.rightDeltaCounts    = 0;
    g_snapshot.leftSpeedCmS        = 0.0f;
    g_snapshot.rightSpeedCmS       = 0.0f;
    g_snapshot.leftDistanceCm      = 0.0f;
    g_snapshot.rightDistanceCm     = 0.0f;
    g_snapshot.vehicleDistanceCm   = 0.0f;
    g_snapshot.samplePeriodMs      = APP_CONTROL_PERIOD_MS;
    g_snapshot.initialized         = true;
    g_initialized                  = true;
    Encoder_ExitCritical(primask);

    DL_GPIO_clearInterruptStatus(
        ENCODER_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN |
                      ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
    return true;
}

void Encoder_HandleGpioInterruptIID(uint32_t iid)
{
    if (!g_initialized) {
        return;
    }

    if ((iid == (uint32_t) ENCODER_LEFT_A_IIDX) ||
        (iid == (uint32_t) ENCODER_LEFT_B_IIDX)) {
        uint32_t pins = DL_GPIO_readPins(
            ENCODER_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN);
        uint8_t currentState = Encoder_StateFromAB(
            pins, ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
        int32_t step = Encoder_QuadratureStep(
            g_leftPreviousState, currentState, APP_ENCODER_LEFT_POLARITY);

        g_leftPreviousState = currentState;
        if (step != 0) {
            g_leftPositionCounts =
                Encoder_SaturatingStep(g_leftPositionCounts, step);
            g_leftWindowCounts =
                Encoder_SaturatingStep(g_leftWindowCounts, step);
        }
    } else if ((iid == (uint32_t) ENCODER_RIGHT_A_IIDX) ||
               (iid == (uint32_t) ENCODER_RIGHT_B_IIDX)) {
        uint32_t pins = DL_GPIO_readPins(
            ENCODER_PORT, ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
        uint8_t currentState = Encoder_StateFromAB(
            pins, ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
        int32_t step = Encoder_QuadratureStep(
            g_rightPreviousState, currentState, APP_ENCODER_RIGHT_POLARITY);

        g_rightPreviousState = currentState;
        if (step != 0) {
            g_rightPositionCounts =
                Encoder_SaturatingStep(g_rightPositionCounts, step);
            g_rightWindowCounts =
                Encoder_SaturatingStep(g_rightWindowCounts, step);
        }
    } else {
        /* The shared Group-1 ISR may dispatch unrelated GPIOB IIDX values. */
    }
}

void Encoder_SamplePeriodMs(uint32_t periodMs)
{
    int32_t leftDelta;
    int32_t rightDelta;
    int32_t leftPosition;
    int32_t rightPosition;

    uint32_t primask = Encoder_EnterCritical();
    leftDelta          = g_leftWindowCounts;
    rightDelta         = g_rightWindowCounts;
    leftPosition       = g_leftPositionCounts;
    rightPosition      = g_rightPositionCounts;
    g_leftWindowCounts = 0;
    g_rightWindowCounts = 0;
    Encoder_ExitCritical(primask);

    if (periodMs == 0U) {
        periodMs = APP_CONTROL_PERIOD_MS;
    }

    const float periodSeconds = ((float)periodMs) * 0.001f;
    const float countsToCm = APP_WHEEL_CIRCUMFERENCE_CM /
        APP_ENCODER_COUNTS_PER_REV;
    const float countsToCmS = APP_WHEEL_CIRCUMFERENCE_CM /
        (APP_ENCODER_COUNTS_PER_REV * periodSeconds);

    g_snapshot.leftPositionCounts  = leftPosition;
    g_snapshot.rightPositionCounts = rightPosition;
    g_snapshot.leftDeltaCounts     = leftDelta;
    g_snapshot.rightDeltaCounts    = rightDelta;
    g_snapshot.leftSpeedCmS        = ((float) leftDelta) * countsToCmS;
    g_snapshot.rightSpeedCmS       = ((float) rightDelta) * countsToCmS;
    g_snapshot.leftDistanceCm      = ((float)leftPosition) * countsToCm;
    g_snapshot.rightDistanceCm     = ((float)rightPosition) * countsToCm;
    g_snapshot.vehicleDistanceCm   =
        0.5f * (g_snapshot.leftDistanceCm +
                g_snapshot.rightDistanceCm);
    g_snapshot.samplePeriodMs      = periodMs;
    g_snapshot.initialized         = g_initialized;
}

void Encoder_Sample10ms(void)
{
    Encoder_SamplePeriodMs(APP_CONTROL_PERIOD_MS);
}

void Encoder_GetSnapshot(EncoderSnapshot *snapshot)
{
    if (snapshot != NULL) {
        *snapshot = g_snapshot;
    }
}

void Encoder_ResetCounts(void)
{
    uint32_t pins = DL_GPIO_readPins(
        ENCODER_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN |
                      ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
    uint32_t primask = Encoder_EnterCritical();
    g_leftPositionCounts  = 0;
    g_rightPositionCounts = 0;
    g_leftWindowCounts    = 0;
    g_rightWindowCounts   = 0;
    g_leftPreviousState = Encoder_StateFromAB(
        pins, ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    g_rightPreviousState = Encoder_StateFromAB(
        pins, ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    Encoder_ExitCritical(primask);

    g_snapshot.leftPositionCounts  = 0;
    g_snapshot.rightPositionCounts = 0;
    g_snapshot.leftDeltaCounts     = 0;
    g_snapshot.rightDeltaCounts    = 0;
    g_snapshot.leftSpeedCmS        = 0.0f;
    g_snapshot.rightSpeedCmS       = 0.0f;
    g_snapshot.leftDistanceCm      = 0.0f;
    g_snapshot.rightDistanceCm     = 0.0f;
    g_snapshot.vehicleDistanceCm   = 0.0f;
}

bool Encoder_IsInitialized(void)
{
    return g_initialized;
}
