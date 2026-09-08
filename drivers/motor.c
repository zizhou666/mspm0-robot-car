/*
================================================================================
TB6612 双电机驱动实现模块
================================================================================
【功能简介】
本文件把有符号左右电机输出映射到 TB6612 的方向输入和速度 PWM。左电机使用
PA15/TIMA1_CCP0 作为 PWMA，PA7/PB1 作为 AIN1/AIN2；右电机使用 PA3/TIMA0_CCP1
作为 PWMB，PB2/PB3 作为 BIN1/BIN2。

================================================================================
【函数定义】
- Motor_ClampOutput：把电机命令限制在允许的正负 PWM 范围。
- Motor_OutputMagnitude：把有符号命令转换为无符号占空计数。
- Motor_SetCompare：写入指定 TIMA0 比较通道。
- Motor_ApplyPhysicalOutputs：按两 PWM 真值表驱动左右物理 H 桥。
- Motor_Init：输出全零、启动 PWM 计数器并置初始化标志。
- Motor_SetOutputs：应用左右逻辑极性后输出到电机。
- Motor_SetPhysicalOutputs：不经过逻辑极性直接驱动 A/B 测试通道。
- Motor_Stop：把左右两侧输出均置零。
- Motor_GetState：复制当前输出、占空计数和初始化状态。
- Motor_IsInitialized：返回电机驱动初始化状态。

================================================================================
【使用说明】
1. 正常闭环只调用 Motor_SetOutputs；第六页点动才调用物理通道接口。
2. 输出 0 会把对应 H 桥两个输入都置低，不启用锁存或动态制动。
3. 电机极性和限幅在 user_config.h 修改；四个引脚在 main.syscfg 修改。
================================================================================
*/
#include "motor.h"

#include <stddef.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

/*
 * TB6612 mapping:
 *
 *   left  PWM: PA15 / TIMA1_CCP0 = PWMA
 *   left  dir: PA7 = AIN1, PB1 = AIN2
 *   right PWM: PA3  / TIMA0_CCP1 = PWMB
 *   right dir: PB2 = BIN1, PB3 = BIN2
 *
 * Positive output drives IN1 high and IN2 low. Negative output reverses the
 * two direction pins. A zero command sets IN1/IN2 low and the PWM duty to 0.
 * TB6612 STBY is expected to be tied high on the board or controlled outside
 * this module because the provided pin map does not include a standby pin.
 */
static MotorState g_motorState;

static int16_t Motor_ClampOutput(int32_t output)
{
    if (output > APP_MOTOR_OUTPUT_LIMIT) {
        return (int16_t) APP_MOTOR_OUTPUT_LIMIT;
    }
    if (output < -APP_MOTOR_OUTPUT_LIMIT) {
        return (int16_t) -APP_MOTOR_OUTPUT_LIMIT;
    }
    return (int16_t) output;
}

static uint16_t Motor_OutputMagnitude(int16_t output)
{
    return (output < 0) ? (uint16_t) (-output) : (uint16_t) output;
}

static void Motor_SetLeftPwm(uint16_t duty)
{
    DL_TimerA_setCaptureCompareValue(PWM_1_INST, duty, GPIO_PWM_1_C0_IDX);
}

static void Motor_SetRightPwm(uint16_t duty)
{
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, duty, GPIO_PWM_0_C1_IDX);
}

static void Motor_SetLeftDirection(int16_t output)
{
    if (output > 0) {
        DL_GPIO_clearPins(TB6612_DIR_AIN2_PORT, TB6612_DIR_AIN2_PIN);
        DL_GPIO_setPins(TB6612_DIR_AIN1_PORT, TB6612_DIR_AIN1_PIN);
    } else if (output < 0) {
        DL_GPIO_clearPins(TB6612_DIR_AIN1_PORT, TB6612_DIR_AIN1_PIN);
        DL_GPIO_setPins(TB6612_DIR_AIN2_PORT, TB6612_DIR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(TB6612_DIR_AIN1_PORT, TB6612_DIR_AIN1_PIN);
        DL_GPIO_clearPins(TB6612_DIR_AIN2_PORT, TB6612_DIR_AIN2_PIN);
    }
}

static void Motor_SetRightDirection(int16_t output)
{
    if (output > 0) {
        DL_GPIO_clearPins(TB6612_DIR_BIN2_PORT, TB6612_DIR_BIN2_PIN);
        DL_GPIO_setPins(TB6612_DIR_BIN1_PORT, TB6612_DIR_BIN1_PIN);
    } else if (output < 0) {
        DL_GPIO_clearPins(TB6612_DIR_BIN1_PORT, TB6612_DIR_BIN1_PIN);
        DL_GPIO_setPins(TB6612_DIR_BIN2_PORT, TB6612_DIR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(TB6612_DIR_BIN1_PORT, TB6612_DIR_BIN1_PIN);
        DL_GPIO_clearPins(TB6612_DIR_BIN2_PORT, TB6612_DIR_BIN2_PIN);
    }
}

static void Motor_ApplyBrake(void)
{
    /*
     * TB6612 short brake: PWM high and both direction inputs high.
     * Drop PWM first so changing the direction inputs cannot create a
     * powered reversal pulse.
     */
    Motor_SetLeftPwm(0U);
    Motor_SetRightPwm(0U);

    DL_GPIO_setPins(TB6612_DIR_AIN1_PORT, TB6612_DIR_AIN1_PIN);
    DL_GPIO_setPins(TB6612_DIR_AIN2_PORT, TB6612_DIR_AIN2_PIN);
    DL_GPIO_setPins(TB6612_DIR_BIN1_PORT, TB6612_DIR_BIN1_PIN);
    DL_GPIO_setPins(TB6612_DIR_BIN2_PORT, TB6612_DIR_BIN2_PIN);

    Motor_SetLeftPwm(APP_MOTOR_BRAKE_DUTY_COUNTS);
    Motor_SetRightPwm(APP_MOTOR_BRAKE_DUTY_COUNTS);

    g_motorState.leftOutput = 0;
    g_motorState.rightOutput = 0;
    g_motorState.leftDutyCounts = 0U;
    g_motorState.rightDutyCounts = 0U;
}

static void Motor_ApplyPhysicalOutputs(int16_t leftOutput,
    int16_t rightOutput)
{
    uint16_t leftDuty = Motor_OutputMagnitude(leftOutput);
    uint16_t rightDuty = Motor_OutputMagnitude(rightOutput);

    Motor_SetLeftPwm(0U);
    Motor_SetRightPwm(0U);
    Motor_SetLeftDirection(leftOutput);
    Motor_SetRightDirection(rightOutput);
    Motor_SetLeftPwm(leftDuty);
    Motor_SetRightPwm(rightDuty);

    g_motorState.leftOutput = leftOutput;
    g_motorState.rightOutput = rightOutput;
    g_motorState.leftDutyCounts = leftDuty;
    g_motorState.rightDutyCounts = rightDuty;
}

bool Motor_Init(void)
{
    Motor_ApplyPhysicalOutputs(0, 0);
    DL_TimerA_startCounter(PWM_0_INST);
    DL_TimerA_startCounter(PWM_1_INST);
    g_motorState.initialized = true;
    return true;
}

void Motor_SetOutputs(int16_t leftOutput, int16_t rightOutput)
{
    int16_t leftLogical = Motor_ClampOutput(leftOutput);
    int16_t rightLogical = Motor_ClampOutput(rightOutput);
    int16_t leftPhysical = Motor_ClampOutput(
        ((int32_t) leftLogical) * APP_MOTOR_LEFT_POLARITY);
    int16_t rightPhysical = Motor_ClampOutput(
        ((int32_t) rightLogical) * APP_MOTOR_RIGHT_POLARITY);

    Motor_ApplyPhysicalOutputs(leftPhysical, rightPhysical);
    g_motorState.leftOutput = leftLogical;
    g_motorState.rightOutput = rightLogical;
}

void Motor_SetPhysicalOutputs(int16_t channelAOutput,
    int16_t channelBOutput)
{
    Motor_ApplyPhysicalOutputs(Motor_ClampOutput(channelAOutput),
        Motor_ClampOutput(channelBOutput));
}

void Motor_Stop(void)
{
#if APP_MOTOR_ACTIVE_BRAKE_ENABLE != 0U
    Motor_ApplyBrake();
#else
    Motor_ApplyPhysicalOutputs(0, 0);
#endif
}

void Motor_GetState(MotorState *state)
{
    if (state != NULL) {
        *state = g_motorState;
    }
}

bool Motor_IsInitialized(void)
{
    return g_motorState.initialized;
}
