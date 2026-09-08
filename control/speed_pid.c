/*
================================================================================
左右轮速度 PID 控制算法实现模块
================================================================================
【功能简介】
本文件实现左右轮独立的位置式离散速度 PID。算法包含误差限幅、条件积分
抗饱和、目标方向输出约束、丢失控制节拍补偿和最终 PWM 限幅。

================================================================================
【函数定义】
- SpeedPID_Clamp：把有符号数值限制在正负 limit 内。
- SpeedPID_ClampRange：把数值限制在 minimum~maximum 范围。
- SpeedPID_RoundOutput：按 PWM 上限限幅并四舍五入为 int16_t。
- SpeedPID_ConstrainOutputDirection：禁止速度超调立即命令反向制动。
- SpeedPID_UpdateAxis：计算单侧 P/I/D、条件积分和方向约束输出。
- SpeedPID_Init：载入默认 Kp/Ki/Kd、目标值和使能状态。
- SpeedPID_Enable：启停闭环，停用时清状态并关闭电机。
- SpeedPID_IsEnabled：查询当前使能状态。
- SpeedPID_SetTargets：更新左右目标速度。
- SpeedPID_SetParameters：校验范围并设置速度 PID 参数。
- SpeedPID_GetParameters：读取当前 Kp/Ki/Kd。
- SpeedPID_Reset：清除两侧积分、历史误差、输出和 PWM。
- SpeedPID_Update10ms：根据编码器快照更新两侧控制量。
- SpeedPID_GetSnapshot：复制完整控制器调试快照。

================================================================================
【使用说明】
1. Kp/Ki/Kd 是速度 PID；Ki 以 10 ms 为一个离散基准周期累计。
2. Update10ms 应紧跟编码器采样，samplePeriodMs 用于补偿偶发积压节拍。
3. 用户参数统一在 user_config.h 中调整，也可由 OLED 页面在运行时覆盖。
4. 条件积分抗饱和不得删除，否则满占空时容易积分堆积并失去调速能力。
================================================================================
*/
#include "speed_pid.h"

#include <math.h>
#include <stddef.h>

#include "app_config.h"
#include "motor.h"

typedef struct {
    float integralContribution;
    float previousError;
} SpeedPIDAxis;

static SpeedPIDAxis g_leftAxis;
static SpeedPIDAxis g_rightAxis;
static SpeedPIDSnapshot g_snapshot;
static SpeedPIDParameters g_parameters;

static float SpeedPID_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float SpeedPID_ClampRange(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t SpeedPID_RoundOutput(float output)
{
    float limited = SpeedPID_Clamp(output, APP_SPEED_PID_OUTPUT_LIMIT);
    return (int16_t) ((limited >= 0.0f) ? (limited + 0.5f) :
                                             (limited - 0.5f));
}

static float SpeedPID_ConstrainOutputDirection(float output, float target)
{
    if (target > APP_SPEED_PID_TARGET_STOP_BAND_CM_S) {
        return (output < 0.0f) ? 0.0f : output;
    }
    if (target < -APP_SPEED_PID_TARGET_STOP_BAND_CM_S) {
        return (output > 0.0f) ? 0.0f : output;
    }
    return 0.0f;
}

static int16_t SpeedPID_UpdateAxis(
    SpeedPIDAxis *axis, float target, float measured, float sampleScale,
    float *errorOut)
{
    float error = SpeedPID_Clamp(
        target - measured, APP_SPEED_PID_ERROR_LIMIT_CM_S);
    float derivative;
    float candidateIntegral;
    float candidateOutput;
    bool drivesIntoWrongDirection;

    if ((target >= -APP_SPEED_PID_TARGET_STOP_BAND_CM_S) &&
        (target <= APP_SPEED_PID_TARGET_STOP_BAND_CM_S)) {
        axis->integralContribution = 0.0f;
        axis->previousError = 0.0f;
        *errorOut = 0.0f;
        return 0;
    }

    /*
     * Preserve the r2 discrete PI convention: Ki is the contribution added
     * once per 10-ms sample (there is no additional multiplication by dt).
     * Conditional integration prevents an unreachable target or a transient
     * large error from winding the controller permanently against the PWM
     * rail. Integration resumes immediately when it drives out of saturation.
     */
    derivative = g_parameters.kd *
        (error - axis->previousError) / sampleScale;
    candidateIntegral = axis->integralContribution +
        g_parameters.ki * error * sampleScale;
    candidateIntegral = SpeedPID_Clamp(candidateIntegral,
        APP_SPEED_PID_INTEGRAL_LIMIT);
    candidateOutput = g_parameters.kp * error +
        candidateIntegral + derivative;

    /*
     * A wheeled chassis should coast when it overshoots a forward/reverse
     * speed target; commanding the opposite bridge direction creates the
     * observed forward/reverse hunting. Treat zero as a target-dependent
     * output rail and apply the same conditional-integration rule used at
     * the +/-999 rails.
     */
    drivesIntoWrongDirection =
        (((target > APP_SPEED_PID_TARGET_STOP_BAND_CM_S) &&
          (candidateOutput < 0.0f) && (error < 0.0f)) ||
         ((target < -APP_SPEED_PID_TARGET_STOP_BAND_CM_S) &&
          (candidateOutput > 0.0f) && (error > 0.0f)));

    if (!(((candidateOutput > APP_SPEED_PID_OUTPUT_LIMIT) &&
           (error > 0.0f)) ||
          ((candidateOutput < -APP_SPEED_PID_OUTPUT_LIMIT) &&
           (error < 0.0f)) || drivesIntoWrongDirection)) {
        axis->integralContribution = candidateIntegral;
    }

    float output = (g_parameters.kp * error) + axis->integralContribution +
        derivative;
    output = SpeedPID_ConstrainOutputDirection(output, target);
    axis->previousError = error;
    *errorOut = error;
    return SpeedPID_RoundOutput(output);
}

void SpeedPID_Init(void)
{
    g_parameters.kp = APP_SPEED_PID_KP;
    g_parameters.ki = APP_SPEED_PID_KI;
    g_parameters.kd = APP_SPEED_PID_KD;
    g_snapshot.leftTargetCmS  = APP_DEFAULT_TARGET_SPEED_CM_S;
    g_snapshot.rightTargetCmS = APP_DEFAULT_TARGET_SPEED_CM_S;
    g_snapshot.leftMeasuredCmS  = 0.0f;
    g_snapshot.rightMeasuredCmS = 0.0f;
    g_snapshot.kp = g_parameters.kp;
    g_snapshot.ki = g_parameters.ki;
    g_snapshot.kd = g_parameters.kd;
    g_snapshot.enabled = (APP_SPEED_CONTROL_ENABLE_AT_BOOT != 0U);
    SpeedPID_Reset();

    if (!g_snapshot.enabled) {
        Motor_Stop();
    }
}

void SpeedPID_Enable(bool enabled)
{
    if (g_snapshot.enabled != enabled) {
        g_snapshot.enabled = enabled;
        SpeedPID_Reset();
    }
    if (!enabled) {
        Motor_Stop();
    }
}

bool SpeedPID_IsEnabled(void)
{
    return g_snapshot.enabled;
}

void SpeedPID_SetTargets(float leftCmS, float rightCmS)
{
    if ((!isfinite(leftCmS)) || (!isfinite(rightCmS))) {
        g_snapshot.leftTargetCmS = 0.0f;
        g_snapshot.rightTargetCmS = 0.0f;
        SpeedPID_Enable(false);
        return;
    }

    g_snapshot.leftTargetCmS = SpeedPID_ClampRange(leftCmS,
        APP_TARGET_SPEED_MIN_CM_S, APP_TARGET_SPEED_MAX_CM_S);
    g_snapshot.rightTargetCmS = SpeedPID_ClampRange(rightCmS,
        APP_TARGET_SPEED_MIN_CM_S, APP_TARGET_SPEED_MAX_CM_S);
}

bool SpeedPID_SetParameters(const SpeedPIDParameters *parameters)
{
    if ((parameters == NULL) || (!isfinite(parameters->kp)) ||
        (!isfinite(parameters->ki)) || (!isfinite(parameters->kd))) {
        return false;
    }

    g_parameters.kp = SpeedPID_ClampRange(parameters->kp,
        APP_SPEED_PID_KP_MIN, APP_SPEED_PID_KP_MAX);
    g_parameters.ki = SpeedPID_ClampRange(parameters->ki,
        APP_SPEED_PID_KI_MIN, APP_SPEED_PID_KI_MAX);
    g_parameters.kd = SpeedPID_ClampRange(parameters->kd,
        APP_SPEED_PID_KD_MIN, APP_SPEED_PID_KD_MAX);
    g_snapshot.kp = g_parameters.kp;
    g_snapshot.ki = g_parameters.ki;
    g_snapshot.kd = g_parameters.kd;
    SpeedPID_Reset();
    return true;
}

void SpeedPID_GetParameters(SpeedPIDParameters *parameters)
{
    if (parameters != NULL) {
        *parameters = g_parameters;
    }
}

void SpeedPID_Reset(void)
{
    g_leftAxis.integralContribution  = 0.0f;
    g_leftAxis.previousError         = 0.0f;
    g_rightAxis.integralContribution = 0.0f;
    g_rightAxis.previousError        = 0.0f;
    g_snapshot.leftErrorCmS  = 0.0f;
    g_snapshot.rightErrorCmS = 0.0f;
    g_snapshot.leftIntegral  = 0.0f;
    g_snapshot.rightIntegral = 0.0f;
    g_snapshot.leftOutput    = 0;
    g_snapshot.rightOutput   = 0;
    g_snapshot.leftDutyCounts  = 0U;
    g_snapshot.rightDutyCounts = 0U;
}

void SpeedPID_Update10ms(const EncoderSnapshot *encoder)
{
    float sampleScale;

    if (encoder == NULL) {
        return;
    }

    g_snapshot.leftMeasuredCmS  = encoder->leftSpeedCmS;
    g_snapshot.rightMeasuredCmS = encoder->rightSpeedCmS;

    if ((!g_snapshot.enabled) || (!encoder->initialized)) {
        SpeedPID_Reset();
        Motor_Stop();
        return;
    }

    sampleScale = (float)encoder->samplePeriodMs /
        (float)APP_CONTROL_PERIOD_MS;
    if (sampleScale < 1.0f) {
        sampleScale = 1.0f;
    } else if (sampleScale > 100.0f) {
        /* Bound recovery after an abnormally long main-loop stall. */
        sampleScale = 100.0f;
    }

    g_snapshot.leftOutput = SpeedPID_UpdateAxis(&g_leftAxis,
        g_snapshot.leftTargetCmS, encoder->leftSpeedCmS,
        sampleScale,
        &g_snapshot.leftErrorCmS);
    g_snapshot.rightOutput = SpeedPID_UpdateAxis(&g_rightAxis,
        g_snapshot.rightTargetCmS, encoder->rightSpeedCmS,
        sampleScale,
        &g_snapshot.rightErrorCmS);
    g_snapshot.leftIntegral  = g_leftAxis.integralContribution;
    g_snapshot.rightIntegral = g_rightAxis.integralContribution;

    Motor_SetOutputs(g_snapshot.leftOutput, g_snapshot.rightOutput);

    MotorState motor;
    Motor_GetState(&motor);
    g_snapshot.leftDutyCounts  = motor.leftDutyCounts;
    g_snapshot.rightDutyCounts = motor.rightDutyCounts;
}

void SpeedPID_GetSnapshot(SpeedPIDSnapshot *snapshot)
{
    if (snapshot != NULL) {
        *snapshot = g_snapshot;
    }
}
