/*
================================================================================
距离行走、直行修正与灰度巡线实现模块
================================================================================
【功能简介】
本文件实现电机调试状态机。距离模式按平均里程停止，并把编码器直行 PID 与
ICM42688 相对航向 PID 合成为回正量；巡线模式用 12 路灰度 PID 生成转向量。
三套上层 PID 均带输出限幅和条件积分抗饱和，最终只修改左右速度环目标值。

================================================================================
【函数定义】
- MotorDebug_Abs：计算浮点绝对值。
- MotorDebug_Clamp：按正负 limit 限制修正量。
- MotorDebug_WrapAngleDeg：把航向差约束到 -180°～180°。
- MotorDebug_ValueInRange：检查可调 PID 参数是否合法。
- MotorDebug_ShouldIntegrate：执行条件积分抗饱和判断。
- MotorDebug_ResetControllerHistory：清除三个上层 PID 的历史量。
- MotorDebug_EnterFault：关闭速度 PID 并记录故障停止原因。
- MotorDebug_Init：初始化模式、状态、默认目标和控制历史。
- MotorDebug_SetConfiguration：校验并更新模式、目标距离和基础速度。
- MotorDebug_SetPIDSettings：校验并应用直行、巡线、角度 PID 参数。
- MotorDebug_GetPIDSettings：读取当前三套上层 PID 参数。
- MotorDebug_Start：校验传感器、记录起点并启动速度 PID。
- MotorDebug_Stop：用户停止并关闭速度 PID。
- MotorDebug_AbortFault：按外部故障原因中止运行。
- MotorDebug_EncoderFault：按编码器故障中止运行。
- MotorDebug_UpdateLineTargets：根据灰度 PID 生成左右轮目标速度。
- MotorDebug_UpdateStraightTargets：合成编码器直行 PID 和相对航向 PID。
- MotorDebug_Update10ms：更新里程、到距停止和当前模式控制目标。
- MotorDebug_IsRunning：查询调试状态是否为 RUNNING。
- MotorDebug_GetSnapshot：复制状态机和灰度数据快照。

================================================================================
【使用说明】
1. 距离模式依赖左右编码器路程符号一致，先确认前进计数均为正。
2. 距离模式启动时记录当前 Yaw 为航向基准；IMU 不可用时自动只用编码器。
3. 角度环不参与巡线，避免和灰度巡线 PID 同时争夺转向控制权。
4. 巡线丢线超过阈值会进入故障并停止，不会永久阻塞其他页面。
5. 本模块只给 SpeedPID 设置目标，不直接操作物理 PWM 通道。
================================================================================
*/
#include "motor_debug.h"

#include "app_config.h"
#include "line_sensor.h"
#include "speed_pid.h"

#include <math.h>
#include <stddef.h>

static MotorDebugSnapshot g_snapshot;
static float g_startDistanceCm;
static float g_startLeftDistanceCm;
static float g_startRightDistanceCm;
static float g_lastLineError;
static float g_lastAngleError;
static float g_straightIntegral;
static float g_lineIntegral;
static float g_angleIntegral;
static float g_headingReferenceDeg;
static uint16_t g_lineLostTicks;
static bool g_lineSensorReady;
static bool g_headingControlEnabled;
static MotorDebugPIDSettings g_pidSettings;

static float MotorDebug_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float MotorDebug_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float MotorDebug_WrapAngleDeg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static bool MotorDebug_ValueInRange(float value, float minimum, float maximum)
{
    return isfinite(value) && (value >= minimum) && (value <= maximum);
}

/* Conditional integration: do not let an error push an already saturated
 * controller farther into saturation, but allow it to unwind the integral. */
static bool MotorDebug_ShouldIntegrate(float error, float output, float limit)
{
    return (MotorDebug_Abs(output) <= limit) ||
        ((output > limit) && (error < 0.0f)) ||
        ((output < -limit) && (error > 0.0f));
}

static void MotorDebug_ResetControllerHistory(void)
{
    g_lastLineError = 0.0f;
    g_lastAngleError = 0.0f;
    g_straightIntegral = 0.0f;
    g_lineIntegral = 0.0f;
    g_angleIntegral = 0.0f;
}

static void MotorDebug_EnterFault(MotorDebugStopReason reason)
{
    SpeedPID_Enable(false);
    MotorDebug_ResetControllerHistory();
    g_snapshot.state = MOTOR_DEBUG_STATE_FAULT;
    g_snapshot.stop_reason = reason;
}

void MotorDebug_Init(bool line_sensor_ready)
{
    g_lineSensorReady = line_sensor_ready;
    g_snapshot.mode = MOTOR_DEBUG_MODE_DISTANCE;
    g_snapshot.state = MOTOR_DEBUG_STATE_IDLE;
    g_snapshot.stop_reason = MOTOR_DEBUG_STOP_NONE;
    g_snapshot.target_distance_cm = APP_DEFAULT_TARGET_DISTANCE_CM;
    g_snapshot.base_speed_cm_s = APP_DEFAULT_TARGET_SPEED_CM_S;
    g_snapshot.traveled_distance_cm = 0.0f;
    g_snapshot.line_error = 0.0f;
    g_snapshot.line_active_mask = 0U;
    g_snapshot.line_active_count = 0U;
    g_snapshot.line_lost = 1U;
    g_startDistanceCm = 0.0f;
    g_startLeftDistanceCm = 0.0f;
    g_startRightDistanceCm = 0.0f;
    g_pidSettings.straight_kp = APP_STRAIGHT_PID_KP;
    g_pidSettings.straight_ki = APP_STRAIGHT_PID_KI;
    g_pidSettings.straight_kd = APP_STRAIGHT_PID_KD;
    g_pidSettings.line_kp = APP_LINE_PID_KP_PWM;
    g_pidSettings.line_ki = APP_LINE_PID_KI_PWM;
    g_pidSettings.line_kd = APP_LINE_PID_KD_PWM;
    g_pidSettings.angle_kp = APP_ANGLE_PID_KP;
    g_pidSettings.angle_ki = APP_ANGLE_PID_KI;
    g_pidSettings.angle_kd = APP_ANGLE_PID_KD;
    MotorDebug_ResetControllerHistory();
    g_lineLostTicks = 0U;
    g_headingControlEnabled = false;
    g_headingReferenceDeg = 0.0f;
    SpeedPID_Enable(false);
}

bool MotorDebug_SetConfiguration(MotorDebugMode mode,
                                 float target_distance_cm,
                                 float base_speed_cm_s)
{
    if (((mode != MOTOR_DEBUG_MODE_DISTANCE) &&
         (mode != MOTOR_DEBUG_MODE_LINE_FOLLOW)) ||
        (!isfinite(target_distance_cm)) || (!isfinite(base_speed_cm_s)) ||
        (target_distance_cm < APP_TARGET_DISTANCE_MIN_CM) ||
        (target_distance_cm > APP_TARGET_DISTANCE_MAX_CM) ||
        (base_speed_cm_s < APP_TARGET_SPEED_MIN_CM_S) ||
        (base_speed_cm_s > APP_TARGET_SPEED_MAX_CM_S)) {
        return false;
    }

    g_snapshot.mode = mode;
    g_snapshot.target_distance_cm = target_distance_cm;
    g_snapshot.base_speed_cm_s = base_speed_cm_s;
    return true;
}

bool MotorDebug_SetBaseSpeed(float base_speed_cm_s)
{
    if ((!isfinite(base_speed_cm_s)) ||
        (base_speed_cm_s < APP_TARGET_SPEED_MIN_CM_S) ||
        (base_speed_cm_s > APP_TARGET_SPEED_MAX_CM_S) ||
        ((g_snapshot.mode == MOTOR_DEBUG_MODE_LINE_FOLLOW) &&
         (base_speed_cm_s < 0.0f))) {
        return false;
    }

    g_snapshot.base_speed_cm_s = base_speed_cm_s;
    if (g_snapshot.state == MOTOR_DEBUG_STATE_RUNNING) {
        SpeedPID_SetTargets(base_speed_cm_s, base_speed_cm_s);
    }
    return true;
}

bool MotorDebug_SetPIDSettings(const MotorDebugPIDSettings *settings)
{
    if ((settings == NULL) ||
        (!MotorDebug_ValueInRange(settings->straight_kp,
            APP_STRAIGHT_PID_KP_MIN, APP_STRAIGHT_PID_KP_MAX)) ||
        (!MotorDebug_ValueInRange(settings->straight_ki,
            APP_STRAIGHT_PID_KI_MIN, APP_STRAIGHT_PID_KI_MAX)) ||
        (!MotorDebug_ValueInRange(settings->straight_kd,
            APP_STRAIGHT_PID_KD_MIN, APP_STRAIGHT_PID_KD_MAX)) ||
        (!MotorDebug_ValueInRange(settings->line_kp,
            APP_LINE_PID_KP_MIN, APP_LINE_PID_KP_MAX)) ||
        (!MotorDebug_ValueInRange(settings->line_ki,
            APP_LINE_PID_KI_MIN, APP_LINE_PID_KI_MAX)) ||
        (!MotorDebug_ValueInRange(settings->line_kd,
            APP_LINE_PID_KD_MIN, APP_LINE_PID_KD_MAX)) ||
        (!MotorDebug_ValueInRange(settings->angle_kp,
            APP_ANGLE_PID_KP_MIN, APP_ANGLE_PID_KP_MAX)) ||
        (!MotorDebug_ValueInRange(settings->angle_ki,
            APP_ANGLE_PID_KI_MIN, APP_ANGLE_PID_KI_MAX)) ||
        (!MotorDebug_ValueInRange(settings->angle_kd,
            APP_ANGLE_PID_KD_MIN, APP_ANGLE_PID_KD_MAX))) {
        return false;
    }

    g_pidSettings = *settings;
    MotorDebug_ResetControllerHistory();
    return true;
}

void MotorDebug_GetPIDSettings(MotorDebugPIDSettings *settings)
{
    if (settings != NULL) {
        *settings = g_pidSettings;
    }
}

bool MotorDebug_Start(const EncoderSnapshot *encoder, bool encoder_ready,
                      float heading_reference_deg, bool heading_ready)
{
    LineSensorSample line = {0};

    if ((encoder == NULL) || (!encoder_ready)) {
        MotorDebug_EnterFault(MOTOR_DEBUG_STOP_ENCODER_FAULT);
        return false;
    }
    if (MotorDebug_Abs(g_snapshot.base_speed_cm_s) <
        MOTOR_DEBUG_MIN_SPEED_CM_S) {
        MotorDebug_EnterFault(MOTOR_DEBUG_STOP_INVALID_CONFIG);
        return false;
    }
    if ((g_snapshot.mode == MOTOR_DEBUG_MODE_LINE_FOLLOW) &&
        ((!g_lineSensorReady) || (g_snapshot.base_speed_cm_s <= 0.0f))) {
        MotorDebug_EnterFault(MOTOR_DEBUG_STOP_LINE_FAULT);
        return false;
    }

    if (g_snapshot.mode == MOTOR_DEBUG_MODE_LINE_FOLLOW) {
        LineSensor_Read(&line);
        if (line.lost != 0U) {
            MotorDebug_EnterFault(MOTOR_DEBUG_STOP_LINE_FAULT);
            return false;
        }
        g_lastLineError = line.error;
        g_snapshot.line_error = line.error;
        g_snapshot.line_active_mask = line.active_mask;
        g_snapshot.line_active_count = line.active_count;
        g_snapshot.line_lost = line.lost;
    }

    g_startDistanceCm = encoder->vehicleDistanceCm;
    g_startLeftDistanceCm = encoder->leftDistanceCm;
    g_startRightDistanceCm = encoder->rightDistanceCm;
    g_snapshot.traveled_distance_cm = 0.0f;
    g_snapshot.state = MOTOR_DEBUG_STATE_RUNNING;
    g_snapshot.stop_reason = MOTOR_DEBUG_STOP_NONE;
    g_lineLostTicks = 0U;
    MotorDebug_ResetControllerHistory();
    if (g_snapshot.mode == MOTOR_DEBUG_MODE_LINE_FOLLOW) {
        g_lastLineError = line.error;
    }
    g_headingControlEnabled =
        (g_snapshot.mode == MOTOR_DEBUG_MODE_DISTANCE) && heading_ready &&
        isfinite(heading_reference_deg);
    g_headingReferenceDeg = g_headingControlEnabled ?
        heading_reference_deg : 0.0f;
    SpeedPID_SetTargets(g_snapshot.base_speed_cm_s,
                        g_snapshot.base_speed_cm_s);
    SpeedPID_Enable(true);
    return true;
}

bool MotorDebug_TransitionToDistance(
    const EncoderSnapshot *encoder,
    float target_distance_cm,
    float base_speed_cm_s,
    float heading_reference_deg,
    bool heading_ready)
{
    if ((encoder == NULL) || (!encoder->initialized) ||
        (g_snapshot.state != MOTOR_DEBUG_STATE_RUNNING) ||
        (!SpeedPID_IsEnabled()) ||
        (MotorDebug_Abs(base_speed_cm_s) < MOTOR_DEBUG_MIN_SPEED_CM_S) ||
        (!MotorDebug_SetConfiguration(MOTOR_DEBUG_MODE_DISTANCE,
            target_distance_cm, base_speed_cm_s))) {
        return false;
    }

    /* Start a new distance segment while preserving speed-loop I/PWM state. */
    g_startDistanceCm = encoder->vehicleDistanceCm;
    g_startLeftDistanceCm = encoder->leftDistanceCm;
    g_startRightDistanceCm = encoder->rightDistanceCm;
    g_snapshot.traveled_distance_cm = 0.0f;
    g_snapshot.stop_reason = MOTOR_DEBUG_STOP_NONE;
    g_snapshot.line_error = 0.0f;
    g_snapshot.line_active_mask = 0U;
    g_snapshot.line_active_count = 0U;
    g_snapshot.line_lost = 0U;
    g_lineLostTicks = 0U;
    MotorDebug_ResetControllerHistory();
    g_headingControlEnabled = heading_ready &&
        isfinite(heading_reference_deg);
    g_headingReferenceDeg = g_headingControlEnabled ?
        heading_reference_deg : 0.0f;

    /* Set the new base targets without toggling or resetting SpeedPID. */
    SpeedPID_SetTargets(base_speed_cm_s, base_speed_cm_s);
    return true;
}

void MotorDebug_Stop(void)
{
    SpeedPID_Enable(false);
    MotorDebug_ResetControllerHistory();
    if (g_snapshot.state == MOTOR_DEBUG_STATE_RUNNING) {
        g_snapshot.state = MOTOR_DEBUG_STATE_STOPPED;
        g_snapshot.stop_reason = MOTOR_DEBUG_STOP_USER;
    }
}

void MotorDebug_AbortFault(MotorDebugStopReason reason)
{
    if ((reason == MOTOR_DEBUG_STOP_NONE) ||
        (reason == MOTOR_DEBUG_STOP_TARGET_REACHED) ||
        (reason == MOTOR_DEBUG_STOP_USER)) {
        reason = MOTOR_DEBUG_STOP_INVALID_CONFIG;
    }
    MotorDebug_EnterFault(reason);
}

void MotorDebug_EncoderFault(void)
{
    MotorDebug_EnterFault(MOTOR_DEBUG_STOP_ENCODER_FAULT);
}

static void MotorDebug_UpdateLineTargets(void)
{
    LineSensorSample line;
    float candidate_integral;
    float candidate_turn;
    float error_delta;
    float legacy_turn;
    float scale;
    float turn_cm_s;
    float turn_limit_cm_s;

    LineSensor_Read(&line);
    g_snapshot.line_active_mask = line.active_mask;
    g_snapshot.line_active_count = line.active_count;
    g_snapshot.line_lost = line.lost;

    scale = MotorDebug_Abs(g_snapshot.base_speed_cm_s) /
            APP_LINE_REFERENCE_BASE_PWM;
    turn_limit_cm_s = APP_LINE_TURN_LIMIT_PWM * scale;

    if (line.lost != 0U) {
        if (g_lineLostTicks < UINT16_MAX) {
            ++g_lineLostTicks;
        }
        if (g_lineLostTicks >= MOTOR_DEBUG_LINE_LOST_MAX_TICKS) {
            MotorDebug_EnterFault(MOTOR_DEBUG_STOP_LINE_FAULT);
            return;
        }
        g_lineIntegral = 0.0f;
        legacy_turn = g_pidSettings.line_kp * g_lastLineError;
        if ((MotorDebug_Abs(g_lastLineError) >=
             APP_LINE_LOST_EDGE_ERROR) &&
            (MotorDebug_Abs(legacy_turn) < APP_LINE_LOST_RECOVERY_PWM)) {
            legacy_turn = (g_lastLineError > 0.0f) ?
                APP_LINE_LOST_RECOVERY_PWM : -APP_LINE_LOST_RECOVERY_PWM;
        }
    } else if (line.marker != 0U) {
        g_lineLostTicks = 0U;
        g_lineIntegral = 0.0f;
        g_lastLineError = 0.0f;
        legacy_turn = 0.0f;
    } else {
        g_lineLostTicks = 0U;
        error_delta = line.error - g_lastLineError;
        candidate_integral = MotorDebug_Clamp(
            g_lineIntegral + g_pidSettings.line_ki * line.error,
            APP_LINE_PID_INTEGRAL_LIMIT_PWM);
        candidate_turn = g_pidSettings.line_kp * line.error +
            candidate_integral + g_pidSettings.line_kd * error_delta;
        if (MotorDebug_ShouldIntegrate(line.error, candidate_turn,
                                       APP_LINE_TURN_LIMIT_PWM)) {
            g_lineIntegral = candidate_integral;
        }
        legacy_turn = g_pidSettings.line_kp * line.error +
            g_lineIntegral + g_pidSettings.line_kd * error_delta;
        g_lastLineError = line.error;
        g_snapshot.line_error = line.error;
    }

    legacy_turn = MotorDebug_Clamp(legacy_turn, APP_LINE_TURN_LIMIT_PWM);
    turn_cm_s = MotorDebug_Clamp(legacy_turn * scale, turn_limit_cm_s);
    SpeedPID_SetTargets(g_snapshot.base_speed_cm_s - turn_cm_s,
                        g_snapshot.base_speed_cm_s + turn_cm_s);
}

static void MotorDebug_UpdateStraightTargets(const EncoderSnapshot *encoder,
                                              float yaw_deg,
                                              bool heading_ready)
{
    float base = g_snapshot.base_speed_cm_s;
    float direction = (base >= 0.0f) ? 1.0f : -1.0f;
    float leftProgress =
        (encoder->leftDistanceCm - g_startLeftDistanceCm) * direction;
    float rightProgress =
        (encoder->rightDistanceCm - g_startRightDistanceCm) * direction;
    float distanceError = leftProgress - rightProgress;
    float speedError =
        (encoder->leftSpeedCmS - encoder->rightSpeedCmS) * direction;
    float maxCorrection = MotorDebug_Abs(base) *
        APP_STRAIGHT_CORRECTION_MAX_RATIO;
    float candidateIntegral;
    float candidateCorrection;
    float straightCorrection;
    float angleCorrection = 0.0f;
    float correction;

    if (maxCorrection > APP_STRAIGHT_CORRECTION_MAX_CM_S) {
        maxCorrection = APP_STRAIGHT_CORRECTION_MAX_CM_S;
    }

    /*
     * Positive error means the left wheel is ahead in the commanded travel
     * direction. Slow that wheel and accelerate the right wheel. The signed
     * direction factor makes the same rule work while reversing.
     */
    candidateIntegral = MotorDebug_Clamp(
        g_straightIntegral + g_pidSettings.straight_ki * distanceError,
        APP_STRAIGHT_PID_INTEGRAL_LIMIT_CM_S);
    candidateCorrection = g_pidSettings.straight_kp * distanceError +
        candidateIntegral + g_pidSettings.straight_kd * speedError;
    if (MotorDebug_ShouldIntegrate(distanceError, candidateCorrection,
                                   maxCorrection)) {
        g_straightIntegral = candidateIntegral;
    }
    straightCorrection = g_pidSettings.straight_kp * distanceError +
        g_straightIntegral + g_pidSettings.straight_kd * speedError;
    straightCorrection = MotorDebug_Clamp(straightCorrection, maxCorrection);

    if (g_headingControlEnabled && heading_ready && isfinite(yaw_deg)) {
        float angleError = APP_ANGLE_YAW_POLARITY * MotorDebug_WrapAngleDeg(
            g_headingReferenceDeg - yaw_deg);
        float angleDelta;
        float maxAngleCorrection = MotorDebug_Abs(base) *
            APP_ANGLE_CORRECTION_MAX_RATIO;

        if (maxAngleCorrection > APP_ANGLE_CORRECTION_MAX_CM_S) {
            maxAngleCorrection = APP_ANGLE_CORRECTION_MAX_CM_S;
        }
        if (MotorDebug_Abs(angleError) < APP_ANGLE_ERROR_DEADBAND_DEG) {
            angleError = 0.0f;
        }
        angleDelta = MotorDebug_WrapAngleDeg(
            angleError - g_lastAngleError);
        candidateIntegral = MotorDebug_Clamp(
            g_angleIntegral + g_pidSettings.angle_ki * angleError,
            APP_ANGLE_PID_INTEGRAL_LIMIT_CM_S);
        candidateCorrection = g_pidSettings.angle_kp * angleError +
            candidateIntegral + g_pidSettings.angle_kd * angleDelta;
        if (MotorDebug_ShouldIntegrate(angleError, candidateCorrection,
                                       maxAngleCorrection)) {
            g_angleIntegral = candidateIntegral;
        }
        angleCorrection = g_pidSettings.angle_kp * angleError +
            g_angleIntegral + g_pidSettings.angle_kd * angleDelta;
        angleCorrection = MotorDebug_Clamp(angleCorrection,
                                            maxAngleCorrection);
        g_lastAngleError = angleError;
    } else {
        g_angleIntegral = 0.0f;
        g_lastAngleError = 0.0f;
    }

    correction = MotorDebug_Clamp(straightCorrection + angleCorrection,
                                  maxCorrection);

    SpeedPID_SetTargets(base - direction * correction,
                        base + direction * correction);
}

void MotorDebug_Update10ms(const EncoderSnapshot *encoder, float yaw_deg,
                           bool heading_ready)
{
    if ((g_snapshot.state != MOTOR_DEBUG_STATE_RUNNING) ||
        (encoder == NULL)) {
        return;
    }

    g_snapshot.traveled_distance_cm = MotorDebug_Abs(
        encoder->vehicleDistanceCm - g_startDistanceCm);
    if (g_snapshot.traveled_distance_cm >=
        g_snapshot.target_distance_cm) {
        SpeedPID_Enable(false);
        MotorDebug_ResetControllerHistory();
        g_snapshot.state = MOTOR_DEBUG_STATE_DONE;
        g_snapshot.stop_reason = MOTOR_DEBUG_STOP_TARGET_REACHED;
        return;
    }

    if (g_snapshot.mode == MOTOR_DEBUG_MODE_LINE_FOLLOW) {
        MotorDebug_UpdateLineTargets();
    } else {
        MotorDebug_UpdateStraightTargets(encoder, yaw_deg, heading_ready);
    }
}

bool MotorDebug_IsRunning(void)
{
    return (g_snapshot.state == MOTOR_DEBUG_STATE_RUNNING);
}

void MotorDebug_GetSnapshot(MotorDebugSnapshot *snapshot)
{
    if (snapshot != NULL) {
        *snapshot = g_snapshot;
    }
}
