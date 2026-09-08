/*
================================================================================
电机距离行走与巡线调试接口模块
================================================================================
【功能简介】
本文件声明电机调试状态机，支持按目标距离直行和 12 路灰度巡线两种模式。
距离模式含编码器直行 PID 与 ICM42688 角度 PID，巡线模式含灰度巡线 PID，
并统一处理参数配置、启动、停止、完成和传感器故障。

================================================================================
【函数定义】
- MotorDebug_Init：初始化调试状态机并记录灰度模块可用状态。
- MotorDebug_SetConfiguration：设置模式、目标距离和基础速度。
- MotorDebug_SetBaseSpeed：运行过程中更新基础速度，用于平滑起步。
- MotorDebug_SetPIDSettings：设置编码器直行、巡线和角度 PID 参数。
- MotorDebug_GetPIDSettings：读取三套上层 PID 参数。
- MotorDebug_Start：记录起点并启动相应模式。
- MotorDebug_Stop：用户正常停止电机调试。
- MotorDebug_AbortFault：以指定故障原因中止。
- MotorDebug_EncoderFault：以编码器故障原因中止。
- MotorDebug_Update10ms：每 10 ms 更新里程、直行修正或巡线速度目标。
- MotorDebug_IsRunning：查询状态机是否运行。
- MotorDebug_GetSnapshot：读取模式、里程、灰度和停止原因。

================================================================================
【使用说明】
1. 上电调用 MotorDebug_Init，之后用 SetConfiguration 设置 OLED 草稿参数。
2. 中心键短按由 main.c 调用 Start/Stop；长按仍用于保存 Flash。
3. Update10ms 必须与编码器和速度 PID 使用同一固定控制节拍。
4. 首次默认值和调节范围在 user_config.h；运行值可在 OLED 第 4 页调整。
================================================================================
*/
#ifndef MOTOR_DEBUG_H_
#define MOTOR_DEBUG_H_

#include "encoder.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_DEBUG_MODE_DISTANCE = 0,
    MOTOR_DEBUG_MODE_LINE_FOLLOW = 1
} MotorDebugMode;

typedef enum {
    MOTOR_DEBUG_STATE_IDLE = 0,
    MOTOR_DEBUG_STATE_RUNNING,
    MOTOR_DEBUG_STATE_DONE,
    MOTOR_DEBUG_STATE_STOPPED,
    MOTOR_DEBUG_STATE_FAULT
} MotorDebugState;

typedef enum {
    MOTOR_DEBUG_STOP_NONE = 0,
    MOTOR_DEBUG_STOP_TARGET_REACHED,
    MOTOR_DEBUG_STOP_USER,
    MOTOR_DEBUG_STOP_MOTOR_FAULT,
    MOTOR_DEBUG_STOP_ENCODER_FAULT,
    MOTOR_DEBUG_STOP_LINE_FAULT,
    MOTOR_DEBUG_STOP_INVALID_CONFIG
} MotorDebugStopReason;

typedef struct {
    MotorDebugMode mode;
    MotorDebugState state;
    MotorDebugStopReason stop_reason;
    float target_distance_cm;
    float base_speed_cm_s;
    float traveled_distance_cm;
    float line_error;
    uint16_t line_active_mask;
    uint8_t line_active_count;
    uint8_t line_lost;
} MotorDebugSnapshot;

/* 第 4 页可调的三套上层 PID；底层左右轮速度 PID 仍由 speed_pid 管理。 */
typedef struct {
    float straight_kp;
    float straight_ki;
    float straight_kd;
    float line_kp;
    float line_ki;
    float line_kd;
    float angle_kp;
    float angle_ki;
    float angle_kd;
} MotorDebugPIDSettings;

void MotorDebug_Init(bool line_sensor_ready);
bool MotorDebug_SetConfiguration(MotorDebugMode mode,
                                 float target_distance_cm,
                                 float base_speed_cm_s);
bool MotorDebug_SetBaseSpeed(float base_speed_cm_s);
bool MotorDebug_SetPIDSettings(const MotorDebugPIDSettings *settings);
void MotorDebug_GetPIDSettings(MotorDebugPIDSettings *settings);
bool MotorDebug_Start(const EncoderSnapshot *encoder, bool encoder_ready,
                      float heading_reference_deg, bool heading_ready);
/*
 * Switch a running controller from line following to encoder distance mode
 * without disabling or resetting the lower wheel-speed PID.
 */
bool MotorDebug_TransitionToDistance(
    const EncoderSnapshot *encoder,
    float target_distance_cm,
    float base_speed_cm_s,
    float heading_reference_deg,
    bool heading_ready);
void MotorDebug_Stop(void);
void MotorDebug_AbortFault(MotorDebugStopReason reason);
void MotorDebug_EncoderFault(void);
void MotorDebug_Update10ms(const EncoderSnapshot *encoder, float yaw_deg,
                           bool heading_ready);
bool MotorDebug_IsRunning(void);
void MotorDebug_GetSnapshot(MotorDebugSnapshot *snapshot);

#endif /* MOTOR_DEBUG_H_ */
