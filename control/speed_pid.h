/*
================================================================================
左右轮速度 PID 控制接口模块
================================================================================
【功能简介】
本文件声明左右轮独立速度闭环接口。控制器接收编码器速度，计算带条件积分
抗饱和和输出方向约束的电机 PWM 指令，并提供完整调试快照。

================================================================================
【函数定义】
- SpeedPID_Init：装载 user_config.h 默认 Kp/Ki/Kd 并清零控制状态。
- SpeedPID_Enable：使能或停止左右轮速度闭环。
- SpeedPID_IsEnabled：查询闭环是否正在输出。
- SpeedPID_SetTargets：设置左右轮目标速度，单位 cm/s。
- SpeedPID_SetParameters：校验并更新左右轮共用的速度 PID 参数。
- SpeedPID_GetParameters：读取当前速度 PID 参数。
- SpeedPID_Reset：清除误差、积分、输出并停止电机。
- SpeedPID_Update10ms：按编码器采样周期执行一次左右轮速度 PID。
- SpeedPID_GetSnapshot：读取目标、测量、积分、输出和 PWM 快照。

================================================================================
【使用说明】
1. 必须先初始化电机和编码器，再调用 SpeedPID_Init。
2. 每次 Encoder_SamplePeriodMs 后调用一次 SpeedPID_Update10ms。
3. Kp/Ki/Kd 是速度环参数，不是距离环或姿态环参数。
4. 默认值和 OLED 调整范围统一在 user_config.h 中修改。
================================================================================
*/
#ifndef SPEED_PID_H_
#define SPEED_PID_H_

#include <stdbool.h>
#include <stdint.h>

#include "encoder.h"

typedef struct {
    float kp;
    float ki;
    float kd;
} SpeedPIDParameters;

typedef struct {
    float leftTargetCmS;
    float rightTargetCmS;
    float leftMeasuredCmS;
    float rightMeasuredCmS;
    float leftErrorCmS;
    float rightErrorCmS;
    float leftIntegral;
    float rightIntegral;
    int16_t leftOutput;
    int16_t rightOutput;
    uint16_t leftDutyCounts;
    uint16_t rightDutyCounts;
    float kp;
    float ki;
    float kd;
    bool enabled;
} SpeedPIDSnapshot;

void SpeedPID_Init(void);
void SpeedPID_Enable(bool enabled);
bool SpeedPID_IsEnabled(void);
void SpeedPID_SetTargets(float leftCmS, float rightCmS);
bool SpeedPID_SetParameters(const SpeedPIDParameters *parameters);
void SpeedPID_GetParameters(SpeedPIDParameters *parameters);
void SpeedPID_Reset(void);

/* Call after an encoder sample. Missed 10-ms ticks are scaled from its period. */
void SpeedPID_Update10ms(const EncoderSnapshot *encoder);
void SpeedPID_GetSnapshot(SpeedPIDSnapshot *snapshot);

#endif /* SPEED_PID_H_ */
