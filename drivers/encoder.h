/*
================================================================================
左右轮增量编码器接口模块
================================================================================
【功能简介】
本文件定义双编码器计数、周期增量、速度和路程快照。A 相使用双边沿 GPIO
中断计数，B 相在中断中采样判向，左右轮累计计数互不干扰。

================================================================================
【函数定义】
- Encoder_Init：清零计数并使能 SysConfig 已配置的 GPIOB 中断。
- Encoder_HandleGpioInterruptIID：按 GPIO IIDX 分派左右 A 相中断并计数。
- Encoder_SamplePeriodMs：原子截取增量并按实际周期计算速度和路程。
- Encoder_Sample10ms：使用默认控制周期的采样兼容接口。
- Encoder_GetSnapshot：原子读取完整编码器状态快照。
- Encoder_ResetCounts：清零累计计数、增量、速度和路程。
- Encoder_IsInitialized：查询编码器模块配置状态。

================================================================================
【使用说明】
1. GROUP1_IRQHandler 只负责读取 IIDX 并调用中断分派函数。
2. 每圈计数、轮径和左右极性只在 user_config.h 中修改。
3. 计数为 0 只表示当前没有位移，不能据此判定编码器初始化失败。
4. 读取共享计数应通过 Encoder_GetSnapshot，不要直接访问中断变量。
================================================================================
*/
#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t leftPositionCounts;
    int32_t rightPositionCounts;
    int32_t leftDeltaCounts;
    int32_t rightDeltaCounts;
    float leftSpeedCmS;
    float rightSpeedCmS;
    float leftDistanceCm;
    float rightDistanceCm;
    float vehicleDistanceCm;
    uint32_t samplePeriodMs;
    bool initialized;
} EncoderSnapshot;

/* Enables the already-SysConfig-configured GPIOB interrupt. */
bool Encoder_Init(void);

/*
 * Dispatch one GPIOB IIDX value returned by DL_GPIO_getPendingInterrupt().
 * The common GROUP1_IRQHandler remains owned by the application so that other
 * Group-1 peripherals can share it safely.
 */
void Encoder_HandleGpioInterruptIID(uint32_t iid);

/* Atomically captures/resets delta counters and calculates speed. */
void Encoder_SamplePeriodMs(uint32_t periodMs);
void Encoder_Sample10ms(void);
void Encoder_GetSnapshot(EncoderSnapshot *snapshot);
void Encoder_ResetCounts(void);
bool Encoder_IsInitialized(void);

#endif /* ENCODER_H_ */
