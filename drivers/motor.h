/*
================================================================================
TB6612 双路电机驱动接口模块
================================================================================
【功能简介】
本文件定义左右电机逻辑输出、PWM 占空比状态和物理通道点动测试接口。
左电机使用 PA15/TIMA1_CCP0 作为 PWMA，PA7/PB1 作为方向输入；右电机
使用 PA3/TIMA0_CCP1 作为 PWMB，PB2/PB3 作为方向输入。

================================================================================
【函数定义】
- Motor_Init：清零四路 PWM、启动 TIMA0 并标记电机模块就绪。
- Motor_SetOutputs：按左右电机极性映射逻辑输出到物理 H 桥。
- Motor_SetPhysicalOutputs：绕过车体极性，直接测试 A/B 物理通道。
- Motor_Stop：把四个 H 桥输入全部置为 0。
- Motor_GetState：读取当前左右输出和占空计数。
- Motor_IsInitialized：查询电机驱动是否初始化完成。

================================================================================
【使用说明】
1. TB6612 接线为 PWMA=PA15、PWMB=PA3、AIN1=PA7、AIN2=PB1、BIN1=PB2、BIN2=PB3。
2. 正常运行只调用 Motor_SetOutputs；PhysicalOutputs 仅用于第六页接线测试。
3. 左右电机极性、PWM 周期和输出限幅在 user_config.h 中修改。
4. 停机必须调用 Motor_Stop 或关闭 SpeedPID，不能仅把目标速度清零后立即断言。
================================================================================
*/
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t leftOutput;
    int16_t rightOutput;
    uint16_t leftDutyCounts;
    uint16_t rightDutyCounts;
    bool initialized;
} MotorState;

typedef enum {
    MOTOR_IO_TEST_COMMAND_OFF = 0,
    MOTOR_IO_TEST_COMMAND_A_POSITIVE,
    MOTOR_IO_TEST_COMMAND_A_NEGATIVE,
    MOTOR_IO_TEST_COMMAND_B_POSITIVE,
    MOTOR_IO_TEST_COMMAND_B_NEGATIVE
} MotorIoTestCommand;

typedef enum {
    MOTOR_IO_TEST_PHASE_IDLE = 0,
    MOTOR_IO_TEST_PHASE_ARMED,
    MOTOR_IO_TEST_PHASE_RUNNING,
    MOTOR_IO_TEST_PHASE_SETTLING,
    MOTOR_IO_TEST_PHASE_DONE
} MotorIoTestPhase;

bool Motor_Init(void);
void Motor_SetOutputs(int16_t leftOutput, int16_t rightOutput);
/*
 * Commissioning-only physical channel access. Channel A is the TB6612 A side
 * driven by PWMA/AIN1/AIN2, and channel B is the TB6612 B side driven by
 * PWMB/BIN1/BIN2. Values bypass vehicle left/right polarity configuration.
 */
void Motor_SetPhysicalOutputs(int16_t channelAOutput,
                              int16_t channelBOutput);
void Motor_Stop(void);
void Motor_GetState(MotorState *state);
bool Motor_IsInitialized(void);

#endif /* MOTOR_H_ */
