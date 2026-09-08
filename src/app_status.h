/*
================================================================================
系统运行状态数据模块
================================================================================
【功能简介】
本文件定义主程序、OLED 和调试器共享的系统状态快照，用于表示 OLED、编码器、
电机、灰度、Flash、按键和 ICM42688 的初始化及故障状态。

================================================================================
【函数定义】
本文件不定义函数。
- AppStatus：系统状态结构体，每个字段对应一个模块或故障标志。
- g_appStatus：由 main.c 更新的 volatile 全局状态，供调试和 UI 读取。

================================================================================
【使用说明】
1. 初始化和故障恢复逻辑在 main.c 中更新 g_appStatus。
2. 中断与主循环共享字段按 volatile 对待，不要保存局部指针长期访问。
3. 编码器计数为 0 表示可能静止，不等价于 encoderConfigured 为 0。
================================================================================
*/
#ifndef APP_STATUS_H_
#define APP_STATUS_H_

#include <stdint.h>

typedef struct {
    uint8_t systemRunning;
    uint8_t oledInitialized;
    uint8_t encoderConfigured;
    uint8_t encoderSignalMissing;
    uint8_t encoderDirectionMismatch;
    uint8_t buttonsInitialized;
    uint8_t motorInitialized;
    uint8_t lineSensorsInitialized;
    uint8_t flashInitialized;
    uint8_t flashRecordLoaded;
    uint8_t flashSaveFailed;
    uint8_t batteryMonitorInitialized;
    uint8_t batteryReady;
    uint8_t batteryLowVoltage;
    uint8_t wirelessInitialized;
    uint8_t wirelessConnected;
    uint8_t imuInitialized;
    uint8_t imuCalibrating;
    uint8_t imuReady;
    uint8_t imuFailed;
    uint32_t buttonPressedMask;
} AppStatus;

/* Public for a debugger/telemetry layer; application code writes it in main. */
extern volatile AppStatus g_appStatus;

#endif /* APP_STATUS_H_ */
