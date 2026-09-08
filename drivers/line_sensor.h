/*
================================================================================
十二路灰度巡线传感器接口模块
================================================================================
【功能简介】
本文件定义 P1~P12 灰度数字量采样结果，提供有效通道掩码、加权位置误差、
横线标记和丢线状态，供电机巡线状态机使用。

================================================================================
【函数定义】
- LineSensor_Init：确认 SysConfig 输入引脚已就绪并初始化模块状态。
- LineSensor_IsInitialized：查询灰度模块是否可用。
- LineSensor_Read：同步采集 12 路输入并计算加权误差、横线和丢线状态。

================================================================================
【使用说明】
1. 本工程保留 12 路灰度，不屏蔽任何通道
2. 主循环控制任务调用 LineSensor_Read，中断中不得执行巡线计算。
3. 通道权重和横线阈值在 user_config.h 中调整。
================================================================================
*/
#ifndef LINE_SENSOR_H_
#define LINE_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t active_mask;
    uint8_t active_count;
    uint8_t marker;
    uint8_t lost;
    float error;
} LineSensorSample;

bool LineSensor_Init(void);
bool LineSensor_IsInitialized(void);
void LineSensor_Read(LineSensorSample *sample);

#endif /* LINE_SENSOR_H_ */
