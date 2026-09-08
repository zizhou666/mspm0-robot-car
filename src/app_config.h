/*
================================================================================
Integrated_42688_Encoder 应用派生配置模块
================================================================================
【功能简介】
本文件在 user_config.h 的用户参数基础上生成控制周期秒数、车轮周长和按键
长按/连发采样数等派生常量，向现有业务模块提供统一的 APP_* 配置接口。

================================================================================
【函数定义】
本文件不定义函数。
- APP_CONTROL_PERIOD_S：由控制周期毫秒数换算得到。
- APP_WHEEL_CIRCUMFERENCE_CM：由圆周率和车轮半径计算得到。
- APP_BUTTON_LONG_PRESS_SAMPLES：由长按时间和按键扫描周期计算得到。
- APP_BUTTON_REPEAT_DELAY_SAMPLES：由方向键连发等待时间换算得到。
- APP_BUTTON_REPEAT_PERIOD_SAMPLES：由方向键连发间隔换算得到。

================================================================================
【使用说明】
1. 用户需要调整的参数统一修改 user_config.h，不要在本文件重复定义。
2. 本文件仅保存数学派生关系和编译期一致性检查。
3. 业务代码继续包含 app_config.h，即可同时获得用户参数和派生参数。
================================================================================
*/
#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include "user_config.h"

#define APP_CONTROL_PERIOD_S                  \
    ((float)APP_CONTROL_PERIOD_MS / 1000.0f)
#define APP_IMU_DT_SECONDS                    \
    ((float)APP_IMU_PERIOD_MS / 1000.0f)

#define APP_PI_F                              (3.14159265358979323846f)
#define APP_WHEEL_CIRCUMFERENCE_CM            \
    (2.0f * APP_PI_F * APP_WHEEL_RADIUS_CM)
#define APP_BUTTON_LONG_PRESS_SAMPLES         \
    (APP_BUTTON_LONG_PRESS_MS / APP_BUTTON_SCAN_PERIOD_MS)
#define APP_BUTTON_REPEAT_DELAY_SAMPLES       \
    (APP_BUTTON_REPEAT_DELAY_MS / APP_BUTTON_SCAN_PERIOD_MS)
#define APP_BUTTON_REPEAT_PERIOD_SAMPLES      \
    (APP_BUTTON_REPEAT_PERIOD_MS / APP_BUTTON_SCAN_PERIOD_MS)

#if ((APP_BUTTON_SCAN_PERIOD_MS == 0U) || \
     (APP_BUTTON_REPEAT_DELAY_MS < APP_BUTTON_SCAN_PERIOD_MS) || \
     (APP_BUTTON_REPEAT_PERIOD_MS < APP_BUTTON_SCAN_PERIOD_MS))
#error "button scan/repeat periods must be non-zero and repeat must be at least one scan"
#endif

#if (((APP_BUTTON_REPEAT_DELAY_MS % APP_BUTTON_SCAN_PERIOD_MS) != 0U) || \
     ((APP_BUTTON_REPEAT_PERIOD_MS % APP_BUTTON_SCAN_PERIOD_MS) != 0U) || \
     ((APP_BUTTON_LONG_PRESS_MS % APP_BUTTON_SCAN_PERIOD_MS) != 0U))
#error "button repeat/long-press times must be exact multiples of scan period"
#endif

#if ((APP_ENCODER_LEFT_POLARITY != 1) && \
     (APP_ENCODER_LEFT_POLARITY != -1))
#error "APP_ENCODER_LEFT_POLARITY must be 1 or -1"
#endif

#if ((APP_ENCODER_RIGHT_POLARITY != 1) && \
     (APP_ENCODER_RIGHT_POLARITY != -1))
#error "APP_ENCODER_RIGHT_POLARITY must be 1 or -1"
#endif

#if ((APP_MOTOR_LEFT_POLARITY != 1) && (APP_MOTOR_LEFT_POLARITY != -1))
#error "APP_MOTOR_LEFT_POLARITY must be 1 or -1"
#endif

#if ((APP_MOTOR_RIGHT_POLARITY != 1) && (APP_MOTOR_RIGHT_POLARITY != -1))
#error "APP_MOTOR_RIGHT_POLARITY must be 1 or -1"
#endif

#if ((APP_MOTOR_ACTIVE_BRAKE_ENABLE != 0U) && \
     (APP_MOTOR_ACTIVE_BRAKE_ENABLE != 1U))
#error "APP_MOTOR_ACTIVE_BRAKE_ENABLE must be 0 or 1"
#endif

#if (APP_MOTOR_BRAKE_DUTY_COUNTS >= APP_MOTOR_PWM_PERIOD_COUNTS)
#error "APP_MOTOR_BRAKE_DUTY_COUNTS must be below PWM period"
#endif

#endif /* APP_CONFIG_H_ */
