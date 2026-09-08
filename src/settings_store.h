/*
================================================================================
运行参数 Flash 持久化接口模块
================================================================================
【功能简介】
本文件声明 W25Q64 参数日志接口，用于保存目标速度、目标距离、速度 PID、
编码器直行 PID、巡线 PID、角度 PID 和电机调试模式，使 OLED 长按保存后的
参数在断电重启后仍能恢复。

================================================================================
【函数定义】
- SettingsStore_Init：初始化 W25Q64 并扫描最后一个 4 KiB 日志扇区。
- SettingsStore_IsReady：返回 Flash 和参数日志是否可用。
- SettingsStore_HasValidRecord：判断是否找到有效历史记录。
- SettingsStore_GetJedecId：返回初始化时读取的 JEDEC ID。
- SettingsStore_Load：读取最新有效参数到 AppPersistentSettings。
- SettingsStore_Save：以追加日志方式保存一条带 CRC 的新参数记录。

================================================================================
【使用说明】
1. 上电先调用 SettingsStore_Init，再根据 HasValidRecord 决定加载或用默认值。
2. Save 会在日志写满时擦除扇区，调用期间不要切断 Flash 供电。
3. 参数合法范围来自 user_config.h，非法数据不会写入或加载。
================================================================================
*/
#ifndef SETTINGS_STORE_H_
#define SETTINGS_STORE_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float target_speed_cm_s;
    float target_distance_cm;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float straight_pid_kp;
    float straight_pid_ki;
    float straight_pid_kd;
    float line_pid_kp;
    float line_pid_ki;
    float line_pid_kd;
    float angle_pid_kp;
    float angle_pid_ki;
    float angle_pid_kd;
    uint8_t motor_mode;
    uint8_t selected_test_index;
    float square_speed_cm_s;
    float square_side_cm;
    float circle_speed_cm_s;
    float circle_radius_cm;
} AppPersistentSettings;

/* Initializes W25Q64 and scans the journal in its last 4-KiB sector. */
bool SettingsStore_Init(void);
bool SettingsStore_IsReady(void);
bool SettingsStore_HasValidRecord(void);
uint32_t SettingsStore_GetJedecId(void);

bool SettingsStore_Load(AppPersistentSettings *settings);
bool SettingsStore_Save(const AppPersistentSettings *settings);

#endif /* SETTINGS_STORE_H_ */
