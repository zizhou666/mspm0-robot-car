/*
================================================================================
ICM42688 数据采集、校准与姿态解算接口模块
================================================================================
【功能简介】
本文件声明 ICM42688 软件 I2C 驱动、非阻塞零偏校准和 Mahony 姿态融合接口。
传感器坐标可映射到车体前/左/上坐标，输出加速度、角速度和欧拉角。

================================================================================
【函数定义】
- ICM42688_Init：检测 WHO_AM_I、配置量程/滤波/采样率并建立初始姿态。
- ICM42688_BeginCalibration：启动非阻塞陀螺零偏校准。
- ICM42688_ProcessCalibration：执行一次校准采样尝试。
- ICM42688_GetCalibrationState：读取当前校准状态。
- ICM42688_GetCalibrationProgress：读取校准次数和完成百分比。
- ICM42688_CalibrateGyro：阻塞式校准兼容接口，新代码不建议使用。
- ICM42688_Update：读取传感器并按 dt 更新 Mahony 姿态。
- ICM42688_GetData：读取最近一次三轴加速度和角速度。
- ICM42688_GetAttitude：读取 Roll、Pitch、Yaw。
- ICM42688_ResetYaw：把当前姿态设为新的零航向参考。
- ICM42688_IsReady：查询初始化和校准是否完成。
- ICM42688_GetI2CErrorCount：读取累计软件 I2C 错误数。
- ICM42688_GetLastStatus：读取最近一次驱动状态码。

================================================================================
【使用说明】
1. PA2=SCL、PB14=SDA，二者需要外接上拉；nCS 拉高，AD0 默认接地。
2. 先 Init，再按 10 ms 周期调用 ProcessCalibration，完成后调用 Update。
3. 校准期间传感器必须静止；不要在中断中执行软件 I2C 通信。
4. 安装方向、I2C 频率和 Mahony 参数统一在 user_config.h 中修改。
================================================================================
*/
#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>
#include "user_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ICM42688_MOUNT_FACE_UP = 0,
    ICM42688_MOUNT_FLIP_X,
    ICM42688_MOUNT_FLIP_Y,
    ICM42688_MOUNT_ROTATE_Z_180,
    ICM42688_MOUNT_SENSOR_Y_FORWARD_X_LEFT
} ICM42688_MountMode_t;

typedef enum
{
    ICM42688_STATUS_OK = 0,
    ICM42688_STATUS_ERROR_ARGUMENT,
    ICM42688_STATUS_ERROR_SOFT_RESET,
    ICM42688_STATUS_ERROR_WHO_READ,
    ICM42688_STATUS_ERROR_WHO_VALUE,
    ICM42688_STATUS_ERROR_BANK_SELECT,
    ICM42688_STATUS_ERROR_GYRO_CONFIG,
    ICM42688_STATUS_ERROR_ACCEL_CONFIG,
    ICM42688_STATUS_ERROR_FILTER_CONFIG,
    ICM42688_STATUS_ERROR_POWER_CONFIG,
    ICM42688_STATUS_ERROR_POWER_VERIFY,
    ICM42688_STATUS_ERROR_CALIBRATION,
    ICM42688_STATUS_ERROR_FIRST_READ,
    ICM42688_STATUS_ERROR_DATA_READ,
    ICM42688_STATUS_ERROR_NOT_READY
} ICM42688_Status_t;

typedef enum
{
    ICM42688_I2C_PHASE_NONE = 0,
    ICM42688_I2C_PHASE_RECOVER,
    ICM42688_I2C_PHASE_START,
    ICM42688_I2C_PHASE_ADDRESS_WRITE,
    ICM42688_I2C_PHASE_REGISTER,
    ICM42688_I2C_PHASE_DATA,
    ICM42688_I2C_PHASE_RESTART,
    ICM42688_I2C_PHASE_ADDRESS_READ,
    ICM42688_I2C_PHASE_READ_DATA,
    ICM42688_I2C_PHASE_STOP
} ICM42688_I2CPhase_t;

/*
 * Calibration is deliberately split into small main-loop steps.  One call to
 * ICM42688_ProcessCalibration() performs at most one sensor burst read and
 * never inserts a delay.  The caller therefore controls the sample interval
 * (10 ms is appropriate for the configured 100 Hz output data rate).
 */
typedef enum
{
    ICM42688_CALIBRATION_IDLE = 0,
    ICM42688_CALIBRATION_IN_PROGRESS,
    ICM42688_CALIBRATION_COMPLETE,
    ICM42688_CALIBRATION_ERROR
} ICM42688_CalibrationState_t;

typedef struct
{
    ICM42688_CalibrationState_t state;
    uint16_t requested_samples;
    uint16_t attempted_samples;
    uint16_t valid_samples;
    uint16_t rejected_samples;
    uint8_t percent_complete;
} ICM42688_CalibrationProgress_t;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} ICM42688_Data_t;

typedef struct
{
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
} ICM42688_Attitude_t;

/*
 * Vehicle frame:
 * X points forward, Y points left, Z points up.
 */
ICM42688_Status_t ICM42688_Init(
    ICM42688_MountMode_t mount_mode);

/*
 * Start or restart a gyro calibration.  The sensor must already have passed
 * ICM42688_Init().  This function only resets calibration state; it performs
 * no I2C transfer and contains no delay.
 */
ICM42688_Status_t ICM42688_BeginCalibration(
    uint16_t sample_count);

/*
 * Advance calibration by one attempt at the configured sample period.
 * Isolated bus-corrupted or moving samples are rejected and replaced; a
 * sustained run of bad samples still terminates calibration with an error.
 */
ICM42688_CalibrationState_t
ICM42688_ProcessCalibration(void);

ICM42688_CalibrationState_t
ICM42688_GetCalibrationState(void);

void ICM42688_GetCalibrationProgress(
    ICM42688_CalibrationProgress_t *progress);

/*
 * Blocking compatibility wrapper.  New code should use Begin/Process above
 * so that encoder, buttons and OLED service continue while calibrating.
 * Keep the sensor completely still while calibrating.
 */
ICM42688_Status_t ICM42688_CalibrateGyro(
    uint16_t sample_count);

/*
 * Call at a fixed rate.
 * For 100Hz sampling, dt_seconds = 0.010f.
 */
uint8_t ICM42688_Update(float dt_seconds);

void ICM42688_GetData(ICM42688_Data_t *data);

void ICM42688_GetAttitude(
    ICM42688_Attitude_t *attitude);

void ICM42688_ResetYaw(void);

uint8_t ICM42688_IsReady(void);

uint32_t ICM42688_GetI2CErrorCount(void);

uint8_t ICM42688_GetI2CAddress(void);

uint8_t ICM42688_GetLastWhoAmI(void);

ICM42688_I2CPhase_t ICM42688_GetLastI2CPhase(void);

uint8_t ICM42688_GetLastI2CLineState(void);

ICM42688_Status_t ICM42688_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif
