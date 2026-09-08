/*
================================================================================
ICM42688 软件 I2C、校准与 Mahony 姿态实现模块
================================================================================
【功能简介】
本文件通过 PB14/PA2 GPIO 开漏模拟 I2C，完成总线恢复、寄存器配置和连续
传感器读取；同时实现非阻塞陀螺零偏校准、车体坐标映射、Mahony 四元数融合、
欧拉角零参考和输出低通滤波。SPI 工程仅作为算法逻辑来源，本工程不启用 SPI。

================================================================================
【函数定义】
- StartupDelayMs：仅用于器件复位/上电初始化阶段的毫秒等待。
- SoftI2C_DelayHalf：产生软件 I2C 半周期。
- SoftI2C_SdaLow/SoftI2C_SdaRelease/SoftI2C_SdaIsHigh：控制并读取 SDA 开漏线。
- SoftI2C_SclLow/SoftI2C_SclRelease/SoftI2C_SclIsHigh：控制并读取 SCL 开漏线。
- SoftI2C_SclReleaseWaitHigh：释放 SCL 并等待时钟拉伸结束。
- SoftI2C_Stop：产生停止条件并验证总线释放。
- SoftI2C_RecoverBus：发送最多九个 SCL 脉冲恢复被从机占用的总线。
- SoftI2C_Start/SoftI2C_RepeatedStart：产生起始和重复起始条件。
- SoftI2C_WriteByte/SoftI2C_ReadByte：按位发送、接收字节并处理 ACK/NACK。
- SoftI2C_Init：释放总线并在必要时执行恢复序列。
- WriteRegister/ReadRegister/ReadRegisters：封装 ICM42688 寄存器收发。
- WriteRegisterVerify：写寄存器后回读验证配置值。
- BytesToInt16：把传感器大端字节转换为有符号原始量。
- ReadRaw：突发读取六轴原始数据。
- MapSensorToVehicle：按安装方式映射为车体前/左/上坐标。
- StoreScaledData/ReadScaledData：保存或读取工程单位的加速度和角速度。
- ClampFloat/WrapAngle180：姿态算法限幅和角度回绕工具。
- ResetAttitudeState：清空四元数、积分、零参考和输出滤波状态。
- InitAttitudeFromAccel：利用重力向量建立初始横滚/俯仰四元数。
- MahonyUpdate：执行加速度可信门控、PI 修正和四元数积分。
- ICM42688_InitOnce：执行一次总线恢复、复位、识别和寄存器配置，供启动重试内部调用。
- ICM42688_Init：等待冷启动电源稳定并自动重试初始化，对外提供统一初始化接口。
- ICM42688_BeginCalibration：清零统计并启动指定样本数的非阻塞校准。
- ICM42688_ProcessCalibration：执行一次读取并更新零偏/稳定性统计。
- ICM42688_GetCalibrationState/ICM42688_GetCalibrationProgress：读取校准状态和进度。
- ICM42688_CalibrateGyro：循环调用非阻塞接口的阻塞兼容封装。
- ICM42688_Update：读取六轴数据并按 dt 更新姿态和滤波输出。
- ICM42688_GetData/ICM42688_GetAttitude：复制最近一次传感器和欧拉角结果。
- ICM42688_ResetYaw：记录当前四元数为新的相对姿态零参考。
- ICM42688_IsReady/ICM42688_GetI2CErrorCount/ICM42688_GetLastStatus：读取健康与诊断信息。

================================================================================
【使用说明】
1. PA2=SCL、PB14=SDA，均按开漏方式工作并需要 3.3 V 外部上拉。
2. 软件 I2C、校准和 Update 只能在主循环调用；中断内禁止执行。
3. 上电校准期间保持静止，校准失败不会阻止编码器、电机和 OLED 继续工作。
4. 地址、总线速率、安装方向、校准和 Mahony 参数在 user_config.h 调整。
================================================================================
*/
#include "icm42688.h"

#include <math.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "user_config.h"

#define ICM42688_DEVICE_CONFIG          (0x11U)
#define ICM42688_TEMP_DATA1             (0x1DU)
#define ICM42688_ACCEL_DATA_X1          (0x1FU)
#define ICM42688_PWR_MGMT0              (0x4EU)
#define ICM42688_GYRO_CONFIG0           (0x4FU)
#define ICM42688_ACCEL_CONFIG0          (0x50U)
#define ICM42688_GYRO_ACCEL_CONFIG0     (0x52U)
#define ICM42688_WHO_AM_I               (0x75U)
#define ICM42688_REG_BANK_SEL           (0x76U)

#define ICM42688_CHIP_ID                (0x47U)
#define SOFT_I2C_HALF_CYCLES            \
    (CPUCLK_FREQ / (2U * SOFT_I2C_BUS_HZ))
#define SOFT_I2C_CYCLES_PER_US          (CPUCLK_FREQ / 1000000U)

#if ((SOFT_I2C_BUS_HZ < 25000U) || (SOFT_I2C_BUS_HZ > 400000U))
#error "SOFT_I2C_BUS_HZ must be between 25 kHz and 400 kHz"
#endif

#if (ICM42688_INIT_RETRY_COUNT < 1U)
#error "ICM42688_INIT_RETRY_COUNT must be at least 1"
#endif

#if (ICM42688_TRANSACTION_RETRY_COUNT < 1U)
#error "ICM42688_TRANSACTION_RETRY_COUNT must be at least 1"
#endif

#if ((ICM42688_FIXED_VALUE_RETRY_COUNT < 1U) || \
     (ICM42688_CONFIG_VERIFY_RETRY_COUNT < 1U))
#error "ICM42688 fixed-register retry counts must be at least 1"
#endif

#if (SOFT_I2C_PHASE_DITHER_STEPS < 1U)
#error "SOFT_I2C_PHASE_DITHER_STEPS must be at least 1"
#endif

#define DEG_TO_RAD                      (0.017453292519943295f)
#define RAD_TO_DEG                      (57.29577951308232f)

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;
} ICM42688_RawData_t;

typedef struct
{
    float gx_dps;
    float gy_dps;
    float gz_dps;
} GyroBias_t;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

    float integral_x;
    float integral_y;
    float integral_z;
} AhrsState_t;

typedef struct
{
    ICM42688_CalibrationState_t state;

    uint16_t requested_samples;
    uint16_t attempted_samples;
    uint16_t valid_samples;
    uint16_t consecutive_failures;
    uint16_t rejected_samples;

    float sum_gx_dps;
    float sum_gy_dps;
    float sum_gz_dps;

    /* Most recent valid mapped sample, used to seed the attitude solution. */
    float last_ax;
    float last_ay;
    float last_az;
    float last_gx;
    float last_gy;
    float last_gz;

    float min_gx;
    float min_gy;
    float min_gz;
    float max_gx;
    float max_gy;
    float max_gz;
} CalibrationState_t;

static ICM42688_MountMode_t g_mount_mode =
    ICM42688_MOUNT_FACE_UP;

static ICM42688_Data_t g_data;
static ICM42688_Attitude_t g_attitude;
static GyroBias_t g_gyro_bias;
static AhrsState_t g_ahrs;
static CalibrationState_t g_calibration;

static float g_zero_q0 = 1.0f;
static float g_zero_q1 = 0.0f;
static float g_zero_q2 = 0.0f;
static float g_zero_q3 = 0.0f;
static float g_filtered_roll = 0.0f;
static float g_filtered_pitch = 0.0f;
static float g_filtered_yaw = 0.0f;
static uint8_t g_attitude_initialized = 0U;
static uint8_t g_zero_reference_set = 0U;
static uint8_t g_output_initialized = 0U;

static uint8_t g_initialized = 0U;
static uint8_t g_ready = 0U;
static uint32_t g_i2c_error_count = 0U;
static uint8_t g_i2c_address = ICM42688_I2C_ADDRESS;
static uint8_t g_last_who_am_i = 0xFFU;
static uint8_t g_i2c_dither_step = 0U;
static ICM42688_RawData_t g_raw_history[2];
static uint8_t g_raw_history_count = 0U;
static ICM42688_I2CPhase_t g_last_i2c_phase = ICM42688_I2C_PHASE_NONE;
static uint8_t g_last_i2c_line_state = 0x03U;

static ICM42688_Status_t g_last_status =
    ICM42688_STATUS_ERROR_NOT_READY;

static void StartupDelayMs(uint32_t milliseconds)
{
    while (milliseconds > 0U)
    {
        delay_cycles(CPUCLK_FREQ / 1000U);
        milliseconds--;
    }
}

/*
 * PB14 (SDA) and PA2 (SCL) are configured as pulled-up GPIO inputs by
 * SysConfig. Open-drain low is emulated by enabling a low output; high is
 * released to the internal backup and external 4.7-kohm pull-ups. The
 * complete transaction runs in the main loop and never masks interrupts.
 */
static void SoftI2C_DelayHalf(void)
{
    delay_cycles(SOFT_I2C_HALF_CYCLES);
}

static void SoftI2C_SdaLow(void)
{
    DL_GPIO_clearPins(ICM_SDA_PORT, ICM_SDA_SW_SDA_PIN);
    DL_GPIO_enableOutput(ICM_SDA_PORT, ICM_SDA_SW_SDA_PIN);
}

static void SoftI2C_SdaRelease(void)
{
    DL_GPIO_disableOutput(ICM_SDA_PORT, ICM_SDA_SW_SDA_PIN);
}

static uint8_t SoftI2C_SdaIsHigh(void)
{
    return ((DL_GPIO_readPins(ICM_SDA_PORT, ICM_SDA_SW_SDA_PIN) &
             ICM_SDA_SW_SDA_PIN) != 0U) ? 1U : 0U;
}

/* Majority voting rejects narrow spikes while SCL is high. */
static uint8_t SoftI2C_SdaReadFiltered(void)
{
    uint8_t sample;
    uint8_t high_count = 0U;

    for (sample = 0U; sample < 3U; sample++)
    {
        if (SoftI2C_SdaIsHigh() != 0U)
        {
            high_count++;
        }

        if (sample < 2U)
        {
            delay_cycles(SOFT_I2C_FILTER_SAMPLE_DELAY_US *
                         SOFT_I2C_CYCLES_PER_US);
        }
    }

    return (high_count >= 2U) ? 1U : 0U;
}

/* Sweep reads across the 1 kHz motor-PWM period instead of phase-locking. */
static void SoftI2C_DelayPhaseDither(void)
{
    uint32_t delay_us =
        (uint32_t)g_i2c_dither_step * SOFT_I2C_PHASE_DITHER_STEP_US;

    g_i2c_dither_step++;
    if (g_i2c_dither_step >= SOFT_I2C_PHASE_DITHER_STEPS)
    {
        g_i2c_dither_step = 0U;
    }

    if (delay_us != 0U)
    {
        delay_cycles(delay_us * SOFT_I2C_CYCLES_PER_US);
    }
}

static uint8_t SoftI2C_SdaReleaseWaitHigh(void)
{
    uint32_t timeout_us = SOFT_I2C_STRETCH_TIMEOUT_US;

    SoftI2C_SdaRelease();

    while (timeout_us > 0U)
    {
        if (SoftI2C_SdaIsHigh() != 0U)
        {
            return 1U;
        }

        delay_cycles(SOFT_I2C_CYCLES_PER_US);
        timeout_us--;
    }

    return 0U;
}

static void SoftI2C_SclLow(void)
{
    DL_GPIO_clearPins(ICM_SCL_PORT, ICM_SCL_SW_SCL_PIN);
    DL_GPIO_enableOutput(ICM_SCL_PORT, ICM_SCL_SW_SCL_PIN);
}

static void SoftI2C_SclRelease(void)
{
    DL_GPIO_disableOutput(ICM_SCL_PORT, ICM_SCL_SW_SCL_PIN);
}

static uint8_t SoftI2C_SclIsHigh(void)
{
    return ((DL_GPIO_readPins(ICM_SCL_PORT, ICM_SCL_SW_SCL_PIN) &
             ICM_SCL_SW_SCL_PIN) != 0U) ? 1U : 0U;
}

static uint8_t SoftI2C_ReadLineState(void)
{
    uint8_t state = 0U;

    if (SoftI2C_SclIsHigh() != 0U)
    {
        state |= 0x01U;
    }
    if (SoftI2C_SdaIsHigh() != 0U)
    {
        state |= 0x02U;
    }

    return state;
}

static void SoftI2C_RecordFailure(ICM42688_I2CPhase_t phase)
{
    g_last_i2c_phase = phase;
    g_last_i2c_line_state = SoftI2C_ReadLineState();
    g_i2c_error_count++;
}

static uint8_t SoftI2C_SclReleaseWaitHigh(void)
{
    uint32_t timeout_us = SOFT_I2C_STRETCH_TIMEOUT_US;

    SoftI2C_SclRelease();

    while (timeout_us > 0U)
    {
        if (SoftI2C_SclIsHigh() != 0U)
        {
            return 1U;
        }

        delay_cycles(SOFT_I2C_CYCLES_PER_US);
        timeout_us--;
    }

    return 0U;
}

static uint8_t SoftI2C_Stop(void)
{
    uint8_t scl_released;
    uint8_t sda_released;

    SoftI2C_SclLow();
    SoftI2C_SdaLow();
    SoftI2C_DelayHalf();

    scl_released = SoftI2C_SclReleaseWaitHigh();
    if (scl_released != 0U)
    {
        SoftI2C_DelayHalf();
    }

    sda_released = SoftI2C_SdaReleaseWaitHigh();
    if (sda_released != 0U)
    {
        SoftI2C_DelayHalf();
    }

    return ((scl_released != 0U) &&
            (sda_released != 0U) &&
            (SoftI2C_SclIsHigh() != 0U) &&
            (SoftI2C_SdaIsHigh() != 0U)) ? 1U : 0U;
}

static uint8_t SoftI2C_RecoverBus(void)
{
    uint8_t pulse;

    SoftI2C_SdaRelease();

    if (SoftI2C_SclReleaseWaitHigh() == 0U)
    {
        SoftI2C_SdaRelease();
        SoftI2C_SclRelease();
        return 0U;
    }

    SoftI2C_DelayHalf();

    if (SoftI2C_SdaIsHigh() != 0U)
    {
        return 1U;
    }

    for (pulse = 0U; pulse < 9U; pulse++)
    {
        SoftI2C_SclLow();
        SoftI2C_DelayHalf();

        if (SoftI2C_SclReleaseWaitHigh() == 0U)
        {
            SoftI2C_SdaRelease();
            SoftI2C_SclRelease();
            return 0U;
        }

        SoftI2C_DelayHalf();

        if (SoftI2C_SdaIsHigh() != 0U)
        {
            break;
        }
    }

    return SoftI2C_Stop();
}

static uint8_t SoftI2C_Start(void)
{
    SoftI2C_SdaRelease();

    if (SoftI2C_SclReleaseWaitHigh() == 0U)
    {
        return 0U;
    }

    SoftI2C_DelayHalf();

    if (SoftI2C_SdaIsHigh() == 0U)
    {
        return 0U;
    }

    SoftI2C_SdaLow();
    SoftI2C_DelayHalf();
    SoftI2C_SclLow();

    return 1U;
}

static uint8_t SoftI2C_RepeatedStart(void)
{
    SoftI2C_SclLow();
    SoftI2C_SdaRelease();
    SoftI2C_DelayHalf();

    if (SoftI2C_SclReleaseWaitHigh() == 0U)
    {
        return 0U;
    }

    SoftI2C_DelayHalf();

    if (SoftI2C_SdaIsHigh() == 0U)
    {
        return 0U;
    }

    SoftI2C_SdaLow();
    SoftI2C_DelayHalf();
    SoftI2C_SclLow();

    return 1U;
}

static uint8_t SoftI2C_WriteByte(uint8_t value)
{
    uint8_t bit;
    uint8_t acknowledged;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((value & 0x80U) != 0U)
        {
            SoftI2C_SdaRelease();
        }
        else
        {
            SoftI2C_SdaLow();
        }

        SoftI2C_DelayHalf();

        if (SoftI2C_SclReleaseWaitHigh() == 0U)
        {
            SoftI2C_SdaRelease();
            return 0U;
        }

        SoftI2C_DelayHalf();
        SoftI2C_SclLow();
        value <<= 1U;
    }

    SoftI2C_SdaRelease();
    SoftI2C_DelayHalf();

    if (SoftI2C_SclReleaseWaitHigh() == 0U)
    {
        return 0U;
    }

    SoftI2C_DelayHalf();
    acknowledged = (SoftI2C_SdaReadFiltered() == 0U) ? 1U : 0U;
    SoftI2C_SclLow();

    return acknowledged;
}

static uint8_t SoftI2C_ReadByte(uint8_t *value, uint8_t send_ack)
{
    uint8_t bit;
    uint8_t data = 0U;

    if (value == 0)
    {
        return 0U;
    }

    SoftI2C_SdaRelease();

    for (bit = 0U; bit < 8U; bit++)
    {
        data <<= 1U;
        SoftI2C_DelayHalf();

        if (SoftI2C_SclReleaseWaitHigh() == 0U)
        {
            SoftI2C_SdaRelease();
            return 0U;
        }

        SoftI2C_DelayHalf();

        if (SoftI2C_SdaReadFiltered() != 0U)
        {
            data |= 0x01U;
        }

        SoftI2C_SclLow();
    }

    if (send_ack != 0U)
    {
        SoftI2C_SdaLow();
    }
    else
    {
        SoftI2C_SdaRelease();
    }

    SoftI2C_DelayHalf();

    if (SoftI2C_SclReleaseWaitHigh() == 0U)
    {
        SoftI2C_SdaRelease();
        return 0U;
    }

    SoftI2C_DelayHalf();
    SoftI2C_SclLow();
    SoftI2C_SdaRelease();
    *value = data;

    return 1U;
}

static uint8_t SoftI2C_Init(void)
{
    DL_GPIO_initDigitalInputFeatures(ICM_SDA_SW_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ICM_SCL_SW_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* Preload zero before an output enable can ever be asserted. */
    DL_GPIO_clearPins(ICM_SDA_PORT, ICM_SDA_SW_SDA_PIN);
    DL_GPIO_clearPins(ICM_SCL_PORT, ICM_SCL_SW_SCL_PIN);
    SoftI2C_SdaRelease();
    SoftI2C_SclRelease();

    return SoftI2C_RecoverBus();
}

static uint8_t WriteRegisterOnce(
    uint8_t reg,
    uint8_t value)
{
    if (SoftI2C_RecoverBus() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_RECOVER);
        return 0U;
    }
    if (SoftI2C_Start() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_START);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte((uint8_t)(g_i2c_address << 1U)) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_ADDRESS_WRITE);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte(reg) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_REGISTER);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte(value) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_DATA);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_Stop() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_STOP);
        return 0U;
    }

    g_last_i2c_phase = ICM42688_I2C_PHASE_NONE;
    g_last_i2c_line_state = 0x03U;
    return 1U;
}

static uint8_t WriteRegister(
    uint8_t reg,
    uint8_t value)
{
    uint32_t attempt;

    for (attempt = 0U;
         attempt < ICM42688_TRANSACTION_RETRY_COUNT;
         attempt++)
    {
        if (WriteRegisterOnce(reg, value) != 0U)
        {
            return 1U;
        }

        if ((attempt + 1U) < ICM42688_TRANSACTION_RETRY_COUNT)
        {
            StartupDelayMs(ICM42688_TRANSACTION_RETRY_DELAY_MS);
        }
    }

    return 0U;
}

static uint8_t WriteSoftResetCommandOnce(void)
{
    if (SoftI2C_RecoverBus() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_RECOVER);
        return 0U;
    }
    if (SoftI2C_Start() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_START);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte((uint8_t)(g_i2c_address << 1U)) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_ADDRESS_WRITE);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte(ICM42688_DEVICE_CONFIG) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_REGISTER);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte(0x01U) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_DATA);
        (void)SoftI2C_Stop();
        return 0U;
    }

    /* Reset starts immediately after the data byte. Generate STOP, then let
     * the post-reset recovery verify that both bus lines are usable again. */
    (void)SoftI2C_Stop();
    g_last_i2c_phase = ICM42688_I2C_PHASE_NONE;
    g_last_i2c_line_state = 0x03U;
    return 1U;
}

static uint8_t WriteSoftResetCommand(void)
{
    uint32_t attempt;

    for (attempt = 0U;
         attempt < ICM42688_TRANSACTION_RETRY_COUNT;
         attempt++)
    {
        if (WriteSoftResetCommandOnce() != 0U)
        {
            return 1U;
        }

        if ((attempt + 1U) < ICM42688_TRANSACTION_RETRY_COUNT)
        {
            StartupDelayMs(ICM42688_TRANSACTION_RETRY_DELAY_MS);
        }
    }

    return 0U;
}

static uint8_t ReadRegistersOnce(
    uint8_t reg,
    uint8_t *buffer,
    uint8_t length)
{
    uint8_t count;

    if ((buffer == 0) ||
        (length == 0U) ||
        (length > 16U))
    {
        return 0U;
    }

    if (SoftI2C_RecoverBus() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_RECOVER);
        return 0U;
    }
    if (SoftI2C_Start() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_START);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte((uint8_t)(g_i2c_address << 1U)) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_ADDRESS_WRITE);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte(reg) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_REGISTER);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_RepeatedStart() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_RESTART);
        (void)SoftI2C_Stop();
        return 0U;
    }
    if (SoftI2C_WriteByte(
            (uint8_t)((g_i2c_address << 1U) | 0x01U)) == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_ADDRESS_READ);
        (void)SoftI2C_Stop();
        return 0U;
    }

    for (count = 0U; count < length; count++)
    {
        uint8_t send_ack = ((uint8_t)(count + 1U) < length) ? 1U : 0U;

        if (SoftI2C_ReadByte(&buffer[count], send_ack) == 0U)
        {
            SoftI2C_RecordFailure(ICM42688_I2C_PHASE_READ_DATA);
            (void)SoftI2C_Stop();
            return 0U;
        }
    }

    if (SoftI2C_Stop() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_STOP);
        return 0U;
    }

    g_last_i2c_phase = ICM42688_I2C_PHASE_NONE;
    g_last_i2c_line_state = 0x03U;
    return 1U;
}

static uint8_t ReadRegisters(
    uint8_t reg,
    uint8_t *buffer,
    uint8_t length)
{
    uint32_t attempt;

    for (attempt = 0U;
         attempt < ICM42688_TRANSACTION_RETRY_COUNT;
         attempt++)
    {
        SoftI2C_DelayPhaseDither();

        if (ReadRegistersOnce(reg, buffer, length) != 0U)
        {
            return 1U;
        }

        if ((attempt + 1U) < ICM42688_TRANSACTION_RETRY_COUNT)
        {
            StartupDelayMs(ICM42688_TRANSACTION_RETRY_DELAY_MS);
        }
    }

    return 0U;
}

static uint8_t ReadRegister(
    uint8_t reg,
    uint8_t *value)
{
    return ReadRegisters(
        reg,
        value,
        1U);
}

static uint8_t ReadRegisterExpected(
    uint8_t reg,
    uint8_t mask,
    uint8_t expected,
    uint8_t *last_value,
    uint8_t *response_seen)
{
    uint32_t attempt;
    uint8_t value = 0xFFU;

    if ((last_value == 0) || (response_seen == 0))
    {
        return 0U;
    }

    *last_value = 0xFFU;
    *response_seen = 0U;
    for (attempt = 0U;
         attempt < ICM42688_FIXED_VALUE_RETRY_COUNT;
         attempt++)
    {
        if (ReadRegister(reg, &value) != 0U)
        {
            *response_seen = 1U;
            *last_value = value;
            if ((value & mask) == expected)
            {
                return 1U;
            }
        }

        if ((attempt + 1U) < ICM42688_FIXED_VALUE_RETRY_COUNT)
        {
            StartupDelayMs(ICM42688_TRANSACTION_RETRY_DELAY_MS);
        }
    }

    return 0U;
}

static uint8_t SelectI2CAddress(void)
{
    const uint8_t addresses[2] = {
        ICM42688_I2C_ADDRESS,
        (ICM42688_I2C_ADDRESS == 0x68U) ? 0x69U : 0x68U
    };
    uint8_t first_response_address = ICM42688_I2C_ADDRESS;
    uint8_t first_response_who = 0xFFU;
    uint8_t response_seen = 0U;
    uint8_t index;

    for (index = 0U; index < 2U; index++)
    {
        uint8_t who_am_i = 0xFFU;
        uint8_t address_responded = 0U;

        g_i2c_address = addresses[index];
        if (ReadRegisterExpected(
                ICM42688_WHO_AM_I,
                0xFFU,
                ICM42688_CHIP_ID,
                &who_am_i,
                &address_responded) == 0U)
        {
            if ((response_seen == 0U) && (address_responded != 0U))
            {
                first_response_address = g_i2c_address;
                first_response_who = who_am_i;
                response_seen = 1U;
            }
            continue;
        }

        g_last_who_am_i = who_am_i;
        return 1U;
    }

    if (response_seen != 0U)
    {
        g_i2c_address = first_response_address;
        g_last_who_am_i = first_response_who;
        g_last_status = ICM42688_STATUS_ERROR_WHO_VALUE;
    }
    else
    {
        g_i2c_address = ICM42688_I2C_ADDRESS;
        g_last_who_am_i = 0xFFU;
        g_last_status = ICM42688_STATUS_ERROR_WHO_READ;
    }

    return 0U;
}

static uint8_t WriteRegisterVerify(uint8_t reg, uint8_t value)
{
    uint32_t attempt;

    for (attempt = 0U;
         attempt < ICM42688_CONFIG_VERIFY_RETRY_COUNT;
         attempt++)
    {
        uint8_t readback = 0U;

        if ((WriteRegister(reg, value) != 0U) &&
            (ReadRegister(reg, &readback) != 0U) &&
            (readback == value))
        {
            return 1U;
        }

        if ((attempt + 1U) < ICM42688_CONFIG_VERIFY_RETRY_COUNT)
        {
            StartupDelayMs(ICM42688_TRANSACTION_RETRY_DELAY_MS);
        }
    }

    return 0U;
}

static int16_t BytesToInt16(
    uint8_t high_byte,
    uint8_t low_byte)
{
    return (int16_t)(
        ((uint16_t)high_byte << 8) |
        (uint16_t)low_byte);
}

static uint8_t ReadRaw(
    ICM42688_RawData_t *raw)
{
    uint8_t bytes[12];
    uint8_t all_zero = 1U;
    uint8_t all_one = 1U;
    uint8_t index;

    if ((raw == 0) ||
        (ReadRegisters(
             ICM42688_ACCEL_DATA_X1,
             bytes,
             (uint8_t)sizeof(bytes)) == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < (uint8_t)sizeof(bytes); ++index)
    {
        if (bytes[index] != 0x00U)
        {
            all_zero = 0U;
        }
        if (bytes[index] != 0xFFU)
        {
            all_one = 0U;
        }
    }
    if ((all_zero != 0U) || (all_one != 0U))
    {
        return 0U;
    }

    raw->ax = BytesToInt16(bytes[0], bytes[1]);
    raw->ay = BytesToInt16(bytes[2], bytes[3]);
    raw->az = BytesToInt16(bytes[4], bytes[5]);
    raw->gx = BytesToInt16(bytes[6], bytes[7]);
    raw->gy = BytesToInt16(bytes[8], bytes[9]);
    raw->gz = BytesToInt16(bytes[10], bytes[11]);

    return 1U;
}

static void MapSensorToVehicle(
    const ICM42688_RawData_t *raw,
    float *ax,
    float *ay,
    float *az,
    float *gx,
    float *gy,
    float *gz)
{
    switch (g_mount_mode)
    {
        case ICM42688_MOUNT_FLIP_X:
            *ax = (float)raw->ax;
            *ay = -(float)raw->ay;
            *az = -(float)raw->az;

            *gx = (float)raw->gx;
            *gy = -(float)raw->gy;
            *gz = -(float)raw->gz;
            break;

        case ICM42688_MOUNT_FLIP_Y:
            *ax = -(float)raw->ax;
            *ay = (float)raw->ay;
            *az = -(float)raw->az;

            *gx = -(float)raw->gx;
            *gy = (float)raw->gy;
            *gz = -(float)raw->gz;
            break;

        case ICM42688_MOUNT_ROTATE_Z_180:
            *ax = -(float)raw->ax;
            *ay = -(float)raw->ay;
            *az = (float)raw->az;

            *gx = -(float)raw->gx;
            *gy = -(float)raw->gy;
            *gz = (float)raw->gz;
            break;

        case ICM42688_MOUNT_SENSOR_Y_FORWARD_X_LEFT:
            /* Sensor +Y is forward, +X is left and +Z points down. */
            *ax = (float)raw->ay;
            *ay = (float)raw->ax;
            *az = -(float)raw->az;

            *gx = (float)raw->gy;
            *gy = (float)raw->gx;
            *gz = -(float)raw->gz;
            break;

        case ICM42688_MOUNT_FACE_UP:
        default:
            *ax = (float)raw->ax;
            *ay = (float)raw->ay;
            *az = (float)raw->az;

            *gx = (float)raw->gx;
            *gy = (float)raw->gy;
            *gz = (float)raw->gz;
            break;
    }
}

static void StoreScaledData(
    ICM42688_Data_t *data,
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz)
{
    data->accel_x_g =
        ax / ACCEL_SCALE_LSB_PER_G;

    data->accel_y_g =
        ay / ACCEL_SCALE_LSB_PER_G;

    data->accel_z_g =
        az / ACCEL_SCALE_LSB_PER_G;

    data->gyro_x_dps =
        (gx / GYRO_SCALE_LSB_PER_DPS) -
        g_gyro_bias.gx_dps;

    data->gyro_y_dps =
        (gy / GYRO_SCALE_LSB_PER_DPS) -
        g_gyro_bias.gy_dps;

    data->gyro_z_dps =
        ((gz / GYRO_SCALE_LSB_PER_DPS) -
         g_gyro_bias.gz_dps) * ICM42688_YAW_RATE_GAIN;
}

static int16_t Median3Int16(int16_t a, int16_t b, int16_t c)
{
    if (a > b)
    {
        int16_t temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c)
    {
        int16_t temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b)
    {
        b = a;
    }

    return b;
}

static void FilterRawMedian3(
    const ICM42688_RawData_t *input,
    ICM42688_RawData_t *output)
{
    if (g_raw_history_count < 2U)
    {
        g_raw_history[g_raw_history_count] = *input;
        g_raw_history_count++;
        *output = *input;
        return;
    }

    output->ax = Median3Int16(
        g_raw_history[0].ax, g_raw_history[1].ax, input->ax);
    output->ay = Median3Int16(
        g_raw_history[0].ay, g_raw_history[1].ay, input->ay);
    output->az = Median3Int16(
        g_raw_history[0].az, g_raw_history[1].az, input->az);
    output->gx = Median3Int16(
        g_raw_history[0].gx, g_raw_history[1].gx, input->gx);
    output->gy = Median3Int16(
        g_raw_history[0].gy, g_raw_history[1].gy, input->gy);
    output->gz = Median3Int16(
        g_raw_history[0].gz, g_raw_history[1].gz, input->gz);

    g_raw_history[0] = g_raw_history[1];
    g_raw_history[1] = *input;
}

static uint8_t ReadScaledData(
    ICM42688_Data_t *data)
{
    ICM42688_RawData_t raw;
    ICM42688_RawData_t filtered_raw;

    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;

    if ((data == 0) ||
        (ReadRaw(&raw) == 0U))
    {
        return 0U;
    }

    FilterRawMedian3(&raw, &filtered_raw);

    MapSensorToVehicle(
        &filtered_raw,
        &ax,
        &ay,
        &az,
        &gx,
        &gy,
        &gz);

    StoreScaledData(
        data,
        ax,
        ay,
        az,
        gx,
        gy,
        gz);

    return 1U;
}

static float ClampFloat(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

static float WrapAngle180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void ResetAttitudeState(void)
{
    g_raw_history_count = 0U;
    g_ahrs.q0 = 1.0f;
    g_ahrs.q1 = 0.0f;
    g_ahrs.q2 = 0.0f;
    g_ahrs.q3 = 0.0f;
    g_ahrs.integral_x = 0.0f;
    g_ahrs.integral_y = 0.0f;
    g_ahrs.integral_z = 0.0f;
    g_zero_q0 = 1.0f;
    g_zero_q1 = 0.0f;
    g_zero_q2 = 0.0f;
    g_zero_q3 = 0.0f;
    g_filtered_roll = 0.0f;
    g_filtered_pitch = 0.0f;
    g_filtered_yaw = 0.0f;
    g_attitude.roll_deg = 0.0f;
    g_attitude.pitch_deg = 0.0f;
    g_attitude.yaw_deg = 0.0f;
    g_attitude_initialized = 0U;
    g_zero_reference_set = 0U;
    g_output_initialized = 0U;
}

static uint8_t InitAttitudeFromAccel(float ax, float ay, float az)
{
    float norm2 = ax * ax + ay * ay + az * az;
    float inverse_norm;

    if (!(norm2 > 0.01f))
    {
        return 0U;
    }

    inverse_norm = 1.0f / sqrtf(norm2);
    ax *= inverse_norm;
    ay *= inverse_norm;
    az *= inverse_norm;

    if (az > -0.999f)
    {
        g_ahrs.q0 = sqrtf(0.5f * (1.0f + az));
        g_ahrs.q1 = ay / (2.0f * g_ahrs.q0);
        g_ahrs.q2 = -ax / (2.0f * g_ahrs.q0);
        g_ahrs.q3 = 0.0f;
    }
    else
    {
        g_ahrs.q0 = 0.0f;
        g_ahrs.q1 = 1.0f;
        g_ahrs.q2 = 0.0f;
        g_ahrs.q3 = 0.0f;
    }

    g_ahrs.integral_x = 0.0f;
    g_ahrs.integral_y = 0.0f;
    g_ahrs.integral_z = 0.0f;
    g_attitude_initialized = 1U;
    return 1U;
}

static uint8_t MahonyUpdate(float dt)
{
    float q0 = g_ahrs.q0;
    float q1 = g_ahrs.q1;
    float q2 = g_ahrs.q2;
    float q3 = g_ahrs.q3;
    float gx = g_data.gyro_x_dps * DEG_TO_RAD;
    float gy = g_data.gyro_y_dps * DEG_TO_RAD;
    float gz = g_data.gyro_z_dps * DEG_TO_RAD;
    float ax = g_data.accel_x_g;
    float ay = g_data.accel_y_g;
    float az = g_data.accel_z_g;
    float accel_norm2 = ax * ax + ay * ay + az * az;
    float inverse_norm;
    float old_q0;
    float old_q1;
    float old_q2;
    float old_q3;
    float quaternion_norm2;

    if ((g_attitude_initialized == 0U) &&
        (InitAttitudeFromAccel(ax, ay, az) == 0U))
    {
        return 0U;
    }

    q0 = g_ahrs.q0;
    q1 = g_ahrs.q1;
    q2 = g_ahrs.q2;
    q3 = g_ahrs.q3;

    if ((accel_norm2 >= ACCEL_NORM2_MIN) &&
        (accel_norm2 <= ACCEL_NORM2_MAX))
    {
        float vx;
        float vy;
        float vz;
        float ex;
        float ey;
        float correction_weight;

        inverse_norm = 1.0f / sqrtf(accel_norm2);
        ax *= inverse_norm;
        ay *= inverse_norm;
        az *= inverse_norm;
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        correction_weight = 1.0f - fabsf(accel_norm2 - 1.0f) / 0.44f;
        correction_weight = ClampFloat(correction_weight, 0.0f, 1.0f);

        g_ahrs.integral_x = ClampFloat(
            g_ahrs.integral_x + MAHONY_KI * ex * correction_weight * dt,
            -INTEGRAL_LIMIT_RAD_S, INTEGRAL_LIMIT_RAD_S);
        g_ahrs.integral_y = ClampFloat(
            g_ahrs.integral_y + MAHONY_KI * ey * correction_weight * dt,
            -INTEGRAL_LIMIT_RAD_S, INTEGRAL_LIMIT_RAD_S);
        gx += MAHONY_KP * ex * correction_weight + g_ahrs.integral_x;
        gy += MAHONY_KP * ey * correction_weight + g_ahrs.integral_y;
    }
    else
    {
        g_ahrs.integral_x *= 0.999f;
        g_ahrs.integral_y *= 0.999f;
    }

    old_q0 = q0;
    old_q1 = q1;
    old_q2 = q2;
    old_q3 = q3;
    q0 += (-old_q1 * gx - old_q2 * gy - old_q3 * gz) * 0.5f * dt;
    q1 += ( old_q0 * gx - old_q3 * gy + old_q2 * gz) * 0.5f * dt;
    q2 += ( old_q3 * gx + old_q0 * gy - old_q1 * gz) * 0.5f * dt;
    q3 += (-old_q2 * gx + old_q1 * gy + old_q0 * gz) * 0.5f * dt;

    quaternion_norm2 = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (!(quaternion_norm2 > 0.000001f))
    {
        ResetAttitudeState();
        return 0U;
    }
    inverse_norm = 1.0f / sqrtf(quaternion_norm2);
    g_ahrs.q0 = q0 * inverse_norm;
    g_ahrs.q1 = q1 * inverse_norm;
    g_ahrs.q2 = q2 * inverse_norm;
    g_ahrs.q3 = q3 * inverse_norm;

    if (g_zero_reference_set == 0U)
    {
        g_zero_q0 = g_ahrs.q0;
        g_zero_q1 = g_ahrs.q1;
        g_zero_q2 = g_ahrs.q2;
        g_zero_q3 = g_ahrs.q3;
        g_zero_reference_set = 1U;
    }

    {
        float qr0 = g_zero_q0 * g_ahrs.q0 + g_zero_q1 * g_ahrs.q1 +
                    g_zero_q2 * g_ahrs.q2 + g_zero_q3 * g_ahrs.q3;
        float qr1 = g_zero_q0 * g_ahrs.q1 - g_zero_q1 * g_ahrs.q0 -
                    g_zero_q2 * g_ahrs.q3 + g_zero_q3 * g_ahrs.q2;
        float qr2 = g_zero_q0 * g_ahrs.q2 + g_zero_q1 * g_ahrs.q3 -
                    g_zero_q2 * g_ahrs.q0 - g_zero_q3 * g_ahrs.q1;
        float qr3 = g_zero_q0 * g_ahrs.q3 - g_zero_q1 * g_ahrs.q2 +
                    g_zero_q2 * g_ahrs.q1 - g_zero_q3 * g_ahrs.q0;
        float relative_norm2 = qr0 * qr0 + qr1 * qr1 + qr2 * qr2 + qr3 * qr3;
        float sin_pitch;
        float pitch_root;
        float pitch_raw;
        float roll_raw;
        float yaw_raw;
        float alpha;
        float yaw_alpha;

        if (!(relative_norm2 > 0.000001f))
        {
            ResetAttitudeState();
            return 0U;
        }
        inverse_norm = 1.0f / sqrtf(relative_norm2);
        qr0 *= inverse_norm;
        qr1 *= inverse_norm;
        qr2 *= inverse_norm;
        qr3 *= inverse_norm;

        sin_pitch = ClampFloat(2.0f * (qr1 * qr3 - qr0 * qr2),
                               -1.0f, 1.0f);
        pitch_root = 1.0f - sin_pitch * sin_pitch;
        if (pitch_root < 0.0f)
        {
            pitch_root = 0.0f;
        }
        pitch_raw = -atan2f(sin_pitch, sqrtf(pitch_root)) * RAD_TO_DEG;
        roll_raw = atan2f(2.0f * (qr0 * qr1 + qr2 * qr3),
                          1.0f - 2.0f * (qr1 * qr1 + qr2 * qr2)) *
                   RAD_TO_DEG;
        yaw_raw = atan2f(2.0f * (qr0 * qr3 + qr1 * qr2),
                         qr0 * qr0 + qr1 * qr1 - qr2 * qr2 - qr3 * qr3) *
                  RAD_TO_DEG;
        if (g_output_initialized == 0U)
        {
            g_filtered_pitch = pitch_raw;
            g_filtered_roll = roll_raw;
            g_filtered_yaw = yaw_raw;
            g_output_initialized = 1U;
        }
        else
        {
            alpha = dt / (OUTPUT_LPF_TAU_S + dt);
            yaw_alpha = dt / (YAW_OUTPUT_LPF_TAU_S + dt);
            g_filtered_pitch += alpha * (pitch_raw - g_filtered_pitch);
            g_filtered_roll = WrapAngle180(g_filtered_roll +
                alpha * WrapAngle180(roll_raw - g_filtered_roll));
            g_filtered_yaw = WrapAngle180(g_filtered_yaw +
                yaw_alpha * WrapAngle180(yaw_raw - g_filtered_yaw));
        }

        g_attitude.pitch_deg = g_filtered_pitch;
        g_attitude.roll_deg = g_filtered_roll;
        g_attitude.yaw_deg = g_filtered_yaw;
    }

    return 1U;
}

static ICM42688_Status_t ICM42688_InitOnce(
    ICM42688_MountMode_t mount_mode)
{
    uint8_t value;
    uint8_t response_seen;

    if ((mount_mode < ICM42688_MOUNT_FACE_UP) ||
        (mount_mode > ICM42688_MOUNT_SENSOR_Y_FORWARD_X_LEFT))
    {
        g_last_status =
            ICM42688_STATUS_ERROR_ARGUMENT;

        return g_last_status;
    }

    g_mount_mode = mount_mode;
    g_initialized = 0U;
    g_ready = 0U;
    g_last_who_am_i = 0xFFU;

    g_calibration.state =
        ICM42688_CALIBRATION_IDLE;
    g_calibration.requested_samples = 0U;
    g_calibration.attempted_samples = 0U;
    g_calibration.valid_samples = 0U;
    g_calibration.consecutive_failures = 0U;
    g_calibration.rejected_samples = 0U;
    g_calibration.sum_gx_dps = 0.0f;
    g_calibration.sum_gy_dps = 0.0f;
    g_calibration.sum_gz_dps = 0.0f;
    g_calibration.last_ax = 0.0f;
    g_calibration.last_ay = 0.0f;
    g_calibration.last_az = 0.0f;
    g_calibration.last_gx = 0.0f;
    g_calibration.last_gy = 0.0f;
    g_calibration.last_gz = 0.0f;
    g_calibration.min_gx = 32767.0f;
    g_calibration.min_gy = 32767.0f;
    g_calibration.min_gz = 32767.0f;
    g_calibration.max_gx = -32768.0f;
    g_calibration.max_gy = -32768.0f;
    g_calibration.max_gz = -32768.0f;

    g_gyro_bias.gx_dps = 0.0f;
    g_gyro_bias.gy_dps = 0.0f;
    g_gyro_bias.gz_dps = 0.0f;
    ResetAttitudeState();

    if (SoftI2C_Init() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_RECOVER);
        g_last_status =
            ICM42688_STATUS_ERROR_SOFT_RESET;

        return g_last_status;
    }

    if (SelectI2CAddress() == 0U)
    {
        return g_last_status;
    }

    if (WriteSoftResetCommand() == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_SOFT_RESET;

        return g_last_status;
    }

    StartupDelayMs(10U);

    if (SoftI2C_Init() == 0U)
    {
        SoftI2C_RecordFailure(ICM42688_I2C_PHASE_RECOVER);
        g_last_status =
            ICM42688_STATUS_ERROR_SOFT_RESET;

        return g_last_status;
    }

    if (ReadRegisterExpected(
            ICM42688_WHO_AM_I,
            0xFFU,
            ICM42688_CHIP_ID,
            &value,
            &response_seen) == 0U)
    {
        g_last_who_am_i = value;
        g_last_status =
            (response_seen != 0U) ?
                ICM42688_STATUS_ERROR_WHO_VALUE :
                ICM42688_STATUS_ERROR_WHO_READ;

        return g_last_status;
    }

    g_last_who_am_i = value;

    if (WriteRegisterVerify(
            ICM42688_REG_BANK_SEL,
            0x00U) == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_BANK_SELECT;

        return g_last_status;
    }

    if (WriteRegister(
            ICM42688_PWR_MGMT0,
            ICM42688_ACCEL_GYRO_LN) == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_POWER_CONFIG;

        return g_last_status;
    }

    StartupDelayMs(50U);

    if (ReadRegisterExpected(
            ICM42688_PWR_MGMT0,
            0x0FU,
            ICM42688_ACCEL_GYRO_LN,
            &value,
            &response_seen) == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_POWER_VERIFY;

        return g_last_status;
    }

    if (WriteRegisterVerify(
            ICM42688_ACCEL_CONFIG0,
            ICM42688_ACCEL_2G_100HZ) == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_ACCEL_CONFIG;

        return g_last_status;
    }

    if (WriteRegisterVerify(
            ICM42688_GYRO_CONFIG0,
            ICM42688_GYRO_500DPS_100HZ) == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_GYRO_CONFIG;

        return g_last_status;
    }

    if (WriteRegisterVerify(
            ICM42688_GYRO_ACCEL_CONFIG0,
            ICM42688_ACCEL_GYRO_BW_20HZ) == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_FILTER_CONFIG;

        return g_last_status;
    }

    /*
     * ODR、量程和数字滤波刚切换时可能出现启动瞬态。若立即统计校准
     * 最大/最小值，单个瞬态样本就会让严格的静止判据误报失败。
     */
    StartupDelayMs(ICM42688_POST_CONFIG_SETTLE_MS);

    g_initialized = 1U;
    g_last_status = ICM42688_STATUS_OK;

    return g_last_status;
}

ICM42688_Status_t ICM42688_Init(
    ICM42688_MountMode_t mount_mode)
{
    uint32_t attempt;
    ICM42688_Status_t status;

    /* 参数错误与冷启动状态无关，不应通过延时和重试掩盖。 */
    if ((mount_mode < ICM42688_MOUNT_FACE_UP) ||
        (mount_mode > ICM42688_MOUNT_SENSOR_Y_FORWARD_X_LEFT))
    {
        g_last_status = ICM42688_STATUS_ERROR_ARGUMENT;
        return g_last_status;
    }

    /*
     * MCU 冷启动可能早于传感器模块电源稳定。先留出明确的上电等待时间，
     * 避免第一次 DEVICE_CONFIG 写入发生在 ICM42688 尚不能应答的阶段。
     */
    StartupDelayMs(ICM42688_POWER_ON_DELAY_MS);
    g_i2c_error_count = 0U;

    for (attempt = 0U; attempt < ICM42688_INIT_RETRY_COUNT; attempt++)
    {
        /* InitOnce 内部每次都会调用 SoftI2C_Init，包含总线释放与 9 脉冲恢复。 */
        status = ICM42688_InitOnce(mount_mode);
        if (status == ICM42688_STATUS_OK)
        {
            return status;
        }

        if ((attempt + 1U) < ICM42688_INIT_RETRY_COUNT)
        {
            StartupDelayMs(ICM42688_INIT_RETRY_DELAY_MS);
        }
    }

    /* 保留最后一次尝试的具体故障码，供 OLED 和 Keil 调试读取。 */
    return status;
}

ICM42688_Status_t ICM42688_BeginCalibration(
    uint16_t sample_count)
{
    if (sample_count == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_ARGUMENT;

        return g_last_status;
    }

    if (g_initialized == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_NOT_READY;

        return g_last_status;
    }

    g_ready = 0U;

    g_gyro_bias.gx_dps = 0.0f;
    g_gyro_bias.gy_dps = 0.0f;
    g_gyro_bias.gz_dps = 0.0f;
    ResetAttitudeState();

    g_calibration.state =
        ICM42688_CALIBRATION_IN_PROGRESS;
    g_calibration.requested_samples = sample_count;
    g_calibration.attempted_samples = 0U;
    g_calibration.valid_samples = 0U;
    g_calibration.consecutive_failures = 0U;
    g_calibration.rejected_samples = 0U;
    g_calibration.sum_gx_dps = 0.0f;
    g_calibration.sum_gy_dps = 0.0f;
    g_calibration.sum_gz_dps = 0.0f;
    g_calibration.last_ax = 0.0f;
    g_calibration.last_ay = 0.0f;
    g_calibration.last_az = 0.0f;
    g_calibration.last_gx = 0.0f;
    g_calibration.last_gy = 0.0f;
    g_calibration.last_gz = 0.0f;
    g_calibration.min_gx = 32767.0f;
    g_calibration.min_gy = 32767.0f;
    g_calibration.min_gz = 32767.0f;
    g_calibration.max_gx = -32768.0f;
    g_calibration.max_gy = -32768.0f;
    g_calibration.max_gz = -32768.0f;

    g_last_status = ICM42688_STATUS_OK;

    return g_last_status;
}

ICM42688_CalibrationState_t
ICM42688_ProcessCalibration(void)
{
    ICM42688_RawData_t raw;

    if (g_calibration.state !=
        ICM42688_CALIBRATION_IN_PROGRESS)
    {
        return g_calibration.state;
    }

    /*
     * Exactly one raw burst is attempted per call.  There is intentionally no
     * delay here; the main-loop scheduler owns the 10 ms sampling cadence.
     */
    if (ReadRaw(&raw) != 0U)
    {
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
        float accel_norm2;
        float accel_scale2;
        float next_min_gx;
        float next_min_gy;
        float next_min_gz;
        float next_max_gx;
        float next_max_gy;
        float next_max_gz;
        uint8_t sample_is_stable = 1U;

        MapSensorToVehicle(
            &raw,
            &ax,
            &ay,
            &az,
            &gx,
            &gy,
            &gz);

        accel_norm2 = (ax * ax) + (ay * ay) + (az * az);
        accel_scale2 =
            ACCEL_SCALE_LSB_PER_G * ACCEL_SCALE_LSB_PER_G;

        if ((accel_norm2 < (ACCEL_NORM2_MIN * accel_scale2)) ||
            (accel_norm2 > (ACCEL_NORM2_MAX * accel_scale2)) ||
            (gx < -GYRO_CAL_MAX_ABS_RAW) ||
            (gx > GYRO_CAL_MAX_ABS_RAW) ||
            (gy < -GYRO_CAL_MAX_ABS_RAW) ||
            (gy > GYRO_CAL_MAX_ABS_RAW) ||
            (gz < -GYRO_CAL_MAX_ABS_RAW) ||
            (gz > GYRO_CAL_MAX_ABS_RAW))
        {
            sample_is_stable = 0U;
        }

        next_min_gx = (gx < g_calibration.min_gx) ? gx : g_calibration.min_gx;
        next_min_gy = (gy < g_calibration.min_gy) ? gy : g_calibration.min_gy;
        next_min_gz = (gz < g_calibration.min_gz) ? gz : g_calibration.min_gz;
        next_max_gx = (gx > g_calibration.max_gx) ? gx : g_calibration.max_gx;
        next_max_gy = (gy > g_calibration.max_gy) ? gy : g_calibration.max_gy;
        next_max_gz = (gz > g_calibration.max_gz) ? gz : g_calibration.max_gz;

        if (g_calibration.valid_samples >=
            CALIBRATION_MEAN_GUARD_START_SAMPLES)
        {
            float valid_count = (float)g_calibration.valid_samples;
            float mean_gx =
                (g_calibration.sum_gx_dps * GYRO_SCALE_LSB_PER_DPS) /
                valid_count;
            float mean_gy =
                (g_calibration.sum_gy_dps * GYRO_SCALE_LSB_PER_DPS) /
                valid_count;
            float mean_gz =
                (g_calibration.sum_gz_dps * GYRO_SCALE_LSB_PER_DPS) /
                valid_count;

            if ((gx < (mean_gx - GYRO_CAL_MAX_SPREAD_RAW)) ||
                (gx > (mean_gx + GYRO_CAL_MAX_SPREAD_RAW)) ||
                (gy < (mean_gy - GYRO_CAL_MAX_SPREAD_RAW)) ||
                (gy > (mean_gy + GYRO_CAL_MAX_SPREAD_RAW)) ||
                (gz < (mean_gz - GYRO_CAL_MAX_SPREAD_RAW)) ||
                (gz > (mean_gz + GYRO_CAL_MAX_SPREAD_RAW)))
            {
                sample_is_stable = 0U;
            }
        }

        g_calibration.consecutive_failures = 0U;

        if (sample_is_stable == 0U)
        {
            if (g_calibration.rejected_samples < UINT16_MAX)
            {
                g_calibration.rejected_samples++;
            }
        }
        else
        {

            g_calibration.sum_gx_dps +=
                gx / GYRO_SCALE_LSB_PER_DPS;

            g_calibration.sum_gy_dps +=
                gy / GYRO_SCALE_LSB_PER_DPS;

            g_calibration.sum_gz_dps +=
                gz / GYRO_SCALE_LSB_PER_DPS;

            g_calibration.last_ax = ax;
            g_calibration.last_ay = ay;
            g_calibration.last_az = az;
            g_calibration.last_gx = gx;
            g_calibration.last_gy = gy;
            g_calibration.last_gz = gz;

            g_calibration.min_gx = next_min_gx;
            g_calibration.min_gy = next_min_gy;
            g_calibration.min_gz = next_min_gz;
            g_calibration.max_gx = next_max_gx;
            g_calibration.max_gy = next_max_gy;
            g_calibration.max_gz = next_max_gz;

            g_calibration.valid_samples++;
        }
    }
    else
    {
        if (g_calibration.consecutive_failures < UINT16_MAX)
        {
            g_calibration.consecutive_failures++;
        }
    }

    if (g_calibration.attempted_samples < UINT16_MAX)
    {
        g_calibration.attempted_samples++;
    }

    if (g_calibration.consecutive_failures >=
        CALIBRATION_MAX_CONSECUTIVE_FAILURES)
    {
        g_ready = 0U;
        g_calibration.state = ICM42688_CALIBRATION_ERROR;
        g_last_status = ICM42688_STATUS_ERROR_DATA_READ;
        return g_calibration.state;
    }

    if (g_calibration.rejected_samples >
        CALIBRATION_MAX_REJECTED_SAMPLES)
    {
        g_ready = 0U;
        g_calibration.state = ICM42688_CALIBRATION_ERROR;
        g_last_status = ICM42688_STATUS_ERROR_CALIBRATION;
        return g_calibration.state;
    }

    if (g_calibration.valid_samples <
        g_calibration.requested_samples)
    {
        uint32_t max_attempts =
            (uint32_t)g_calibration.requested_samples +
            (uint32_t)CALIBRATION_MAX_REJECTED_SAMPLES +
            (uint32_t)CALIBRATION_MAX_CONSECUTIVE_FAILURES;

        if ((uint32_t)g_calibration.attempted_samples < max_attempts)
        {
            return g_calibration.state;
        }

        g_ready = 0U;
        g_calibration.state = ICM42688_CALIBRATION_ERROR;
        g_last_status = ICM42688_STATUS_ERROR_DATA_READ;
        return g_calibration.state;
    }

    g_gyro_bias.gx_dps =
        g_calibration.sum_gx_dps /
        (float)g_calibration.valid_samples;

    g_gyro_bias.gy_dps =
        g_calibration.sum_gy_dps /
        (float)g_calibration.valid_samples;

    g_gyro_bias.gz_dps =
        g_calibration.sum_gz_dps /
        (float)g_calibration.valid_samples;

    /*
     * Reuse the final valid calibration sample instead of issuing a second
     * I2C read in this call.  This preserves the one-read-per-step contract.
     */
    StoreScaledData(
        &g_data,
        g_calibration.last_ax,
        g_calibration.last_ay,
        g_calibration.last_az,
        g_calibration.last_gx,
        g_calibration.last_gy,
        g_calibration.last_gz);

    if (InitAttitudeFromAccel(
            g_data.accel_x_g,
            g_data.accel_y_g,
            g_data.accel_z_g) == 0U)
    {
        g_ready = 0U;
        g_calibration.state = ICM42688_CALIBRATION_ERROR;
        g_last_status = ICM42688_STATUS_ERROR_CALIBRATION;
        return g_calibration.state;
    }

    g_ready = 1U;
    g_calibration.state =
        ICM42688_CALIBRATION_COMPLETE;
    g_last_status = ICM42688_STATUS_OK;

    return g_calibration.state;
}

ICM42688_CalibrationState_t
ICM42688_GetCalibrationState(void)
{
    return g_calibration.state;
}

void ICM42688_GetCalibrationProgress(
    ICM42688_CalibrationProgress_t *progress)
{
    if (progress == 0)
    {
        return;
    }

    progress->state = g_calibration.state;
    progress->requested_samples =
        g_calibration.requested_samples;
    progress->attempted_samples =
        g_calibration.attempted_samples;
    progress->valid_samples =
        g_calibration.valid_samples;
    progress->rejected_samples =
        g_calibration.rejected_samples;

    if (g_calibration.requested_samples == 0U)
    {
        progress->percent_complete = 0U;
    }
    else
    {
        uint32_t percent =
            ((uint32_t)g_calibration.valid_samples * 100U) /
            (uint32_t)g_calibration.requested_samples;

        progress->percent_complete =
            (uint8_t)((percent > 100U) ? 100U : percent);
    }
}

ICM42688_Status_t ICM42688_CalibrateGyro(
    uint16_t sample_count)
{
    ICM42688_CalibrationState_t state;

    if (ICM42688_BeginCalibration(sample_count) !=
        ICM42688_STATUS_OK)
    {
        return g_last_status;
    }

    state = ICM42688_CALIBRATION_IN_PROGRESS;

    while (state ==
           ICM42688_CALIBRATION_IN_PROGRESS)
    {
        state = ICM42688_ProcessCalibration();

        if (state ==
            ICM42688_CALIBRATION_IN_PROGRESS)
        {
            StartupDelayMs(
                CALIBRATION_PERIOD_MS);
        }
    }

    return g_last_status;
}

uint8_t ICM42688_Update(float dt_seconds)
{
    if ((g_ready == 0U) ||
        (dt_seconds <= 0.0f) ||
        (dt_seconds > 0.100f))
    {
        if (g_ready == 0U)
        {
            g_last_status =
                ICM42688_STATUS_ERROR_NOT_READY;
        }
        else
        {
            g_last_status =
                ICM42688_STATUS_ERROR_ARGUMENT;
        }

        return 0U;
    }

    if (ReadScaledData(&g_data) == 0U)
    {
        g_last_status =
            ICM42688_STATUS_ERROR_DATA_READ;

        return 0U;
    }

    if (MahonyUpdate(dt_seconds) == 0U)
    {
        g_last_status = ICM42688_STATUS_ERROR_FIRST_READ;
        return 0U;
    }

    g_last_status = ICM42688_STATUS_OK;

    return 1U;
}

void ICM42688_GetData(ICM42688_Data_t *data)
{
    if (data != 0)
    {
        *data = g_data;
    }
}

void ICM42688_GetAttitude(
    ICM42688_Attitude_t *attitude)
{
    if (attitude != 0)
    {
        *attitude = g_attitude;
    }
}

void ICM42688_ResetYaw(void)
{
    g_zero_q0 = g_ahrs.q0;
    g_zero_q1 = g_ahrs.q1;
    g_zero_q2 = g_ahrs.q2;
    g_zero_q3 = g_ahrs.q3;
    g_zero_reference_set = 1U;
    g_output_initialized = 0U;
    g_filtered_yaw = 0.0f;
    g_attitude.yaw_deg = 0.0f;
}

uint8_t ICM42688_IsReady(void)
{
    return g_ready;
}

uint32_t ICM42688_GetI2CErrorCount(void)
{
    return g_i2c_error_count;
}

uint8_t ICM42688_GetI2CAddress(void)
{
    return g_i2c_address;
}

uint8_t ICM42688_GetLastWhoAmI(void)
{
    return g_last_who_am_i;
}

ICM42688_I2CPhase_t ICM42688_GetLastI2CPhase(void)
{
    return g_last_i2c_phase;
}

uint8_t ICM42688_GetLastI2CLineState(void)
{
    return g_last_i2c_line_state;
}

ICM42688_Status_t ICM42688_GetLastStatus(void)
{
    return g_last_status;
}
