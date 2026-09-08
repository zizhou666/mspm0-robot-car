/*
================================================================================
MSPM0G3507 小车系统主程序与任务调度模块
================================================================================
【功能简介】
本文件完成所有外设初始化，并以裸机非阻塞主循环调度编码器采样、速度 PID、
ICM42688、按键、OLED、蜂鸣器/RGB、Flash 参数保存、距离/巡线/角度控制和
物理通道测试。
中断只累计时基、控制节拍和编码器边沿，阻塞总线及整屏刷新均在主循环执行。

================================================================================
【函数定义】
- App_EnterCritical/App_ExitCritical：成对保存 PRIMASK 并进入/退出临界区。
- App_GetMillis/App_GetControlTicks：原子读取毫秒时基和待处理控制节拍。
- App_Elapsed：使用无符号回绕安全方式判断时间间隔。
- App_AccumulateMs：带饱和地累计诊断时间。
- App_CountDifference：计算累计编码器计数相对测试起点的差值。
- App_UpdateMotorIoTestDeltas：更新第六页左右编码器测试增量。
- App_ApplyMotorIoTestCommand：把 A+/A-/B+/B- 测试命令送入物理电机通道。
- App_FeedbackDirectionWrong：判断目标速度和反馈速度符号是否相反。
- App_UpdateButtonState：更新按键按下掩码供状态监视。
- App_ProcessControl：消费控制节拍，采样编码器并执行诊断、调试和速度 PID。
- App_InitializeImu：统一执行 ICM42688 初始化和校准启动，并更新应用状态。
- App_ProcessImu：非阻塞推进 ICM42688 校准、采样、重试和姿态更新。
- App_GetImuUiStatus：把 IMU 内部状态转换成 UI 模块状态。
- App_BuildUiModel：汇总状态、编码器、PID、姿态和调试数据供 OLED 使用。
- App_FillDefaultSettings：从 user_config.h 生成无 Flash 记录时的默认参数。
- App_ApplySettings：校验并应用目标值、四套 PID 和电机模式参数。
- App_ProcessUiSaveRequest：处理 UI 长按保存并反馈 Flash 结果。
- App_ProcessMotorDebugButton：处理第五页中心键短按启停。
- App_MotorIoTestIsBusy：判断第六页物理通道测试是否占用电机。
- App_AbortMotorIoTest：立即安全终止物理通道测试。
- App_BeginMotorIoTestSettling：关闭输出并进入编码器停稳阶段。
- App_StartMotorIoTest：记录起点并启动指定 A/B 正反向测试。
- App_ProcessMotorIoTest：处理第六页选择、计时、停机、增量和刷新。
- main：系统入口，初始化模块并运行非阻塞任务循环。
- SysTick_Handler：提供 1 ms 系统时基。
- CONTROL_TIMER_INST_IRQHandler：确认定时器零事件并累计控制节拍。
- GROUP1_IRQHandler：读取 GPIOB IIDX 并交给编码器中断分派函数。

================================================================================
【使用说明】
1. Keil 下载后上电从 main 进入；默认电机保持停止，中心键短按才启动调试。
2. 不要在三个中断入口中加入 OLED、Flash、软件 I2C 或延时函数。
3. 调整车辆参数只修改 user_config.h；引脚只修改 main.syscfg 后重新生成配置。
4. 主循环中的单外设失败不会锁死系统，其余模块和 OLED 状态页仍继续运行。
================================================================================
*/
#include "ti_msp_dl_config.h"

#include "app_config.h"
#include "app_status.h"
#include "battery_monitor.h"
#include "buttons.h"
#include "buzzer_rgb.h"
#include "encoder.h"
#include "icm42688.h"
#include "line_sensor.h"
#include "stm32_uart.h"
#include "motor.h"
#include "motor_debug.h"
#include "oled.h"
#include "settings_store.h"
#include "speed_pid.h"
#include "ui.h"
#include "wireless_test.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

volatile AppStatus g_appStatus;

static volatile uint32_t g_millis;
static volatile uint32_t g_controlTicks;

static EncoderSnapshot g_encoderSnapshot;
static SpeedPIDSnapshot g_pidSnapshot;
static ICM42688_Attitude_t g_attitude;
static MotorDebugSnapshot g_motorDebugSnapshot;
static BatteryMonitorSnapshot g_batterySnapshot;
static WirelessTestSnapshot g_wirelessSnapshot;
static uint32_t g_imuReadySinceMs;
static bool g_imuReadyTimerActive;

typedef struct {
    MotorIoTestCommand command;
    MotorIoTestPhase phase;
    uint32_t startedMs;
    uint32_t stoppedMs;
    uint32_t lastUiRefreshMs;
    int32_t startLeftCounts;
    int32_t startRightCounts;
    int32_t leftDeltaCounts;
    int32_t rightDeltaCounts;
} AppMotorIoTestState;

typedef enum {
    APP_AUTO_RUN_NONE = 0,
    APP_AUTO_RUN_TEST_SLOT,
    APP_AUTO_RUN_SQUARE,
    APP_AUTO_RUN_CIRCLE
} AppAutoRunKind;

typedef enum {
    APP_AUTO_PHASE_IDLE = 0,
    APP_AUTO_PHASE_DELAY,
    APP_AUTO_PHASE_RUNNING,
    APP_AUTO_PHASE_PAUSED,
    APP_AUTO_PHASE_DONE,
    APP_AUTO_PHASE_STOPPED,
    APP_AUTO_PHASE_FAULT,
    APP_AUTO_PHASE_EMPTY,
    APP_AUTO_PHASE_SAVE_FAILED
} AppAutoRunPhase;

typedef enum {
    APP_SQUARE_STEP_SIDE = 0,
    APP_SQUARE_STEP_TURN
} AppSquareStep;

typedef struct {
    AppAutoRunKind kind;
    AppAutoRunPhase phase;
    AppSquareStep squareStep;
    uint8_t testIndex;
    uint8_t squareSideIndex;
    uint32_t delayStartedMs;
    float speedCmS;
    float squareSideCm;
    float circleRadiusCm;
    float circleStartDistanceCm;
    float circleTargetDistanceCm;
    float squareTurnStartYawDeg;
    float circleProgressCm;
    float task1StartDistanceCm;
    float task1DistanceCm;
    float task1TargetDistanceCm;
    float task1CommandSpeedCmS;
    uint32_t task1RunStartedMs;
    uint32_t task1ElapsedMs;
    float task2StartDistanceCm;
    float task2DistanceCm;
    float task2CommandSpeedCmS;
    uint32_t task2RunStartedMs;
    uint32_t task2ElapsedMs;
    uint8_t task2TimingDone;
    uint8_t task2ExtensionActive;
    float task2ExtensionStartDistanceCm;
    uint32_t task2DecelStartedMs;
    uint8_t task2DecelActive;
    float task3StartDistanceCm;
    float task3DistanceCm;
    float task3CommandSpeedCmS;
    float task3MarkerDistanceCm;
    uint32_t task3RunStartedMs;
    uint32_t task3ElapsedMs;
    uint8_t task3TimingDone;
    uint8_t task3FinishConfirmTicks;
    uint8_t task3FinishSeen;
} AppAutoRunState;

typedef struct {
    UiWirelessActionState actionState;
    WirelessRemoteCommand command;
    uint16_t value;
    uint32_t lastCommandGeneration;
    float startDistanceCm;
    float lastYawDeg;
    float accumulatedTurnDeg;
    uint8_t running;
} AppWirelessRemoteState;

static AppMotorIoTestState g_motorIoTest;
static AppAutoRunState g_autoRun;
static AppWirelessRemoteState g_wirelessRemote;
static AppPersistentSettings g_currentSettings;

static uint32_t App_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void App_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

static uint32_t App_GetMillis(void)
{
    return g_millis;
}

static uint32_t App_GetControlTicks(void)
{
    uint32_t ticks;
    uint32_t primask = App_EnterCritical();
    ticks = g_controlTicks;
    App_ExitCritical(primask);
    return ticks;
}

static bool App_Elapsed(uint32_t now, uint32_t then, uint32_t interval)
{
    return ((uint32_t)(now - then) >= interval);
}

static uint32_t App_AccumulateMs(uint32_t accumulated, uint32_t increment)
{
    if ((UINT32_MAX - accumulated) < increment) {
        return UINT32_MAX;
    }
    return accumulated + increment;
		
}

static int32_t App_CountDifference(int32_t current, int32_t start)
{
    int64_t difference = (int64_t)current - (int64_t)start;

	
	
	
	
	
    if (difference > INT32_MAX) {
        return INT32_MAX;
    }
    if (difference < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)difference;
}

static float App_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float App_ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float App_SmoothStep01(float value)
{
    float x = App_ClampFloat(value, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

static int16_t App_RoundToInt16(float value)
{
    if (value >= 32767.0f) {
        return INT16_MAX;
    }
    if (value <= -32768.0f) {
        return INT16_MIN;
    }
    return (int16_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t App_RoundToInt32(double value)
{
    if (value >= 2147483647.0) {
        return INT32_MAX;
    }
    if (value <= -2147483648.0) {
        return INT32_MIN;
    }
    return (int32_t)(value + ((value >= 0.0) ? 0.5 : -0.5));
}

static void App_ProcessMaixCamPacket(uint32_t now)
{
    static uint32_t lastPacketMs;
    ICM42688_Data_t imuData;
    STM32UARTTelemetry telemetry;
    float targetSpeedCmS;

    if (!App_Elapsed(
            now, lastPacketMs, APP_MAIXCAM_PACKET_PERIOD_MS)) {
        return;
    }
    lastPacketMs = now;

    /*
     * Protocol frame uses +Y for vehicle forward and +X for vehicle left.
     * The IMU driver internally calls these vehicle X (forward) and vehicle Y
     * (left), so protocol AY comes from accel_x_g and ANGLE_X from pitch_deg.
     */
    ICM42688_GetData(&imuData);
    SpeedPID_GetSnapshot(&g_pidSnapshot);
    targetSpeedCmS = g_pidSnapshot.enabled ?
        0.5f * (g_pidSnapshot.leftTargetCmS +
                g_pidSnapshot.rightTargetCmS) : 0.0f;

    telemetry.accel_y_mg =
        App_RoundToInt16(imuData.accel_x_g * 1000.0f);
    telemetry.pitch_cdeg =
        App_RoundToInt16(g_attitude.pitch_deg * 100.0f);
    telemetry.left_speed_mm_s =
        App_RoundToInt16(g_encoderSnapshot.leftSpeedCmS * 10.0f);
    telemetry.right_speed_mm_s =
        App_RoundToInt16(g_encoderSnapshot.rightSpeedCmS * 10.0f);
    telemetry.target_speed_mm_s =
        App_RoundToInt16(targetSpeedCmS * 10.0f);
    telemetry.distance_mm = App_RoundToInt32(
        (double)g_encoderSnapshot.vehicleDistanceCm * 10.0);
    telemetry.phase = (uint8_t)g_autoRun.phase;

    (void)STM32_UART_TrySendTelemetry(&telemetry);
}

static float App_WrapAngleDeg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static void App_ClearImuReadyTimer(void)
{
    g_imuReadySinceMs = 0U;
    g_imuReadyTimerActive = false;
}

static void App_StartImuReadyTimer(uint32_t now)
{
    if (!g_imuReadyTimerActive) {
        g_imuReadySinceMs = now;
        g_imuReadyTimerActive = true;
    }
}

static UiRunStatus App_AutoRunUiStatus(AppAutoRunKind kind)
{
    if (g_autoRun.kind != kind) {
        return UI_RUN_STATUS_IDLE;
    }

    switch (g_autoRun.phase) {
        case APP_AUTO_PHASE_DELAY:
            return UI_RUN_STATUS_DELAY;
        case APP_AUTO_PHASE_RUNNING:
            return UI_RUN_STATUS_RUNNING;
        case APP_AUTO_PHASE_PAUSED:
            return UI_RUN_STATUS_PAUSED;
        case APP_AUTO_PHASE_DONE:
            return UI_RUN_STATUS_DONE;
        case APP_AUTO_PHASE_STOPPED:
            return UI_RUN_STATUS_STOPPED;
        case APP_AUTO_PHASE_FAULT:
        case APP_AUTO_PHASE_SAVE_FAILED:
            return UI_RUN_STATUS_FAILED;
        case APP_AUTO_PHASE_EMPTY:
            return UI_RUN_STATUS_EMPTY;
        case APP_AUTO_PHASE_IDLE:
        default:
            return UI_RUN_STATUS_IDLE;
    }
}

static bool App_AutoRunIsBusy(void)
{
    return (g_autoRun.phase == APP_AUTO_PHASE_DELAY) ||
        (g_autoRun.phase == APP_AUTO_PHASE_RUNNING) ||
        (g_autoRun.phase == APP_AUTO_PHASE_PAUSED);
}

static bool App_AutoRunOwnsControl(void)
{
    return App_AutoRunIsBusy();
}

static void App_UpdateMotorIoTestDeltas(void)
{
    g_motorIoTest.leftDeltaCounts = App_CountDifference(
        g_encoderSnapshot.leftPositionCounts,
        g_motorIoTest.startLeftCounts);
    g_motorIoTest.rightDeltaCounts = App_CountDifference(
        g_encoderSnapshot.rightPositionCounts,
        g_motorIoTest.startRightCounts);
}

static void App_ApplyMotorIoTestCommand(void)
{
    int16_t channelA = 0;
    int16_t channelB = 0;

    switch (g_motorIoTest.command) {
        case MOTOR_IO_TEST_COMMAND_A_POSITIVE:
            channelA = APP_MOTOR_IO_TEST_DUTY_COUNTS;
            break;
        case MOTOR_IO_TEST_COMMAND_A_NEGATIVE:
            channelA = -APP_MOTOR_IO_TEST_DUTY_COUNTS;
            break;
        case MOTOR_IO_TEST_COMMAND_B_POSITIVE:
            channelB = APP_MOTOR_IO_TEST_DUTY_COUNTS;
            break;
        case MOTOR_IO_TEST_COMMAND_B_NEGATIVE:
            channelB = -APP_MOTOR_IO_TEST_DUTY_COUNTS;
            break;
        case MOTOR_IO_TEST_COMMAND_OFF:
        default:
            break;
    }

    Motor_SetPhysicalOutputs(channelA, channelB);
}

static bool App_FeedbackDirectionWrong(float targetCmS, float measuredCmS,
                                       uint16_t dutyCounts)
{
    if ((dutyCounts < APP_ENCODER_SIGNAL_TEST_DUTY_COUNTS) ||
        ((targetCmS > -APP_ENCODER_DIRECTION_TEST_SPEED_CM_S) &&
         (targetCmS < APP_ENCODER_DIRECTION_TEST_SPEED_CM_S)) ||
        ((measuredCmS > -APP_ENCODER_DIRECTION_TEST_SPEED_CM_S) &&
         (measuredCmS < APP_ENCODER_DIRECTION_TEST_SPEED_CM_S))) {
        return false;
    }
    return ((targetCmS > 0.0f) != (measuredCmS > 0.0f));
}

static void App_UpdateButtonState(void)
{
    uint32_t pressed = 0U;

    for (uint32_t i = 0U; i < (uint32_t)BUTTON_ID_COUNT; ++i) {
        if (Buttons_IsPressed((ButtonId)i)) {
            pressed |= (1UL << i);
        }
    }

    g_appStatus.buttonPressedMask = pressed;
}

static float App_SquareTurnSpeed(void)
{
    return App_ClampFloat(
        App_AbsFloat(g_autoRun.speedCmS) * APP_SQUARE_TURN_SPEED_RATIO,
        APP_SQUARE_TURN_SPEED_MIN_CM_S,
        APP_SQUARE_TURN_SPEED_MAX_CM_S);
}

static bool App_IsTaskOne(void)
{
    return (g_autoRun.kind == APP_AUTO_RUN_TEST_SLOT) &&
        (g_autoRun.testIndex == 0U);
}

static bool App_IsTaskTwo(void)
{
    return (g_autoRun.kind == APP_AUTO_RUN_TEST_SLOT) &&
        (g_autoRun.testIndex == 1U);
}

static bool App_IsTaskThree(void)
{
    return (g_autoRun.kind == APP_AUTO_RUN_TEST_SLOT) &&
        (g_autoRun.testIndex == 2U);
}

static uint32_t App_TaskOneElapsedMs(uint32_t now)
{
    if (App_IsTaskOne() &&
        (g_autoRun.phase == APP_AUTO_PHASE_RUNNING)) {
        return App_AccumulateMs(
            g_autoRun.task1ElapsedMs,
            (uint32_t)(now - g_autoRun.task1RunStartedMs));
    }
    return g_autoRun.task1ElapsedMs;
}

static void App_FreezeTaskOneTimer(uint32_t now)
{
    if (App_IsTaskOne() &&
        (g_autoRun.phase == APP_AUTO_PHASE_RUNNING)) {
        g_autoRun.task1ElapsedMs = App_TaskOneElapsedMs(now);
    }
}

static uint32_t App_TaskTwoElapsedMs(uint32_t now)
{
    if (App_IsTaskTwo() &&
        (g_autoRun.phase == APP_AUTO_PHASE_RUNNING) &&
        (g_autoRun.task2TimingDone == 0U)) {
        return App_AccumulateMs(
            g_autoRun.task2ElapsedMs,
            (uint32_t)(now - g_autoRun.task2RunStartedMs));
    }
    return g_autoRun.task2ElapsedMs;
}

static void App_FreezeTaskTwoTimer(uint32_t now)
{
    if (App_IsTaskTwo() &&
        (g_autoRun.phase == APP_AUTO_PHASE_RUNNING) &&
        (g_autoRun.task2TimingDone == 0U)) {
        g_autoRun.task2ElapsedMs = App_TaskTwoElapsedMs(now);
    }
}

static uint32_t App_TaskThreeElapsedMs(uint32_t now)
{
    if (App_IsTaskThree() &&
        (g_autoRun.phase == APP_AUTO_PHASE_RUNNING) &&
        (g_autoRun.task3TimingDone == 0U)) {
        return App_AccumulateMs(
            g_autoRun.task3ElapsedMs,
            (uint32_t)(now - g_autoRun.task3RunStartedMs));
    }
    return g_autoRun.task3ElapsedMs;
}

static void App_FreezeTaskThreeTimer(uint32_t now)
{
    if (App_IsTaskThree() &&
        (g_autoRun.phase == APP_AUTO_PHASE_RUNNING) &&
        (g_autoRun.task3TimingDone == 0U)) {
        g_autoRun.task3ElapsedMs = App_TaskThreeElapsedMs(now);
    }
}

static void App_StopAutoRun(AppAutoRunPhase phase)
{
    uint32_t now = App_GetMillis();

    App_FreezeTaskOneTimer(now);
    App_FreezeTaskTwoTimer(now);
    App_FreezeTaskThreeTimer(now);
    SpeedPID_Enable(false);
    MotorDebug_Stop();
    if (App_IsTaskOne()) {
        g_autoRun.task1CommandSpeedCmS = 0.0f;
    } else if (App_IsTaskTwo()) {
        g_autoRun.task2CommandSpeedCmS = 0.0f;
    } else if (App_IsTaskThree()) {
        g_autoRun.task3CommandSpeedCmS = 0.0f;
    }
    g_autoRun.phase = phase;
    UI_RequestRefresh();
}

static bool App_AutoRunSensorsReady(bool needs_heading)
{
    if ((g_appStatus.motorInitialized == 0U) ||
        (g_appStatus.encoderConfigured == 0U) ||
        (g_appStatus.encoderSignalMissing != 0U) ||
        (g_appStatus.encoderDirectionMismatch != 0U)) {
        return false;
    }
    if (needs_heading &&
        ((g_appStatus.imuReady == 0U) ||
         (g_appStatus.imuFailed != 0U) ||
         (ICM42688_IsReady() == 0U))) {
        return false;
    }
    return true;
}

static void App_StopWirelessRemote(UiWirelessActionState state)
{
    SpeedPID_Enable(false);
    MotorDebug_Stop();
    Motor_Stop();
    g_wirelessRemote.running = 0U;
    g_wirelessRemote.actionState = state;
    UI_RequestRefresh();
}

static void App_StartWirelessRemote(
    WirelessRemoteCommand command, uint16_t value)
{
    bool needsHeading =
        (command == WIRELESS_COMMAND_TURN_LEFT) ||
        (command == WIRELESS_COMMAND_TURN_RIGHT);

    if (command == WIRELESS_COMMAND_STOP)
    {
        g_wirelessRemote.command = command;
        g_wirelessRemote.value = 0U;
        App_StopWirelessRemote(UI_WIRELESS_ACTION_STOPPED);
        return;
    }
    if ((command < WIRELESS_COMMAND_FORWARD) ||
        (command > WIRELESS_COMMAND_TURN_RIGHT) ||
        (value == 0U) ||
        (!App_AutoRunSensorsReady(needsHeading)))
    {
        g_wirelessRemote.command = command;
        g_wirelessRemote.value = value;
        App_StopWirelessRemote(UI_WIRELESS_ACTION_SENSOR_FAILED);
        return;
    }

    if (App_AutoRunIsBusy())
    {
        App_StopAutoRun(APP_AUTO_PHASE_STOPPED);
    }
    MotorDebug_Stop();
    g_wirelessRemote.command = command;
    g_wirelessRemote.value = value;
    g_wirelessRemote.startDistanceCm =
        g_encoderSnapshot.vehicleDistanceCm;
    g_wirelessRemote.lastYawDeg = g_attitude.yaw_deg;
    g_wirelessRemote.accumulatedTurnDeg = 0.0f;
    g_wirelessRemote.running = 1U;
    g_wirelessRemote.actionState = UI_WIRELESS_ACTION_RUNNING;

    if (command == WIRELESS_COMMAND_FORWARD)
    {
        SpeedPID_SetTargets(APP_NRF24_REMOTE_LINEAR_SPEED_CM_S,
                            APP_NRF24_REMOTE_LINEAR_SPEED_CM_S);
    }
    else if (command == WIRELESS_COMMAND_BACKWARD)
    {
        SpeedPID_SetTargets(-APP_NRF24_REMOTE_LINEAR_SPEED_CM_S,
                            -APP_NRF24_REMOTE_LINEAR_SPEED_CM_S);
    }
    else if (command == WIRELESS_COMMAND_TURN_LEFT)
    {
        SpeedPID_SetTargets(-APP_NRF24_REMOTE_TURN_SPEED_CM_S,
                             APP_NRF24_REMOTE_TURN_SPEED_CM_S);
    }
    else
    {
        SpeedPID_SetTargets( APP_NRF24_REMOTE_TURN_SPEED_CM_S,
                            -APP_NRF24_REMOTE_TURN_SPEED_CM_S);
    }
    SpeedPID_Enable(true);
    UI_RequestRefresh();
}

static bool App_ProcessWirelessRemoteControl10ms(void)
{
    float progress;
    float target;

    if (g_wirelessRemote.running == 0U)
    {
        return false;
    }
    if ((g_wirelessSnapshot.sessionActive == 0U) ||
        (g_wirelessSnapshot.activeMode != WIRELESS_TEST_MODE_RX) ||
        (g_wirelessSnapshot.linkStatus != WIRELESS_LINK_OK))
    {
        App_StopWirelessRemote(UI_WIRELESS_ACTION_LINK_LOST);
        return true;
    }

    if ((g_wirelessRemote.command == WIRELESS_COMMAND_FORWARD) ||
        (g_wirelessRemote.command == WIRELESS_COMMAND_BACKWARD))
    {
        if (!App_AutoRunSensorsReady(false))
        {
            App_StopWirelessRemote(UI_WIRELESS_ACTION_SENSOR_FAILED);
            return true;
        }
        progress = App_AbsFloat(g_encoderSnapshot.vehicleDistanceCm -
                                g_wirelessRemote.startDistanceCm);
        target = (float)g_wirelessRemote.value -
                 APP_NRF24_REMOTE_DISTANCE_TOLERANCE_CM;
    }
    else
    {
        float yawStep;

        if (!App_AutoRunSensorsReady(true))
        {
            App_StopWirelessRemote(UI_WIRELESS_ACTION_SENSOR_FAILED);
            return true;
        }
        yawStep = App_WrapAngleDeg(
            g_attitude.yaw_deg - g_wirelessRemote.lastYawDeg);
        g_wirelessRemote.lastYawDeg = g_attitude.yaw_deg;
        g_wirelessRemote.accumulatedTurnDeg += App_AbsFloat(yawStep);
        progress = g_wirelessRemote.accumulatedTurnDeg;
        target = (float)g_wirelessRemote.value -
                 APP_NRF24_REMOTE_ANGLE_TOLERANCE_DEG;
    }

    if (target < 0.0f)
    {
        target = 0.0f;
    }
    if (progress >= target)
    {
        App_StopWirelessRemote(UI_WIRELESS_ACTION_DONE);
        return true;
    }

    if (g_wirelessRemote.command == WIRELESS_COMMAND_FORWARD)
    {
        SpeedPID_SetTargets(APP_NRF24_REMOTE_LINEAR_SPEED_CM_S,
                            APP_NRF24_REMOTE_LINEAR_SPEED_CM_S);
    }
    else if (g_wirelessRemote.command == WIRELESS_COMMAND_BACKWARD)
    {
        SpeedPID_SetTargets(-APP_NRF24_REMOTE_LINEAR_SPEED_CM_S,
                            -APP_NRF24_REMOTE_LINEAR_SPEED_CM_S);
    }
    else if (g_wirelessRemote.command == WIRELESS_COMMAND_TURN_LEFT)
    {
        SpeedPID_SetTargets(-APP_NRF24_REMOTE_TURN_SPEED_CM_S,
                             APP_NRF24_REMOTE_TURN_SPEED_CM_S);
    }
    else
    {
        SpeedPID_SetTargets( APP_NRF24_REMOTE_TURN_SPEED_CM_S,
                            -APP_NRF24_REMOTE_TURN_SPEED_CM_S);
    }
    SpeedPID_Enable(true);
    return true;
}

static bool App_StartSquareSide(void)
{
    bool headingReady =
        (g_appStatus.imuReady != 0U) &&
        (g_appStatus.imuFailed == 0U) &&
        (ICM42688_IsReady() != 0U);

    if (!MotorDebug_SetConfiguration(MOTOR_DEBUG_MODE_DISTANCE,
            g_autoRun.squareSideCm, g_autoRun.speedCmS)) {
        return false;
    }
    if (!MotorDebug_Start(&g_encoderSnapshot,
            (g_appStatus.encoderConfigured != 0U) &&
            (g_appStatus.encoderSignalMissing == 0U) &&
            (g_appStatus.encoderDirectionMismatch == 0U),
            g_attitude.yaw_deg, headingReady)) {
        return false;
    }
    g_autoRun.squareStep = APP_SQUARE_STEP_SIDE;
    return true;
}

static bool App_StartCircleRun(void)
{
    float halfTrack = 0.5f * APP_CHASSIS_TRACK_WIDTH_CM;
    float leftTarget;
    float rightTarget;

    if ((g_autoRun.circleRadiusCm <= halfTrack) ||
        (!isfinite(g_autoRun.circleRadiusCm)) ||
        (!isfinite(g_autoRun.speedCmS))) {
        return false;
    }

    leftTarget = g_autoRun.speedCmS *
        (g_autoRun.circleRadiusCm - halfTrack) / g_autoRun.circleRadiusCm;
    rightTarget = g_autoRun.speedCmS *
        (g_autoRun.circleRadiusCm + halfTrack) / g_autoRun.circleRadiusCm;
    if ((leftTarget < APP_SPEED_PID_TARGET_STOP_BAND_CM_S) ||
        (rightTarget > APP_TARGET_SPEED_MAX_CM_S)) {
        return false;
    }

    g_autoRun.circleStartDistanceCm =
        g_encoderSnapshot.vehicleDistanceCm;
    g_autoRun.circleTargetDistanceCm =
        2.0f * APP_PI_F * g_autoRun.circleRadiusCm;
    g_autoRun.circleProgressCm = 0.0f;
    SpeedPID_SetTargets(leftTarget, rightTarget);
    SpeedPID_Enable(true);
    return true;
}

static bool App_StartTaskOneLine(uint32_t now, bool reset_run)
{
    if ((!App_AutoRunSensorsReady(false)) ||
        (g_appStatus.lineSensorsInitialized == 0U)) {
        return false;
    }

    if ((g_autoRun.task1TargetDistanceCm <
         APP_TASK1_STOP_DISTANCE_MIN_CM) ||
        (g_autoRun.task1TargetDistanceCm >
         APP_TASK1_STOP_DISTANCE_MAX_CM)) {
        g_autoRun.task1TargetDistanceCm =
            APP_TASK1_STOP_DISTANCE_DEFAULT_CM;
    }

    if (!MotorDebug_SetConfiguration(
            MOTOR_DEBUG_MODE_LINE_FOLLOW,
            APP_TASK1_LINE_SAFETY_DISTANCE_CM,
            APP_TASK1_LAP_SPEED_CM_S)) {
        return false;
    }

    if (reset_run) {
        g_autoRun.task1StartDistanceCm =
            g_encoderSnapshot.vehicleDistanceCm;
        g_autoRun.task1DistanceCm = 0.0f;
        g_autoRun.task1ElapsedMs = 0U;
    }
    g_autoRun.task1CommandSpeedCmS = APP_TASK1_LAP_SPEED_CM_S;

    if (!MotorDebug_Start(
            &g_encoderSnapshot, true, 0.0f, false)) {
        return false;
    }
    if (!MotorDebug_SetBaseSpeed(APP_TASK1_LAP_SPEED_CM_S)) {
        MotorDebug_Stop();
        return false;
    }

    g_autoRun.task1RunStartedMs = now;
    g_autoRun.phase = APP_AUTO_PHASE_RUNNING;
    return true;
}

static bool App_StartTaskTwoLine(uint32_t now)
{
    if ((!App_AutoRunSensorsReady(false)) ||
        (g_appStatus.lineSensorsInitialized == 0U)) {
        return false;
    }

    if (!MotorDebug_SetConfiguration(
            MOTOR_DEBUG_MODE_LINE_FOLLOW,
            APP_TASK2_TIMED_DISTANCE_CM,
            APP_TASK2_CRUISE_SPEED_CM_S)) {
        return false;
    }

    g_autoRun.task2StartDistanceCm =
        g_encoderSnapshot.vehicleDistanceCm;
    g_autoRun.task2DistanceCm = 0.0f;
    g_autoRun.task2CommandSpeedCmS = APP_TASK2_START_SPEED_CM_S;
    g_autoRun.task2ElapsedMs = 0U;
    g_autoRun.task2TimingDone = 0U;
    g_autoRun.task2ExtensionActive = 0U;
    g_autoRun.task2ExtensionStartDistanceCm = 0.0f;
    g_autoRun.task2DecelStartedMs = 0U;
    g_autoRun.task2DecelActive = 0U;

    if (!MotorDebug_Start(
            &g_encoderSnapshot, true, 0.0f, false)) {
        return false;
    }
    if (!MotorDebug_SetBaseSpeed(APP_TASK2_START_SPEED_CM_S)) {
        MotorDebug_Stop();
        return false;
    }

    g_autoRun.task2RunStartedMs = now;
    g_autoRun.phase = APP_AUTO_PHASE_RUNNING;
    Buzzer_Pulse(now, APP_TASK_CUE_BEEP_DURATION_MS);
    return true;
}

static bool App_StartTaskTwoEncoderExtension(void)
{
    if (!MotorDebug_TransitionToDistance(
            &g_encoderSnapshot,
            APP_TASK2_EXTENSION_DISTANCE_CM,
            APP_TASK2_CRUISE_SPEED_CM_S,
            0.0f,
            false)) {
        return false;
    }
    g_autoRun.task2ExtensionStartDistanceCm =
        g_encoderSnapshot.vehicleDistanceCm;
    g_autoRun.task2DecelStartedMs = 0U;
    g_autoRun.task2DecelActive = 0U;
    g_autoRun.task2ExtensionActive = 1U;
    return true;
}

static float App_TaskTwoSpeedCommand(uint32_t now)
{
    if (g_autoRun.task2ExtensionActive == 0U) {
        float rampRatio = (float)(uint32_t)(
            now - g_autoRun.task2RunStartedMs) /
            (float)APP_TASK2_START_RAMP_MS;

        return APP_TASK2_CRUISE_SPEED_CM_S *
            App_ClampFloat(rampRatio, 0.0f, 1.0f);
    } else {
        float extensionDistance = App_AbsFloat(
            g_encoderSnapshot.vehicleDistanceCm -
            g_autoRun.task2ExtensionStartDistanceCm);
        float remaining = APP_TASK2_EXTENSION_DISTANCE_CM -
            extensionDistance;

        if (remaining <= APP_TASK2_DECEL_DISTANCE_CM) {
            float decelRatio;

            if (g_autoRun.task2DecelActive == 0U) {
                g_autoRun.task2DecelActive = 1U;
                g_autoRun.task2DecelStartedMs = now;
            }
            decelRatio = (float)(uint32_t)(
                now - g_autoRun.task2DecelStartedMs) /
                (float)APP_TASK2_STOP_RAMP_MS;
            return App_ClampFloat(
                APP_TASK2_END_SPEED_CM_S +
                    (APP_TASK2_CRUISE_SPEED_CM_S -
                     APP_TASK2_END_SPEED_CM_S) *
                    (1.0f - App_ClampFloat(decelRatio, 0.0f, 1.0f)),
                APP_TASK2_FINAL_CRAWL_SPEED_CM_S,
                APP_TASK2_CRUISE_SPEED_CM_S);
        }
    }
    return APP_TASK2_CRUISE_SPEED_CM_S;
}

static bool App_StartTaskThreeLap(uint32_t now)
{
    if ((!App_AutoRunSensorsReady(false)) ||
        (g_appStatus.lineSensorsInitialized == 0U)) {
        return false;
    }

    if (!MotorDebug_SetConfiguration(
            MOTOR_DEBUG_MODE_LINE_FOLLOW,
            APP_TASK3_SAFETY_DISTANCE_CM,
            APP_TASK3_CRUISE_SPEED_CM_S)) {
        return false;
    }

    g_autoRun.task3StartDistanceCm =
        g_encoderSnapshot.vehicleDistanceCm;
    g_autoRun.task3DistanceCm = 0.0f;
    g_autoRun.task3CommandSpeedCmS = 0.0f;
    g_autoRun.task3MarkerDistanceCm = 0.0f;
    g_autoRun.task3ElapsedMs = 0U;
    g_autoRun.task3TimingDone = 0U;
    g_autoRun.task3FinishConfirmTicks = 0U;
    g_autoRun.task3FinishSeen = 0U;

    if (!MotorDebug_Start(
            &g_encoderSnapshot, true, 0.0f, false)) {
        return false;
    }
    if (!MotorDebug_SetBaseSpeed(0.0f)) {
        MotorDebug_Stop();
        return false;
    }

    g_autoRun.task3RunStartedMs = now;
    g_autoRun.phase = APP_AUTO_PHASE_RUNNING;
    Buzzer_Pulse(now, APP_TASK_CUE_BEEP_DURATION_MS);
    return true;
}

static float App_TaskThreeSpeedCommand(uint32_t now)
{
    float rampRatio = (float)(uint32_t)(
        now - g_autoRun.task3RunStartedMs) /
        (float)APP_TASK3_START_RAMP_MS;
    float command = APP_TASK3_START_SPEED_CM_S +
        (APP_TASK3_CRUISE_SPEED_CM_S - APP_TASK3_START_SPEED_CM_S) *
        App_SmoothStep01(rampRatio);

    if (g_autoRun.task3FinishSeen != 0U) {
        float stopProgress = g_autoRun.task3DistanceCm -
            g_autoRun.task3MarkerDistanceCm;
        float stopRemaining = APP_TASK3_AFTER_MARKER_STOP_CM - stopProgress;
        float stopRatio = stopRemaining / APP_TASK3_AFTER_MARKER_STOP_CM;
        float stopSpeed = APP_TASK3_FINAL_CRAWL_SPEED_CM_S +
            (APP_TASK3_APPROACH_SPEED_CM_S -
             APP_TASK3_FINAL_CRAWL_SPEED_CM_S) *
            App_SmoothStep01(stopRatio);

        if (stopSpeed < command) {
            command = stopSpeed;
        }
    } else if (g_autoRun.task3DistanceCm >
               APP_TASK3_APPROACH_START_CM) {
        float approachRatio =
            (APP_TASK3_EXPECTED_FINISH_CM - g_autoRun.task3DistanceCm) /
            (APP_TASK3_EXPECTED_FINISH_CM -
             APP_TASK3_APPROACH_START_CM);
        float approachSpeed = APP_TASK3_APPROACH_SPEED_CM_S +
            (APP_TASK3_CRUISE_SPEED_CM_S -
             APP_TASK3_APPROACH_SPEED_CM_S) *
            App_SmoothStep01(approachRatio);

        if (approachSpeed < command) {
            command = approachSpeed;
        }
    }

    return App_ClampFloat(command,
        APP_TASK3_FINAL_CRAWL_SPEED_CM_S,
        APP_TASK3_CRUISE_SPEED_CM_S);
}

static bool App_StartAutoRunNow(void)
{
    if (g_autoRun.kind == APP_AUTO_RUN_TEST_SLOT) {
        if (g_autoRun.testIndex == 0U) {
            return App_StartTaskOneLine(App_GetMillis(), true);
        }
        if (g_autoRun.testIndex == 1U) {
            return App_StartTaskTwoLine(App_GetMillis());
        }
        if (g_autoRun.testIndex == 2U) {
            return App_StartTaskThreeLap(App_GetMillis());
        }
        return false;
    }
    if (g_autoRun.kind == APP_AUTO_RUN_SQUARE) {
        if (!App_AutoRunSensorsReady(true)) {
            return false;
        }
        g_autoRun.squareSideIndex = 0U;
        return App_StartSquareSide();
    }
    if (g_autoRun.kind == APP_AUTO_RUN_CIRCLE) {
        if (!App_AutoRunSensorsReady(false)) {
            return false;
        }
        return App_StartCircleRun();
    }
    return false;
}

static bool App_ProcessAutoRunControl10ms(uint32_t now)
{
    if (!App_AutoRunOwnsControl()) {
        return false;
    }

    if (g_autoRun.phase == APP_AUTO_PHASE_DELAY) {
        SpeedPID_Enable(false);
        if (App_Elapsed(now, g_autoRun.delayStartedMs,
                        APP_TEST_START_DELAY_MS)) {
            if (App_StartAutoRunNow()) {
                g_autoRun.phase = APP_AUTO_PHASE_RUNNING;
            } else {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            }
        }
        return true;
    }

    if (g_autoRun.kind == APP_AUTO_RUN_TEST_SLOT) {
        if (g_autoRun.testIndex == 2U) {
            if ((!App_AutoRunSensorsReady(false)) ||
                (g_appStatus.lineSensorsInitialized == 0U)) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                return true;
            }

            g_autoRun.task3DistanceCm = App_AbsFloat(
                g_encoderSnapshot.vehicleDistanceCm -
                g_autoRun.task3StartDistanceCm);
            g_autoRun.task3CommandSpeedCmS =
                App_TaskThreeSpeedCommand(now);
            if (!MotorDebug_SetBaseSpeed(
                    g_autoRun.task3CommandSpeedCmS)) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                return true;
            }

            MotorDebug_Update10ms(
                &g_encoderSnapshot, 0.0f, false);
            MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
            if (g_motorDebugSnapshot.state == MOTOR_DEBUG_STATE_FAULT) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                return true;
            }
            if (g_motorDebugSnapshot.state !=
                MOTOR_DEBUG_STATE_RUNNING) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                return true;
            }

            if (g_autoRun.task3FinishSeen != 0U) {
                if ((g_autoRun.task3DistanceCm -
                     g_autoRun.task3MarkerDistanceCm) >=
                    APP_TASK3_AFTER_MARKER_STOP_CM) {
                    App_StopAutoRun(APP_AUTO_PHASE_DONE);
                }
            } else if (g_autoRun.task3DistanceCm >=
                       APP_TASK3_FINISH_GATE_CM) {
                if (g_motorDebugSnapshot.line_active_count >=
                    APP_TASK3_FINISH_ACTIVE_COUNT) {
                    if (g_autoRun.task3FinishConfirmTicks < UINT8_MAX) {
                        ++g_autoRun.task3FinishConfirmTicks;
                    }
                } else {
                    g_autoRun.task3FinishConfirmTicks = 0U;
                }

                if (g_autoRun.task3FinishConfirmTicks >=
                    APP_TASK3_FINISH_CONFIRM_TICKS) {
                    g_autoRun.task3ElapsedMs =
                        App_TaskThreeElapsedMs(now);
                    g_autoRun.task3TimingDone = 1U;
                    g_autoRun.task3FinishSeen = 1U;
                    g_autoRun.task3MarkerDistanceCm =
                        g_autoRun.task3DistanceCm;
                    Buzzer_Pulse(now, APP_TASK_CUE_BEEP_DURATION_MS);
                    UI_RequestRefresh();
                }
            }
            return true;
        }

        if (g_autoRun.testIndex == 1U) {
            if ((!App_AutoRunSensorsReady(false)) ||
                ((g_autoRun.task2ExtensionActive == 0U) &&
                 (g_appStatus.lineSensorsInitialized == 0U))) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                return true;
            }

            g_autoRun.task2DistanceCm = App_AbsFloat(
                g_encoderSnapshot.vehicleDistanceCm -
                g_autoRun.task2StartDistanceCm);
            if ((g_autoRun.task2TimingDone == 0U) &&
                (g_autoRun.task2DistanceCm >=
                 APP_TASK2_TIMED_DISTANCE_CM)) {
                g_autoRun.task2ElapsedMs = App_TaskTwoElapsedMs(now);
                g_autoRun.task2TimingDone = 1U;
                Buzzer_Pulse(now, APP_TASK_CUE_BEEP_DURATION_MS);
                UI_RequestRefresh();
            }
            if ((g_autoRun.task2ExtensionActive == 0U) &&
                (g_autoRun.task2DistanceCm >=
                 APP_TASK2_TIMED_DISTANCE_CM)) {
                if (!App_StartTaskTwoEncoderExtension()) {
                    App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                    return true;
                }
            }
            g_autoRun.task2CommandSpeedCmS =
                App_TaskTwoSpeedCommand(now);
            if (!MotorDebug_SetBaseSpeed(
                    g_autoRun.task2CommandSpeedCmS)) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                return true;
            }

            MotorDebug_Update10ms(
                &g_encoderSnapshot, 0.0f, false);
            MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
            if (g_motorDebugSnapshot.state == MOTOR_DEBUG_STATE_FAULT) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            } else if (g_motorDebugSnapshot.state ==
                       MOTOR_DEBUG_STATE_DONE) {
                if (g_autoRun.task2ExtensionActive != 0U) {
                    App_StopAutoRun(APP_AUTO_PHASE_DONE);
                } else {
                    App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                }
            } else if (g_motorDebugSnapshot.state !=
                       MOTOR_DEBUG_STATE_RUNNING) {
                App_StopAutoRun(APP_AUTO_PHASE_STOPPED);
            }
            return true;
        }

        if (g_autoRun.testIndex != 0U) {
            App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            return true;
        }

        if (g_autoRun.phase == APP_AUTO_PHASE_PAUSED) {
            SpeedPID_Enable(false);
            return true;
        }

        if ((!App_AutoRunSensorsReady(false)) ||
            (g_appStatus.lineSensorsInitialized == 0U)) {
            App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            return true;
        }

        g_autoRun.task1DistanceCm = App_AbsFloat(
            g_encoderSnapshot.vehicleDistanceCm -
            g_autoRun.task1StartDistanceCm);

        g_autoRun.task1CommandSpeedCmS = APP_TASK1_LAP_SPEED_CM_S;
        if (!MotorDebug_SetBaseSpeed(
                g_autoRun.task1CommandSpeedCmS)) {
            App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            return true;
        }

        MotorDebug_Update10ms(
            &g_encoderSnapshot, 0.0f, false);
        MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
        if (g_motorDebugSnapshot.state == MOTOR_DEBUG_STATE_FAULT) {
            App_StopAutoRun(APP_AUTO_PHASE_FAULT);
        } else if (g_motorDebugSnapshot.state !=
                   MOTOR_DEBUG_STATE_RUNNING) {
            App_StopAutoRun(APP_AUTO_PHASE_STOPPED);
        } else if ((g_autoRun.task1DistanceCm >=
                    g_autoRun.task1TargetDistanceCm) &&
                   (g_motorDebugSnapshot.line_active_count >=
                    APP_TASK1_FINISH_ACTIVE_COUNT)) {
            App_StopAutoRun(APP_AUTO_PHASE_DONE);
        }
        return true;
    }

    if (g_autoRun.kind == APP_AUTO_RUN_SQUARE) {
        if (g_autoRun.squareStep == APP_SQUARE_STEP_SIDE) {
            MotorDebug_Update10ms(&g_encoderSnapshot, g_attitude.yaw_deg,
                (g_appStatus.imuReady != 0U) &&
                (g_appStatus.imuFailed == 0U));
            MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
            if (g_motorDebugSnapshot.state == MOTOR_DEBUG_STATE_FAULT) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            } else if (g_motorDebugSnapshot.state ==
                       MOTOR_DEBUG_STATE_DONE) {
                if (g_autoRun.squareSideIndex >= 3U) {
                    App_StopAutoRun(APP_AUTO_PHASE_DONE);
                } else if (!App_AutoRunSensorsReady(true)) {
                    App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                } else {
                    float turnSpeed = App_SquareTurnSpeed();
                    g_autoRun.squareStep = APP_SQUARE_STEP_TURN;
                    g_autoRun.squareTurnStartYawDeg = g_attitude.yaw_deg;
                    SpeedPID_SetTargets(-turnSpeed, turnSpeed);
                    SpeedPID_Enable(true);
                    UI_RequestRefresh();
                }
            }
        } else {
            float turnDeg = App_AbsFloat(App_WrapAngleDeg(
                g_attitude.yaw_deg - g_autoRun.squareTurnStartYawDeg));
            float turnSpeed = App_SquareTurnSpeed();

            if (!App_AutoRunSensorsReady(true)) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            } else if (turnDeg >=
                (APP_SQUARE_TURN_DEG - APP_SQUARE_TURN_TOLERANCE_DEG)) {
                SpeedPID_Enable(false);
                ++g_autoRun.squareSideIndex;
                if (!App_StartSquareSide()) {
                    App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                }
                UI_RequestRefresh();
            } else {
                SpeedPID_SetTargets(-turnSpeed, turnSpeed);
                SpeedPID_Enable(true);
            }
        }
        return true;
    }

    if (g_autoRun.kind == APP_AUTO_RUN_CIRCLE) {
        float halfTrack = 0.5f * APP_CHASSIS_TRACK_WIDTH_CM;
        float leftTarget = g_autoRun.speedCmS *
            (g_autoRun.circleRadiusCm - halfTrack) /
            g_autoRun.circleRadiusCm;
        float rightTarget = g_autoRun.speedCmS *
            (g_autoRun.circleRadiusCm + halfTrack) /
            g_autoRun.circleRadiusCm;

        g_autoRun.circleProgressCm = App_AbsFloat(
            g_encoderSnapshot.vehicleDistanceCm -
            g_autoRun.circleStartDistanceCm);
        if (g_autoRun.circleProgressCm >=
            g_autoRun.circleTargetDistanceCm) {
            App_StopAutoRun(APP_AUTO_PHASE_DONE);
        } else {
            SpeedPID_SetTargets(leftTarget, rightTarget);
            SpeedPID_Enable(true);
        }
        return true;
    }

    return true;
}

static void App_ProcessControl(void)
{
    static uint32_t lastControlTicks;
    static uint32_t leftNoSignalMs;
    static uint32_t rightNoSignalMs;
    static uint32_t leftWrongDirectionMs;
    static uint32_t rightWrongDirectionMs;
    uint32_t ticks = App_GetControlTicks();
    uint32_t elapsedTicks = ticks - lastControlTicks;

    if (elapsedTicks == 0U) {
        SpeedPID_GetSnapshot(&g_pidSnapshot);
        return;
    }

    {
        uint32_t periodMs;

        lastControlTicks = ticks;
        if (elapsedTicks > (UINT32_MAX / APP_CONTROL_PERIOD_MS)) {
            periodMs = UINT32_MAX;
        } else {
            periodMs = elapsedTicks * APP_CONTROL_PERIOD_MS;
        }

        /* Collapse backlog into one correctly timed sample, not false zeros. */
        Encoder_SamplePeriodMs(periodMs);
        Encoder_GetSnapshot(&g_encoderSnapshot);
        if (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_ARMED) {
            /*
             * Capture the baseline from the same sampled control tick that
             * first applies PWM. This removes the button/UI-to-control skew.
             */
            g_motorIoTest.startLeftCounts =
                g_encoderSnapshot.leftPositionCounts;
            g_motorIoTest.startRightCounts =
                g_encoderSnapshot.rightPositionCounts;
            g_motorIoTest.leftDeltaCounts = 0;
            g_motorIoTest.rightDeltaCounts = 0;
            g_motorIoTest.startedMs = App_GetMillis();
            g_motorIoTest.lastUiRefreshMs = g_motorIoTest.startedMs;
            g_motorIoTest.phase = MOTOR_IO_TEST_PHASE_RUNNING;
            App_ApplyMotorIoTestCommand();
            UI_RequestRefresh();
            SpeedPID_GetSnapshot(&g_pidSnapshot);
            MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
            return;
        }
        if (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_RUNNING) {
            /*
             * Explicit page-6 commissioning bypass: keep encoder sampling,
             * but do not let the disabled PID overwrite the raw output and
             * do not accumulate normal DIR/NoSig timers during the pulse.
             */
            App_UpdateMotorIoTestDeltas();
            App_ApplyMotorIoTestCommand();
            SpeedPID_GetSnapshot(&g_pidSnapshot);
            MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
            return;
        }
        if (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_SETTLING) {
            /* PWM is already off; include the bounded coast-down pulses. */
            App_UpdateMotorIoTestDeltas();
            Motor_SetPhysicalOutputs(0, 0);
            if (App_Elapsed(App_GetMillis(), g_motorIoTest.stoppedMs,
                            APP_MOTOR_IO_TEST_SETTLING_MS)) {
                /* Freeze only after this control tick sampled the encoders. */
                g_motorIoTest.phase = MOTOR_IO_TEST_PHASE_DONE;
                UI_RequestRefresh();
            }
            SpeedPID_GetSnapshot(&g_pidSnapshot);
            MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
            return;
        }
        if ((g_appStatus.encoderSignalMissing != 0U) ||
            (g_appStatus.encoderDirectionMismatch != 0U)) {
            /* Reassert the latch before PID can write a nonzero PWM output. */
            if (g_wirelessRemote.running != 0U) {
                App_StopWirelessRemote(UI_WIRELESS_ACTION_SENSOR_FAILED);
            }
            if (App_AutoRunIsBusy()) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            }
            MotorDebug_AbortFault(MOTOR_DEBUG_STOP_ENCODER_FAULT);
        }
        if ((!App_ProcessWirelessRemoteControl10ms()) &&
            (!App_ProcessAutoRunControl10ms(App_GetMillis()))) {
            MotorDebug_Update10ms(&g_encoderSnapshot, g_attitude.yaw_deg,
                (g_appStatus.imuReady != 0U) &&
                (g_appStatus.imuFailed == 0U));
        }
        SpeedPID_Update10ms(&g_encoderSnapshot);
        MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
    }

    SpeedPID_GetSnapshot(&g_pidSnapshot);

    if ((g_appStatus.encoderSignalMissing != 0U) ||
        (g_appStatus.encoderDirectionMismatch != 0U)) {
        /* Latched safety fault: reset/power-cycle after checking the plant. */
        return;
    }

    if (!g_pidSnapshot.enabled) {
        leftNoSignalMs = 0U;
        rightNoSignalMs = 0U;
        leftWrongDirectionMs = 0U;
        rightWrongDirectionMs = 0U;
        return;
    }

    if (g_pidSnapshot.leftDutyCounts >=
        APP_ENCODER_SIGNAL_TEST_DUTY_COUNTS) {
        if (g_encoderSnapshot.leftDeltaCounts != 0) {
            leftNoSignalMs = 0U;
        } else if ((UINT32_MAX - leftNoSignalMs) <
                   g_encoderSnapshot.samplePeriodMs) {
            leftNoSignalMs = UINT32_MAX;
        } else {
            leftNoSignalMs += g_encoderSnapshot.samplePeriodMs;
        }
    } else {
        leftNoSignalMs = 0U;
    }

    if (g_pidSnapshot.rightDutyCounts >=
        APP_ENCODER_SIGNAL_TEST_DUTY_COUNTS) {
        if (g_encoderSnapshot.rightDeltaCounts != 0) {
            rightNoSignalMs = 0U;
        } else if ((UINT32_MAX - rightNoSignalMs) <
                   g_encoderSnapshot.samplePeriodMs) {
            rightNoSignalMs = UINT32_MAX;
        } else {
            rightNoSignalMs += g_encoderSnapshot.samplePeriodMs;
        }
    } else {
        rightNoSignalMs = 0U;
    }

    if ((leftNoSignalMs >= APP_ENCODER_SIGNAL_TIMEOUT_MS) ||
        (rightNoSignalMs >= APP_ENCODER_SIGNAL_TIMEOUT_MS)) {
        uint8_t signalMask = 0U;
        if (leftNoSignalMs >= APP_ENCODER_SIGNAL_TIMEOUT_MS) {
            signalMask |= 1U;
        }
        if (rightNoSignalMs >= APP_ENCODER_SIGNAL_TIMEOUT_MS) {
            signalMask |= 2U;
        }
        g_appStatus.encoderSignalMissing = signalMask;
        MotorDebug_AbortFault(MOTOR_DEBUG_STOP_ENCODER_FAULT);
        SpeedPID_GetSnapshot(&g_pidSnapshot);
        MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
        UI_RequestRefresh();
        return;
    }

    if (App_FeedbackDirectionWrong(g_pidSnapshot.leftTargetCmS,
            g_encoderSnapshot.leftSpeedCmS,
            g_pidSnapshot.leftDutyCounts)) {
        leftWrongDirectionMs = App_AccumulateMs(leftWrongDirectionMs,
            g_encoderSnapshot.samplePeriodMs);
    } else {
        leftWrongDirectionMs = 0U;
    }
    if (App_FeedbackDirectionWrong(g_pidSnapshot.rightTargetCmS,
            g_encoderSnapshot.rightSpeedCmS,
            g_pidSnapshot.rightDutyCounts)) {
        rightWrongDirectionMs = App_AccumulateMs(rightWrongDirectionMs,
            g_encoderSnapshot.samplePeriodMs);
    } else {
        rightWrongDirectionMs = 0U;
    }

    if ((leftWrongDirectionMs >= APP_ENCODER_DIRECTION_TIMEOUT_MS) ||
        (rightWrongDirectionMs >= APP_ENCODER_DIRECTION_TIMEOUT_MS)) {
        uint8_t directionMask = 0U;
        if (leftWrongDirectionMs >= APP_ENCODER_DIRECTION_TIMEOUT_MS) {
            directionMask |= 1U;
        }
        if (rightWrongDirectionMs >= APP_ENCODER_DIRECTION_TIMEOUT_MS) {
            directionMask |= 2U;
        }
        g_appStatus.encoderDirectionMismatch = directionMask;
        MotorDebug_AbortFault(MOTOR_DEBUG_STOP_ENCODER_FAULT);
        SpeedPID_GetSnapshot(&g_pidSnapshot);
        MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
        UI_RequestRefresh();
    }
}

static uint8_t App_InitializeImu(void)
{
    g_appStatus.imuInitialized = 0U;
    g_appStatus.imuCalibrating = 0U;
    g_appStatus.imuReady = 0U;
    App_ClearImuReadyTimer();

    if (ICM42688_Init((ICM42688_MountMode_t)APP_ICM_MOUNT_MODE) !=
        ICM42688_STATUS_OK) {
        g_appStatus.imuFailed = 1U;
        return 0U;
    }

    g_appStatus.imuInitialized = 1U;
    if (ICM42688_BeginCalibration(
            ICM42688_DEFAULT_CALIBRATION_SAMPLES) !=
        ICM42688_STATUS_OK) {
        g_appStatus.imuFailed = 1U;
        return 0U;
    }

    g_appStatus.imuCalibrating = 1U;
    g_appStatus.imuFailed = 0U;
    return 1U;
}

static void App_ProcessImu(uint32_t now)
{
    static uint32_t lastImuAttemptMs;
    static uint32_t lastSuccessfulUpdateMs;
    static uint8_t consecutiveFailures;
    uint32_t intervalMs;

    if ((g_appStatus.imuFailed != 0U) &&
        (g_appStatus.imuInitialized == 0U)) {
        intervalMs = APP_IMU_RECONNECT_BACKOFF_MS;
    } else if (g_appStatus.imuFailed != 0U) {
        intervalMs = APP_IMU_RETRY_BACKOFF_MS;
    } else {
        intervalMs = APP_IMU_PERIOD_MS;
    }

    if (!App_Elapsed(now, lastImuAttemptMs, intervalMs)) {
        return;
    }
    lastImuAttemptMs = now;

    /* A full reinitialization can take hundreds of milliseconds.  Retry only
     * while the motors are stopped so the control loop cannot be starved. */
    if ((g_appStatus.imuFailed != 0U) &&
        (g_appStatus.imuInitialized == 0U)) {
        if (MotorDebug_IsRunning() || SpeedPID_IsEnabled()) {
            return;
        }

        (void)App_InitializeImu();
        consecutiveFailures = 0U;
        lastSuccessfulUpdateMs = 0U;
        lastImuAttemptMs = App_GetMillis();
        UI_RequestRefresh();
        return;
    }

    /*
     * 初始化已经成功但静止校准因启动瞬态或车体移动失败时，按照
     * APP_IMU_RETRY_BACKOFF_MS 自动重新开始校准。保持原静止阈值，
     * 避免用放宽判据的方式保存错误零偏。
     */
    if ((g_appStatus.imuFailed != 0U) &&
        (g_appStatus.imuInitialized != 0U) &&
        (g_appStatus.imuCalibrating == 0U) &&
        (ICM42688_IsReady() == 0U)) {
        if (ICM42688_BeginCalibration(
                ICM42688_DEFAULT_CALIBRATION_SAMPLES) ==
            ICM42688_STATUS_OK) {
            g_appStatus.imuCalibrating = 1U;
            g_appStatus.imuFailed = 0U;
            consecutiveFailures = 0U;
            UI_RequestRefresh();
        }
        return;
    }

    if (g_appStatus.imuCalibrating != 0U) {
        ICM42688_CalibrationState_t calibration =
            ICM42688_ProcessCalibration();

        if (calibration == ICM42688_CALIBRATION_COMPLETE) {
            g_appStatus.imuCalibrating = 0U;
            App_StartImuReadyTimer(now);
            g_appStatus.imuReady = 1U;
            g_appStatus.imuFailed = 0U;
            consecutiveFailures = 0U;
            lastSuccessfulUpdateMs = now;
            UI_RequestRefresh();
        } else if (calibration == ICM42688_CALIBRATION_ERROR) {
            g_appStatus.imuCalibrating = 0U;
            g_appStatus.imuReady = 0U;
            App_ClearImuReadyTimer();
            g_appStatus.imuFailed = 1U;
            if (ICM42688_GetLastStatus() ==
                ICM42688_STATUS_ERROR_DATA_READ) {
                g_appStatus.imuInitialized = 0U;
            }
            UI_RequestRefresh();
        }
        return;
    }

    /* A calibrated device continues to be polled after transient failures. */
    if (ICM42688_IsReady() == 0U) {
        return;
    }

    {
        uint32_t integrationElapsedMs =
            (lastSuccessfulUpdateMs == 0U) ?
                APP_IMU_PERIOD_MS :
                (uint32_t)(now - lastSuccessfulUpdateMs);
        float dtSeconds = ((float)integrationElapsedMs) * 0.001f;
        uint8_t wasFailed = g_appStatus.imuFailed;

        if (dtSeconds < APP_IMU_DT_SECONDS) {
            dtSeconds = APP_IMU_DT_SECONDS;
        } else if (dtSeconds > APP_IMU_MAX_DT_SECONDS) {
            dtSeconds = APP_IMU_MAX_DT_SECONDS;
        }

        if (ICM42688_Update(dtSeconds) != 0U) {
            lastSuccessfulUpdateMs = now;
            consecutiveFailures = 0U;
            App_StartImuReadyTimer(now);
            g_appStatus.imuReady = 1U;
            g_appStatus.imuFailed = 0U;
            ICM42688_GetAttitude(&g_attitude);
            if (wasFailed != 0U) {
                UI_RequestRefresh();
            }
        } else {
            if (consecutiveFailures < UINT8_MAX) {
                ++consecutiveFailures;
            }
            if (consecutiveFailures >= APP_IMU_FAILURE_THRESHOLD) {
                if (g_appStatus.imuFailed == 0U) {
                    UI_RequestRefresh();
                }
                g_appStatus.imuReady = 0U;
                App_ClearImuReadyTimer();
                g_appStatus.imuFailed = 1U;
                g_appStatus.imuInitialized = 0U;
            }
        }
    }
}

static UiModuleStatus App_GetImuUiStatus(void)
{
    if (g_appStatus.imuReady != 0U) {
        return UI_MODULE_READY;
    }
    if ((g_appStatus.imuInitialized != 0U) ||
        (g_appStatus.imuCalibrating != 0U)) {
        return UI_MODULE_INITIALIZING;
    }
    if (g_appStatus.imuFailed != 0U) {
        return UI_MODULE_FAILED;
    }
    return UI_MODULE_NOT_READY;
}

static void App_BuildUiModel(UiModel *model, uint32_t now)
{
    MotorDebugPIDSettings upperPid;
    ICM42688_CalibrationProgress_t calibrationProgress;
    int64_t positionSum = (int64_t)g_encoderSnapshot.leftPositionCounts +
        (int64_t)g_encoderSnapshot.rightPositionCounts;
    int64_t deltaSum = (int64_t)g_encoderSnapshot.leftDeltaCounts +
        (int64_t)g_encoderSnapshot.rightDeltaCounts;

    model->system_running = g_appStatus.systemRunning;
    model->oled_initialized = g_appStatus.oledInitialized;
    model->motor_enabled = SpeedPID_IsEnabled() ? 1U : 0U;
    model->imu_status = App_GetImuUiStatus();
    model->imu_last_status = (uint8_t)ICM42688_GetLastStatus();
    model->imu_i2c_address = ICM42688_GetI2CAddress();
    model->imu_who_am_i = ICM42688_GetLastWhoAmI();
    model->imu_i2c_error_count = ICM42688_GetI2CErrorCount();
    model->imu_i2c_phase = (uint8_t)ICM42688_GetLastI2CPhase();
    model->imu_i2c_line_state = ICM42688_GetLastI2CLineState();
    ICM42688_GetCalibrationProgress(&calibrationProgress);
    model->imu_calibration_percent =
        calibrationProgress.percent_complete;
    model->imu_calibration_rejected =
        calibrationProgress.rejected_samples;
    model->imu_ready_elapsed_ms =
        ((g_appStatus.imuReady != 0U) && g_imuReadyTimerActive) ?
            (uint32_t)(now - g_imuReadySinceMs) : 0U;
    model->encoder_direction_error_mask =
        g_appStatus.encoderDirectionMismatch;
    model->encoder_signal_missing_mask =
        g_appStatus.encoderSignalMissing;
    if (g_appStatus.encoderConfigured == 0U) {
        model->encoder_status = UI_MODULE_FAILED;
    } else if (g_appStatus.encoderDirectionMismatch != 0U) {
        model->encoder_status = UI_MODULE_DIRECTION_ERROR;
    } else if (g_appStatus.encoderSignalMissing != 0U) {
        model->encoder_status = UI_MODULE_SIGNAL_MISSING;
    } else {
        model->encoder_status = UI_MODULE_READY;
    }
    model->flash_status = ((g_appStatus.flashInitialized != 0U) &&
                           (g_appStatus.flashSaveFailed == 0U)) ?
        UI_MODULE_READY : UI_MODULE_FAILED;
    model->line_sensor_status =
        (g_appStatus.lineSensorsInitialized != 0U) ?
            UI_MODULE_READY : UI_MODULE_FAILED;
    model->battery_voltage_mv = g_batterySnapshot.voltageMilliVolts;
    model->battery_ready = g_batterySnapshot.ready;
    model->battery_low_voltage =
        g_batterySnapshot.lowVoltageWarning;
    model->wireless_initialized = g_wirelessSnapshot.initialized;
    model->wireless_session_active =
        g_wirelessSnapshot.sessionActive;
    model->wireless_connected = g_wirelessSnapshot.connected;
    model->wireless_sequence_valid =
        g_wirelessSnapshot.sequenceValid;
    model->wireless_rpd_detected = g_wirelessSnapshot.rpdDetected;
    model->wireless_ce_high = g_wirelessSnapshot.ceHigh;
    model->wireless_active_mode = g_wirelessSnapshot.activeMode;
    model->wireless_state = g_wirelessSnapshot.state;
    model->wireless_last_tx_failure =
        g_wirelessSnapshot.lastTxFailure;
    model->wireless_link_status = g_wirelessSnapshot.linkStatus;
    model->wireless_send_status = g_wirelessSnapshot.sendStatus;
    model->wireless_tx_command = g_wirelessSnapshot.txCommand;
    model->wireless_rx_command = g_wirelessSnapshot.rxCommand;
    model->wireless_tx_value = g_wirelessSnapshot.txValue;
    model->wireless_rx_value = g_wirelessSnapshot.rxValue;
    model->wireless_retry_attempt = g_wirelessSnapshot.retryAttempt;
    model->wireless_command_generation =
        g_wirelessSnapshot.commandGeneration;
    model->wireless_action_state = g_wirelessRemote.actionState;
    model->wireless_action_command = g_wirelessRemote.command;
    model->wireless_action_value = g_wirelessRemote.value;
    model->wireless_rx_event_count =
        g_wirelessSnapshot.rxEventCount;
    model->wireless_rx_count = g_wirelessSnapshot.rxCount;
    model->wireless_tx_success_count =
        g_wirelessSnapshot.txSuccessCount;
    model->wireless_tx_failure_count =
        g_wirelessSnapshot.txFailureCount;
    model->wireless_last_sequence =
        g_wirelessSnapshot.lastSequence;
    model->wireless_last_activity_age_ms =
        g_wirelessSnapshot.lastActivityAgeMs;
    for (uint8_t i = 0U; i < 4U; ++i) {
        model->wireless_data_prefix[i] =
            g_wirelessSnapshot.dataPrefix[i];
    }

    model->roll_deg = g_attitude.roll_deg;
    model->pitch_deg = g_attitude.pitch_deg;
    model->yaw_deg = g_attitude.yaw_deg;

    /* The single UI value is the real arithmetic mean of the two wheels. */
    model->encoder_speed_cm_s =
        0.5f * (g_encoderSnapshot.leftSpeedCmS +
                g_encoderSnapshot.rightSpeedCmS);
    model->encoder_distance_cm = g_encoderSnapshot.vehicleDistanceCm;
    model->target_speed_cm_s =
        g_motorDebugSnapshot.base_speed_cm_s;
    model->target_distance_cm = g_motorDebugSnapshot.target_distance_cm;
    model->encoder_position_count = (int32_t)(positionSum / 2);
    model->left_encoder_position_count =
        g_encoderSnapshot.leftPositionCounts;
    model->right_encoder_position_count =
        g_encoderSnapshot.rightPositionCounts;
    model->encoder_delta_count = (int32_t)(deltaSum / 2);
    model->pid_output = 0.5f *
        ((float)g_pidSnapshot.leftOutput +
         (float)g_pidSnapshot.rightOutput);
    model->pwm_duty_percent = 0.05f *
        ((float)g_pidSnapshot.leftDutyCounts +
         (float)g_pidSnapshot.rightDutyCounts);
    model->pid_kp = g_pidSnapshot.kp;
    model->pid_ki = g_pidSnapshot.ki;
    model->pid_kd = g_pidSnapshot.kd;
    MotorDebug_GetPIDSettings(&upperPid);
    model->straight_pid_kp = upperPid.straight_kp;
    model->straight_pid_ki = upperPid.straight_ki;
    model->straight_pid_kd = upperPid.straight_kd;
    model->line_pid_kp = upperPid.line_kp;
    model->line_pid_ki = upperPid.line_ki;
    model->line_pid_kd = upperPid.line_kd;
    model->angle_pid_kp = upperPid.angle_kp;
    model->angle_pid_ki = upperPid.angle_ki;
    model->angle_pid_kd = upperPid.angle_kd;

    model->motor_debug_mode = g_motorDebugSnapshot.mode;
    model->motor_debug_state = g_motorDebugSnapshot.state;
    model->motor_debug_stop_reason = g_motorDebugSnapshot.stop_reason;
    model->motor_debug_travel_cm =
        g_motorDebugSnapshot.traveled_distance_cm;
    model->line_error = g_motorDebugSnapshot.line_error;
    model->line_active_count = g_motorDebugSnapshot.line_active_count;
    model->line_lost = g_motorDebugSnapshot.line_lost;
    model->motor_io_test_command = g_motorIoTest.command;
    model->motor_io_test_phase = g_motorIoTest.phase;
    model->motor_io_test_active =
        (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_RUNNING) ? 1U : 0U;
    model->motor_io_test_left_delta = g_motorIoTest.leftDeltaCounts;
    model->motor_io_test_right_delta = g_motorIoTest.rightDeltaCounts;
    model->selected_test_index = g_currentSettings.selected_test_index;
    model->test_select_status =
        App_AutoRunUiStatus(APP_AUTO_RUN_TEST_SLOT);
    model->task1_elapsed_ms = App_TaskOneElapsedMs(now);
    model->task1_distance_cm = g_autoRun.task1DistanceCm;
    model->task1_target_distance_cm =
        g_autoRun.task1TargetDistanceCm;
    model->task1_command_speed_cm_s =
        g_autoRun.task1CommandSpeedCmS;
    model->task2_elapsed_ms = App_TaskTwoElapsedMs(now);
    model->task2_distance_cm = g_autoRun.task2DistanceCm;
    model->task2_target_distance_cm = APP_TASK2_TARGET_DISTANCE_CM;
    model->task2_command_speed_cm_s =
        g_autoRun.task2CommandSpeedCmS;
    model->task3_elapsed_ms = App_TaskThreeElapsedMs(now);
    model->task3_distance_cm = g_autoRun.task3DistanceCm;
    model->task3_command_speed_cm_s =
        g_autoRun.task3CommandSpeedCmS;
    model->task3_finish_armed =
        (g_autoRun.task3DistanceCm >= APP_TASK3_FINISH_GATE_CM) ? 1U : 0U;
    model->task3_finish_seen = g_autoRun.task3FinishSeen;
    model->square_test_status =
        App_AutoRunUiStatus(APP_AUTO_RUN_SQUARE);
    model->circle_test_status =
        App_AutoRunUiStatus(APP_AUTO_RUN_CIRCLE);
    model->square_side_index = g_autoRun.squareSideIndex;
    model->square_progress_cm = g_motorDebugSnapshot.traveled_distance_cm;
    model->circle_progress_cm = g_autoRun.circleProgressCm;
    model->square_speed_cm_s = g_currentSettings.square_speed_cm_s;
    model->square_side_cm = g_currentSettings.square_side_cm;
    model->circle_speed_cm_s = g_currentSettings.circle_speed_cm_s;
    model->circle_radius_cm = g_currentSettings.circle_radius_cm;
}

static void App_FillDefaultSettings(AppPersistentSettings *settings)
{
    settings->target_speed_cm_s = APP_DEFAULT_TARGET_SPEED_CM_S;
    settings->target_distance_cm = APP_DEFAULT_TARGET_DISTANCE_CM;
    settings->pid_kp = APP_SPEED_PID_KP;
    settings->pid_ki = APP_SPEED_PID_KI;
    settings->pid_kd = APP_SPEED_PID_KD;
    settings->straight_pid_kp = APP_STRAIGHT_PID_KP;
    settings->straight_pid_ki = APP_STRAIGHT_PID_KI;
    settings->straight_pid_kd = APP_STRAIGHT_PID_KD;
    settings->line_pid_kp = APP_LINE_PID_KP_PWM;
    settings->line_pid_ki = APP_LINE_PID_KI_PWM;
    settings->line_pid_kd = APP_LINE_PID_KD_PWM;
    settings->angle_pid_kp = APP_ANGLE_PID_KP;
    settings->angle_pid_ki = APP_ANGLE_PID_KI;
    settings->angle_pid_kd = APP_ANGLE_PID_KD;
    settings->motor_mode = (uint8_t)MOTOR_DEBUG_MODE_DISTANCE;
    settings->selected_test_index = APP_DEFAULT_SELECTED_TEST_INDEX;
    settings->square_speed_cm_s = APP_DEFAULT_SQUARE_SPEED_CM_S;
    settings->square_side_cm = APP_DEFAULT_SQUARE_SIDE_CM;
    settings->circle_speed_cm_s = APP_DEFAULT_CIRCLE_SPEED_CM_S;
    settings->circle_radius_cm = APP_DEFAULT_CIRCLE_RADIUS_CM;
}

static bool App_ApplySettings(const AppPersistentSettings *settings)
{
    SpeedPIDParameters parameters;
    MotorDebugPIDSettings upperPid;

    if ((settings == NULL) ||
        (settings->selected_test_index >= APP_TEST_SLOT_COUNT) ||
        (!isfinite(settings->square_speed_cm_s)) ||
        (!isfinite(settings->square_side_cm)) ||
        (!isfinite(settings->circle_speed_cm_s)) ||
        (!isfinite(settings->circle_radius_cm)) ||
        (settings->square_speed_cm_s < APP_AUTO_TEST_SPEED_MIN_CM_S) ||
        (settings->square_speed_cm_s > APP_AUTO_TEST_SPEED_MAX_CM_S) ||
        (settings->square_side_cm < APP_SQUARE_SIDE_MIN_CM) ||
        (settings->square_side_cm > APP_SQUARE_SIDE_MAX_CM) ||
        (settings->circle_speed_cm_s < APP_AUTO_TEST_SPEED_MIN_CM_S) ||
        (settings->circle_speed_cm_s > APP_AUTO_TEST_SPEED_MAX_CM_S) ||
        (settings->circle_radius_cm < APP_CIRCLE_RADIUS_MIN_CM) ||
        (settings->circle_radius_cm > APP_CIRCLE_RADIUS_MAX_CM)) {
        return false;
    }

    parameters.kp = settings->pid_kp;
    parameters.ki = settings->pid_ki;
    parameters.kd = settings->pid_kd;
    if (!SpeedPID_SetParameters(&parameters)) {
        return false;
    }
    upperPid.straight_kp = settings->straight_pid_kp;
    upperPid.straight_ki = settings->straight_pid_ki;
    upperPid.straight_kd = settings->straight_pid_kd;
    upperPid.line_kp = settings->line_pid_kp;
    upperPid.line_ki = settings->line_pid_ki;
    upperPid.line_kd = settings->line_pid_kd;
    upperPid.angle_kp = settings->angle_pid_kp;
    upperPid.angle_ki = settings->angle_pid_ki;
    upperPid.angle_kd = settings->angle_pid_kd;
    if (!MotorDebug_SetPIDSettings(&upperPid)) {
        return false;
    }
    SpeedPID_SetTargets(settings->target_speed_cm_s,
                        settings->target_speed_cm_s);
    if (!MotorDebug_SetConfiguration(
            (MotorDebugMode)settings->motor_mode,
            settings->target_distance_cm,
            settings->target_speed_cm_s)) {
        return false;
    }
    SpeedPID_GetSnapshot(&g_pidSnapshot);
    MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
    g_currentSettings = *settings;
    return true;
}

static void App_CopyUiSettingsToPersistent(
    const UiControlSettings *uiSettings,
    AppPersistentSettings *settings)
{
    settings->target_speed_cm_s = uiSettings->target_speed_cm_s;
    settings->target_distance_cm = uiSettings->target_distance_cm;
    settings->pid_kp = uiSettings->pid_kp;
    settings->pid_ki = uiSettings->pid_ki;
    settings->pid_kd = uiSettings->pid_kd;
    settings->straight_pid_kp = uiSettings->straight_pid_kp;
    settings->straight_pid_ki = uiSettings->straight_pid_ki;
    settings->straight_pid_kd = uiSettings->straight_pid_kd;
    settings->line_pid_kp = uiSettings->line_pid_kp;
    settings->line_pid_ki = uiSettings->line_pid_ki;
    settings->line_pid_kd = uiSettings->line_pid_kd;
    settings->angle_pid_kp = uiSettings->angle_pid_kp;
    settings->angle_pid_ki = uiSettings->angle_pid_ki;
    settings->angle_pid_kd = uiSettings->angle_pid_kd;
    settings->motor_mode = (uint8_t)uiSettings->motor_mode;
    settings->selected_test_index = uiSettings->selected_test_index;
    settings->square_speed_cm_s = uiSettings->square_speed_cm_s;
    settings->square_side_cm = uiSettings->square_side_cm;
    settings->circle_speed_cm_s = uiSettings->circle_speed_cm_s;
    settings->circle_radius_cm = uiSettings->circle_radius_cm;
}

static void App_ProcessUiSaveRequest(void)
{
    UiControlSettings uiSettings;

    if (UI_TakeSavedControlSettings(&uiSettings) != 0U) {
        AppPersistentSettings settings;
        bool applied;
        bool saved = false;

        /* External flash erase/program is intentionally done with motors off. */
        if (MotorDebug_IsRunning()) {
            MotorDebug_Stop();
        }

        App_CopyUiSettingsToPersistent(&uiSettings, &settings);
        applied = App_ApplySettings(&settings);
        if (applied && (g_appStatus.flashInitialized != 0U)) {
            saved = SettingsStore_Save(&settings);
        }
        g_appStatus.flashSaveFailed = saved ? 0U : 1U;
        UI_ReportSaveResult(saved ? UI_SAVE_RESULT_SUCCESS :
                                    UI_SAVE_RESULT_FAILED);
        UI_RequestRefresh();
    }
}

static void App_ProcessMotorDebugButton(uint32_t buttonEvents)
{
    if (App_AutoRunIsBusy()) {
        return;
    }
    if ((buttonEvents & BUTTON_EVENT_CENTER) == 0U) {
        return;
    }

    /* A short center press stops a running test. Starting is accepted only
     * on the Motor Debug page after the logo, preventing starts on parameter pages. The
     * button driver suppresses the short event after a long save press. */
    if (MotorDebug_IsRunning()) {
        MotorDebug_Stop();
    } else if ((UI_IsLogoActive() == 0U) &&
               (UI_GetPage() == UI_PAGE_MOTOR_DEBUG)) {
        UiControlSettings uiSettings;
        AppPersistentSettings runSettings;
        bool settingsApplied;

        UI_GetDraftControlSettings(&uiSettings);
        App_CopyUiSettingsToPersistent(&uiSettings, &runSettings);
        settingsApplied = App_ApplySettings(&runSettings);
        if (!settingsApplied) {
            MotorDebug_AbortFault(MOTOR_DEBUG_STOP_INVALID_CONFIG);
        }
        if (g_appStatus.motorInitialized == 0U) {
            MotorDebug_AbortFault(MOTOR_DEBUG_STOP_MOTOR_FAULT);
        } else if (settingsApplied) {
            bool headingReady =
                ((MotorDebugMode)runSettings.motor_mode ==
                    MOTOR_DEBUG_MODE_DISTANCE) &&
                (g_appStatus.imuReady != 0U) &&
                (g_appStatus.imuFailed == 0U) &&
                (ICM42688_IsReady() != 0U);

            (void)MotorDebug_Start(
                &g_encoderSnapshot,
                (g_appStatus.encoderConfigured != 0U) &&
                (g_appStatus.encoderSignalMissing == 0U) &&
                (g_appStatus.encoderDirectionMismatch == 0U),
                g_attitude.yaw_deg,
                headingReady);
        }
    }
    MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
    SpeedPID_GetSnapshot(&g_pidSnapshot);
    UI_RequestRefresh();
}

static bool App_MotorIoTestIsBusy(void)
{
    return (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_ARMED) ||
        (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_RUNNING) ||
        (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_SETTLING);
}

static void App_AbortMotorIoTest(void)
{
    if ((g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_RUNNING) ||
        (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_SETTLING)) {
        App_UpdateMotorIoTestDeltas();
    }
    Motor_SetPhysicalOutputs(0, 0);
    if (g_motorIoTest.phase != MOTOR_IO_TEST_PHASE_IDLE) {
        g_motorIoTest.phase = MOTOR_IO_TEST_PHASE_DONE;
    }
    UI_RequestRefresh();
}

static void App_BeginMotorIoTestSettling(uint32_t now)
{
    if (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_RUNNING) {
        App_UpdateMotorIoTestDeltas();
        Motor_SetPhysicalOutputs(0, 0);
        g_motorIoTest.stoppedMs = now;
        g_motorIoTest.lastUiRefreshMs = now;
        g_motorIoTest.phase = MOTOR_IO_TEST_PHASE_SETTLING;
        UI_RequestRefresh();
    } else if (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_ARMED) {
        /* No PWM was applied yet, so there is no coast-down interval. */
        Motor_SetPhysicalOutputs(0, 0);
        g_motorIoTest.phase = MOTOR_IO_TEST_PHASE_DONE;
        UI_RequestRefresh();
    } else if (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_SETTLING) {
        Motor_SetPhysicalOutputs(0, 0);
    }
}

static void App_StartMotorIoTest(MotorIoTestCommand command, uint32_t now)
{
    if ((command == MOTOR_IO_TEST_COMMAND_OFF) ||
        (command > MOTOR_IO_TEST_COMMAND_B_NEGATIVE)) {
        App_AbortMotorIoTest();
        return;
    }

    if (MotorDebug_IsRunning()) {
        MotorDebug_Stop();
    }
    SpeedPID_Enable(false);
    Motor_SetPhysicalOutputs(0, 0);

    g_motorIoTest.command = command;
    g_motorIoTest.phase = MOTOR_IO_TEST_PHASE_ARMED;
    g_motorIoTest.lastUiRefreshMs = now;
    g_motorIoTest.leftDeltaCounts = 0;
    g_motorIoTest.rightDeltaCounts = 0;
    UI_RequestRefresh();
}

static void App_ProcessMotorIoTest(uint32_t now, uint32_t buttonEvents)
{
    MotorIoTestCommand requested = MOTOR_IO_TEST_COMMAND_OFF;

    if (App_AutoRunIsBusy()) {
        return;
    }

    if ((UI_IsLogoActive() != 0U) ||
        (UI_GetPage() != UI_PAGE_MOTOR_IO_TEST)) {
        if (App_MotorIoTestIsBusy()) {
            App_AbortMotorIoTest();
        }
        return;
    }

    /* Page 6 is exclusively open-loop; never leave page-5 PID motion active. */
    if (MotorDebug_IsRunning()) {
        MotorDebug_Stop();
        MotorDebug_GetSnapshot(&g_motorDebugSnapshot);
        SpeedPID_GetSnapshot(&g_pidSnapshot);
        UI_RequestRefresh();
    }

    if ((buttonEvents & (BUTTON_EVENT_PAGE_PREV |
                         BUTTON_EVENT_PAGE_NEXT)) != 0U) {
        App_AbortMotorIoTest();
        return;
    }
    if (((buttonEvents & (BUTTON_EVENT_CENTER |
                          BUTTON_EVENT_CENTER_LONG)) != 0U) ||
        Buttons_IsPressed(BUTTON_ID_CENTER)) {
        if (App_MotorIoTestIsBusy()) {
            App_BeginMotorIoTestSettling(now);
        }
        return;
    }
    if ((buttonEvents & BUTTON_EVENT_UP) != 0U) {
        requested = MOTOR_IO_TEST_COMMAND_A_POSITIVE;
    } else if ((buttonEvents & BUTTON_EVENT_DOWN) != 0U) {
        requested = MOTOR_IO_TEST_COMMAND_A_NEGATIVE;
    } else if ((buttonEvents & BUTTON_EVENT_RIGHT) != 0U) {
        requested = MOTOR_IO_TEST_COMMAND_B_POSITIVE;
    } else if ((buttonEvents & BUTTON_EVENT_LEFT) != 0U) {
        requested = MOTOR_IO_TEST_COMMAND_B_NEGATIVE;
    }

    if (requested != MOTOR_IO_TEST_COMMAND_OFF) {
        App_StartMotorIoTest(requested, now);
    } else if ((g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_RUNNING) &&
        App_Elapsed(now, g_motorIoTest.startedMs,
                    APP_MOTOR_IO_TEST_DURATION_MS)) {
        App_BeginMotorIoTestSettling(now);
    } else if (((g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_RUNNING) ||
                (g_motorIoTest.phase == MOTOR_IO_TEST_PHASE_SETTLING)) &&
        App_Elapsed(now, g_motorIoTest.lastUiRefreshMs,
                    APP_MOTOR_IO_TEST_UI_REFRESH_MS)) {
        g_motorIoTest.lastUiRefreshMs = now;
        UI_RequestRefresh();
    }
}

static void App_ResetTaskOneReady(void)
{
    SpeedPID_Enable(false);
    MotorDebug_Stop();
    if ((g_autoRun.task1TargetDistanceCm <
         APP_TASK1_STOP_DISTANCE_MIN_CM) ||
        (g_autoRun.task1TargetDistanceCm >
         APP_TASK1_STOP_DISTANCE_MAX_CM)) {
        g_autoRun.task1TargetDistanceCm =
            APP_TASK1_STOP_DISTANCE_DEFAULT_CM;
    }
    g_autoRun.phase = APP_AUTO_PHASE_IDLE;
    g_autoRun.task1StartDistanceCm =
        g_encoderSnapshot.vehicleDistanceCm;
    g_autoRun.task1DistanceCm = 0.0f;
    g_autoRun.task1CommandSpeedCmS = 0.0f;
    g_autoRun.task1RunStartedMs = 0U;
    g_autoRun.task1ElapsedMs = 0U;
    UI_RequestRefresh();
}

static void App_ResetTaskTwoReady(void)
{
    SpeedPID_Enable(false);
    MotorDebug_Stop();
    g_autoRun.phase = APP_AUTO_PHASE_IDLE;
    g_autoRun.task2StartDistanceCm =
        g_encoderSnapshot.vehicleDistanceCm;
    g_autoRun.task2DistanceCm = 0.0f;
    g_autoRun.task2CommandSpeedCmS = 0.0f;
    g_autoRun.task2RunStartedMs = 0U;
    g_autoRun.task2ElapsedMs = 0U;
    g_autoRun.task2TimingDone = 0U;
    g_autoRun.task2ExtensionActive = 0U;
    g_autoRun.task2ExtensionStartDistanceCm = 0.0f;
    g_autoRun.task2DecelStartedMs = 0U;
    g_autoRun.task2DecelActive = 0U;
    UI_RequestRefresh();
}

static void App_ResetTaskThreeReady(void)
{
    SpeedPID_Enable(false);
    MotorDebug_Stop();
    g_autoRun.phase = APP_AUTO_PHASE_IDLE;
    g_autoRun.task3StartDistanceCm =
        g_encoderSnapshot.vehicleDistanceCm;
    g_autoRun.task3DistanceCm = 0.0f;
    g_autoRun.task3CommandSpeedCmS = 0.0f;
    g_autoRun.task3MarkerDistanceCm = 0.0f;
    g_autoRun.task3RunStartedMs = 0U;
    g_autoRun.task3ElapsedMs = 0U;
    g_autoRun.task3TimingDone = 0U;
    g_autoRun.task3FinishConfirmTicks = 0U;
    g_autoRun.task3FinishSeen = 0U;
    UI_RequestRefresh();
}

static void App_AdjustTaskOneStopDistance(float deltaCm)
{
    float previous = g_autoRun.task1TargetDistanceCm;

    g_autoRun.task1TargetDistanceCm = App_ClampFloat(
        previous + deltaCm,
        APP_TASK1_STOP_DISTANCE_MIN_CM,
        APP_TASK1_STOP_DISTANCE_MAX_CM);
    if (g_autoRun.task1TargetDistanceCm != previous) {
        UI_RequestRefresh();
    }
}

static uint32_t App_ProcessAutoRunButton(uint32_t buttonEvents)
{
    if (App_IsTaskOne()) {
        uint32_t now = App_GetMillis();

        if ((g_autoRun.phase == APP_AUTO_PHASE_RUNNING) &&
            ((buttonEvents & BUTTON_EVENT_CENTER) != 0U)) {
            App_FreezeTaskOneTimer(now);
            SpeedPID_Enable(false);
            MotorDebug_Stop();
            g_autoRun.task1CommandSpeedCmS = 0.0f;
            g_autoRun.phase = APP_AUTO_PHASE_PAUSED;
            buttonEvents &= ~BUTTON_EVENT_CENTER;
            UI_RequestRefresh();
        } else if (g_autoRun.phase == APP_AUTO_PHASE_PAUSED) {
            if ((buttonEvents & BUTTON_EVENT_UP) != 0U) {
                App_ResetTaskOneReady();
                buttonEvents &= ~BUTTON_EVENT_UP;
            } else if ((buttonEvents & BUTTON_EVENT_DOWN) != 0U) {
                if (!App_StartTaskOneLine(now, false)) {
                    App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                }
                buttonEvents &= ~BUTTON_EVENT_DOWN;
                UI_RequestRefresh();
            }
        } else if (g_autoRun.phase == APP_AUTO_PHASE_IDLE) {
            if ((buttonEvents & BUTTON_EVENT_CENTER_LONG) != 0U) {
                SpeedPID_Enable(false);
                MotorDebug_Stop();
                g_autoRun.kind = APP_AUTO_RUN_NONE;
                g_autoRun.phase = APP_AUTO_PHASE_IDLE;
                UI_RequestRefresh();
            } else if ((buttonEvents & (BUTTON_EVENT_LEFT |
                                 BUTTON_EVENT_LEFT_REPEAT)) != 0U) {
                App_AdjustTaskOneStopDistance(
                    -APP_TASK1_STOP_DISTANCE_STEP_CM);
                buttonEvents &= ~(BUTTON_EVENT_LEFT |
                                  BUTTON_EVENT_LEFT_REPEAT);
            } else if ((buttonEvents & (BUTTON_EVENT_RIGHT |
                                        BUTTON_EVENT_RIGHT_REPEAT)) != 0U) {
                App_AdjustTaskOneStopDistance(
                    APP_TASK1_STOP_DISTANCE_STEP_CM);
                buttonEvents &= ~(BUTTON_EVENT_RIGHT |
                                  BUTTON_EVENT_RIGHT_REPEAT);
            } else if ((buttonEvents & BUTTON_EVENT_CENTER) != 0U) {
                if (!App_StartTaskOneLine(now, true)) {
                    App_StopAutoRun(APP_AUTO_PHASE_FAULT);
                }
                buttonEvents &= ~BUTTON_EVENT_CENTER;
                UI_RequestRefresh();
            }
        } else if (((g_autoRun.phase == APP_AUTO_PHASE_DONE) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_STOPPED) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_FAULT) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_SAVE_FAILED) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_IDLE)) &&
                   ((buttonEvents & BUTTON_EVENT_CENTER_LONG) != 0U)) {
            SpeedPID_Enable(false);
            MotorDebug_Stop();
            g_autoRun.kind = APP_AUTO_RUN_NONE;
            g_autoRun.phase = APP_AUTO_PHASE_IDLE;
            UI_RequestRefresh();
        }
        return buttonEvents;
    }

    if (App_IsTaskTwo()) {
        if ((g_autoRun.phase == APP_AUTO_PHASE_IDLE) &&
            ((buttonEvents & BUTTON_EVENT_CENTER) != 0U)) {
            if (!App_StartTaskTwoLine(App_GetMillis())) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            }
            buttonEvents &= ~BUTTON_EVENT_CENTER;
            UI_RequestRefresh();
        } else if ((g_autoRun.phase == APP_AUTO_PHASE_RUNNING) &&
                   ((buttonEvents & BUTTON_EVENT_CENTER) != 0U)) {
            App_StopAutoRun(APP_AUTO_PHASE_STOPPED);
            buttonEvents &= ~BUTTON_EVENT_CENTER;
        } else if (((g_autoRun.phase == APP_AUTO_PHASE_DONE) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_STOPPED) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_FAULT) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_SAVE_FAILED)) &&
                   ((buttonEvents & BUTTON_EVENT_CENTER_LONG) != 0U)) {
            SpeedPID_Enable(false);
            MotorDebug_Stop();
            g_autoRun.kind = APP_AUTO_RUN_NONE;
            g_autoRun.phase = APP_AUTO_PHASE_IDLE;
            buttonEvents &= ~BUTTON_EVENT_CENTER_LONG;
            UI_RequestRefresh();
        }
        return buttonEvents;
    }

    if (App_IsTaskThree()) {
        if ((g_autoRun.phase == APP_AUTO_PHASE_IDLE) &&
            ((buttonEvents & BUTTON_EVENT_CENTER) != 0U)) {
            if (!App_StartTaskThreeLap(App_GetMillis())) {
                App_StopAutoRun(APP_AUTO_PHASE_FAULT);
            }
            buttonEvents &= ~BUTTON_EVENT_CENTER;
            UI_RequestRefresh();
        } else if ((g_autoRun.phase == APP_AUTO_PHASE_RUNNING) &&
                   ((buttonEvents & BUTTON_EVENT_CENTER) != 0U)) {
            App_StopAutoRun(APP_AUTO_PHASE_STOPPED);
            buttonEvents &= ~BUTTON_EVENT_CENTER;
        } else if (((g_autoRun.phase == APP_AUTO_PHASE_DONE) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_STOPPED) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_FAULT) ||
                    (g_autoRun.phase == APP_AUTO_PHASE_SAVE_FAILED)) &&
                   ((buttonEvents & BUTTON_EVENT_CENTER_LONG) != 0U)) {
            SpeedPID_Enable(false);
            MotorDebug_Stop();
            g_autoRun.kind = APP_AUTO_RUN_NONE;
            g_autoRun.phase = APP_AUTO_PHASE_IDLE;
            buttonEvents &= ~BUTTON_EVENT_CENTER_LONG;
            UI_RequestRefresh();
        }
        return buttonEvents;
    }

    if (((buttonEvents & (BUTTON_EVENT_CENTER |
                          BUTTON_EVENT_CENTER_LONG)) != 0U) &&
        App_AutoRunIsBusy()) {
        App_StopAutoRun(APP_AUTO_PHASE_STOPPED);
        buttonEvents &= ~(BUTTON_EVENT_CENTER | BUTTON_EVENT_CENTER_LONG);
    }
    return buttonEvents;
}

static void App_MarkAutoRunSaveFailed(UiRunRequestTarget target,
                                      uint8_t testIndex)
{
    g_autoRun.kind = (target == UI_RUN_REQUEST_SQUARE) ?
        APP_AUTO_RUN_SQUARE :
        ((target == UI_RUN_REQUEST_CIRCLE) ?
            APP_AUTO_RUN_CIRCLE : APP_AUTO_RUN_TEST_SLOT);
    g_autoRun.testIndex = testIndex;
    g_autoRun.phase = APP_AUTO_PHASE_SAVE_FAILED;
    UI_ReportSaveResult(UI_SAVE_RESULT_FAILED);
    UI_RequestRefresh();
}

static void App_ProcessUiRunRequest(uint32_t now)
{
    UiRunRequest request;
    UiControlSettings uiSettings;
    AppPersistentSettings settings;
    bool applied;
    bool saved;

    if (UI_TakeRunRequest(&request) == 0U) {
        return;
    }

    UI_GetDraftControlSettings(&uiSettings);
    App_CopyUiSettingsToPersistent(&uiSettings, &settings);
    if (request.test_index < APP_TEST_SLOT_COUNT) {
        settings.selected_test_index = request.test_index;
    }

    if (App_AutoRunIsBusy()) {
        App_StopAutoRun(APP_AUTO_PHASE_STOPPED);
    }
    if (MotorDebug_IsRunning()) {
        MotorDebug_Stop();
    }
    if (App_MotorIoTestIsBusy()) {
        App_AbortMotorIoTest();
    }
    SpeedPID_Enable(false);

    applied = App_ApplySettings(&settings);
    saved = false;
    if (applied && (g_appStatus.flashInitialized != 0U)) {
        saved = SettingsStore_Save(&settings);
    }
    g_appStatus.flashSaveFailed = saved ? 0U : 1U;
    if (!saved) {
        App_MarkAutoRunSaveFailed(request.target, request.test_index);
        return;
    }

    UI_ReportSaveResult(UI_SAVE_RESULT_SUCCESS);
    g_autoRun.testIndex = request.test_index;
    g_autoRun.delayStartedMs = now;
    g_autoRun.squareSideIndex = 0U;
    g_autoRun.circleProgressCm = 0.0f;

    if (request.target == UI_RUN_REQUEST_TEST_SELECT) {
        g_autoRun.kind = APP_AUTO_RUN_TEST_SLOT;
        if (request.test_index == 0U) {
            App_ResetTaskOneReady();
        } else if (request.test_index == 1U) {
            App_ResetTaskTwoReady();
        } else if (request.test_index == 2U) {
            App_ResetTaskThreeReady();
        } else {
            g_autoRun.phase = APP_AUTO_PHASE_EMPTY;
        }
    } else if (request.target == UI_RUN_REQUEST_SQUARE) {
        g_autoRun.kind = APP_AUTO_RUN_SQUARE;
        g_autoRun.phase = APP_AUTO_PHASE_DELAY;
        g_autoRun.speedCmS = settings.square_speed_cm_s;
        g_autoRun.squareSideCm = settings.square_side_cm;
        g_autoRun.squareStep = APP_SQUARE_STEP_SIDE;
    } else {
        g_autoRun.kind = APP_AUTO_RUN_CIRCLE;
        g_autoRun.phase = APP_AUTO_PHASE_DELAY;
        g_autoRun.speedCmS = settings.circle_speed_cm_s;
        g_autoRun.circleRadiusCm = settings.circle_radius_cm;
    }
    UI_RequestRefresh();
}

static void App_ProcessWirelessUiRequest(uint32_t now)
{
    UiWirelessRequest request;

    if (UI_TakeWirelessRequest(&request) == 0U) {
        return;
    }

    if (request.type == UI_WIRELESS_REQUEST_ENTER_MODE) {
        if (App_AutoRunIsBusy()) {
            App_StopAutoRun(APP_AUTO_PHASE_STOPPED);
        }
        SpeedPID_Enable(false);
        MotorDebug_Stop();
        Motor_Stop();
        g_wirelessRemote.actionState = UI_WIRELESS_ACTION_IDLE;
        g_wirelessRemote.command = WIRELESS_COMMAND_NONE;
        g_wirelessRemote.value = 0U;
        g_wirelessRemote.running = 0U;
        g_wirelessRemote.lastCommandGeneration =
            g_wirelessSnapshot.commandGeneration;
        (void)WirelessTest_RequestMode(request.mode);
    } else if (request.type == UI_WIRELESS_REQUEST_SEND_COMMAND) {
        (void)WirelessTest_RequestCommand(
            request.command, request.value, now);
    } else if (request.type == UI_WIRELESS_REQUEST_EXIT_MODE) {
        App_StopWirelessRemote(UI_WIRELESS_ACTION_STOPPED);
        WirelessTest_LeaveMode();
    }
    UI_RequestRefresh();
}

static void App_ProcessWirelessReceivedCommand(void)
{
    if ((g_wirelessSnapshot.sessionActive != 0U) &&
        (g_wirelessSnapshot.activeMode == WIRELESS_TEST_MODE_RX)) {
        if (g_wirelessSnapshot.commandGeneration !=
            g_wirelessRemote.lastCommandGeneration) {
            g_wirelessRemote.lastCommandGeneration =
                g_wirelessSnapshot.commandGeneration;
            App_StartWirelessRemote(g_wirelessSnapshot.rxCommand,
                                    g_wirelessSnapshot.rxValue);
        } else if ((g_wirelessRemote.running != 0U) &&
                   (g_wirelessSnapshot.linkStatus != WIRELESS_LINK_OK)) {
            App_StopWirelessRemote(UI_WIRELESS_ACTION_LINK_LOST);
        }
    }
}

static uint32_t App_ProcessWirelessExitHold(
    uint32_t now, uint32_t buttonEvents)
{
    static uint32_t holdStartedMs;
    static uint8_t tracking;
    static uint8_t exitSent;
    UiPage page = UI_GetPage();

    if ((page != UI_PAGE_WIRELESS_RX) &&
        (page != UI_PAGE_WIRELESS_TX)) {
        tracking = 0U;
        exitSent = 0U;
        return buttonEvents;
    }

    buttonEvents &= ~BUTTON_EVENT_CENTER_LONG;
    if (Buttons_IsPressed(BUTTON_ID_CENTER)) {
        if (tracking == 0U) {
            tracking = 1U;
            exitSent = 0U;
            holdStartedMs = now;
        } else if ((exitSent == 0U) &&
                   App_Elapsed(now, holdStartedMs,
                               APP_NRF24_EXIT_HOLD_MS)) {
            exitSent = 1U;
            buttonEvents |= UI_EVENT_CENTER_EXIT;
        }
    } else {
        tracking = 0U;
        exitSent = 0U;
    }
    return buttonEvents;
}

int main(void)
{
    uint32_t lastButtonScanMs;
    UiModel uiModel = {0};
    AppPersistentSettings bootSettings;

    SYSCFG_DL_init();
    MaixCamUART_Init();
    BuzzerRGB_Init();
    g_appStatus.batteryMonitorInitialized =
        BatteryMonitor_Init() ? 1U : 0U;
    WirelessTest_Init(App_GetMillis());

    g_appStatus.systemRunning = 1U;
    g_appStatus.motorInitialized = Motor_Init() ? 1U : 0U;
    SpeedPID_Init();

    NVIC_SetPriority(ENCODER_INT_IRQN, 0U);
    g_appStatus.encoderConfigured = Encoder_Init() ? 1U : 0U;
    NVIC_EnableIRQ(NRF24_PINS_INT_IRQN);
    g_appStatus.buttonsInitialized = Buttons_Init() ? 1U : 0U;
    g_appStatus.lineSensorsInitialized = LineSensor_Init() ? 1U : 0U;
    MotorDebug_Init(g_appStatus.lineSensorsInitialized != 0U);

    App_FillDefaultSettings(&bootSettings);
    g_appStatus.flashInitialized = SettingsStore_Init() ? 1U : 0U;
    if ((g_appStatus.flashInitialized != 0U) &&
        SettingsStore_Load(&bootSettings)) {
        g_appStatus.flashRecordLoaded = 1U;
    }
    if (!App_ApplySettings(&bootSettings)) {
        App_FillDefaultSettings(&bootSettings);
        (void)App_ApplySettings(&bootSettings);
        g_appStatus.flashRecordLoaded = 0U;
    }

    OLED_Init();
    g_appStatus.oledInitialized = 1U;

    if (SysTick_Config(CPUCLK_FREQ / 1000U) != 0U) {
        /* This can only fail if the 24-bit SysTick reload is out of range. */
        g_appStatus.systemRunning = 0U;
    }
    NVIC_SetPriority(SysTick_IRQn, 2U);

    UI_Init(App_GetMillis());

    (void)App_InitializeImu();

    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(CONTROL_TIMER_INST);

    lastButtonScanMs = App_GetMillis();
    __enable_irq();

    for (;;) {
        uint32_t now = App_GetMillis();
        uint32_t buttonEvents = BUTTON_EVENT_NONE;

        App_ProcessControl();
        App_ProcessImu(now);
        App_ProcessMaixCamPacket(now);
        BatteryMonitor_Service(now);
        Buzzer_Service(now);
        BatteryMonitor_GetSnapshot(&g_batterySnapshot);
        g_appStatus.batteryReady = g_batterySnapshot.ready;
        g_appStatus.batteryLowVoltage =
            g_batterySnapshot.lowVoltageWarning;
        WirelessTest_Service(now);
        WirelessTest_GetSnapshot(now, &g_wirelessSnapshot);
        App_ProcessWirelessReceivedCommand();
        g_appStatus.wirelessInitialized =
            g_wirelessSnapshot.initialized;
        g_appStatus.wirelessConnected =
            g_wirelessSnapshot.connected;

        if (App_Elapsed(now, lastButtonScanMs, APP_BUTTON_SCAN_PERIOD_MS)) {
            lastButtonScanMs = now;
            Buttons_Scan10ms();
            App_UpdateButtonState();
            buttonEvents = Buttons_GetAndClearEvents();
        }

        buttonEvents = App_ProcessWirelessExitHold(now, buttonEvents);
        buttonEvents = App_ProcessAutoRunButton(buttonEvents);
        App_ProcessMotorDebugButton(buttonEvents);
        App_ProcessMotorIoTest(now, buttonEvents);
        App_BuildUiModel(&uiModel, now);
        UI_Service(now, buttonEvents, &uiModel);
        App_ProcessUiSaveRequest();
        App_ProcessUiRunRequest(now);
        App_ProcessWirelessUiRequest(now);

        __WFI();
    }
}

void SysTick_Handler(void)
{
    ++g_millis;
}

void CONTROL_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST) ==
        DL_TIMERG_IIDX_ZERO) {
        ++g_controlTicks;
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t iid;

    do {
        iid = (uint32_t)DL_GPIO_getPendingInterrupt(NRF24_PINS_PORT);
        if (iid == (uint32_t)NRF24_PINS_RADIO_IRQ_IIDX) {
            NRF24L01_NotifyIrqFromIsr();
        }
    } while (iid != (uint32_t)DL_GPIO_IIDX_NO_INTR);

    do {
        iid = (uint32_t)DL_GPIO_getPendingInterrupt(ENCODER_PORT);
        if (iid != (uint32_t)DL_GPIO_IIDX_NO_INTR) {
            Encoder_HandleGpioInterruptIID(iid);
        }
    } while (iid != (uint32_t)DL_GPIO_IIDX_NO_INTR);
}
