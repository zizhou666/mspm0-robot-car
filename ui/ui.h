/*
================================================================================
OLED 六页用户界面与参数编辑接口模块
================================================================================
【功能简介】
本文件定义启动 LOGO、系统主页、姿态页、编码器页、PID 参数页、电机调试页
和物理通道测试页的数据模型、事件位及非阻塞 UI 状态机接口。PID 参数页用
滚动列表编辑速度、编码器直行、12 路灰度巡线和 ICM42688 角度四套 PID。

================================================================================
【函数定义】
- UI_Init：初始化页面、参数草稿和非阻塞 LOGO 显示状态。
- UI_Service：处理按键、页面切换、参数编辑并分片刷新 OLED。
- UI_RequestRefresh：请求下一次可用时立即重绘。
- UI_TakeSavedControlSettings：取出中心键长按产生的一次保存请求。
- UI_GetDraftControlSettings：读取当前 OLED 参数草稿。
- UI_ReportSaveResult：把 Flash 保存成功/失败结果反馈给界面。
- UI_GetPage：读取当前页面编号。
- UI_IsLogoActive：查询启动 LOGO 是否仍在显示。

================================================================================
【使用说明】
1. OLED_Init 后调用 UI_Init，主循环应尽可能频繁调用 UI_Service。
2. PA18/PB21 单次翻页；五向上下选择、左右减/增，参数页方向键支持长按连发。
3. 中心键短按启停、长按保存，中心键不参与连发。
4. UiModel 是只读快照，UI 不直接访问中断变量或控制器内部状态。
5. LOGO 时间、刷新周期和参数调节范围在 user_config.h 中修改。
================================================================================
*/
#ifndef INTEGRATED_42688_ENCODER_UI_H_
#define INTEGRATED_42688_ENCODER_UI_H_

#include <stdint.h>

#include "motor.h"
#include "motor_debug.h"
#include "user_config.h"
#include "wireless_test.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * UI_Service() consumes one-shot, debounced press-edge events.  The button
 * driver may use the same bit layout directly or translate its own event mask.
 */
#define UI_EVENT_PAGE_PREV   (1U << 0)
#define UI_EVENT_PAGE_NEXT   (1U << 1)
#define UI_EVENT_UP          (1U << 2)
#define UI_EVENT_DOWN        (1U << 3)
#define UI_EVENT_LEFT        (1U << 4)
#define UI_EVENT_RIGHT       (1U << 5)
#define UI_EVENT_CENTER      (1U << 6)
#define UI_EVENT_CENTER_LONG (1U << 7)
#define UI_EVENT_UP_REPEAT    (1U << 8)
#define UI_EVENT_DOWN_REPEAT  (1U << 9)
#define UI_EVENT_LEFT_REPEAT  (1U << 10)
#define UI_EVENT_RIGHT_REPEAT (1U << 11)
#define UI_EVENT_CENTER_EXIT  (1U << 12)

typedef enum
{
    UI_PAGE_TEST_SELECT = 0,
    UI_PAGE_HOME,
    UI_PAGE_ICM42688,
    UI_PAGE_ENCODER_PID,
    UI_PAGE_PID_SETTINGS,
    UI_PAGE_MOTOR_DEBUG,
    UI_PAGE_MOTOR_IO_TEST,
    UI_PAGE_SQUARE_TEST,
    UI_PAGE_CIRCLE_TEST,
    UI_PAGE_WIRELESS_TEST,
    UI_PAGE_WIRELESS_RX,
    UI_PAGE_WIRELESS_TX,
    UI_PAGE_TASK1_RUN,
    UI_PAGE_COUNT
} UiPage;

typedef enum
{
    UI_MODULE_NOT_READY = 0,
    UI_MODULE_INITIALIZING,
    UI_MODULE_READY,
    UI_MODULE_SIGNAL_MISSING,
    UI_MODULE_DIRECTION_ERROR,
    UI_MODULE_FAILED
} UiModuleStatus;

typedef enum
{
    UI_RUN_STATUS_IDLE = 0,
    UI_RUN_STATUS_SAVING,
    UI_RUN_STATUS_DELAY,
    UI_RUN_STATUS_RUNNING,
    UI_RUN_STATUS_PAUSED,
    UI_RUN_STATUS_DONE,
    UI_RUN_STATUS_STOPPED,
    UI_RUN_STATUS_FAILED,
    UI_RUN_STATUS_EMPTY
} UiRunStatus;

typedef enum
{
    UI_RUN_REQUEST_TEST_SELECT = 0,
    UI_RUN_REQUEST_SQUARE,
    UI_RUN_REQUEST_CIRCLE
} UiRunRequestTarget;

typedef struct
{
    UiRunRequestTarget target;
    uint8_t test_index;
} UiRunRequest;

typedef enum
{
    UI_WIRELESS_REQUEST_ENTER_MODE = 0,
    UI_WIRELESS_REQUEST_SEND_COMMAND,
    UI_WIRELESS_REQUEST_EXIT_MODE
} UiWirelessRequestType;

typedef struct
{
    UiWirelessRequestType type;
    WirelessTestMode mode;
    WirelessRemoteCommand command;
    uint16_t value;
} UiWirelessRequest;

typedef enum
{
    UI_WIRELESS_ACTION_IDLE = 0,
    UI_WIRELESS_ACTION_RUNNING,
    UI_WIRELESS_ACTION_DONE,
    UI_WIRELESS_ACTION_STOPPED,
    UI_WIRELESS_ACTION_LINK_LOST,
    UI_WIRELESS_ACTION_SENSOR_FAILED
} UiWirelessActionState;

/*
 * Snapshot supplied by the main loop.  A zero encoder count is valid while
 * UI_MODULE_READY and is therefore displayed as a stationary encoder, not as
 * an initialization failure.
 */
typedef struct
{
    uint8_t system_running;
    uint8_t oled_initialized;
    uint8_t motor_enabled;
    UiModuleStatus imu_status;
    uint8_t imu_last_status;
    uint8_t imu_i2c_address;
    uint8_t imu_who_am_i;
    uint32_t imu_i2c_error_count;
    uint8_t imu_i2c_phase;
    uint8_t imu_i2c_line_state;
    uint8_t imu_calibration_percent;
    uint16_t imu_calibration_rejected;
    uint32_t imu_ready_elapsed_ms;
    UiModuleStatus encoder_status;
    uint8_t encoder_direction_error_mask;
    uint8_t encoder_signal_missing_mask;
    UiModuleStatus flash_status;
    UiModuleStatus line_sensor_status;
    uint32_t battery_voltage_mv;
    uint8_t battery_ready;
    uint8_t battery_low_voltage;
    uint8_t wireless_initialized;
    uint8_t wireless_session_active;
    uint8_t wireless_connected;
    uint8_t wireless_sequence_valid;
    uint8_t wireless_rpd_detected;
    uint8_t wireless_ce_high;
    WirelessTestMode wireless_active_mode;
    WirelessTestState wireless_state;
    WirelessTestTxFailure wireless_last_tx_failure;
    WirelessLinkStatus wireless_link_status;
    WirelessSendStatus wireless_send_status;
    WirelessRemoteCommand wireless_tx_command;
    WirelessRemoteCommand wireless_rx_command;
    uint16_t wireless_tx_value;
    uint16_t wireless_rx_value;
    uint8_t wireless_retry_attempt;
    uint32_t wireless_command_generation;
    UiWirelessActionState wireless_action_state;
    WirelessRemoteCommand wireless_action_command;
    uint16_t wireless_action_value;
    uint32_t wireless_rx_event_count;
    uint32_t wireless_rx_count;
    uint32_t wireless_tx_success_count;
    uint32_t wireless_tx_failure_count;
    uint16_t wireless_last_sequence;
    uint32_t wireless_last_activity_age_ms;
    uint8_t wireless_data_prefix[4];

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float encoder_speed_cm_s;
    float encoder_distance_cm;
    float target_speed_cm_s;
    float target_distance_cm;
    int32_t encoder_position_count;
    int32_t left_encoder_position_count;
    int32_t right_encoder_position_count;
    int32_t encoder_delta_count;
    float pid_output;
    float pwm_duty_percent;
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

    MotorDebugMode motor_debug_mode;
    MotorDebugState motor_debug_state;
    MotorDebugStopReason motor_debug_stop_reason;
    float motor_debug_travel_cm;
    float line_error;
    uint8_t line_active_count;
    uint8_t line_lost;

    MotorIoTestCommand motor_io_test_command;
    MotorIoTestPhase motor_io_test_phase;
    uint8_t motor_io_test_active;
    int32_t motor_io_test_left_delta;
    int32_t motor_io_test_right_delta;

    uint8_t selected_test_index;
    UiRunStatus test_select_status;
    uint32_t task1_elapsed_ms;
    float task1_distance_cm;
    float task1_target_distance_cm;
    float task1_command_speed_cm_s;
    uint32_t task2_elapsed_ms;
    float task2_distance_cm;
    float task2_target_distance_cm;
    float task2_command_speed_cm_s;
    uint32_t task3_elapsed_ms;
    float task3_distance_cm;
    float task3_command_speed_cm_s;
    uint8_t task3_finish_armed;
    uint8_t task3_finish_seen;
    UiRunStatus square_test_status;
    UiRunStatus circle_test_status;
    uint8_t square_side_index;
    float square_progress_cm;
    float circle_progress_cm;
    float square_speed_cm_s;
    float square_side_cm;
    float circle_speed_cm_s;
    float circle_radius_cm;
} UiModel;

typedef struct
{
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
    MotorDebugMode motor_mode;
    uint8_t selected_test_index;
    float square_speed_cm_s;
    float square_side_cm;
    float circle_speed_cm_s;
    float circle_radius_cm;
} UiControlSettings;

typedef enum
{
    UI_SAVE_RESULT_NONE = 0,
    UI_SAVE_RESULT_PENDING,
    UI_SAVE_RESULT_SUCCESS,
    UI_SAVE_RESULT_FAILED
} UiSaveResult;

/* Call after OLED_Init().  The logo transfer is queued without a delay. */
void UI_Init(uint32_t now_ms);

/*
 * Call frequently from the non-blocking main loop.  It advances one OLED
 * transfer chunk, handles at most one page transition, and submits a new frame
 * only after the previous frame has finished.
 */
void UI_Service(uint32_t now_ms, uint32_t button_events,
                const UiModel *model);

/* Queue a redraw (for example after an out-of-band state change). */
void UI_RequestRefresh(void);

/* Returns one persistent-save request produced by a center-key long press. */
uint8_t UI_TakeSavedControlSettings(UiControlSettings *settings);
void UI_GetDraftControlSettings(UiControlSettings *settings);
void UI_ReportSaveResult(UiSaveResult result);
uint8_t UI_TakeRunRequest(UiRunRequest *request);
void UI_AdvanceSelectedTest(void);
uint8_t UI_TakeWirelessRequest(UiWirelessRequest *request);

UiPage UI_GetPage(void);
uint8_t UI_IsLogoActive(void);

#ifdef __cplusplus
}
#endif

#endif
