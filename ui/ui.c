/*
================================================================================
OLED 六页界面、按键编辑与保存请求实现模块
================================================================================
【功能简介】
本文件实现启动 LOGO 和六个数据/调试页面。PA18/PB21 负责前后翻页，五向键
上下选择、左右调整；第 3、4 页支持方向键长按连续选择或增减，选中项以“*”
标识。中心键短按用于电机调试启停，长按产生一次参数保存请求。第 4 页把四套
PID 组织为带标题的滚动列表；绘制和发送采用非阻塞帧提交机制。

================================================================================
【函数定义】
- ui_elapsed：使用回绕安全方式判断 UI 时间间隔。
- ui_clampf：限制 OLED 参数草稿的调整范围。
- ui_status_text：把模块状态枚举转换为短文本。
- ui_show_float：显示浮点值并处理 NaN 和越界。
- ui_draw_page_marker：绘制当前页/总页数。
- ui_save_status_text：返回参数未改、待保存、成功或失败提示。
- ui_pid_group_title/ui_pid_value：读取 PID 分组标题和当前参数值。
- ui_update_pid_scroll：保证当前选中参数始终位于第 4 页可见区域。
- ui_draw_home：绘制系统、IMU、编码器速度和里程主页。
- ui_draw_icm42688：绘制 Roll/Pitch/Yaw 或初始化失败信息。
- ui_draw_encoder_pid：绘制左右计数、速度、路程和目标参数。
- ui_draw_pid_settings：滚动绘制速度、直行、巡线和角度 PID。
- ui_motor_mode_text/ui_motor_state_text/ui_motor_fault_text：调试状态文本转换。
- ui_draw_motor_debug：绘制距离/巡线模式、目标、进度和故障。
- ui_motor_io_command_text/ui_motor_io_phase_text：物理通道命令和阶段文本。
- ui_draw_motor_io_test：绘制 A/B 通道点动测试及左右编码器增量。
- ui_render：清空帧缓冲并分派当前页面绘制函数。
- ui_sync_draft：首次进入时从主程序模型同步参数草稿。
- ui_adjust_selected：按当前页面和选择项增减参数。
- ui_handle_events：处理翻页、选择、增减、短按和长按事件。
- UI_Init：初始化页面、草稿、LOGO 和刷新状态。
- UI_Service：推进 OLED 发送、处理事件并按需提交下一帧。
- UI_RequestRefresh：标记一次立即重绘请求。
- UI_TakeSavedControlSettings：取出并清除一次待保存参数。
- UI_GetDraftControlSettings：复制当前未必已保存的参数草稿。
- UI_ReportSaveResult：记录 Flash 保存结果并触发刷新。
- UI_GetPage：返回当前页面。
- UI_IsLogoActive：返回启动 LOGO 是否有效。

================================================================================
【使用说明】
1. 主循环传入同一时刻构建的 UiModel，避免直接读取异步共享变量。
2. LOGO 期间忽略翻页；结束后自动进入主页且不使用 2 秒阻塞延时。
3. 长按只提交保存请求，实际 Flash 写入由 main.c 在主循环处理。
4. 调节范围、步长、LOGO 时间和刷新周期集中在 user_config.h。
================================================================================
*/
#include "ui.h"

#include "app_config.h"
#include "icm42688.h"
#include "oled.h"

#include <stddef.h>

typedef enum
{
    UI_WIRELESS_MENU_RX = 0,
    UI_WIRELESS_MENU_TX,
    UI_WIRELESS_MENU_DISTANCE,
    UI_WIRELESS_MENU_ANGLE,
    UI_WIRELESS_MENU_STEP,
    UI_WIRELESS_MENU_COUNT
} UiWirelessMenuItem;

static UiPage ui_page = UI_PAGE_TEST_SELECT;
static uint8_t ui_logo_active;
static uint8_t ui_logo_transfer_complete;
static uint8_t ui_logo_frame;
static uint8_t ui_logo_frame_phase;
static uint8_t ui_render_pending;
static uint8_t ui_draft_initialized;
static uint8_t ui_test_selection;
static uint8_t ui_square_selection;
static uint8_t ui_circle_selection;
static uint8_t ui_encoder_selection;
static uint8_t ui_pid_selection;
static uint8_t ui_pid_scroll_top;
static uint8_t ui_settings_dirty;
static uint8_t ui_save_pending;
static uint8_t ui_run_request_pending;
static uint8_t ui_wireless_request_pending;
static uint8_t ui_motor_running;
static uint8_t ui_square_busy;
static uint8_t ui_circle_busy;
static UiSaveResult ui_save_result;
static uint32_t ui_logo_visible_since_ms;
static uint32_t ui_logo_last_frame_ms;
static uint32_t ui_last_submit_ms;
static UiControlSettings ui_draft;
static UiControlSettings ui_saved_settings;
static UiRunRequest ui_run_request;
static UiWirelessRequest ui_wireless_request;
static UiWirelessMenuItem ui_wireless_menu_selection;
static uint16_t ui_wireless_distance_cm;
static uint16_t ui_wireless_angle_deg;
static uint16_t ui_wireless_step;

#define UI_PID_GROUP_COUNT         (4U)
#define UI_PID_PARAMETERS_PER_GROUP (3U)
#define UI_TEST_SELECTION_COUNT    APP_TEST_SLOT_COUNT
#define UI_PID_SELECTION_COUNT     \
    (UI_PID_GROUP_COUNT * UI_PID_PARAMETERS_PER_GROUP)
#define UI_PID_LOGICAL_ROW_COUNT   (16U)
#define UI_PID_VISIBLE_ROWS        (6U)
#define UI_LOGO_FRAME_COUNT        (30U)
#define UI_LOGO_FRAME_PHASE_COUNT  (3U)

static uint8_t ui_elapsed(uint32_t now_ms, uint32_t since_ms,
                          uint32_t interval_ms)
{
    return (uint8_t)(((uint32_t)(now_ms - since_ms) >= interval_ms) ? 1U : 0U);
}

static float ui_clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static const char *ui_status_text(UiModuleStatus status)
{
    switch (status)
    {
        case UI_MODULE_INITIALIZING:
            return "STARTING";
        case UI_MODULE_READY:
            return "READY";
        case UI_MODULE_SIGNAL_MISSING:
            return "NO SIGNAL";
        case UI_MODULE_DIRECTION_ERROR:
            return "DIR ERROR";
        case UI_MODULE_FAILED:
            return "FAILED";
        case UI_MODULE_NOT_READY:
        default:
            return "NOT READY";
    }
}

static const char *ui_imu_error_text(uint8_t status)
{
    switch ((ICM42688_Status_t)status)
    {
        case ICM42688_STATUS_OK:
            return "OK";
        case ICM42688_STATUS_ERROR_ARGUMENT:
            return "ARGUMENT";
        case ICM42688_STATUS_ERROR_SOFT_RESET:
            return "BUS/RESET";
        case ICM42688_STATUS_ERROR_WHO_READ:
            return "WHO READ";
        case ICM42688_STATUS_ERROR_WHO_VALUE:
            return "WHO VALUE";
        case ICM42688_STATUS_ERROR_BANK_SELECT:
            return "BANK SELECT";
        case ICM42688_STATUS_ERROR_GYRO_CONFIG:
            return "GYRO CFG";
        case ICM42688_STATUS_ERROR_ACCEL_CONFIG:
            return "ACCEL CFG";
        case ICM42688_STATUS_ERROR_FILTER_CONFIG:
            return "FILTER CFG";
        case ICM42688_STATUS_ERROR_POWER_CONFIG:
            return "POWER CFG";
        case ICM42688_STATUS_ERROR_POWER_VERIFY:
            return "POWER VERIFY";
        case ICM42688_STATUS_ERROR_CALIBRATION:
            return "CALIBRATION";
        case ICM42688_STATUS_ERROR_FIRST_READ:
            return "FIRST READ";
        case ICM42688_STATUS_ERROR_DATA_READ:
            return "DATA READ";
        case ICM42688_STATUS_ERROR_NOT_READY:
        default:
            return "NOT READY";
    }
}

static const char *ui_imu_i2c_phase_text(uint8_t phase)
{
    switch ((ICM42688_I2CPhase_t)phase)
    {
        case ICM42688_I2C_PHASE_RECOVER:
            return "RECOV";
        case ICM42688_I2C_PHASE_START:
            return "START";
        case ICM42688_I2C_PHASE_ADDRESS_WRITE:
            return "ADDR-W";
        case ICM42688_I2C_PHASE_REGISTER:
            return "REG";
        case ICM42688_I2C_PHASE_DATA:
            return "DATA";
        case ICM42688_I2C_PHASE_RESTART:
            return "RSTART";
        case ICM42688_I2C_PHASE_ADDRESS_READ:
            return "ADDR-R";
        case ICM42688_I2C_PHASE_READ_DATA:
            return "READ";
        case ICM42688_I2C_PHASE_STOP:
            return "STOP";
        case ICM42688_I2C_PHASE_NONE:
        default:
            return "NONE";
    }
}

static char ui_hex_digit(uint8_t value)
{
    value &= 0x0FU;
    return (value < 10U) ? (char)('0' + value) :
                           (char)('A' + value - 10U);
}

static void ui_show_hex_byte(uint8_t x, uint8_t page, uint8_t value)
{
    OLED_ShowString6x8(x, page, "0x");
    OLED_ShowChar6x8((uint8_t)(x + 12U), page, ui_hex_digit(value >> 4U));
    OLED_ShowChar6x8((uint8_t)(x + 18U), page, ui_hex_digit(value));
}

static void ui_show_uint_fixed(
    uint8_t x, uint8_t page, uint32_t value, uint8_t digits)
{
    char text[11];
    uint32_t divisor = 1U;

    if (digits > 10U)
    {
        digits = 10U;
    }
    for (uint8_t i = 1U; i < digits; ++i)
    {
        divisor *= 10U;
    }
    for (uint8_t i = 0U; i < digits; ++i)
    {
        text[i] = (char)('0' + ((value / divisor) % 10U));
        if (divisor > 1U)
        {
            divisor /= 10U;
        }
    }
    text[digits] = '\0';
    OLED_ShowString6x8(x, page, text);
}

static void ui_format_elapsed_ms_3(char *text, uint32_t elapsed_ms)
{
    char reverse_seconds[6];
    uint32_t seconds;
    uint32_t milliseconds;
    uint8_t index = 0U;
    uint8_t digit_count = 0U;

    elapsed_ms %= 1000000000U;

    seconds = elapsed_ms / 1000U;
    milliseconds = elapsed_ms % 1000U;

    if (seconds < 10U)
    {
        text[index++] = '0';
    }

    do
    {
        reverse_seconds[digit_count] =
            (char)('0' + (seconds % 10U));
        digit_count++;
        seconds /= 10U;
    } while ((seconds != 0U) &&
             (digit_count < (uint8_t)sizeof(reverse_seconds)));

    while (digit_count > 0U)
    {
        digit_count--;
        text[index] = reverse_seconds[digit_count];
        index++;
    }

    text[index++] = '.';
    text[index++] = (char)('0' + (milliseconds / 100U));
    text[index++] = (char)('0' + ((milliseconds / 10U) % 10U));
    text[index++] = (char)('0' + (milliseconds % 10U));
    text[index++] = 's';
    text[index] = '\0';
}

static void ui_show_elapsed_ms_3(
    uint8_t x, uint8_t page, uint32_t elapsed_ms)
{
    char text[12];

    ui_format_elapsed_ms_3(text, elapsed_ms);
    OLED_ShowString6x8(x, page, text);
}

static void ui_show_elapsed_ms_3_large(
    uint8_t page, uint32_t elapsed_ms)
{
    char text[12];
    uint8_t length = 0U;
    uint8_t x;

    ui_format_elapsed_ms_3(text, elapsed_ms);
    while (text[length] != '\0')
    {
        ++length;
    }
    x = (length < 16U) ? (uint8_t)((128U - (length * 8U)) / 2U) : 0U;
    OLED_ShowString8x16(x, page, text);
}

static void ui_show_float(uint8_t x, uint8_t page, float value,
                          uint8_t decimals, float magnitude_limit)
{
    if (value != value)
    {
        OLED_ShowString6x8(x, page, "--");
    }
    else if ((value > magnitude_limit) || (value < -magnitude_limit))
    {
        OLED_ShowString6x8(x, page, "OVF");
    }
    else
    {
        OLED_ShowFloat6x8(x, page, value, decimals);
    }
}

static void ui_draw_page_marker(uint8_t page_number)
{
    if (page_number >= 10U)
    {
        OLED_ShowString6x8(96U, 0U, "10/10");
    }
    else
    {
        OLED_ShowChar6x8(102U, 0U, (char)('0' + page_number));
        OLED_ShowString6x8(108U, 0U, "/10");
    }
}

static const char *ui_save_status_text(void)
{
    switch (ui_save_result)
    {
        case UI_SAVE_RESULT_PENDING:
            return "Saving...";
        case UI_SAVE_RESULT_SUCCESS:
            return "Saved to Flash";
        case UI_SAVE_RESULT_FAILED:
            return "Flash Save Failed";
        case UI_SAVE_RESULT_NONE:
        default:
            return (ui_settings_dirty != 0U) ?
                "Unsaved changes" : "Flash settings";
    }
}

static const char *ui_run_status_text(UiRunStatus status)
{
    switch (status)
    {
        case UI_RUN_STATUS_SAVING:
            return "Saving...";
        case UI_RUN_STATUS_DELAY:
            return "Delay 1s";
        case UI_RUN_STATUS_RUNNING:
            return "RUNNING";
        case UI_RUN_STATUS_PAUSED:
            return "PAUSED";
        case UI_RUN_STATUS_DONE:
            return "DONE";
        case UI_RUN_STATUS_STOPPED:
            return "STOPPED";
        case UI_RUN_STATUS_FAILED:
            return "FAILED";
        case UI_RUN_STATUS_EMPTY:
            return "Empty slot";
        case UI_RUN_STATUS_IDLE:
        default:
            return "Ctr:Run";
    }
}

static const char *ui_pid_group_title(uint8_t group)
{
    switch (group)
    {
        case 1U:
            return "[Straight PID]";
        case 2U:
            return "[Line PID]";
        case 3U:
            return "[Angle PID]";
        case 0U:
        default:
            return "[Speed PID]";
    }
}

static float ui_pid_value(uint8_t group, uint8_t parameter)
{
    if (group == 0U)
    {
        return (parameter == 0U) ? ui_draft.pid_kp :
            ((parameter == 1U) ? ui_draft.pid_ki : ui_draft.pid_kd);
    }
    if (group == 1U)
    {
        return (parameter == 0U) ? ui_draft.straight_pid_kp :
            ((parameter == 1U) ? ui_draft.straight_pid_ki :
                                 ui_draft.straight_pid_kd);
    }
    if (group == 2U)
    {
        return (parameter == 0U) ? ui_draft.line_pid_kp :
            ((parameter == 1U) ? ui_draft.line_pid_ki :
                                 ui_draft.line_pid_kd);
    }
    return (parameter == 0U) ? ui_draft.angle_pid_kp :
        ((parameter == 1U) ? ui_draft.angle_pid_ki :
                             ui_draft.angle_pid_kd);
}

static void ui_update_pid_scroll(void)
{
    uint8_t group = ui_pid_selection / UI_PID_PARAMETERS_PER_GROUP;
    uint8_t parameter = ui_pid_selection % UI_PID_PARAMETERS_PER_GROUP;
    uint8_t selectedRow = (uint8_t)(group * 4U + 1U + parameter);
    uint8_t maximumTop =
        UI_PID_LOGICAL_ROW_COUNT - UI_PID_VISIBLE_ROWS;

    if (selectedRow < ui_pid_scroll_top)
    {
        ui_pid_scroll_top = selectedRow;
    }
    else if (selectedRow >=
             (uint8_t)(ui_pid_scroll_top + UI_PID_VISIBLE_ROWS))
    {
        ui_pid_scroll_top =
            (uint8_t)(selectedRow - UI_PID_VISIBLE_ROWS + 1U);
    }
    if (ui_pid_scroll_top > maximumTop)
    {
        ui_pid_scroll_top = maximumTop;
    }
}

static void ui_draw_test_select(const UiModel *model)
{
    static const char *const task_labels[UI_TEST_SELECTION_COUNT] = {
        "Task1 LAP630",
        "Task2 B+40 ENC",
        "Task3 LAP BALL",
        "Task4 EMPTY"
    };
    uint8_t index;

    OLED_ClearBuffer();

    for (index = 0U; index < UI_TEST_SELECTION_COUNT; ++index)
    {
        uint8_t row = index;

        OLED_ShowChar6x8(0U, row,
                         (index == ui_test_selection) ? '*' : ' ');
        OLED_ShowString6x8(6U, row, task_labels[index]);
    }

    if ((model->test_select_status != UI_RUN_STATUS_IDLE) &&
        (model->selected_test_index == ui_test_selection))
    {
        OLED_ShowString6x8(0U, 7U,
                           ui_run_status_text(model->test_select_status));
    }
    else
    {
        OLED_ShowString6x8(0U, 7U, "Ctr:Start");
    }
}

static void ui_draw_task1_run(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "TASK1 LAP");
    OLED_ShowString6x8(60U, 0U,
                       ui_run_status_text(model->test_select_status));
    ui_show_elapsed_ms_3_large(1U, model->task1_elapsed_ms);
    OLED_ShowString6x8(0U, 3U, "Dist:");
    ui_show_float(30U, 3U, model->task1_distance_cm, 1U, 9999.9f);
    OLED_ShowChar6x8(66U, 3U, '/');
    OLED_ShowInt6x8(72U, 3U, (int32_t)model->task1_target_distance_cm);
    OLED_ShowString6x8(102U, 3U, "cm");
    OLED_ShowString6x8(0U, 4U, "Speed:");
    ui_show_float(36U, 4U, model->task1_command_speed_cm_s,
                  1U, 99.9f);
    OLED_ShowString6x8(66U, 4U, "/35cm/s");
    OLED_ShowString6x8(0U, 5U, "Gray:");
    OLED_ShowInt6x8(30U, 5U, (int32_t)model->line_active_count);
    OLED_ShowString6x8(60U, 5U, "Need:5");
    OLED_ShowString6x8(0U, 6U, "Gate:");
    OLED_ShowString6x8(30U, 6U,
                       (model->task1_distance_cm >=
                        model->task1_target_distance_cm) ?
                           "READY" : "WAIT");

    if (model->test_select_status == UI_RUN_STATUS_RUNNING)
    {
        OLED_ShowString6x8(0U, 7U, "Ctr:Pause");
    }
    else if (model->test_select_status == UI_RUN_STATUS_PAUSED)
    {
        OLED_ShowString6x8(0U, 7U, "UP:Reset DN:Continue");
    }
    else if ((model->test_select_status == UI_RUN_STATUS_DONE) ||
             (model->test_select_status == UI_RUN_STATUS_FAILED) ||
             (model->test_select_status == UI_RUN_STATUS_STOPPED))
    {
        OLED_ShowString6x8(0U, 7U, "Hold C 2s:Back");
    }
    else
    {
        OLED_ShowString6x8(0U, 7U, "L/R:Set C:Start");
    }
}

static void ui_draw_task2_run(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "TASK2 A-B");
    OLED_ShowString6x8(60U, 0U,
                       ui_run_status_text(model->test_select_status));
    ui_show_elapsed_ms_3_large(1U, model->task2_elapsed_ms);
    OLED_ShowString6x8(0U, 3U, "Dist:");
    ui_show_float(30U, 3U, model->task2_distance_cm, 1U, 999.9f);
    OLED_ShowChar6x8(66U, 3U, '/');
    OLED_ShowInt6x8(72U, 3U,
                    (int32_t)model->task2_target_distance_cm);
    OLED_ShowString6x8(102U, 3U, "cm");
    OLED_ShowString6x8(0U, 4U, "Speed:");
    ui_show_float(36U, 4U, model->task2_command_speed_cm_s,
                  1U, 99.9f);
    OLED_ShowString6x8(72U, 4U, "cm/s");
    OLED_ShowString6x8(0U, 5U, "LIN 0-25-0 2.5s");
    OLED_ShowString6x8(0U, 6U, "B+40cm ENCODER");

    if (model->test_select_status == UI_RUN_STATUS_RUNNING)
    {
        OLED_ShowString6x8(0U, 7U, "Ctr:Stop");
    }
    else if ((model->test_select_status == UI_RUN_STATUS_DONE) ||
             (model->test_select_status == UI_RUN_STATUS_FAILED) ||
             (model->test_select_status == UI_RUN_STATUS_STOPPED))
    {
        OLED_ShowString6x8(0U, 7U, "Hold C 2s:Back");
    }
    else
    {
        OLED_ShowString6x8(0U, 7U, "Ctr:Start");
    }
}

static void ui_draw_task3_run(const UiModel *model)
{
    const char *finishText = "WAIT";

    if (model->task3_finish_seen != 0U)
    {
        finishText = "SEEN";
    }
    else if (model->task3_finish_armed != 0U)
    {
        finishText = "ARMED";
    }

    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "TASK3 LAP");
    OLED_ShowString6x8(60U, 0U,
                       ui_run_status_text(model->test_select_status));
    ui_show_elapsed_ms_3_large(1U, model->task3_elapsed_ms);
    OLED_ShowString6x8(0U, 3U, "Dist:");
    ui_show_float(30U, 3U, model->task3_distance_cm, 1U, 999.9f);
    OLED_ShowString6x8(78U, 3U, "cm");
    OLED_ShowString6x8(0U, 4U, "Speed:");
    ui_show_float(36U, 4U, model->task3_command_speed_cm_s,
                  1U, 99.9f);
    OLED_ShowString6x8(72U, 4U, "/24cm/s");
    OLED_ShowString6x8(0U, 5U, "Finish:");
    OLED_ShowString6x8(42U, 5U, finishText);
    OLED_ShowString6x8(0U, 6U, "Smooth IN + OUT");

    if (model->test_select_status == UI_RUN_STATUS_RUNNING)
    {
        OLED_ShowString6x8(0U, 7U, "Ctr:Stop");
    }
    else if ((model->test_select_status == UI_RUN_STATUS_DONE) ||
             (model->test_select_status == UI_RUN_STATUS_FAILED) ||
             (model->test_select_status == UI_RUN_STATUS_STOPPED))
    {
        OLED_ShowString6x8(0U, 7U, "Hold C 2s:Back");
    }
    else
    {
        OLED_ShowString6x8(0U, 7U, "Ctr:Start");
    }
}

static void ui_draw_home(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "System Home");
    ui_draw_page_marker(2U);

    OLED_ShowString6x8(0U, 1U, "Bat:");
    if (model->battery_ready != 0U)
    {
        ui_show_float(24U, 1U,
                      ((float)model->battery_voltage_mv) * 0.001f,
                      2U, 99.99f);
    }
    else
    {
        OLED_ShowString6x8(24U, 1U, "--");
    }
    OLED_ShowChar6x8(60U, 1U, 'V');
    if (model->battery_low_voltage != 0U)
    {
        OLED_ShowString6x8(78U, 1U, "LOW");
    }
    else
    {
        OLED_ShowString6x8(72U, 1U, "F:");
        OLED_ShowString6x8(84U, 1U, ui_status_text(model->flash_status));
    }
    OLED_ShowString6x8(0U, 2U, "ICM42688:");
    OLED_ShowString6x8(60U, 2U, ui_status_text(model->imu_status));
    OLED_ShowString6x8(0U, 3U, "Encoder:");
    OLED_ShowString6x8(60U, 3U, ui_status_text(model->encoder_status));

    OLED_ShowString6x8(0U, 4U, "Distance:");
    ui_show_float(60U, 4U, model->encoder_distance_cm, 1U, 999999.9f);
    OLED_ShowString6x8(114U, 4U, "cm");
    OLED_ShowString6x8(0U, 5U, "Speed:");
    ui_show_float(42U, 5U, model->encoder_speed_cm_s, 1U, 9999.9f);
    OLED_ShowString6x8(96U, 5U, "cm/s");
    OLED_ShowString6x8(0U, 6U, "Target:");
    ui_show_float(48U, 6U, model->target_speed_cm_s, 1U, 9999.9f);
    OLED_ShowString6x8(0U, 7U, "Motor  :");
    OLED_ShowString6x8(60U, 7U,
                       (model->motor_enabled != 0U) ? "ENABLED" : "SAFE OFF");
}

static void ui_draw_icm42688(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "ICM42688");
    ui_draw_page_marker(3U);

    if (model->imu_status == UI_MODULE_FAILED)
    {
        OLED_ShowString6x8(0U, 2U, "Connect Failed");
        OLED_ShowString6x8(0U, 3U, "Err:");
        OLED_ShowString6x8(24U, 3U,
                           ui_imu_error_text(model->imu_last_status));
        OLED_ShowString6x8(0U, 4U, "Addr:");
        ui_show_hex_byte(30U, 4U, model->imu_i2c_address);
        OLED_ShowString6x8(66U, 4U, "WHO:");
        if ((model->imu_last_status ==
             (uint8_t)ICM42688_STATUS_ERROR_WHO_READ) ||
            (model->imu_last_status ==
             (uint8_t)ICM42688_STATUS_ERROR_SOFT_RESET))
        {
            OLED_ShowString6x8(90U, 4U, "--");
        }
        else
        {
            ui_show_hex_byte(90U, 4U, model->imu_who_am_i);
        }
        OLED_ShowString6x8(0U, 5U, "I2C Err:");
        OLED_ShowInt6x8(48U, 5U, (int32_t)model->imu_i2c_error_count);
        OLED_ShowString6x8(0U, 6U, "Step:");
        OLED_ShowString6x8(30U, 6U,
                           ui_imu_i2c_phase_text(model->imu_i2c_phase));
        OLED_ShowString6x8(72U, 6U, "S");
        OLED_ShowChar6x8(78U, 6U,
            ((model->imu_i2c_line_state & 0x01U) != 0U) ? '1' : '0');
        OLED_ShowString6x8(90U, 6U, "D");
        OLED_ShowChar6x8(96U, 6U,
            ((model->imu_i2c_line_state & 0x02U) != 0U) ? '1' : '0');
        OLED_ShowString6x8(0U, 7U, "Auto Retry...");
        return;
    }

    if (model->imu_status != UI_MODULE_READY)
    {
        OLED_ShowString6x8(0U, 2U,
                           (model->imu_status == UI_MODULE_INITIALIZING) ?
                               "Calibrating..." : "Not Ready");
        OLED_ShowString6x8(0U, 3U, "Progress:");
        OLED_ShowInt6x8(54U, 3U,
                        (int32_t)model->imu_calibration_percent);
        OLED_ShowChar6x8(78U, 3U, '%');
        OLED_ShowString6x8(0U, 4U, "Reject:");
        OLED_ShowInt6x8(48U, 4U,
                        (int32_t)model->imu_calibration_rejected);
        OLED_ShowString6x8(0U, 5U, "I2C Err:");
        OLED_ShowInt6x8(48U, 5U,
                        (int32_t)model->imu_i2c_error_count);
        return;
    }

    OLED_ShowString6x8(0U, 2U, "Roll :");
    ui_show_float(48U, 2U, model->roll_deg, 2U, 999.99f);
    OLED_ShowString6x8(0U, 3U, "Pitch:");
    ui_show_float(48U, 3U, model->pitch_deg, 2U, 999.99f);
    OLED_ShowString6x8(0U, 4U, "Yaw  :");
    ui_show_float(48U, 4U, model->yaw_deg, 2U, 999.99f);
    OLED_ShowString6x8(0U, 5U, "Ready:");
    ui_show_elapsed_ms_3(42U, 5U, model->imu_ready_elapsed_ms);
}

static void ui_draw_encoder_pid(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "Encoder Ctrl");
    ui_draw_page_marker(4U);

    if (model->encoder_status == UI_MODULE_FAILED)
    {
        OLED_ShowString6x8(0U, 2U, "Encoder Init Failed");
        return;
    }
    if (model->encoder_status == UI_MODULE_SIGNAL_MISSING)
    {
        if (model->encoder_signal_missing_mask == 1U)
        {
            OLED_ShowString6x8(0U, 2U, "Encoder NoSig: LEFT");
        }
        else if (model->encoder_signal_missing_mask == 2U)
        {
            OLED_ShowString6x8(0U, 2U, "Encoder NoSig: RIGHT");
        }
        else
        {
            OLED_ShowString6x8(0U, 2U, "Encoder NoSig: BOTH");
        }
        OLED_ShowString6x8(0U, 5U, "Control: SAFE OFF");
        return;
    }
    if (model->encoder_status == UI_MODULE_DIRECTION_ERROR)
    {
        if (model->encoder_direction_error_mask == 1U)
        {
            OLED_ShowString6x8(0U, 2U, "Encoder Dir: LEFT");
        }
        else if (model->encoder_direction_error_mask == 2U)
        {
            OLED_ShowString6x8(0U, 2U, "Encoder Dir: RIGHT");
        }
        else
        {
            OLED_ShowString6x8(0U, 2U, "Encoder Dir: BOTH");
        }
        OLED_ShowString6x8(0U, 5U, "Control: SAFE OFF");
        return;
    }
    if (model->encoder_status != UI_MODULE_READY)
    {
        OLED_ShowString6x8(0U, 2U, "Encoder Not Ready");
        return;
    }

    OLED_ShowString6x8(0U, 1U, " Speed :");
    ui_show_float(54U, 1U, model->encoder_speed_cm_s, 1U, 9999.9f);
    OLED_ShowChar6x8(0U, 2U, (ui_encoder_selection == 0U) ? '*' : ' ');
    OLED_ShowString6x8(6U, 2U, "T Speed:");
    ui_show_float(54U, 2U, ui_draft.target_speed_cm_s, 1U, 9999.9f);
    OLED_ShowChar6x8(0U, 3U, (ui_encoder_selection == 1U) ? '*' : ' ');
    OLED_ShowString6x8(6U, 3U, "T Dist :");
    ui_show_float(54U, 3U, ui_draft.target_distance_cm, 1U, 99999.9f);
    OLED_ShowString6x8(108U, 3U, "cm");
    OLED_ShowString6x8(0U, 4U, " Distance:");
    ui_show_float(60U, 4U, model->encoder_distance_cm, 1U, 999999.9f);
    OLED_ShowString6x8(114U, 4U, "cm");
    OLED_ShowString6x8(0U, 5U, " L Count:");
    OLED_ShowInt6x8(60U, 5U, model->left_encoder_position_count);
    OLED_ShowString6x8(0U, 6U, " R Count:");
    OLED_ShowInt6x8(60U, 6U, model->right_encoder_position_count);
    OLED_ShowString6x8(0U, 7U, ui_save_status_text());
}

static void ui_draw_pid_settings(void)
{
    static const char *parameterNames[3] = {"Kp:", "Ki:", "Kd:"};
    uint8_t visibleRow;

    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "PID Tuning");
    ui_draw_page_marker(5U);

    for (visibleRow = 0U; visibleRow < UI_PID_VISIBLE_ROWS; ++visibleRow)
    {
        uint8_t logicalRow = (uint8_t)(ui_pid_scroll_top + visibleRow);
        uint8_t group = logicalRow / 4U;
        uint8_t rowInGroup = logicalRow % 4U;
        uint8_t oledRow = (uint8_t)(visibleRow + 1U);

        if (rowInGroup == 0U)
        {
            OLED_ShowString6x8(0U, oledRow, ui_pid_group_title(group));
        }
        else
        {
            uint8_t parameter = (uint8_t)(rowInGroup - 1U);
            uint8_t selection = (uint8_t)(
                group * UI_PID_PARAMETERS_PER_GROUP + parameter);
            OLED_ShowChar6x8(0U, oledRow,
                             (selection == ui_pid_selection) ? '*' : ' ');
            OLED_ShowString6x8(6U, oledRow, parameterNames[parameter]);
            ui_show_float(36U, oledRow,
                          ui_pid_value(group, parameter), 2U, 999.99f);
        }
    }
    OLED_ShowString6x8(0U, 7U, ui_save_status_text());
}

static const char *ui_motor_mode_text(MotorDebugMode mode)
{
    return (mode == MOTOR_DEBUG_MODE_LINE_FOLLOW) ? "LINE" : "DISTANCE";
}

static const char *ui_motor_state_text(MotorDebugState state)
{
    switch (state)
    {
        case MOTOR_DEBUG_STATE_RUNNING:
            return "RUNNING";
        case MOTOR_DEBUG_STATE_DONE:
            return "DONE";
        case MOTOR_DEBUG_STATE_STOPPED:
            return "STOPPED";
        case MOTOR_DEBUG_STATE_FAULT:
            return "FAULT";
        case MOTOR_DEBUG_STATE_IDLE:
        default:
            return "IDLE";
    }
}

static const char *ui_motor_fault_text(MotorDebugStopReason reason)
{
    switch (reason)
    {
        case MOTOR_DEBUG_STOP_MOTOR_FAULT:
            return "MOTOR";
        case MOTOR_DEBUG_STOP_ENCODER_FAULT:
            return "ENCODER";
        case MOTOR_DEBUG_STOP_LINE_FAULT:
            return "LINE";
        case MOTOR_DEBUG_STOP_INVALID_CONFIG:
            return "SETTING";
        case MOTOR_DEBUG_STOP_NONE:
        default:
            return "NONE";
    }
}

static void ui_draw_motor_debug(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "Motor Debug");
    ui_draw_page_marker(6U);

    OLED_ShowString6x8(0U, 1U, "*Mode:");
    OLED_ShowString6x8(42U, 1U, ui_motor_mode_text(ui_draft.motor_mode));
    OLED_ShowString6x8(0U, 2U, " Speed:");
    ui_show_float(48U, 2U, ui_draft.target_speed_cm_s, 1U, 9999.9f);
    OLED_ShowString6x8(0U, 3U, " Target:");
    ui_show_float(48U, 3U, ui_draft.target_distance_cm, 1U, 9999.9f);
    OLED_ShowString6x8(108U, 3U, "cm");
    OLED_ShowString6x8(0U, 4U, " Travel:");
    ui_show_float(48U, 4U, model->motor_debug_travel_cm, 1U, 9999.9f);

    if (model->motor_debug_state == MOTOR_DEBUG_STATE_FAULT)
    {
        OLED_ShowString6x8(0U, 5U, " Fault:");
        OLED_ShowString6x8(48U, 5U,
            ui_motor_fault_text(model->motor_debug_stop_reason));
    }
    else if (ui_draft.motor_mode == MOTOR_DEBUG_MODE_LINE_FOLLOW)
    {
        OLED_ShowString6x8(0U, 5U,
            (model->line_lost != 0U) ? " Line: LOST" : " Line Err:");
        if (model->line_lost == 0U)
        {
            ui_show_float(66U, 5U, model->line_error, 1U, 99.9f);
        }
    }
    else
    {
        OLED_ShowString6x8(0U, 5U, " Line: --");
    }

    OLED_ShowString6x8(0U, 6U, " State:");
    OLED_ShowString6x8(48U, 6U,
                       ui_motor_state_text(model->motor_debug_state));
    OLED_ShowString6x8(0U, 7U,
        (ui_save_result == UI_SAVE_RESULT_NONE) ?
            "Ctr:S/S Hold:Save" : ui_save_status_text());
}

static const char *ui_motor_io_command_text(MotorIoTestCommand command)
{
    switch (command)
    {
        case MOTOR_IO_TEST_COMMAND_A_POSITIVE:
            return "L+";
        case MOTOR_IO_TEST_COMMAND_A_NEGATIVE:
            return "L-";
        case MOTOR_IO_TEST_COMMAND_B_POSITIVE:
            return "R+";
        case MOTOR_IO_TEST_COMMAND_B_NEGATIVE:
            return "R-";
        case MOTOR_IO_TEST_COMMAND_OFF:
        default:
            return "OFF";
    }
}

static const char *ui_motor_io_phase_text(MotorIoTestPhase phase)
{
    switch (phase)
    {
        case MOTOR_IO_TEST_PHASE_ARMED:
            return "ARM";
        case MOTOR_IO_TEST_PHASE_RUNNING:
            return "RUN";
        case MOTOR_IO_TEST_PHASE_SETTLING:
            return "COAST";
        case MOTOR_IO_TEST_PHASE_DONE:
            return "DONE";
        case MOTOR_IO_TEST_PHASE_IDLE:
        default:
            return "IDLE";
    }
}

static void ui_draw_motor_io_test(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "Motor IO Test");
    ui_draw_page_marker(7U);

    OLED_ShowString6x8(0U, 1U, "UP:L+  DOWN:L-");
    OLED_ShowString6x8(0U, 2U, "RIGHT:R+ LEFT:R-");
    OLED_ShowString6x8(0U, 3U, "L PWM PA15 A7/B1");
    OLED_ShowString6x8(0U, 4U, "R PWM PA3  B2/B3");
    OLED_ShowString6x8(0U, 5U, "Cmd:");
    OLED_ShowString6x8(24U, 5U,
        ui_motor_io_command_text(model->motor_io_test_command));
    OLED_ShowString6x8(54U, 5U,
        ui_motor_io_phase_text(model->motor_io_test_phase));
    OLED_ShowString6x8(0U, 6U, "dL:");
    OLED_ShowInt6x8(18U, 6U, model->motor_io_test_left_delta);
    OLED_ShowString6x8(66U, 6U, "dR:");
    OLED_ShowInt6x8(84U, 6U, model->motor_io_test_right_delta);
    OLED_ShowString6x8(0U, 7U, "Ctr:STOP 2s+150ms");
}

static void ui_draw_square_test(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "Square Test");
    ui_draw_page_marker(8U);

    OLED_ShowChar6x8(0U, 1U, (ui_square_selection == 0U) ? '*' : ' ');
    OLED_ShowString6x8(6U, 1U, "Speed:");
    ui_show_float(48U, 1U, ui_draft.square_speed_cm_s, 1U, 9999.9f);
    OLED_ShowString6x8(96U, 1U, "cm/s");

    OLED_ShowChar6x8(0U, 2U, (ui_square_selection == 1U) ? '*' : ' ');
    OLED_ShowString6x8(6U, 2U, "Side :");
    ui_show_float(48U, 2U, ui_draft.square_side_cm, 1U, 9999.9f);
    OLED_ShowString6x8(102U, 2U, "cm");

    OLED_ShowString6x8(0U, 4U, "Side:");
    OLED_ShowInt6x8(36U, 4U, (int32_t)(model->square_side_index + 1U));
    OLED_ShowString6x8(48U, 4U, "/4");
    OLED_ShowString6x8(0U, 5U, "Run :");
    ui_show_float(36U, 5U, model->square_progress_cm, 1U, 9999.9f);
    OLED_ShowString6x8(90U, 5U, "cm");
    OLED_ShowString6x8(0U, 6U, "State:");
    OLED_ShowString6x8(42U, 6U,
                       ui_run_status_text(model->square_test_status));
    OLED_ShowString6x8(0U, 7U, "Ctr:Run/Stop");
}

static void ui_draw_circle_test(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "Circle Test");
    ui_draw_page_marker(9U);

    OLED_ShowChar6x8(0U, 1U, (ui_circle_selection == 0U) ? '*' : ' ');
    OLED_ShowString6x8(6U, 1U, "Speed:");
    ui_show_float(48U, 1U, ui_draft.circle_speed_cm_s, 1U, 9999.9f);
    OLED_ShowString6x8(96U, 1U, "cm/s");

    OLED_ShowChar6x8(0U, 2U, (ui_circle_selection == 1U) ? '*' : ' ');
    OLED_ShowString6x8(6U, 2U, "Radius:");
    ui_show_float(54U, 2U, ui_draft.circle_radius_cm, 1U, 9999.9f);
    OLED_ShowString6x8(108U, 2U, "cm");

    OLED_ShowString6x8(0U, 5U, "Run :");
    ui_show_float(36U, 5U, model->circle_progress_cm, 1U, 9999.9f);
    OLED_ShowString6x8(90U, 5U, "cm");
    OLED_ShowString6x8(0U, 6U, "State:");
    OLED_ShowString6x8(42U, 6U,
                       ui_run_status_text(model->circle_test_status));
    OLED_ShowString6x8(0U, 7U, "Ctr:Run/Stop");
}

static const char *ui_wireless_link_text(WirelessLinkStatus status)
{
    switch (status)
    {
        case WIRELESS_LINK_OK:
            return "LINK OK";
        case WIRELESS_LINK_LOST:
            return "LINK LOST";
        case WIRELESS_LINK_FIRST_FAILED:
            return "FIRST SEND FAILED";
        case WIRELESS_LINK_INIT_FAILED:
            return "FIRST FAIL: INIT";
        case WIRELESS_LINK_WAITING:
        default:
            return "CONNECTING";
    }
}

static const char *ui_wireless_command_text(WirelessRemoteCommand command)
{
    switch (command)
    {
        case WIRELESS_COMMAND_STOP:
            return "STOP";
        case WIRELESS_COMMAND_FORWARD:
            return "FWD";
        case WIRELESS_COMMAND_BACKWARD:
            return "BACK";
        case WIRELESS_COMMAND_TURN_LEFT:
            return "L-TURN";
        case WIRELESS_COMMAND_TURN_RIGHT:
            return "R-TURN";
        case WIRELESS_COMMAND_NONE:
        default:
            return "--";
    }
}

static void ui_draw_wireless_command(uint8_t x, uint8_t page,
    WirelessRemoteCommand command, uint16_t value)
{
    OLED_ShowString6x8(x, page, ui_wireless_command_text(command));
    if ((command == WIRELESS_COMMAND_FORWARD) ||
        (command == WIRELESS_COMMAND_BACKWARD))
    {
        OLED_ShowChar6x8(66U, page, ' ');
        ui_show_uint_fixed(72U, page, value, 3U);
        OLED_ShowString6x8(90U, page, "cm");
    }
    else if ((command == WIRELESS_COMMAND_TURN_LEFT) ||
             (command == WIRELESS_COMMAND_TURN_RIGHT))
    {
        OLED_ShowChar6x8(66U, page, ' ');
        ui_show_uint_fixed(72U, page, value, 3U);
        OLED_ShowString6x8(90U, page, "deg");
    }
}

static const char *ui_wireless_action_text(UiWirelessActionState state)
{
    switch (state)
    {
        case UI_WIRELESS_ACTION_RUNNING:
            return "RUNNING";
        case UI_WIRELESS_ACTION_DONE:
            return "DONE";
        case UI_WIRELESS_ACTION_STOPPED:
            return "STOPPED";
        case UI_WIRELESS_ACTION_LINK_LOST:
            return "LINK LOST STOP";
        case UI_WIRELESS_ACTION_SENSOR_FAILED:
            return "SENSOR FAILED";
        case UI_WIRELESS_ACTION_IDLE:
        default:
            return "IDLE";
    }
}

static void ui_draw_wireless_test(const UiModel *model)
{
    (void)model;
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "NRF24 MODE");
    ui_draw_page_marker(10U);

    OLED_ShowChar6x8(0U, 1U,
        (ui_wireless_menu_selection == UI_WIRELESS_MENU_RX) ? '*' : ' ');
    OLED_ShowString6x8(12U, 1U, "RX");
    OLED_ShowChar6x8(42U, 1U,
        (ui_wireless_menu_selection == UI_WIRELESS_MENU_TX) ? '*' : ' ');
    OLED_ShowString6x8(54U, 1U, "TX");

    OLED_ShowChar6x8(0U, 2U,
        (ui_wireless_menu_selection == UI_WIRELESS_MENU_DISTANCE) ? '*' : ' ');
    OLED_ShowString6x8(12U, 2U, "DIST:");
    ui_show_uint_fixed(42U, 2U, ui_wireless_distance_cm, 3U);
    OLED_ShowString6x8(60U, 2U, "cm");

    OLED_ShowChar6x8(0U, 3U,
        (ui_wireless_menu_selection == UI_WIRELESS_MENU_ANGLE) ? '*' : ' ');
    OLED_ShowString6x8(12U, 3U, "ANGLE:");
    ui_show_uint_fixed(48U, 3U, ui_wireless_angle_deg, 3U);
    OLED_ShowString6x8(66U, 3U, "deg");

    OLED_ShowChar6x8(0U, 4U,
        (ui_wireless_menu_selection == UI_WIRELESS_MENU_STEP) ? '*' : ' ');
    OLED_ShowString6x8(12U, 4U, "STEP:");
    ui_show_uint_fixed(42U, 4U, ui_wireless_step, 3U);

    OLED_ShowString6x8(0U, 6U, "U/D:SELECT L- R+");
    OLED_ShowString6x8(0U, 7U, "C:ENTER RX/TX");
}

static void ui_draw_wireless_rx(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "MODE: RX");
    OLED_ShowString6x8(0U, 1U,
        ui_wireless_link_text(model->wireless_link_status));
    OLED_ShowString6x8(0U, 2U, "CMD:");
    ui_draw_wireless_command(30U, 2U, model->wireless_rx_command,
                             model->wireless_rx_value);
    OLED_ShowString6x8(0U, 3U, "ACT:");
    OLED_ShowString6x8(30U, 3U,
        ui_wireless_action_text(model->wireless_action_state));
    OLED_ShowString6x8(0U, 5U, "RX:");
    ui_show_uint_fixed(18U, 5U, model->wireless_rx_count, 5U);
    OLED_ShowString6x8(60U, 5U, "CE:");
    OLED_ShowChar6x8(78U, 5U,
        (model->wireless_ce_high != 0U) ? '1' : '0');
    OLED_ShowString6x8(0U, 7U, "HOLD C 2S:EXIT");
}

static void ui_draw_wireless_tx(const UiModel *model)
{
    OLED_ClearBuffer();
    OLED_ShowString6x8(0U, 0U, "MODE: TX");
    OLED_ShowString6x8(0U, 1U,
        ui_wireless_link_text(model->wireless_link_status));
    OLED_ShowString6x8(0U, 2U, "CMD:");
    ui_draw_wireless_command(30U, 2U, model->wireless_tx_command,
                             model->wireless_tx_value);
    OLED_ShowString6x8(0U, 3U, "SEND:");
    if (model->wireless_send_status == WIRELESS_SEND_SENDING)
    {
        OLED_ShowString6x8(36U, 3U, "SENDING");
    }
    else if (model->wireless_send_status == WIRELESS_SEND_RETRYING)
    {
        OLED_ShowString6x8(36U, 3U, "RETRY ");
        ui_show_uint_fixed(72U, 3U, model->wireless_retry_attempt, 1U);
        OLED_ShowString6x8(78U, 3U, "/4");
    }
    else if (model->wireless_send_status == WIRELESS_SEND_OK)
    {
        OLED_ShowString6x8(36U, 3U, "SUCCESS");
    }
    else if (model->wireless_send_status == WIRELESS_SEND_FAILED)
    {
        OLED_ShowString6x8(36U, 3U, "FAILED");
    }
    else
    {
        OLED_ShowString6x8(36U, 3U, "IDLE");
    }
    OLED_ShowString6x8(0U, 5U, "U:FWD D:BACK");
    OLED_ShowString6x8(0U, 6U, "L:LEFT R:RIGHT");
    OLED_ShowString6x8(0U, 7U, "HOLD C 2S:EXIT");
}

static void ui_render(const UiModel *model)
{
    switch (ui_page)
    {
        case UI_PAGE_TEST_SELECT:
            ui_draw_test_select(model);
            break;
        case UI_PAGE_TASK1_RUN:
            if (model->selected_test_index == 2U)
            {
                ui_draw_task3_run(model);
            }
            else if (model->selected_test_index == 1U)
            {
                ui_draw_task2_run(model);
            }
            else
            {
                ui_draw_task1_run(model);
            }
            break;
        case UI_PAGE_ICM42688:
            ui_draw_icm42688(model);
            break;
        case UI_PAGE_ENCODER_PID:
            ui_draw_encoder_pid(model);
            break;
        case UI_PAGE_PID_SETTINGS:
            ui_draw_pid_settings();
            break;
        case UI_PAGE_MOTOR_DEBUG:
            ui_draw_motor_debug(model);
            break;
        case UI_PAGE_MOTOR_IO_TEST:
            ui_draw_motor_io_test(model);
            break;
        case UI_PAGE_SQUARE_TEST:
            ui_draw_square_test(model);
            break;
        case UI_PAGE_CIRCLE_TEST:
            ui_draw_circle_test(model);
            break;
        case UI_PAGE_WIRELESS_TEST:
            ui_draw_wireless_test(model);
            break;
        case UI_PAGE_WIRELESS_RX:
            ui_draw_wireless_rx(model);
            break;
        case UI_PAGE_WIRELESS_TX:
            ui_draw_wireless_tx(model);
            break;
        case UI_PAGE_HOME:
        default:
            ui_draw_home(model);
            break;
    }
    OLED_RequestRefresh();
}

static void ui_sync_draft(const UiModel *model)
{
    if ((ui_draft_initialized == 0U) && (model != NULL))
    {
        ui_draft.target_speed_cm_s = model->target_speed_cm_s;
        ui_draft.target_distance_cm = model->target_distance_cm;
        ui_draft.pid_kp = model->pid_kp;
        ui_draft.pid_ki = model->pid_ki;
        ui_draft.pid_kd = model->pid_kd;
        ui_draft.straight_pid_kp = model->straight_pid_kp;
        ui_draft.straight_pid_ki = model->straight_pid_ki;
        ui_draft.straight_pid_kd = model->straight_pid_kd;
        ui_draft.line_pid_kp = model->line_pid_kp;
        ui_draft.line_pid_ki = model->line_pid_ki;
        ui_draft.line_pid_kd = model->line_pid_kd;
        ui_draft.angle_pid_kp = model->angle_pid_kp;
        ui_draft.angle_pid_ki = model->angle_pid_ki;
        ui_draft.angle_pid_kd = model->angle_pid_kd;
        ui_draft.motor_mode = model->motor_debug_mode;
        ui_draft.selected_test_index = model->selected_test_index;
        ui_draft.square_speed_cm_s = model->square_speed_cm_s;
        ui_draft.square_side_cm = model->square_side_cm;
        ui_draft.circle_speed_cm_s = model->circle_speed_cm_s;
        ui_draft.circle_radius_cm = model->circle_radius_cm;
        if (model->selected_test_index < UI_TEST_SELECTION_COUNT)
        {
            ui_test_selection = model->selected_test_index;
        }
        ui_draft_initialized = 1U;
    }
}

static void ui_adjust_selected(float direction)
{
    if (ui_page == UI_PAGE_ENCODER_PID)
    {
        if (ui_encoder_selection == 0U)
        {
            ui_draft.target_speed_cm_s = ui_clampf(
                ui_draft.target_speed_cm_s +
                    direction * APP_TARGET_SPEED_STEP_CM_S,
                APP_TARGET_SPEED_MIN_CM_S, APP_TARGET_SPEED_MAX_CM_S);
        }
        else
        {
            ui_draft.target_distance_cm = ui_clampf(
                ui_draft.target_distance_cm +
                    direction * APP_TARGET_DISTANCE_STEP_CM,
                APP_TARGET_DISTANCE_MIN_CM, APP_TARGET_DISTANCE_MAX_CM);
        }
    }
    else if (ui_page == UI_PAGE_PID_SETTINGS)
    {
        uint8_t group = ui_pid_selection / UI_PID_PARAMETERS_PER_GROUP;
        uint8_t parameter = ui_pid_selection % UI_PID_PARAMETERS_PER_GROUP;
        float *value;
        float minimum;
        float maximum;
        float step;

        if (group == 0U)
        {
            value = (parameter == 0U) ? &ui_draft.pid_kp :
                ((parameter == 1U) ? &ui_draft.pid_ki : &ui_draft.pid_kd);
            minimum = (parameter == 0U) ? APP_SPEED_PID_KP_MIN :
                ((parameter == 1U) ? APP_SPEED_PID_KI_MIN :
                                     APP_SPEED_PID_KD_MIN);
            maximum = (parameter == 0U) ? APP_SPEED_PID_KP_MAX :
                ((parameter == 1U) ? APP_SPEED_PID_KI_MAX :
                                     APP_SPEED_PID_KD_MAX);
            step = (parameter == 0U) ? APP_SPEED_PID_KP_STEP :
                ((parameter == 1U) ? APP_SPEED_PID_KI_STEP :
                                     APP_SPEED_PID_KD_STEP);
        }
        else if (group == 1U)
        {
            value = (parameter == 0U) ? &ui_draft.straight_pid_kp :
                ((parameter == 1U) ? &ui_draft.straight_pid_ki :
                                     &ui_draft.straight_pid_kd);
            minimum = (parameter == 0U) ? APP_STRAIGHT_PID_KP_MIN :
                ((parameter == 1U) ? APP_STRAIGHT_PID_KI_MIN :
                                     APP_STRAIGHT_PID_KD_MIN);
            maximum = (parameter == 0U) ? APP_STRAIGHT_PID_KP_MAX :
                ((parameter == 1U) ? APP_STRAIGHT_PID_KI_MAX :
                                     APP_STRAIGHT_PID_KD_MAX);
            step = (parameter == 0U) ? APP_STRAIGHT_PID_KP_STEP :
                ((parameter == 1U) ? APP_STRAIGHT_PID_KI_STEP :
                                     APP_STRAIGHT_PID_KD_STEP);
        }
        else if (group == 2U)
        {
            value = (parameter == 0U) ? &ui_draft.line_pid_kp :
                ((parameter == 1U) ? &ui_draft.line_pid_ki :
                                     &ui_draft.line_pid_kd);
            minimum = (parameter == 0U) ? APP_LINE_PID_KP_MIN :
                ((parameter == 1U) ? APP_LINE_PID_KI_MIN :
                                     APP_LINE_PID_KD_MIN);
            maximum = (parameter == 0U) ? APP_LINE_PID_KP_MAX :
                ((parameter == 1U) ? APP_LINE_PID_KI_MAX :
                                     APP_LINE_PID_KD_MAX);
            step = (parameter == 0U) ? APP_LINE_PID_KP_STEP :
                ((parameter == 1U) ? APP_LINE_PID_KI_STEP :
                                     APP_LINE_PID_KD_STEP);
        }
        else
        {
            value = (parameter == 0U) ? &ui_draft.angle_pid_kp :
                ((parameter == 1U) ? &ui_draft.angle_pid_ki :
                                     &ui_draft.angle_pid_kd);
            minimum = (parameter == 0U) ? APP_ANGLE_PID_KP_MIN :
                ((parameter == 1U) ? APP_ANGLE_PID_KI_MIN :
                                     APP_ANGLE_PID_KD_MIN);
            maximum = (parameter == 0U) ? APP_ANGLE_PID_KP_MAX :
                ((parameter == 1U) ? APP_ANGLE_PID_KI_MAX :
                                     APP_ANGLE_PID_KD_MAX);
            step = (parameter == 0U) ? APP_ANGLE_PID_KP_STEP :
                ((parameter == 1U) ? APP_ANGLE_PID_KI_STEP :
                                     APP_ANGLE_PID_KD_STEP);
        }
        *value = ui_clampf(*value + direction * step, minimum, maximum);
    }
    else if (ui_page == UI_PAGE_MOTOR_DEBUG)
    {
        if (ui_motor_running != 0U)
        {
            return;
        }
        ui_draft.motor_mode =
            (ui_draft.motor_mode == MOTOR_DEBUG_MODE_DISTANCE) ?
                MOTOR_DEBUG_MODE_LINE_FOLLOW : MOTOR_DEBUG_MODE_DISTANCE;
    }
    else if (ui_page == UI_PAGE_SQUARE_TEST)
    {
        if (ui_square_busy != 0U)
        {
            return;
        }
        if (ui_square_selection == 0U)
        {
            ui_draft.square_speed_cm_s = ui_clampf(
                ui_draft.square_speed_cm_s +
                    direction * APP_AUTO_TEST_SPEED_STEP_CM_S,
                APP_AUTO_TEST_SPEED_MIN_CM_S,
                APP_AUTO_TEST_SPEED_MAX_CM_S);
        }
        else
        {
            ui_draft.square_side_cm = ui_clampf(
                ui_draft.square_side_cm +
                    direction * APP_SQUARE_SIDE_STEP_CM,
                APP_SQUARE_SIDE_MIN_CM, APP_SQUARE_SIDE_MAX_CM);
        }
    }
    else if (ui_page == UI_PAGE_CIRCLE_TEST)
    {
        if (ui_circle_busy != 0U)
        {
            return;
        }
        if (ui_circle_selection == 0U)
        {
            ui_draft.circle_speed_cm_s = ui_clampf(
                ui_draft.circle_speed_cm_s +
                    direction * APP_AUTO_TEST_SPEED_STEP_CM_S,
                APP_AUTO_TEST_SPEED_MIN_CM_S,
                APP_AUTO_TEST_SPEED_MAX_CM_S);
        }
        else
        {
            ui_draft.circle_radius_cm = ui_clampf(
                ui_draft.circle_radius_cm +
                    direction * APP_CIRCLE_RADIUS_STEP_CM,
                APP_CIRCLE_RADIUS_MIN_CM, APP_CIRCLE_RADIUS_MAX_CM);
        }
    }
    else
    {
        return;
    }

    ui_settings_dirty = 1U;
    ui_save_result = UI_SAVE_RESULT_NONE;
    ui_render_pending = 1U;
}

static void ui_queue_run_request(UiRunRequestTarget target)
{
    ui_draft.selected_test_index = ui_test_selection;
    ui_run_request.target = target;
    ui_run_request.test_index = ui_test_selection;
    ui_run_request_pending = 1U;
    ui_render_pending = 1U;
}

static void ui_handle_events(uint32_t events, const UiModel *model)
{
    uint32_t edit_events = events;
    uint32_t amount;

    (void)model;

    if ((ui_page == UI_PAGE_WIRELESS_RX) ||
        (ui_page == UI_PAGE_WIRELESS_TX))
    {
        if (((events & UI_EVENT_CENTER_EXIT) != 0U) &&
            (ui_wireless_request_pending == 0U))
        {
            ui_wireless_request.type = UI_WIRELESS_REQUEST_EXIT_MODE;
            ui_wireless_request.mode =
                (ui_page == UI_PAGE_WIRELESS_RX) ?
                WIRELESS_TEST_MODE_RX : WIRELESS_TEST_MODE_TX;
            ui_wireless_request.command = WIRELESS_COMMAND_STOP;
            ui_wireless_request.value = 0U;
            ui_wireless_request_pending = 1U;
            ui_page = UI_PAGE_WIRELESS_TEST;
            ui_render_pending = 1U;
        }
        else if ((ui_page == UI_PAGE_WIRELESS_TX) &&
                 (ui_wireless_request_pending == 0U))
        {
            ui_wireless_request.type = UI_WIRELESS_REQUEST_SEND_COMMAND;
            ui_wireless_request.mode = WIRELESS_TEST_MODE_TX;
            if ((events & UI_EVENT_UP) != 0U)
            {
                ui_wireless_request.command = WIRELESS_COMMAND_FORWARD;
                ui_wireless_request.value = ui_wireless_distance_cm;
            }
            else if ((events & UI_EVENT_DOWN) != 0U)
            {
                ui_wireless_request.command = WIRELESS_COMMAND_BACKWARD;
                ui_wireless_request.value = ui_wireless_distance_cm;
            }
            else if ((events & UI_EVENT_LEFT) != 0U)
            {
                ui_wireless_request.command = WIRELESS_COMMAND_TURN_LEFT;
                ui_wireless_request.value = ui_wireless_angle_deg;
            }
            else if ((events & UI_EVENT_RIGHT) != 0U)
            {
                ui_wireless_request.command = WIRELESS_COMMAND_TURN_RIGHT;
                ui_wireless_request.value = ui_wireless_angle_deg;
            }
            else if ((events & UI_EVENT_CENTER) != 0U)
            {
                ui_wireless_request.command = WIRELESS_COMMAND_STOP;
                ui_wireless_request.value = 0U;
            }
            else
            {
                return;
            }
            ui_wireless_request_pending = 1U;
            ui_render_pending = 1U;
        }
        return;
    }

    if (ui_page == UI_PAGE_TASK1_RUN)
    {
        if (((events & UI_EVENT_CENTER_LONG) != 0U) &&
            (model->test_select_status != UI_RUN_STATUS_RUNNING) &&
            (model->test_select_status != UI_RUN_STATUS_PAUSED))
        {
            ui_page = UI_PAGE_TEST_SELECT;
            ui_render_pending = 1U;
        }
        return;
    }

    if ((events & UI_EVENT_PAGE_PREV) != 0U)
    {
        ui_page = (ui_page == UI_PAGE_TEST_SELECT) ?
            UI_PAGE_WIRELESS_TEST : (UiPage)(ui_page - 1);
        ui_save_result = UI_SAVE_RESULT_NONE;
        ui_render_pending = 1U;
        return;
    }
    if ((events & UI_EVENT_PAGE_NEXT) != 0U)
    {
        ui_page = (ui_page == UI_PAGE_WIRELESS_TEST) ?
            UI_PAGE_TEST_SELECT : (UiPage)(ui_page + 1);
        ui_save_result = UI_SAVE_RESULT_NONE;
        ui_render_pending = 1U;
        return;
    }

    if (ui_page == UI_PAGE_TEST_SELECT)
    {
        if ((events & UI_EVENT_UP_REPEAT) != 0U)
        {
            edit_events |= UI_EVENT_UP;
        }
        if ((events & UI_EVENT_DOWN_REPEAT) != 0U)
        {
            edit_events |= UI_EVENT_DOWN;
        }

        if ((edit_events & UI_EVENT_UP) != 0U)
        {
            ui_test_selection = (ui_test_selection == 0U) ?
                (UI_TEST_SELECTION_COUNT - 1U) :
                (uint8_t)(ui_test_selection - 1U);
            ui_draft.selected_test_index = ui_test_selection;
            ui_settings_dirty = 1U;
            ui_render_pending = 1U;
        }
        else if ((edit_events & UI_EVENT_DOWN) != 0U)
        {
            ui_test_selection = (uint8_t)(
                (ui_test_selection + 1U) % UI_TEST_SELECTION_COUNT);
            ui_draft.selected_test_index = ui_test_selection;
            ui_settings_dirty = 1U;
            ui_render_pending = 1U;
        }
        else if ((events & UI_EVENT_CENTER) != 0U)
        {
            ui_queue_run_request(UI_RUN_REQUEST_TEST_SELECT);
            if (ui_test_selection <= 2U)
            {
                ui_page = UI_PAGE_TASK1_RUN;
                ui_render_pending = 1U;
            }
        }
        return;
    }

    if (ui_page == UI_PAGE_WIRELESS_TEST)
    {
        if ((events & UI_EVENT_UP_REPEAT) != 0U)
        {
            edit_events |= UI_EVENT_UP;
        }
        if ((events & UI_EVENT_DOWN_REPEAT) != 0U)
        {
            edit_events |= UI_EVENT_DOWN;
        }
        if ((events & UI_EVENT_LEFT_REPEAT) != 0U)
        {
            edit_events |= UI_EVENT_LEFT;
        }
        if ((events & UI_EVENT_RIGHT_REPEAT) != 0U)
        {
            edit_events |= UI_EVENT_RIGHT;
        }

        if ((edit_events & UI_EVENT_UP) != 0U)
        {
            ui_wireless_menu_selection =
                (ui_wireless_menu_selection == UI_WIRELESS_MENU_RX) ?
                (UI_WIRELESS_MENU_COUNT - 1U) :
                (UiWirelessMenuItem)(ui_wireless_menu_selection - 1U);
            ui_render_pending = 1U;
        }
        else if ((edit_events & UI_EVENT_DOWN) != 0U)
        {
            ui_wireless_menu_selection = (UiWirelessMenuItem)(
                (ui_wireless_menu_selection + 1U) %
                UI_WIRELESS_MENU_COUNT);
            ui_render_pending = 1U;
        }
        else if (((edit_events & (UI_EVENT_LEFT | UI_EVENT_RIGHT)) != 0U) &&
                 (ui_wireless_menu_selection >= UI_WIRELESS_MENU_DISTANCE))
        {
            amount = (ui_wireless_menu_selection == UI_WIRELESS_MENU_STEP) ?
                1U : ui_wireless_step;
            if (ui_wireless_menu_selection == UI_WIRELESS_MENU_DISTANCE)
            {
                if ((edit_events & UI_EVENT_LEFT) != 0U)
                {
                    ui_wireless_distance_cm =
                        (ui_wireless_distance_cm <=
                         APP_NRF24_DISTANCE_MIN_CM + amount) ?
                        APP_NRF24_DISTANCE_MIN_CM :
                        (uint16_t)(ui_wireless_distance_cm - amount);
                }
                else
                {
                    ui_wireless_distance_cm =
                        (ui_wireless_distance_cm >=
                         APP_NRF24_DISTANCE_MAX_CM - amount) ?
                        APP_NRF24_DISTANCE_MAX_CM :
                        (uint16_t)(ui_wireless_distance_cm + amount);
                }
            }
            else if (ui_wireless_menu_selection == UI_WIRELESS_MENU_ANGLE)
            {
                if ((edit_events & UI_EVENT_LEFT) != 0U)
                {
                    ui_wireless_angle_deg =
                        (ui_wireless_angle_deg <=
                         APP_NRF24_ANGLE_MIN_DEG + amount) ?
                        APP_NRF24_ANGLE_MIN_DEG :
                        (uint16_t)(ui_wireless_angle_deg - amount);
                }
                else
                {
                    ui_wireless_angle_deg =
                        (ui_wireless_angle_deg >=
                         APP_NRF24_ANGLE_MAX_DEG - amount) ?
                        APP_NRF24_ANGLE_MAX_DEG :
                        (uint16_t)(ui_wireless_angle_deg + amount);
                }
            }
            else if ((edit_events & UI_EVENT_LEFT) != 0U)
            {
                ui_wireless_step =
                    (ui_wireless_step <= APP_NRF24_STEP_MIN + 1U) ?
                    APP_NRF24_STEP_MIN :
                    (uint16_t)(ui_wireless_step - 1U);
            }
            else
            {
                ui_wireless_step =
                    (ui_wireless_step >= APP_NRF24_STEP_MAX - 1U) ?
                    APP_NRF24_STEP_MAX :
                    (uint16_t)(ui_wireless_step + 1U);
            }
            ui_render_pending = 1U;
        }
        else if (((events & UI_EVENT_CENTER) != 0U) &&
                 (ui_wireless_request_pending == 0U) &&
                 (ui_wireless_menu_selection <= UI_WIRELESS_MENU_TX))
        {
            ui_wireless_request.type = UI_WIRELESS_REQUEST_ENTER_MODE;
            ui_wireless_request.mode =
                (ui_wireless_menu_selection == UI_WIRELESS_MENU_RX) ?
                WIRELESS_TEST_MODE_RX : WIRELESS_TEST_MODE_TX;
            ui_wireless_request.command = WIRELESS_COMMAND_NONE;
            ui_wireless_request.value = 0U;
            ui_wireless_request_pending = 1U;
            ui_page =
                (ui_wireless_request.mode == WIRELESS_TEST_MODE_RX) ?
                UI_PAGE_WIRELESS_RX : UI_PAGE_WIRELESS_TX;
            ui_render_pending = 1U;
        }
        return;
    }

    if ((ui_page == UI_PAGE_ENCODER_PID) ||
        (ui_page == UI_PAGE_PID_SETTINGS) ||
        (ui_page == UI_PAGE_MOTOR_DEBUG) ||
        (ui_page == UI_PAGE_SQUARE_TEST) ||
        (ui_page == UI_PAGE_CIRCLE_TEST))
    {
        /*
         * Only parameter pages consume direction-repeat bits. Motor mode and
         * physical IO testing keep one action for each real press edge.
         */
        if ((ui_page == UI_PAGE_ENCODER_PID) ||
            (ui_page == UI_PAGE_PID_SETTINGS) ||
            (ui_page == UI_PAGE_SQUARE_TEST) ||
            (ui_page == UI_PAGE_CIRCLE_TEST))
        {
            if ((events & UI_EVENT_UP_REPEAT) != 0U)
            {
                edit_events |= UI_EVENT_UP;
            }
            if ((events & UI_EVENT_DOWN_REPEAT) != 0U)
            {
                edit_events |= UI_EVENT_DOWN;
            }
            if ((events & UI_EVENT_LEFT_REPEAT) != 0U)
            {
                edit_events |= UI_EVENT_LEFT;
            }
            if ((events & UI_EVENT_RIGHT_REPEAT) != 0U)
            {
                edit_events |= UI_EVENT_RIGHT;
            }
        }

        if ((events & UI_EVENT_CENTER_LONG) != 0U)
        {
            ui_saved_settings = ui_draft;
            ui_save_pending = 1U;
            ui_settings_dirty = 0U;
            ui_save_result = UI_SAVE_RESULT_PENDING;
            ui_render_pending = 1U;
        }
        else if ((edit_events & UI_EVENT_LEFT) != 0U)
        {
            ui_adjust_selected(-1.0f);
        }
        else if ((edit_events & UI_EVENT_RIGHT) != 0U)
        {
            ui_adjust_selected(1.0f);
        }
        else if (ui_page == UI_PAGE_ENCODER_PID)
        {
            if ((edit_events & UI_EVENT_UP) != 0U)
            {
                ui_encoder_selection = (ui_encoder_selection == 0U) ? 1U : 0U;
                ui_save_result = UI_SAVE_RESULT_NONE;
                ui_render_pending = 1U;
            }
            else if ((edit_events & UI_EVENT_DOWN) != 0U)
            {
                ui_encoder_selection = (uint8_t)((ui_encoder_selection + 1U) % 2U);
                ui_save_result = UI_SAVE_RESULT_NONE;
                ui_render_pending = 1U;
            }
        }
        else if (ui_page == UI_PAGE_PID_SETTINGS)
        {
            if ((edit_events & UI_EVENT_UP) != 0U)
            {
                ui_pid_selection = (ui_pid_selection == 0U) ?
                                    (UI_PID_SELECTION_COUNT - 1U) :
                                    (uint8_t)(ui_pid_selection - 1U);
                ui_update_pid_scroll();
                ui_save_result = UI_SAVE_RESULT_NONE;
                ui_render_pending = 1U;
            }
            else if ((edit_events & UI_EVENT_DOWN) != 0U)
            {
                ui_pid_selection = (uint8_t)(
                    (ui_pid_selection + 1U) % UI_PID_SELECTION_COUNT);
                ui_update_pid_scroll();
                ui_save_result = UI_SAVE_RESULT_NONE;
                ui_render_pending = 1U;
            }
        }
        else if (ui_page == UI_PAGE_SQUARE_TEST)
        {
            if ((edit_events & UI_EVENT_UP) != 0U)
            {
                ui_square_selection = (ui_square_selection == 0U) ? 1U : 0U;
                ui_save_result = UI_SAVE_RESULT_NONE;
                ui_render_pending = 1U;
            }
            else if ((edit_events & UI_EVENT_DOWN) != 0U)
            {
                ui_square_selection =
                    (uint8_t)((ui_square_selection + 1U) % 2U);
                ui_save_result = UI_SAVE_RESULT_NONE;
                ui_render_pending = 1U;
            }
            else if ((events & UI_EVENT_CENTER) != 0U)
            {
                ui_queue_run_request(UI_RUN_REQUEST_SQUARE);
            }
        }
        else if (ui_page == UI_PAGE_CIRCLE_TEST)
        {
            if ((edit_events & UI_EVENT_UP) != 0U)
            {
                ui_circle_selection = (ui_circle_selection == 0U) ? 1U : 0U;
                ui_save_result = UI_SAVE_RESULT_NONE;
                ui_render_pending = 1U;
            }
            else if ((edit_events & UI_EVENT_DOWN) != 0U)
            {
                ui_circle_selection =
                    (uint8_t)((ui_circle_selection + 1U) % 2U);
                ui_save_result = UI_SAVE_RESULT_NONE;
                ui_render_pending = 1U;
            }
            else if ((events & UI_EVENT_CENTER) != 0U)
            {
                ui_queue_run_request(UI_RUN_REQUEST_CIRCLE);
            }
        }
    }
}

void UI_Init(uint32_t now_ms)
{
    ui_page = UI_PAGE_TEST_SELECT;
    ui_logo_active = (OLED_SHOW_STARTUP_LOGO != 0U) ? 1U : 0U;
    ui_logo_transfer_complete = (ui_logo_active != 0U) ? 0U : 1U;
    ui_render_pending = (ui_logo_active != 0U) ? 0U : 1U;
    ui_draft_initialized = 0U;
    ui_test_selection = 0U;
    ui_square_selection = 0U;
    ui_circle_selection = 0U;
    ui_encoder_selection = 0U;
    ui_pid_selection = 0U;
    ui_pid_scroll_top = 0U;
    ui_settings_dirty = 0U;
    ui_save_result = UI_SAVE_RESULT_NONE;
    ui_save_pending = 0U;
    ui_run_request_pending = 0U;
    ui_wireless_request_pending = 0U;
    ui_wireless_menu_selection = UI_WIRELESS_MENU_RX;
    ui_wireless_distance_cm = APP_NRF24_DISTANCE_DEFAULT_CM;
    ui_wireless_angle_deg = APP_NRF24_ANGLE_DEFAULT_DEG;
    ui_wireless_step = APP_NRF24_STEP_DEFAULT;
    ui_motor_running = 0U;
    ui_square_busy = 0U;
    ui_circle_busy = 0U;
    ui_logo_frame = 0U;
    ui_logo_frame_phase = 0U;
    ui_logo_visible_since_ms = now_ms;
    ui_logo_last_frame_ms = now_ms;
    ui_last_submit_ms = now_ms;
    if (ui_logo_active != 0U)
    {
        OLED_ShowStartupLogo();
    }
}

void UI_Service(uint32_t now_ms, uint32_t button_events,
                const UiModel *model)
{
    static const UiModel empty_model = {0};
    uint32_t logo_elapsed_ms;
    uint32_t logo_frame_interval_ms;
    uint32_t logo_progress;

    OLED_Process();

    if (ui_logo_active != 0U)
    {
        if ((ui_logo_transfer_complete == 0U) && (OLED_IsBusy() == 0U))
        {
            ui_logo_transfer_complete = 1U;
            ui_logo_visible_since_ms = now_ms;
            ui_logo_last_frame_ms = now_ms;
        }
        logo_frame_interval_ms =
            (ui_logo_frame_phase < 2U) ?
            UI_LOGO_FRAME_LONG_MS : UI_LOGO_FRAME_SHORT_MS;
        if ((ui_logo_transfer_complete != 0U) &&
            (OLED_IsBusy() == 0U) &&
            (ui_elapsed(now_ms, ui_logo_visible_since_ms,
                        UI_LOGO_VISIBLE_MS) == 0U) &&
            (ui_elapsed(now_ms, ui_logo_last_frame_ms,
                        logo_frame_interval_ms) != 0U))
        {
            ui_logo_last_frame_ms = now_ms;
            ui_logo_frame = (uint8_t)((ui_logo_frame + 1U) %
                                      UI_LOGO_FRAME_COUNT);
            ui_logo_frame_phase++;
            if (ui_logo_frame_phase >= UI_LOGO_FRAME_PHASE_COUNT)
            {
                ui_logo_frame_phase = 0U;
            }
            logo_elapsed_ms = (uint32_t)(now_ms - ui_logo_visible_since_ms);
            logo_progress =
                ((logo_elapsed_ms + UI_LOGO_FRAME_LONG_MS) * 100UL) /
                UI_LOGO_VISIBLE_MS;
            if (logo_progress > 100UL)
            {
                logo_progress = 100UL;
            }
            OLED_ShowStartupLogoFrame(ui_logo_frame,
                                      (uint8_t)logo_progress);
        }
        if ((ui_logo_transfer_complete != 0U) &&
            (ui_elapsed(now_ms, ui_logo_visible_since_ms,
                        UI_LOGO_VISIBLE_MS) != 0U))
        {
            ui_logo_active = 0U;
            ui_page = UI_PAGE_TEST_SELECT;
            ui_render_pending = 1U;
        }
        else
        {
            return;
        }
    }

    if (model == NULL)
    {
        model = &empty_model;
    }
    ui_sync_draft(model);
    ui_motor_running =
        (model->motor_debug_state == MOTOR_DEBUG_STATE_RUNNING) ? 1U : 0U;
    ui_square_busy =
        ((model->square_test_status == UI_RUN_STATUS_DELAY) ||
         (model->square_test_status == UI_RUN_STATUS_RUNNING)) ? 1U : 0U;
    ui_circle_busy =
        ((model->circle_test_status == UI_RUN_STATUS_DELAY) ||
         (model->circle_test_status == UI_RUN_STATUS_RUNNING)) ? 1U : 0U;
    ui_handle_events(button_events, model);

    if ((OLED_IsBusy() == 0U) &&
        ((ui_render_pending != 0U) ||
         (ui_elapsed(now_ms, ui_last_submit_ms,
                     UI_REFRESH_PERIOD_MS) != 0U)))
    {
        ui_render(model);
        ui_render_pending = 0U;
        ui_last_submit_ms = now_ms;
    }
}

void UI_RequestRefresh(void)
{
    ui_render_pending = 1U;
}

uint8_t UI_TakeSavedControlSettings(UiControlSettings *settings)
{
    if ((ui_save_pending == 0U) || (settings == NULL))
    {
        return 0U;
    }
    *settings = ui_saved_settings;
    ui_save_pending = 0U;
    return 1U;
}

void UI_GetDraftControlSettings(UiControlSettings *settings)
{
    if (settings != NULL)
    {
        *settings = ui_draft;
    }
}

void UI_ReportSaveResult(UiSaveResult result)
{
    ui_save_result = result;
    if (result == UI_SAVE_RESULT_SUCCESS)
    {
        ui_settings_dirty = 0U;
    }
    else if (result == UI_SAVE_RESULT_FAILED)
    {
        ui_settings_dirty = 1U;
    }
    ui_render_pending = 1U;
}

uint8_t UI_TakeRunRequest(UiRunRequest *request)
{
    if ((ui_run_request_pending == 0U) || (request == NULL))
    {
        return 0U;
    }
    *request = ui_run_request;
    ui_run_request_pending = 0U;
    return 1U;
}

void UI_AdvanceSelectedTest(void)
{
    ui_test_selection = (uint8_t)(
        (ui_test_selection + 1U) % UI_TEST_SELECTION_COUNT);
    ui_draft.selected_test_index = ui_test_selection;
    ui_render_pending = 1U;
}

uint8_t UI_TakeWirelessRequest(UiWirelessRequest *request)
{
    if ((ui_wireless_request_pending == 0U) || (request == NULL))
    {
        return 0U;
    }
    *request = ui_wireless_request;
    ui_wireless_request_pending = 0U;
    return 1U;
}

UiPage UI_GetPage(void)
{
    return ui_page;
}

uint8_t UI_IsLogoActive(void)
{
    return ui_logo_active;
}
