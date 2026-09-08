/*
================================================================================
W25Q64 参数日志实现模块
================================================================================
【功能简介】
本文件在 W25Q64 最后一个 4 KiB 扇区中实现 128 字节定长追加日志。每条记录
包含魔数、版本、序号、目标值、四套 PID、模式和 CRC32；掉电后扫描并恢复
序号最新的有效记录。

================================================================================
【函数定义】
- SettingsStore_Crc32：计算记录正文的 IEEE CRC32。
- SettingsStore_SettingsValid：检查浮点有效性、范围和电机模式。
- SettingsStore_RecordEmpty：判断日志槽是否保持全 0xFF 擦除态。
- SettingsStore_RecordToSettings：把 Flash 记录转换为应用参数结构。
- SettingsStore_RecordValid：校验魔数、版本、参数范围和 CRC。
- SettingsStore_IsNewer：使用回绕安全序号比较两条记录的新旧。
- SettingsStore_Scan：遍历扇区并定位最新记录和下一个空槽。
- SettingsStore_Init：初始化 W25Q64 并扫描参数日志。
- SettingsStore_IsReady：返回日志模块是否可用。
- SettingsStore_HasValidRecord：返回是否存在可加载记录。
- SettingsStore_GetJedecId：转发底层 Flash JEDEC ID。
- SettingsStore_Load：复制缓存中的最新有效参数。
- SettingsStore_Save：必要时擦除扇区，写入并回读验证新记录。

================================================================================
【使用说明】
1. 本模块只由主循环调用，不可在中断中擦除或写入 Flash。
2. 记录结构、版本和扇区布局属于存储协议，不作为日常调车参数修改。
3. 参数上下限来自 user_config.h；格式升级时必须同步增加 SETTINGS_VERSION。
================================================================================
*/
#include "settings_store.h"

#include "app_config.h"
#include "w25q64.h"

#include <math.h>
#include <stddef.h>

#define SETTINGS_MAGIC                (0x53343236UL) /* "S426" */
#define SETTINGS_VERSION              (3UL)
#define SETTINGS_LEGACY_VERSION       (2UL)
#define SETTINGS_SECTOR_ADDRESS       \
    (W25Q64_CAPACITY_BYTES - W25Q64_SECTOR_SIZE_BYTES)
#define SETTINGS_RECORD_SIZE          (128UL)
#define SETTINGS_RECORD_COUNT         \
    (W25Q64_SECTOR_SIZE_BYTES / SETTINGS_RECORD_SIZE)
#define SETTINGS_NO_SLOT              (UINT32_MAX)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
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
    uint32_t motor_mode;
    uint32_t selected_test_index;
    float square_speed_cm_s;
    float square_side_cm;
    float circle_speed_cm_s;
    float circle_radius_cm;
    uint32_t reserved[8];
    uint32_t crc32;
} SettingsRecord;

_Static_assert(sizeof(SettingsRecord) == SETTINGS_RECORD_SIZE,
               "SettingsRecord must stay at one 128-byte journal slot");

static bool g_ready;
static bool g_hasValidRecord;
static uint32_t g_latestSequence;
static uint32_t g_nextSlot;
static AppPersistentSettings g_cachedSettings;

static uint32_t SettingsStore_Crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t bit;

    for (i = 0U; i < length; ++i) {
        crc ^= bytes[i];
        for (bit = 0U; bit < 8U; ++bit) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1UL));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static bool SettingsStore_SettingsValid(
    const AppPersistentSettings *settings)
{
    if ((settings == NULL) || (!isfinite(settings->target_speed_cm_s)) ||
        (!isfinite(settings->target_distance_cm)) ||
        (!isfinite(settings->pid_kp)) || (!isfinite(settings->pid_ki)) ||
        (!isfinite(settings->pid_kd)) ||
        (!isfinite(settings->straight_pid_kp)) ||
        (!isfinite(settings->straight_pid_ki)) ||
        (!isfinite(settings->straight_pid_kd)) ||
        (!isfinite(settings->line_pid_kp)) ||
        (!isfinite(settings->line_pid_ki)) ||
        (!isfinite(settings->line_pid_kd)) ||
        (!isfinite(settings->angle_pid_kp)) ||
        (!isfinite(settings->angle_pid_ki)) ||
        (!isfinite(settings->angle_pid_kd)) ||
        (!isfinite(settings->square_speed_cm_s)) ||
        (!isfinite(settings->square_side_cm)) ||
        (!isfinite(settings->circle_speed_cm_s)) ||
        (!isfinite(settings->circle_radius_cm))) {
        return false;
    }
    return ((settings->target_speed_cm_s >= APP_TARGET_SPEED_MIN_CM_S) &&
            (settings->target_speed_cm_s <= APP_TARGET_SPEED_MAX_CM_S) &&
            (settings->target_distance_cm >= APP_TARGET_DISTANCE_MIN_CM) &&
            (settings->target_distance_cm <= APP_TARGET_DISTANCE_MAX_CM) &&
            (settings->pid_kp >= APP_SPEED_PID_KP_MIN) &&
            (settings->pid_kp <= APP_SPEED_PID_KP_MAX) &&
            (settings->pid_ki >= APP_SPEED_PID_KI_MIN) &&
            (settings->pid_ki <= APP_SPEED_PID_KI_MAX) &&
            (settings->pid_kd >= APP_SPEED_PID_KD_MIN) &&
            (settings->pid_kd <= APP_SPEED_PID_KD_MAX) &&
            (settings->straight_pid_kp >= APP_STRAIGHT_PID_KP_MIN) &&
            (settings->straight_pid_kp <= APP_STRAIGHT_PID_KP_MAX) &&
            (settings->straight_pid_ki >= APP_STRAIGHT_PID_KI_MIN) &&
            (settings->straight_pid_ki <= APP_STRAIGHT_PID_KI_MAX) &&
            (settings->straight_pid_kd >= APP_STRAIGHT_PID_KD_MIN) &&
            (settings->straight_pid_kd <= APP_STRAIGHT_PID_KD_MAX) &&
            (settings->line_pid_kp >= APP_LINE_PID_KP_MIN) &&
            (settings->line_pid_kp <= APP_LINE_PID_KP_MAX) &&
            (settings->line_pid_ki >= APP_LINE_PID_KI_MIN) &&
            (settings->line_pid_ki <= APP_LINE_PID_KI_MAX) &&
            (settings->line_pid_kd >= APP_LINE_PID_KD_MIN) &&
            (settings->line_pid_kd <= APP_LINE_PID_KD_MAX) &&
            (settings->angle_pid_kp >= APP_ANGLE_PID_KP_MIN) &&
            (settings->angle_pid_kp <= APP_ANGLE_PID_KP_MAX) &&
            (settings->angle_pid_ki >= APP_ANGLE_PID_KI_MIN) &&
            (settings->angle_pid_ki <= APP_ANGLE_PID_KI_MAX) &&
            (settings->angle_pid_kd >= APP_ANGLE_PID_KD_MIN) &&
            (settings->angle_pid_kd <= APP_ANGLE_PID_KD_MAX) &&
            (settings->motor_mode <= 1U) &&
            (settings->selected_test_index < APP_TEST_SLOT_COUNT) &&
            (settings->square_speed_cm_s >=
                APP_AUTO_TEST_SPEED_MIN_CM_S) &&
            (settings->square_speed_cm_s <=
                APP_AUTO_TEST_SPEED_MAX_CM_S) &&
            (settings->square_side_cm >= APP_SQUARE_SIDE_MIN_CM) &&
            (settings->square_side_cm <= APP_SQUARE_SIDE_MAX_CM) &&
            (settings->circle_speed_cm_s >=
                APP_AUTO_TEST_SPEED_MIN_CM_S) &&
            (settings->circle_speed_cm_s <=
                APP_AUTO_TEST_SPEED_MAX_CM_S) &&
            (settings->circle_radius_cm >= APP_CIRCLE_RADIUS_MIN_CM) &&
            (settings->circle_radius_cm <= APP_CIRCLE_RADIUS_MAX_CM));
}

static bool SettingsStore_RecordEmpty(const SettingsRecord *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t i;

    for (i = 0U; i < sizeof(*record); ++i) {
        if (bytes[i] != 0xFFU) {
            return false;
        }
    }
    return true;
}

static void SettingsStore_RecordToSettings(
    const SettingsRecord *record, AppPersistentSettings *settings)
{
    settings->target_speed_cm_s = record->target_speed_cm_s;
    settings->target_distance_cm = record->target_distance_cm;
    settings->pid_kp = record->pid_kp;
    settings->pid_ki = record->pid_ki;
    settings->pid_kd = record->pid_kd;
    settings->straight_pid_kp = record->straight_pid_kp;
    settings->straight_pid_ki = record->straight_pid_ki;
    settings->straight_pid_kd = record->straight_pid_kd;
    settings->line_pid_kp = record->line_pid_kp;
    settings->line_pid_ki = record->line_pid_ki;
    settings->line_pid_kd = record->line_pid_kd;
    settings->angle_pid_kp = record->angle_pid_kp;
    settings->angle_pid_ki = record->angle_pid_ki;
    settings->angle_pid_kd = record->angle_pid_kd;
    settings->motor_mode = (uint8_t)record->motor_mode;
    if (record->version >= SETTINGS_VERSION) {
        settings->selected_test_index =
            (uint8_t)record->selected_test_index;
        settings->square_speed_cm_s = record->square_speed_cm_s;
        settings->square_side_cm = record->square_side_cm;
        settings->circle_speed_cm_s = record->circle_speed_cm_s;
        settings->circle_radius_cm = record->circle_radius_cm;
    } else {
        settings->selected_test_index = APP_DEFAULT_SELECTED_TEST_INDEX;
        settings->square_speed_cm_s = APP_DEFAULT_SQUARE_SPEED_CM_S;
        settings->square_side_cm = APP_DEFAULT_SQUARE_SIDE_CM;
        settings->circle_speed_cm_s = APP_DEFAULT_CIRCLE_SPEED_CM_S;
        settings->circle_radius_cm = APP_DEFAULT_CIRCLE_RADIUS_CM;
    }
}

static bool SettingsStore_RecordValid(const SettingsRecord *record)
{
    AppPersistentSettings settings;
    uint32_t crc;

    if ((record->magic != SETTINGS_MAGIC) ||
        ((record->version != SETTINGS_VERSION) &&
         (record->version != SETTINGS_LEGACY_VERSION))) {
        return false;
    }
    crc = SettingsStore_Crc32(record,
        (uint32_t)offsetof(SettingsRecord, crc32));
    if (crc != record->crc32) {
        return false;
    }
    SettingsStore_RecordToSettings(record, &settings);
    return SettingsStore_SettingsValid(&settings);
}

static bool SettingsStore_IsNewer(uint32_t candidate, uint32_t current)
{
    return ((int32_t)(candidate - current) > 0);
}

static bool SettingsStore_Scan(void)
{
    uint32_t slot;
    SettingsRecord record;

    g_hasValidRecord = false;
    g_latestSequence = 0U;
    g_nextSlot = SETTINGS_NO_SLOT;

    for (slot = 0U; slot < SETTINGS_RECORD_COUNT; ++slot) {
        uint32_t address = SETTINGS_SECTOR_ADDRESS +
            slot * SETTINGS_RECORD_SIZE;
        if (!W25Q64_Read(address, &record, sizeof(record))) {
            return false;
        }
        if (SettingsStore_RecordEmpty(&record)) {
            if (g_nextSlot == SETTINGS_NO_SLOT) {
                g_nextSlot = slot;
            }
        } else if (SettingsStore_RecordValid(&record) &&
                   ((!g_hasValidRecord) ||
                    SettingsStore_IsNewer(record.sequence,
                                          g_latestSequence))) {
            g_latestSequence = record.sequence;
            SettingsStore_RecordToSettings(&record, &g_cachedSettings);
            g_hasValidRecord = true;
        }
    }
    return true;
}

bool SettingsStore_Init(void)
{
    g_ready = false;
    g_hasValidRecord = false;
    g_nextSlot = SETTINGS_NO_SLOT;

    if (!W25Q64_Init()) {
        return false;
    }
    if (!SettingsStore_Scan()) {
        return false;
    }
    g_ready = true;
    return true;
}

bool SettingsStore_IsReady(void)
{
    return g_ready;
}

bool SettingsStore_HasValidRecord(void)
{
    return g_hasValidRecord;
}

uint32_t SettingsStore_GetJedecId(void)
{
    return W25Q64_GetJedecId();
}

bool SettingsStore_Load(AppPersistentSettings *settings)
{
    if ((!g_ready) || (!g_hasValidRecord) || (settings == NULL)) {
        return false;
    }
    *settings = g_cachedSettings;
    return true;
}

bool SettingsStore_Save(const AppPersistentSettings *settings)
{
    SettingsRecord record = {0};
    SettingsRecord verify;
    uint32_t address;
    uint32_t i;
    const uint8_t *written;
    const uint8_t *read_back;

    if ((!g_ready) || (!SettingsStore_SettingsValid(settings))) {
        return false;
    }

    if (g_nextSlot == SETTINGS_NO_SLOT) {
        if (!W25Q64_EraseSector4K(SETTINGS_SECTOR_ADDRESS)) {
            return false;
        }
        g_nextSlot = 0U;
    }

    record.magic = SETTINGS_MAGIC;
    record.version = SETTINGS_VERSION;
    record.sequence = g_hasValidRecord ? (g_latestSequence + 1U) : 1U;
    record.target_speed_cm_s = settings->target_speed_cm_s;
    record.target_distance_cm = settings->target_distance_cm;
    record.pid_kp = settings->pid_kp;
    record.pid_ki = settings->pid_ki;
    record.pid_kd = settings->pid_kd;
    record.straight_pid_kp = settings->straight_pid_kp;
    record.straight_pid_ki = settings->straight_pid_ki;
    record.straight_pid_kd = settings->straight_pid_kd;
    record.line_pid_kp = settings->line_pid_kp;
    record.line_pid_ki = settings->line_pid_ki;
    record.line_pid_kd = settings->line_pid_kd;
    record.angle_pid_kp = settings->angle_pid_kp;
    record.angle_pid_ki = settings->angle_pid_ki;
    record.angle_pid_kd = settings->angle_pid_kd;
    record.motor_mode = settings->motor_mode;
    record.selected_test_index = settings->selected_test_index;
    record.square_speed_cm_s = settings->square_speed_cm_s;
    record.square_side_cm = settings->square_side_cm;
    record.circle_speed_cm_s = settings->circle_speed_cm_s;
    record.circle_radius_cm = settings->circle_radius_cm;
    record.crc32 = SettingsStore_Crc32(&record,
        (uint32_t)offsetof(SettingsRecord, crc32));

    address = SETTINGS_SECTOR_ADDRESS + g_nextSlot * SETTINGS_RECORD_SIZE;
    if (!W25Q64_PageProgram(address, &record, sizeof(record)) ||
        !W25Q64_Read(address, &verify, sizeof(verify)) ||
        !SettingsStore_RecordValid(&verify)) {
        (void)SettingsStore_Scan();
        return false;
    }

    written = (const uint8_t *)&record;
    read_back = (const uint8_t *)&verify;
    for (i = 0U; i < sizeof(record); ++i) {
        if (written[i] != read_back[i]) {
            (void)SettingsStore_Scan();
            return false;
        }
    }

    g_cachedSettings = *settings;
    g_latestSequence = record.sequence;
    g_hasValidRecord = true;
    ++g_nextSlot;
    if (g_nextSlot >= SETTINGS_RECORD_COUNT) {
        g_nextSlot = SETTINGS_NO_SLOT;
    }
    return true;
}
