#include "battery_monitor.h"

#include "buzzer_rgb.h"
#include "ti_msp_dl_config.h"
#include "user_config.h"

#include <stddef.h>

#if (APP_BATTERY_DIVIDER_BOTTOM_KOHM == 0U)
#error "Battery divider bottom resistance must be nonzero"
#endif

#if (APP_BATTERY_FILTER_SHIFT == 0U) || (APP_BATTERY_FILTER_SHIFT > 8U)
#error "Battery filter shift must be in the range 1..8"
#endif

#if (APP_BATTERY_WARNING_RECOVERY_MV <= APP_BATTERY_WARNING_ENTER_MV)
#error "Battery warning recovery voltage must exceed the entry voltage"
#endif

#if (APP_BATTERY_WARNING_BLINK_PERIOD_MS < 2U)
#error "Battery warning blink period must be at least 2 ms"
#endif

typedef struct {
    uint16_t rawCounts;
    uint32_t voltageMilliVolts;
    int32_t filteredMilliVoltsScaled;
    uint32_t lastSampleStartMs;
    uint32_t conversionStartMs;
    uint32_t lowSinceMs;
    uint32_t warningStartMs;
    uint32_t lastBeepStartMs;
    uint32_t beepStartMs;
    uint8_t initialized;
    uint8_t sampleStarted;
    uint8_t conversionPending;
    uint8_t ready;
    uint8_t lowCandidate;
    uint8_t lowVoltageWarning;
    uint8_t redOn;
    uint8_t beepOn;
} BatteryMonitorState;

static BatteryMonitorState g_battery;

static bool BatteryMonitor_Elapsed(uint32_t now, uint32_t then,
                                   uint32_t interval)
{
    return ((uint32_t)(now - then) >= interval);
}

static uint32_t BatteryMonitor_ConvertRawToMilliVolts(uint16_t rawCounts)
{
    const uint64_t dividerNumerator =
        (uint64_t)APP_BATTERY_DIVIDER_TOP_KOHM +
        (uint64_t)APP_BATTERY_DIVIDER_BOTTOM_KOHM;
    const uint64_t numerator =
        (uint64_t)rawCounts *
        (uint64_t)APP_BATTERY_ADC_REFERENCE_MV *
        dividerNumerator *
        (uint64_t)APP_BATTERY_CALIBRATION_PERMILLE;
    const uint64_t denominator =
        (uint64_t)APP_BATTERY_ADC_FULL_SCALE_COUNTS *
        (uint64_t)APP_BATTERY_DIVIDER_BOTTOM_KOHM *
        1000ULL;

    return (uint32_t)((numerator + (denominator / 2ULL)) / denominator);
}

static void BatteryMonitor_SetWarning(bool enabled, uint32_t now)
{
    if (enabled) {
        g_battery.lowVoltageWarning = 1U;
        g_battery.warningStartMs = now;
        g_battery.lastBeepStartMs = now;
        g_battery.beepStartMs = now;
        g_battery.redOn = 1U;
        g_battery.beepOn = 1U;
        RGB_SetColor(RGB_COLOR_RED);
        Buzzer_On();
    } else {
        g_battery.lowVoltageWarning = 0U;
        g_battery.lowCandidate = 0U;
        g_battery.redOn = 0U;
        g_battery.beepOn = 0U;
        RGB_Off();
        Buzzer_Off();
    }
}

static void BatteryMonitor_UpdateWarningState(uint32_t now)
{
    if (g_battery.lowVoltageWarning != 0U) {
        if (g_battery.voltageMilliVolts >=
            APP_BATTERY_WARNING_RECOVERY_MV) {
            BatteryMonitor_SetWarning(false, now);
        }
        return;
    }

    if (g_battery.voltageMilliVolts < APP_BATTERY_WARNING_ENTER_MV) {
        if (g_battery.lowCandidate == 0U) {
            g_battery.lowCandidate = 1U;
            g_battery.lowSinceMs = now;
        } else if (BatteryMonitor_Elapsed(
                       now, g_battery.lowSinceMs,
                       APP_BATTERY_WARNING_QUALIFY_MS)) {
            BatteryMonitor_SetWarning(true, now);
        }
    } else {
        g_battery.lowCandidate = 0U;
    }
}

static void BatteryMonitor_UpdateWarningOutputs(uint32_t now)
{
    uint32_t blinkHalfPeriod;
    uint8_t redOn;

    if (g_battery.lowVoltageWarning == 0U) {
        return;
    }

    blinkHalfPeriod = APP_BATTERY_WARNING_BLINK_PERIOD_MS / 2U;
    redOn = (((uint32_t)(now - g_battery.warningStartMs) /
              blinkHalfPeriod) & 1U) == 0U ? 1U : 0U;
    if (redOn != g_battery.redOn) {
        g_battery.redOn = redOn;
        RGB_SetColor((redOn != 0U) ? RGB_COLOR_RED : RGB_COLOR_OFF);
    }

    if (g_battery.beepOn != 0U) {
        if (BatteryMonitor_Elapsed(
                now, g_battery.beepStartMs,
                APP_BATTERY_WARNING_BEEP_DURATION_MS)) {
            g_battery.beepOn = 0U;
            Buzzer_Off();
        }
    } else if (BatteryMonitor_Elapsed(
                   now, g_battery.lastBeepStartMs,
                   APP_BATTERY_WARNING_BEEP_PERIOD_MS)) {
        g_battery.lastBeepStartMs = now;
        g_battery.beepStartMs = now;
        g_battery.beepOn = 1U;
        Buzzer_On();
    }
}

static void BatteryMonitor_ProcessSample(uint16_t rawCounts, uint32_t now)
{
    uint32_t sampleMilliVolts =
        BatteryMonitor_ConvertRawToMilliVolts(rawCounts);
    const int32_t scale = (int32_t)(1UL << APP_BATTERY_FILTER_SHIFT);

    g_battery.rawCounts = rawCounts;
    if (g_battery.ready == 0U) {
        g_battery.filteredMilliVoltsScaled =
            (int32_t)sampleMilliVolts * scale;
        g_battery.ready = 1U;
    } else {
        g_battery.filteredMilliVoltsScaled +=
            (int32_t)sampleMilliVolts -
            (g_battery.filteredMilliVoltsScaled >>
             APP_BATTERY_FILTER_SHIFT);
    }

    g_battery.voltageMilliVolts = (uint32_t)(
        (g_battery.filteredMilliVoltsScaled + (scale / 2)) >>
        APP_BATTERY_FILTER_SHIFT);
    BatteryMonitor_UpdateWarningState(now);
}

bool BatteryMonitor_Init(void)
{
    g_battery = (BatteryMonitorState){0};

    if (!DL_ADC12_isPowerEnabled(BATTERY_ADC_INST)) {
        return false;
    }

    DL_ADC12_clearInterruptStatus(
        BATTERY_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(BATTERY_ADC_INST);
    g_battery.initialized = 1U;
    return true;
}

void BatteryMonitor_Service(uint32_t nowMs)
{
    if (g_battery.initialized == 0U) {
        return;
    }

    if (g_battery.conversionPending != 0U) {
        if ((DL_ADC12_getRawInterruptStatus(
                 BATTERY_ADC_INST,
                 DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) &
             DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) != 0U) {
            uint16_t rawCounts = DL_ADC12_getMemResult(
                BATTERY_ADC_INST, BATTERY_ADC_ADCMEM_0);

            DL_ADC12_clearInterruptStatus(
                BATTERY_ADC_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
            g_battery.conversionPending = 0U;
            BatteryMonitor_ProcessSample(rawCounts, nowMs);
        } else if (BatteryMonitor_Elapsed(
                       nowMs, g_battery.conversionStartMs,
                       APP_BATTERY_CONVERSION_TIMEOUT_MS)) {
            DL_ADC12_disableConversions(BATTERY_ADC_INST);
            DL_ADC12_clearInterruptStatus(
                BATTERY_ADC_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
            DL_ADC12_enableConversions(BATTERY_ADC_INST);
            g_battery.conversionPending = 0U;
        }
    }

    if ((g_battery.conversionPending == 0U) &&
        ((g_battery.sampleStarted == 0U) ||
         BatteryMonitor_Elapsed(
             nowMs, g_battery.lastSampleStartMs,
             APP_BATTERY_SAMPLE_PERIOD_MS))) {
        DL_ADC12_clearInterruptStatus(
            BATTERY_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        DL_ADC12_enableConversions(BATTERY_ADC_INST);
        DL_ADC12_startConversion(BATTERY_ADC_INST);
        g_battery.sampleStarted = 1U;
        g_battery.conversionPending = 1U;
        g_battery.lastSampleStartMs = nowMs;
        g_battery.conversionStartMs = nowMs;
    }

    BatteryMonitor_UpdateWarningOutputs(nowMs);
}

void BatteryMonitor_GetSnapshot(BatteryMonitorSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    snapshot->rawCounts = g_battery.rawCounts;
    snapshot->voltageMilliVolts = g_battery.voltageMilliVolts;
    snapshot->ready = g_battery.ready;
    snapshot->lowVoltageWarning = g_battery.lowVoltageWarning;
}
