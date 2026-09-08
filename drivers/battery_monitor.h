#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t rawCounts;
    uint32_t voltageMilliVolts;
    uint8_t ready;
    uint8_t lowVoltageWarning;
} BatteryMonitorSnapshot;

bool BatteryMonitor_Init(void);
void BatteryMonitor_Service(uint32_t nowMs);
void BatteryMonitor_GetSnapshot(BatteryMonitorSnapshot *snapshot);

#endif /* BATTERY_MONITOR_H_ */
