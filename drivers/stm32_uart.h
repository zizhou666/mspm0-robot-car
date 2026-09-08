#ifndef STM32_UART_H
#define STM32_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PA8=TX, PA9=RX; 115200 baud, 8-N-1, no flow control. */
#define STM32_UART_TELEMETRY_PACKET_SIZE (19U)

typedef struct {
    int16_t accel_y_mg;
    int16_t pitch_cdeg;
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
    int16_t target_speed_mm_s;
    int32_t distance_mm;
    uint8_t phase;
} STM32UARTTelemetry;

void MaixCamUART_Init(void);
bool MaixCamUART_ReadByte(uint8_t *data);
size_t MaixCamUART_Read(uint8_t *data, size_t capacity);
bool MaixCamUART_TryWrite(const uint8_t *data, size_t length);
void MaixCamUART_Write(const uint8_t *data, size_t length);
void MaixCamUART_WriteString(const char *text);
uint32_t MaixCamUART_GetDroppedRxCount(void);

/*
 * Queue one 19-byte little-endian telemetry frame:
 * AA 56 SEQ AY PITCH LEFT RIGHT TARGET DISTANCE PHASE XOR.
 */
bool STM32_UART_TrySendTelemetry(const STM32UARTTelemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif
