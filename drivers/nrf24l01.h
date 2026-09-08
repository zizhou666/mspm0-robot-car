#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <stdbool.h>
#include <stdint.h>

#define NRF24L01_PAYLOAD_SIZE (32U)

#define NRF24L01_EVENT_RX_READY (1U << 0)
#define NRF24L01_EVENT_TX_DONE  (1U << 1)
#define NRF24L01_EVENT_MAX_RT   (1U << 2)

typedef enum {
    NRF24L01_MODE_RX = 0,
    NRF24L01_MODE_TX
} NRF24L01_Mode;

bool NRF24L01_Init(void);
bool NRF24L01_SetMode(NRF24L01_Mode mode);
bool NRF24L01_StartTransmit(
    const uint8_t payload[NRF24L01_PAYLOAD_SIZE]);
uint8_t NRF24L01_GetAndClearEvents(void);
bool NRF24L01_IsRxFifoEmpty(void);
bool NRF24L01_ReadPayload(uint8_t payload[NRF24L01_PAYLOAD_SIZE]);
void NRF24L01_AbortTransmit(void);
uint8_t NRF24L01_GetRpd(void);
uint8_t NRF24L01_GetCeLevel(void);

void NRF24L01_NotifyIrqFromIsr(void);
bool NRF24L01_TakeIrqPending(void);

#endif /* NRF24L01_H_ */
