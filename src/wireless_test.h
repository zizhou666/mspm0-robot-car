#ifndef WIRELESS_TEST_H_
#define WIRELESS_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#include "nrf24l01.h"

typedef enum {
    WIRELESS_TEST_MODE_RX = 0,
    WIRELESS_TEST_MODE_TX
} WirelessTestMode;

typedef enum {
    WIRELESS_TEST_STATE_INIT = 0,
    WIRELESS_TEST_STATE_INIT_FAILED,
    WIRELESS_TEST_STATE_SWITCHING,
    WIRELESS_TEST_STATE_SWITCH_FAILED,
    WIRELESS_TEST_STATE_RX_READY,
    WIRELESS_TEST_STATE_TX_READY,
    WIRELESS_TEST_STATE_TX_SENDING,
    WIRELESS_TEST_STATE_TX_SUCCESS,
    WIRELESS_TEST_STATE_TX_FAILED
} WirelessTestState;

typedef enum {
    WIRELESS_TEST_TX_FAILURE_NONE = 0,
    WIRELESS_TEST_TX_FAILURE_START,
    WIRELESS_TEST_TX_FAILURE_MAX_RT,
    WIRELESS_TEST_TX_FAILURE_TIMEOUT
} WirelessTestTxFailure;

typedef enum {
    WIRELESS_LINK_WAITING = 0,
    WIRELESS_LINK_OK,
    WIRELESS_LINK_LOST,
    WIRELESS_LINK_FIRST_FAILED,
    WIRELESS_LINK_INIT_FAILED
} WirelessLinkStatus;

typedef enum {
    WIRELESS_SEND_IDLE = 0,
    WIRELESS_SEND_SENDING,
    WIRELESS_SEND_RETRYING,
    WIRELESS_SEND_OK,
    WIRELESS_SEND_FAILED
} WirelessSendStatus;

typedef enum {
    WIRELESS_COMMAND_NONE = 0,
    WIRELESS_COMMAND_STOP,
    WIRELESS_COMMAND_FORWARD,
    WIRELESS_COMMAND_BACKWARD,
    WIRELESS_COMMAND_TURN_LEFT,
    WIRELESS_COMMAND_TURN_RIGHT
} WirelessRemoteCommand;

typedef struct {
    uint8_t initialized;
    uint8_t sessionActive;
    uint8_t connected;
    uint8_t sequenceValid;
    uint8_t rpdDetected;
    uint8_t ceHigh;
    WirelessTestMode activeMode;
    WirelessTestState state;
    WirelessTestTxFailure lastTxFailure;
    WirelessLinkStatus linkStatus;
    WirelessSendStatus sendStatus;
    WirelessRemoteCommand txCommand;
    WirelessRemoteCommand rxCommand;
    uint16_t txValue;
    uint16_t rxValue;
    uint8_t retryAttempt;
    uint32_t commandGeneration;
    uint32_t rxEventCount;
    uint32_t rxCount;
    uint32_t txSuccessCount;
    uint32_t txFailureCount;
    uint16_t lastSequence;
    uint32_t lastActivityAgeMs;
    uint8_t dataPrefix[4];
} WirelessTestSnapshot;

void WirelessTest_Init(uint32_t nowMs);
void WirelessTest_Service(uint32_t nowMs);
bool WirelessTest_RequestMode(WirelessTestMode mode);
void WirelessTest_LeaveMode(void);
bool WirelessTest_RequestTransmit(uint32_t nowMs);
bool WirelessTest_RequestCommand(WirelessRemoteCommand command,
                                 uint16_t value, uint32_t nowMs);
void WirelessTest_GetSnapshot(
    uint32_t nowMs, WirelessTestSnapshot *snapshot);
bool WirelessTest_CopyLatestPacket(
    uint8_t packet[NRF24L01_PAYLOAD_SIZE], uint32_t *generation);

#endif /* WIRELESS_TEST_H_ */
