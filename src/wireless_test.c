#include "wireless_test.h"

#include "user_config.h"

#include <limits.h>
#include <stddef.h>

#define WIRELESS_PACKET_HELLO      (1U)
#define WIRELESS_PACKET_HEARTBEAT  (2U)
#define WIRELESS_PACKET_COMMAND    (3U)

typedef struct {
    WirelessTestMode activeMode;
    WirelessTestMode requestedMode;
    WirelessTestState state;
    WirelessTestTxFailure lastTxFailure;
    WirelessLinkStatus linkStatus;
    WirelessSendStatus sendStatus;
    WirelessRemoteCommand txCommand;
    WirelessRemoteCommand rxCommand;
    WirelessRemoteCommand queuedCommand;
    uint16_t txValue;
    uint16_t rxValue;
    uint16_t queuedValue;
    uint16_t nextSequence;
    uint16_t lastSequence;
    uint16_t lastCommandSequence;
    uint32_t lastInitAttemptMs;
    uint32_t lastStatusPollMs;
    uint32_t switchStartedMs;
    uint32_t modeEnteredMs;
    uint32_t txStartedMs;
    uint32_t nextRetryMs;
    uint32_t nextHeartbeatMs;
    uint32_t nextReconnectMs;
    uint32_t lastActivityMs;
    uint32_t packetGeneration;
    uint32_t commandGeneration;
    uint32_t rxEventCount;
    uint32_t rxCount;
    uint32_t txSuccessCount;
    uint32_t txFailureCount;
    uint8_t txPacket[NRF24L01_PAYLOAD_SIZE];
    uint8_t latestPacket[NRF24L01_PAYLOAD_SIZE];
    uint8_t initialized;
    uint8_t sessionActive;
    uint8_t switchPending;
    uint8_t switchSettling;
    uint8_t txPending;
    uint8_t transactionActive;
    uint8_t retryScheduled;
    uint8_t retryAttempt;
    uint8_t queuedCommandValid;
    uint8_t activityValid;
    uint8_t sequenceValid;
    uint8_t lastCommandSequenceValid;
    uint8_t everConnected;
    uint8_t rpdDetected;
} WirelessTestContext;

static WirelessTestContext g_wireless;

static bool WirelessTest_Elapsed(
    uint32_t now, uint32_t then, uint32_t interval)
{
    return (uint32_t)(now - then) >= interval;
}

static bool WirelessTest_TimeReached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static void WirelessTest_Increment(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        ++(*value);
    }
}

static bool WirelessTest_CommandValid(WirelessRemoteCommand command)
{
    return (command >= WIRELESS_COMMAND_STOP) &&
           (command <= WIRELESS_COMMAND_TURN_RIGHT);
}

static bool WirelessTest_IsProtocolPacket(
    const uint8_t packet[NRF24L01_PAYLOAD_SIZE])
{
    return (packet[0] == (uint8_t)'N') &&
           (packet[1] == (uint8_t)'R') &&
           (packet[2] == (uint8_t)'F') &&
           (packet[3] == (uint8_t)'2') &&
           (packet[4] == APP_NRF24_TEST_PROTOCOL_VERSION) &&
           (packet[5] >= WIRELESS_PACKET_HELLO) &&
           (packet[5] <= WIRELESS_PACKET_COMMAND);
}

static void WirelessTest_RecordPacket(
    const uint8_t packet[NRF24L01_PAYLOAD_SIZE], uint32_t nowMs)
{
    uint16_t sequence;

    for (uint8_t i = 0U; i < NRF24L01_PAYLOAD_SIZE; ++i) {
        g_wireless.latestPacket[i] = packet[i];
    }
    WirelessTest_Increment(&g_wireless.packetGeneration);
    WirelessTest_Increment(&g_wireless.rxCount);
    if (NRF24L01_GetRpd() != 0U) {
        g_wireless.rpdDetected = 1U;
    }

    if (!WirelessTest_IsProtocolPacket(packet)) {
        g_wireless.sequenceValid = 0U;
        return;
    }

    sequence = (uint16_t)packet[6] | ((uint16_t)packet[7] << 8U);
    g_wireless.lastSequence = sequence;
    g_wireless.sequenceValid = 1U;
    g_wireless.lastActivityMs = nowMs;
    g_wireless.activityValid = 1U;
    g_wireless.everConnected = 1U;
    g_wireless.linkStatus = WIRELESS_LINK_OK;
    g_wireless.state = WIRELESS_TEST_STATE_RX_READY;

    if (packet[5] == WIRELESS_PACKET_COMMAND) {
        WirelessRemoteCommand command = (WirelessRemoteCommand)packet[8];
        uint16_t value =
            (uint16_t)packet[10] | ((uint16_t)packet[11] << 8U);

        if (WirelessTest_CommandValid(command) &&
            ((g_wireless.lastCommandSequenceValid == 0U) ||
             (sequence != g_wireless.lastCommandSequence))) {
            g_wireless.lastCommandSequence = sequence;
            g_wireless.lastCommandSequenceValid = 1U;
            g_wireless.rxCommand = command;
            g_wireless.rxValue = value;
            WirelessTest_Increment(&g_wireless.commandGeneration);
        }
    }
}

static void WirelessTest_BuildPacket(uint8_t packetType,
    WirelessRemoteCommand command, uint16_t value)
{
    uint16_t sequence = g_wireless.nextSequence++;

    g_wireless.txPacket[0] = (uint8_t)'N';
    g_wireless.txPacket[1] = (uint8_t)'R';
    g_wireless.txPacket[2] = (uint8_t)'F';
    g_wireless.txPacket[3] = (uint8_t)'2';
    g_wireless.txPacket[4] = APP_NRF24_TEST_PROTOCOL_VERSION;
    g_wireless.txPacket[5] = packetType;
    g_wireless.txPacket[6] = (uint8_t)(sequence & 0xFFU);
    g_wireless.txPacket[7] = (uint8_t)(sequence >> 8U);
    g_wireless.txPacket[8] = (uint8_t)command;
    g_wireless.txPacket[9] = 0U;
    g_wireless.txPacket[10] = (uint8_t)(value & 0xFFU);
    g_wireless.txPacket[11] = (uint8_t)(value >> 8U);
    for (uint8_t i = 12U; i < NRF24L01_PAYLOAD_SIZE; ++i) {
        g_wireless.txPacket[i] = (uint8_t)(
            APP_NRF24_TEST_PATTERN ^ i ^ (uint8_t)sequence);
    }
}

static void WirelessTest_HandleAttemptFailure(
    WirelessTestTxFailure failure, uint32_t nowMs)
{
    g_wireless.txPending = 0U;
    g_wireless.lastTxFailure = failure;
    NRF24L01_AbortTransmit();
    WirelessTest_Increment(&g_wireless.txFailureCount);

    if (g_wireless.retryAttempt < APP_NRF24_APP_SEND_ATTEMPTS) {
        g_wireless.retryScheduled = 1U;
        g_wireless.nextRetryMs = nowMs + APP_NRF24_APP_RETRY_INTERVAL_MS;
        g_wireless.sendStatus = WIRELESS_SEND_RETRYING;
        g_wireless.state = WIRELESS_TEST_STATE_TX_SENDING;
        return;
    }

    g_wireless.transactionActive = 0U;
    g_wireless.retryScheduled = 0U;
    g_wireless.activityValid = 0U;
    g_wireless.linkStatus = (g_wireless.everConnected != 0U) ?
        WIRELESS_LINK_LOST : WIRELESS_LINK_FIRST_FAILED;
    g_wireless.sendStatus = WIRELESS_SEND_FAILED;
    g_wireless.state = WIRELESS_TEST_STATE_TX_FAILED;
    g_wireless.nextReconnectMs = nowMs + APP_NRF24_RECONNECT_MS;
}

static void WirelessTest_StartAttempt(uint32_t nowMs)
{
    ++g_wireless.retryAttempt;
    g_wireless.sendStatus = (g_wireless.retryAttempt > 1U) ?
        WIRELESS_SEND_RETRYING : WIRELESS_SEND_SENDING;
    g_wireless.state = WIRELESS_TEST_STATE_TX_SENDING;
    g_wireless.lastTxFailure = WIRELESS_TEST_TX_FAILURE_NONE;
    if (!NRF24L01_StartTransmit(g_wireless.txPacket)) {
        WirelessTest_HandleAttemptFailure(
            WIRELESS_TEST_TX_FAILURE_START, nowMs);
        return;
    }
    g_wireless.txPending = 1U;
    g_wireless.txStartedMs = nowMs;
}

static void WirelessTest_BeginTransaction(uint8_t packetType,
    WirelessRemoteCommand command, uint16_t value, uint32_t nowMs)
{
    WirelessTest_BuildPacket(packetType, command, value);
    g_wireless.retryAttempt = 0U;
    g_wireless.transactionActive = 1U;
    g_wireless.retryScheduled = 0U;
    WirelessTest_StartAttempt(nowMs);
}

static void WirelessTest_AttemptInit(uint32_t nowMs)
{
    g_wireless.lastInitAttemptMs = nowMs;
    g_wireless.state = WIRELESS_TEST_STATE_INIT;
    if (NRF24L01_Init()) {
        g_wireless.initialized = 1U;
        g_wireless.activeMode = WIRELESS_TEST_MODE_RX;
        g_wireless.state = WIRELESS_TEST_STATE_RX_READY;
        g_wireless.linkStatus = WIRELESS_LINK_WAITING;
        g_wireless.lastStatusPollMs = nowMs;
        if (g_wireless.sessionActive != 0U) {
            g_wireless.modeEnteredMs = nowMs;
            if (g_wireless.requestedMode == WIRELESS_TEST_MODE_TX) {
                g_wireless.switchPending = 1U;
            }
        }
    } else {
        g_wireless.initialized = 0U;
        g_wireless.state = WIRELESS_TEST_STATE_INIT_FAILED;
        g_wireless.linkStatus = WIRELESS_LINK_INIT_FAILED;
    }
}

static void WirelessTest_ProcessRadioEvents(uint32_t nowMs)
{
    uint8_t events = NRF24L01_GetAndClearEvents();
    uint8_t packet[NRF24L01_PAYLOAD_SIZE];
    uint8_t drained = 0U;

    if ((events & NRF24L01_EVENT_RX_READY) != 0U) {
        WirelessTest_Increment(&g_wireless.rxEventCount);
    }
    while ((drained < APP_NRF24_RX_FIFO_DEPTH) &&
           !NRF24L01_IsRxFifoEmpty()) {
        if (!NRF24L01_ReadPayload(packet)) {
            break;
        }
        WirelessTest_RecordPacket(packet, nowMs);
        ++drained;
    }

    if (((events & NRF24L01_EVENT_TX_DONE) != 0U) &&
        (g_wireless.txPending != 0U)) {
        g_wireless.txPending = 0U;
        g_wireless.transactionActive = 0U;
        g_wireless.retryScheduled = 0U;
        g_wireless.lastTxFailure = WIRELESS_TEST_TX_FAILURE_NONE;
        NRF24L01_AbortTransmit();
        WirelessTest_Increment(&g_wireless.txSuccessCount);
        g_wireless.lastActivityMs = nowMs;
        g_wireless.activityValid = 1U;
        g_wireless.everConnected = 1U;
        g_wireless.linkStatus = WIRELESS_LINK_OK;
        g_wireless.sendStatus = WIRELESS_SEND_OK;
        g_wireless.state = WIRELESS_TEST_STATE_TX_SUCCESS;
        g_wireless.nextHeartbeatMs = nowMs + APP_NRF24_HEARTBEAT_MS;
    } else if (((events & NRF24L01_EVENT_MAX_RT) != 0U) &&
               (g_wireless.txPending != 0U)) {
        WirelessTest_HandleAttemptFailure(
            WIRELESS_TEST_TX_FAILURE_MAX_RT, nowMs);
    }
}

void WirelessTest_Init(uint32_t nowMs)
{
    g_wireless = (WirelessTestContext){0};
    g_wireless.activeMode = WIRELESS_TEST_MODE_RX;
    g_wireless.requestedMode = WIRELESS_TEST_MODE_RX;
    g_wireless.state = WIRELESS_TEST_STATE_INIT;
    g_wireless.linkStatus = WIRELESS_LINK_WAITING;
    WirelessTest_AttemptInit(nowMs);
}

void WirelessTest_Service(uint32_t nowMs)
{
    bool radioEvent;

    if (g_wireless.initialized == 0U) {
        if (WirelessTest_Elapsed(nowMs, g_wireless.lastInitAttemptMs,
                                APP_NRF24_INIT_RETRY_MS)) {
            WirelessTest_AttemptInit(nowMs);
        }
        return;
    }

    if (g_wireless.switchPending != 0U) {
        NRF24L01_Mode radioMode =
            (g_wireless.requestedMode == WIRELESS_TEST_MODE_RX) ?
                NRF24L01_MODE_RX : NRF24L01_MODE_TX;

        g_wireless.switchPending = 0U;
        g_wireless.txPending = 0U;
        g_wireless.transactionActive = 0U;
        g_wireless.retryScheduled = 0U;
        g_wireless.queuedCommandValid = 0U;
        NRF24L01_AbortTransmit();
        g_wireless.activityValid = 0U;
        g_wireless.everConnected = 0U;
        g_wireless.rpdDetected = 0U;
        g_wireless.linkStatus = WIRELESS_LINK_WAITING;
        g_wireless.sendStatus = WIRELESS_SEND_IDLE;
        g_wireless.state = WIRELESS_TEST_STATE_SWITCHING;
        g_wireless.modeEnteredMs = nowMs;
        if (NRF24L01_SetMode(radioMode)) {
            g_wireless.activeMode = g_wireless.requestedMode;
            g_wireless.switchStartedMs = nowMs;
            g_wireless.switchSettling = 1U;
        } else {
            g_wireless.switchSettling = 0U;
            g_wireless.state = WIRELESS_TEST_STATE_SWITCH_FAILED;
            g_wireless.linkStatus = WIRELESS_LINK_INIT_FAILED;
        }
    }

    if ((g_wireless.switchSettling != 0U) &&
        WirelessTest_Elapsed(nowMs, g_wireless.switchStartedMs,
                             APP_NRF24_MODE_SETTLE_MS)) {
        g_wireless.switchSettling = 0U;
        if (g_wireless.activeMode == WIRELESS_TEST_MODE_RX) {
            g_wireless.state = WIRELESS_TEST_STATE_RX_READY;
        } else {
            g_wireless.state = WIRELESS_TEST_STATE_TX_READY;
            WirelessTest_BeginTransaction(WIRELESS_PACKET_HELLO,
                WIRELESS_COMMAND_NONE, 0U, nowMs);
        }
    }

    radioEvent = NRF24L01_TakeIrqPending();
    if (radioEvent ||
        WirelessTest_Elapsed(nowMs, g_wireless.lastStatusPollMs,
                             APP_NRF24_STATUS_POLL_MS) ||
        (g_wireless.txPending != 0U)) {
        g_wireless.lastStatusPollMs = nowMs;
        WirelessTest_ProcessRadioEvents(nowMs);
    }

    if ((g_wireless.txPending != 0U) &&
        WirelessTest_Elapsed(nowMs, g_wireless.txStartedMs,
                             APP_NRF24_TX_TIMEOUT_MS)) {
        WirelessTest_HandleAttemptFailure(
            WIRELESS_TEST_TX_FAILURE_TIMEOUT, nowMs);
    }

    if ((g_wireless.retryScheduled != 0U) &&
        WirelessTest_TimeReached(nowMs, g_wireless.nextRetryMs)) {
        g_wireless.retryScheduled = 0U;
        WirelessTest_StartAttempt(nowMs);
    }

    if ((g_wireless.sessionActive != 0U) &&
        (g_wireless.activeMode == WIRELESS_TEST_MODE_RX) &&
        (g_wireless.switchSettling == 0U)) {
        if (NRF24L01_GetRpd() != 0U) {
            g_wireless.rpdDetected = 1U;
        }
        if ((g_wireless.everConnected != 0U) &&
            WirelessTest_Elapsed(nowMs, g_wireless.lastActivityMs,
                                 APP_NRF24_LINK_TIMEOUT_MS)) {
            g_wireless.activityValid = 0U;
            g_wireless.linkStatus = WIRELESS_LINK_LOST;
        } else if ((g_wireless.everConnected == 0U) &&
            WirelessTest_Elapsed(nowMs, g_wireless.modeEnteredMs,
                                 APP_NRF24_FIRST_LINK_TIMEOUT_MS)) {
            g_wireless.linkStatus = WIRELESS_LINK_FIRST_FAILED;
        }
    }

    if ((g_wireless.sessionActive != 0U) &&
        (g_wireless.activeMode == WIRELESS_TEST_MODE_TX) &&
        (g_wireless.switchSettling == 0U) &&
        (g_wireless.transactionActive == 0U) &&
        (g_wireless.txPending == 0U)) {
        if (g_wireless.queuedCommandValid != 0U) {
            WirelessRemoteCommand command = g_wireless.queuedCommand;
            uint16_t value = g_wireless.queuedValue;
            g_wireless.queuedCommandValid = 0U;
            WirelessTest_BeginTransaction(WIRELESS_PACKET_COMMAND,
                                           command, value, nowMs);
        } else if ((g_wireless.linkStatus != WIRELESS_LINK_OK) &&
                   WirelessTest_TimeReached(
                       nowMs, g_wireless.nextReconnectMs)) {
            WirelessTest_BeginTransaction(WIRELESS_PACKET_HELLO,
                WIRELESS_COMMAND_NONE, 0U, nowMs);
        } else if ((g_wireless.linkStatus == WIRELESS_LINK_OK) &&
                   WirelessTest_TimeReached(
                       nowMs, g_wireless.nextHeartbeatMs)) {
            WirelessTest_BeginTransaction(WIRELESS_PACKET_HEARTBEAT,
                WIRELESS_COMMAND_NONE, 0U, nowMs);
        }
    }
}

bool WirelessTest_RequestMode(WirelessTestMode mode)
{
    if ((mode != WIRELESS_TEST_MODE_RX) &&
        (mode != WIRELESS_TEST_MODE_TX)) {
        return false;
    }
    g_wireless.requestedMode = mode;
    g_wireless.sessionActive = 1U;
    g_wireless.linkStatus = (g_wireless.initialized != 0U) ?
        WIRELESS_LINK_WAITING : WIRELESS_LINK_INIT_FAILED;
    g_wireless.sendStatus = WIRELESS_SEND_IDLE;
    if (g_wireless.initialized == 0U) {
        return false;
    }
    g_wireless.switchPending = 1U;
    g_wireless.state = WIRELESS_TEST_STATE_SWITCHING;
    return true;
}

void WirelessTest_LeaveMode(void)
{
    g_wireless.sessionActive = 0U;
    g_wireless.txPending = 0U;
    g_wireless.transactionActive = 0U;
    g_wireless.retryScheduled = 0U;
    g_wireless.queuedCommandValid = 0U;
    NRF24L01_AbortTransmit();
    if (g_wireless.initialized != 0U) {
        (void)NRF24L01_SetMode(NRF24L01_MODE_RX);
    }
    g_wireless.activeMode = WIRELESS_TEST_MODE_RX;
    g_wireless.requestedMode = WIRELESS_TEST_MODE_RX;
    g_wireless.state = WIRELESS_TEST_STATE_RX_READY;
    g_wireless.linkStatus = WIRELESS_LINK_WAITING;
    g_wireless.sendStatus = WIRELESS_SEND_IDLE;
    g_wireless.activityValid = 0U;
    g_wireless.everConnected = 0U;
}

bool WirelessTest_RequestTransmit(uint32_t nowMs)
{
    if ((g_wireless.initialized == 0U) ||
        (g_wireless.sessionActive == 0U) ||
        (g_wireless.activeMode != WIRELESS_TEST_MODE_TX)) {
        return false;
    }
    if ((g_wireless.transactionActive == 0U) &&
        (g_wireless.switchSettling == 0U)) {
        WirelessTest_BeginTransaction(WIRELESS_PACKET_HELLO,
            WIRELESS_COMMAND_NONE, 0U, nowMs);
    }
    return true;
}

bool WirelessTest_RequestCommand(WirelessRemoteCommand command,
                                 uint16_t value, uint32_t nowMs)
{
    if (!WirelessTest_CommandValid(command) ||
        (g_wireless.initialized == 0U) ||
        (g_wireless.sessionActive == 0U) ||
        (g_wireless.activeMode != WIRELESS_TEST_MODE_TX)) {
        return false;
    }

    g_wireless.txCommand = command;
    g_wireless.txValue = value;
    if ((g_wireless.transactionActive != 0U) ||
        (g_wireless.txPending != 0U) ||
        (g_wireless.switchSettling != 0U)) {
        g_wireless.queuedCommand = command;
        g_wireless.queuedValue = value;
        g_wireless.queuedCommandValid = 1U;
        return true;
    }

    WirelessTest_BeginTransaction(WIRELESS_PACKET_COMMAND,
                                   command, value, nowMs);
    return true;
}

void WirelessTest_GetSnapshot(
    uint32_t nowMs, WirelessTestSnapshot *snapshot)
{
    uint32_t age;

    if (snapshot == NULL) {
        return;
    }

    age = (g_wireless.activityValid != 0U) ?
        (uint32_t)(nowMs - g_wireless.lastActivityMs) : UINT32_MAX;
    snapshot->initialized = g_wireless.initialized;
    snapshot->sessionActive = g_wireless.sessionActive;
    snapshot->connected =
        (g_wireless.linkStatus == WIRELESS_LINK_OK) ? 1U : 0U;
    snapshot->sequenceValid = g_wireless.sequenceValid;
    snapshot->rpdDetected = g_wireless.rpdDetected;
    snapshot->ceHigh = NRF24L01_GetCeLevel();
    snapshot->activeMode = g_wireless.activeMode;
    snapshot->state = g_wireless.state;
    snapshot->lastTxFailure = g_wireless.lastTxFailure;
    snapshot->linkStatus = g_wireless.linkStatus;
    snapshot->sendStatus = g_wireless.sendStatus;
    snapshot->txCommand = g_wireless.txCommand;
    snapshot->rxCommand = g_wireless.rxCommand;
    snapshot->txValue = g_wireless.txValue;
    snapshot->rxValue = g_wireless.rxValue;
    snapshot->retryAttempt = g_wireless.retryAttempt;
    snapshot->commandGeneration = g_wireless.commandGeneration;
    snapshot->rxEventCount = g_wireless.rxEventCount;
    snapshot->rxCount = g_wireless.rxCount;
    snapshot->txSuccessCount = g_wireless.txSuccessCount;
    snapshot->txFailureCount = g_wireless.txFailureCount;
    snapshot->lastSequence = g_wireless.lastSequence;
    snapshot->lastActivityAgeMs = age;
    for (uint8_t i = 0U; i < 4U; ++i) {
        snapshot->dataPrefix[i] = g_wireless.latestPacket[i];
    }
}

bool WirelessTest_CopyLatestPacket(
    uint8_t packet[NRF24L01_PAYLOAD_SIZE], uint32_t *generation)
{
    if ((packet == NULL) || (generation == NULL) ||
        (g_wireless.packetGeneration == 0U)) {
        return false;
    }
    for (uint8_t i = 0U; i < NRF24L01_PAYLOAD_SIZE; ++i) {
        packet[i] = g_wireless.latestPacket[i];
    }
    *generation = g_wireless.packetGeneration;
    return true;
}
