#include "stm32_uart.h"

#include "ti_msp_dl_config.h"

#define MAIXCAM_UART_RX_BUFFER_SIZE (128U)
#define MAIXCAM_UART_RX_BUFFER_MASK (MAIXCAM_UART_RX_BUFFER_SIZE - 1U)
#define MAIXCAM_UART_TX_BUFFER_SIZE (64U)
#define MAIXCAM_UART_TX_BUFFER_MASK (MAIXCAM_UART_TX_BUFFER_SIZE - 1U)

static volatile uint16_t g_rxWriteIndex;
static volatile uint16_t g_rxReadIndex;
static volatile uint32_t g_rxDroppedCount;
static uint8_t g_rxBuffer[MAIXCAM_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_txWriteIndex;
static volatile uint16_t g_txReadIndex;
static uint8_t g_txBuffer[MAIXCAM_UART_TX_BUFFER_SIZE];
static uint8_t g_telemetrySequence;

static void WriteInt16LE(uint8_t *destination, int16_t value)
{
    uint16_t encoded = (uint16_t)value;

    destination[0] = (uint8_t)(encoded & 0xFFU);
    destination[1] = (uint8_t)(encoded >> 8U);
}

static void WriteInt32LE(uint8_t *destination, int32_t value)
{
    uint32_t encoded = (uint32_t)value;

    destination[0] = (uint8_t)(encoded & 0xFFU);
    destination[1] = (uint8_t)((encoded >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((encoded >> 16U) & 0xFFU);
    destination[3] = (uint8_t)(encoded >> 24U);
}

static uint32_t EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

void MaixCamUART_Init(void)
{
    g_rxWriteIndex = 0U;
    g_rxReadIndex = 0U;
    g_rxDroppedCount = 0U;
    g_txWriteIndex = 0U;
    g_txReadIndex = 0U;
    g_telemetrySequence = 0U;

    NVIC_SetPriority(UART_1_INST_INT_IRQN, 2U);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

bool MaixCamUART_ReadByte(uint8_t *data)
{
    uint16_t readIndex;

    if (data == NULL) {
        return false;
    }

    readIndex = g_rxReadIndex;
    if (readIndex == g_rxWriteIndex) {
        return false;
    }

    *data = g_rxBuffer[readIndex];
    g_rxReadIndex = (uint16_t)((readIndex + 1U) & MAIXCAM_UART_RX_BUFFER_MASK);
    return true;
}

size_t MaixCamUART_Read(uint8_t *data, size_t capacity)
{
    size_t count = 0U;

    if (data == NULL) {
        return 0U;
    }

    while ((count < capacity) && MaixCamUART_ReadByte(&data[count])) {
        ++count;
    }
    return count;
}

bool MaixCamUART_TryWrite(const uint8_t *data, size_t length)
{
    uint32_t primask;
    uint16_t freeSpace;
    uint16_t writeIndex;
    size_t index;

    if ((data == NULL) || (length == 0U) ||
        (length >= MAIXCAM_UART_TX_BUFFER_SIZE)) {
        return false;
    }

    primask = EnterCritical();
    writeIndex = g_txWriteIndex;
    freeSpace = (uint16_t)(
        (g_txReadIndex - writeIndex - 1U) & MAIXCAM_UART_TX_BUFFER_MASK);
    if (length > (size_t)freeSpace) {
        ExitCritical(primask);
        return false;
    }

    for (index = 0U; index < length; ++index) {
        g_txBuffer[writeIndex] = data[index];
        writeIndex = (uint16_t)(
            (writeIndex + 1U) & MAIXCAM_UART_TX_BUFFER_MASK);
    }
    g_txWriteIndex = writeIndex;
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_TX);
    ExitCritical(primask);
    return true;
}

void MaixCamUART_Write(const uint8_t *data, size_t length)
{
    size_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0U; index < length; ++index) {
        DL_UART_Main_transmitDataBlocking(UART_1_INST, data[index]);
    }
}

void MaixCamUART_WriteString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_1_INST, (uint8_t)*text);
        ++text;
    }
}

uint32_t MaixCamUART_GetDroppedRxCount(void)
{
    return g_rxDroppedCount;
}

bool STM32_UART_TrySendTelemetry(const STM32UARTTelemetry *telemetry)
{
    uint8_t packet[STM32_UART_TELEMETRY_PACKET_SIZE];
    uint8_t checksum;
    uint8_t index;
    bool queued;

    if (telemetry == NULL) {
        return false;
    }

    packet[0] = 0xAAU;
    packet[1] = 0x56U;
    packet[2] = g_telemetrySequence;
    WriteInt16LE(&packet[3], telemetry->accel_y_mg);
    WriteInt16LE(&packet[5], telemetry->pitch_cdeg);
    WriteInt16LE(&packet[7], telemetry->left_speed_mm_s);
    WriteInt16LE(&packet[9], telemetry->right_speed_mm_s);
    WriteInt16LE(&packet[11], telemetry->target_speed_mm_s);
    WriteInt32LE(&packet[13], telemetry->distance_mm);
    packet[17] = telemetry->phase;

    checksum = packet[0];
    for (index = 1U; index < 18U; ++index) {
        checksum ^= packet[index];
    }
    packet[18] = checksum;

    queued = MaixCamUART_TryWrite(packet, sizeof(packet));
    ++g_telemetrySequence;
    return queued;
}

void UART_1_INST_IRQHandler(void)
{
    uint32_t iidx;
    uint16_t nextWriteIndex;
    uint8_t received;

    do {
        iidx = (uint32_t)DL_UART_Main_getPendingInterrupt(UART_1_INST);

        if (iidx == (uint32_t)DL_UART_MAIN_IIDX_RX) {
            while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
                received = (uint8_t)DL_UART_Main_receiveData(UART_1_INST);
                nextWriteIndex = (uint16_t)(
                    (g_rxWriteIndex + 1U) & MAIXCAM_UART_RX_BUFFER_MASK);

                if (nextWriteIndex == g_rxReadIndex) {
                    ++g_rxDroppedCount;
                } else {
                    g_rxBuffer[g_rxWriteIndex] = received;
                    g_rxWriteIndex = nextWriteIndex;
                }
            }
        } else if (iidx == (uint32_t)DL_UART_MAIN_IIDX_TX) {
            while ((g_txReadIndex != g_txWriteIndex) &&
                   (!DL_UART_Main_isTXFIFOFull(UART_1_INST))) {
                DL_UART_Main_transmitData(
                    UART_1_INST, g_txBuffer[g_txReadIndex]);
                g_txReadIndex = (uint16_t)(
                    (g_txReadIndex + 1U) & MAIXCAM_UART_TX_BUFFER_MASK);
            }

            if (g_txReadIndex == g_txWriteIndex) {
                DL_UART_Main_disableInterrupt(
                    UART_1_INST, DL_UART_MAIN_INTERRUPT_TX);
            }
        }
    } while (iidx != (uint32_t)DL_UART_MAIN_IIDX_NO_INTERRUPT);
}
