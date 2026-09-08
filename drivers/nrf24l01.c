#include "nrf24l01.h"

#include "ti_msp_dl_config.h"
#include "user_config.h"

#include <stddef.h>

#define NRF24_CMD_R_REGISTER    (0x00U)
#define NRF24_CMD_W_REGISTER    (0x20U)
#define NRF24_CMD_R_RX_PAYLOAD  (0x61U)
#define NRF24_CMD_W_TX_PAYLOAD  (0xA0U)
#define NRF24_CMD_FLUSH_TX      (0xE1U)
#define NRF24_CMD_FLUSH_RX      (0xE2U)
#define NRF24_CMD_NOP           (0xFFU)
#define NRF24_REGISTER_MASK     (0x1FU)

#define NRF24_REG_CONFIG        (0x00U)
#define NRF24_REG_EN_AA         (0x01U)
#define NRF24_REG_EN_RXADDR     (0x02U)
#define NRF24_REG_SETUP_AW      (0x03U)
#define NRF24_REG_SETUP_RETR    (0x04U)
#define NRF24_REG_RF_CH         (0x05U)
#define NRF24_REG_RF_SETUP      (0x06U)
#define NRF24_REG_STATUS        (0x07U)
#define NRF24_REG_RPD           (0x09U)
#define NRF24_REG_RX_ADDR_P0    (0x0AU)
#define NRF24_REG_TX_ADDR       (0x10U)
#define NRF24_REG_RX_PW_P0      (0x11U)
#define NRF24_REG_FIFO_STATUS   (0x17U)
#define NRF24_REG_DYNPD         (0x1CU)
#define NRF24_REG_FEATURE       (0x1DU)

#define NRF24_CONFIG_PRIM_RX    (1U << 0)
#define NRF24_CONFIG_PWR_UP     (1U << 1)
#define NRF24_CONFIG_CRCO       (1U << 2)
#define NRF24_CONFIG_EN_CRC     (1U << 3)

#define NRF24_STATUS_MAX_RT     (1U << 4)
#define NRF24_STATUS_TX_DS      (1U << 5)
#define NRF24_STATUS_RX_DR      (1U << 6)
#define NRF24_STATUS_IRQ_MASK   \
    (NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)

#define NRF24_FIFO_RX_EMPTY     (1U << 0)
#define NRF24_FIFO_TX_EMPTY     (1U << 4)
#define NRF24_RPD_DETECTED      (1U << 0)

#define NRF24_SETUP_AW_5_BYTES  (0x03U)
#define NRF24_RF_SETUP_1M_0DBM  (0x06U)
#define NRF24_CONFIG_BASE       (NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO)

static volatile uint8_t g_irqPending;
static uint8_t g_initialized;
static NRF24L01_Mode g_mode;

static void NRF24L01_SpiDelay(void)
{
    for (uint32_t i = 0U; i < APP_NRF24_SPI_DELAY_CYCLES; ++i) {
        __NOP();
    }
}

static uint8_t NRF24L01_SpiTransfer(uint8_t output)
{
    uint8_t input = 0U;

    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        DL_GPIO_clearPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_SCK_PIN);
        if ((output & mask) != 0U) {
            DL_GPIO_setPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_MOSI_PIN);
        } else {
            DL_GPIO_clearPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_MOSI_PIN);
        }
        NRF24L01_SpiDelay();
        DL_GPIO_setPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_SCK_PIN);
        NRF24L01_SpiDelay();
        if ((DL_GPIO_readPins(NRF24_PINS_PORT,
                 NRF24_PINS_RADIO_MISO_PIN) &
             NRF24_PINS_RADIO_MISO_PIN) != 0U) {
            input |= mask;
        }
    }

    DL_GPIO_clearPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_SCK_PIN);
    return input;
}

static void NRF24L01_Select(void)
{
    DL_GPIO_clearPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CSN_PIN);
}

static void NRF24L01_Deselect(void)
{
    DL_GPIO_setPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CSN_PIN);
}

static uint8_t NRF24L01_Command(uint8_t command)
{
    uint8_t status;

    NRF24L01_Select();
    status = NRF24L01_SpiTransfer(command);
    NRF24L01_Deselect();
    return status;
}

static uint8_t NRF24L01_ReadRegister(uint8_t reg)
{
    uint8_t value;

    NRF24L01_Select();
    (void)NRF24L01_SpiTransfer(
        (uint8_t)(NRF24_CMD_R_REGISTER | (reg & NRF24_REGISTER_MASK)));
    value = NRF24L01_SpiTransfer(NRF24_CMD_NOP);
    NRF24L01_Deselect();
    return value;
}

static void NRF24L01_WriteRegister(uint8_t reg, uint8_t value)
{
    NRF24L01_Select();
    (void)NRF24L01_SpiTransfer(
        (uint8_t)(NRF24_CMD_W_REGISTER | (reg & NRF24_REGISTER_MASK)));
    (void)NRF24L01_SpiTransfer(value);
    NRF24L01_Deselect();
}

static void NRF24L01_WriteBuffer(
    uint8_t command, const uint8_t *data, uint8_t length)
{
    NRF24L01_Select();
    (void)NRF24L01_SpiTransfer(command);
    for (uint8_t i = 0U; i < length; ++i) {
        (void)NRF24L01_SpiTransfer(data[i]);
    }
    NRF24L01_Deselect();
}

static void NRF24L01_ReadBuffer(
    uint8_t command, uint8_t *data, uint8_t length)
{
    NRF24L01_Select();
    (void)NRF24L01_SpiTransfer(command);
    for (uint8_t i = 0U; i < length; ++i) {
        data[i] = NRF24L01_SpiTransfer(NRF24_CMD_NOP);
    }
    NRF24L01_Deselect();
}

static bool NRF24L01_VerifyRegister(uint8_t reg, uint8_t expected)
{
    return NRF24L01_ReadRegister(reg) == expected;
}

static bool NRF24L01_VerifyBuffer(
    uint8_t reg, const uint8_t *expected, uint8_t length)
{
    uint8_t actual[APP_NRF24_ADDRESS_WIDTH];

    if ((expected == NULL) ||
        (length == 0U) ||
        (length > APP_NRF24_ADDRESS_WIDTH)) {
        return false;
    }

    NRF24L01_ReadBuffer(
        (uint8_t)(NRF24_CMD_R_REGISTER | (reg & NRF24_REGISTER_MASK)),
        actual, length);
    for (uint8_t i = 0U; i < length; ++i) {
        if (actual[i] != expected[i]) {
            return false;
        }
    }
    return true;
}

static void NRF24L01_LeavePoweredDown(void)
{
    DL_GPIO_clearPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CE_PIN);
    NRF24L01_WriteRegister(NRF24_REG_CONFIG, NRF24_CONFIG_BASE);
    g_initialized = 0U;
}

bool NRF24L01_Init(void)
{
    static const uint8_t address[APP_NRF24_ADDRESS_WIDTH] = {
        APP_NRF24_ADDRESS_BYTE,
        APP_NRF24_ADDRESS_BYTE,
        APP_NRF24_ADDRESS_BYTE,
        APP_NRF24_ADDRESS_BYTE,
        APP_NRF24_ADDRESS_BYTE
    };
    const uint8_t rxConfig = (uint8_t)(
        NRF24_CONFIG_BASE | NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX);

    g_initialized = 0U;
    g_irqPending = 0U;
    g_mode = NRF24L01_MODE_RX;

    DL_GPIO_clearPins(NRF24_PINS_PORT,
        NRF24_PINS_RADIO_SCK_PIN |
        NRF24_PINS_RADIO_MOSI_PIN |
        NRF24_PINS_RADIO_CE_PIN);
    DL_GPIO_setPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CSN_PIN);

    NRF24L01_WriteRegister(NRF24_REG_CONFIG, NRF24_CONFIG_BASE);
    NRF24L01_WriteRegister(NRF24_REG_EN_AA, 0x01U);
    NRF24L01_WriteRegister(NRF24_REG_EN_RXADDR, 0x01U);
    NRF24L01_WriteRegister(NRF24_REG_SETUP_AW, NRF24_SETUP_AW_5_BYTES);
    NRF24L01_WriteRegister(
        NRF24_REG_SETUP_RETR, APP_NRF24_AUTO_RETRY_REGISTER);
    NRF24L01_WriteRegister(NRF24_REG_RF_CH, APP_NRF24_RF_CHANNEL);
    NRF24L01_WriteRegister(NRF24_REG_RF_SETUP, NRF24_RF_SETUP_1M_0DBM);
    NRF24L01_WriteRegister(NRF24_REG_DYNPD, 0x00U);
    NRF24L01_WriteRegister(NRF24_REG_FEATURE, 0x00U);
    NRF24L01_WriteBuffer(
        (uint8_t)(NRF24_CMD_W_REGISTER | NRF24_REG_RX_ADDR_P0),
        address, APP_NRF24_ADDRESS_WIDTH);
    NRF24L01_WriteBuffer(
        (uint8_t)(NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR),
        address, APP_NRF24_ADDRESS_WIDTH);
    NRF24L01_WriteRegister(
        NRF24_REG_RX_PW_P0, NRF24L01_PAYLOAD_SIZE);
    NRF24L01_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    (void)NRF24L01_Command(NRF24_CMD_FLUSH_RX);
    (void)NRF24L01_Command(NRF24_CMD_FLUSH_TX);
    NRF24L01_WriteRegister(NRF24_REG_CONFIG, rxConfig);
    delay_cycles((CPUCLK_FREQ / 500U) + 1U);

    if (!NRF24L01_VerifyRegister(NRF24_REG_CONFIG, rxConfig) ||
        !NRF24L01_VerifyRegister(NRF24_REG_EN_AA, 0x01U) ||
        !NRF24L01_VerifyRegister(NRF24_REG_EN_RXADDR, 0x01U) ||
        !NRF24L01_VerifyRegister(
            NRF24_REG_SETUP_AW, NRF24_SETUP_AW_5_BYTES) ||
        !NRF24L01_VerifyRegister(
            NRF24_REG_SETUP_RETR, APP_NRF24_AUTO_RETRY_REGISTER) ||
        !NRF24L01_VerifyRegister(
            NRF24_REG_RF_CH, APP_NRF24_RF_CHANNEL) ||
        !NRF24L01_VerifyRegister(
            NRF24_REG_RF_SETUP, NRF24_RF_SETUP_1M_0DBM) ||
        !NRF24L01_VerifyRegister(NRF24_REG_DYNPD, 0x00U) ||
        !NRF24L01_VerifyRegister(NRF24_REG_FEATURE, 0x00U) ||
        !NRF24L01_VerifyBuffer(
            NRF24_REG_RX_ADDR_P0, address, APP_NRF24_ADDRESS_WIDTH) ||
        !NRF24L01_VerifyBuffer(
            NRF24_REG_TX_ADDR, address, APP_NRF24_ADDRESS_WIDTH) ||
        !NRF24L01_VerifyRegister(
            NRF24_REG_RX_PW_P0, NRF24L01_PAYLOAD_SIZE)) {
        NRF24L01_LeavePoweredDown();
        return false;
    }

    g_initialized = 1U;
    DL_GPIO_setPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CE_PIN);
    return true;
}

bool NRF24L01_SetMode(NRF24L01_Mode mode)
{
    uint8_t config;

    if ((g_initialized == 0U) ||
        ((mode != NRF24L01_MODE_RX) && (mode != NRF24L01_MODE_TX))) {
        return false;
    }

    DL_GPIO_clearPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CE_PIN);
    config = (uint8_t)(NRF24_CONFIG_BASE | NRF24_CONFIG_PWR_UP);
    if (mode == NRF24L01_MODE_RX) {
        config |= NRF24_CONFIG_PRIM_RX;
    }
    NRF24L01_WriteRegister(NRF24_REG_CONFIG, config);
    if (!NRF24L01_VerifyRegister(NRF24_REG_CONFIG, config)) {
        return false;
    }

    NRF24L01_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    if (mode == NRF24L01_MODE_RX) {
        (void)NRF24L01_Command(NRF24_CMD_FLUSH_RX);
        DL_GPIO_setPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CE_PIN);
    } else {
        (void)NRF24L01_Command(NRF24_CMD_FLUSH_TX);
    }
    g_mode = mode;
    return true;
}

bool NRF24L01_StartTransmit(
    const uint8_t payload[NRF24L01_PAYLOAD_SIZE])
{
    const uint8_t txConfig =
        (uint8_t)(NRF24_CONFIG_BASE | NRF24_CONFIG_PWR_UP);

    if ((g_initialized == 0U) ||
        (g_mode != NRF24L01_MODE_TX) ||
        (payload == NULL)) {
        return false;
    }

    DL_GPIO_clearPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CE_PIN);
    NRF24L01_WriteRegister(
        NRF24_REG_STATUS, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    (void)NRF24L01_Command(NRF24_CMD_FLUSH_TX);
    NRF24L01_WriteBuffer(
        NRF24_CMD_W_TX_PAYLOAD, payload, NRF24L01_PAYLOAD_SIZE);
    if (!NRF24L01_VerifyRegister(NRF24_REG_CONFIG, txConfig) ||
        !NRF24L01_VerifyRegister(
            NRF24_REG_RF_SETUP, NRF24_RF_SETUP_1M_0DBM) ||
        ((NRF24L01_ReadRegister(NRF24_REG_FIFO_STATUS) &
          NRF24_FIFO_TX_EMPTY) != 0U)) {
        (void)NRF24L01_Command(NRF24_CMD_FLUSH_TX);
        return false;
    }

    /*
     * Keep CE asserted until TX_DS/MAX_RT is observed. This removes the
     * minimum-width CE pulse from the diagnosis and is valid in PTX mode.
     */
    DL_GPIO_setPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CE_PIN);
    return true;
}

uint8_t NRF24L01_GetAndClearEvents(void)
{
    uint8_t events = 0U;
    uint8_t status;

    if (g_initialized == 0U) {
        return 0U;
    }

    status = NRF24L01_Command(NRF24_CMD_NOP);
    if ((status & NRF24_STATUS_RX_DR) != 0U) {
        events |= NRF24L01_EVENT_RX_READY;
    }
    if ((status & NRF24_STATUS_TX_DS) != 0U) {
        events |= NRF24L01_EVENT_TX_DONE;
    }
    if ((status & NRF24_STATUS_MAX_RT) != 0U) {
        events |= NRF24L01_EVENT_MAX_RT;
    }
    if ((status & NRF24_STATUS_IRQ_MASK) != 0U) {
        NRF24L01_WriteRegister(
            NRF24_REG_STATUS, (uint8_t)(status & NRF24_STATUS_IRQ_MASK));
    }
    return events;
}

bool NRF24L01_IsRxFifoEmpty(void)
{
    if (g_initialized == 0U) {
        return true;
    }
    return (NRF24L01_ReadRegister(NRF24_REG_FIFO_STATUS) &
            NRF24_FIFO_RX_EMPTY) != 0U;
}

bool NRF24L01_ReadPayload(uint8_t payload[NRF24L01_PAYLOAD_SIZE])
{
    if ((g_initialized == 0U) ||
        (payload == NULL) ||
        NRF24L01_IsRxFifoEmpty()) {
        return false;
    }
    NRF24L01_ReadBuffer(
        NRF24_CMD_R_RX_PAYLOAD, payload, NRF24L01_PAYLOAD_SIZE);
    return true;
}

void NRF24L01_AbortTransmit(void)
{
    if (g_initialized == 0U) {
        return;
    }
    DL_GPIO_clearPins(NRF24_PINS_PORT, NRF24_PINS_RADIO_CE_PIN);
    NRF24L01_WriteRegister(
        NRF24_REG_STATUS, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    (void)NRF24L01_Command(NRF24_CMD_FLUSH_TX);
}

uint8_t NRF24L01_GetRpd(void)
{
    if (g_initialized == 0U) {
        return 0U;
    }
    return ((NRF24L01_ReadRegister(NRF24_REG_RPD) &
             NRF24_RPD_DETECTED) != 0U) ? 1U : 0U;
}

uint8_t NRF24L01_GetCeLevel(void)
{
    return ((NRF24_PINS_PORT->DOUT31_0 &
             NRF24_PINS_RADIO_CE_PIN) != 0U) ? 1U : 0U;
}

void NRF24L01_NotifyIrqFromIsr(void)
{
    g_irqPending = 1U;
}

bool NRF24L01_TakeIrqPending(void)
{
    if (g_irqPending == 0U) {
        return false;
    }
    g_irqPending = 0U;
    return true;
}
