/*
================================================================================
W25Q64 GPIO 软件 SPI 驱动实现模块
================================================================================
【功能简介】
本文件使用 GPIO 模拟 SPI Mode 0，完成 W25Q64 片选、字节收发、状态轮询、
JEDEC 检测、读取、写使能、4 KiB 擦除和 256 字节页编程。

================================================================================
【函数定义】
- W25_SpiDelay：提供 GPIO 翻转之间的最小时序间隔。
- W25_Select/W25_Deselect：拉低/拉高 Flash 片选。
- W25_Transfer：按 MSB 先行同时发送并采样一个字节。
- W25_SendAddress：发送 24 位 Flash 地址。
- W25_ReadStatus1：读取状态寄存器 1。
- W25_WaitReady：轮询 BUSY 位并提供超时保护。
- W25_WriteEnable：发送写使能并等待器件可用。
- W25Q64_Init：恢复掉电、读取 JEDEC ID 并校验容量码。
- W25Q64_IsInitialized：返回底层驱动初始化状态。
- W25Q64_GetJedecId：返回缓存的 JEDEC ID。
- W25Q64_Read：执行连续数据读取。
- W25Q64_EraseSector4K：校验对齐和范围后擦除一个扇区。
- W25Q64_PageProgram：校验页边界后写入一个页内的数据。

================================================================================
【使用说明】
1. Flash 操作是阻塞轮询，只能在主循环低频执行，禁止放入中断。
2. 软件 SPI 引脚由 main.syscfg 生成，修改接线后必须重新生成配置。
3. 日常参数保存应使用 SettingsStore_Save，以获得 CRC 和掉电恢复能力。
================================================================================
*/
#include "w25q64.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define W25_CMD_WRITE_ENABLE        (0x06U)
#define W25_CMD_READ_STATUS1        (0x05U)
#define W25_CMD_READ_DATA           (0x03U)
#define W25_CMD_PAGE_PROGRAM        (0x02U)
#define W25_CMD_SECTOR_ERASE_4K     (0x20U)
#define W25_CMD_RELEASE_POWER_DOWN  (0xABU)
#define W25_CMD_JEDEC_ID            (0x9FU)

#define W25_STATUS_BUSY             (0x01U)
#define W25Q64_BUSY_POLL_LIMIT      (1000000UL)

static bool g_initialized;
static uint32_t g_jedecId;

static void W25_SpiDelay(void)
{
    __NOP();
    __NOP();
}

static void W25_Select(void)
{
    DL_GPIO_clearPins(FLASH_PINS_CS_PORT, FLASH_PINS_CS_PIN);
}

static void W25_Deselect(void)
{
    DL_GPIO_setPins(FLASH_PINS_CS_PORT, FLASH_PINS_CS_PIN);
}

static uint8_t W25_Transfer(uint8_t output)
{
    uint8_t input = 0U;
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U) {
        DL_GPIO_clearPins(FLASH_PINS_SCLK_PORT, FLASH_PINS_SCLK_PIN);
        if ((output & mask) != 0U) {
            DL_GPIO_setPins(FLASH_PINS_MOSI_PORT, FLASH_PINS_MOSI_PIN);
        } else {
            DL_GPIO_clearPins(FLASH_PINS_MOSI_PORT, FLASH_PINS_MOSI_PIN);
        }
        W25_SpiDelay();
        DL_GPIO_setPins(FLASH_PINS_SCLK_PORT, FLASH_PINS_SCLK_PIN);
        W25_SpiDelay();
        if ((DL_GPIO_readPins(FLASH_PINS_MISO_PORT,
                              FLASH_PINS_MISO_PIN) &
             FLASH_PINS_MISO_PIN) != 0U) {
            input |= mask;
        }
    }
    DL_GPIO_clearPins(FLASH_PINS_SCLK_PORT, FLASH_PINS_SCLK_PIN);
    return input;
}

static void W25_SendAddress(uint32_t address)
{
    (void)W25_Transfer((uint8_t)(address >> 16U));
    (void)W25_Transfer((uint8_t)(address >> 8U));
    (void)W25_Transfer((uint8_t)address);
}

static uint8_t W25_ReadStatus1(void)
{
    uint8_t status;

    W25_Select();
    (void)W25_Transfer(W25_CMD_READ_STATUS1);
    status = W25_Transfer(0xFFU);
    W25_Deselect();
    return status;
}

static bool W25_WaitReady(void)
{
    uint32_t attempts;

    for (attempts = 0U; attempts < W25Q64_BUSY_POLL_LIMIT; ++attempts) {
        if ((W25_ReadStatus1() & W25_STATUS_BUSY) == 0U) {
            return true;
        }
    }
    return false;
}

static void W25_WriteEnable(void)
{
    W25_Select();
    (void)W25_Transfer(W25_CMD_WRITE_ENABLE);
    W25_Deselect();
}

bool W25Q64_Init(void)
{
    uint32_t wake_delay;
    uint8_t manufacturer;
    uint8_t capacity;

    g_initialized = false;
    g_jedecId = 0U;
    DL_GPIO_clearPins(FLASH_PINS_SCLK_PORT, FLASH_PINS_SCLK_PIN);
    DL_GPIO_clearPins(FLASH_PINS_MOSI_PORT, FLASH_PINS_MOSI_PIN);
    W25_Deselect();

    W25_Select();
    (void)W25_Transfer(W25_CMD_RELEASE_POWER_DOWN);
    W25_Deselect();
    for (wake_delay = 0U; wake_delay < 2000U; ++wake_delay) {
        __NOP();
    }

    W25_Select();
    (void)W25_Transfer(W25_CMD_JEDEC_ID);
    g_jedecId = ((uint32_t)W25_Transfer(0xFFU) << 16U);
    g_jedecId |= ((uint32_t)W25_Transfer(0xFFU) << 8U);
    g_jedecId |= (uint32_t)W25_Transfer(0xFFU);
    W25_Deselect();

    manufacturer = (uint8_t)(g_jedecId >> 16U);
    capacity = (uint8_t)g_jedecId;
    /* W25Q64 is 64 Mbit / 8 MiB, so its JEDEC capacity byte is 0x17. */
    g_initialized = ((manufacturer == 0xEFU) && (capacity == 0x17U));
    return g_initialized;
}

bool W25Q64_IsInitialized(void)
{
    return g_initialized;
}

uint32_t W25Q64_GetJedecId(void)
{
    return g_jedecId;
}

bool W25Q64_Read(uint32_t address, void *data, uint32_t length)
{
    uint8_t *bytes = (uint8_t *)data;
    uint32_t i;

    if ((!g_initialized) || (data == NULL) || (length == 0U) ||
        (address >= W25Q64_CAPACITY_BYTES) ||
        (length > (W25Q64_CAPACITY_BYTES - address)) ||
        (!W25_WaitReady())) {
        return false;
    }

    W25_Select();
    (void)W25_Transfer(W25_CMD_READ_DATA);
    W25_SendAddress(address);
    for (i = 0U; i < length; ++i) {
        bytes[i] = W25_Transfer(0xFFU);
    }
    W25_Deselect();
    return true;
}

bool W25Q64_EraseSector4K(uint32_t sector_address)
{
    if ((!g_initialized) ||
        ((sector_address & (W25Q64_SECTOR_SIZE_BYTES - 1UL)) != 0U) ||
        (sector_address >= W25Q64_CAPACITY_BYTES) ||
        (!W25_WaitReady())) {
        return false;
    }

    W25_WriteEnable();
    W25_Select();
    (void)W25_Transfer(W25_CMD_SECTOR_ERASE_4K);
    W25_SendAddress(sector_address);
    W25_Deselect();
    return W25_WaitReady();
}

bool W25Q64_PageProgram(uint32_t address, const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t i;
    uint32_t page_offset = address & (W25Q64_PAGE_SIZE_BYTES - 1UL);

    if ((!g_initialized) || (data == NULL) || (length == 0U) ||
        (length > W25Q64_PAGE_SIZE_BYTES) ||
        ((page_offset + length) > W25Q64_PAGE_SIZE_BYTES) ||
        (address >= W25Q64_CAPACITY_BYTES) ||
        (length > (W25Q64_CAPACITY_BYTES - address)) ||
        (!W25_WaitReady())) {
        return false;
    }

    W25_WriteEnable();
    W25_Select();
    (void)W25_Transfer(W25_CMD_PAGE_PROGRAM);
    W25_SendAddress(address);
    for (i = 0U; i < length; ++i) {
        (void)W25_Transfer(bytes[i]);
    }
    W25_Deselect();
    return W25_WaitReady();
}
