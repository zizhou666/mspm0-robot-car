/*
================================================================================
W25Q64 软件 SPI Flash 接口模块
================================================================================
【功能简介】
本文件声明 8 MiB W25Q64 的 JEDEC 检测、任意地址读取、4 KiB 扇区擦除和
页编程接口，为 OLED 参数断电保存提供底层存储。

================================================================================
【函数定义】
- W25Q64_Init：初始化软件 SPI 空闲电平并校验 JEDEC 容量码 0x17。
- W25Q64_IsInitialized：查询 Flash 初始化状态。
- W25Q64_GetJedecId：返回最近读取的 24 位 JEDEC ID。
- W25Q64_Read：从指定地址读取任意长度数据。
- W25Q64_EraseSector4K：擦除指定 4 KiB 对齐扇区。
- W25Q64_PageProgram：在单个 256 字节页范围内写入数据。

================================================================================
【使用说明】
1. 接线为 PA12=SCLK、PA13=MISO、PA14=MOSI、PB25=CS。
2. 写入前必须先擦除目标区域，且 PageProgram 不允许跨越页边界。
3. 上层参数保存应调用 settings_store，不要直接覆盖日志扇区。
================================================================================
*/
#ifndef W25Q64_H_
#define W25Q64_H_

#include <stdbool.h>
#include <stdint.h>

#define W25Q64_CAPACITY_BYTES        (8UL * 1024UL * 1024UL)
#define W25Q64_SECTOR_SIZE_BYTES     (4096UL)
#define W25Q64_PAGE_SIZE_BYTES       (256UL)

/* Initializes the resource-table software-SPI pins and verifies EFxx17 ID. */
bool W25Q64_Init(void);
bool W25Q64_IsInitialized(void);
uint32_t W25Q64_GetJedecId(void);

bool W25Q64_Read(uint32_t address, void *data, uint32_t length);
bool W25Q64_EraseSector4K(uint32_t sector_address);
bool W25Q64_PageProgram(uint32_t address, const void *data, uint32_t length);

#endif /* W25Q64_H_ */
