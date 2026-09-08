# 引脚冲突、决策与最终映射

## 新版资源表结论

新版 `MSPM0G3507电赛小车资源分配表_42688独立软件I2C版.xlsx` 已解除旧表中的三项关键冲突：

- ICM42688 不占用五向键 PB8～PB11；用户后续指定使用 PA29/PA30，因此取消 MaixCam I2C 预留；
- 用户后续明确要求电机驱动逻辑和引脚均照搬 r2 参考工程；
- PA19/PA20 仍保留 SWDIO/SWCLK，PA18 仍为独立上一页键。

检查确认 PA29/PA30 只在文档中为 MaixCam 预留，当前固件没有实际占用。按用户要求，ICM42688 最终改为 PA29 SCL、PA30 SDA 的独立 GPIO 软件 I2C；两线必须外接 4.7 kΩ 上拉到 3.3 V，nCS 固定接 3.3 V，AD0 接 GND。MaixCam 不得再并接这两根线。

## 原工程到新版资源表的调整

| 原资源 | 原引脚/实例 | 与新版分配的关系 | 最终处理 |
|---|---|---|---|
| A 左/右电机 PWM | TIMG6、TIMG0 | 与编码器、Flash 冲突 | 最终由 r2 TIMA0 四路 PWM 完整替代 |
| A 编码器 | PA17/PA22/PA15/PA16 | 与新版电机、UART2、灰度等资源冲突 | 统一改为 PB4～PB7 |
| A 控制定时器 | TIMA0/TIMA1 | 两个 TimerA 分别用于两路 PWM | 10 ms 控制定时器使用 TIMG0 |
| B ICM42688 硬件 I2C | PA3/PA4 | 与 r2 电机 TIMA0 通道冲突 | 改为 PA30 SDA、PA29 SCL 的独立 GPIO 软件 I2C |
| 旧整合版六线电机 | PA15/PA3 PWM + PA7/PB1/PB2/PB3 方向 | 用户要求按 r2 参考工程照搬 | 删除旧方向 GPIO，改为 PB14/PA3/PA7/PA4 四路 TIMA0 PWM |
| OLED | 新版表未列出 | 无法从新表重新指定 | 保持源工程 B 的 PA17/PB15/PB16/PB17/PB20 接线 |
| B 模板 PWM/编码器 | TIMG6/TIMG0、PA/PB 若干 | 与新版资源表大面积冲突且实际未使用 | 不复制模板残留 |

源工程 A/B 的旧引脚只用于来源审计，不能按旧表接入最终板卡。

## 最终工程实际使用映射

| 功能 | MSPM0G3507 | 外设/说明 |
|---|---|---|
| 左电机 H 桥输入 1/2 | PA4/PA7 | TIMA0 CCP3/CCP2，1 kHz；第 6 页实测归属 |
| 右电机 H 桥输入 1/2 | PA3/PB14 | TIMA0 CCP1/CCP0，1 kHz；第 6 页实测归属 |
| 左编码器 A/B | PB4/PB5 | PB4 双边沿中断，PB5 判向；按实车线束方案二 |
| 右编码器 A/B | PB6/PB7 | PB6 双边沿中断，PB7 判向；按实车线束方案二 |
| ICM42688 SDA/SCL | PA30/PA29 | 独立 GPIO 软件 I2C，地址 0x68 |
| OLED CLK/MOSI | PA17/PB15 | GPIO 软件 SPI；不与 ICM42688 软件 I2C 共线 |
| OLED RST/DC/CS | PB16/PB17/PB20 | 沿用现有整合工程控制脚 |
| W25Q64 SCLK/MISO/MOSI/CS | PA12/PA13/PA14/PB25 | 独立 GPIO 软件 SPI；最后一个扇区保存参数 |
| 蜂鸣器 | PA27 | GPIO 输出，高电平有效，上电默认关闭 |
| RGB 红/绿/蓝 | PB26/PB27/PB22 | GPIO 输出，高电平有效，上电默认熄灭 |
| 12 路灰度 P1～P12 | PA31/PA28/PA1/PA0/PA25/PA24/PB24/PB23/PB19/PB18/PA16/PB13 | 高电平有效；PB23 保持 P8 |
| 五向键上/下/左/右/中 | PB12/PB8/PB9/PB10/PB11 | 低有效、内部上拉 |
| 控制定时器 | TIMG0 | 10 ms 周期，零事件中断 |
| SWDIO/SWCLK | PA19/PA20 | 保留调试功能，不作普通 GPIO |
| MaixCam I2C SCL/SDA | PA29/PA30 | 已被 ICM42688 占用，不可同时使用 |

编码器最终按实车验证后的方案二分组：左轮 PB4/PB5，右轮 PB6/PB7。该分组与资源表中 PB4 E2A、PB5 E1A、PB6 E2B、PB7 E1B 的交错排列不同，是为匹配现有线束及原 PID 工程而作的明确覆盖。若实车左右相反，应成对交换左右电机/编码器连接或修改左右映射，不能只交换其中一侧而破坏闭环对应关系。

## 尚未启用但需要保留的资源表项目

MaixCam UART/I2C、蓝牙 UART和 ADC 电池采样没有进入本次功能范围。12 路灰度、W25Q64、PA27 蜂鸣器以及 PB26/PB27/PB22 RGB 已按资源表启用。PA29/PA30 已明确改给 ICM42688；若以后恢复 MaixCam I2C，必须重新分配 ICM42688 引脚，不能复用同一对 GPIO。

## 蜂鸣器与 RGB 决策

PA27、PB26、PB27、PB22 在最终工程其他模块中均无占用，现分别固定为蜂鸣器、红、绿、蓝。有效电平沿用 r2 参考工程：`DL_GPIO_setPins()` 表示发声或点亮，`DL_GPIO_clearPins()` 表示关闭。`main.syscfg` 把四路初值设为 `CLEARED`，生成代码先清零再使能输出，运行时统一由 `drivers/buzzer_rgb.c/.h` 操作。

## SWD 状态

r2 四路 PWM 电机引脚为 PB14/PA3/PA7/PA4，不占用 PA19/PA20。`main.syscfg` 保持 `Board.debugOn = true`，因此：

1. 执行 `SYSCFG_DL_init()` 后 SWD 仍保持可用；
2. Keil 可正常进行在线下载和调试；
3. PA19/PA20 不得在其他代码中重新配置为 GPIO；
4. 若后续再次调整引脚，必须继续把 SWD 作为保留资源检查。

## ICM42688 独立软件 I2C 硬件前提

ICM42688 必须按当前映射连接：SDA=PA30、SCL=PA29、nCS=3.3 V、AD0=GND，7 位地址为 0x68。SDA 和 SCL 各外接一只 4.7 kΩ 电阻上拉到 3.3 V。

软件用“输出低/释放为输入”的方式模拟开漏 I2C，不会把总线主动推高。驱动不使用 ICM INT，传感器由主循环定时轮询。PA29/PA30 不得再并接 MaixCam，也不得再接旧版 PB8～PB11 SPI 网络；PB14 只接右电机驱动输入。

## OLED 接线说明

新版资源表没有 OLED 行，因此 OLED 保持源工程 B 接线：PA17 CLK、PB15 MOSI、PB16 RST、PB17 DC、PB20 CS。OLED 与 W25Q64 使用不同的软件 SPI 引脚和独立片选，不会因保存参数而同时选中。

## PB23 决策

PB23 严格保留为第 8 路灰度输入 P8，不再作为电机启动按键。电机调试页用五向中键区分操作：短按释放后启动/停止，长按约 1 秒仅产生 Flash 保存事件，因此 12 路灰度全部可用且不存在 PB23 复用冲突。
