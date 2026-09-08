# SysConfig 与接线说明

## 配置源与生成规则

唯一手工维护的外设配置是根目录 `main.syscfg`。Keil target 直接编译根目录中已生成的 `ti_msp_dl_config.c/.h`：

- 器件：MSPM0G3507，LQFP-64；
- 工程内构建头文件/DriverLib：MSPM0 SDK 2.04.00.06；本次可用的 SysConfig 产品描述为 2.09.00.01；
- 最终生成工具：SysConfig 1.25.0；
- 编译器目标：Keil/ArmClang；
- 唯一引脚依据：`MSPM0G3507电赛小车资源分配表_42688独立软件I2C版.xlsx`。

不要直接手改生成文件。Keil 工程已关闭源 B 中失效的自动生成前置命令；修改 `main.syscfg` 后应按 `08_USER_GUIDE.md` 显式以 `--compiler keil` 重新生成，再回到 Keil Rebuild。

## SysConfig 模块

| 实例名 | DriverLib 外设 | 配置 |
|---|---|---|
| `PWM_0` | TIMA0 | Edge-aligned up，1 kHz、period=1000；CC0 PB14、CC1 PA3、CC2 PA7、CC3 PA4；比较值立即更新，由 `Motor_Init()` 启动 |
| `CONTROL_TIMER` | TIMG0 | 10 ms periodic-up，ZERO 中断，优先级 1，主程序启动 |
| `ENCODER` | GPIOB | 左 A/B=PB4/PB5，右 A/B=PB6/PB7；PB4/PB6 双边沿中断 |
| `ICM_SDA` | GPIOA | PA30，软件 I2C 开漏模拟 |
| `ICM_SCL` | GPIOA | PA29，软件 I2C 开漏模拟 |
| `KEY5` | GPIOB | PB12/PB8/PB9/PB10/PB11，输入上拉 |
| `PAGE_PREV` | GPIOA | PA18，上一页键，输入上拉、低有效 |
| `PAGE_NEXT` | GPIOB | PB21，下一页键，输入上拉、低有效 |
| `OLED_PINS` | GPIOA/B | PA17/PB15/PB16/PB17/PB20，软件 SPI |
| `FLASH_PINS` | GPIOA/B | PA12 SCLK、PA13 MISO、PA14 MOSI、PB25 CS，W25Q64 软件 SPI |
| `LINE_SENSORS` | GPIOA/B | 12 路灰度 P1～P12 输入，PB23 为 P8 |
| `RGB_PINS` | GPIOB | PB26 红、PB27 绿、PB22 蓝，高电平有效，初值为低 |
| `BUZZER_PIN` | GPIOA | PA27 蜂鸣器，高电平有效，初值为低 |
| `Board` | DEBUGSS | `debugOn=true`，保留 PA19/PA20 SWD |

最终配置不创建硬件 I2C 实例。PA29/PA30 作为 GPIO 软件 I2C 专用于 ICM42688，不再为 MaixCam 保留。

## 接线

### ICM42688（独立软件 I2C）

| ICM42688 | MSPM0G3507/连接 |
|---|---|
| SDA | PA30，并用 4.7 kΩ 上拉到 3.3 V |
| SCL | PA29，并用 4.7 kΩ 上拉到 3.3 V |
| VCC | 3.3 V（同时满足所用模块规格） |
| GND | GND |
| AD0/SDO | GND，使 7 位地址为 0x68 |
| nCS | 固定连接 3.3 V，选择 I2C 模式 |
| INT | 不连接；软件使用定时轮询 |

SDA/SCL 的外部 4.7 kΩ 上拉是必需条件，不能只依赖 GPIO 内部电阻。软件用“输出低/释放为输入”模拟开漏，不会主动输出高电平；总线释放、ACK 和 SCL 拉伸等待均有超时。

`WHO_AM_I` 应读到 0x47。PA29/PA30 是 ICM42688 专用软件总线，不得再并接 MaixCam。PB14 已用于右电机 PWM 输入，不得连接 ICM42688。

### 蜂鸣器与 RGB 指示灯

| 模块/颜色 | MSPM0G3507 | 有效电平 | 上电状态 |
|---|---|---|---|
| 蜂鸣器 BEEP | PA27 | 高电平发声 | 关闭 |
| RGB 红 R | PB26 | 高电平点亮 | 熄灭 |
| RGB 绿 G | PB27 | 高电平点亮 | 熄灭 |
| RGB 蓝 B | PB22 | 高电平点亮 | 熄灭 |

`SYSCFG_DL_GPIO_init()` 会先把四路输出清零，再使能输出；`main()` 随后调用 `BuzzerRGB_Init()` 再次确认安全状态。实时控制只调用 `drivers/buzzer_rgb.h` 中的接口。RGB 裸灯需要限流电阻；超过 GPIO 驱动能力的蜂鸣器需要三极管或 MOSFET 驱动。

### OLED（4 线软件 SPI + Reset）

| OLED | MSPM0G3507 |
|---|---|
| CLK/SCL | PA17 |
| DIN/MOSI/SDA | PB15 |
| RST | PB16 |
| DC | PB17 |
| CS | PB20 |
| VCC/GND | 按面板规格接电源/地 |

新版资源表没有 OLED 行，因此 OLED 沿用源工程 B 接线，不自行猜测替换。OLED 使用 PA17 CLK、PB15 MOSI、PB16 RST、PB17 DC、PB20 CS。W25Q64 使用另一组软件 SPI，不与 OLED 共线。

### W25Q64 参数 Flash

| W25Q64 | MSPM0G3507 |
|---|---|
| CLK | PA12 |
| DO/POCI/MISO | PA13 |
| DI/PICO/MOSI | PA14 |
| CS0 | PB25 |
| VCC/GND | 3.3 V/GND |

程序用 `0x9F` 检查 W25Q64 JEDEC ID `EF4017`，只占用最后一个 4 KiB 扇区 `0x7FF000`。每条参数记录为 64 字节，包含 magic、版本、递增序号和 CRC32；同一扇区可追加 64 次，写满后才擦除。保存后立即回读验证。

### 12 路灰度

| 通道 | P1 | P2 | P3 | P4 | P5 | P6 | P7 | P8 | P9 | P10 | P11 | P12 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 引脚 | PA31 | PA28 | PA1 | PA0 | PA25 | PA24 | PB24 | PB23 | PB19 | PB18 | PA16 | PB13 |

灰度输入保持 r2 参考工程的“高电平表示检测到黑线”极性，P1～P12 按车头观察从左到右排列。PB23 仍为 P8，没有作为启动键。

### 双电机驱动

| 驱动信号 | MSPM0G3507 | r2 默认方向配置 0 |
|---|---|---|
| 左输入 1 | PA4/TIMA0_CC3 | 物理 L+ 为 0，L− 为 PWM |
| 左输入 2 | PA7/TIMA0_CC2 | 物理 L+ 为 PWM，L− 为 0 |
| 右输入 1 | PA3/TIMA0_CC1 | 物理 R+ 为 PWM，R− 为 0 |
| 右输入 2 | PB14/TIMA0_CC0 | 物理 R+ 为 0，R− 为 PWM |

四路 PWM 全部属于 TIMA0，周期 1 kHz、计数周期 1000，输出范围限制为 ±999。代码与 r2 参考工程相同：正/反方向分别在同一 H 桥的一个输入输出 PWM、另一个输入输出 0；零命令把两路比较值都置 0。确认所用驱动芯片的 STBY/EN 已正确处理；参考工程没有给 STBY GPIO，本工程也没有虚构一个。

旧六线拓扑使用的 PA15、PB1、PB2、PB3 方向/使能逻辑已经删除；PB2 当前释放。正反切换时先把原活动通道写 0，再把另一通道写入新占空比。

第 5 页闭环电机调试的逻辑输出应用底盘极性：左 `-1`、右 `-1`。左右软件归属修正后，第 6 页实测 L−、R− 分别为实际左右轮前进方向；闭环右侧使用 `+1` 时会反转并准确报告 `Encoder Dir: RIGHT`，因此最终两侧都取 `-1`。这不是修改编码器。第 6 页物理 IO 自检绕过该极性，仍直接输出上表真值表。

PA17～PA20 不作电机 GPIO。特别是 PA19/PA20 保持 SWDIO/SWCLK，PA18 独立键/BSL 支路继续作为上一页键。

### 编码器

| 编码器 | A 相 | B 相 |
|---|---|---|
| 左 | PB4 | PB5 |
| 右 | PB6 | PB7 |

输入未擅自启用内部上拉。编码器输出必须与 MSPM0G3507 电平兼容；开漏输出需外部上拉。当前为 A 相双边沿计数（x2），B 相只用于判向。根据实车“实际路程约为设定路程两倍”的结果，`APP_ENCODER_COUNTS_PER_REV` 已从 1060 修正为 530。

编码器使用实车验证后的方案二：左 A/B=PB4/PB5，右 A/B=PB6/PB7。它覆盖资源表的交错 E1/E2 排列，以匹配现有线束和原 PID 工程；若底盘左右接线相反，应成对调整并重新确认闭环方向。

### 翻页键与五向键

PA18 为上一页，PB21 为下一页。五向键为 PB12 上、PB8 下、PB9 左、PB10 右、PB11 中。所有按键均为按下接地、内部上拉、低有效。五向键用于页面内参数操作：上下选择，左减右加；第 5 页短按中键启动/停止电机，长按约 1 秒把目标速度、目标距离、PID 和电机模式保存到 W25Q64。长按达到阈值后不会再产生短按事件。

## 生成验证

新版 `main.syscfg` 已由 TI SysConfig 1.25.0 CLI 以 `--compiler keil` 实际运行，生成成功；控制台只有 TIMA0 在 STOP/STANDBY 不保留寄存器的提示，没有引脚或外设冲突。生成结果包含 TIMA0 四路电机 PWM（PB14/PA3/PA7/PA4）、PA30/PA29 ICM 软件 I2C、PA12/PA13/PA14/PB25 Flash、完整 P1～P12 灰度、PA18/PB21 翻页输入、PA27 蜂鸣器及 PB26/PB27/PB22 RGB，并保持 `Board.debugOn=true`。
