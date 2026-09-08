# 静态检查与 Keil 待验证项

## 已完成的工程闭合检查

- `keil/Integrated_42688_Encoder.uvprojx` XML 可解析；
- target/output 均为 `Integrated_42688_Encoder`，器件为 MSPM0G3507，`uAC6=1`；
- 沿用源 B 的 ArmClang 6.23 与 TI DFP 1.3.1 设置；
- target 中 32 个工程文件全部存在，C、汇编和头文件类型正确；
- 所有 include、scatter 和 DriverLib 路径均为工程内相对路径，没有依赖原 A/B/r1/r2 目录或本机 `D:` 绝对路径；
- target 仅含一份 Keil startup、一份 `ti_msp_dl_config.c`、一套 ICM/OLED/编码器/电机/按键/蜂鸣器/RGB 实现；
- startup 栈为 0x800（2 KiB），heap 保持 0；
- scatter 保留 MSPM0G3507 的 128 KiB FLASH、32 KiB SRAM、BCR 和 BSL 区；
- 链接库是 SDK 2.04 对应 MSPM0G1X0X/G3X0X 的 Keil `driverlib.a`，没有混入 TI-Clang/GCC 库或 DriverLib `.c`；
- `.uvoptx`、旧 Objects/Listings/Watch 状态未打包，不会覆盖新的工程分组。

## SysConfig 验证

最终 `main.syscfg` 已用 SysConfig 1.25.0、当前安装的 MSPM0 SDK 2.09.00.01 产品描述和 `--compiler keil` 实际生成验证；生成 C/H 随后又对工程自带 2.04 头文件做了静态检查：

- 0 error；
- 0 warning；
- 一条 PWM 在 STOP/STANDBY 不保留寄存器的信息，不属于错误或警告；
- Keil 生成的 `ti_msp_dl_config.c/.h` 使用新版映射并保持 `Board.debugOn=true`。

PWM_0 为 TIMA0 四路 1 kHz 输出：CC0 PB14、CC1 PA3、CC2 PA7、CC3 PA4；TIMG0 为 10 ms 控制定时器；ICM42688 使用 PA30/PA29 GPIO 软件 I2C（SDA/SCL）；PA18/PB21 分别为上一页/下一页。编码器保持左 PB4/PB5、右 PB6/PB7，中断为 PB4/PB6。W25Q64 使用 PA12/PA13/PA14/PB25；灰度 P1～P12 全部生成，PB23 为 P8；PA27 蜂鸣器和 PB26/PB27/PB22 RGB 均生成为初值低的 GPIO 输出。

## 额外的 C 源码验证

新增蜂鸣器/RGB 引脚与驱动后，已用 TI Arm Clang 4.0.4.LTS 对最终工作副本做独立严格检查。这不是 Keil 编译，但覆盖了 14 个工程 C 文件和生成配置：

- C11、Cortex-M0+、Thumb、soft-float、`-O2`；
- `-Wall -Wextra`；
- 14 个 C 源文件静态语法检查成功，0 error；仅有 SDK 内联函数未使用形参警告，项目源码无新增警告；
- 未发现重复定义、缺失头文件、未声明函数或未解析符号；
- 旧 `I2C_0`、`DL_I2C_*`、`PWM_LEFT/PWM_RIGHT`、`MOTOR_AIN*`、`MOTOR_BIN*` 残留均为 0；
这项检查证明最终 C 源码在 TI Arm Clang 前端下语法闭合；它仍不能代替用户本机的 Keil Build，也不能证明 Keil startup/scatter、DFP Pack、链接、下载器或实物电气已经验证。

## 明确没有声称完成

按用户要求，本次没有调用 Keil 执行 Build，也没有连接 MSPM0G3507、ICM42688、OLED、编码器或电机做硬件测试。因此不能声称“Keil 已编译通过”或“实物已验证”。

## 用户仍需在 Keil/硬件确认

1. 安装并选中 `TexasInstruments.MSPM0G1X0X_G3X0X_DFP` 1.3.1，Arm Compiler 6 可用；
2. 打开 `.uvprojx` 后执行 Rebuild，确认 0 error，检查实际 code/data/stack；
3. 工程没有被 Pack/RTE 自动加入第二份 startup；
4. 所用下载器和 Flash Algorithm 能识别 MSPM0G3507；
5. ICM42688 的 PA30 SDA、PA29 SCL 各有 4.7 kΩ 上拉，nCS=3.3 V、AD0=GND、WHO_AM_I=0x47，且没有并接 MaixCam；PB14 只接右电机驱动；
6. OLED 控制器、列偏移、扫描方向和电平正确；
7. 左右编码器 A/B 顺序、正方向和一圈 x2 计数正确；
8. 电机驱动按 PA7/PA4 左桥、PA3/PB14 右桥接线，STBY/EN、滑行/制动语义和 1 kHz PWM 可接受；
9. 电机噪声下 ICM 软件 I2C、OLED 软件 SPI 和编码器边沿仍稳定；
10. 实际轮径、CPR 和 PI 参数在负载下重新标定；
11. PA27 蜂鸣器及 PB26/PB27/PB22 RGB 的高有效电气、限流/驱动电路和上电默认关闭状态符合实物原理图。
