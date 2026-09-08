# 文件来源与变更清单

## 原工程保护

四个源工程以及旧版 XLS、新版 XLSX 资源表均只读分析，没有修改、删除或覆盖原文件。所有实现均位于新建的 `Integrated_42688_Encoder` 工程。

## 最终 Keil 工程结构

```text
Integrated_42688_Encoder/
├─ keil/
│  ├─ Integrated_42688_Encoder.uvprojx
│  └─ mspm0g3507.sct
├─ startup/      Keil/ArmClang 启动汇编，栈 2 KiB
├─ src/          主程序、user_config 用户参数、派生配置和状态标志
├─ drivers/      ICM42688、OLED、编码器、电机、按键、蜂鸣器/RGB
├─ control/      双路速度 PI
├─ ui/           LOGO 与六页 OLED 状态机、参数编辑与 Flash 保存
├─ sdk/
│  ├─ source/    SDK 2.04 所需 DriverLib/device/CMSIS 头文件
│  ├─ lib/       Keil 版 driverlib.a
│  └─ license/manifest 文件
├─ docs/         架构、冲突、调度、接线、验证、使用说明
│  └─ reference/ 资源表副本及新版引脚文本转录
├─ main.syscfg
├─ ti_msp_dl_config.c
├─ ti_msp_dl_config.h
└─ README.md
```

Keil 的 `.uvoptx` 是用户界面/调试状态文件，未复制源 B 中含旧 Watch 变量的版本；uVision 首次打开 `.uvprojx` 时会自动生成。工程不需要 `.uvmpw` 或 RTE 组件。

## 从工程 B 复制后修改

| 新工程文件 | 来源 | 修改内容 |
|---|---|---|
| `drivers/icm42688.c/.h` | B 的模块化 ICM42688 代码及 SPI 参考工程的算法逻辑 | 使用 PA30/PA29 独立 GPIO 软件 I2C；迁入 SPI 版的 100 Hz 配置、100 样本判稳校准、Mahony、零姿态和输出低通逻辑 |
| `drivers/oled.c/.h` | B 的 OLED 代码 | 引脚改为生成宏；framebuffer 绘制；分块刷新；LOGO 不做秒级阻塞 |
| `main.syscfg` | 以 B 的配置为主体重建 | 编码器保持原映射；电机为 r2 TIMA0 四路 PWM；ICM 软件 I2C 改为 PA30 SDA/PA29 SCL；保留 OLED、W25Q64、12 路灰度、按键和控制定时器；新增 PA27 蜂鸣器及 PB26/PB27/PB22 RGB 输出 |
| `keil/*.uvprojx` | B 的 active Keil 工程模板 | 重建分组和相对路径；关闭失效的 SysConfig 前置命令；使用本地 SDK 子集 |
| `keil/mspm0g3507.sct` | B/SDK 2.04 Keil scatter | 保留 FLASH、SRAM、BCR、BSL 布局 |
| `startup/*_uvision.s` | B/SDK 2.04 Keil startup | 仅把栈从 0x100 调整到 0x800 |

## 新增实现

- `src/main.c`：初始化、状态降级和非阻塞调度；
- `src/user_config.h`：速度 PID、电机/编码器极性、机械标定、速度/距离、巡线、IMU、按键和 OLED 等用户可调参数；
- `src/app_config.h`、`src/app_status.h`：用户参数的派生常量及系统状态标志；
- `drivers/encoder.c/.h`：A 相双边沿、B 相判向、饱和计数、原子快照和左右/车体中心路程统计；
- `drivers/w25q64.c/.h`：PA12/PA13/PA14/PB25 软件 SPI、JEDEC ID、扇区擦除、页编程和有界忙等待；
- `drivers/line_sensor.c/.h`：P1～P12 灰度采样、有效位图、加权误差、丢线和宽线标记；
- `drivers/motor.c/.h`：r2 参考工程四路 PWM H 桥真值表及左右逻辑接口；
- `drivers/buttons.c/.h`：PA18/PB21 翻页键和五向键 30 ms 消抖、独立方向连发事件、单次翻页事件及中键长按事件；
- `drivers/buzzer_rgb.c/.h`：PA27 蜂鸣器和 PB26/PB27/PB22 RGB 高电平有效、上电安全关闭及非阻塞实时控制接口；
- `control/speed_pid.c/.h`：双路离散速度 PID及运行期目标值/Kp/Ki/Kd更新；
- `control/motor_debug.c/.h`：按距离行走、目标距离停车、r2 巡线 PD 和短按启停状态机；
- `src/settings_store.c/.h`：W25Q64 最后扇区 64 槽日志、版本/序号/CRC32、上电恢复与回读验证；
- `ui/ui.c/.h`：LOGO与六页 UI、目标速度/目标距离/PID/模式编辑、Flash 保存和物理电机 IO 自检；
- `keil/Integrated_42688_Encoder.uvprojx`：完整 Arm Compiler 6 工程；
- `sdk/`：构建所需、从 MSPM0 SDK 2.04.00.06 复制的头文件、Keil DriverLib 与许可证；
- `docs/*.md`：用户要求的十项分析与交付说明。

工程自有的 C/H、`main.syscfg`、启动汇编和 scatter 文件均已按统一模板增加“功能简介、函数定义、使用说明”文件头；TI 官方 `sdk/` 子集保持原样，避免修改第三方库。

## SysConfig 生成文件

`ti_msp_dl_config.c/.h` 由 `main.syscfg` 生成。新版映射已用 SysConfig 1.25.0、已安装的 MSPM0 SDK 2.09.00.01 产品描述和 `--compiler keil` 重新生成成功；只有一条 TIMA0 在 STOP/STANDBY 不保留寄存器的信息。生成 C/H 已用工程自带 SDK 头文件完成静态语法检查，保留 SWD，不创建硬件 I2C 实例。

Keil 工程的构建前自动生成已关闭，避免源 B 原有失效命令在错误 SDK 路径下静默继续使用旧文件。修改 `main.syscfg` 后应显式重新生成，再回到 Keil Build。

## 未进入最终 Keil target

- A 的 MPU6050、旧定时器、陈旧 `.o`；
- B `User/main.c` 中与模块版重复的 ICM/OLED实现；
- r1/r2 的整套旧 OLED 页面和按键实现；
- 不相关的 UART 和完整竞赛任务状态机；没有照搬参考工程的旧蜂鸣器/RGB 业务逻辑，只使用本工程新增的独立驱动；巡线只移植独立的灰度加权与 PD 逻辑；
- TI-Clang startup、`.cmd` linker、ProjectSpec、旧 `.out/.hex/.map`；
- 任何源工程的 Objects、Listings、`.uvguix` 或旧 `.uvoptx`。
