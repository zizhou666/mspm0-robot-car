<div align="center">

# MSPM0 Robot Car

TI MSPM0G3507 裸机智能小车底层 / Bare-metal robot-car firmware for TI MSPM0G3507

![C](https://img.shields.io/badge/C-00599C?logo=c&logoColor=white)
![MSPM0](https://img.shields.io/badge/MCU-MSPM0G3507-CC0000)
![Keil](https://img.shields.io/badge/IDE-Keil_MDK5-1F6FEB)
![License](https://img.shields.io/badge/license-MIT-green)

</div>

## About / 项目简介

This project integrates dual motors and encoders, ICM42688 attitude sensing,
12-channel line sensing, an OLED tuning interface, W25Q64 parameter storage,
buttons, a buzzer, and RGB indicators on an MSPM0G3507 robot-car controller.

本项目面向 TI MSPM0G3507，将双电机、双编码器、ICM42688、12 路灰度、OLED
在线调参、W25Q64 参数保存、五向按键、蜂鸣器和 RGB 指示灯整合为一个裸机工程。
控制层包含左右轮速度 PID、编码器直行 PID、航向 PID 与巡线 PID。

## Architecture / 架构

```text
GPIO / timers / software buses
        ↓
drivers/  →  control/  →  src/main.c
        ↘               ↙
          ui/ + settings
```

- Encoder edge counting runs in GPIO interrupts.
- A 10 ms timer drives speed sampling and closed-loop control.
- IMU, buttons, OLED, and Flash operations run cooperatively in the main loop.
- The firmware is bare metal; it does not use an RTOS or network service.

Detailed design notes are available in [`docs/`](docs/).

## Build

1. Install Keil MDK5 with Arm Compiler 6.
2. Obtain TI MSPM0 SDK **2.04.00.06** directly from Texas Instruments.
3. Place the required SDK `source/` and `lib/` directories under
   `external/mspm0-sdk/`. This directory is intentionally ignored and must not
   be committed.
4. Open `keil/Integrated_42688_Encoder.uvprojx` and build the target.
5. When changing pins, edit `main.syscfg`, regenerate
   `ti_msp_dl_config.c/.h`, and then rebuild.

The SDK is excluded because its license is separate from this repository. See
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

## Hardware overview / 硬件概览

- MSPM0G3507 + dual H-bridge motors and encoders
- ICM42688 six-axis IMU
- 12-channel digital line sensor
- 128×64 OLED and five-way buttons
- W25Q64 Flash, buzzer, and RGB LED
- Reserved SWDIO/SWCLK for programming and debugging

Pin assignments and wiring details are documented in
[`docs/05_SYSCONFIG_AND_WIRING.md`](docs/05_SYSCONFIG_AND_WIRING.md).

## Current status / 当前状态

The hardware abstraction and four control loops are implemented. Mechanical
calibration and PID values still require tuning on the actual chassis. A build
and hardware validation with the final board remains necessary.

## Safety

Motor tests can move the robot unexpectedly. Raise the wheels for the first
test, verify motor polarity at low output, keep an emergency stop available,
and do not treat the default PID values as final competition settings.

## Authentication and credentials

This firmware has no account login, OAuth, JWT, session, password, or API-token
flow. See [`docs/AUTHENTICATION.md`](docs/AUTHENTICATION.md) for the distinction
between firmware interfaces, ARM debug-authentication terminology, and the
separate GitHub publishing connection.

## License

Original project code is available under the [MIT License](LICENSE).
Third-party notices and exceptions are listed separately.
