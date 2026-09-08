# 最终使用说明

## 1. 上电前

1. 阅读 `02_PIN_CONFLICTS.md`，确认 ICM42688 使用 PA30 SDA/PA29 SCL 独立软件 I2C，PA29/PA30 没有再连接 MaixCam；
2. 按 `05_SYSCONFIG_AND_WIRING.md` 逐线核对；
3. 确认 ICM42688 的 SDA/SCL 各有 4.7 kΩ 上拉，nCS=3.3 V、AD0=GND；
4. 核对左电机驱动输入 PA4/PA7、右电机驱动输入 PA3/PB14；PA29/PA30 分别是 ICM42688 SCL/SDA；
5. 核对 PA27 蜂鸣器及 PB26/PB27/PB22 RGB 红/绿/蓝通道，确认外部驱动和限流符合原理图；
6. 断开电机主电源或把车轮架空；
7. 确认电机驱动 STBY/EN、电源地与逻辑地；
8. ICM42688 校准期间保持完全静止。
9. 确认上一页键接 PA18、下一页键接 PB21，按下时接地。

## 2. Keil 打开与构建

打开：

`keil/Integrated_42688_Encoder.uvprojx`

确认 Keil 已安装 Arm Compiler 6 和 `TexasInstruments.MSPM0G1X0X_G3X0X_DFP` 1.3.1。工程已启用 AC6、宏 `__MSPM0G3507__`、本地 SDK 头文件、Keil DriverLib、Keil startup 和 `mspm0g3507.sct`，直接执行 `Rebuild all target files`。

`.uvoptx` 不属于编译配置，uVision 首次打开会自动创建。不要让 Pack/RTE 再加入第二份 startup，也不要把 `startup_mspm0g350x_ticlang.c`、`device_linker.cmd` 或其他工具链的 DriverLib 加入 target。

按用户要求，交付方没有执行 Keil Build；首次 Rebuild 的错误/警告、内存占用和链接 map 由用户在本机确认。

## 3. 修改 SysConfig 时

普通 Keil Build 直接使用已交付的 `ti_msp_dl_config.c/.h`，没有自动生成前置命令。只有修改 `main.syscfg` 后才需重新生成。可在当前机器上执行（把 `<工程根目录>` 换成实际路径）：

```bat
"C:\ti\sysconfig_1.25.0\sysconfig_cli.bat" --script "<工程根目录>\main.syscfg" --output "<工程根目录>" --product "D:\TI-Keil\mspm0_sdk_2_04_00_06\.metadata\product.json" --compiler keil
```

生成后检查应为 0 error、0 warning；两条 PWM 参数保留信息可以保留。不要沿用源 B 那条搜索错误 SDK 路径的构建前命令。

## 4. 首次烧录

默认 `APP_SPEED_CONTROL_ENABLE_AT_BOOT=0`，电机输出保持 0。新版电机方向脚不占 PA19/PA20，`SYSCFG_DL_init()` 后 SWD 仍保持可用。首次烧录先断开电机主电源，确认 OLED、IMU 和编码器页面再接动力。

## 5. 正常启动

- OLED 显示 LOGO 约 2 秒；
- ICM42688 初始化成功后后台执行 100 样本、约 1 秒陀螺零偏校准；若采样离散度超过运动阈值则判定校准失败；
- LOGO 结束进入主页，校准中 ICM 状态为 STARTING；
- 校准完成后姿态页显示 Roll/Pitch/Yaw；
- 编码器静止时应显示 READY 与 0，不会被误判成初始化失败。

## 6. 编码器标定

1. 正向手动转左轮一整圈，读取位置增量；右轮同样测量；
2. 当前为 A 相双边沿 x2，已根据实车约 2:1 的距离误差把 `APP_ENCODER_COUNTS_PER_REV` 从 1060 修正为 530。进一步精调时使用：`新计数/圈 = 当前计数/圈 × 设定距离 ÷ 实际距离`；
3. 当前实测极性为左编码器 `-1`、右编码器 `+1`；车辆前进时两侧计数和速度都应为正；
4. 测量负载后的有效轮半径，更新 `APP_WHEEL_RADIUS_CM`；
5. 静止计数为 0 是正常状态，不能单凭 0 判定断线。

## 7. 电机方向与闭环启用

当前 `APP_MOTOR_LEFT_POLARITY`/`APP_MOTOR_RIGHT_POLARITY` 为 `-1/-1`。软件左右归属修正后，第 6 页实测 L−、R− 分别是实际左右轮前进方向；右侧使用 `+1` 时反转并报告 `Encoder Dir: RIGHT`，因此逻辑输出层同时反转左右桥。编码器左/右极性继续保持 `-1/+1`，编码器代码、PB4/PB5、PB6/PB7 映射和中断均未改动。DIR 与 NoSig 检测仍会明确显示 `LEFT`、`RIGHT` 或 `BOTH` 并安全停机；速度 PID 继续使用条件积分抗饱和。首次测试必须架空车轮，不要直接提高速度。

速度闭环不会再用反转主动制动：目标为正时 PID 输出下限为 0，目标为负时输出上限为 0，目标绝对值不超过 0.5 cm/s 时该轴直接停止。超速只会把 PWM 降到 0 等待车轮减速，零输出边界同样使用条件积分抗饱和，从而避免正转/反转周期振荡；真正设置负目标时仍可正常倒车。

确认急停手段后，可把 `APP_SPEED_CONTROL_ENABLE_AT_BOOT` 改为 `1U`，或在安全条件满足时调用：

```c
SpeedPID_SetTargets(left_cm_s, right_cm_s);
SpeedPID_Enable(true);
```

停止：

```c
SpeedPID_Enable(false);
```

### 第 6 页物理电机 IO 自检

第 6 页不经过速度 PID，也不应用 `APP_MOTOR_LEFT_POLARITY` 或
`APP_MOTOR_RIGHT_POLARITY`，用于直接核对参考驱动器左右 PWM 输入对。必须先架空车轮：

- 上键：L+，PA7/TIMA0_CC2=350，PA4/TIMA0_CC3=0；
- 下键：L−，PA7/TIMA0_CC2=0，PA4/TIMA0_CC3=350；
- 右键：R+，PA3/TIMA0_CC1=350，PB14/TIMA0_CC0=0；
- 左键：R−，PA3/TIMA0_CC1=0，PB14/TIMA0_CC0=350；
- 中键、切换页面或运行满 2 秒都会把四路比较值清零；自动停止后继续统计 150 ms 惯性脉冲，再冻结结果。

页面中的 `dL`、`dR` 是与首次 PWM 输出严格对齐的左右编码器增量。状态依次为
`ARM`、`RUN`、`COAST`、`DONE`，应在 `DONE` 后读取最终值。自检期间编码器
仍按原代码采样，但 PID 不会覆盖开环输出，DIR/NoSig 计时器不累计；
退出自检后原来的故障锁存和保护逻辑继续有效。

## 8. PI 调参

所有编译期可调值集中在 `src/user_config.h`。其中 `APP_SPEED_PID_KP/KI/KD` 是左右轮独立速度环参数；`APP_MOTOR_LEFT/RIGHT_POLARITY` 和 `APP_ENCODER_LEFT/RIGHT_POLARITY` 分别控制电机命令与编码器反馈方向；轮径、每圈计数、默认速度/距离、直行修正、巡线 PD、IMU/Mahony 和 OLED 时序也在同一文件分区说明。不要直接在驱动 C 文件中增加重复参数。

初值来自 r2：Kp=8.0、Ki=0.6、Kd=0，名义周期 10 ms，输出范围 ±999。Ki 表示每个 10 ms 离散样本累加的贡献；主循环偶发积压时程序按实际跨过的样本数缩放积分。输出继续向饱和方向增长时暂停积分，误差反向时恢复积分，以避免长期顶住 PWM 上限。

建议先令 Ki=0，从低目标速度逐步增加 Kp；在不过度振荡的前提下再增加 Ki 消除静差，并观察误差、积分和输出限幅是否长期饱和。

## 8.1 蜂鸣器与 RGB 实时调用

`main.c` 已在 `SYSCFG_DL_init()` 后调用 `BuzzerRGB_Init()`，正常业务代码直接包含 `buzzer_rgb.h` 即可。`Buzzer_On()`、`Buzzer_Off()`、`Buzzer_Toggle()` 控制蜂鸣器；`RGB_SetColor()` 选择常用颜色，`RGB_Set()` 可分别指定红、绿、蓝，`RGB_Off()` 全部熄灭。所有函数立即返回，不会阻塞编码器、PID、IMU 或 OLED 调度。

提示音和闪灯不要写成“打开—延时—关闭”，而应记录开始时间，在后续主循环到期时调用关闭函数。若其他代码绕过本驱动直接操作 GPIO，状态查询函数将不能反映真实输出。

OLED 操作：PA18 上一页，PB21 下一页，两键长按都不会连续翻页。第 3 页用五向上/下选择目标速度或目标距离，左/右减小或增加；第 4 页用五向上/下选择 Kp、Ki、Kd，选中项前显示 `*`，左/右调整。第 3、4 页的方向键首次按下立即执行，按住约 500 ms 后每 100 ms 连续执行一次，例如一直向左掰会连续减小当前 PID。第 5 页在电机停止时用左/右切换按距离/巡线模式，短按五向中键启动或停止；第 5、6 页忽略方向连发，防止反复切模式或重启物理测试。所有左/右调整先保存在 OLED 草稿中，下次启动电机测试时应用；长按五向中键约 1 秒会应用并写入 W25Q64，屏幕显示 `Saved to Flash` 后复位或断电仍可恢复。运行过程中修改草稿不会立即改变正在执行的控制目标。连发时间由 `APP_BUTTON_REPEAT_DELAY_MS` 和 `APP_BUTTON_REPEAT_PERIOD_MS` 配置。

首次启用电机调试时必须架空车轮。巡线模式使用完整 12 路灰度：P1～P12 依次为 PA31、PA28、PA1、PA0、PA25、PA24、PB24、PB23、PB19、PB18、PA16、PB13，其中 PB23 是 P8，不是独立启动键。两种模式都按启动点的相对路程判断目标距离并自动停车。

按距离模式额外使用左右编码器交叉同步：比较两轮从启动点开始的累计路程和瞬时速度，自动降低领先轮目标、提高落后轮目标。默认路程差增益为 5.0、速度差阻尼为 0.15，最大修正为基础速度的 35% 且不超过 30 cm/s。该修正只作用于按距离模式，不覆盖巡线模式的灰度转向目标。

## 9. 故障显示

- `ICM42688 Init Failed`：检查 PA30 SDA/PA29 SCL 接线、两只外接 4.7 kΩ 上拉、nCS=3.3 V、AD0=GND、地址 0x68 和 WHO_AM_I；确认 PA29/PA30 没有同时连接 MaixCam；其他页面仍运行；
- `Encoder Init Failed`：软件模块配置失败，不等同于计数为 0；
- `Encoder NoSig: LEFT/RIGHT/BOTH`：闭环启用且 PWM≥150/999 后对应侧连续 1 秒无边沿，故障锁存并关闭双电机；检查断线、堵转和阈值后复位；
- `Encoder Dir Error`/`DIR ERROR`：目标与实测速度持续异号，程序已关闭双电机；检查对应电机方向、编码器 A/B 顺序和 `APP_ENCODER_*_POLARITY` 后复位；
- `Flash Save Failed`：检查 W25Q64 的 PA12/PA13/PA14/PB25 接线、3.3 V 供电和 `EF4017` JEDEC ID；失败时当前参数仍可在本次上电使用，但不会伪报已经持久化；
- 电机调试页 `Fault: LINE`：启动时未检测到黑线或运行中持续丢线；确认 P1～P12 顺序和“黑线为高电平”的极性；
- OLED 无显示但系统运行：检查控制器、CS/DC/RST、列偏移和电平；软件 SPI 无 ACK，程序无法确认面板在线；
- 下载后若调试器异常断开：新版配置已保留 PA19/PA20 SWD，这不再是预期行为；检查供电、复位、下载器、Flash Algorithm，以及是否有其他代码重新配置 PA19/PA20。
