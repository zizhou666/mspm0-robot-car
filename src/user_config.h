/*
================================================================================
Integrated_42688_Encoder 用户可调参数模块
================================================================================
【功能简介】
本文件集中保存整车调试时允许修改的参数，避免在驱动、中断和控制算法中
散落参数。参数按控制周期、编码器、电机、速度/直行/巡线/角度 PID、按键、
ICM42688 和 OLED 分类，并在定义处标明单位、作用和修改限制。

================================================================================
【函数定义】
本文件只定义编译期配置宏，不定义函数。
- 速度 PID：APP_SPEED_PID_KP/KI/KD，作用于左右轮闭环速度控制。
- 直行/巡线/角度 PID：上层转向修正默认值、限幅、调节范围和步长。
- 电机/编码器：极性、PWM 限幅、轮径和每圈计数等机械标定参数。
- 运动目标：默认速度、目标距离以及 OLED 调节范围和步长。
- 姿态解算：ICM42688 采样、校准、软件 I2C 和 Mahony 参数。
- 人机界面：按键消抖、方向键连发、中心键长按时间、OLED 方向/亮度和刷新周期。

================================================================================
【使用说明】
1. 一般调车只修改本文件，不要直接修改 encoder.c、motor.c 或 speed_pid.c。
2. 修改编码器/电机极性时，每次只改一侧并重新验证“车体前进为正”。
3. APP_MOTOR_PWM_PERIOD_COUNTS 必须与 main.syscfg 中 TIMA0 周期保持一致。
4. 速度 PID 为左右轮独立的位置式离散 PID；先调 Kp，再调 Ki，通常 Kd 为 0。
5. 改动 I2C 速率或 OLED 时序后，应重新验证上电初始化和长线通信可靠性。
6. OLED 中修改并长按保存的是运行参数；本文件给出首次上电/无记录默认值。
================================================================================
*/
#ifndef USER_CONFIG_H_
#define USER_CONFIG_H_

/* ======================== 控制与任务调度参数 ======================== */
/* 编码器采样和速度 PID 的固定执行周期，单位 ms。 */
#define APP_CONTROL_PERIOD_MS                 (10U)

/* ICM42688 采样、连续失败阈值、校准重试、掉线重连和最大积分步长。 */
#define APP_IMU_PERIOD_MS                     (10U)
#define APP_MAIXCAM_PACKET_PERIOD_MS           (10U)
#define APP_IMU_FAILURE_THRESHOLD             (5U)
#define APP_IMU_RETRY_BACKOFF_MS              (250U)
#define APP_IMU_RECONNECT_BACKOFF_MS          (250U)
#define APP_IMU_MAX_DT_SECONDS                (0.050f)

/* ======================== Battery voltage monitor ======================== */
/* PA26/ADC0 channel 1 measures VIN through the fitted 100k/10k divider. */
#define APP_BATTERY_SAMPLE_PERIOD_MS           (20U)
#define APP_BATTERY_CONVERSION_TIMEOUT_MS      (5U)
#define APP_BATTERY_ADC_REFERENCE_MV           (3300UL)
#define APP_BATTERY_ADC_FULL_SCALE_COUNTS      (4095UL)
#define APP_BATTERY_DIVIDER_TOP_KOHM           (100UL)
#define APP_BATTERY_DIVIDER_BOTTOM_KOHM        (10UL)
/* Adjust after comparing the OLED value with a multimeter: true/displayed. */
#define APP_BATTERY_CALIBRATION_PERMILLE       (1000UL)
/* IIR alpha = 1 / 2^shift; 3 gives alpha = 1/8. */
#define APP_BATTERY_FILTER_SHIFT               (3U)
#define APP_BATTERY_WARNING_ENTER_MV           (10300UL)
#define APP_BATTERY_WARNING_RECOVERY_MV        (10600UL)
#define APP_BATTERY_WARNING_QUALIFY_MS         (2000UL)
#define APP_BATTERY_WARNING_BLINK_PERIOD_MS    (500UL)
#define APP_BATTERY_WARNING_BEEP_PERIOD_MS     (5000UL)
#define APP_BATTERY_WARNING_BEEP_DURATION_MS   (100UL)
#define APP_TASK_CUE_BEEP_DURATION_MS           (100UL)

/* ======================== NRF24L01 wireless link ======================== */
/* Fixed peer-compatible radio settings: CH40, 1 Mbps, 5-byte E7 address. */
#define APP_NRF24_RF_CHANNEL                    (40U)
#define APP_NRF24_ADDRESS_WIDTH                 (5U)
#define APP_NRF24_ADDRESS_BYTE                  (0xE7U)
/* 750-us retry delay (ARD=2), maximum 10 retries (ARC=10). */
#define APP_NRF24_AUTO_RETRY_REGISTER           (0x2AU)
#define APP_NRF24_SPI_DELAY_CYCLES              (2U)
#define APP_NRF24_RX_FIFO_DEPTH                 (3U)
#define APP_NRF24_STATUS_POLL_MS                (10U)
#define APP_NRF24_INIT_RETRY_MS                 (1000U)
#define APP_NRF24_MODE_SETTLE_MS                (1U)
#define APP_NRF24_TX_TIMEOUT_MS                 (100U)
#define APP_NRF24_RESULT_HOLD_MS                (1000U)
#define APP_NRF24_LINK_TIMEOUT_MS               (500U)
#define APP_NRF24_FIRST_LINK_TIMEOUT_MS         (500U)
#define APP_NRF24_HEARTBEAT_MS                  (100U)
#define APP_NRF24_RECONNECT_MS                  (500U)
#define APP_NRF24_APP_RETRY_INTERVAL_MS         (50U)
#define APP_NRF24_APP_SEND_ATTEMPTS             (4U)
#define APP_NRF24_TEST_PROTOCOL_VERSION         (2U)
#define APP_NRF24_TEST_PACKET_TYPE              (1U)
#define APP_NRF24_TEST_PATTERN                  (0xA5U)

/* Wireless remote-control menu and motion limits. */
#define APP_NRF24_DISTANCE_DEFAULT_CM           (10U)
#define APP_NRF24_DISTANCE_MIN_CM               (1U)
#define APP_NRF24_DISTANCE_MAX_CM               (999U)
#define APP_NRF24_ANGLE_DEFAULT_DEG             (90U)
#define APP_NRF24_ANGLE_MIN_DEG                 (1U)
#define APP_NRF24_ANGLE_MAX_DEG                 (360U)
#define APP_NRF24_STEP_DEFAULT                  (5U)
#define APP_NRF24_STEP_MIN                      (1U)
#define APP_NRF24_STEP_MAX                      (50U)
#define APP_NRF24_EXIT_HOLD_MS                  (2000U)
#define APP_NRF24_REMOTE_LINEAR_SPEED_CM_S      (30.0f)
#define APP_NRF24_REMOTE_TURN_SPEED_CM_S        (20.0f)
#define APP_NRF24_REMOTE_DISTANCE_TOLERANCE_CM  (0.5f)
#define APP_NRF24_REMOTE_ANGLE_TOLERANCE_DEG    (2.0f)

/* ======================== 编码器与机械标定参数 ====================== */
/* A/B 双相双边沿四倍频计数模式下的单轮每圈计数；轮径/轮胎变化后需重新标定。 */
#define APP_ENCODER_COUNTS_PER_REV            (1351.0f)
/* 驱动轮半径，单位 cm，用于脉冲到速度、路程的换算。 */
#define APP_WHEEL_RADIUS_CM                   (3.25f)

/* 车体前进时左右编码器计数应为正；若某侧相反，只改对应的 1/-1。 */
#define APP_ENCODER_LEFT_POLARITY             (1)
#define APP_ENCODER_RIGHT_POLARITY            (-1)

/* 有电机命令时的无信号与方向错误诊断阈值。 */
#define APP_ENCODER_SIGNAL_TEST_DUTY_COUNTS    (150U)
#define APP_ENCODER_SIGNAL_TIMEOUT_MS          (1000U)
#define APP_ENCODER_DIRECTION_TEST_SPEED_CM_S  (3.0f)
#define APP_ENCODER_DIRECTION_TIMEOUT_MS       (500U)

/* =========================== 电机输出参数 =========================== */
/* 逻辑前进到物理 H 桥方向的映射；只允许填写 1 或 -1。 */
#define APP_MOTOR_LEFT_POLARITY                (-1)
#define APP_MOTOR_RIGHT_POLARITY               (-1)

/* 必须与 TIMA0 PWM 周期匹配；输出上限应小于 PWM 周期。 */
#define APP_MOTOR_PWM_PERIOD_COUNTS            (1000U)
#define APP_MOTOR_OUTPUT_LIMIT                 (999)
/* TB6612 short-brake: IN1/IN2 high with PWM high when Motor_Stop is called. */
#define APP_MOTOR_ACTIVE_BRAKE_ENABLE          (1U)
#define APP_MOTOR_BRAKE_DUTY_COUNTS            (800U)

/* OLED 第六页物理通道点动测试的占空计数、持续时间、停稳和刷新周期。 */
#define APP_MOTOR_IO_TEST_DUTY_COUNTS          (350)
#define APP_MOTOR_IO_TEST_DURATION_MS          (2000U)
#define APP_MOTOR_IO_TEST_SETTLING_MS          (150U)
#define APP_MOTOR_IO_TEST_UI_REFRESH_MS        (50U)

/* 复位后是否立即使能速度闭环：0=保持停机，1=上电使能。 */
#define APP_SPEED_CONTROL_ENABLE_AT_BOOT       (0U)

/* ======================== 左右轮速度 PID 参数 ======================= */
/*
 * 这是左右轮各自的速度环位置式离散 PID：
 * Kp 抑制当前速度误差；Ki 消除稳态误差；Kd 抑制误差变化。
 * Ki 按每个 10-ms 基准周期累计，算法内部带条件积分抗饱和。
 */
#define APP_SPEED_PID_KP                       (8.0f)
#define APP_SPEED_PID_KI                       (0.6f)
#define APP_SPEED_PID_KD                       (0.0f)
#define APP_SPEED_PID_ERROR_LIMIT_CM_S         (50.0f)
#define APP_SPEED_PID_INTEGRAL_LIMIT           (600.0f)
#define APP_SPEED_PID_OUTPUT_LIMIT             (999.0f)
#define APP_SPEED_PID_TARGET_STOP_BAND_CM_S    (0.5f)

/* 无 Flash 有效记录时使用的速度和行驶距离默认值。 */
#define APP_DEFAULT_TARGET_SPEED_CM_S          (80.0f)
#define APP_DEFAULT_TARGET_DISTANCE_CM         (100.0f)

/* OLED 参数调节的上下限和单次按键步长。 */
#define APP_TARGET_SPEED_MIN_CM_S              (-200.0f)
#define APP_TARGET_SPEED_MAX_CM_S              (200.0f)
#define APP_TARGET_SPEED_STEP_CM_S             (1.0f)
#define APP_TARGET_DISTANCE_MIN_CM             (1.0f)
#define APP_TARGET_DISTANCE_MAX_CM             (10000.0f)
#define APP_TARGET_DISTANCE_STEP_CM            (5.0f)
#define APP_SPEED_PID_KP_MIN                   (0.0f)
#define APP_SPEED_PID_KP_MAX                   (50.0f)
#define APP_SPEED_PID_KP_STEP                  (0.1f)
#define APP_SPEED_PID_KI_MIN                   (0.0f)
#define APP_SPEED_PID_KI_MAX                   (10.0f)
#define APP_SPEED_PID_KI_STEP                  (0.05f)
#define APP_SPEED_PID_KD_MIN                   (0.0f)
#define APP_SPEED_PID_KD_MAX                   (10.0f)
#define APP_SPEED_PID_KD_STEP                  (0.05f)

/* ======================== 自动题目测试参数 ============================= */
#define APP_TEST_START_DELAY_MS                (1000U)
#define APP_TEST_SLOT_COUNT                    (4U)

/* Task 1: odometry-gated finish-line stop for H-problem requirement 2. */
#define APP_TASK1_STOP_DISTANCE_DEFAULT_CM     (610.0f)
#define APP_TASK1_STOP_DISTANCE_MIN_CM         (550.0f)
#define APP_TASK1_STOP_DISTANCE_MAX_CM         (700.0f)
#define APP_TASK1_STOP_DISTANCE_STEP_CM        (1.0f)
#define APP_TASK1_LAP_SPEED_CM_S               (35.0f)
#define APP_TASK1_FINISH_ACTIVE_COUNT          (5U)
/* Keep MotorDebug's internal distance stop beyond the task finish gate. */
#define APP_TASK1_LINE_SAFETY_DISTANCE_CM      (10000.0f)

/* Task 2: smooth line-following run from A to B (AB = 1.5 m). */
#define APP_TASK2_CRUISE_SPEED_CM_S             (25.0f)
#define APP_TASK2_START_SPEED_CM_S              (0.0f)
#define APP_TASK2_END_SPEED_CM_S                (0.0f)
#define APP_TASK2_START_RAMP_MS                 (1000UL)
#define APP_TASK2_STOP_RAMP_MS                  (2500UL)
#define APP_TASK2_ACCEL_DISTANCE_CM             (12.50f)
#define APP_TASK2_CRUISE_DISTANCE_CM            (137.50f)
#define APP_TASK2_TIMED_DISTANCE_CM             \
    (APP_TASK2_ACCEL_DISTANCE_CM + APP_TASK2_CRUISE_DISTANCE_CM)
#define APP_TASK2_EXTENSION_DISTANCE_CM         (40.0f)
#define APP_TASK2_TARGET_DISTANCE_CM            \
    (APP_TASK2_TIMED_DISTANCE_CM + APP_TASK2_EXTENSION_DISTANCE_CM)
#define APP_TASK2_DECEL_DISTANCE_CM             (31.25f)
#define APP_TASK2_FINAL_CRAWL_SPEED_CM_S         (1.0f)

/*
 * Task 3: requirement 5, one smooth lap and pass A before stopping.
 * 20 cm/s cannot meet the 30 s limit on the 6.14 m track, so use 24 cm/s.
 */
#define APP_TASK3_CRUISE_SPEED_CM_S             (24.0f)
#define APP_TASK3_START_SPEED_CM_S              (5.0f)
#define APP_TASK3_APPROACH_SPEED_CM_S           (14.0f)
#define APP_TASK3_START_RAMP_MS                 (2000UL)
#define APP_TASK3_APPROACH_START_CM             (550.0f)
#define APP_TASK3_EXPECTED_FINISH_CM             (610.0f)
#define APP_TASK3_FINISH_GATE_CM                (500.0f)
#define APP_TASK3_FINISH_ACTIVE_COUNT           (4U)
#define APP_TASK3_FINISH_CONFIRM_TICKS          (3U)
#define APP_TASK3_AFTER_MARKER_STOP_CM          (50.0f)
#define APP_TASK3_FINAL_CRAWL_SPEED_CM_S         (5.0f)
#define APP_TASK3_SAFETY_DISTANCE_CM            (750.0f)

#define APP_AUTO_TEST_SPEED_MIN_CM_S           (1.0f)
#define APP_AUTO_TEST_SPEED_MAX_CM_S           (120.0f)
#define APP_AUTO_TEST_SPEED_STEP_CM_S          (1.0f)

#define APP_DEFAULT_SELECTED_TEST_INDEX        (0U)
#define APP_DEFAULT_SQUARE_SPEED_CM_S          (60.0f)
#define APP_DEFAULT_SQUARE_SIDE_CM             (100.0f)
#define APP_SQUARE_SIDE_MIN_CM                 (5.0f)
#define APP_SQUARE_SIDE_MAX_CM                 (10000.0f)
#define APP_SQUARE_SIDE_STEP_CM                (5.0f)
#define APP_SQUARE_TURN_DEG                    (90.0f)
#define APP_SQUARE_TURN_TOLERANCE_DEG          (3.0f)
#define APP_SQUARE_TURN_SPEED_RATIO            (0.4f)
#define APP_SQUARE_TURN_SPEED_MIN_CM_S         (15.0f)
#define APP_SQUARE_TURN_SPEED_MAX_CM_S         (50.0f)

#define APP_DEFAULT_CIRCLE_SPEED_CM_S          (60.0f)
#define APP_DEFAULT_CIRCLE_RADIUS_CM           (50.0f)
#define APP_CIRCLE_RADIUS_MIN_CM               (15.0f)
#define APP_CIRCLE_RADIUS_MAX_CM               (10000.0f)
#define APP_CIRCLE_RADIUS_STEP_CM              (5.0f)
#define APP_CHASSIS_TRACK_WIDTH_CM             (14.0f)

/* ======================= 直行、巡线和角度环参数 ====================== */
/*
 * 编码器直行 PID：误差为“左轮前进路程 - 右轮前进路程”。
 * Kd 使用左右轮速度差作为误差变化的物理量，Ki 按每个 10-ms 周期累加。
 */
#define APP_STRAIGHT_PID_KP                    (5.0f)
#define APP_STRAIGHT_PID_KI                    (0.0f)
#define APP_STRAIGHT_PID_KD                    (0.15f)
#define APP_STRAIGHT_PID_INTEGRAL_LIMIT_CM_S   (15.0f)
#define APP_STRAIGHT_CORRECTION_MAX_RATIO      (0.35f)
#define APP_STRAIGHT_CORRECTION_MAX_CM_S       (30.0f)

#define APP_STRAIGHT_PID_KP_MIN                (0.0f)
#define APP_STRAIGHT_PID_KP_MAX                (20.0f)
#define APP_STRAIGHT_PID_KP_STEP               (0.1f)
#define APP_STRAIGHT_PID_KI_MIN                (0.0f)
#define APP_STRAIGHT_PID_KI_MAX                (5.0f)
#define APP_STRAIGHT_PID_KI_STEP               (0.01f)
#define APP_STRAIGHT_PID_KD_MIN                (0.0f)
#define APP_STRAIGHT_PID_KD_MAX                (5.0f)
#define APP_STRAIGHT_PID_KD_STEP               (0.01f)

/* 12 路灰度巡线 PID：先按旧工程的 PWM 比例计算，再换算成速度差。 */
#define APP_LINE_REFERENCE_BASE_PWM            (1050.0f)
#define APP_LINE_PID_KP_PWM                    (100.0f)
#define APP_LINE_PID_KI_PWM                    (0.0f)
#define APP_LINE_PID_KD_PWM                    (25.0f)
#define APP_LINE_PID_INTEGRAL_LIMIT_PWM        (300.0f)
#define APP_LINE_TURN_LIMIT_PWM                (650.0f)
#define APP_LINE_LOST_RECOVERY_PWM             (500.0f)
#define APP_LINE_LOST_EDGE_ERROR               (1.5f)
#define MOTOR_DEBUG_MIN_SPEED_CM_S             (1.0f)
#define MOTOR_DEBUG_LINE_LOST_MAX_TICKS        (50U)

#define APP_LINE_PID_KP_MIN                    (0.0f)
#define APP_LINE_PID_KP_MAX                    (500.0f)
#define APP_LINE_PID_KP_STEP                   (5.0f)
#define APP_LINE_PID_KI_MIN                    (0.0f)
#define APP_LINE_PID_KI_MAX                    (100.0f)
#define APP_LINE_PID_KI_STEP                   (1.0f)
#define APP_LINE_PID_KD_MIN                    (0.0f)
#define APP_LINE_PID_KD_MAX                    (200.0f)
#define APP_LINE_PID_KD_STEP                   (1.0f)

/*
 * IMU 角度 PID：距离模式启动时记录当前 Yaw，之后以该方向为目标回正。
 * 正常安装时保持极性为 +1；若实车回正方向相反，只改 POLARITY 为 -1。
 */
#define APP_ANGLE_PID_KP                       (0.8f)
#define APP_ANGLE_PID_KI                       (0.0f)
#define APP_ANGLE_PID_KD                       (0.2f)
#define APP_ANGLE_PID_INTEGRAL_LIMIT_CM_S      (10.0f)
#define APP_ANGLE_CORRECTION_MAX_RATIO         (0.25f)
#define APP_ANGLE_CORRECTION_MAX_CM_S          (20.0f)
#define APP_ANGLE_ERROR_DEADBAND_DEG           (0.3f)
#define APP_ANGLE_YAW_POLARITY                 (1.0f)

#define APP_ANGLE_PID_KP_MIN                   (0.0f)
#define APP_ANGLE_PID_KP_MAX                   (10.0f)
#define APP_ANGLE_PID_KP_STEP                  (0.05f)
#define APP_ANGLE_PID_KI_MIN                   (0.0f)
#define APP_ANGLE_PID_KI_MAX                   (2.0f)
#define APP_ANGLE_PID_KI_STEP                  (0.01f)
#define APP_ANGLE_PID_KD_MIN                   (0.0f)
#define APP_ANGLE_PID_KD_MAX                   (10.0f)
#define APP_ANGLE_PID_KD_STEP                  (0.05f)

/* P1~P12 从左到右的误差权重（放大 10 倍）及横线判定通道数。 */
#define APP_LINE_SENSOR_WEIGHTS_X10             \
    {-40, -33, -25, -18, -11, -4, 4, 11, 18, 25, 33, 40}
#define APP_LINE_SENSOR_MARKER_MIN_ACTIVE      (8U)

/* =========================== 按键参数 =============================== */
/*
 * 3 个连续 10-ms 相同样本形成 30-ms 消抖。
 * 五向上/下/左/右首次按下立即触发，继续按住 500 ms 后每 100 ms 连发一次。
 * 中心键仍在 1 s 时只触发一次保存，不参与连发。
 */
#define APP_BUTTON_SCAN_PERIOD_MS              (10U)
#define APP_BUTTON_DEBOUNCE_SAMPLES            (3U)
#define APP_BUTTON_REPEAT_DELAY_MS             (500U)
#define APP_BUTTON_REPEAT_PERIOD_MS            (100U)
#define APP_BUTTON_LONG_PRESS_MS               (1000U)

/* ======================= ICM42688 与姿态参数 ======================== */
/* 安装方向编码：0=正面朝上，1=绕 X 翻转，2=绕 Y 翻转，3=绕 Z 旋转 180°。 */
#define APP_ICM_MOUNT_MODE                     (4U) /* +Y forward, +X left, +Z down */
/* 首选地址；驱动初始化时也会自动探测另一个地址 0x68/0x69。 */
#define ICM42688_I2C_ADDRESS                   (0x68U)

/*
 * 冷启动保护参数：上电后先等待传感器及模块电源稳定，再执行初始化。
 * 若一次初始化失败，驱动会恢复软件 I2C 总线并自动重试；该等待发生在
 * 约 2 秒的启动 LOGO 阶段内，不影响进入数据页面后的非阻塞调度。
 */
#define ICM42688_POWER_ON_DELAY_MS             (50U)
#define ICM42688_INIT_RETRY_COUNT              (3U)
#define ICM42688_INIT_RETRY_DELAY_MS           (20U)
/* 完成量程、ODR 和滤波配置后，等待内部滤波器及陀螺仪输出稳定。 */
#define ICM42688_POST_CONFIG_SETTLE_MS         (50U)

/*
 * 量程、输出速率、数字滤波和低噪声模式寄存器值。
 * 当前组合：陀螺仪 ±500 dps/100 Hz，加速度计 ±2 g/100 Hz，20 Hz 滤波。
 * 如果修改量程，必须同步修改下面两个 LSB 比例，否则物理单位会计算错误。
 */
#define ICM42688_GYRO_500DPS_100HZ             (0x48U)
#define ICM42688_ACCEL_2G_100HZ                (0x68U)
#define ICM42688_ACCEL_GYRO_BW_20HZ            (0x66U)
#define ICM42688_ACCEL_GYRO_LN                 (0x0FU)
#define ACCEL_SCALE_LSB_PER_G                   (16384.0f)
#define GYRO_SCALE_LSB_PER_DPS                  (65.5f)
/* 已在当前新车板上验证的偏航角速度修正系数。 */
#define ICM42688_YAW_RATE_GAIN                  (1.543f)

/* PB14/PA2 software I2C timing and digital interference rejection. */
#define SOFT_I2C_BUS_HZ                        (40000U)
#define SOFT_I2C_STRETCH_TIMEOUT_US            (1000U)
#define SOFT_I2C_FILTER_SAMPLE_DELAY_US        (1U)
#define SOFT_I2C_PHASE_DITHER_STEP_US          (37U)
#define SOFT_I2C_PHASE_DITHER_STEPS            (27U)
#define ICM42688_TRANSACTION_RETRY_COUNT        (5U)
#define ICM42688_TRANSACTION_RETRY_DELAY_MS     (1U)
#define ICM42688_FIXED_VALUE_RETRY_COUNT        (5U)
#define ICM42688_CONFIG_VERIFY_RETRY_COUNT      (3U)

/* 陀螺仪静态零偏校准参数。校准期间必须保持车辆静止。 */
#define ICM42688_DEFAULT_CALIBRATION_SAMPLES   (50U)
#define CALIBRATION_PERIOD_MS                  (10U)
#define CALIBRATION_MAX_CONSECUTIVE_FAILURES  (10U)
#define CALIBRATION_MAX_REJECTED_SAMPLES       (25U)
#define CALIBRATION_MEAN_GUARD_START_SAMPLES   (5U)
#define GYRO_CAL_MAX_ABS_RAW                   (328.0f)
#define GYRO_CAL_MAX_SPREAD_RAW                (131.0f)

/* Mahony 姿态融合参数及加速度可信区间。 */
#define MAHONY_KP                              (0.50f)
#define MAHONY_KI                              (0.005f)
#define INTEGRAL_LIMIT_RAD_S                   (0.10f)
#define OUTPUT_LPF_TAU_S                       (0.025f)
#define YAW_OUTPUT_LPF_TAU_S                   (0.010f)
#define ACCEL_NORM2_MIN                        (0.64f)
#define ACCEL_NORM2_MAX                        (1.44f)

/* ========================= OLED 与 UI 参数 ========================== */
/* 启动 LOGO 保持时间和常规页面提交周期。 */
#define UI_LOGO_VISIBLE_MS                     (2000UL)
#define UI_LOGO_FRAME_SHORT_MS                 (16UL)
#define UI_LOGO_FRAME_LONG_MS                  (17UL)
#define UI_REFRESH_PERIOD_MS                   (100UL)

/* OLED 控制器/模组差异参数：列偏移、SPI 延时、方向、亮度和分片字节数。 */
#define OLED_COLUMN_OFFSET                     (1U)
#define OLED_SPI_DELAY_CYCLES                  (2U)
#define OLED_SHOW_STARTUP_LOGO                 (0U)
#define OLED_SEG_REMAP_COMMAND                 (0xA1U)
#define OLED_COM_SCAN_COMMAND                  (0xC8U)
#define OLED_CONTRAST                          (0xCFU)
#define OLED_SERVICE_BYTES_PER_CALL            (32U)

#endif /* USER_CONFIG_H_ */
