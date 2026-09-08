/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000


/* Defines for UART_1: MaixCam communication, 115200 8-N-1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                    (UART1_INT_IRQn)
#define UART_1_BAUD_RATE                                              (115200U)
#define UART_1_IBRD_32_MHZ_115200_BAUD                                  (17U)
#define UART_1_FBRD_32_MHZ_115200_BAUD                                  (23U)
#define GPIO_UART_1_TX_PORT                                               GPIOA
#define GPIO_UART_1_TX_PIN                                        DL_GPIO_PIN_8
#define GPIO_UART_1_TX_IOMUX                                     (IOMUX_PINCM19)
#define GPIO_UART_1_TX_IOMUX_FUNC                    IOMUX_PINCM19_PF_UART1_TX
#define GPIO_UART_1_RX_PORT                                               GPIOA
#define GPIO_UART_1_RX_PIN                                        DL_GPIO_PIN_9
#define GPIO_UART_1_RX_IOMUX                                     (IOMUX_PINCM20)
#define GPIO_UART_1_RX_IOMUX_FUNC                    IOMUX_PINCM20_PF_UART1_RX



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              1000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_3
#define GPIO_PWM_0_C1_IOMUX                                       (IOMUX_PINCM8)
#define GPIO_PWM_0_C1_IOMUX_FUNC                      IOMUX_PINCM8_PF_TIMA0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMA1
#define PWM_1_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                              1000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_1_C0_PORT                                                 GPIOA
#define GPIO_PWM_1_C0_PIN                                         DL_GPIO_PIN_15
#define GPIO_PWM_1_C0_IOMUX                                      (IOMUX_PINCM37)
#define GPIO_PWM_1_C0_IOMUX_FUNC                     IOMUX_PINCM37_PF_TIMA1_CCP0
#define GPIO_PWM_1_C0_IDX                                    DL_TIMER_CC_0_INDEX



/* Defines for CONTROL_TIMER */
#define CONTROL_TIMER_INST                                               (TIMG0)
#define CONTROL_TIMER_INST_IRQHandler                           TIMG0_IRQHandler
#define CONTROL_TIMER_INST_INT_IRQN                             (TIMG0_INT_IRQn)
#define CONTROL_TIMER_INST_LOAD_VALUE                                   (31999U)




/* Defines for BATTERY_ADC */
#define BATTERY_ADC_INST                                                    ADC0
#define BATTERY_ADC_INST_IRQHandler                              ADC0_IRQHandler
#define BATTERY_ADC_INST_INT_IRQN                                (ADC0_INT_IRQn)
#define BATTERY_ADC_ADCMEM_0                                  DL_ADC12_MEM_IDX_0
#define BATTERY_ADC_ADCMEM_0_REF                 DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define BATTERY_ADC_ADCMEM_0_REF_VOLTAGE_V                                     3.3
#define GPIO_BATTERY_ADC_C1_PORT                                           GPIOA
#define GPIO_BATTERY_ADC_C1_PIN                                   DL_GPIO_PIN_26



/* Port definition for Pin Group ICM_SDA */
#define ICM_SDA_PORT                                                     (GPIOB)

/* Defines for SW_SDA: GPIOB.14 with pinCMx 31 on package pin 2 */
#define ICM_SDA_SW_SDA_PIN                                      (DL_GPIO_PIN_14)
#define ICM_SDA_SW_SDA_IOMUX                                     (IOMUX_PINCM31)
/* Port definition for Pin Group ICM_SCL */
#define ICM_SCL_PORT                                                     (GPIOA)

/* Defines for SW_SCL: GPIOA.2 with pinCMx 7 on package pin 42 */
#define ICM_SCL_SW_SCL_PIN                                       (DL_GPIO_PIN_2)
#define ICM_SCL_SW_SCL_IOMUX                                      (IOMUX_PINCM7)
/* Port definition for Pin Group PAGE_PREV */
#define PAGE_PREV_PORT                                                   (GPIOA)

/* Defines for PREV: GPIOA.18 with pinCMx 40 on package pin 11 */
#define PAGE_PREV_PREV_PIN                                      (DL_GPIO_PIN_18)
#define PAGE_PREV_PREV_IOMUX                                     (IOMUX_PINCM40)
/* Port definition for Pin Group PAGE_NEXT */
#define PAGE_NEXT_PORT                                                   (GPIOB)

/* Defines for NEXT: GPIOB.21 with pinCMx 49 on package pin 20 */
#define PAGE_NEXT_NEXT_PIN                                      (DL_GPIO_PIN_21)
#define PAGE_NEXT_NEXT_IOMUX                                     (IOMUX_PINCM49)
/* Port definition for Pin Group BUZZER_PIN */
#define BUZZER_PIN_PORT                                                  (GPIOA)

/* Defines for BUZZER: GPIOA.27 with pinCMx 60 on package pin 31 */
#define BUZZER_PIN_BUZZER_PIN                                   (DL_GPIO_PIN_27)
#define BUZZER_PIN_BUZZER_IOMUX                                  (IOMUX_PINCM60)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOB)

/* Defines for LEFT_A: GPIOB.5 with pinCMx 18 on package pin 53 */
// pins affected by this interrupt request:["LEFT_A","LEFT_B","RIGHT_A","RIGHT_B"]
#define ENCODER_INT_IRQN                                        (GPIOB_INT_IRQn)
#define ENCODER_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODER_LEFT_A_IIDX                                  (DL_GPIO_IIDX_DIO5)
#define ENCODER_LEFT_A_PIN                                       (DL_GPIO_PIN_5)
#define ENCODER_LEFT_A_IOMUX                                     (IOMUX_PINCM18)
/* Defines for LEFT_B: GPIOB.7 with pinCMx 24 on package pin 59 */
#define ENCODER_LEFT_B_IIDX                                  (DL_GPIO_IIDX_DIO7)
#define ENCODER_LEFT_B_PIN                                       (DL_GPIO_PIN_7)
#define ENCODER_LEFT_B_IOMUX                                     (IOMUX_PINCM24)
/* Defines for RIGHT_A: GPIOB.4 with pinCMx 17 on package pin 52 */
#define ENCODER_RIGHT_A_IIDX                                 (DL_GPIO_IIDX_DIO4)
#define ENCODER_RIGHT_A_PIN                                      (DL_GPIO_PIN_4)
#define ENCODER_RIGHT_A_IOMUX                                    (IOMUX_PINCM17)
/* Defines for RIGHT_B: GPIOB.6 with pinCMx 23 on package pin 58 */
#define ENCODER_RIGHT_B_IIDX                                 (DL_GPIO_IIDX_DIO6)
#define ENCODER_RIGHT_B_PIN                                      (DL_GPIO_PIN_6)
#define ENCODER_RIGHT_B_IOMUX                                    (IOMUX_PINCM23)
/* Port definition for Pin Group KEY5 */
#define KEY5_PORT                                                        (GPIOB)

/* Defines for UP: GPIOB.12 with pinCMx 29 on package pin 64 */
#define KEY5_UP_PIN                                             (DL_GPIO_PIN_12)
#define KEY5_UP_IOMUX                                            (IOMUX_PINCM29)
/* Defines for DOWN: GPIOB.8 with pinCMx 25 on package pin 60 */
#define KEY5_DOWN_PIN                                            (DL_GPIO_PIN_8)
#define KEY5_DOWN_IOMUX                                          (IOMUX_PINCM25)
/* Defines for LEFT: GPIOB.9 with pinCMx 26 on package pin 61 */
#define KEY5_LEFT_PIN                                            (DL_GPIO_PIN_9)
#define KEY5_LEFT_IOMUX                                          (IOMUX_PINCM26)
/* Defines for RIGHT: GPIOB.10 with pinCMx 27 on package pin 62 */
#define KEY5_RIGHT_PIN                                          (DL_GPIO_PIN_10)
#define KEY5_RIGHT_IOMUX                                         (IOMUX_PINCM27)
/* Defines for CENTER: GPIOB.11 with pinCMx 28 on package pin 63 */
#define KEY5_CENTER_PIN                                         (DL_GPIO_PIN_11)
#define KEY5_CENTER_IOMUX                                        (IOMUX_PINCM28)
/* Defines for SCL: GPIOA.17 with pinCMx 39 on package pin 10 */
#define OLED_PINS_SCL_PORT                                               (GPIOA)
#define OLED_PINS_SCL_PIN                                       (DL_GPIO_PIN_17)
#define OLED_PINS_SCL_IOMUX                                      (IOMUX_PINCM39)
/* Defines for SDA: GPIOB.15 with pinCMx 32 on package pin 3 */
#define OLED_PINS_SDA_PORT                                               (GPIOB)
#define OLED_PINS_SDA_PIN                                       (DL_GPIO_PIN_15)
#define OLED_PINS_SDA_IOMUX                                      (IOMUX_PINCM32)
/* Defines for RST: GPIOB.16 with pinCMx 33 on package pin 4 */
#define OLED_PINS_RST_PORT                                               (GPIOB)
#define OLED_PINS_RST_PIN                                       (DL_GPIO_PIN_16)
#define OLED_PINS_RST_IOMUX                                      (IOMUX_PINCM33)
/* Defines for DC: GPIOB.17 with pinCMx 43 on package pin 14 */
#define OLED_PINS_DC_PORT                                                (GPIOB)
#define OLED_PINS_DC_PIN                                        (DL_GPIO_PIN_17)
#define OLED_PINS_DC_IOMUX                                       (IOMUX_PINCM43)
/* Defines for OLED_CS: GPIOB.20 with pinCMx 48 on package pin 19 */
#define OLED_PINS_OLED_CS_PORT                                           (GPIOB)
#define OLED_PINS_OLED_CS_PIN                                   (DL_GPIO_PIN_20)
#define OLED_PINS_OLED_CS_IOMUX                                  (IOMUX_PINCM48)
/* Defines for SCLK: GPIOA.12 with pinCMx 34 on package pin 5 */
#define FLASH_PINS_SCLK_PORT                                             (GPIOA)
#define FLASH_PINS_SCLK_PIN                                     (DL_GPIO_PIN_12)
#define FLASH_PINS_SCLK_IOMUX                                    (IOMUX_PINCM34)
/* Defines for MISO: GPIOA.13 with pinCMx 35 on package pin 6 */
#define FLASH_PINS_MISO_PORT                                             (GPIOA)
#define FLASH_PINS_MISO_PIN                                     (DL_GPIO_PIN_13)
#define FLASH_PINS_MISO_IOMUX                                    (IOMUX_PINCM35)
/* Defines for MOSI: GPIOA.14 with pinCMx 36 on package pin 7 */
#define FLASH_PINS_MOSI_PORT                                             (GPIOA)
#define FLASH_PINS_MOSI_PIN                                     (DL_GPIO_PIN_14)
#define FLASH_PINS_MOSI_IOMUX                                    (IOMUX_PINCM36)
/* Defines for CS: GPIOB.25 with pinCMx 56 on package pin 27 */
#define FLASH_PINS_CS_PORT                                               (GPIOB)
#define FLASH_PINS_CS_PIN                                       (DL_GPIO_PIN_25)
#define FLASH_PINS_CS_IOMUX                                      (IOMUX_PINCM56)
/* Defines for P1: GPIOA.31 with pinCMx 6 on package pin 39 */
#define LINE_SENSORS_P1_PORT                                             (GPIOA)
#define LINE_SENSORS_P1_PIN                                     (DL_GPIO_PIN_31)
#define LINE_SENSORS_P1_IOMUX                                     (IOMUX_PINCM6)
/* Defines for P2: GPIOA.28 with pinCMx 3 on package pin 35 */
#define LINE_SENSORS_P2_PORT                                             (GPIOA)
#define LINE_SENSORS_P2_PIN                                     (DL_GPIO_PIN_28)
#define LINE_SENSORS_P2_IOMUX                                     (IOMUX_PINCM3)
/* Defines for P3: GPIOA.1 with pinCMx 2 on package pin 34 */
#define LINE_SENSORS_P3_PORT                                             (GPIOA)
#define LINE_SENSORS_P3_PIN                                      (DL_GPIO_PIN_1)
#define LINE_SENSORS_P3_IOMUX                                     (IOMUX_PINCM2)
/* Defines for P4: GPIOA.0 with pinCMx 1 on package pin 33 */
#define LINE_SENSORS_P4_PORT                                             (GPIOA)
#define LINE_SENSORS_P4_PIN                                      (DL_GPIO_PIN_0)
#define LINE_SENSORS_P4_IOMUX                                     (IOMUX_PINCM1)
/* Defines for P5: GPIOA.25 with pinCMx 55 on package pin 26 */
#define LINE_SENSORS_P5_PORT                                             (GPIOA)
#define LINE_SENSORS_P5_PIN                                     (DL_GPIO_PIN_25)
#define LINE_SENSORS_P5_IOMUX                                    (IOMUX_PINCM55)
/* Defines for P6: GPIOA.24 with pinCMx 54 on package pin 25 */
#define LINE_SENSORS_P6_PORT                                             (GPIOA)
#define LINE_SENSORS_P6_PIN                                     (DL_GPIO_PIN_24)
#define LINE_SENSORS_P6_IOMUX                                    (IOMUX_PINCM54)
/* Defines for P7: GPIOB.24 with pinCMx 52 on package pin 23 */
#define LINE_SENSORS_P7_PORT                                             (GPIOB)
#define LINE_SENSORS_P7_PIN                                     (DL_GPIO_PIN_24)
#define LINE_SENSORS_P7_IOMUX                                    (IOMUX_PINCM52)
/* Defines for P8: GPIOB.23 with pinCMx 51 on package pin 22 */
#define LINE_SENSORS_P8_PORT                                             (GPIOB)
#define LINE_SENSORS_P8_PIN                                     (DL_GPIO_PIN_23)
#define LINE_SENSORS_P8_IOMUX                                    (IOMUX_PINCM51)
/* Defines for P9: GPIOB.19 with pinCMx 45 on package pin 16 */
#define LINE_SENSORS_P9_PORT                                             (GPIOB)
#define LINE_SENSORS_P9_PIN                                     (DL_GPIO_PIN_19)
#define LINE_SENSORS_P9_IOMUX                                    (IOMUX_PINCM45)
/* Defines for P10: GPIOB.18 with pinCMx 44 on package pin 15 */
#define LINE_SENSORS_P10_PORT                                            (GPIOB)
#define LINE_SENSORS_P10_PIN                                    (DL_GPIO_PIN_18)
#define LINE_SENSORS_P10_IOMUX                                   (IOMUX_PINCM44)
/* Defines for P11: GPIOA.16 with pinCMx 38 on package pin 9 */
#define LINE_SENSORS_P11_PORT                                            (GPIOA)
#define LINE_SENSORS_P11_PIN                                    (DL_GPIO_PIN_16)
#define LINE_SENSORS_P11_IOMUX                                   (IOMUX_PINCM38)
/* Defines for P12: GPIOB.13 with pinCMx 30 on package pin 1 */
#define LINE_SENSORS_P12_PORT                                            (GPIOB)
#define LINE_SENSORS_P12_PIN                                    (DL_GPIO_PIN_13)
#define LINE_SENSORS_P12_IOMUX                                   (IOMUX_PINCM30)
/* Port definition for Pin Group RGB_PINS */
#define RGB_PINS_PORT                                                    (GPIOB)

/* Defines for RED: GPIOB.26 with pinCMx 57 on package pin 28 */
#define RGB_PINS_RED_PIN                                        (DL_GPIO_PIN_26)
#define RGB_PINS_RED_IOMUX                                       (IOMUX_PINCM57)
/* Defines for GREEN: GPIOB.27 with pinCMx 58 on package pin 29 */
#define RGB_PINS_GREEN_PIN                                      (DL_GPIO_PIN_27)
#define RGB_PINS_GREEN_IOMUX                                     (IOMUX_PINCM58)
/* Defines for BLUE: GPIOB.22 with pinCMx 50 on package pin 21 */
#define RGB_PINS_BLUE_PIN                                       (DL_GPIO_PIN_22)
#define RGB_PINS_BLUE_IOMUX                                      (IOMUX_PINCM50)
/* Defines for AIN1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define TB6612_DIR_AIN1_PORT                                             (GPIOA)
#define TB6612_DIR_AIN1_PIN                                      (DL_GPIO_PIN_7)
#define TB6612_DIR_AIN1_IOMUX                                    (IOMUX_PINCM14)
/* Defines for AIN2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define TB6612_DIR_AIN2_PORT                                             (GPIOB)
#define TB6612_DIR_AIN2_PIN                                      (DL_GPIO_PIN_1)
#define TB6612_DIR_AIN2_IOMUX                                    (IOMUX_PINCM13)
/* Defines for BIN1: GPIOB.2 with pinCMx 15 on package pin 50 */
#define TB6612_DIR_BIN1_PORT                                             (GPIOB)
#define TB6612_DIR_BIN1_PIN                                      (DL_GPIO_PIN_2)
#define TB6612_DIR_BIN1_IOMUX                                    (IOMUX_PINCM15)
/* Defines for BIN2: GPIOB.3 with pinCMx 16 on package pin 51 */
#define TB6612_DIR_BIN2_PORT                                             (GPIOB)
#define TB6612_DIR_BIN2_PIN                                      (DL_GPIO_PIN_3)
#define TB6612_DIR_BIN2_IOMUX                                    (IOMUX_PINCM16)
/* Port definition for Pin Group NRF24_PINS */
#define NRF24_PINS_PORT                                                  (GPIOA)

/* Defines for RADIO_SCK: GPIOA.11 with pinCMx 22 on package pin 57 */
#define NRF24_PINS_RADIO_SCK_PIN                                (DL_GPIO_PIN_11)
#define NRF24_PINS_RADIO_SCK_IOMUX                               (IOMUX_PINCM22)
/* Defines for RADIO_MOSI: GPIOA.10 with pinCMx 21 on package pin 56 */
#define NRF24_PINS_RADIO_MOSI_PIN                               (DL_GPIO_PIN_10)
#define NRF24_PINS_RADIO_MOSI_IOMUX                              (IOMUX_PINCM21)
/* Defines for RADIO_MISO: GPIOA.23 with pinCMx 53 on package pin 24 */
#define NRF24_PINS_RADIO_MISO_PIN                               (DL_GPIO_PIN_23)
#define NRF24_PINS_RADIO_MISO_IOMUX                              (IOMUX_PINCM53)
/* Defines for RADIO_CSN: GPIOA.29 with pinCMx 4 on package pin 36 */
#define NRF24_PINS_RADIO_CSN_PIN                                (DL_GPIO_PIN_29)
#define NRF24_PINS_RADIO_CSN_IOMUX                                (IOMUX_PINCM4)
/* Defines for RADIO_CE: GPIOA.21 with pinCMx 46 on package pin 17 */
#define NRF24_PINS_RADIO_CE_PIN                                 (DL_GPIO_PIN_21)
#define NRF24_PINS_RADIO_CE_IOMUX                                (IOMUX_PINCM46)
/* Defines for RADIO_IRQ: GPIOA.30 with pinCMx 5 on package pin 37 */
// pins affected by this interrupt request:["RADIO_IRQ"]
#define NRF24_PINS_INT_IRQN                                     (GPIOA_INT_IRQn)
#define NRF24_PINS_INT_IIDX                     (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define NRF24_PINS_RADIO_IRQ_IIDX                           (DL_GPIO_IIDX_DIO30)
#define NRF24_PINS_RADIO_IRQ_PIN                                (DL_GPIO_PIN_30)
#define NRF24_PINS_RADIO_IRQ_IOMUX                                (IOMUX_PINCM5)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_CONTROL_TIMER_init(void);
void SYSCFG_DL_BATTERY_ADC_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
