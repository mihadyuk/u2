/*
    ChibiOS/RT - Copyright (C) 2006-2014 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics STM32F4-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_MNU
#define BOARD_NAME                  "Monoblock (MNU)"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000
#endif

#define FRAM_I2CD                   I2CD2

#define MPU6050_I2CD                I2CD3
#define LSM303_I2CD                 I2CD3
#define MS5806_I2CD                 I2CD3
#define NPA700_I2CD                 I2CD3

#define GPSSD                       SD2

#define XBEESD                      SD6
#define XBEE_BAUDRATE               115200
#define XBEE_USE_CTS_RTS            TRUE

#define MODSD                       SD1
#define MOD_BAUDRATE                115200

#define ADIS_SPI                    SPID1 // fake spi port for compileability only

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F407xx

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0
#define GPIOA_PIN1                  1
#define GPIOA_UART2_TX              2
#define GPIOA_UART2_RX              3
#define GPIOA_PIN4                  4
#define GPIOA_PIN5                  5
#define GPIOA_PIN6                  6
#define GPIOA_PIN7                  7
#define GPIOA_MCU_I2C_SCL           8
#define GPIOA_USB_PRESENT           9
#define GPIOA_PIN10                 10
#define GPIOA_OTG_FS_DM             11
#define GPIOA_OTG_FS_DP             12
#define GPIOA_JTMS                  13
#define GPIOA_JTCK                  14
#define GPIOA_JTDI                  15

#define GPIOB_PIN0                  0
#define GPIOB_PIN1                  1
#define GPIOB_FPGA_IO1              2
#define GPIOB_JTDO                  3
#define GPIOB_JTRST                 4
#define GPIOB_PIN5                  5
#define GPIOB_MOD_TX                6
#define GPIOB_MOD_RX                7
#define GPIOB_SDIO_D4               8
#define GPIOB_SDIO_D5               9
#define GPIOB_UBLOX_PPS             10
#define GPIOB_PPS4                  11
#define GPIOB_PIN12                 12
#define GPIOB_PIN13                 13
#define GPIOB_ADIS_MISO             14
#define GPIOB_FPGA_IO2              15

#define GPIOC_ADC_VCC_P             0
#define GPIOC_ADC_VCC_N             1
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_PIN4                  4
#define GPIOC_PIN5                  5
#define GPIOC_SDIO_D6               6
#define GPIOC_SDIO_D7               7
#define GPIOC_SDIO_D0               8
#define GPIOC_SDIO_D1               9
#define GPIOC_SDIO_D2               10
#define GPIOC_SDIO_D3               11
#define GPIOC_SDIO_CLK              12
#define GPIOC_FPGA_IO3              13
#define GPIOC_PIN14                 14
#define GPIOC_PIN15                 15

#define GPIOD_MEM_D2                0
#define GPIOD_MEM_D3                1
#define GPIOD_PIN2                  2
#define GPIOD_MEM_CLK               3
#define GPIOD_MEM_OE                4
#define GPIOD_MEM_WE                5
#define GPIOD_FPGA_IO5              6
#define GPIOD_MEM_NE1               7
#define GPIOD_MEM_D13               8
#define GPIOD_MEM_D14               9
#define GPIOD_MEM_D15               10
#define GPIOD_MEM_A16               11
#define GPIOD_MEM_A17               12
#define GPIOD_MEM_A18               13
#define GPIOD_MEM_D0                14
#define GPIOD_MEM_D1                15

#define GPIOE_MEM_LB                0
#define GPIOE_MEM_UB                1
#define GPIOE_PIN2                  2
#define GPIOE_MEM_A19               3
#define GPIOE_MEM_A20               4
#define GPIOE_MEM_A21               5
#define GPIOE_MEM_A22               6
#define GPIOE_MEM_D4                7
#define GPIOE_MEM_D5                8
#define GPIOE_MEM_D6                9
#define GPIOE_MEM_D7                10
#define GPIOE_MEM_D8                11
#define GPIOE_MEM_D9                12
#define GPIOE_MEM_D10               13
#define GPIOE_MEM_D11               14
#define GPIOE_MEM_D12               15

#define GPIOF_MEM_A0                0
#define GPIOF_MEM_A1                1
#define GPIOF_MEM_A2                2
#define GPIOF_MEM_A3                3
#define GPIOF_MEM_A4                4
#define GPIOF_MEM_A5                5
#define GPIOF_FPGA_IO12             6
#define GPIOF_PIN7                  7
#define GPIOF_PIN8                  8
#define GPIOF_PIN9                  9
#define GPIOF_FPGA_IO6              10
#define GPIOF_FPGA_IO13             11
#define GPIOF_MEM_A6                12
#define GPIOF_MEM_A7                13
#define GPIOF_MEM_A8                14
#define GPIOF_MEM_A9                15

#define GPIOG_MEM_A10               0
#define GPIOG_MEM_A11               1
#define GPIOG_MEM_A12               2
#define GPIOG_MEM_A13               3
#define GPIOG_MEM_A14               4
#define GPIOG_MEM_A15               5
#define GPIOG_MMC_NRST              6
#define GPIOG_MPU9150_INT           7
#define GPIOG_FPGA_IO7              8
#define GPIOG_UART6_RX              9
#define GPIOG_FPGA_IO8              10
#define GPIOG_PIN11                 11
#define GPIOG_UART6_RTS             12
#define GPIOG_FPGA_IO9              13
#define GPIOG_UART6_TX              14
#define GPIOG_UART6_CTS             15

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_NVRAM_I2C_SCL         4
#define GPIOH_NVRAM_I2C_SDA         5
#define GPIOH_PIN6                  6
#define GPIOH_PIN7                  7
#define GPIOH_MCU_I2C_SDA           8
#define GPIOH_NVRAM_PWR             9
#define GPIOH_FPGA_IO10             10
#define GPIOH_PIN11                 11
#define GPIOH_PIN12                 12
#define GPIOH_PIN13                 13
#define GPIOH_MPU6050_PWR           14
#define GPIOH_FPGA_IO11             15

#define GPIOI_ADIS_NSS              0
#define GPIOI_ADIS_SCK              1
#define GPIOI_TIM8_CH4              2
#define GPIOI_ADIS_MOSI             3
#define GPIOI_LED_O                 4
#define GPIOI_TIM8_CH1              5
#define GPIOI_TIM8_CH2              6
#define GPIOI_TIM8_CH3              7
#define GPIOI_LED_R                 8
#define GPIOI_PIN9                  9
#define GPIOI_PIN10                 10
#define GPIOI_LED_G                 11
#define GPIOI_PIN12                 12
#define GPIOI_PIN13                 13
#define GPIOI_PIN14                 14
#define GPIOI_PIN15                 15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN1) |             \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2_RX) |     \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN7) |             \
                                     PIN_MODE_ALTERNATE(GPIOA_MCU_I2C_SCL) |  \
                                     PIN_MODE_INPUT(GPIOA_USB_PRESENT) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN10) |            \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_JTMS) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_JTCK) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_JTDI))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7) |         \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_MCU_I2C_SCL) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_PRESENT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTMS) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTCK) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTDI))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOA_PIN1) |            \
                                     PIN_OSPEED_2M(GPIOA_UART2_TX) |          \
                                     PIN_OSPEED_2M(GPIOA_UART2_RX) |          \
                                     PIN_OSPEED_100M(GPIOA_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOA_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOA_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOA_PIN7) |            \
                                     PIN_OSPEED_2M(GPIOA_MCU_I2C_SCL) |       \
                                     PIN_OSPEED_100M(GPIOA_USB_PRESENT) |     \
                                     PIN_OSPEED_100M(GPIOA_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) |       \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) |       \
                                     PIN_OSPEED_100M(GPIOA_JTMS) |            \
                                     PIN_OSPEED_100M(GPIOA_JTCK) |            \
                                     PIN_OSPEED_100M(GPIOA_JTDI))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_TX) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_RX) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_MCU_I2C_SCL) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_PRESENT) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_JTMS) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_JTCK) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_JTDI))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOA_UART2_TX) |           \
                                     PIN_ODR_HIGH(GPIOA_UART2_RX) |           \
                                     PIN_ODR_HIGH(GPIOA_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOA_MCU_I2C_SCL) |        \
                                     PIN_ODR_HIGH(GPIOA_USB_PRESENT) |        \
                                     PIN_ODR_HIGH(GPIOA_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |          \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |          \
                                     PIN_ODR_HIGH(GPIOA_JTMS) |               \
                                     PIN_ODR_HIGH(GPIOA_JTCK) |               \
                                     PIN_ODR_HIGH(GPIOA_JTDI))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOA_UART2_TX, 7) |         \
                                     PIN_AFIO_AF(GPIOA_UART2_RX, 7) |         \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOA_PIN7, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_MCU_I2C_SCL, 4) |      \
                                     PIN_AFIO_AF(GPIOA_USB_PRESENT, 0) |      \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |       \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |       \
                                     PIN_AFIO_AF(GPIOA_JTMS, 0) |             \
                                     PIN_AFIO_AF(GPIOA_JTCK, 0) |             \
                                     PIN_AFIO_AF(GPIOA_JTDI, 0))

/*
 * GPIOB setup:
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN1) |             \
                                     PIN_MODE_OUTPUT(GPIOB_FPGA_IO1) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_JTDO) |         \
                                     PIN_MODE_ALTERNATE(GPIOB_JTRST) |        \
                                     PIN_MODE_INPUT(GPIOB_PIN5) |             \
                                     PIN_MODE_ALTERNATE(GPIOB_MOD_TX) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_MOD_RX) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_SDIO_D4) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_SDIO_D5) |      \
                                     PIN_MODE_INPUT(GPIOB_UBLOX_PPS) |        \
                                     PIN_MODE_INPUT(GPIOB_PPS4) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOB_PIN13) |            \
                                     PIN_MODE_ALTERNATE(GPIOB_ADIS_MISO) |    \
                                     PIN_MODE_OUTPUT(GPIOB_FPGA_IO2))

#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FPGA_IO1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTDO) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTRST) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOD_TX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOD_RX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SDIO_D4) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SDIO_D5) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_UBLOX_PPS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PPS4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ADIS_MISO) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FPGA_IO2))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN1) |            \
                                     PIN_OSPEED_2M(GPIOB_FPGA_IO1) |          \
                                     PIN_OSPEED_100M(GPIOB_JTDO) |            \
                                     PIN_OSPEED_100M(GPIOB_JTRST) |           \
                                     PIN_OSPEED_100M(GPIOB_PIN5) |            \
                                     PIN_OSPEED_2M(GPIOB_MOD_TX) |            \
                                     PIN_OSPEED_2M(GPIOB_MOD_RX) |            \
                                     PIN_OSPEED_100M(GPIOB_SDIO_D4) |         \
                                     PIN_OSPEED_100M(GPIOB_SDIO_D5) |         \
                                     PIN_OSPEED_100M(GPIOB_UBLOX_PPS) |       \
                                     PIN_OSPEED_100M(GPIOB_PPS4) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOB_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOB_ADIS_MISO) |       \
                                     PIN_OSPEED_2M(GPIOB_FPGA_IO2))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PIN0) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_FPGA_IO1) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_JTDO) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_JTRST) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_MOD_TX) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_MOD_RX) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_SDIO_D4) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_SDIO_D5) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_UBLOX_PPS) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_PPS4) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_ADIS_MISO) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_FPGA_IO2))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOB_FPGA_IO1) |           \
                                     PIN_ODR_HIGH(GPIOB_JTDO) |               \
                                     PIN_ODR_HIGH(GPIOB_JTRST) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOB_MOD_TX) |             \
                                     PIN_ODR_HIGH(GPIOB_MOD_RX) |             \
                                     PIN_ODR_HIGH(GPIOB_SDIO_D4) |            \
                                     PIN_ODR_HIGH(GPIOB_SDIO_D5) |            \
                                     PIN_ODR_HIGH(GPIOB_UBLOX_PPS) |          \
                                     PIN_ODR_HIGH(GPIOB_PPS4) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOB_ADIS_MISO) |          \
                                     PIN_ODR_HIGH(GPIOB_FPGA_IO2))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOB_FPGA_IO1, 0) |         \
                                     PIN_AFIO_AF(GPIOB_JTDO, 0) |             \
                                     PIN_AFIO_AF(GPIOB_JTRST, 0) |            \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOB_MOD_TX, 7) |           \
                                     PIN_AFIO_AF(GPIOB_MOD_RX, 7))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_SDIO_D4, 12) |         \
                                     PIN_AFIO_AF(GPIOB_SDIO_D5, 12) |         \
                                     PIN_AFIO_AF(GPIOB_UBLOX_PPS, 0) |        \
                                     PIN_AFIO_AF(GPIOB_PPS4, 0) |             \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOB_ADIS_MISO, 5) |        \
                                     PIN_AFIO_AF(GPIOB_FPGA_IO2, 0))

/*
 * GPIOC setup:
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_ADC_VCC_P) |       \
                                     PIN_MODE_ANALOG(GPIOC_ADC_VCC_N) |       \
                                     PIN_MODE_INPUT(GPIOC_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN5) |             \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D6) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D7) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D0) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D1) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D2) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D3) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_CLK) |     \
                                     PIN_MODE_INPUT(GPIOC_FPGA_IO3) |         \
                                     PIN_MODE_INPUT(GPIOC_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ADC_VCC_P) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC_VCC_N) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D6) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D7) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CLK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_FPGA_IO3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_ADC_VCC_P) |\
                                     PIN_OSPEED_100M(GPIOC_ADC_VCC_N) |       \
                                     PIN_OSPEED_100M(GPIOC_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D6) |         \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D7) |         \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D0) |         \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D1) |         \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D2) |         \
                                     PIN_OSPEED_100M(GPIOC_SDIO_D3) |         \
                                     PIN_OSPEED_100M(GPIOC_SDIO_CLK) |        \
                                     PIN_OSPEED_100M(GPIOC_FPGA_IO3) |        \
                                     PIN_OSPEED_100M(GPIOC_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_ADC_VCC_P) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC_VCC_N) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D6) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D7) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D2) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D3) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_SDIO_CLK) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_FPGA_IO3) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_ADC_VCC_P) |          \
                                     PIN_ODR_HIGH(GPIOC_ADC_VCC_N) |          \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D6) |            \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D7) |            \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D0) |            \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D1) |            \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D2) |            \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D3) |            \
                                     PIN_ODR_HIGH(GPIOC_SDIO_CLK) |           \
                                     PIN_ODR_HIGH(GPIOC_FPGA_IO3) |           \
                                     PIN_ODR_HIGH(GPIOC_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_ADC_VCC_P, 0) |        \
                                     PIN_AFIO_AF(GPIOC_ADC_VCC_N, 0) |        \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOC_SDIO_D6, 12) |         \
                                     PIN_AFIO_AF(GPIOC_SDIO_D7, 12))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SDIO_D0, 12) |         \
                                     PIN_AFIO_AF(GPIOC_SDIO_D1, 12) |         \
                                     PIN_AFIO_AF(GPIOC_SDIO_D2, 12) |         \
                                     PIN_AFIO_AF(GPIOC_SDIO_D3, 12) |         \
                                     PIN_AFIO_AF(GPIOC_SDIO_CLK, 12) |        \
                                     PIN_AFIO_AF(GPIOC_FPGA_IO3, 0) |         \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0))

/*
 * GPIOD setup:
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_MEM_D2) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_D3) |       \
                                     PIN_MODE_INPUT(GPIOD_PIN2) |             \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_CLK) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_OE) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_WE) |       \
                                     PIN_MODE_INPUT(GPIOD_FPGA_IO5) |         \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_NE1) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_D13) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_D14) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_D15) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_A16) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_A17) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_A18) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_D0) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_MEM_D1))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_MEM_D2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_D3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_CLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_OE) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_WE) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FPGA_IO5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_NE1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_D13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_D14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_D15) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_A16) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_A17) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_A18) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_D0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEM_D1))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_MEM_D2) |          \
                                     PIN_OSPEED_100M(GPIOD_MEM_D3) |          \
                                     PIN_OSPEED_100M(GPIOD_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOD_MEM_CLK) |         \
                                     PIN_OSPEED_100M(GPIOD_MEM_OE) |          \
                                     PIN_OSPEED_100M(GPIOD_MEM_WE) |          \
                                     PIN_OSPEED_100M(GPIOD_FPGA_IO5) |        \
                                     PIN_OSPEED_100M(GPIOD_MEM_NE1) |         \
                                     PIN_OSPEED_100M(GPIOD_MEM_D13) |         \
                                     PIN_OSPEED_100M(GPIOD_MEM_D14) |         \
                                     PIN_OSPEED_100M(GPIOD_MEM_D15) |         \
                                     PIN_OSPEED_100M(GPIOD_MEM_A16) |         \
                                     PIN_OSPEED_100M(GPIOD_MEM_A17) |         \
                                     PIN_OSPEED_100M(GPIOD_MEM_A18) |         \
                                     PIN_OSPEED_100M(GPIOD_MEM_D0) |          \
                                     PIN_OSPEED_100M(GPIOD_MEM_D1))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_MEM_D2) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_D3) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_CLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_OE) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_WE) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_FPGA_IO5) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_NE1) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_D13) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_D14) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_D15) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_A16) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_A17) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_A18) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_D0) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_MEM_D1))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_MEM_D2) |             \
                                     PIN_ODR_HIGH(GPIOD_MEM_D3) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN2) |               \
                                     PIN_ODR_LOW(GPIOD_MEM_CLK) |             \
                                     PIN_ODR_HIGH(GPIOD_MEM_OE) |             \
                                     PIN_ODR_HIGH(GPIOD_MEM_WE) |             \
                                     PIN_ODR_HIGH(GPIOD_FPGA_IO5) |           \
                                     PIN_ODR_HIGH(GPIOD_MEM_NE1) |            \
                                     PIN_ODR_HIGH(GPIOD_MEM_D13) |            \
                                     PIN_ODR_HIGH(GPIOD_MEM_D14) |            \
                                     PIN_ODR_HIGH(GPIOD_MEM_D15) |            \
                                     PIN_ODR_HIGH(GPIOD_MEM_A16) |            \
                                     PIN_ODR_HIGH(GPIOD_MEM_A17) |            \
                                     PIN_ODR_HIGH(GPIOD_MEM_A18) |            \
                                     PIN_ODR_HIGH(GPIOD_MEM_D0) |             \
                                     PIN_ODR_HIGH(GPIOD_MEM_D1))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_MEM_D2, 12) |          \
                                     PIN_AFIO_AF(GPIOD_MEM_D3, 12) |          \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOD_MEM_CLK, 12) |         \
                                     PIN_AFIO_AF(GPIOD_MEM_OE, 12) |          \
                                     PIN_AFIO_AF(GPIOD_MEM_WE, 12) |          \
                                     PIN_AFIO_AF(GPIOD_FPGA_IO5, 0) |         \
                                     PIN_AFIO_AF(GPIOD_MEM_NE1, 12))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_MEM_D13, 12) |         \
                                     PIN_AFIO_AF(GPIOD_MEM_D14, 12) |         \
                                     PIN_AFIO_AF(GPIOD_MEM_D15, 12) |         \
                                     PIN_AFIO_AF(GPIOD_MEM_A16, 12) |         \
                                     PIN_AFIO_AF(GPIOD_MEM_A17, 12) |         \
                                     PIN_AFIO_AF(GPIOD_MEM_A18, 12) |         \
                                     PIN_AFIO_AF(GPIOD_MEM_D0, 12) |          \
                                     PIN_AFIO_AF(GPIOD_MEM_D1, 12))

/*
 * GPIOE setup:
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_MEM_LB) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_UB) |       \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |             \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_A19) |      \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_A20) |      \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_A21) |      \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_A22) |      \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D4) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D5) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D6) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D7) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D8) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D9) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D10) |      \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D11) |      \
                                     PIN_MODE_ALTERNATE(GPIOE_MEM_D12))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_MEM_LB) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_UB) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_A19) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_A20) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_A21) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_A22) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MEM_D12))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(GPIOE_MEM_LB) |          \
                                     PIN_OSPEED_100M(GPIOE_MEM_UB) |          \
                                     PIN_OSPEED_100M(GPIOE_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOE_MEM_A19) |         \
                                     PIN_OSPEED_100M(GPIOE_MEM_A20) |         \
                                     PIN_OSPEED_100M(GPIOE_MEM_A21) |         \
                                     PIN_OSPEED_100M(GPIOE_MEM_A22) |         \
                                     PIN_OSPEED_100M(GPIOE_MEM_D4) |          \
                                     PIN_OSPEED_100M(GPIOE_MEM_D5) |          \
                                     PIN_OSPEED_100M(GPIOE_MEM_D6) |          \
                                     PIN_OSPEED_100M(GPIOE_MEM_D7) |          \
                                     PIN_OSPEED_100M(GPIOE_MEM_D8) |          \
                                     PIN_OSPEED_100M(GPIOE_MEM_D9) |          \
                                     PIN_OSPEED_100M(GPIOE_MEM_D10) |         \
                                     PIN_OSPEED_100M(GPIOE_MEM_D11) |         \
                                     PIN_OSPEED_100M(GPIOE_MEM_D12))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_MEM_LB) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_UB) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_A19) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_A20) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_A21) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_A22) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D4) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D5) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D6) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D7) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D8) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D9) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D10) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D11) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_MEM_D12))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_MEM_LB) |             \
                                     PIN_ODR_HIGH(GPIOE_MEM_UB) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOE_MEM_A19) |            \
                                     PIN_ODR_HIGH(GPIOE_MEM_A20) |            \
                                     PIN_ODR_HIGH(GPIOE_MEM_A21) |            \
                                     PIN_ODR_HIGH(GPIOE_MEM_A22) |            \
                                     PIN_ODR_HIGH(GPIOE_MEM_D4) |             \
                                     PIN_ODR_HIGH(GPIOE_MEM_D5) |             \
                                     PIN_ODR_HIGH(GPIOE_MEM_D6) |             \
                                     PIN_ODR_HIGH(GPIOE_MEM_D7) |             \
                                     PIN_ODR_HIGH(GPIOE_MEM_D8) |             \
                                     PIN_ODR_HIGH(GPIOE_MEM_D9) |             \
                                     PIN_ODR_HIGH(GPIOE_MEM_D10) |            \
                                     PIN_ODR_HIGH(GPIOE_MEM_D11) |            \
                                     PIN_ODR_HIGH(GPIOE_MEM_D12))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_MEM_LB, 12) |          \
                                     PIN_AFIO_AF(GPIOE_MEM_UB, 12) |          \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOE_MEM_A19, 12) |         \
                                     PIN_AFIO_AF(GPIOE_MEM_A20, 12) |         \
                                     PIN_AFIO_AF(GPIOE_MEM_A21, 12) |         \
                                     PIN_AFIO_AF(GPIOE_MEM_A22, 12) |         \
                                     PIN_AFIO_AF(GPIOE_MEM_D4, 12))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_MEM_D5, 12) |          \
                                     PIN_AFIO_AF(GPIOE_MEM_D6, 12) |          \
                                     PIN_AFIO_AF(GPIOE_MEM_D7, 12) |          \
                                     PIN_AFIO_AF(GPIOE_MEM_D8, 12) |          \
                                     PIN_AFIO_AF(GPIOE_MEM_D9, 12) |          \
                                     PIN_AFIO_AF(GPIOE_MEM_D10, 12) |         \
                                     PIN_AFIO_AF(GPIOE_MEM_D11, 12) |         \
                                     PIN_AFIO_AF(GPIOE_MEM_D12, 12))

/*
 * GPIOF setup:
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_MEM_A0) |       \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A1) |       \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A2) |       \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A3) |       \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A4) |       \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A5) |       \
                                     PIN_MODE_INPUT(GPIOF_FPGA_IO12) |        \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOF_FPGA_IO6) |         \
                                     PIN_MODE_INPUT(GPIOF_FPGA_IO13) |        \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A6) |       \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A7) |       \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A8) |       \
                                     PIN_MODE_ALTERNATE(GPIOF_MEM_A9))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_MEM_A0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FPGA_IO12) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FPGA_IO6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FPGA_IO13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_MEM_A9))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(GPIOF_MEM_A0) |          \
                                     PIN_OSPEED_100M(GPIOF_MEM_A1) |          \
                                     PIN_OSPEED_100M(GPIOF_MEM_A2) |          \
                                     PIN_OSPEED_100M(GPIOF_MEM_A3) |          \
                                     PIN_OSPEED_100M(GPIOF_MEM_A4) |          \
                                     PIN_OSPEED_100M(GPIOF_MEM_A5) |          \
                                     PIN_OSPEED_100M(GPIOF_FPGA_IO12) |       \
                                     PIN_OSPEED_100M(GPIOF_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOF_FPGA_IO6) |        \
                                     PIN_OSPEED_100M(GPIOF_FPGA_IO13) |       \
                                     PIN_OSPEED_100M(GPIOF_MEM_A6) |          \
                                     PIN_OSPEED_100M(GPIOF_MEM_A7) |          \
                                     PIN_OSPEED_100M(GPIOF_MEM_A8) |          \
                                     PIN_OSPEED_100M(GPIOF_MEM_A9))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_MEM_A0) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A1) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A2) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A3) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A4) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A5) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_FPGA_IO12) |    \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_FPGA_IO6) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FPGA_IO13) |    \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A6) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A7) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A8) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_MEM_A9))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_MEM_A0) |             \
                                     PIN_ODR_HIGH(GPIOF_MEM_A1) |             \
                                     PIN_ODR_HIGH(GPIOF_MEM_A2) |             \
                                     PIN_ODR_HIGH(GPIOF_MEM_A3) |             \
                                     PIN_ODR_HIGH(GPIOF_MEM_A4) |             \
                                     PIN_ODR_HIGH(GPIOF_MEM_A5) |             \
                                     PIN_ODR_HIGH(GPIOF_FPGA_IO12) |          \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOF_FPGA_IO6) |           \
                                     PIN_ODR_HIGH(GPIOF_FPGA_IO13) |          \
                                     PIN_ODR_HIGH(GPIOF_MEM_A6) |             \
                                     PIN_ODR_HIGH(GPIOF_MEM_A7) |             \
                                     PIN_ODR_HIGH(GPIOF_MEM_A8) |             \
                                     PIN_ODR_HIGH(GPIOF_MEM_A9))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_MEM_A0, 12) |          \
                                     PIN_AFIO_AF(GPIOF_MEM_A1, 12) |          \
                                     PIN_AFIO_AF(GPIOF_MEM_A2, 12) |          \
                                     PIN_AFIO_AF(GPIOF_MEM_A3, 12) |          \
                                     PIN_AFIO_AF(GPIOF_MEM_A4, 12) |          \
                                     PIN_AFIO_AF(GPIOF_MEM_A5, 12) |          \
                                     PIN_AFIO_AF(GPIOF_FPGA_IO12, 0) |        \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOF_FPGA_IO6, 0) |         \
                                     PIN_AFIO_AF(GPIOF_FPGA_IO13, 0) |        \
                                     PIN_AFIO_AF(GPIOF_MEM_A6, 12) |          \
                                     PIN_AFIO_AF(GPIOF_MEM_A7, 12) |          \
                                     PIN_AFIO_AF(GPIOF_MEM_A8, 12) |          \
                                     PIN_AFIO_AF(GPIOF_MEM_A9, 12))

/*
 * GPIOG setup:
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ALTERNATE(GPIOG_MEM_A10) |      \
                                     PIN_MODE_ALTERNATE(GPIOG_MEM_A11) |      \
                                     PIN_MODE_ALTERNATE(GPIOG_MEM_A12) |      \
                                     PIN_MODE_ALTERNATE(GPIOG_MEM_A13) |      \
                                     PIN_MODE_ALTERNATE(GPIOG_MEM_A14) |      \
                                     PIN_MODE_ALTERNATE(GPIOG_MEM_A15) |      \
                                     PIN_MODE_OUTPUT(GPIOG_MMC_NRST) |        \
                                     PIN_MODE_INPUT(GPIOG_MPU9150_INT) |      \
                                     PIN_MODE_INPUT(GPIOG_FPGA_IO7) |         \
                                     PIN_MODE_ALTERNATE(GPIOG_UART6_RX) |     \
                                     PIN_MODE_OUTPUT(GPIOG_FPGA_IO8) |        \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |            \
                                     PIN_MODE_ALTERNATE(GPIOG_UART6_RTS) |    \
                                     PIN_MODE_INPUT(GPIOG_FPGA_IO9) |         \
                                     PIN_MODE_ALTERNATE(GPIOG_UART6_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOG_UART6_CTS))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_MEM_A10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MEM_A11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MEM_A12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MEM_A13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MEM_A14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MEM_A15) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_MMC_NRST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MPU9150_INT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FPGA_IO7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_UART6_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FPGA_IO8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_UART6_RTS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FPGA_IO9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_UART6_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_UART6_CTS))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_100M(GPIOG_MEM_A10) |         \
                                     PIN_OSPEED_100M(GPIOG_MEM_A11) |         \
                                     PIN_OSPEED_100M(GPIOG_MEM_A12) |         \
                                     PIN_OSPEED_100M(GPIOG_MEM_A13) |         \
                                     PIN_OSPEED_100M(GPIOG_MEM_A14) |         \
                                     PIN_OSPEED_100M(GPIOG_MEM_A15) |         \
                                     PIN_OSPEED_2M(GPIOG_MMC_NRST) |          \
                                     PIN_OSPEED_2M(GPIOG_MPU9150_INT) |       \
                                     PIN_OSPEED_100M(GPIOG_FPGA_IO7) |        \
                                     PIN_OSPEED_2M(GPIOG_UART6_RX) |          \
                                     PIN_OSPEED_2M(GPIOG_FPGA_IO8) |          \
                                     PIN_OSPEED_100M(GPIOG_PIN11) |           \
                                     PIN_OSPEED_2M(GPIOG_UART6_RTS) |         \
                                     PIN_OSPEED_100M(GPIOG_FPGA_IO9) |        \
                                     PIN_OSPEED_2M(GPIOG_UART6_TX) |          \
                                     PIN_OSPEED_2M(GPIOG_UART6_CTS))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_MEM_A10) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_MEM_A11) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_MEM_A12) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_MEM_A13) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_MEM_A14) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_MEM_A15) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_MMC_NRST) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_MPU9150_INT) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_FPGA_IO7) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_UART6_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_FPGA_IO8) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) |        \
                                     PIN_PUPDR_FLOATING(GPIOG_UART6_RTS) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FPGA_IO9) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_UART6_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_UART6_CTS))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_MEM_A10) |            \
                                     PIN_ODR_HIGH(GPIOG_MEM_A11) |            \
                                     PIN_ODR_HIGH(GPIOG_MEM_A12) |            \
                                     PIN_ODR_HIGH(GPIOG_MEM_A13) |            \
                                     PIN_ODR_HIGH(GPIOG_MEM_A14) |            \
                                     PIN_ODR_HIGH(GPIOG_MEM_A15) |            \
                                     PIN_ODR_HIGH(GPIOG_MMC_NRST) |           \
                                     PIN_ODR_HIGH(GPIOG_MPU9150_INT) |        \
                                     PIN_ODR_HIGH(GPIOG_FPGA_IO7) |           \
                                     PIN_ODR_HIGH(GPIOG_UART6_RX) |           \
                                     PIN_ODR_HIGH(GPIOG_FPGA_IO8) |           \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOG_UART6_RTS) |          \
                                     PIN_ODR_HIGH(GPIOG_FPGA_IO9) |           \
                                     PIN_ODR_HIGH(GPIOG_UART6_TX) |           \
                                     PIN_ODR_HIGH(GPIOG_UART6_CTS))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_MEM_A10, 12) |         \
                                     PIN_AFIO_AF(GPIOG_MEM_A11, 12) |         \
                                     PIN_AFIO_AF(GPIOG_MEM_A12, 12) |         \
                                     PIN_AFIO_AF(GPIOG_MEM_A13, 12) |         \
                                     PIN_AFIO_AF(GPIOG_MEM_A14, 12) |         \
                                     PIN_AFIO_AF(GPIOG_MEM_A15, 12) |         \
                                     PIN_AFIO_AF(GPIOG_MMC_NRST, 0) |         \
                                     PIN_AFIO_AF(GPIOG_MPU9150_INT, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_FPGA_IO7, 0) |         \
                                     PIN_AFIO_AF(GPIOG_UART6_RX, 8) |         \
                                     PIN_AFIO_AF(GPIOG_FPGA_IO8, 0) |         \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOG_UART6_RTS, 8) |        \
                                     PIN_AFIO_AF(GPIOG_FPGA_IO9, 0) |         \
                                     PIN_AFIO_AF(GPIOG_UART6_TX, 8) |         \
                                     PIN_AFIO_AF(GPIOG_UART6_CTS, 8))

/*
 * GPIOH setup:
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |           \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |             \
                                     PIN_MODE_ALTERNATE(GPIOH_NVRAM_I2C_SCL) |\
                                     PIN_MODE_ALTERNATE(GPIOH_NVRAM_I2C_SDA) |\
                                     PIN_MODE_INPUT(GPIOH_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |             \
                                     PIN_MODE_ALTERNATE(GPIOH_MCU_I2C_SDA) |  \
                                     PIN_MODE_OUTPUT(GPIOH_NVRAM_PWR) |       \
                                     PIN_MODE_INPUT(GPIOH_FPGA_IO10) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |            \
                                     PIN_MODE_OUTPUT(GPIOH_MPU6050_PWR) |     \
                                     PIN_MODE_INPUT(GPIOH_FPGA_IO11))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |         \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_NVRAM_I2C_SCL) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOH_NVRAM_I2C_SDA) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |         \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_MCU_I2C_SDA) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_NVRAM_PWR) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FPGA_IO10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |        \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_MPU6050_PWR) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FPGA_IO11))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_100M(GPIOH_OSC_IN) |          \
                                     PIN_OSPEED_100M(GPIOH_OSC_OUT) |         \
                                     PIN_OSPEED_100M(GPIOH_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN3) |            \
                                     PIN_OSPEED_2M(GPIOH_NVRAM_I2C_SCL) |     \
                                     PIN_OSPEED_2M(GPIOH_NVRAM_I2C_SDA) |     \
                                     PIN_OSPEED_100M(GPIOH_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN7) |            \
                                     PIN_OSPEED_2M(GPIOH_MCU_I2C_SDA) |       \
                                     PIN_OSPEED_2M(GPIOH_NVRAM_PWR) |         \
                                     PIN_OSPEED_100M(GPIOH_FPGA_IO10) |       \
                                     PIN_OSPEED_100M(GPIOH_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN13) |           \
                                     PIN_OSPEED_2M(GPIOH_MPU6050_PWR) |       \
                                     PIN_OSPEED_100M(GPIOH_FPGA_IO11))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_NVRAM_I2C_SCL) |\
                                     PIN_PUPDR_FLOATING(GPIOH_NVRAM_I2C_SDA) |\
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_MCU_I2C_SDA) |  \
                                     PIN_PUPDR_FLOATING(GPIOH_NVRAM_PWR) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FPGA_IO10) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11) |        \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOH_MPU6050_PWR) |  \
                                     PIN_PUPDR_FLOATING(GPIOH_FPGA_IO11))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |             \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOH_NVRAM_I2C_SCL) |      \
                                     PIN_ODR_HIGH(GPIOH_NVRAM_I2C_SDA) |      \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOH_MCU_I2C_SDA) |        \
                                     PIN_ODR_HIGH(GPIOH_NVRAM_PWR) |          \
                                     PIN_ODR_HIGH(GPIOH_FPGA_IO10) |          \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOH_MPU6050_PWR) |        \
                                     PIN_ODR_HIGH(GPIOH_FPGA_IO11))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) |           \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOH_NVRAM_I2C_SCL, 4) |    \
                                     PIN_AFIO_AF(GPIOH_NVRAM_I2C_SDA, 4) |    \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_MCU_I2C_SDA, 4) |      \
                                     PIN_AFIO_AF(GPIOH_NVRAM_PWR, 0) |        \
                                     PIN_AFIO_AF(GPIOH_FPGA_IO10, 0) |        \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOH_MPU6050_PWR, 0) |      \
                                     PIN_AFIO_AF(GPIOH_FPGA_IO11, 0))

/*
 * GPIOI setup:
 */
#define VAL_GPIOI_MODER             (PIN_MODE_OUTPUT(GPIOI_ADIS_NSS) |        \
                                     PIN_MODE_ALTERNATE(GPIOI_ADIS_SCK) |     \
                                     PIN_MODE_INPUT(GPIOI_TIM8_CH4) |         \
                                     PIN_MODE_ALTERNATE(GPIOI_ADIS_MOSI) |    \
                                     PIN_MODE_OUTPUT(GPIOI_LED_O) |           \
                                     PIN_MODE_INPUT(GPIOI_TIM8_CH1) |         \
                                     PIN_MODE_INPUT(GPIOI_TIM8_CH2) |         \
                                     PIN_MODE_INPUT(GPIOI_TIM8_CH3) |         \
                                     PIN_MODE_OUTPUT(GPIOI_LED_R) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |            \
                                     PIN_MODE_OUTPUT(GPIOI_LED_G) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_ADIS_NSS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_ADIS_SCK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_TIM8_CH4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_ADIS_MOSI) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LED_O) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_TIM8_CH1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_TIM8_CH2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_TIM8_CH3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LED_R) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LED_G) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_100M(GPIOI_ADIS_NSS) |        \
                                     PIN_OSPEED_100M(GPIOI_ADIS_SCK) |        \
                                     PIN_OSPEED_100M(GPIOI_TIM8_CH4) |        \
                                     PIN_OSPEED_100M(GPIOI_ADIS_MOSI) |       \
                                     PIN_OSPEED_100M(GPIOI_LED_O) |           \
                                     PIN_OSPEED_100M(GPIOI_TIM8_CH1) |        \
                                     PIN_OSPEED_100M(GPIOI_TIM8_CH2) |        \
                                     PIN_OSPEED_100M(GPIOI_TIM8_CH3) |        \
                                     PIN_OSPEED_100M(GPIOI_LED_R) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOI_LED_G) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_ADIS_NSS) |     \
                                     PIN_PUPDR_FLOATING(GPIOI_ADIS_SCK) |     \
                                     PIN_PUPDR_FLOATING(GPIOI_TIM8_CH4) |     \
                                     PIN_PUPDR_FLOATING(GPIOI_ADIS_MOSI) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_LED_O) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_TIM8_CH1) |     \
                                     PIN_PUPDR_FLOATING(GPIOI_TIM8_CH2) |     \
                                     PIN_PUPDR_FLOATING(GPIOI_TIM8_CH3) |     \
                                     PIN_PUPDR_FLOATING(GPIOI_LED_R) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_LED_G) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN14) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_ADIS_NSS) |           \
                                     PIN_ODR_LOW(GPIOI_ADIS_SCK) |            \
                                     PIN_ODR_LOW(GPIOI_TIM8_CH4) |            \
                                     PIN_ODR_HIGH(GPIOI_ADIS_MOSI) |          \
                                     PIN_ODR_LOW(GPIOI_LED_O) |               \
                                     PIN_ODR_LOW(GPIOI_TIM8_CH1) |            \
                                     PIN_ODR_LOW(GPIOI_TIM8_CH2) |            \
                                     PIN_ODR_LOW(GPIOI_TIM8_CH3) |            \
                                     PIN_ODR_LOW(GPIOI_LED_R) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |              \
                                     PIN_ODR_LOW(GPIOI_LED_G) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |              \
                                     PIN_ODR_LOW(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_ADIS_NSS, 0) |         \
                                     PIN_AFIO_AF(GPIOI_ADIS_SCK, 5) |         \
                                     PIN_AFIO_AF(GPIOI_TIM8_CH4, 0) |         \
                                     PIN_AFIO_AF(GPIOI_ADIS_MOSI, 5) |        \
                                     PIN_AFIO_AF(GPIOI_LED_O, 0) |            \
                                     PIN_AFIO_AF(GPIOI_TIM8_CH1, 0) |         \
                                     PIN_AFIO_AF(GPIOI_TIM8_CH2, 0) |         \
                                     PIN_AFIO_AF(GPIOI_TIM8_CH3, 0))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_LED_R, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOI_LED_G, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0))

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  bool FPGAReady(void);
  unsigned int usb_lld_plug_state(void);
  void usb_lld_connect_bus_workaround(void);
  void usb_lld_disconnect_bus_workaround(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
