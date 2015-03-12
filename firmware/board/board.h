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
 * Human readable aliases for different peripheral drivers.
 */
#define I2CD_SLOW               I2CD1
#define I2CD_FAST               I2CD2

#define XBEESD                  SD3
#define GPSSD                   SD1

#define XBEE_BAUDRATE           115200
#define XBEE_USE_CTS_RTS        TRUE

/*
 * Board identifier.
 */
#define BOARD_BEZVODIATEL
#define BOARD_NAME              "Hand made STM32F4x board"

/*
 * Board frequencies.
 * NOTE: The LSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD               300

/*
 * MCU type as defined in the ST header.
 */
#define STM32F407xx

/*
 * IO pins assignments.
 */
#define GPIOA_GPS_PPS           0 /* tim2_ch1 OR tim5_ch1 */
#define GPIOA_NVRAM_PWR_EN      1 /* tim2_ch2 OR tim5_ch2 */
#define GPIOA_SONAR_PWM         2 /* tim2_ch3 OR tim5_ch3 OR tim9_ch1 */
#define GPIOA_AD_CLK            3 /* tim2_ch4 OR tim5_ch4 OR tim9_ch2 */
#define GPIOA_ADIS_NSS          4 /* spi1 */
#define GPIOA_ADIS_SCK          5 /* spi1 */
#define GPIOA_ADIS_MISO         6 /* spi1 */
#define GPIOA_ADIS_MOSI         7 /* spi1 */
#define GPIOA_ADIS_NRST         8
#define GPIOA_TO_GPS            9 /* usart1 */
#define GPIOA_FROM_GPS          10/* usart1 */
#define GPIOA_OTG_FS_DM         11
#define GPIOA_OTG_FS_DP         12
#define GPIOA_JTMS              13
#define GPIOA_JTCK              14
#define GPIOA_JTDI              15

#define GPIOB_LED_R             0
#define GPIOB_LED_G             1
#define GPIOB_BOOT1             2
#define GPIOB_JTDO              3
#define GPIOB_NJTRST            4
#define GPIOB_LED_B             5
#define GPIOB_I2C_SLOW_SCL      6 /* I2C1*/
#define GPIOB_I2C_SLOW_SDA      7 /* I2C1*/
#define GPIOB_TACHOMETER        8
#define GPIOB_RECEIVER_PPM      9
#define GPIOB_I2C_FAST_SCL      10 /* I2C2*/
#define GPIOB_I2C_FAST_SDA      11 /* I2C2*/
#define GPIOB_SPI2_NSS_UEXT     12
#define GPIOB_SPI2_SCK_UEXT     13
#define GPIOB_SPI2_MISO_UEXT    14
#define GPIOB_SPI2_MOSI_UEXT    15

#define GPIOC_PRESS_DIFF        0
#define GPIOC_MPXV_TEMP         1
#define GPIOC_CURRENT_SENS      2
#define GPIOC_MAIN_SUPPLY       3
#define GPIOC_6V_SUPPLY         4
#define GPIOC_AN_RESERVED       5
#define GPIOC_USART6_TX_UEXT    6
#define GPIOC_USART6_RX_UEXT    7
#define GPIOC_SDIO_D0           8
#define GPIOC_SDIO_D1           9
#define GPIOC_SDIO_D2           10
#define GPIOC_SDIO_D3           11
#define GPIOC_SDIO_CK           12
#define GPIOC_TAMPER_RTC        13
#define GPIOC_OSC32_IN          14
#define GPIOC_OSC32_OUT         15

#define GPIOD_AD_CS             0
#define GPIOD_AD_SDI            1
#define GPIOD_SDIO_CMD          2
#define GPIOD_USART2_CTS        3 /* reserved */
#define GPIOD_USART2_RTS        4 /* reserved */
#define GPIOD_USART2_TX         5 /* reserved */
#define GPIOD_USART2_RX         6 /* reserved */
#define GPIOD_XBEE_RESET        7
#define GPIOD_TO_XBEE           8 /* usart3 */
#define GPIOD_FROM_XBEE         9 /* usart3 */
#define GPIOD_5V_ENABLE         10
#define GPIOD_XBEE_CTS          11 /* usart3 */
#define GPIOD_TIM4_PWM1         12
#define GPIOD_TIM4_PWM2         13
#define GPIOD_TIM4_PWM3         14
#define GPIOD_TIM4_PWM4         15

#define GPIOE_GPS_ENABLE        0
#define GPIOE_ADIS_INT          1
#define GPIOE_USB_PRESENCE      2
#define GPIOE_MPU9150_INT       3
#define GPIOE_BMP085_EOC        4
#define GPIOE_MAG_INT           5
#define GPIOE_PIN6              6
#define GPIOE_PIN7              7
#define GPIOE_MPU6050_PWR       8
#define GPIOE_TIM1_PWM1         9
#define GPIOE_USB_DISCOVERY     10
#define GPIOE_TIM1_PWM2         11
#define GPIOE_SDIO_DETECT       12
#define GPIOE_TIM1_PWM3         13
#define GPIOE_TIM1_PWM4         14
#define GPIOE_SDIO_PWR_EN       15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 *
 * 1 for open drain outputs denotes hi-Z state
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
 * GPIOA setup
 */
#define VAL_GPIOA_MODER        (PIN_MODE_INPUT(GPIOA_GPS_PPS) |               \
                                PIN_MODE_OUTPUT(GPIOA_NVRAM_PWR_EN) |         \
                                PIN_MODE_ALTERNATE(GPIOA_SONAR_PWM) |         \
                                PIN_MODE_OUTPUT(GPIOA_AD_CLK) |               \
                                PIN_MODE_OUTPUT(GPIOA_ADIS_NSS) |             \
                                PIN_MODE_ALTERNATE(GPIOA_ADIS_SCK) |          \
                                PIN_MODE_ALTERNATE(GPIOA_ADIS_MISO) |         \
                                PIN_MODE_ALTERNATE(GPIOA_ADIS_MOSI) |         \
                                PIN_MODE_OUTPUT(GPIOA_ADIS_NRST) |            \
                                PIN_MODE_ALTERNATE(GPIOA_TO_GPS) |            \
                                PIN_MODE_ALTERNATE(GPIOA_FROM_GPS) |          \
                                PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |         \
                                PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |         \
                                PIN_MODE_ALTERNATE(GPIOA_JTMS) |              \
                                PIN_MODE_ALTERNATE(GPIOA_JTCK) |              \
                                PIN_MODE_ALTERNATE(GPIOA_JTDI))
#define VAL_GPIOA_OTYPER       (PIN_OTYPE_PUSHPULL(GPIOA_GPS_PPS) |           \
                                PIN_OTYPE_OPENDRAIN(GPIOA_NVRAM_PWR_EN) |     \
                                PIN_OTYPE_PUSHPULL(GPIOA_SONAR_PWM) |         \
                                PIN_OTYPE_PUSHPULL(GPIOA_AD_CLK) |            \
                                PIN_OTYPE_PUSHPULL(GPIOA_ADIS_NSS) |          \
                                PIN_OTYPE_PUSHPULL(GPIOA_ADIS_SCK) |          \
                                PIN_OTYPE_PUSHPULL(GPIOA_ADIS_MISO) |         \
                                PIN_OTYPE_PUSHPULL(GPIOA_ADIS_MOSI) |         \
                                PIN_OTYPE_PUSHPULL(GPIOA_ADIS_NRST) |         \
                                PIN_OTYPE_PUSHPULL(GPIOA_TO_GPS) |            \
                                PIN_OTYPE_PUSHPULL(GPIOA_FROM_GPS) |          \
                                PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |         \
                                PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |         \
                                PIN_OTYPE_PUSHPULL(GPIOA_JTMS) |              \
                                PIN_OTYPE_PUSHPULL(GPIOA_JTCK) |              \
                                PIN_OTYPE_PUSHPULL(GPIOA_JTDI))
#define VAL_GPIOA_OSPEEDR      (PIN_OSPEED_2M(GPIOA_GPS_PPS) |                \
                                PIN_OSPEED_2M(GPIOA_NVRAM_PWR_EN) |           \
                                PIN_OSPEED_2M(GPIOA_SONAR_PWM) |              \
                                PIN_OSPEED_100M(GPIOA_AD_CLK) |               \
                                PIN_OSPEED_2M(GPIOA_ADIS_NSS) |               \
                                PIN_OSPEED_25M(GPIOA_ADIS_SCK) |              \
                                PIN_OSPEED_25M(GPIOA_ADIS_MISO) |             \
                                PIN_OSPEED_25M(GPIOA_ADIS_MOSI) |             \
                                PIN_OSPEED_2M(GPIOA_ADIS_NRST) |              \
                                PIN_OSPEED_2M(GPIOA_TO_GPS) |                 \
                                PIN_OSPEED_2M(GPIOA_FROM_GPS) |               \
                                PIN_OSPEED_100M(GPIOA_OTG_FS_DM) |            \
                                PIN_OSPEED_100M(GPIOA_OTG_FS_DP) |            \
                                PIN_OSPEED_100M(GPIOA_JTMS) |                 \
                                PIN_OSPEED_100M(GPIOA_JTCK) |                 \
                                PIN_OSPEED_100M(GPIOA_JTDI))
#define VAL_GPIOA_PUPDR        (PIN_PUPDR_PULLDOWN(GPIOA_GPS_PPS) |           \
                                PIN_PUPDR_FLOATING(GPIOA_NVRAM_PWR_EN) |      \
                                PIN_PUPDR_PULLDOWN(GPIOA_SONAR_PWM) |         \
                                PIN_PUPDR_PULLDOWN(GPIOA_AD_CLK) |            \
                                PIN_PUPDR_PULLUP(GPIOA_ADIS_NSS) |            \
                                PIN_PUPDR_PULLUP(GPIOA_ADIS_SCK) |            \
                                PIN_PUPDR_PULLUP(GPIOA_ADIS_MISO) |           \
                                PIN_PUPDR_PULLUP(GPIOA_ADIS_MOSI) |           \
                                PIN_PUPDR_PULLDOWN(GPIOA_ADIS_NRST) |         \
                                PIN_PUPDR_FLOATING(GPIOA_TO_GPS) |            \
                                PIN_PUPDR_FLOATING(GPIOA_FROM_GPS) |          \
                                PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |         \
                                PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |         \
                                PIN_PUPDR_FLOATING(GPIOA_JTMS) |              \
                                PIN_PUPDR_FLOATING(GPIOA_JTCK) |              \
                                PIN_PUPDR_FLOATING(GPIOA_JTDI))
#define VAL_GPIOA_ODR          (PIN_ODR_HIGH(GPIOA_GPS_PPS) |                 \
                                PIN_ODR_LOW(GPIOA_NVRAM_PWR_EN) |             \
                                PIN_ODR_HIGH(GPIOA_SONAR_PWM) |               \
                                PIN_ODR_LOW(GPIOA_AD_CLK) |                   \
                                PIN_ODR_HIGH(GPIOA_ADIS_NSS) |                \
                                PIN_ODR_HIGH(GPIOA_ADIS_SCK) |                \
                                PIN_ODR_HIGH(GPIOA_ADIS_MISO) |               \
                                PIN_ODR_HIGH(GPIOA_ADIS_MOSI) |               \
                                PIN_ODR_LOW(GPIOA_ADIS_NRST) |                \
                                PIN_ODR_HIGH(GPIOA_TO_GPS) |                  \
                                PIN_ODR_HIGH(GPIOA_FROM_GPS) |                \
                                PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |               \
                                PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |               \
                                PIN_ODR_HIGH(GPIOA_JTMS) |                    \
                                PIN_ODR_HIGH(GPIOA_JTCK) |                    \
                                PIN_ODR_HIGH(GPIOA_JTDI))
#define VAL_GPIOA_AFRL         (PIN_AFIO_AF(GPIOA_GPS_PPS, 0) |               \
                                PIN_AFIO_AF(GPIOA_NVRAM_PWR_EN, 0) |          \
                                PIN_AFIO_AF(GPIOA_SONAR_PWM, 3) |             \
                                PIN_AFIO_AF(GPIOA_AD_CLK, 0) |                \
                                PIN_AFIO_AF(GPIOA_ADIS_NSS, 0) |              \
                                PIN_AFIO_AF(GPIOA_ADIS_SCK, 5) |              \
                                PIN_AFIO_AF(GPIOA_ADIS_MISO, 5) |             \
                                PIN_AFIO_AF(GPIOA_ADIS_MOSI, 5))
#define VAL_GPIOA_AFRH         (PIN_AFIO_AF(GPIOA_ADIS_NRST, 0) |             \
                                PIN_AFIO_AF(GPIOA_TO_GPS, 7) |                \
                                PIN_AFIO_AF(GPIOA_FROM_GPS, 7) |              \
                                PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |            \
                                PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |            \
                                PIN_AFIO_AF(GPIOA_JTMS, 0) |                  \
                                PIN_AFIO_AF(GPIOA_JTCK, 0) |                  \
                                PIN_AFIO_AF(GPIOA_JTDI, 0))
/*
 * GPOIB setup
 */
#define VAL_GPIOB_MODER        (PIN_MODE_OUTPUT(GPIOB_LED_R) |                \
                                PIN_MODE_OUTPUT(GPIOB_LED_G) |                \
                                PIN_MODE_INPUT(GPIOB_BOOT1) |                 \
                                PIN_MODE_ALTERNATE(GPIOB_JTDO) |              \
                                PIN_MODE_ALTERNATE(GPIOB_NJTRST) |            \
                                PIN_MODE_OUTPUT(GPIOB_LED_B) |                \
                                PIN_MODE_ALTERNATE(GPIOB_I2C_SLOW_SCL) |      \
                                PIN_MODE_ALTERNATE(GPIOB_I2C_SLOW_SDA) |      \
                                PIN_MODE_INPUT(GPIOB_TACHOMETER) |            \
                                PIN_MODE_INPUT(GPIOB_RECEIVER_PPM) |          \
                                PIN_MODE_ALTERNATE(GPIOB_I2C_FAST_SCL) |      \
                                PIN_MODE_ALTERNATE(GPIOB_I2C_FAST_SDA) |      \
                                PIN_MODE_ALTERNATE(GPIOB_SPI2_NSS_UEXT) |     \
                                PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK_UEXT) |     \
                                PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO_UEXT) |    \
                                PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI_UEXT))
#define VAL_GPIOB_OTYPER       (PIN_OTYPE_OPENDRAIN(GPIOB_LED_R) |            \
                                PIN_OTYPE_OPENDRAIN(GPIOB_LED_G) |            \
                                PIN_OTYPE_PUSHPULL(GPIOB_BOOT1) |             \
                                PIN_OTYPE_PUSHPULL(GPIOB_JTDO) |              \
                                PIN_OTYPE_PUSHPULL(GPIOB_NJTRST) |            \
                                PIN_OTYPE_OPENDRAIN(GPIOB_LED_B) |            \
                                PIN_OTYPE_OPENDRAIN(GPIOB_I2C_SLOW_SCL) |     \
                                PIN_OTYPE_OPENDRAIN(GPIOB_I2C_SLOW_SDA) |     \
                                PIN_OTYPE_PUSHPULL(GPIOB_TACHOMETER) |        \
                                PIN_OTYPE_PUSHPULL(GPIOB_RECEIVER_PPM) |      \
                                PIN_OTYPE_OPENDRAIN(GPIOB_I2C_FAST_SCL) |     \
                                PIN_OTYPE_OPENDRAIN(GPIOB_I2C_FAST_SDA) |     \
                                PIN_OTYPE_PUSHPULL(GPIOB_SPI2_NSS_UEXT) |     \
                                PIN_OTYPE_PUSHPULL(GPIOB_SPI2_SCK_UEXT) |     \
                                PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MISO_UEXT) |    \
                                PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MOSI_UEXT))
#define VAL_GPIOB_OSPEEDR      (PIN_OSPEED_2M(GPIOB_LED_R) |                  \
                                PIN_OSPEED_2M(GPIOB_LED_G) |                  \
                                PIN_OSPEED_2M(GPIOB_BOOT1) |                  \
                                PIN_OSPEED_100M(GPIOB_JTDO) |                 \
                                PIN_OSPEED_100M(GPIOB_NJTRST) |               \
                                PIN_OSPEED_2M(GPIOB_LED_B) |                  \
                                PIN_OSPEED_2M(GPIOB_I2C_SLOW_SCL) |           \
                                PIN_OSPEED_2M(GPIOB_I2C_SLOW_SDA) |           \
                                PIN_OSPEED_2M(GPIOB_TACHOMETER) |             \
                                PIN_OSPEED_2M(GPIOB_RECEIVER_PPM) |           \
                                PIN_OSPEED_2M(GPIOB_I2C_FAST_SCL) |           \
                                PIN_OSPEED_2M(GPIOB_I2C_FAST_SDA) |           \
                                PIN_OSPEED_25M(GPIOB_SPI2_NSS_UEXT) |         \
                                PIN_OSPEED_25M(GPIOB_SPI2_SCK_UEXT) |         \
                                PIN_OSPEED_25M(GPIOB_SPI2_MISO_UEXT) |        \
                                PIN_OSPEED_25M(GPIOB_SPI2_MOSI_UEXT))
#define VAL_GPIOB_PUPDR        (PIN_PUPDR_FLOATING(GPIOB_LED_R) |             \
                                PIN_PUPDR_FLOATING(GPIOB_LED_G) |             \
                                PIN_PUPDR_FLOATING(GPIOB_BOOT1) |             \
                                PIN_PUPDR_FLOATING(GPIOB_JTDO) |              \
                                PIN_PUPDR_FLOATING(GPIOB_NJTRST) |            \
                                PIN_PUPDR_FLOATING(GPIOB_LED_B) |             \
                                PIN_PUPDR_FLOATING(GPIOB_I2C_SLOW_SCL) |      \
                                PIN_PUPDR_FLOATING(GPIOB_I2C_SLOW_SDA) |      \
                                PIN_PUPDR_PULLUP(GPIOB_TACHOMETER) |          \
                                PIN_PUPDR_PULLDOWN(GPIOB_RECEIVER_PPM) |      \
                                PIN_PUPDR_FLOATING(GPIOB_I2C_FAST_SCL) |      \
                                PIN_PUPDR_FLOATING(GPIOB_I2C_FAST_SDA) |      \
                                PIN_PUPDR_PULLUP(GPIOB_SPI2_NSS_UEXT) |       \
                                PIN_PUPDR_PULLUP(GPIOB_SPI2_SCK_UEXT) |       \
                                PIN_PUPDR_PULLUP(GPIOB_SPI2_MISO_UEXT) |      \
                                PIN_PUPDR_PULLUP(GPIOB_SPI2_MOSI_UEXT))
#define VAL_GPIOB_ODR          (PIN_ODR_HIGH(GPIOB_LED_R) |                   \
                                PIN_ODR_HIGH(GPIOB_LED_G) |                   \
                                PIN_ODR_HIGH(GPIOB_BOOT1) |                   \
                                PIN_ODR_HIGH(GPIOB_JTDO) |                    \
                                PIN_ODR_HIGH(GPIOB_NJTRST) |                  \
                                PIN_ODR_HIGH(GPIOB_LED_B) |                   \
                                PIN_ODR_HIGH(GPIOB_I2C_SLOW_SCL) |            \
                                PIN_ODR_HIGH(GPIOB_I2C_SLOW_SDA) |            \
                                PIN_ODR_HIGH(GPIOB_TACHOMETER) |              \
                                PIN_ODR_HIGH(GPIOB_RECEIVER_PPM) |            \
                                PIN_ODR_HIGH(GPIOB_I2C_FAST_SCL) |            \
                                PIN_ODR_HIGH(GPIOB_I2C_FAST_SDA) |            \
                                PIN_ODR_HIGH(GPIOB_SPI2_NSS_UEXT) |           \
                                PIN_ODR_HIGH(GPIOB_SPI2_SCK_UEXT) |           \
                                PIN_ODR_HIGH(GPIOB_SPI2_MISO_UEXT) |          \
                                PIN_ODR_HIGH(GPIOB_SPI2_MOSI_UEXT))
#define VAL_GPIOB_AFRL         (PIN_AFIO_AF(GPIOB_LED_R, 0) |                 \
                                PIN_AFIO_AF(GPIOB_LED_G, 0) |                 \
                                PIN_AFIO_AF(GPIOB_BOOT1, 0) |                 \
                                PIN_AFIO_AF(GPIOB_JTDO, 0) |                  \
                                PIN_AFIO_AF(GPIOB_NJTRST, 0) |                \
                                PIN_AFIO_AF(GPIOB_LED_B, 0) |                 \
                                PIN_AFIO_AF(GPIOB_I2C_SLOW_SCL, 4) |          \
                                PIN_AFIO_AF(GPIOB_I2C_SLOW_SDA, 4))
#define VAL_GPIOB_AFRH         (PIN_AFIO_AF(GPIOB_TACHOMETER, 2) |            \
                                PIN_AFIO_AF(GPIOB_RECEIVER_PPM, 0) |          \
                                PIN_AFIO_AF(GPIOB_I2C_FAST_SCL, 4) |          \
                                PIN_AFIO_AF(GPIOB_I2C_FAST_SDA, 4) |          \
                                PIN_AFIO_AF(GPIOB_SPI2_NSS_UEXT, 5) |         \
                                PIN_AFIO_AF(GPIOB_SPI2_SCK_UEXT, 5) |         \
                                PIN_AFIO_AF(GPIOB_SPI2_MISO_UEXT, 5) |        \
                                PIN_AFIO_AF(GPIOB_SPI2_MOSI_UEXT, 5))

/*
 * Port C setup.
 */
#define VAL_GPIOC_MODER        (PIN_MODE_ANALOG(GPIOC_PRESS_DIFF) |           \
                                PIN_MODE_ANALOG(GPIOC_MPXV_TEMP) |            \
                                PIN_MODE_ANALOG(GPIOC_CURRENT_SENS) |         \
                                PIN_MODE_ANALOG(GPIOC_MAIN_SUPPLY) |          \
                                PIN_MODE_ANALOG(GPIOC_6V_SUPPLY) |            \
                                PIN_MODE_ANALOG(GPIOC_AN_RESERVED) |          \
                                PIN_MODE_ALTERNATE(GPIOC_USART6_TX_UEXT) |    \
                                PIN_MODE_ALTERNATE(GPIOC_USART6_RX_UEXT) |    \
                                PIN_MODE_ALTERNATE(GPIOC_SDIO_D0) |           \
                                PIN_MODE_ALTERNATE(GPIOC_SDIO_D1) |           \
                                PIN_MODE_ALTERNATE(GPIOC_SDIO_D2) |           \
                                PIN_MODE_ALTERNATE(GPIOC_SDIO_D3) |           \
                                PIN_MODE_ALTERNATE(GPIOC_SDIO_CK) |           \
                                PIN_MODE_INPUT(GPIOC_TAMPER_RTC) |            \
                                PIN_MODE_INPUT(GPIOC_OSC32_IN) |              \
                                PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER       (PIN_OTYPE_PUSHPULL(GPIOC_PRESS_DIFF) |        \
                                PIN_OTYPE_PUSHPULL(GPIOC_MPXV_TEMP) |         \
                                PIN_OTYPE_PUSHPULL(GPIOC_CURRENT_SENS) |      \
                                PIN_OTYPE_PUSHPULL(GPIOC_MAIN_SUPPLY) |       \
                                PIN_OTYPE_PUSHPULL(GPIOC_6V_SUPPLY) |         \
                                PIN_OTYPE_PUSHPULL(GPIOC_AN_RESERVED) |       \
                                PIN_OTYPE_PUSHPULL(GPIOC_USART6_TX_UEXT) |    \
                                PIN_OTYPE_PUSHPULL(GPIOC_USART6_RX_UEXT) |    \
                                PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0) |           \
                                PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1) |           \
                                PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2) |           \
                                PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3) |           \
                                PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CK) |           \
                                PIN_OTYPE_PUSHPULL(GPIOC_TAMPER_RTC) |        \
                                PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |          \
                                PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR      (PIN_OSPEED_2M(GPIOC_PRESS_DIFF) |             \
                                PIN_OSPEED_2M(GPIOC_MPXV_TEMP) |              \
                                PIN_OSPEED_2M(GPIOC_CURRENT_SENS) |           \
                                PIN_OSPEED_2M(GPIOC_MAIN_SUPPLY) |            \
                                PIN_OSPEED_2M(GPIOC_6V_SUPPLY) |              \
                                PIN_OSPEED_2M(GPIOC_AN_RESERVED) |            \
                                PIN_OSPEED_2M(GPIOC_USART6_TX_UEXT) |         \
                                PIN_OSPEED_2M(GPIOC_USART6_RX_UEXT) |         \
                                PIN_OSPEED_100M(GPIOC_SDIO_D0) |              \
                                PIN_OSPEED_100M(GPIOC_SDIO_D1) |              \
                                PIN_OSPEED_100M(GPIOC_SDIO_D2) |              \
                                PIN_OSPEED_100M(GPIOC_SDIO_D3) |              \
                                PIN_OSPEED_100M(GPIOC_SDIO_CK) |              \
                                PIN_OSPEED_2M(GPIOC_TAMPER_RTC) |             \
                                PIN_OSPEED_2M(GPIOC_OSC32_IN) |               \
                                PIN_OSPEED_2M(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR        (PIN_PUPDR_FLOATING(GPIOC_PRESS_DIFF) |        \
                                PIN_PUPDR_FLOATING(GPIOC_MPXV_TEMP) |         \
                                PIN_PUPDR_FLOATING(GPIOC_CURRENT_SENS) |      \
                                PIN_PUPDR_FLOATING(GPIOC_MAIN_SUPPLY) |       \
                                PIN_PUPDR_FLOATING(GPIOC_6V_SUPPLY) |         \
                                PIN_PUPDR_FLOATING(GPIOC_AN_RESERVED) |       \
                                PIN_PUPDR_PULLUP(GPIOC_USART6_TX_UEXT) |      \
                                PIN_PUPDR_PULLUP(GPIOC_USART6_RX_UEXT) |      \
                                PIN_PUPDR_PULLUP(GPIOC_SDIO_D0) |             \
                                PIN_PUPDR_PULLUP(GPIOC_SDIO_D1) |             \
                                PIN_PUPDR_PULLUP(GPIOC_SDIO_D2) |             \
                                PIN_PUPDR_PULLUP(GPIOC_SDIO_D3) |             \
                                PIN_PUPDR_PULLDOWN(GPIOC_SDIO_CK) |           \
                                PIN_PUPDR_FLOATING(GPIOC_TAMPER_RTC) |        \
                                PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |          \
                                PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR          (PIN_ODR_HIGH(GPIOC_PRESS_DIFF) |              \
                                PIN_ODR_HIGH(GPIOC_MPXV_TEMP) |               \
                                PIN_ODR_HIGH(GPIOC_CURRENT_SENS) |            \
                                PIN_ODR_HIGH(GPIOC_MAIN_SUPPLY) |             \
                                PIN_ODR_HIGH(GPIOC_6V_SUPPLY) |               \
                                PIN_ODR_HIGH(GPIOC_AN_RESERVED) |             \
                                PIN_ODR_HIGH(GPIOC_USART6_TX_UEXT) |          \
                                PIN_ODR_HIGH(GPIOC_USART6_RX_UEXT) |          \
                                PIN_ODR_HIGH(GPIOC_SDIO_D0) |                 \
                                PIN_ODR_HIGH(GPIOC_SDIO_D1) |                 \
                                PIN_ODR_HIGH(GPIOC_SDIO_D2) |                 \
                                PIN_ODR_HIGH(GPIOC_SDIO_D3) |                 \
                                PIN_ODR_HIGH(GPIOC_SDIO_CK) |                 \
                                PIN_ODR_HIGH(GPIOC_TAMPER_RTC) |              \
                                PIN_ODR_HIGH(GPIOC_OSC32_IN) |                \
                                PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL         (PIN_AFIO_AF(GPIOC_PRESS_DIFF, 0) |            \
                                PIN_AFIO_AF(GPIOC_MPXV_TEMP, 0) |             \
                                PIN_AFIO_AF(GPIOC_CURRENT_SENS, 0) |          \
                                PIN_AFIO_AF(GPIOC_MAIN_SUPPLY, 0) |           \
                                PIN_AFIO_AF(GPIOC_6V_SUPPLY, 0) |             \
                                PIN_AFIO_AF(GPIOC_AN_RESERVED, 0) |           \
                                PIN_AFIO_AF(GPIOC_USART6_TX_UEXT, 8) |        \
                                PIN_AFIO_AF(GPIOC_USART6_RX_UEXT, 8))
#define VAL_GPIOC_AFRH         (PIN_AFIO_AF(GPIOC_SDIO_D0, 12) |              \
                                PIN_AFIO_AF(GPIOC_SDIO_D1, 12) |              \
                                PIN_AFIO_AF(GPIOC_SDIO_D2, 12) |              \
                                PIN_AFIO_AF(GPIOC_SDIO_D3, 12) |              \
                                PIN_AFIO_AF(GPIOC_SDIO_CK, 12) |              \
                                PIN_AFIO_AF(GPIOC_TAMPER_RTC, 0) |            \
                                PIN_AFIO_AF(GPIOC_OSC32_IN, 0) |              \
                                PIN_AFIO_AF(GPIOC_OSC32_OUT, 9))

/*
 * Port D setup.
 */
#define VAL_GPIOD_MODER        (PIN_MODE_OUTPUT(GPIOD_AD_CS) |                \
                                PIN_MODE_OUTPUT(GPIOD_AD_SDI) |               \
                                PIN_MODE_ALTERNATE(GPIOD_SDIO_CMD) |          \
                                PIN_MODE_ALTERNATE(GPIOD_USART2_CTS) |        \
                                PIN_MODE_ALTERNATE(GPIOD_USART2_RTS) |        \
                                PIN_MODE_ALTERNATE(GPIOD_USART2_TX) |         \
                                PIN_MODE_ALTERNATE(GPIOD_USART2_RX) |         \
                                PIN_MODE_OUTPUT(GPIOD_XBEE_RESET) |           \
                                PIN_MODE_ALTERNATE(GPIOD_TO_XBEE) |           \
                                PIN_MODE_ALTERNATE(GPIOD_FROM_XBEE) |         \
                                PIN_MODE_OUTPUT(GPIOD_5V_ENABLE) |            \
                                PIN_MODE_ALTERNATE(GPIOD_XBEE_CTS) |          \
                                PIN_MODE_ALTERNATE(GPIOD_TIM4_PWM1) |         \
                                PIN_MODE_ALTERNATE(GPIOD_TIM4_PWM2) |         \
                                PIN_MODE_ALTERNATE(GPIOD_TIM4_PWM3) |         \
                                PIN_MODE_ALTERNATE(GPIOD_TIM4_PWM4))
#define VAL_GPIOD_OTYPER       (PIN_OTYPE_PUSHPULL(GPIOD_AD_CS) |             \
                                PIN_OTYPE_PUSHPULL(GPIOD_AD_SDI) |            \
                                PIN_OTYPE_PUSHPULL(GPIOD_SDIO_CMD) |          \
                                PIN_OTYPE_PUSHPULL(GPIOD_USART2_CTS) |        \
                                PIN_OTYPE_PUSHPULL(GPIOD_USART2_RTS) |        \
                                PIN_OTYPE_PUSHPULL(GPIOD_USART2_TX) |         \
                                PIN_OTYPE_PUSHPULL(GPIOD_USART2_RX) |         \
                                PIN_OTYPE_PUSHPULL(GPIOD_XBEE_RESET) |        \
                                PIN_OTYPE_PUSHPULL(GPIOD_TO_XBEE) |           \
                                PIN_OTYPE_PUSHPULL(GPIOD_FROM_XBEE) |         \
                                PIN_OTYPE_PUSHPULL(GPIOD_5V_ENABLE) |         \
                                PIN_OTYPE_PUSHPULL(GPIOD_XBEE_CTS) |          \
                                PIN_OTYPE_PUSHPULL(GPIOD_TIM4_PWM1) |         \
                                PIN_OTYPE_PUSHPULL(GPIOD_TIM4_PWM2) |         \
                                PIN_OTYPE_PUSHPULL(GPIOD_TIM4_PWM3) |         \
                                PIN_OTYPE_PUSHPULL(GPIOD_TIM4_PWM4))
#define VAL_GPIOD_OSPEEDR      (PIN_OSPEED_100M(GPIOD_AD_CS) |                \
                                PIN_OSPEED_100M(GPIOD_AD_SDI) |               \
                                PIN_OSPEED_100M(GPIOD_SDIO_CMD) |             \
                                PIN_OSPEED_2M(GPIOD_USART2_CTS) |             \
                                PIN_OSPEED_2M(GPIOD_USART2_RTS) |             \
                                PIN_OSPEED_2M(GPIOD_USART2_TX) |              \
                                PIN_OSPEED_2M(GPIOD_USART2_RX) |              \
                                PIN_OSPEED_2M(GPIOD_XBEE_RESET) |             \
                                PIN_OSPEED_2M(GPIOD_TO_XBEE) |                \
                                PIN_OSPEED_2M(GPIOD_FROM_XBEE) |              \
                                PIN_OSPEED_2M(GPIOD_5V_ENABLE) |              \
                                PIN_OSPEED_2M(GPIOD_XBEE_CTS) |               \
                                PIN_OSPEED_2M(GPIOD_TIM4_PWM1) |              \
                                PIN_OSPEED_2M(GPIOD_TIM4_PWM2) |              \
                                PIN_OSPEED_2M(GPIOD_TIM4_PWM3) |              \
                                PIN_OSPEED_2M(GPIOD_TIM4_PWM4))
#define VAL_GPIOD_PUPDR        (PIN_PUPDR_PULLUP(GPIOD_AD_CS) |               \
                                PIN_PUPDR_PULLUP(GPIOD_AD_SDI) |              \
                                PIN_PUPDR_PULLUP(GPIOD_SDIO_CMD) |            \
                                PIN_PUPDR_PULLUP(GPIOD_USART2_CTS) |          \
                                PIN_PUPDR_PULLUP(GPIOD_USART2_RTS) |          \
                                PIN_PUPDR_PULLUP(GPIOD_USART2_TX) |           \
                                PIN_PUPDR_PULLUP(GPIOD_USART2_RX) |           \
                                PIN_PUPDR_FLOATING(GPIOD_XBEE_RESET) |        \
                                PIN_PUPDR_PULLUP(GPIOD_TO_XBEE) |             \
                                PIN_PUPDR_PULLUP(GPIOD_FROM_XBEE) |           \
                                PIN_PUPDR_PULLDOWN(GPIOD_5V_ENABLE) |         \
                                PIN_PUPDR_PULLUP(GPIOD_XBEE_CTS) |            \
                                PIN_PUPDR_PULLDOWN(GPIOD_TIM4_PWM1) |         \
                                PIN_PUPDR_PULLDOWN(GPIOD_TIM4_PWM2) |         \
                                PIN_PUPDR_PULLDOWN(GPIOD_TIM4_PWM3) |         \
                                PIN_PUPDR_PULLDOWN(GPIOD_TIM4_PWM4))
#define VAL_GPIOD_ODR          (PIN_ODR_HIGH(GPIOD_AD_CS) |                   \
                                PIN_ODR_HIGH(GPIOD_AD_SDI) |                  \
                                PIN_ODR_HIGH(GPIOD_SDIO_CMD) |                \
                                PIN_ODR_HIGH(GPIOD_USART2_CTS) |              \
                                PIN_ODR_HIGH(GPIOD_USART2_RTS) |              \
                                PIN_ODR_HIGH(GPIOD_USART2_TX) |               \
                                PIN_ODR_HIGH(GPIOD_USART2_RX) |               \
                                PIN_ODR_LOW(GPIOD_XBEE_RESET) |               \
                                PIN_ODR_HIGH(GPIOD_TO_XBEE) |                 \
                                PIN_ODR_HIGH(GPIOD_FROM_XBEE) |               \
                                PIN_ODR_LOW(GPIOD_5V_ENABLE) |                \
                                PIN_ODR_HIGH(GPIOD_XBEE_CTS) |                \
                                PIN_ODR_LOW(GPIOD_TIM4_PWM1) |                \
                                PIN_ODR_LOW(GPIOD_TIM4_PWM2) |                \
                                PIN_ODR_LOW(GPIOD_TIM4_PWM3) |                \
                                PIN_ODR_LOW(GPIOD_TIM4_PWM4))
#define VAL_GPIOD_AFRL         (PIN_AFIO_AF(GPIOD_AD_CS, 0) |                 \
                                PIN_AFIO_AF(GPIOD_AD_SDI, 0) |                \
                                PIN_AFIO_AF(GPIOD_SDIO_CMD, 12) |             \
                                PIN_AFIO_AF(GPIOD_USART2_CTS, 7) |            \
                                PIN_AFIO_AF(GPIOD_USART2_RTS, 7) |            \
                                PIN_AFIO_AF(GPIOD_USART2_TX, 7) |             \
                                PIN_AFIO_AF(GPIOD_USART2_RX, 7) |             \
                                PIN_AFIO_AF(GPIOD_XBEE_RESET, 0))
#define VAL_GPIOD_AFRH         (PIN_AFIO_AF(GPIOD_TO_XBEE, 7) |               \
                                PIN_AFIO_AF(GPIOD_FROM_XBEE, 7) |             \
                                PIN_AFIO_AF(GPIOD_5V_ENABLE, 0) |             \
                                PIN_AFIO_AF(GPIOD_XBEE_CTS, 7) |              \
                                PIN_AFIO_AF(GPIOD_TIM4_PWM1, 2) |             \
                                PIN_AFIO_AF(GPIOD_TIM4_PWM2, 2) |             \
                                PIN_AFIO_AF(GPIOD_TIM4_PWM3, 2) |             \
                                PIN_AFIO_AF(GPIOD_TIM4_PWM4, 2))

/*
 * Port E setup.
 */
#define VAL_GPIOE_MODER        (PIN_MODE_OUTPUT(GPIOE_GPS_ENABLE) |           \
                                PIN_MODE_INPUT(GPIOE_ADIS_INT) |              \
                                PIN_MODE_INPUT(GPIOE_USB_PRESENCE) |          \
                                PIN_MODE_INPUT(GPIOE_MPU9150_INT) |           \
                                PIN_MODE_INPUT(GPIOE_BMP085_EOC) |            \
                                PIN_MODE_INPUT(GPIOE_MAG_INT) |               \
                                PIN_MODE_INPUT(GPIOE_PIN6) |                  \
                                PIN_MODE_INPUT(GPIOE_PIN7) |                  \
                                PIN_MODE_OUTPUT(GPIOE_MPU6050_PWR) |          \
                                PIN_MODE_ALTERNATE(GPIOE_TIM1_PWM1) |         \
                                PIN_MODE_INPUT(GPIOE_USB_DISCOVERY) |         \
                                PIN_MODE_ALTERNATE(GPIOE_TIM1_PWM2) |         \
                                PIN_MODE_INPUT(GPIOE_SDIO_DETECT) |           \
                                PIN_MODE_ALTERNATE(GPIOE_TIM1_PWM3) |         \
                                PIN_MODE_ALTERNATE(GPIOE_TIM1_PWM4) |         \
                                PIN_MODE_OUTPUT(GPIOE_SDIO_PWR_EN))
#define VAL_GPIOE_OTYPER       (PIN_OTYPE_PUSHPULL(GPIOE_GPS_ENABLE) |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_ADIS_INT) |          \
                                PIN_OTYPE_PUSHPULL(GPIOE_USB_PRESENCE) |      \
                                PIN_OTYPE_PUSHPULL(GPIOE_MPU9150_INT) |       \
                                PIN_OTYPE_PUSHPULL(GPIOE_BMP085_EOC) |        \
                                PIN_OTYPE_PUSHPULL(GPIOE_MAG_INT) |           \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |              \
                                PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |              \
                                PIN_OTYPE_PUSHPULL(GPIOE_MPU6050_PWR) |       \
                                PIN_OTYPE_PUSHPULL(GPIOE_TIM1_PWM1) |         \
                                PIN_OTYPE_PUSHPULL(GPIOE_USB_DISCOVERY) |     \
                                PIN_OTYPE_PUSHPULL(GPIOE_TIM1_PWM2) |         \
                                PIN_OTYPE_PUSHPULL(GPIOE_SDIO_DETECT) |       \
                                PIN_OTYPE_PUSHPULL(GPIOE_TIM1_PWM3) |         \
                                PIN_OTYPE_PUSHPULL(GPIOE_TIM1_PWM4) |         \
                                PIN_OTYPE_OPENDRAIN(GPIOE_SDIO_PWR_EN))
#define VAL_GPIOE_OSPEEDR      (PIN_OSPEED_2M(GPIOE_GPS_ENABLE) |             \
                                PIN_OSPEED_2M(GPIOE_ADIS_INT) |               \
                                PIN_OSPEED_2M(GPIOE_USB_PRESENCE) |           \
                                PIN_OSPEED_2M(GPIOE_MPU9150_INT) |            \
                                PIN_OSPEED_2M(GPIOE_BMP085_EOC) |             \
                                PIN_OSPEED_2M(GPIOE_MAG_INT) |                \
                                PIN_OSPEED_2M(GPIOE_PIN6) |                   \
                                PIN_OSPEED_2M(GPIOE_PIN7) |                   \
                                PIN_OSPEED_2M(GPIOE_MPU6050_PWR) |            \
                                PIN_OSPEED_2M(GPIOE_TIM1_PWM1) |              \
                                PIN_OSPEED_2M(GPIOE_USB_DISCOVERY) |          \
                                PIN_OSPEED_2M(GPIOE_TIM1_PWM2) |              \
                                PIN_OSPEED_2M(GPIOE_SDIO_DETECT) |            \
                                PIN_OSPEED_2M(GPIOE_TIM1_PWM3) |              \
                                PIN_OSPEED_2M(GPIOE_TIM1_PWM4) |              \
                                PIN_OSPEED_2M(GPIOE_SDIO_PWR_EN))
#define VAL_GPIOE_PUPDR        (PIN_PUPDR_PULLDOWN(GPIOE_GPS_ENABLE) |        \
                                PIN_PUPDR_PULLDOWN(GPIOE_ADIS_INT) |          \
                                PIN_PUPDR_PULLDOWN(GPIOE_USB_PRESENCE) |      \
                                PIN_PUPDR_PULLUP(GPIOE_MPU9150_INT) |         \
                                PIN_PUPDR_PULLDOWN(GPIOE_BMP085_EOC) |        \
                                PIN_PUPDR_PULLUP(GPIOE_MAG_INT) |             \
                                PIN_PUPDR_FLOATING(GPIOE_PIN6) |              \
                                PIN_PUPDR_FLOATING(GPIOE_PIN7) |              \
                                PIN_PUPDR_FLOATING(GPIOE_MPU6050_PWR) |       \
                                PIN_PUPDR_PULLDOWN(GPIOE_TIM1_PWM1) |         \
                                PIN_PUPDR_FLOATING(GPIOE_USB_DISCOVERY) |     \
                                PIN_PUPDR_PULLDOWN(GPIOE_TIM1_PWM2) |         \
                                PIN_PUPDR_FLOATING(GPIOE_SDIO_DETECT) |       \
                                PIN_PUPDR_PULLDOWN(GPIOE_TIM1_PWM3) |         \
                                PIN_PUPDR_PULLDOWN(GPIOE_TIM1_PWM4) |         \
                                PIN_PUPDR_FLOATING(GPIOE_SDIO_PWR_EN))
#define VAL_GPIOE_ODR          (PIN_ODR_HIGH(GPIOE_GPS_ENABLE) |              \
                                PIN_ODR_HIGH(GPIOE_ADIS_INT) |                \
                                PIN_ODR_HIGH(GPIOE_USB_PRESENCE) |            \
                                PIN_ODR_HIGH(GPIOE_MPU9150_INT) |             \
                                PIN_ODR_HIGH(GPIOE_BMP085_EOC) |              \
                                PIN_ODR_HIGH(GPIOE_MAG_INT) |                 \
                                PIN_ODR_HIGH(GPIOE_PIN6) |                    \
                                PIN_ODR_HIGH(GPIOE_PIN7) |                    \
                                PIN_ODR_LOW(GPIOE_MPU6050_PWR) |              \
                                PIN_ODR_LOW(GPIOE_TIM1_PWM1) |                \
                                PIN_ODR_HIGH(GPIOE_USB_DISCOVERY) |           \
                                PIN_ODR_LOW(GPIOE_TIM1_PWM2) |                \
                                PIN_ODR_HIGH(GPIOE_SDIO_DETECT) |             \
                                PIN_ODR_LOW(GPIOE_TIM1_PWM3) |                \
                                PIN_ODR_LOW(GPIOE_TIM1_PWM4) |                \
                                PIN_ODR_HIGH(GPIOE_SDIO_PWR_EN))
#define VAL_GPIOE_AFRL         (PIN_AFIO_AF(GPIOE_GPS_ENABLE, 0) |            \
                                PIN_AFIO_AF(GPIOE_ADIS_INT, 0) |              \
                                PIN_AFIO_AF(GPIOE_USB_PRESENCE, 0) |          \
                                PIN_AFIO_AF(GPIOE_MPU9150_INT, 0) |           \
                                PIN_AFIO_AF(GPIOE_BMP085_EOC, 0) |            \
                                PIN_AFIO_AF(GPIOE_MAG_INT, 0) |               \
                                PIN_AFIO_AF(GPIOE_PIN6, 0) |                  \
                                PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH         (PIN_AFIO_AF(GPIOE_MPU6050_PWR, 0) |           \
                                PIN_AFIO_AF(GPIOE_TIM1_PWM1, 1) |             \
                                PIN_AFIO_AF(GPIOE_USB_DISCOVERY, 0) |         \
                                PIN_AFIO_AF(GPIOE_TIM1_PWM2, 1) |             \
                                PIN_AFIO_AF(GPIOE_SDIO_DETECT, 0) |           \
                                PIN_AFIO_AF(GPIOE_TIM1_PWM3, 1) |             \
                                PIN_AFIO_AF(GPIOE_TIM1_PWM4, 1) |             \
                                PIN_AFIO_AF(GPIOE_SDIO_PWR_EN, 0))

/*
 * Port F setup.
 */
#define VAL_GPIOF_MODER             0x00000000
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0x00000000
#define VAL_GPIOF_PUPDR             0x00000000
#define VAL_GPIOF_ODR               0x00000000
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              0x00000000

/*
 * Port G setup.
 */
#define VAL_GPIOG_MODER             0x00000000
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0x00000000
#define VAL_GPIOG_PUPDR             0x00000000
#define VAL_GPIOG_ODR               0x00000000
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              0x00000000

/*
 * Port H setup.
 */
#define VAL_GPIOH_MODER             0x00000000
#define VAL_GPIOH_OTYPER            0x00000000
#define VAL_GPIOH_OSPEEDR           0x00000000
#define VAL_GPIOH_PUPDR             0x00000000
#define VAL_GPIOH_ODR               0x00000000
#define VAL_GPIOH_AFRL              0x00000000
#define VAL_GPIOH_AFRH              0x00000000

/*
 * Port I setup.
 */
#define VAL_GPIOI_MODER             0x00000000
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0x00000000
#define VAL_GPIOI_PUPDR             0x00000000
#define VAL_GPIOI_ODR               0x00000000
#define VAL_GPIOI_AFRL              0x00000000
#define VAL_GPIOI_AFRH              0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  unsigned int usb_lld_plug_state(void);
  void usb_lld_connect_bus_workaround(void);
  void usb_lld_disconnect_bus_workaround(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
