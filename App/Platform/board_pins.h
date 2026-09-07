#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "main.h"

/* IMU CS: PA4 */
#define BOARD_IMU_CS_PORT          GPIOA
#define BOARD_IMU_CS_PIN           GPIO_PIN_4

/* Barometer CS: PB12 */
#define BOARD_BARO_CS_PORT         GPIOB
#define BOARD_BARO_CS_PIN          GPIO_PIN_12

/* nRF24L01+ on SPI3 */
#define BOARD_NRF24_CSN_PORT       GPIOD
#define BOARD_NRF24_CSN_PIN        GPIO_PIN_0
#define BOARD_NRF24_CE_PORT        GPIOD
#define BOARD_NRF24_CE_PIN         GPIO_PIN_1
#define BOARD_NRF24_IRQ_PORT       GPIOD
#define BOARD_NRF24_IRQ_PIN        GPIO_PIN_3

/* STM32F4-Discovery safety/status LEDs.
 * PD12 green  : physical vent pulse active
 * PD13 orange : valid nRF remote link
 * PD14 red    : emergency STOP latched */
#define BOARD_STATUS_LED_PORT      GPIOD
#define BOARD_STATUS_LED_PIN       GPIO_PIN_12
#define BOARD_LINK_LED_PORT        LINK_LED_GPIO_Port
#define BOARD_LINK_LED_PIN         LINK_LED_Pin
#define BOARD_ESTOP_LED_PORT       ESTOP_LED_GPIO_Port
#define BOARD_ESTOP_LED_PIN        ESTOP_LED_Pin

/* Compatibility aliases for older application code. */
#define BOARD_NRF_CSN_PORT         BOARD_NRF24_CSN_PORT
#define BOARD_NRF_CSN_PIN          BOARD_NRF24_CSN_PIN
#define BOARD_NRF_CE_PORT          BOARD_NRF24_CE_PORT
#define BOARD_NRF_CE_PIN           BOARD_NRF24_CE_PIN

#endif
