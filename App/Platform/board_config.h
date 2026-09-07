#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"

/* -------------------------------------------------------------------------- */
/* System Clock                                                               */
/* -------------------------------------------------------------------------- */

#define BOARD_USE_HSE_PLL_168MHZ                1

/* -------------------------------------------------------------------------- */
/* SPI1 - IMU                                                                 */
/* Hardware SPI, Mode 3, slow safe start                                      */
/* -------------------------------------------------------------------------- */

#define BOARD_SPI1_PRESCALER                    SPI_BAUDRATEPRESCALER_256
#define BOARD_SPI1_CLK_POLARITY                 SPI_POLARITY_HIGH
#define BOARD_SPI1_CLK_PHASE                    SPI_PHASE_2EDGE

/* -------------------------------------------------------------------------- */
/* SPI2 - MS5611 Barometer                                                    */
/* -------------------------------------------------------------------------- */

#define BOARD_SPI2_PRESCALER                    SPI_BAUDRATEPRESCALER_128
#define BOARD_SPI2_CLK_POLARITY                 SPI_POLARITY_LOW
#define BOARD_SPI2_CLK_PHASE                    SPI_PHASE_1EDGE

/* -------------------------------------------------------------------------- */
/* SPI3 - NRF24L01+                                                           */
/* NRF24 SPI Mode 0                                                           */
/* -------------------------------------------------------------------------- */

#define BOARD_SPI3_PRESCALER                    SPI_BAUDRATEPRESCALER_32
#define BOARD_SPI3_CLK_POLARITY                 SPI_POLARITY_LOW
#define BOARD_SPI3_CLK_PHASE                    SPI_PHASE_1EDGE

#endif
