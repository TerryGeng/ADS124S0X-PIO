/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "spi.pio.h"

#define ADS1X4S0X_CS_PIN        5
#define ADS1X4S0X_SCK_PIN       2
#define ADS1X4S0X_TX_PIN        3
#define ADS1X4S0X_RX_PIN        4

#define ADS1X4S0X_DRDY_PIN      6

struct pio_spi_odm_transfer_schedule {
    uint32_t cnt;
    uint8_t *buffer;
};

struct pio_spi_odm_config {
    PIO pio;
    uint8_t sm;
    uint8_t cs_pin;
    uint8_t dma_chan_tx;
    uint8_t dma_chan_rx1;
    uint8_t dma_chan_rx2;
    uint8_t dma_chan_tx_ctrl1;
    uint8_t dma_chan_tx_ctrl2;
    struct pio_spi_odm_transfer_schedule tx_sche;
    struct pio_spi_odm_transfer_schedule rx_sche1;
    struct pio_spi_odm_transfer_schedule rx_sche2;
};

struct pio_spi_odm_raw_program {
    uint8_t *raw_inst;
    size_t tx_cnt;
    size_t rx_cnt;
    size_t iptr;
    bool half;
};

///////////////////////////////////

#define ADS1X4S0X_CLK_FREQ_IN_KHZ                           4096
#define ADS1X4S0X_RESET_LOW_TIME_IN_CLOCK_CYCLES            4
#define ADS1X4S0X_START_SYNC_PULSE_DURATION_IN_CLOCK_CYCLES 4
#define ADS1X4S0X_SETUP_TIME_IN_CLOCK_CYCLES                32
#define ADS1X4S0X_INPUT_SELECTION_AINCOM                    12
#define ADS1X4S0X_RESOLUTION                                24
#define ADS1X4S0X_REF_INTERNAL                              2500
#define ADS1X4S0X_GPIO_MAX                                  3
#define ADS1X4S0X_POWER_ON_RESET_TIME_IN_US                 2200
#define ADS1X4S0X_VBIAS_PIN_MAX                             7
#define ADS1X4S0X_VBIAS_PIN_MIN                             0

/* Not mentioned in the datasheet, but instead determined experimentally. */
#define ADS1X4S0X_RESET_DELAY_TIME_SAFETY_MARGIN_IN_US 1000
#define ADS1X4S0X_RESET_DELAY_TIME_IN_US                                                           \
	(4096 * 1000 / ADS1X4S0X_CLK_FREQ_IN_KHZ + ADS1X4S0X_RESET_DELAY_TIME_SAFETY_MARGIN_IN_US)

#define ADS1X4S0X_RESET_LOW_TIME_IN_US                                                             \
	(ADS1X4S0X_RESET_LOW_TIME_IN_CLOCK_CYCLES * 1000 / ADS1X4S0X_CLK_FREQ_IN_KHZ)
#define ADS1X4S0X_START_SYNC_PULSE_DURATION_IN_US                                                  \
	(ADS1X4S0X_START_SYNC_PULSE_DURATION_IN_CLOCK_CYCLES * 1000 / ADS1X4S0X_CLK_FREQ_IN_KHZ)
#define ADS1X4S0X_SETUP_TIME_IN_US                                                                 \
	(ADS1X4S0X_SETUP_TIME_IN_CLOCK_CYCLES * 1000 / ADS1X4S0X_CLK_FREQ_IN_KHZ)

enum ads1x4s0x_command {
	ADS1X4S0X_COMMAND_NOP = 0x00,
	ADS1X4S0X_COMMAND_WAKEUP = 0x02,
	ADS1X4S0X_COMMAND_POWERDOWN = 0x04,
	ADS1X4S0X_COMMAND_RESET = 0x06,
	ADS1X4S0X_COMMAND_START = 0x08,
	ADS1X4S0X_COMMAND_STOP = 0x0A,
	ADS1X4S0X_COMMAND_SYOCAL = 0x16,
	ADS1X4S0X_COMMAND_SYGCAL = 0x17,
	ADS1X4S0X_COMMAND_SFOCAL = 0x19,
	ADS1X4S0X_COMMAND_RDATA = 0x12,
	ADS1X4S0X_COMMAND_RREG = 0x20,
	ADS1X4S0X_COMMAND_WREG = 0x40,
};

enum ads1x4s0x_register {
	ADS1X4S0X_REGISTER_ID = 0x00,
	ADS1X4S0X_REGISTER_STATUS = 0x01,
	ADS1X4S0X_REGISTER_INPMUX = 0x02,
	ADS1X4S0X_REGISTER_PGA = 0x03,
	ADS1X4S0X_REGISTER_DATARATE = 0x04,
	ADS1X4S0X_REGISTER_REF = 0x05,
	ADS1X4S0X_REGISTER_IDACMAG = 0x06,
	ADS1X4S0X_REGISTER_IDACMUX = 0x07,
	ADS1X4S0X_REGISTER_VBIAS = 0x08,
	ADS1X4S0X_REGISTER_SYS = 0x09,
	ADS1X4S0X_REGISTER_GPIODAT = 0x10,
	ADS1X4S0X_REGISTER_GPIOCON = 0x11,
	ADS114S0X_REGISTER_OFCAL0 = 0x0B,
	ADS114S0X_REGISTER_OFCAL1 = 0x0C,
	ADS114S0X_REGISTER_FSCAL0 = 0x0E,
	ADS114S0X_REGISTER_FSCAL1 = 0x0F,
	ADS124S0X_REGISTER_OFCAL0 = 0x0A,
	ADS124S0X_REGISTER_OFCAL1 = 0x0B,
	ADS124S0X_REGISTER_OFCAL2 = 0x0C,
	ADS124S0X_REGISTER_FSCAL0 = 0x0E,
	ADS124S0X_REGISTER_FSCAL1 = 0x0F,
	ADS124S0X_REGISTER_FSCAL2 = 0x0F,
};

#define ADS1X4S0X_REGISTER_GET_VALUE(value, pos, length)                                           \
	FIELD_GET(GENMASK(pos + length - 1, pos), value)
#define ADS1X4S0X_REGISTER_SET_VALUE(target, value, pos, length)                                   \
	target &= ~GENMASK(pos + length - 1, pos);                                                 \
	target |= FIELD_PREP(GENMASK(pos + length - 1, pos), value)

#define ADS1X4S0X_REGISTER_ID_DEV_ID_LENGTH 3
#define ADS1X4S0X_REGISTER_ID_DEV_ID_POS    0
#define ADS1X4S0X_REGISTER_ID_DEV_ID_GET(value)                                                    \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_ID_DEV_ID_POS,                      \
				     ADS1X4S0X_REGISTER_ID_DEV_ID_LENGTH)
#define ADS1X4S0X_REGISTER_ID_DEV_ID_SET(target, value)                                            \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_ID_DEV_ID_POS,              \
				     ADS1X4S0X_REGISTER_ID_DEV_ID_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_POR_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_POR_POS    7
#define ADS1X4S0X_REGISTER_STATUS_FL_POR_GET(value)                                                \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_POR_POS,                  \
				     ADS1X4S0X_REGISTER_STATUS_FL_POR_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_POR_SET(target, value)                                        \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_POR_POS,          \
				     ADS1X4S0X_REGISTER_STATUS_FL_POR_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_NOT_RDY_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_NOT_RDY_POS    6
#define ADS1X4S0X_REGISTER_STATUS_NOT_RDY_GET(value)                                               \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_NOT_RDY_POS,                 \
				     ADS1X4S0X_REGISTER_STATUS_NOT_RDY_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_NOT_RDY_SET(target, value)                                       \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_NOT_RDY_POS,         \
				     ADS1X4S0X_REGISTER_STATUS_NOT_RDY_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_POS    5
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_GET(value)                                            \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_POS,              \
				     ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_SET(target, value)                                    \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_POS,      \
				     ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_POS    4
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_GET(value)                                            \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_POS,              \
				     ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_SET(target, value)                                    \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_POS,      \
				     ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_POS    3
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_GET(value)                                            \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_POS,              \
				     ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_SET(target, value)                                    \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_POS,      \
				     ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_POS    2
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_GET(value)                                            \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_POS,              \
				     ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_SET(target, value)                                    \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_POS,      \
				     ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_POS    1
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_GET(value)                                             \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_POS,               \
				     ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_SET(target, value)                                     \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_POS,       \
				     ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_POS    0
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_GET(value)                                             \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_POS,               \
				     ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_SET(target, value)                                     \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_POS,       \
				     ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_LENGTH)
#define ADS1X4S0X_REGISTER_INPMUX_MUXP_LENGTH 4
#define ADS1X4S0X_REGISTER_INPMUX_MUXP_POS    4
#define ADS1X4S0X_REGISTER_INPMUX_MUXP_GET(value)                                                  \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_INPMUX_MUXP_POS,                    \
				     ADS1X4S0X_REGISTER_INPMUX_MUXP_LENGTH)
#define ADS1X4S0X_REGISTER_INPMUX_MUXP_SET(target, value)                                          \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_INPMUX_MUXP_POS,            \
				     ADS1X4S0X_REGISTER_INPMUX_MUXP_LENGTH)
#define ADS1X4S0X_REGISTER_INPMUX_MUXN_LENGTH 4
#define ADS1X4S0X_REGISTER_INPMUX_MUXN_POS    0
#define ADS1X4S0X_REGISTER_INPMUX_MUXN_GET(value)                                                  \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_INPMUX_MUXN_POS,                    \
				     ADS1X4S0X_REGISTER_INPMUX_MUXN_LENGTH)
#define ADS1X4S0X_REGISTER_INPMUX_MUXN_SET(target, value)                                          \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_INPMUX_MUXN_POS,            \
				     ADS1X4S0X_REGISTER_INPMUX_MUXN_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_DELAY_LENGTH 3
#define ADS1X4S0X_REGISTER_PGA_DELAY_POS    5
#define ADS1X4S0X_REGISTER_PGA_DELAY_GET(value)                                                    \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_PGA_DELAY_POS,                      \
				     ADS1X4S0X_REGISTER_PGA_DELAY_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_DELAY_SET(target, value)                                            \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_PGA_DELAY_POS,              \
				     ADS1X4S0X_REGISTER_PGA_DELAY_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_PGA_EN_LENGTH 2
#define ADS1X4S0X_REGISTER_PGA_PGA_EN_POS    3
#define ADS1X4S0X_REGISTER_PGA_PGA_EN_GET(value)                                                   \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_PGA_PGA_EN_POS,                     \
				     ADS1X4S0X_REGISTER_PGA_PGA_EN_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_PGA_EN_SET(target, value)                                           \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_PGA_PGA_EN_POS,             \
				     ADS1X4S0X_REGISTER_PGA_PGA_EN_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_GAIN_LENGTH 3
#define ADS1X4S0X_REGISTER_PGA_GAIN_POS    0
#define ADS1X4S0X_REGISTER_PGA_GAIN_GET(value)                                                     \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_PGA_GAIN_POS,                       \
				     ADS1X4S0X_REGISTER_PGA_GAIN_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_GAIN_SET(target, value)                                             \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_PGA_GAIN_POS,               \
				     ADS1X4S0X_REGISTER_PGA_GAIN_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_G_CHOP_LENGTH 1
#define ADS1X4S0X_REGISTER_DATARATE_G_CHOP_POS    7
#define ADS1X4S0X_REGISTER_DATARATE_G_CHOP_GET(value)                                              \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_G_CHOP_POS,                \
				     ADS1X4S0X_REGISTER_DATARATE_G_CHOP_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_G_CHOP_SET(target, value)                                      \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_G_CHOP_POS,        \
				     ADS1X4S0X_REGISTER_DATARATE_G_CHOP_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_CLK_LENGTH 1
#define ADS1X4S0X_REGISTER_DATARATE_CLK_POS    6
#define ADS1X4S0X_REGISTER_DATARATE_CLK_GET(value)                                                 \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_CLK_POS,                   \
				     ADS1X4S0X_REGISTER_DATARATE_CLK_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_CLK_SET(target, value)                                         \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_CLK_POS,           \
				     ADS1X4S0X_REGISTER_DATARATE_CLK_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_MODE_LENGTH 1
#define ADS1X4S0X_REGISTER_DATARATE_MODE_POS    5
#define ADS1X4S0X_REGISTER_DATARATE_MODE_GET(value)                                                \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_MODE_POS,                  \
				     ADS1X4S0X_REGISTER_DATARATE_MODE_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_MODE_SET(target, value)                                        \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_MODE_POS,          \
				     ADS1X4S0X_REGISTER_DATARATE_MODE_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_FILTER_LENGTH 1
#define ADS1X4S0X_REGISTER_DATARATE_FILTER_POS    4
#define ADS1X4S0X_REGISTER_DATARATE_FILTER_GET(value)                                              \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_FILTER_POS,                \
				     ADS1X4S0X_REGISTER_DATARATE_FILTER_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_FILTER_SET(target, value)                                      \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_FILTER_POS,        \
				     ADS1X4S0X_REGISTER_DATARATE_FILTER_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_DR_LENGTH 4
#define ADS1X4S0X_REGISTER_DATARATE_DR_POS    0
#define ADS1X4S0X_REGISTER_DATARATE_DR_GET(value)                                                  \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_DR_POS,                    \
				     ADS1X4S0X_REGISTER_DATARATE_DR_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_DR_SET(target, value)                                          \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_DR_POS,            \
				     ADS1X4S0X_REGISTER_DATARATE_DR_LENGTH)
#define ADS1X4S0X_REGISTER_REF_FL_REF_EN_LENGTH 2
#define ADS1X4S0X_REGISTER_REF_FL_REF_EN_POS    6
#define ADS1X4S0X_REGISTER_REF_FL_REF_EN_GET(value)                                                \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_FL_REF_EN_POS,                  \
				     ADS1X4S0X_REGISTER_REF_FL_REF_EN_LENGTH)
#define ADS1X4S0X_REGISTER_REF_FL_REF_EN_SET(target, value)                                        \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_FL_REF_EN_POS,          \
				     ADS1X4S0X_REGISTER_REF_FL_REF_EN_LENGTH)
#define ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_LENGTH 1
#define ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_POS    5
#define ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_GET(value)                                             \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_POS,               \
				     ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_LENGTH)
#define ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_SET(target, value)                                     \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_POS,       \
				     ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_LENGTH)
#define ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_LENGTH 1
#define ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_POS    4
#define ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_GET(value)                                             \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_POS,               \
				     ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_LENGTH)
#define ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_SET(target, value)                                     \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_POS,       \
				     ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_LENGTH)
#define ADS1X4S0X_REGISTER_REF_REFSEL_LENGTH 2
#define ADS1X4S0X_REGISTER_REF_REFSEL_POS    2
#define ADS1X4S0X_REGISTER_REF_REFSEL_GET(value)                                                   \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_REFSEL_POS,                     \
				     ADS1X4S0X_REGISTER_REF_REFSEL_LENGTH)
#define ADS1X4S0X_REGISTER_REF_REFSEL_SET(target, value)                                           \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_REFSEL_POS,             \
				     ADS1X4S0X_REGISTER_REF_REFSEL_LENGTH)
#define ADS1X4S0X_REGISTER_REF_REFCON_LENGTH 2
#define ADS1X4S0X_REGISTER_REF_REFCON_POS    0
#define ADS1X4S0X_REGISTER_REF_REFCON_GET(value)                                                   \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_REFCON_POS,                     \
				     ADS1X4S0X_REGISTER_REF_REFCON_LENGTH)
#define ADS1X4S0X_REGISTER_REF_REFCON_SET(target, value)                                           \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_REFCON_POS,             \
				     ADS1X4S0X_REGISTER_REF_REFCON_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_LENGTH 1
#define ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_POS    7
#define ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_GET(value)                                           \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_POS,             \
				     ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_SET(target, value)                                   \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_POS,     \
				     ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_PSW_LENGTH 1
#define ADS1X4S0X_REGISTER_IDACMAG_PSW_POS    6
#define ADS1X4S0X_REGISTER_IDACMAG_PSW_GET(value)                                                  \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMAG_PSW_POS,                    \
				     ADS1X4S0X_REGISTER_IDACMAG_PSW_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_PSW_SET(target, value)                                          \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMAG_PSW_POS,            \
				     ADS1X4S0X_REGISTER_IDACMAG_PSW_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_IMAG_LENGTH 4
#define ADS1X4S0X_REGISTER_IDACMAG_IMAG_POS    0
#define ADS1X4S0X_REGISTER_IDACMAG_IMAG_GET(value)                                                 \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMAG_IMAG_POS,                   \
				     ADS1X4S0X_REGISTER_IDACMAG_IMAG_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(target, value)                                         \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMAG_IMAG_POS,           \
				     ADS1X4S0X_REGISTER_IDACMAG_IMAG_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMUX_I2MUX_LENGTH 4
#define ADS1X4S0X_REGISTER_IDACMUX_I2MUX_POS    4
#define ADS1X4S0X_REGISTER_IDACMUX_I2MUX_GET(value)                                                \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMUX_I2MUX_POS,                  \
				     ADS1X4S0X_REGISTER_IDACMUX_I2MUX_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMUX_I2MUX_SET(target, value)                                        \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMUX_I2MUX_POS,          \
				     ADS1X4S0X_REGISTER_IDACMUX_I2MUX_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMUX_I1MUX_LENGTH 4
#define ADS1X4S0X_REGISTER_IDACMUX_I1MUX_POS    0
#define ADS1X4S0X_REGISTER_IDACMUX_I1MUX_GET(value)                                                \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMUX_I1MUX_POS,                  \
				     ADS1X4S0X_REGISTER_IDACMUX_I1MUX_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMUX_I1MUX_SET(target, value)                                        \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMUX_I1MUX_POS,          \
				     ADS1X4S0X_REGISTER_IDACMUX_I1MUX_LENGTH)
#define ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_LENGTH 1
#define ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_POS    7
#define ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_GET(value)                                               \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_POS,                 \
				     ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_LENGTH)
#define ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_SET(target, value)                                       \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_POS,         \
				     ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_LENGTH)
#define ADS1X4S0X_REGISTER_GPIODAT_DIR_LENGTH 4
#define ADS1X4S0X_REGISTER_GPIODAT_DIR_POS    4
#define ADS1X4S0X_REGISTER_GPIODAT_DIR_GET(value)                                                  \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_GPIODAT_DIR_POS,                    \
				     ADS1X4S0X_REGISTER_GPIODAT_DIR_LENGTH)
#define ADS1X4S0X_REGISTER_GPIODAT_DIR_SET(target, value)                                          \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_GPIODAT_DIR_POS,            \
				     ADS1X4S0X_REGISTER_GPIODAT_DIR_LENGTH)
#define ADS1X4S0X_REGISTER_GPIODAT_DAT_LENGTH 4
#define ADS1X4S0X_REGISTER_GPIODAT_DAT_POS    0
#define ADS1X4S0X_REGISTER_GPIODAT_DAT_GET(value)                                                  \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_GPIODAT_DAT_POS,                    \
				     ADS1X4S0X_REGISTER_GPIODAT_DAT_LENGTH)
#define ADS1X4S0X_REGISTER_GPIODAT_DAT_SET(target, value)                                          \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_GPIODAT_DAT_POS,            \
				     ADS1X4S0X_REGISTER_GPIODAT_DAT_LENGTH)
#define ADS1X4S0X_REGISTER_GPIOCON_CON_LENGTH 4
#define ADS1X4S0X_REGISTER_GPIOCON_CON_POS    0
#define ADS1X4S0X_REGISTER_GPIOCON_CON_GET(value)                                                  \
	ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_GPIOCON_CON_POS,                    \
				     ADS1X4S0X_REGISTER_GPIOCON_CON_LENGTH)
#define ADS1X4S0X_REGISTER_GPIOCON_CON_SET(target, value)                                          \
	ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_GPIOCON_CON_POS,            \
				     ADS1X4S0X_REGISTER_GPIOCON_CON_LENGTH)

/*
 * - AIN0 as positive input
 * - AIN1 as negative input
 */
#define ADS1X4S0X_REGISTER_INPMUX_SET_DEFAULTS(target)                                             \
	ADS1X4S0X_REGISTER_INPMUX_MUXP_SET(target, 0b0000);                                        \
	ADS1X4S0X_REGISTER_INPMUX_MUXN_SET(target, 0b0001)
/*
 * - disable reference monitor
 * - enable positive reference buffer
 * - disable negative reference buffer
 * - use internal reference
 * - enable internal voltage reference
 */
#define ADS1X4S0X_REGISTER_REF_SET_DEFAULTS(target)                                                \
	ADS1X4S0X_REGISTER_REF_FL_REF_EN_SET(target, 0b00);                                        \
	ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_SET(target, 0b0);                                      \
	ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_SET(target, 0b1);                                      \
	ADS1X4S0X_REGISTER_REF_REFSEL_SET(target, 0b10);                                           \
	ADS1X4S0X_REGISTER_REF_REFCON_SET(target, 0b01)
/*
 * - disable global chop
 * - use internal oscillator
 * - single shot conversion mode
 * - low latency filter
 * - 20 samples per second
 */
#define ADS1X4S0X_REGISTER_DATARATE_SET_DEFAULTS(target)                                           \
	ADS1X4S0X_REGISTER_DATARATE_G_CHOP_SET(target, 0b0);                                       \
	ADS1X4S0X_REGISTER_DATARATE_CLK_SET(target, 0b0);                                          \
	ADS1X4S0X_REGISTER_DATARATE_MODE_SET(target, 0b1);                                         \
	ADS1X4S0X_REGISTER_DATARATE_FILTER_SET(target, 0b1);                                       \
	ADS1X4S0X_REGISTER_DATARATE_DR_SET(target, 0b0100)
/*
 * - delay of 14*t_mod
 * - disable gain
 * - gain 1
 */
#define ADS1X4S0X_REGISTER_PGA_SET_DEFAULTS(target)                                                \
	ADS1X4S0X_REGISTER_PGA_DELAY_SET(target, 0b000);                                           \
	ADS1X4S0X_REGISTER_PGA_PGA_EN_SET(target, 0b00);                                           \
	ADS1X4S0X_REGISTER_PGA_GAIN_SET(target, 0b000)
/*
 * - disable PGA output rail flag
 * - low-side power switch
 * - IDAC off
 */
#define ADS1X4S0X_REGISTER_IDACMAG_SET_DEFAULTS(target)                                            \
	ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_SET(target, 0b0);                                    \
	ADS1X4S0X_REGISTER_IDACMAG_PSW_SET(target, 0b0);                                           \
	ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(target, 0b0000)
/*
 * - disconnect IDAC1
 * - disconnect IDAC2
 */
#define ADS1X4S0X_REGISTER_IDACMUX_SET_DEFAULTS(target)                                            \
	ADS1X4S0X_REGISTER_IDACMUX_I1MUX_SET(target, 0b1111);                                      \
	ADS1X4S0X_REGISTER_IDACMUX_I2MUX_SET(target, 0b1111)

static inline uint16_t sys_get_be16(const uint8_t src[2])
{
	return ((uint16_t)src[0] << 8) | src[1];
}

static inline uint32_t sys_get_be32(const uint8_t src[4])
{
	return ((uint32_t)sys_get_be16(&src[0]) << 16) | sys_get_be16(&src[2]);
}

#ifdef PICO_DEFAULT_SPI_CSN_PIN
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(ADS1X4S0X_CS_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(ADS1X4S0X_CS_PIN, 1);
    asm volatile("nop \n nop \n nop");
}
#endif

#if defined(spi_default) && defined( ADS1X4S0X_CS_PIN)
static void ads124s06_send_command(uint8_t cmd) {
    cs_select();
    sleep_ms(1);

    spi_write_blocking(spi_default, &cmd, 1);

    cs_deselect();
}

static void ads124s06_write_register(uint8_t register_address, uint8_t value) {
    uint8_t buffer_tx[3];
    cs_select();
    sleep_ms(1);

	buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_WREG) | ((uint8_t)register_address);
	/* write one register */
	buffer_tx[1] = 0x00;
	buffer_tx[2] = value;

    spi_write_blocking(spi_default, buffer_tx, 3);

	//printf("ads124s06: writing to register 0x%02X value 0x%02X\n", register_address, value);

    cs_deselect();
}

static void ads124s06_read_register(uint8_t register_address, uint8_t *buf) {
    cs_select();

	uint8_t buffer_tx[3];
	uint8_t buffer_rx[3];

	buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_RREG) | ((uint8_t)register_address);
	buffer_tx[1] = 0x00;
	buffer_tx[2] = 0x00;
	/* read one register */

	spi_write_read_blocking(spi_default, buffer_tx, buffer_rx, 3);

	//printf("ads124s06: read register 0x%02X (send 0x%02X%02X) value 0x%02X\n",
    //        register_address,
    //        buffer_tx[0],
    //        buffer_tx[1],
    //        buffer_rx[2]
    //        );

	*buf = buffer_rx[2];

    cs_deselect();
}

static int32_t ads124s06_read_sample() {
    uint8_t buffer_tx[1];
    uint8_t buffer_rx[4] = {0};
    cs_select();

	buffer_tx[0] = (uint8_t)ADS1X4S0X_COMMAND_RDATA;

    spi_write_blocking(spi_default, buffer_tx, 1);

    spi_read_blocking(spi_default, 0, buffer_rx, 3);

    cs_deselect();

	return (int32_t)sys_get_be32(buffer_rx) >> (32 - ADS1X4S0X_RESOLUTION);
}

#endif


void pio_spi_odm_write_read(
        const struct pio_spi_odm_config *spi_odm,
        struct pio_spi_odm_raw_program *pgm,
        uint8_t *dst) {

    io_rw_8 *txfifo = (io_rw_8 *) &spi_odm->pio->txf[spi_odm->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi_odm->pio->rxf[spi_odm->sm];

    size_t write_len = pgm->iptr + 1;
    size_t read_len = pgm->rx_cnt;
    uint8_t *src = pgm->raw_inst;

    while (write_len || read_len) {
        if (write_len && !pio_sm_is_tx_fifo_full(spi_odm->pio, spi_odm->sm)) {
            *txfifo = *src++;
            --write_len;
        }
        if (read_len && !pio_sm_is_rx_fifo_empty(spi_odm->pio, spi_odm->sm)) {
            *dst++ = *rxfifo;
            --read_len;
        }
    }
}

void pio_spi_odm_write8_read8_blocking_dma(
        const struct pio_spi_odm_config *spi_odm,
        struct pio_spi_odm_raw_program *pgm,
        uint8_t *dst) {
    size_t write_len = pgm->iptr + 1;
    size_t read_len = pgm->rx_cnt;
    uint8_t *src = pgm->raw_inst;

    dma_channel_config ctx = dma_channel_get_default_config(spi_odm->dma_chan_tx);
    dma_channel_config crx = dma_channel_get_default_config(spi_odm->dma_chan_rx1);

    channel_config_set_read_increment(&ctx, true);
    channel_config_set_write_increment(&ctx, false);
    channel_config_set_dreq(&ctx, pio_get_dreq(spi_odm->pio, spi_odm->sm, true));
    channel_config_set_transfer_data_size(&ctx, DMA_SIZE_8);

    channel_config_set_read_increment(&crx, false);
    channel_config_set_write_increment(&crx, true);
    channel_config_set_dreq(&crx, pio_get_dreq(spi_odm->pio, spi_odm->sm, false));
    channel_config_set_transfer_data_size(&crx, DMA_SIZE_8);

    dma_channel_configure(spi_odm->dma_chan_tx, &ctx, &spi_odm->pio->txf[spi_odm->sm], src, write_len, true);
    dma_channel_configure(spi_odm->dma_chan_rx1, &crx, dst, &spi_odm->pio->rxf[spi_odm->sm], read_len, true);

    dma_channel_wait_for_finish_blocking(spi_odm->dma_chan_rx1);
}

static void pio_spi_odm_dma_run_forever(struct pio_spi_odm_config *spi_odm,
        struct pio_spi_odm_raw_program *pgm,
        uint8_t *rx_buf1, size_t rx_buf_len1,
        uint8_t *rx_buf2, size_t rx_buf_len2) {

    dma_channel_config txcc1 = dma_channel_get_default_config(spi_odm->dma_chan_tx_ctrl1);
    dma_channel_config txcc2 = dma_channel_get_default_config(spi_odm->dma_chan_tx_ctrl2);
    dma_channel_config txc = dma_channel_get_default_config(spi_odm->dma_chan_tx);
    dma_channel_config rxc1 = dma_channel_get_default_config(spi_odm->dma_chan_rx1);
    dma_channel_config rxc2 = dma_channel_get_default_config(spi_odm->dma_chan_rx2);


    spi_odm->tx_sche.buffer = pgm->raw_inst;
    spi_odm->tx_sche.cnt = pgm->tx_cnt;

    spi_odm->rx_sche1.buffer = rx_buf1;
    spi_odm->rx_sche1.cnt = rx_buf_len1;

    spi_odm->rx_sche2.buffer = rx_buf2;
    spi_odm->rx_sche2.cnt = rx_buf_len2;

    /* Config tx */
    channel_config_set_transfer_data_size(&txcc1, DMA_SIZE_32);
    channel_config_set_read_increment(&txcc1, false);
    channel_config_set_write_increment(&txcc1, false);

    dma_channel_configure(
        spi_odm->dma_chan_tx_ctrl1,
        &txcc1,
        &dma_hw->ch[spi_odm->dma_chan_tx_ctrl2].al3_read_addr_trig,
        &spi_odm->tx_sche,
        1,
        false
    );

    channel_config_set_transfer_data_size(&txcc2, DMA_SIZE_32);
    channel_config_set_read_increment(&txcc2, true);
    channel_config_set_write_increment(&txcc2, true);

    dma_channel_configure(
        spi_odm->dma_chan_tx_ctrl2,
        &txcc2,
        &dma_hw->ch[spi_odm->dma_chan_tx].al3_transfer_count,
        NULL,
        2,
        false
    );

    channel_config_set_transfer_data_size(&txc, DMA_SIZE_8);
    channel_config_set_read_increment(&txc, true);
    channel_config_set_write_increment(&txc, false);
    dma_channel_configure(spi_odm->dma_chan_tx, &txc, &spi_odm->pio->txf[spi_odm->sm], NULL, 0, false);
    channel_config_set_chain_to(&txc, spi_odm->dma_chan_tx_ctrl1);

    /* Config rx */
    /*channel_config_set_read_increment(&rxc1, false);*/
    /*channel_config_set_write_increment(&rxc1, true);*/
    /*channel_config_set_dreq(&rxc1, pio_get_dreq(spi->pio, spi->sm, false));*/
    /*channel_config_set_transfer_data_size(&rxc1, DMA_SIZE_8);*/
    /*channel_config_set_chain_to(&rxc1, spi_odm->dma_chan_rx2);*/
    /**/
    /*dma_channel_configure(spi->dma_chan_rx1, &crx1, spi_odm->rx_sche1.buffer, &spi->pio->rxf[spi->sm], spi_odm->rx_sche1.cnt, false);*/
    /**/
    /*channel_config_set_read_increment(&rxc2, false);*/
    /*channel_config_set_write_increment(&rxc2, true);*/
    /*channel_config_set_dreq(&rxc1, pio_get_dreq(spi->pio, spi->sm, false));*/
    /*channel_config_set_transfer_data_size(&rxc2, DMA_SIZE_8);*/
    /*channel_config_set_chain_to(&rxc2, spi_odm->dma_chan_rx1);*/
    /**/
    /*dma_channel_configure(spi->dma_chan_rx2, &crx2, spi_odm->rx_sche2.buffer, &spi->pio->rxf[spi->sm], spi_odm->rx_sche2.cnt, false);*/
}

void pio_spi_odm_inst_do_tx_rx(struct pio_spi_odm_raw_program *pgm, uint8_t tx_byte, bool do_rx) {
    uint8_t raw_pio_inst = 0;

    pgm->rx_cnt += do_rx;

    for (int j = 8; j > 0; j--) {
        raw_pio_inst = (1 << 2) | ((~do_rx & 1) << 1) | ((tx_byte >> (j-1)) & 1);

        if (!pgm->half) {
            pgm->raw_inst[pgm->iptr] = raw_pio_inst << 4;
            pgm->half = true;

        } else {
            pgm->raw_inst[pgm->iptr] |= raw_pio_inst;
            pgm->half = false;

            ++pgm->iptr;
        }
    }

    pgm->tx_cnt = pgm->iptr + 1;
}

void pio_spi_odm_inst_do_wait(struct pio_spi_odm_raw_program *pgm) {

    uint8_t raw_pio_inst = 0;

    if (!pgm->half) {
        pgm->raw_inst[pgm->iptr] = raw_pio_inst << 4;
        pgm->half = true;

    } else {
        pgm->raw_inst[pgm->iptr] |= raw_pio_inst;
        pgm->half = false;

        ++pgm->iptr;
    }

    pgm->tx_cnt = pgm->iptr + 1;
}

void pio_spi_odm_inst_finalize(struct pio_spi_odm_raw_program *pgm) {
    pgm->tx_cnt = pgm->iptr + 1;

    if (pgm->half) {
        pgm->raw_inst[pgm->iptr] |= 0x06;
    }
}

void pio_spi_odm_print_pgm(struct pio_spi_odm_raw_program *pgm) {
    size_t raw_pio_insts_cnt = pgm->iptr + 1;

    printf("Total insts: %d = (%d bytes)\n", 2 * pgm->iptr + pgm->half, raw_pio_insts_cnt);
    for (int i = 0; i < raw_pio_insts_cnt; ++i) {
        printf("%d:     0x%02x\n", i, pgm->raw_inst[i]);
    }
    printf("End.\n");

}


struct pio_spi_odm_config spi_odm = {
        .pio = pio0,
        .sm = 0,
        .cs_pin = ADS1X4S0X_CS_PIN,
        .dma_chan_tx = 0,
        .dma_chan_tx_ctrl1 = 1,
        .dma_chan_tx_ctrl2 = 2,
        .dma_chan_rx1 = 3,
        .dma_chan_rx2 = 4,
};

#define NUM_OF_CHAN 3
static uint8_t chan_config[] = {0x01, 0x23, 0x45};
static uint8_t raw_pio_insts[NUM_OF_CHAN*40];

static struct pio_spi_odm_raw_program pgm = {
    .raw_inst = raw_pio_insts,
    .half = 0,
    .iptr = 0,
    .rx_cnt = 0
};

int main() {
    stdio_init_all();
#if !defined(spi_default) || !defined(ADS1X4S0X_SCK_PIN) || !defined(ADS1X4S0X_TX_PIN) || !defined(ADS1X4S0X_RX_PIN) || !defined(ADS1X4S0X_CS_PIN)
#warning this example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else

    // This example will use SPI0 at 0.5MHz.
    spi_init(spi_default, 1000 * 1000);

    spi_set_format(spi_default, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_dir(ADS1X4S0X_DRDY_PIN, GPIO_IN);
    gpio_set_dir(ADS1X4S0X_SCK_PIN, GPIO_OUT);

    gpio_set_dir(ADS1X4S0X_SCK_PIN, GPIO_OUT);
    gpio_set_function(ADS1X4S0X_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADS1X4S0X_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADS1X4S0X_TX_PIN, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(ADS1X4S0X_RX_PIN, ADS1X4S0X_TX_PIN, ADS1X4S0X_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(ADS1X4S0X_CS_PIN);
    gpio_set_dir(ADS1X4S0X_CS_PIN, GPIO_OUT);
    gpio_put(ADS1X4S0X_CS_PIN, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(ADS1X4S0X_CS_PIN, "SPI CS"));

    ads124s06_send_command(ADS1X4S0X_COMMAND_RESET);

    uint8_t ret;
    ads124s06_read_register(ADS1X4S0X_REGISTER_ID, &ret);
    printf("Chip ID is 0x%x\n", ret);

    ads124s06_read_register(ADS1X4S0X_REGISTER_STATUS, &ret);
    printf("Chip STATUS is 0x%x\n", ret);

    ads124s06_write_register(ADS1X4S0X_REGISTER_GPIOCON, 0x01);
    ads124s06_write_register(ADS1X4S0X_REGISTER_GPIODAT, 0xe1);

    ads124s06_write_register(ADS1X4S0X_REGISTER_PGA, 0x00);
    ads124s06_write_register(ADS1X4S0X_REGISTER_DATARATE, 0x38);
    ads124s06_write_register(ADS1X4S0X_REGISTER_REF, 0x39);
    ads124s06_write_register(ADS1X4S0X_REGISTER_IDACMAG, 0x00);
    ads124s06_write_register(ADS1X4S0X_REGISTER_IDACMUX, 0xff);
    ads124s06_write_register(ADS1X4S0X_REGISTER_VBIAS, 0x00);

    int32_t sample;
    uint8_t offset = pio_add_program(spi_odm.pio, &spi_cpha1_read_on_demand_program);

    printf("Loaded PIO program at %d\n", offset);

    pio_spi_read_on_demand_init(spi_odm.pio, spi_odm.sm, offset,
                 8,       // 8 bits per SPI frame
                 31.25f,  // 1 MHz @ 125 clk_sys
                 ADS1X4S0X_SCK_PIN,
                 ADS1X4S0X_TX_PIN,
                 ADS1X4S0X_RX_PIN
    );

    for (int i = 0; i < NUM_OF_CHAN; ++i) {
        pio_spi_odm_inst_do_tx_rx(&pgm, ADS1X4S0X_COMMAND_WREG | ADS1X4S0X_REGISTER_INPMUX, false);
        pio_spi_odm_inst_do_tx_rx(&pgm, 0x00, false);
        pio_spi_odm_inst_do_tx_rx(&pgm, chan_config[i], false);

        pio_spi_odm_inst_do_tx_rx(&pgm, ADS1X4S0X_COMMAND_START, false);
        pio_spi_odm_inst_do_wait(&pgm);

        /*pio_spi_odm_inst_do_tx_rx(&pgm, ADS1X4S0X_COMMAND_RDATA, false);*/
        pio_spi_odm_inst_do_tx_rx(&pgm, 0x00, true);
        pio_spi_odm_inst_do_tx_rx(&pgm, 0x00, true);
        pio_spi_odm_inst_do_tx_rx(&pgm, 0x00, true);
        pio_spi_odm_inst_do_tx_rx(&pgm, 0x00, true);
    }
    pio_spi_odm_inst_finalize(&pgm);
    pio_spi_odm_print_pgm(&pgm);

    uint8_t rx_buf[4*NUM_OF_CHAN];

    while (true) {
        gpio_put(ADS1X4S0X_CS_PIN, 0);
        sleep_ms(1);

    /*    pio_spi_odm_write_read(&spi, &pgm, rx_buf);*/
        pio_spi_odm_write8_read8_blocking_dma(&spi_odm, &pgm, rx_buf);

        gpio_put(ADS1X4S0X_CS_PIN, 1);

        for (int i = 0; i < NUM_OF_CHAN; ++i) {
            sample = (int32_t)sys_get_be32(&rx_buf[i*4]) >> (32 - ADS1X4S0X_RESOLUTION);
            int64_t my_val_mv = (int64_t)sample * 2500 >> 23;
            float my_fval_mv = 2500.0 * sample / (1 << 23);

            printf("%d: Read 0x%06x = %d = %d mV = %.3f mV\n", i, sample, sample, my_val_mv, my_fval_mv);
        }
        sleep_ms(1000);
    }

    /*uint32_t tot_read_cnt = 0;*/
    /*io_rw_8 *rxfifo = (io_rw_8 *) &spi_odm->pio->rxf[spi_odm->sm];*/
    /*uint8_t len = pgm.rx_cnt;*/
    /**/
    /*while (true) {*/
    /*    for (uint8_t j = 0; j < pgm.rx_cnt; j++) {*/
    /*        if (pio_sm_is_rx_fifo_empty(spi_odm->pio, spi_odm->sm)) {*/
    /*            continue;*/
    /*        }*/
    /*        rx_buf[j] = *rxfifo;*/
    /*    }*/
    /**/
    /*    for (int i = 0; i < NUM_OF_CHAN; ++i) {*/
    /*        sample = (int32_t)sys_get_be32(&rx_buf[i*4]) >> (32 - ADS1X4S0X_RESOLUTION);*/
    /*        int64_t my_val_mv = (int64_t)sample * 2500 >> 23;*/
    /*        float my_fval_mv = 2500.0 * sample / (1 << 23);*/
    /**/
    /*        printf("[%d] %d: Read 0x%06x = %d = %d mV = %.3f mV\n",*/
    /*                read_cnt,*/
    /*                i,*/
    /*                sample,*/
    /*                sample,*/
    /*                my_val_mv,*/
    /*                my_fval_mv);*/
    /**/
    /*        read_cnt++;*/
    /*    }*/
    /*}*/

#endif
}
