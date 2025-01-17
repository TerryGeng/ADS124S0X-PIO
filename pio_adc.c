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

typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
    uint dma_chan_tx;
    uint dma_chan_rx;
} pio_spi_inst_t;

void pio_spi_write8_read8_blocking_dma(const pio_spi_inst_t *spi, uint8_t *buffer_tx, uint8_t *buffer_rx, size_t len) {
    dma_channel_config ctx = dma_channel_get_default_config(spi->dma_chan_tx);
    dma_channel_config crx = dma_channel_get_default_config(spi->dma_chan_rx);

    channel_config_set_read_increment(&ctx, true);
    channel_config_set_write_increment(&ctx, false);
    channel_config_set_dreq(&ctx, pio_get_dreq(spi->pio, spi->sm, true));
    channel_config_set_transfer_data_size(&ctx, DMA_SIZE_8);

    channel_config_set_read_increment(&crx, false);
    channel_config_set_write_increment(&crx, true);
    channel_config_set_dreq(&crx, pio_get_dreq(spi->pio, spi->sm, false));
    channel_config_set_transfer_data_size(&crx, DMA_SIZE_8);

    dma_channel_configure(spi->dma_chan_tx, &ctx, &spi->pio->txf[spi->sm], buffer_tx, len, true);
    dma_channel_configure(spi->dma_chan_rx, &crx, buffer_rx, &spi->pio->rxf[spi->sm], len, true);

    dma_channel_wait_for_finish_blocking(spi->dma_chan_rx);
}

#define ADS1X4S0X_CS_PIN        5
#define ADS1X4S0X_SCK_PIN       2
#define ADS1X4S0X_TX_PIN        3
#define ADS1X4S0X_RX_PIN        4

#define ADS1X4S0X_DRDY_PIN      6


const pio_spi_inst_t spi = {
        .pio = pio0,
        .sm = 0,
        .cs_pin = ADS1X4S0X_CS_PIN,
        .dma_chan_tx = 1,
        .dma_chan_rx = 0
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

static void ads124s06_write_register_pio(uint8_t register_address, uint8_t value) {
    uint8_t buffer_tx[3];
    uint8_t buffer_rx[3];
    cs_select();

	buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_WREG) | ((uint8_t)register_address);
	/* write one register */
	buffer_tx[1] = 0x00;
	buffer_tx[2] = value;

	pio_spi_write8_read8_blocking_dma(&spi, buffer_tx, buffer_rx, 3);

	//printf("ads124s06: writing to register 0x%02X value 0x%02X\n", register_address, value);

    cs_deselect();
}

static void ads124s06_read_register_pio(uint8_t register_address, uint8_t *buf) {
    cs_select();

	uint8_t buffer_tx[3];
	uint8_t buffer_rx[3];

	buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_RREG) | ((uint8_t)register_address);
	buffer_tx[1] = 0x00;
	buffer_tx[2] = 0x00;
	/* read one register */

	pio_spi_write8_read8_blocking_dma(&spi, buffer_tx, buffer_rx, 3);

	//printf("ads124s06: read register 0x%02X (send 0x%02X%02X) value 0x%02X\n",
    //        register_address,
    //        buffer_tx[0],
    //        buffer_tx[1],
    //        buffer_rx[2]
    //        );

	*buf = buffer_rx[2];

    cs_deselect();
}

#endif


typedef struct pio_spi_odm_inst {
    uint8_t tx_byte;
    bool read;
    bool delay;
} pio_spi_odm_inst_t;

struct pio_spi_odm_raw_program {
    uint8_t *raw_inst;
    size_t iptr;
    size_t rx_cnt;
    bool half;
};

int make_pio_spi_odm_inst(pio_spi_odm_inst_t *buf_tx, size_t len, uint8_t *pio_inst_tx) {
    int iptr = 0;
    bool half = false;

    for (int i = 0; i < len; i++) {
        pio_spi_odm_inst_t inst = buf_tx[i];

        for (int j = 8; j > 0; j--) {
            uint8_t raw_pio_inst = 0;

            if (!inst.delay) {
                raw_pio_inst = (1 << 2) | ((~inst.read & 1) << 1) | ((inst.tx_byte >> (j-1)) & 1);
            } else {
                raw_pio_inst = 0;
            }

            if (!half) {
                pio_inst_tx[iptr] = raw_pio_inst << 4;
                half = true;

            } else {
                pio_inst_tx[iptr] |= raw_pio_inst;
                half = false;

                ++iptr;
            }

            if (inst.delay)  {
                break;
            }
        }

    }

    return iptr * 2 + half;
}

void pio_spi_odm_write_read(
        const pio_spi_inst_t *spi_odm,
        struct pio_spi_odm_raw_program *pgm,
        uint8_t *dst,
        bool fix_incompl) {

    io_rw_8 *txfifo = (io_rw_8 *) &spi_odm->pio->txf[spi_odm->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi_odm->pio->rxf[spi_odm->sm];

    size_t write_len = pgm->iptr + 1;
    size_t read_len = pgm->rx_cnt;
    uint8_t *src = pgm->raw_inst;

    if (fix_incompl && pgm->half) {
        pgm->raw_inst[pgm->iptr] |= 0x06;
    }

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

void pio_spi_odm_inst_do_tx(struct pio_spi_odm_raw_program *pgm,
        uint8_t tx_byte, bool do_rx) {

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
}


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

    uint8_t offset = pio_add_program(spi.pio, &spi_cpha1_read_on_demand_program);
    printf("Loaded PIO program at %d\n", offset);

    pio_spi_read_on_demand_init(spi.pio, spi.sm, offset,
                 8,       // 8 bits per SPI frame
                 31.25f,  // 1 MHz @ 125 clk_sys
                 ADS1X4S0X_SCK_PIN,
                 ADS1X4S0X_TX_PIN,
                 ADS1X4S0X_RX_PIN
    );
    const uint8_t num_of_chan = 3;
    uint8_t chan_config[] = {0x01, 0x23, 0x45};

    uint8_t raw_pio_insts[num_of_chan*40];
    struct pio_spi_odm_raw_program pgm = {
        .raw_inst = raw_pio_insts,
        .half = 0,
        .iptr = 0,
        .rx_cnt = 0
    };

    for (int i = 0; i < num_of_chan; ++i) {
        pio_spi_odm_inst_do_tx(&pgm, ADS1X4S0X_COMMAND_WREG | ADS1X4S0X_REGISTER_INPMUX, false);

        pio_spi_odm_inst_do_tx(&pgm, 0x00, false);
        pio_spi_odm_inst_do_tx(&pgm, chan_config[i], false);
        pio_spi_odm_inst_do_tx(&pgm, ADS1X4S0X_COMMAND_START, false);
        pio_spi_odm_inst_do_wait(&pgm);
        pio_spi_odm_inst_do_tx(&pgm, ADS1X4S0X_COMMAND_RDATA, false);
        pio_spi_odm_inst_do_tx(&pgm, 0x00, true);
        pio_spi_odm_inst_do_tx(&pgm, 0x00, true);
        pio_spi_odm_inst_do_tx(&pgm, 0x00, true);
        pio_spi_odm_inst_do_tx(&pgm, 0x00, true);
    }

    size_t raw_pio_insts_cnt = pgm.iptr + 1;

    raw_pio_insts[pgm.iptr] |= 0x06;

    printf("Total insts: %d = (%d bytes)\n", 2 * pgm.iptr + pgm.half, raw_pio_insts_cnt);
    for (int i = 0; i < raw_pio_insts_cnt; ++i) {
        printf("%d:     0x%02x\n", i, raw_pio_insts[i]);
    }
    printf("End.\n");

    uint8_t rx_buf[4*num_of_chan];

    while (true) {
        gpio_put(ADS1X4S0X_CS_PIN, 0);
        sleep_ms(1);
        pio_spi_odm_write_read(&spi, &pgm, rx_buf, true);

        gpio_put(ADS1X4S0X_CS_PIN, 1);

        for (int i = 0; i < num_of_chan; ++i) {
            sample = (int32_t)sys_get_be32(&rx_buf[i*4]) >> (32 - ADS1X4S0X_RESOLUTION);
            int64_t my_val_mv = (int64_t)sample * 2500 >> 23;
            float my_fval_mv = 2500.0 * sample / (1 << 23);

            printf("%d: Read 0x%06x = %d = %d mV = %.3f mV\n", i, sample, sample, my_val_mv, my_fval_mv);
        }
        sleep_ms(1000);
    }

#endif
}
