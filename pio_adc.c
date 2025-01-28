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
#include "hardware/irq.h"
#include "spi.pio.h"

#define ADS1X4S0X_CS_PIN        5
#define ADS1X4S0X_SCK_PIN       2
#define ADS1X4S0X_TX_PIN        3
#define ADS1X4S0X_RX_PIN        4

#define ADS1X4S0X_DRDY_PIN      6

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

struct pio_spi_odm_config {
    PIO pio;
    uint8_t sm;
    uint8_t cs_pin;
    uint8_t dma_chan_tx;
    uint8_t dma_chan_rx1;
    uint8_t dma_chan_rx2;
    uint8_t dma_chan_tx_ctrl1;
    uint8_t dma_chan_tx_ctrl2;
};

struct pio_spi_odm_raw_program {
    uint8_t *raw_inst;
    size_t tx_cnt;
    size_t rx_cnt;
    size_t iptr;
    bool half;
};


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

#if defined(spi_default) && defined(ADS1X4S0X_CS_PIN)
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

	/*printf("ads124s06: read register 0x%02X (send 0x%02X%02X) value 0x%02X\n",*/
	/*           register_address,*/
	/*           buffer_tx[0],*/
	/*           buffer_tx[1],*/
	/*           buffer_rx[2]*/
	/*           );*/

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

void pio_spi_odm_transceive(
        const struct pio_spi_odm_config *spi_odm,
        uint8_t *src,
        uint8_t *dst,
        size_t len) {

    io_rw_8 *txfifo = (io_rw_8 *) &spi_odm->pio->txf[spi_odm->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi_odm->pio->rxf[spi_odm->sm];

    uint8_t tmp;
    uint8_t inst;
    uint8_t tx_byte;
    bool half;
    uint8_t tx_len;
    uint8_t rx_len;

    tx_len = len;
    rx_len = len;

    while (tx_len || rx_len) {
        if (tx_len) {
            tx_byte = *src;
            for (int j = 8; j > 0; j--) {
                while (pio_sm_is_tx_fifo_full(spi_odm->pio, spi_odm->sm)) {
                    ;
                }

                tmp = (1 << 2) | ((tx_byte >> (j-1)) & 1);

                if (!half) {
                    inst = tmp << 4;
                    half = true;

                } else {
                    inst |= tmp;
                    half = false;
                    *txfifo = inst;
                }
            }
            tx_len--;
            src++;
        }

        if (rx_len) {
            if (!pio_sm_is_rx_fifo_empty(spi_odm->pio, spi_odm->sm)) {
                *dst++ = *rxfifo;
                rx_len--;
            }
        }
    }
}

static void ads124s06_read_register_pio(
        const struct pio_spi_odm_config *spi_odm,
        uint8_t register_address, uint8_t *buf) {
	uint8_t buffer_tx[3];
	uint8_t buffer_rx[3];

	buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_RREG) | ((uint8_t)register_address);
	buffer_tx[1] = 0x00;
	buffer_tx[2] = 0x00;
	/* read one register */

	pio_spi_odm_transceive(spi_odm, buffer_tx, buffer_rx, 3);

	*buf = buffer_rx[2];
}

void pio_spi_odm_write_read(
        const struct pio_spi_odm_config *spi_odm,
        struct pio_spi_odm_raw_program *pgm,
        uint8_t *dst) {

    io_rw_8 *txfifo = (io_rw_8 *) &spi_odm->pio->txf[spi_odm->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi_odm->pio->rxf[spi_odm->sm];

    size_t write_len = pgm->tx_cnt;
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
        irq_handler_t handler,
        uint8_t *rx_buf1, size_t rx_buf1_len,
        uint8_t *rx_buf2, size_t rx_buf2_len) {

    dma_channel_config txcc1 = dma_channel_get_default_config(spi_odm->dma_chan_tx_ctrl1);
    dma_channel_config txcc2 = dma_channel_get_default_config(spi_odm->dma_chan_tx_ctrl2);
    dma_channel_config txc = dma_channel_get_default_config(spi_odm->dma_chan_tx);
    dma_channel_config rxc1 = dma_channel_get_default_config(spi_odm->dma_chan_rx1);
    dma_channel_config rxc2 = dma_channel_get_default_config(spi_odm->dma_chan_rx2);

    /* Config tx */

    channel_config_set_transfer_data_size(&txcc1, DMA_SIZE_32);
    channel_config_set_read_increment(&txcc1, false);
    channel_config_set_write_increment(&txcc1, false);
    channel_config_set_chain_to(&txcc1, spi_odm->dma_chan_tx_ctrl2);

    dma_channel_configure(
        spi_odm->dma_chan_tx_ctrl1,
        &txcc1,
        &dma_hw->ch[spi_odm->dma_chan_tx].al3_transfer_count,
        &pgm->tx_cnt,
        1,
        false
    );

    channel_config_set_transfer_data_size(&txcc2, DMA_SIZE_32);
    channel_config_set_read_increment(&txcc2, false);
    channel_config_set_write_increment(&txcc2, false);

    dma_channel_configure(
        spi_odm->dma_chan_tx_ctrl2,
        &txcc2,
        &dma_hw->ch[spi_odm->dma_chan_tx].al3_read_addr_trig,
        &pgm->raw_inst,
        1,
        false
    );

    channel_config_set_read_increment(&txc, true);
    channel_config_set_write_increment(&txc, false);
    channel_config_set_dreq(&txc, pio_get_dreq(spi_odm->pio, spi_odm->sm, true));
    channel_config_set_transfer_data_size(&txc, DMA_SIZE_8);
    channel_config_set_chain_to(&txc, spi_odm->dma_chan_tx_ctrl1);

    dma_channel_configure(spi_odm->dma_chan_tx, &txc, &spi_odm->pio->txf[spi_odm->sm], NULL, 0, false);

    /* Config rx */

    channel_config_set_read_increment(&rxc1, false);
    channel_config_set_write_increment(&rxc1, true);
    channel_config_set_dreq(&rxc1, pio_get_dreq(spi_odm->pio, spi_odm->sm, false));
    channel_config_set_transfer_data_size(&rxc1, DMA_SIZE_8);
    channel_config_set_chain_to(&rxc1, spi_odm->dma_chan_rx2);

    dma_channel_configure(spi_odm->dma_chan_rx1, &rxc1, rx_buf1, &spi_odm->pio->rxf[spi_odm->sm], rx_buf1_len, false);

    channel_config_set_read_increment(&rxc2, false);
    channel_config_set_write_increment(&rxc2, true);
    channel_config_set_dreq(&rxc2, pio_get_dreq(spi_odm->pio, spi_odm->sm, false));
    channel_config_set_transfer_data_size(&rxc2, DMA_SIZE_8);
    channel_config_set_chain_to(&rxc2, spi_odm->dma_chan_rx1);

    dma_channel_configure(spi_odm->dma_chan_rx2, &rxc2, rx_buf2, &spi_odm->pio->rxf[spi_odm->sm], rx_buf2_len, false);

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(spi_odm->dma_chan_rx1, true);
    dma_channel_set_irq0_enabled(spi_odm->dma_chan_rx2, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_start_channel_mask((1u << spi_odm->dma_chan_rx1) | (1u << spi_odm->dma_chan_tx_ctrl1));
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

void pio_spi_odm_inst_finalize(struct pio_spi_odm_raw_program *pgm) {
    if (pgm->half) {
        pgm->tx_cnt = pgm->iptr + 1;
        pgm->raw_inst[pgm->iptr] |= (1 << 3);  /* write nop bit */
    } else {
        pgm->tx_cnt = pgm->iptr;
    }
}

void pio_spi_odm_print_pgm(struct pio_spi_odm_raw_program *pgm) {
    size_t raw_pio_insts_cnt = pgm->tx_cnt;

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

struct multiple_reading_buf {
    uint32_t readings[NUM_OF_CHAN];
};

static void print_buffer(struct multiple_reading_buf *buf, size_t len, bool verbose) {
    int32_t sample;
    int64_t my_val_mv;
    float my_fval_mv;

    for (size_t j = 0; j < len; ++j) {
        if (!verbose) {
            printf("#%2d\n", j);
        }

        for (int i = 0; i < NUM_OF_CHAN; ++i) {
            sample = (int32_t)sys_get_be32((uint8_t *)&(buf[j].readings[i])) >> (32 - ADS1X4S0X_RESOLUTION);

            if (verbose) {
                my_val_mv = (int64_t)sample * 2500 >> 23;
                my_fval_mv = 2500.0 * sample / (1 << 23);
                printf("[%2d] %d: Read 0x%06x = %d = %lld mV = %.3f mV\n", j, i, sample, sample, my_val_mv, my_fval_mv);
            } else {
                printf("%d\n", sample);
            }
        }
    }
}

static void init_ads124s06 (uint8_t datarate) {
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

    /*uint8_t ret;*/
    /*ads124s06_read_register(ADS1X4S0X_REGISTER_ID, &ret);*/
    /*printf("Chip ID is 0x%x\n", ret);*/
    /**/
    /*ads124s06_read_register(ADS1X4S0X_REGISTER_STATUS, &ret);*/
    /*printf("Chip STATUS is 0x%x\n", ret);*/

    ads124s06_write_register(ADS1X4S0X_REGISTER_GPIOCON, 0x01);
    ads124s06_write_register(ADS1X4S0X_REGISTER_GPIODAT, 0xe1);

    ads124s06_write_register(ADS1X4S0X_REGISTER_PGA, 0x00);
    ads124s06_write_register(ADS1X4S0X_REGISTER_DATARATE, 0x30 | datarate);
    ads124s06_write_register(ADS1X4S0X_REGISTER_REF, 0x39);
    ads124s06_write_register(ADS1X4S0X_REGISTER_IDACMAG, 0x00);
    ads124s06_write_register(ADS1X4S0X_REGISTER_IDACMUX, 0xff);
    ads124s06_write_register(ADS1X4S0X_REGISTER_VBIAS, 0x00);
}

void static pio_spi_odm_make_ads124s06_pgm(struct pio_spi_odm_raw_program *pgm) {
    for (int i = 0; i < NUM_OF_CHAN; ++i) {
        pio_spi_odm_inst_do_tx_rx(pgm, ADS1X4S0X_COMMAND_WREG | ADS1X4S0X_REGISTER_INPMUX, false);
        pio_spi_odm_inst_do_tx_rx(pgm, 0x00, false);
        pio_spi_odm_inst_do_tx_rx(pgm, chan_config[i], false);

        pio_spi_odm_inst_do_tx_rx(pgm, ADS1X4S0X_COMMAND_START, false);
        pio_spi_odm_inst_do_wait(pgm);

        /*pio_spi_odm_inst_do_tx_rx(pgm, ADS1X4S0X_COMMAND_RDATA, false);*/
        pio_spi_odm_inst_do_tx_rx(pgm, 0x00, true);
        pio_spi_odm_inst_do_tx_rx(pgm, 0x00, true);
        pio_spi_odm_inst_do_tx_rx(pgm, 0x00, true);
        pio_spi_odm_inst_do_tx_rx(pgm, 0x00, true);
    }
    pio_spi_odm_inst_finalize(pgm);
}


static struct multiple_reading_buf rx_buf1[6];
static struct multiple_reading_buf rx_buf2[6];

bool rx_buf1_ready = false;
bool rx_buf2_ready = false;

static void __not_in_flash_func(dma_handler)() {
    uint8_t ints0 = dma_hw->ints0;

    if (ints0 & (1u << spi_odm.dma_chan_rx1)) {
        dma_hw->ch[spi_odm.dma_chan_rx1].write_addr = (uint8_t *)rx_buf1;
        rx_buf1_ready = true;
    }
    if (ints0 & (1u << spi_odm.dma_chan_rx2)) {
        dma_hw->ch[spi_odm.dma_chan_rx2].write_addr = (uint8_t *)rx_buf2;
        rx_buf2_ready = true;
    }

    dma_hw->ints0 = ints0;
}

char user_input[256] = {0};
size_t user_input_len = 0;
bool is_started = false;
bool verbose = true;
uint8_t datarate = 7;

int main() {
    stdio_init_all();
    stdio_uart_init();

    init_ads124s06(7);

    pio_spi_odm_make_ads124s06_pgm(&pgm);

    uint8_t offset = pio_add_program_spi_cpha1_read_on_demand_program_with_trigger_pin(spi_odm.pio, ADS1X4S0X_DRDY_PIN);
    printf("Loaded PIO program at %d\n", offset);

    printf("Compiled pio_spi_odm program:\n");
    pio_spi_odm_print_pgm(&pgm);

    int c;

    printf("Type `start` to start sampling: \n");
    while (true) {
        if (rx_buf1_ready) {
            rx_buf1_ready = false;
            print_buffer(rx_buf1, sizeof(rx_buf1)/sizeof(struct multiple_reading_buf), verbose);
        }

        if (rx_buf2_ready) {
            rx_buf2_ready = false;
            print_buffer(rx_buf2, sizeof(rx_buf2)/sizeof(struct multiple_reading_buf), verbose);
        }

        c = getchar_timeout_us(1000);

        if (c != -2 && c != '\r') {
            user_input[user_input_len] = c;
            user_input_len++;
            putchar(c);
        }

        if (user_input_len == 256) {
            memset(user_input, '\0', sizeof(user_input)/sizeof(char));
        }

        if (c == '\r') {
            printf("\n");
            if (strcmp(user_input, "start") == 0) {
                if (is_started) {
                    printf("started\n");
                    goto next;
                }
                is_started = true;

                init_ads124s06(datarate);
                gpio_put(ADS1X4S0X_CS_PIN, 0);

                pio_spi_read_on_demand_init(spi_odm.pio, spi_odm.sm, offset,
                        8,       // 8 bits per SPI frame
                        10,      // 1 MHz @ 125 clk_sys
                        ADS1X4S0X_SCK_PIN,
                        ADS1X4S0X_TX_PIN,
                        ADS1X4S0X_RX_PIN,
                        ADS1X4S0X_DRDY_PIN
                        );
                pio_sm_restart(spi_odm.pio, spi_odm.sm);

                uint8_t ret;
                ads124s06_read_register_pio(&spi_odm, ADS1X4S0X_REGISTER_ID, &ret);
                printf("Chip ID is 0x%x\n", ret);

                pio_spi_odm_dma_run_forever(&spi_odm, &pgm,
                        dma_handler,
                        (uint8_t *)rx_buf1,
                        sizeof(rx_buf1)/sizeof(uint8_t),
                        (uint8_t *)rx_buf2,
                        sizeof(rx_buf2)/sizeof(uint8_t)
                        );

                printf("started\n");
            } else if (strcmp(user_input, "stop") == 0) {
                if (!is_started) {
                    printf("stopped\n");
                    goto next;
                }
                is_started = false;
                hw_clear_bits(&dma_hw->ch[spi_odm.dma_chan_tx_ctrl1].ctrl_trig, DMA_CH0_CTRL_TRIG_EN_BITS);

                while (dma_hw->ch[spi_odm.dma_chan_tx].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) {
                    sleep_ms(1);
                }

                hw_clear_bits(&dma_hw->ch[spi_odm.dma_chan_rx1].ctrl_trig, DMA_CH0_CTRL_TRIG_EN_BITS);
                hw_clear_bits(&dma_hw->ch[spi_odm.dma_chan_rx2].ctrl_trig, DMA_CH0_CTRL_TRIG_EN_BITS);
                hw_set_bits(&dma_hw->abort, (1u << spi_odm.dma_chan_rx1) | (1u << spi_odm.dma_chan_rx2));

                gpio_put(ADS1X4S0X_CS_PIN, 1);

                printf("stopped\n");
            } else if (strcmp(user_input, "set_slow") == 0) {
                datarate = 4;
                printf("set slow\n");
            } else if (strcmp(user_input, "set_fast") == 0) {
                datarate = 11;
                printf("set fast\n");
            } else if (strcmp(user_input, "set_verbose") == 0) {
                verbose = true;
                printf("set verbose\n");
            } else if (strcmp(user_input, "unset_verbose") == 0) {
                verbose = false;
                printf("unset verbose\n");
            } else if (strcmp(user_input, "ping") == 0) {
                printf("pong\n");
            } else {
                printf("?\n");
            }

next:
            memset(user_input, '\0', sizeof(user_input)/sizeof(char));
            user_input_len = 0;
        }
    }

#endif
}
