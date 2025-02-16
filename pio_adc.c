/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/dma.h"
#include "spi_odm.h"

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

static inline uint16_t sys_get_be16(const uint8_t src[2])
{
	return ((uint16_t)src[0] << 8) | src[1];
}

static inline uint32_t sys_get_be32(const uint8_t src[4])
{
	return ((uint32_t)sys_get_be16(&src[0]) << 16) | sys_get_be16(&src[2]);
}

static void ads124s06_send_command_pio(
        const struct pio_spi_odm_config *spi_odm,
        uint8_t cmd) {
	uint8_t buf;

	pio_spi_odm_transceive(spi_odm, &cmd, &buf, 1);
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

static void ads124s06_write_register_pio(
        const struct pio_spi_odm_config *spi_odm,
        uint8_t register_address, uint8_t value) {
	uint8_t buffer_tx[3];
	uint8_t buffer_rx[3];

	buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_WREG) | ((uint8_t)register_address);
	/* write one register */
	buffer_tx[1] = 0x00;
	buffer_tx[2] = value;
	/* read one register */

	pio_spi_odm_transceive(spi_odm, buffer_tx, buffer_rx, 3);
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

static void init_ads124s06 (struct pio_spi_odm_config *spi_odm, uint8_t datarate) {
#if !defined(ADS1X4S0X_SCK_PIN) || !defined(ADS1X4S0X_TX_PIN) || !defined(ADS1X4S0X_RX_PIN) || !defined(ADS1X4S0X_CS_PIN)
#warning this example requires a board with SPI pins
    puts("SPI pins were not defined");
#else
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(ADS1X4S0X_CS_PIN);
    gpio_set_dir(ADS1X4S0X_CS_PIN, GPIO_OUT);
    gpio_put(ADS1X4S0X_CS_PIN, 0);

    ads124s06_send_command_pio(spi_odm, ADS1X4S0X_COMMAND_RESET);

    /*uint8_t ret;*/
    /*ads124s06_read_register(ADS1X4S0X_REGISTER_ID, &ret);*/
    /*printf("Chip ID is 0x%x\n", ret);*/
    /**/
    /*ads124s06_read_register(ADS1X4S0X_REGISTER_STATUS, &ret);*/
    /*printf("Chip STATUS is 0x%x\n", ret);*/

    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_GPIOCON, 0x01);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_GPIODAT, 0xe1);

    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_PGA, 0x00);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_DATARATE, 0x30 | datarate);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_REF, 0x39);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_IDACMAG, 0x00);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_IDACMUX, 0xff);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_VBIAS, 0x00);

    gpio_put(ADS1X4S0X_CS_PIN, 1);
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
    uint8_t ints1 = dma_hw->ints1;

    if (ints0) {
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

    if (ints1) {
        if (ints1 & (1u << spi_odm.dma_chan_rx1)) {
            dma_hw->ch[spi_odm.dma_chan_rx1].write_addr = (uint8_t *)rx_buf1;
            rx_buf1_ready = true;
        }
        if (ints1 & (1u << spi_odm.dma_chan_rx2)) {
            dma_hw->ch[spi_odm.dma_chan_rx2].write_addr = (uint8_t *)rx_buf2;
            rx_buf2_ready = true;
        }
        dma_hw->ints1 = ints1;
    }
}

char user_input[256] = {0};
size_t user_input_len = 0;
bool is_started = false;
bool verbose = true;
uint8_t datarate = 7;

int main() {
    stdio_init_all();
    stdio_uart_init();

    uint8_t offset = pio_add_program_spi_cpha1_read_on_demand_program_with_trigger_pin(spi_odm.pio, ADS1X4S0X_DRDY_PIN);
    printf("Loaded PIO program at %d\n", offset);

    pio_spi_read_on_demand_init(spi_odm.pio, spi_odm.sm, offset,
            8,       // 8 bits per SPI frame
            10,      // 1 MHz @ 125 clk_sys
            ADS1X4S0X_SCK_PIN,
            ADS1X4S0X_TX_PIN,
            ADS1X4S0X_RX_PIN,
            ADS1X4S0X_DRDY_PIN
            );

    init_ads124s06(&spi_odm, 7);

    pio_spi_odm_make_ads124s06_pgm(&pgm);

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

                pio_spi_read_on_demand_init(spi_odm.pio, spi_odm.sm, offset,
                        8,       // 8 bits per SPI frame
                        10,      // 1 MHz @ 125 clk_sys
                        ADS1X4S0X_SCK_PIN,
                        ADS1X4S0X_TX_PIN,
                        ADS1X4S0X_RX_PIN,
                        ADS1X4S0X_DRDY_PIN
                        );
                pio_sm_restart(spi_odm.pio, spi_odm.sm);

                init_ads124s06(&spi_odm, datarate);

                gpio_put(ADS1X4S0X_CS_PIN, 0);

                if (verbose) {
                    uint8_t ret;
                    ads124s06_read_register_pio(&spi_odm, ADS1X4S0X_REGISTER_ID, &ret);
                    printf("Chip ID is 0x%x\n", ret);
                }

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
