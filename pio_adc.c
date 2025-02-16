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
#include "ads124s0x_pio.h"

#define ADS1X4S0X_CS_PIN        5
#define ADS1X4S0X_SCK_PIN       2
#define ADS1X4S0X_TX_PIN        3
#define ADS1X4S0X_RX_PIN        4
#define ADS1X4S0X_DRDY_PIN      6


struct pio_spi_odm_config spi_odm = {
    .pio = pio0,
    .sm = 0,
    .dma_chan_tx = 0,
    .dma_chan_tx_ctrl1 = 1,
    .dma_chan_tx_ctrl2 = 2,
    .dma_chan_rx1 = 3,
    .dma_chan_rx2 = 4,
};

struct ads124s0x_config ads_cfg = {
    .cs_pin = ADS1X4S0X_CS_PIN,
    .sck_pin = ADS1X4S0X_SCK_PIN,
    .tx_pin = ADS1X4S0X_TX_PIN,
    .rx_pin = ADS1X4S0X_RX_PIN,
    .drdy_pin = ADS1X4S0X_DRDY_PIN,
    .data_rate = 4,
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

    ads124s06_init(&spi_odm, &ads_cfg);

    ads124s06_make_pio_spi_odm_pgm(&pgm, chan_config, NUM_OF_CHAN);

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

                ads_cfg.data_rate = datarate;
                ads124s06_init(&spi_odm, &ads_cfg);

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
}
