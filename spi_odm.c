#include "hardware/dma.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <string.h>

#include "spi_odm.h"

void pio_spi_odm_transceive(
        const struct pio_spi_odm_config *spi_odm,
        uint8_t *src,
        uint8_t *dst,
        size_t len) {

    io_rw_8 *txfifo = (io_rw_8 *) &spi_odm->pio->txf[spi_odm->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi_odm->pio->rxf[spi_odm->sm];

    uint8_t inst = 0;
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

                inst |= (1 << 2) | ((tx_byte >> (j-1)) & 1);

                if (!half) {
                    inst <<= 4;
                    half = true;

                } else {
                    half = false;
                    *txfifo = inst;
                    inst = 0;
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

void pio_spi_odm_inst_do_tx_rx(struct pio_spi_odm_raw_program *pgm,
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

void pio_spi_odm_dma_run_forever(struct pio_spi_odm_config *spi_odm,
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
    dma_irqn_set_channel_enabled(spi_odm->dma_chan_rx1 % 2, spi_odm->dma_chan_rx1, true);
    dma_irqn_set_channel_enabled(spi_odm->dma_chan_rx2 % 2, spi_odm->dma_chan_rx2, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, handler);
    irq_set_exclusive_handler(DMA_IRQ_1, handler);
    irq_set_enabled(DMA_IRQ_0, true);
    irq_set_enabled(DMA_IRQ_1, true);

    dma_start_channel_mask((1u << spi_odm->dma_chan_rx1) | (1u << spi_odm->dma_chan_tx_ctrl1));
}
