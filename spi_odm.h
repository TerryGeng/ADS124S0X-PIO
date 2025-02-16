#include "hardware/pio.h"
#include "spi.pio.h"

struct pio_spi_odm_config {
    PIO pio;
    uint8_t sm;
    uint8_t cs_pin;
    uint8_t sck_pin;
    uint8_t tx_pin;
    uint8_t rx_pin;
    uint8_t trig_pin;
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


void pio_spi_odm_transceive(const struct pio_spi_odm_config *spi_odm,
        uint8_t *src, uint8_t *dst, size_t len);

void pio_spi_odm_write_read(const struct pio_spi_odm_config *spi_odm,
        struct pio_spi_odm_raw_program *pgm, uint8_t *dst);

void pio_spi_odm_write8_read8_blocking_dma(const struct pio_spi_odm_config *spi_odm,
        struct pio_spi_odm_raw_program *pgm, uint8_t *dst);

void pio_spi_odm_inst_do_tx_rx(struct pio_spi_odm_raw_program *pgm,
        uint8_t tx_byte, bool do_rx);

void pio_spi_odm_inst_do_wait(struct pio_spi_odm_raw_program *pgm);

void pio_spi_odm_inst_finalize(struct pio_spi_odm_raw_program *pgm);

void pio_spi_odm_print_pgm(struct pio_spi_odm_raw_program *pgm);

void pio_spi_odm_dma_run_forever(struct pio_spi_odm_config *spi_odm,
        struct pio_spi_odm_raw_program *pgm,
        irq_handler_t handler,
        uint8_t *rx_buf1, size_t rx_buf1_len,
        uint8_t *rx_buf2, size_t rx_buf2_len);
