#include "spi_odm.h"
#include "ads124s0x_pio.h"

void ads124s06_send_command_pio(
        const struct pio_spi_odm_config *spi_odm,
        uint8_t cmd) {
	uint8_t buf;

	pio_spi_odm_transceive(spi_odm, &cmd, &buf, 1);
}

void ads124s06_read_register_pio(
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

void ads124s06_write_register_pio(
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

void ads124s06_init(struct pio_spi_odm_config *spi_odm, struct ads124s0x_config *ads_cfg) {
    // Chip select is active-low, so we'll initialise it to a driven-high state

    if (!ads_cfg->pio_init) {
        uint8_t offset = pio_add_program_spi_cpha1_read_on_demand_program_with_trigger_pin(
                spi_odm->pio, ads_cfg->drdy_pin);

        pio_spi_read_on_demand_init(spi_odm->pio, spi_odm->sm, offset,
                8,       // 8 bits per SPI frame
                10,      // 1 MHz @ 125 clk_sys
                ads_cfg->sck_pin,
                ads_cfg->tx_pin,
                ads_cfg->rx_pin,
                ads_cfg->drdy_pin
                );

        ads_cfg->pio_init = true;
    }

    pio_sm_restart(spi_odm->pio, spi_odm->sm);

    gpio_init(ads_cfg->cs_pin);
    gpio_set_dir(ads_cfg->cs_pin, GPIO_OUT);
    gpio_put(ads_cfg->cs_pin, 0);

    ads124s06_send_command_pio(spi_odm, ADS1X4S0X_COMMAND_RESET);

    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_GPIOCON, 0x01);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_GPIODAT, 0xe1);

    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_PGA, 0x00);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_DATARATE, 0x30 | ads_cfg->data_rate);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_REF, 0x39);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_IDACMAG, 0x00);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_IDACMUX, 0xff);
    ads124s06_write_register_pio(spi_odm, ADS1X4S0X_REGISTER_VBIAS, 0x00);

    gpio_put(ads_cfg->cs_pin, 1);
}

void ads124s06_make_pio_spi_odm_pgm(struct pio_spi_odm_raw_program *pgm,
        uint8_t *chan_cfg, uint8_t chan_cnt) {
    for (int i = 0; i < chan_cnt; ++i) {
        pio_spi_odm_inst_do_tx_rx(pgm, ADS1X4S0X_COMMAND_WREG | ADS1X4S0X_REGISTER_INPMUX, false);
        pio_spi_odm_inst_do_tx_rx(pgm, 0x00, false);
        pio_spi_odm_inst_do_tx_rx(pgm, chan_cfg[i], false);

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


