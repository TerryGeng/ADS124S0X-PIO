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

struct ads124s0x_config {
    uint8_t cs_pin;
    uint8_t sck_pin;
    uint8_t tx_pin;
    uint8_t rx_pin;
    uint8_t drdy_pin;
    uint8_t data_rate;
    bool pio_init;
};

static inline uint16_t sys_get_be16(const uint8_t src[2])
{
	return ((uint16_t)src[0] << 8) | src[1];
}

static inline uint32_t sys_get_be32(const uint8_t src[4])
{
	return ((uint32_t)sys_get_be16(&src[0]) << 16) | sys_get_be16(&src[2]);
}

void ads124s06_send_command_pio(const struct pio_spi_odm_config *spi_odm, uint8_t cmd);

void ads124s06_read_register_pio(const struct pio_spi_odm_config *spi_odm,
        uint8_t register_address, uint8_t *buf);

void ads124s06_write_register_pio(const struct pio_spi_odm_config *spi_odm,
        uint8_t register_address, uint8_t value);

void ads124s06_init(struct pio_spi_odm_config *spi_odm, struct ads124s0x_config *ads_cfg);

void ads124s06_make_pio_spi_odm_pgm(struct pio_spi_odm_raw_program *pgm,
        uint8_t *chan_cfg, uint8_t chan_cnt);
