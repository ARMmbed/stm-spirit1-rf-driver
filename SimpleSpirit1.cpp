/*** Mbed Includes ***/
#include "SimpleSpirit1.h"


/*** Macros from Cube Implementation ***/
#define CLEAR_TXBUF()           (spirit_txbuf[0] = 0)
#define CLEAR_RXBUF()           (spirit_rxbuf[0] = 0)
/* transceiver state. */
#define ON     0
#define OFF    1


SimpleSpirit1 *SimpleSpirit1::_singleton = NULL;

/*** Class Implementation ***/
SimpleSpirit1::SimpleSpirit1(PinName mosi, PinName miso, PinName sclk,
			     PinName irq, PinName cs, PinName sdn,
			     PinName led) :
    _spi(mosi, miso, sclk),
    _irq(irq),
    _chip_select(cs),
    _shut_down(sdn),
    _led(led),
    _nr_of_irq_disables(0)
{
    /* reset irq disable counter & disable irq */
    disable_irq();

    /* unselect chip */
    chip_unselect();

    /* configure spi */
    _spi.format(8, 0); /* 8-bit, mode = 0, [order = SPI_MSB] only available in mbed3 */
    _spi.frequency(5000000); // 5MHz

    /* set frequencies */
    radio_set_xtal_freq(XTAL_FREQUENCY);
    mgmt_set_freq_base(XTAL_FREQUENCY);

    /* restart board */
    enter_shutdown();
    exit_shutdown();

    /* soft core reset */
    cmd_strobe_command(SPIRIT1_STROBE_SRES);

    /* Configures the SPIRIT1 radio part */
    SRadioInit x_radio_init = {
      XTAL_OFFSET_PPM,
      (uint32_t)BASE_FREQUENCY,
	  (uint32_t)CHANNEL_SPACE,
      CHANNEL_NUMBER,
      MODULATION_SELECT,
      DATARATE,
      (uint32_t)FREQ_DEVIATION,
      (uint32_t)BANDWIDTH
    };
    radio_init(&x_radio_init);
    radio_set_pa_level_dbm(0,POWER_DBM);
    radio_set_pa_level_max_index(0);

    /* Configures the SPIRIT1 packet handler part*/
    PktBasicInit x_basic_init = {
      PREAMBLE_LENGTH,
      SYNC_LENGTH,
      SYNC_WORD,
      LENGTH_TYPE,
      LENGTH_WIDTH,
      CRC_MODE,
      CONTROL_LENGTH,
      EN_ADDRESS,
      EN_FEC,
      EN_WHITENING
    };
    pkt_basic_init(&x_basic_init);

    /* Enable the following interrupt sources, routed to GPIO */
    irq_de_init(NULL);
    irq_clear_status();
    irq_set_status(TX_DATA_SENT, S_ENABLE);
    irq_set_status(RX_DATA_READY,S_ENABLE);
    irq_set_status(VALID_SYNC,S_ENABLE);
    irq_set_status(RX_DATA_DISC, S_ENABLE);
    irq_set_status(TX_FIFO_ERROR, S_ENABLE);
    irq_set_status(RX_FIFO_ERROR, S_ENABLE);

    /* Configure Spirit1 */
    radio_persisten_rx(S_ENABLE);
    qi_set_sqi_threshold(SQI_TH_0);
    qi_sqi_check(S_ENABLE);
    qi_set_rssi_threshold_dbm(CCA_THRESHOLD);
    timer_set_rx_timeout_stop_condition(SQI_ABOVE_THRESHOLD);
    timer_set_infinite_rx_timeout();
    radio_afc_freeze_on_sync(S_ENABLE);

    /* Puts the SPIRIT1 in STANDBY mode (125us -> rx/tx) */
    cmd_strobe_command(SPIRIT1_STROBE_STANDBY);
    spirit_on = OFF;
    CLEAR_RXBUF();
    CLEAR_TXBUF();

    /* Configure the radio to route the IRQ signal to its GPIO 3 */
    SGpioInit x_gpio_init = {
	SPIRIT_GPIO_IRQ,
	SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
	SPIRIT_GPIO_DIG_OUT_IRQ
    };
    spirit_gpio_init(&x_gpio_init);
}
