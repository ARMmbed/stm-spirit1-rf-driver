/*** Mbed Includes ***/
#include "SimpleSpirit1.h"
#include "radio_spi.h"

#define SPIRIT_GPIO_IRQ			(SPIRIT_GPIO_3)

#define SPIRIT1_STATUS()		(arch_refresh_status() & SPIRIT1_STATE_STATEBITS)

#define BUSYWAIT_UNTIL(cond, millisecs)                                        					\
		do {                                                                 					 		\
			uint32_t start = us_ticker_read();                         							\
			while (!(cond) && ((us_ticker_read() - start) < ((uint32_t)millisecs)*1000U));	\
		} while(0)

extern volatile SpiritStatus 	g_xStatus;
#define st_lib_g_x_status	 	(g_xStatus)

#define st_lib_spirit_irqs		SpiritIrqs


/*** Class Implementation ***/
/** Static Class Variables **/
SimpleSpirit1 *SimpleSpirit1::_singleton = NULL;

/** Constructor **/
SimpleSpirit1::SimpleSpirit1(PinName mosi, PinName miso, PinName sclk,
		PinName irq, PinName cs, PinName sdn,
		PinName led) :
    		_spi(mosi, miso, sclk),
			_irq(irq),
			_chip_select(cs),
			_shut_down(sdn),
			_led(led),
			_current_irq_callback(),
			_rx_receiving_timeout()
{
}

/** Init Function **/
void SimpleSpirit1::init() {
	/* reset irq disable counter and irq callback & disable irq */
	_nr_of_irq_disables = 0;
	disable_spirit_irq();

	/* unselect chip */
	chip_unselect();

	/* configure spi */
	_spi.format(8, 0); /* 8-bit, mode = 0, [order = SPI_MSB] only available in mbed3 */
	_spi.frequency(5000000); // 5MHz

	/* install irq handler */
	_irq.mode(PullUp);
	_irq.fall(Callback<void()>(this, &SimpleSpirit1::IrqHandler));

	/* init cube vars */
	spirit_on = OFF;
	last_rssi = 0 ; //MGR
	last_lqi = 0 ;  //MGR

	/* set frequencies */
	radio_set_xtal_freq(XTAL_FREQUENCY);
	mgmt_set_freq_base((uint32_t)BASE_FREQUENCY);

	/* restart board */
	enter_shutdown();
	exit_shutdown();

	/* soft core reset */
	cmd_strobe(SPIRIT1_STROBE_SRES);

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
	irq_set_status(RX_DATA_DISC, S_ENABLE);
	irq_set_status(TX_FIFO_ERROR, S_ENABLE);
	irq_set_status(RX_FIFO_ERROR, S_ENABLE);
	irq_set_status(RX_FIFO_ALMOST_FULL, S_ENABLE);
	irq_set_status(VALID_SYNC, S_ENABLE);
	irq_set_status(MAX_BO_CCA_REACH, S_ENABLE);

	/* Configure Spirit1 */
	radio_persisten_rx(S_ENABLE);
	qi_set_sqi_threshold(SQI_TH_0);
	qi_sqi_check(S_ENABLE);
	qi_set_rssi_threshold_dbm(CCA_THRESHOLD);
	timer_set_rx_timeout_stop_condition(SQI_ABOVE_THRESHOLD);
	timer_set_infinite_rx_timeout();
	radio_afc_freeze_on_sync(S_ENABLE);

	/* Puts the SPIRIT1 in STANDBY mode (125us -> rx/tx) */
	cmd_strobe(SPIRIT1_STROBE_STANDBY);
	spirit_on = OFF;
	CLEAR_TXBUF();
	CLEAR_RXBUF();
	_spirit_tx_started = false;
	_spirit_rx_err = false;
	_is_receiving = false;

	/* Configure the radio to route the IRQ signal to its GPIO 3 */
	SGpioInit x_gpio_init = {
			SPIRIT_GPIO_IRQ,
			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
			SPIRIT_GPIO_DIG_OUT_IRQ
	};
	spirit_gpio_init(&x_gpio_init);

	/* Setup CSMA/CA */
	CsmaInit x_csma_init = {
			S_DISABLE,        // no persistent mode
			TBIT_TIME_64,     // Tcca time
			TCCA_TIME_3,      // Lcca length
			3,                // max nr of backoffs (<8)
			1,                // BU counter seed
			8                 // BU prescaler
	};
	csma_ca_init(&x_csma_init);

#ifdef RX_FIFO_THR_WA
	linear_fifo_set_almost_full_thr_rx(SPIRIT_MAX_FIFO_LEN-(MAX_PACKET_LEN+1));
#endif
}

int SimpleSpirit1::send(const void *payload, unsigned int payload_len) {
	/* Checks if the payload length is supported */
	if(payload_len > MAX_PACKET_LEN) {
		return RADIO_TX_ERR;
	}

	disable_spirit_irq();

	/* Reset State to Ready */
	set_ready_state();

	cmd_strobe(SPIRIT1_STROBE_FTX); // flush TX FIFO buffer
#ifndef NDEBUG
	debug_if(!(linear_fifo_read_num_elements_tx_fifo() == 0), "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif

	pkt_basic_set_payload_length(payload_len); // set desired payload len
	csma_ca_state(S_ENABLE); // enable CSMA/CA

	int i = 0;
	int remaining = payload_len;
	bool trigger_tx = true;
	do {
		uint8_t fifo_available = SPIRIT_MAX_FIFO_LEN - linear_fifo_read_num_elements_tx_fifo();
		uint8_t to_send = (remaining > fifo_available) ? fifo_available : remaining;
		const uint8_t *buffer = (const uint8_t*)payload;

		/* Fill FIFO Buffer */
		spi_write_linear_fifo(to_send, (uint8_t*)&buffer[i]);

		/* Start Transmit FIFO Buffer */
		if(trigger_tx) {
#ifndef NDEBUG
			debug_if(!(linear_fifo_read_num_elements_tx_fifo() == to_send), "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif
			cmd_strobe(SPIRIT1_STROBE_TX);
			trigger_tx = false;
		}

		i += to_send;
		remaining -= to_send;
	} while(remaining != 0);

	_spirit_tx_started = true;

	enable_spirit_irq();

	BUSYWAIT_UNTIL(SPIRIT1_STATUS() != SPIRIT1_STATE_TX, 50);
#ifndef NDEBUG
	// debug_if(!(linear_fifo_read_num_elements_tx_fifo() == 0), "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif

	return RADIO_TX_OK;
}

/** Set Ready State **/
void SimpleSpirit1::set_ready_state(void) {
	disable_spirit_irq();

	_is_receiving = false;
	stop_rx_timeout();

	if(SPIRIT1_STATUS() == SPIRIT1_STATE_STANDBY) {
		cmd_strobe(SPIRIT1_STROBE_READY);
	} else if(SPIRIT1_STATUS() == SPIRIT1_STATE_RX) {
		cmd_strobe(SPIRIT1_STROBE_SABORT);
	}
	irq_clear_status();

	enable_spirit_irq();
}

int SimpleSpirit1::off(void) {
	if(spirit_on == ON) {
		/* Disables the mcu to get IRQ from the SPIRIT1 */
		disable_spirit_irq();

		/* first stop rx/tx */
		cmd_strobe(SPIRIT1_STROBE_SABORT);

		/* Clear any pending irqs */
		irq_clear_status();

		BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_READY, 3000);
		if(SPIRIT1_STATUS() != SPIRIT1_STATE_READY) {
#ifndef NDEBUG
			debug("\n\rSpirit1: failed off->ready\n\r");
#endif
			return 1;
		}

		/* Puts the SPIRIT1 in STANDBY */
		cmd_strobe(SPIRIT1_STROBE_STANDBY);
		BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_STANDBY, 3000);
		if(SPIRIT1_STATUS() != SPIRIT1_STATE_STANDBY) {
#ifndef NDEBUG
			debug("\n\rSpirit1: failed off->standby\n\r");
#endif
			return 1;
		}

		spirit_on = OFF;
		_nr_of_irq_disables = 1;
		_spirit_tx_started = false;
		_is_receiving = false;
		stop_rx_timeout();

		CLEAR_TXBUF();
		CLEAR_RXBUF();
	}
	return 0;
}

int SimpleSpirit1::on(void) {
	cmd_strobe(SPIRIT1_STROBE_SABORT);
	if(spirit_on == OFF) {
		/* ensure we are in READY state as we go from there to Rx */
		cmd_strobe(SPIRIT1_STROBE_READY);
		BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_READY, 3000);
		if(SPIRIT1_STATUS() != SPIRIT1_STATE_READY) {
#ifndef NDEBUG
			debug("\n\rSpirit1: failed to turn on\n\r");
#endif
			return 1;
		}

		/* now we go to Rx */
		cmd_strobe(SPIRIT1_STROBE_FRX);
		cmd_strobe(SPIRIT1_STROBE_RX);
		BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, 3000);
		if(SPIRIT1_STATUS() != SPIRIT1_STATE_RX) {
#ifndef NDEBUG
			debug("\n\rSpirit1: failed to enter rx\n\r");
#endif
			return 1;
		}
		CLEAR_RXBUF();
		_spirit_rx_err = false;
		_is_receiving = false;
		stop_rx_timeout();

		/* Enables the mcu to get IRQ from the SPIRIT1 */
		spirit_on = ON;
#ifndef NDEBUG
		debug_if(!(_nr_of_irq_disables == 1), "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif
		enable_spirit_irq();
	}

	return 0;
}

uint16_t SimpleSpirit1::arch_refresh_status(void) {
	uint16_t mcstate;
	uint8_t header[2];
	header[0]=READ_HEADER;
	header[1]=MC_STATE1_BASE;

	/* Puts the SPI chip select low to start the transaction */
	chip_sync_select();

	/* Write the aHeader bytes and read the SPIRIT1 status bytes */
	mcstate = _spi.write(header[0]);
	mcstate = mcstate<<8;

	/* Write the aHeader bytes and read the SPIRIT1 status bytes */
	mcstate |= _spi.write(header[1]);

	/* Puts the SPI chip select high to end the transaction */
	chip_sync_unselect();

	return mcstate;
}

int SimpleSpirit1::read(void *buf, unsigned int bufsize)
{
	disable_spirit_irq();

	/* Checks if the RX buffer is empty */
	if(IS_RXBUF_EMPTY()) {
		CLEAR_RXBUF();
		cmd_strobe(SPIRIT1_STROBE_SABORT);
		cmd_strobe(SPIRIT1_STROBE_READY);
		BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_READY, 3000);
		cmd_strobe(SPIRIT1_STROBE_FRX);
		cmd_strobe(SPIRIT1_STROBE_RX);
		BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, 3000);
		_spirit_rx_err = false;
		_is_receiving = false;
		stop_rx_timeout();
		enable_spirit_irq();
		return 0;
	}

	if(bufsize < spirit_rx_len) {
		enable_spirit_irq();

		/* If buf has the correct size */
#ifndef NDEBUG
		debug("\n\rTOO SMALL BUF\n\r");
#endif
		return 0;
	} else {
		/* Copies the packet received */
		memcpy(buf, spirit_rx_buf, spirit_rx_len);

		bufsize = spirit_rx_len;
		_is_receiving = false;
		stop_rx_timeout();
		CLEAR_RXBUF();

		enable_spirit_irq();

		return bufsize;
	}

}

int SimpleSpirit1::channel_clear(void)
{
	float rssi_value;
	/* Local variable used to memorize the SPIRIT1 state */
	uint8_t spirit_state = ON;

	if(spirit_on == OFF) {
		/* Wakes up the SPIRIT1 */
		on();
		spirit_state = OFF;
	}

	disable_spirit_irq();

	/* Reset State to Ready */
	set_ready_state();
	{
		uint32_t timeout = us_ticker_read() + 5000;
		do {
			mgmt_refresh_status();
		} while((st_lib_g_x_status.MC_STATE != MC_STATE_READY) && (us_ticker_read() < timeout));
		if(st_lib_g_x_status.MC_STATE != MC_STATE_READY) {
			enable_spirit_irq();
#ifndef NDEBUG
			debug("\n\rSpirit1: channel clear failed\n\r");
#endif
			return 1;
		}
	}

	/* Stores the RSSI value */
	rssi_value = qi_get_rssi_dbm();

	enable_spirit_irq();

	/* Puts the SPIRIT1 in its previous state */
	if(spirit_state==OFF) {
		off();
	} else {
		disable_spirit_irq();
		cmd_strobe(SPIRIT1_STROBE_FRX);
		cmd_strobe(SPIRIT1_STROBE_RX);
		BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, 10);
		CLEAR_RXBUF();
		_spirit_rx_err = false;
		_is_receiving = false;
		stop_rx_timeout();
		enable_spirit_irq();
	}

	/* Checks the RSSI value with the threshold */
	if(rssi_value<CCA_THRESHOLD) {
		return 0;
	} else {
		return 1;
	}
}

int SimpleSpirit1::get_pending_packet(void)
{
	return !IS_RXBUF_EMPTY();
}

/** Spirit Irq Callback **/
void SimpleSpirit1::IrqHandler() {
	st_lib_spirit_irqs x_irq_status;

	/* get interrupt source from radio */
	irq_get_status(&x_irq_status);
	// betzw - WAS: irq_clear_status(); BUT already cleared by get status!

	/* Reception errors */
	if((x_irq_status.IRQ_RX_FIFO_ERROR) || (x_irq_status.IRQ_RX_DATA_DISC) || (x_irq_status.IRQ_RX_TIMEOUT)) {
#ifndef NDEBUG
		debug("\n\r%s (%d)", __func__, __LINE__);
#endif
		_spirit_rx_err = true;
		_is_receiving = false;
		CLEAR_RXBUF();
		cmd_strobe(SPIRIT1_STROBE_FRX);
		stop_rx_timeout();
		if(_spirit_tx_started) {
			_spirit_tx_started = false;
			CLEAR_TXBUF();
			/* call user callback */
			if(_current_irq_callback) {
				_current_irq_callback(TX_ERR);
			}
		}
	}

	/* Transmission error */
	if(x_irq_status.IRQ_TX_FIFO_ERROR) {
#ifndef NDEBUG
		debug("\n\r%s (%d)", __func__, __LINE__);
#endif
		csma_ca_state(S_DISABLE); // disable CSMA/CA
		cmd_strobe(SPIRIT1_STROBE_FTX);
		if(_spirit_tx_started) {
			_spirit_tx_started = false;
			CLEAR_TXBUF();
			/* call user callback */
			if(_current_irq_callback) {
				_current_irq_callback(TX_ERR);
			}
		}
	}

	/* The IRQ_VALID_SYNC is used to notify a new packet is coming */
	if(x_irq_status.IRQ_VALID_SYNC) {
		_is_receiving = true;
		_spirit_rx_err = false;
		CLEAR_RXBUF();
#ifndef NDEBUG
		debug_if(_spirit_tx_started, "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif
		start_rx_timeout();
	}

	/* The IRQ_TX_DATA_SENT notifies the packet received. Puts the SPIRIT1 in RX */
	if(x_irq_status.IRQ_TX_DATA_SENT) {
#ifndef NDEBUG
		debug_if(!_spirit_tx_started, "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif

		csma_ca_state(S_DISABLE); // disable CSMA/CA
		cmd_strobe(SPIRIT1_STROBE_FRX);
		cmd_strobe(SPIRIT1_STROBE_RX);
		CLEAR_TXBUF();
		CLEAR_RXBUF();
		_spirit_rx_err = false;
		_spirit_tx_started = false;

		/* call user callback */
		if(_current_irq_callback) {
			_current_irq_callback(TX_DONE);
		}
	}

	/* RX FIFO almost full */
	if(x_irq_status.IRQ_RX_FIFO_ALMOST_FULL) {
		if(_spirit_rx_err) {
			_is_receiving = false;
			cmd_strobe(SPIRIT1_STROBE_FRX);
			CLEAR_RXBUF();
			stop_rx_timeout();
		} else {
			uint8_t fifo_available = linear_fifo_read_num_elements_rx_fifo();
			unsigned int remaining = MAX_PACKET_LEN - _spirit_rx_pos;
			if(fifo_available > remaining) {
				_spirit_rx_err = true;
				_is_receiving = false;
				CLEAR_RXBUF();
				cmd_strobe(SPIRIT1_STROBE_FRX);
				stop_rx_timeout();
			} else {
				spi_read_linear_fifo(fifo_available, &spirit_rx_buf[_spirit_rx_pos]);
				_spirit_rx_pos += fifo_available;
				if(!_is_receiving) {
					_is_receiving = true;
					start_rx_timeout();
				}
			}
		}
	}

	/* The IRQ_RX_DATA_READY notifies a new packet arrived */
	if(x_irq_status.IRQ_RX_DATA_READY) {
		_is_receiving = false; // Finished receiving
		stop_rx_timeout();

		if(_spirit_rx_err) {
			_spirit_rx_err = false;
			CLEAR_RXBUF();
			cmd_strobe(SPIRIT1_STROBE_FRX);
		} else {
			spirit_rx_len = pkt_basic_get_received_pkt_length();
			unsigned int remaining = 0;
			// uint8_t fifo_available = 0; // betzw: optimized out in favor of a request less to SPIRIT device
			uint8_t to_receive = 0;

#ifndef NDEBUG
			debug_if(!(spirit_rx_len <= MAX_PACKET_LEN), "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif

			for(; _spirit_rx_pos < spirit_rx_len;) {
				remaining = spirit_rx_len - _spirit_rx_pos;
				// fifo_available = linear_fifo_read_num_elements_rx_fifo();  // betzw: optimized out in favor of a request less to SPIRIT device
				to_receive = remaining; // betzw - WAS: (remaining < fifo_available) ? remaining : fifo_available;
				if(to_receive > 0) {
					spi_read_linear_fifo(to_receive, &spirit_rx_buf[_spirit_rx_pos]);
					_spirit_rx_pos += to_receive;
				}
			}

			cmd_strobe(SPIRIT1_STROBE_FRX);

			last_rssi = qi_get_rssi(); //MGR
			last_lqi  = qi_get_lqi();  //MGR

			/* call user callback */
			if(_current_irq_callback) {
				_current_irq_callback(RX_DONE);
			}
		}
	}

	/* Max number of back-off during CCA */
	if(x_irq_status.IRQ_MAX_BO_CCA_REACH) {
#ifndef NDEBUG
		debug("\n\r%s (%d)", __func__, __LINE__);
#endif
		csma_ca_state(S_DISABLE); // disable CSMA/CA
		cmd_strobe(SPIRIT1_STROBE_FRX);
		cmd_strobe(SPIRIT1_STROBE_RX);
		CLEAR_TXBUF();
		CLEAR_RXBUF();
		_spirit_rx_err = false;
		_spirit_tx_started = false;

		/* call user callback */
		if(_current_irq_callback) {
			_current_irq_callback(TX_ERR);
		}

	}
}
