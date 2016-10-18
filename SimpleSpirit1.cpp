/*** Mbed Includes ***/
#include "SimpleSpirit1.h"
#include "radio_spi.h"


/*** Macros from Cube Implementation ***/
#define CLEAR_TXBUF()			(spirit_tx_len = 0)
#define CLEAR_RXBUF()			(spirit_rx_len = 0)
#define IS_RXBUF_EMPTY()        (spirit_rx_len == 0)

#ifndef NDEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if NULLRDC_CONF_802154_AUTOACK
#define ACK_LEN 3
static int wants_an_ack = 0; /* The packet sent expects an ack */
//#define ACKPRINTF printf
#define ACKPRINTF(...)
#endif /* NULLRDC_CONF_802154_AUTOACK */

#define SPIRIT_GPIO_IRQ			(SPIRIT_GPIO_3)

#define SPIRIT1_STATUS()		(arch_refresh_status() & SPIRIT1_STATE_STATEBITS)

#define SABORT_WAIT_US			(400)

#define BUSYWAIT_UNTIL(cond, millisecs)                                        					\
  do {                                                                 					 		\
    uint32_t start = _busywait_timer.read_us();                         							\
    while (!(cond) && ((((uint32_t)(_busywait_timer.read_us())) - start) < ((uint32_t)millisecs)*1000U));	\
  } while(0)

extern volatile SpiritStatus 	g_xStatus;
#define st_lib_g_x_status	 	(g_xStatus)

#define st_lib_spirit_irqs		SpiritIrqs


/*** Class Implementation ***/
/** Static Class Variables **/
SimpleSpirit1 *SimpleSpirit1::_singleton = NULL;
Timer SimpleSpirit1::_busywait_timer;

/** Constructor **/
SimpleSpirit1::SimpleSpirit1(PinName mosi, PinName miso, PinName sclk,
			     PinName irq, PinName cs, PinName sdn,
			     PinName led) :
    _spi(mosi, miso, sclk),
    _irq(irq),
    _chip_select(cs),
    _shut_down(sdn),
    _led(led),
	_current_irq_callback()
{
    /* reset irq disable counter and irq callback & disable irq */
	_nr_of_irq_disables = 0;
    disable_spirit_irq();

    /* unselect chip */
    chip_unselect();

    /* configure spi */
    _spi.format(8, 0); /* 8-bit, mode = 0, [order = SPI_MSB] only available in mbed3 */
    _spi.frequency(5000000); // 5MHz

    /* start timer */
    _busywait_timer.start();

    /* init cube vars */
    spirit_on = OFF;
    packet_is_prepared = 0;
    receiving_packet = 0;
    just_got_an_ack = 0;
    last_rssi = 0 ; //MGR
    last_lqi = 0 ;  //MGR
}

/** Init Function **/
void SimpleSpirit1::init() {
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
    cmd_strobe(SPIRIT1_STROBE_STANDBY);
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

/** Prepare the radio with a packet to be sent. **/
int SimpleSpirit1::prepare(const void *payload, unsigned short payload_len) {
	PRINTF("Spirit1: prep %u\n", payload_len);
	packet_is_prepared = 0;

	/* Checks if the payload length is supported */
	if(payload_len > MAX_PACKET_LEN) {
		return RADIO_TX_ERR;
	}

	/* Should we delay for an ack? */
#if NULLRDC_CONF_802154_AUTOACK
	frame802154_t info154;
	wants_an_ack = 0;
	if(payload_len > ACK_LEN
			&& frame802154_parse((char*)payload, payload_len, &info154) != 0) {
		if(info154.fcf.frame_type == FRAME802154_DATAFRAME
				&& info154.fcf.ack_required != 0) {
			wants_an_ack = 1;
		}
	}
#endif /* NULLRDC_CONF_802154_AUTOACK */

	/* Sets the length of the packet to send */
	disable_spirit_irq();
	cmd_strobe(SPIRIT1_STROBE_FTX);
	pkt_basic_set_payload_length(payload_len);
	spi_write_linear_fifo(payload_len, (uint8_t *)payload);
	enable_spirit_irq();

	PRINTF("PREPARE OUT\n");

	packet_is_prepared = 1;
	return RADIO_TX_OK;
}

/** Send the packet that has previously been prepared. **/
int SimpleSpirit1::transmit(unsigned short payload_len) {
	/* This function blocks until the packet has been transmitted */
	//rtimer_clock_t rtimer_txdone, rtimer_rxack;

	PRINTF("TRANSMIT IN\n");
	if(!packet_is_prepared) {
		return RADIO_TX_ERR;
	}

	/* Stores the length of the packet to send */
	/* Others spirit_radio_prepare will be in hold */
	spirit_tx_len = payload_len;

	/* Puts the SPIRIT1 in TX state */
	receiving_packet = 0;
	set_ready_state();
	cmd_strobe(SPIRIT1_STROBE_TX);
	just_got_an_ack = 0;
	BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_TX, 1);
	//BUSYWAIT_UNTIL(SPIRIT1_STATUS() != SPIRIT1_STATE_TX, 4); //For GFSK with high data rate
	BUSYWAIT_UNTIL(SPIRIT1_STATUS() != SPIRIT1_STATE_TX, 50); //For FSK with low data rate

	/* Reset radio - needed for immediate RX of ack */
	CLEAR_TXBUF();
	CLEAR_RXBUF();
	disable_spirit_irq();
	irq_clear_status();
	cmd_strobe(SPIRIT1_STROBE_SABORT);
	wait_us(SABORT_WAIT_US);
	cmd_strobe(SPIRIT1_STROBE_READY);
	BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_READY, 1);
	cmd_strobe(SPIRIT1_STROBE_RX);
	BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, 1);
	enable_spirit_irq();

#if XXX_ACK_WORKAROUND
	just_got_an_ack = 1;
#endif /* XXX_ACK_WORKAROUND */

#if NULLRDC_CONF_802154_AUTOACK
	if (wants_an_ack) {
		rtimer_txdone = _busywait_timer.read_us();
		BUSYWAIT_UNTIL(just_got_an_ack, 2);
		rtimer_rxack = _busywait_timer.read_us();

		if(just_got_an_ack) {
			ACKPRINTF("debug_ack: ack received after %u us\n",
					(uint32_t)(rtimer_rxack - rtimer_txdone));
		} else {
			ACKPRINTF("debug_ack: no ack received\n");
		}
	}
#endif /* NULLRDC_CONF_802154_AUTOACK */

	PRINTF("TRANSMIT OUT\n");

	CLEAR_TXBUF();

	packet_is_prepared = 0;

	wait_us(1);

	return RADIO_TX_OK;
}

/** Set Ready State **/
void SimpleSpirit1::set_ready_state(void) {
  PRINTF("READY IN\n");

  irq_clear_status();
  disable_spirit_irq();

  if(SPIRIT1_STATUS() == SPIRIT1_STATE_STANDBY) {
	  cmd_strobe(SPIRIT1_STROBE_READY);
  } else if(SPIRIT1_STATUS() == SPIRIT1_STATE_RX) {
	  cmd_strobe(SPIRIT1_STROBE_SABORT);
	  irq_clear_status();
  }

  enable_spirit_irq();

  PRINTF("READY OUT\n");
}

int SimpleSpirit1::off(void) {
  PRINTF("Spirit1: ->off\n");
  if(spirit_on == ON) {
    /* Disables the mcu to get IRQ from the SPIRIT1 */
    disable_spirit_irq();

    /* first stop rx/tx */
    cmd_strobe(SPIRIT1_STROBE_SABORT);

    /* Clear any pending irqs */
    irq_clear_status();

    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_READY, 5);
    if(SPIRIT1_STATUS() != SPIRIT1_STATE_READY) {
      PRINTF("Spirit1: failed off->ready\n");
      return 1;
    }

    /* Puts the SPIRIT1 in STANDBY */
    cmd_strobe(SPIRIT1_STROBE_STANDBY);
    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_STANDBY, 5);
    if(SPIRIT1_STATUS() != SPIRIT1_STATE_STANDBY) {
      PRINTF("Spirit1: failed off->standby\n");
      return 1;
    }

    spirit_on = OFF;
    _nr_of_irq_disables = 1;
    CLEAR_TXBUF();
    CLEAR_RXBUF();
  }
  PRINTF("Spirit1: off.\n");
  return 0;
}

int SimpleSpirit1::on(void) {
  PRINTF("Spirit1: on\n");
  cmd_strobe(SPIRIT1_STROBE_SABORT);
  wait_us(SABORT_WAIT_US);
  if(spirit_on == OFF) {
    /* ensure we are in READY state as we go from there to Rx */
    cmd_strobe(SPIRIT1_STROBE_READY);
    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_READY, 5);
    if(SPIRIT1_STATUS() != SPIRIT1_STATE_READY) {
      PRINTF("Spirit1: failed to turn on\n");
	  while(1);
      //return 1;
    }

    /* now we go to Rx */
    cmd_strobe(SPIRIT1_STROBE_RX);
    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, 5);
    if(SPIRIT1_STATUS() != SPIRIT1_STATE_RX) {
      PRINTF("Spirit1: failed to enter rx\n");
	  while(1);
      //return 1;
    }

    /* Enables the mcu to get IRQ from the SPIRIT1 */
    spirit_on = ON;
    if(_current_irq_callback) {
    	MBED_ASSERT(_nr_of_irq_disables == 1);
    	enable_spirit_irq();
    }
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

int SimpleSpirit1::read(void *buf, unsigned short bufsize)
{
  PRINTF("READ IN\n");

  /* Checks if the RX buffer is empty */
  if(IS_RXBUF_EMPTY()) {
    disable_spirit_irq();
    CLEAR_RXBUF();
    cmd_strobe(SPIRIT1_STROBE_SABORT);
    wait_us(SABORT_WAIT_US);
    cmd_strobe(SPIRIT1_STROBE_READY);
    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_READY, 1);
    cmd_strobe(SPIRIT1_STROBE_RX);
    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, 1);
    PRINTF("READ OUT RX BUF EMPTY\n");
    enable_spirit_irq();
    return 0;
  }

  if(bufsize < spirit_rx_len) {
    /* If buf has the correct size */
    PRINTF("TOO SMALL BUF\n");
    return 0;
  } else {
    /* Copies the packet received */
    memcpy(buf, spirit_rx_buf, spirit_rx_len);

#ifdef CONTIKI // betzw - TODO
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, last_rssi);        //MGR
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, last_lqi); //MGR
#endif

    bufsize = spirit_rx_len;
    CLEAR_RXBUF();

    PRINTF("READ OUT\n");

    return bufsize;
  }

}

/*---------------------------------------------------------------------------*/
int SimpleSpirit1::channel_clear(void)
{
  float rssi_value;
  /* Local variable used to memorize the SPIRIT1 state */
  uint8_t spirit_state = ON;

  PRINTF("CHANNEL CLEAR IN\n");

  if(spirit_on == OFF) {
    /* Wakes up the SPIRIT1 */
    on();
    spirit_state = OFF;
  }

  /*  */
  disable_spirit_irq();
  cmd_strobe(SPIRIT1_STROBE_SABORT);
  /*  SpiritCmdStrobeSabort();*/
  irq_clear_status();
  enable_spirit_irq();
  {
    uint32_t timeout = _busywait_timer.read_us() + 5000;
    do {
    	mgmt_refresh_status();
    } while((st_lib_g_x_status.MC_STATE != MC_STATE_READY) && (((uint32_t)_busywait_timer.read_us()) < timeout));
    if(st_lib_g_x_status.MC_STATE != MC_STATE_READY) {
      return 1;
    }
  }

  /* Stores the RSSI value */
  rssi_value = qi_get_rssi_dbm();

  /* Puts the SPIRIT1 in its previous state */
  if(spirit_state==OFF) {
    off();
  } else {
    cmd_strobe(SPIRIT1_STROBE_RX);
    /*    SpiritCmdStrobeRx();*/
    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, 5);
  }

  PRINTF("CHANNEL CLEAR OUT\n");

  /* Checks the RSSI value with the threshold */
  if(rssi_value<CCA_THRESHOLD) {
    return 0;
  } else {
    return 1;
  }
}

int SimpleSpirit1::incoming_packet(void)
{
  return receiving_packet;
}

int SimpleSpirit1::pending_packet(void)
{
  PRINTF("PENDING PACKET\n");
  return !IS_RXBUF_EMPTY();
}

/** Contiki Irq Callback **/
void SimpleSpirit1::ContikiIrqHandler() {
  st_lib_spirit_irqs x_irq_status;

  /* get interrupt source from radio */
  irq_get_status(&x_irq_status);
  irq_clear_status();

  if(x_irq_status.IRQ_RX_FIFO_ERROR) {
    receiving_packet = 0;
    cmd_strobe(SPIRIT1_STROBE_FRX);
    return;
  }

  if(x_irq_status.IRQ_TX_FIFO_ERROR) {
    receiving_packet = 0;
    cmd_strobe(SPIRIT1_STROBE_FTX);
    return;
  }

  /* The IRQ_VALID_SYNC is used to notify a new packet is coming */
  if(x_irq_status.IRQ_VALID_SYNC) {
    receiving_packet = 1;
  }

  /* The IRQ_TX_DATA_SENT notifies the packet received. Puts the SPIRIT1 in RX */
  if(x_irq_status.IRQ_TX_DATA_SENT) {
    cmd_strobe(SPIRIT1_STROBE_RX);
    /*    SpiritCmdStrobeRx();*/
    CLEAR_TXBUF();
    return;
  }

  /* The IRQ_RX_DATA_READY notifies a new packet arrived */
  if(x_irq_status.IRQ_RX_DATA_READY) {
    spi_read_linear_fifo(linear_fifo_read_num_elements_rx_fifo(), spirit_rx_buf);
    spirit_rx_len = pkt_basic_get_received_pkt_length();
    cmd_strobe(SPIRIT1_STROBE_FRX);

    last_rssi = qi_get_rssi(); //MGR
    last_lqi  = qi_get_lqi();  //MGR

    receiving_packet = 0;

#if NULLRDC_CONF_802154_AUTOACK
    if (spirit_rxbuf[0] == ACK_LEN) {
      /* For debugging purposes we assume this is an ack for us */
      just_got_an_ack = 1;
    }
#endif /* NULLRDC_CONF_802154_AUTOACK */

    /* betzw - TODO: call user callback */
    return;
  }

  if(x_irq_status.IRQ_RX_DATA_DISC)
  {
    /* RX command - to ensure the device will be ready for the next reception */
    if(x_irq_status.IRQ_RX_TIMEOUT) {
      cmd_strobe_flush_rx_fifo();
    }
  }
}
