/*** Mbed Includes ***/
#include "SimpleSpirit1.h"
#include "radio_spi.h"

#define SPIRIT_GPIO_IRQ			(SPIRIT_GPIO_3)

static uint16_t last_state;
#define SPIRIT1_STATUS()		((last_state = (uint16_t)refresh_state()) & SPIRIT1_STATE_STATEBITS)

#define XO_ON                   (0x1)

#define BUSYWAIT_UNTIL(cond, millisecs) \
        do { \
            uint32_t start = us_ticker_read(); \
            uint32_t limit = (uint32_t)millisecs*1000U; \
            while (!(cond)) { \
                uint32_t now = us_ticker_read(); \
                if((now - start) > limit) break; \
            } \
        } while(0)

#define st_lib_spirit_irqs		SpiritIrqs

#define STATE_TIMEOUT           (100)

// betzw: switching force & back from standby is on some devices quite unstable
#define USE_STANDBY_STATE

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
    _spi.frequency(10000000); // 10MHz (i.e. max speed allowed for Spirit1)

    /* install irq handler */
    _irq.mode(PullUp);
    _irq.fall(Callback<void()>(this, &SimpleSpirit1::IrqHandler));

    /* init cube vars */
    spirit_on = OFF;
    last_rssi = 0 ; //MGR
    last_sqi = 0 ;  //MGR

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
    irq_set_status(VALID_SYNC, S_ENABLE);
    irq_set_status(TX_FIFO_ERROR, S_ENABLE);
    irq_set_status(RX_FIFO_ERROR, S_ENABLE);
#ifndef RX_FIFO_THR_WA
    irq_set_status(TX_FIFO_ALMOST_EMPTY, S_ENABLE);
    irq_set_status(RX_FIFO_ALMOST_FULL, S_ENABLE);
#endif // !RX_FIFO_THR_WA

    /* Configure Spirit1 */
    radio_persistent_rx(S_ENABLE);
    qi_set_sqi_threshold(SQI_TH_0);
    qi_sqi_check(S_ENABLE);
    qi_set_rssi_threshold_dbm(CCA_THRESHOLD);
    timer_set_rx_timeout_stop_condition(SQI_ABOVE_THRESHOLD);
    timer_set_infinite_rx_timeout();
    radio_afc_freeze_on_sync(S_ENABLE);
    calibration_rco(S_ENABLE);

    spirit_on = OFF;
    CLEAR_TXBUF();
    CLEAR_RXBUF();
    _spirit_tx_started = false;
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
            S_ENABLE,         // enable persistent mode
            TBIT_TIME_64,     // Tcca time
            TCCA_TIME_3,      // Lcca length
            5,                // max nr of backoffs (<8)
            1,                // BU counter seed
            8                 // BU prescaler
    };
    csma_ca_init(&x_csma_init);

#ifdef USE_STANDBY_STATE
    /* Puts the SPIRIT1 in STANDBY mode (125us -> rx/tx) */
    cmd_strobe(SPIRIT1_STROBE_STANDBY);
#endif // USE_STANDBY_STATE
}

static volatile int tx_fifo_remaining = 0;            // to be used in irq handler
static volatile int tx_buffer_pos = 0;                // to be used in irq handler
static const volatile uint8_t *tx_fifo_buffer = NULL; // to be used in irq handler
int SimpleSpirit1::send(const void *payload, unsigned int payload_len, bool use_csma_ca) {
    /* Checks if the payload length is supported */
    if(payload_len > MAX_PACKET_LEN) {
        return RADIO_TX_ERR;
    }

    disable_spirit_irq();

    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, STATE_TIMEOUT);
#ifndef NDEBUG
    if((last_state & SPIRIT1_STATE_STATEBITS) != SPIRIT1_STATE_RX) {
        debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
    }
#endif

    /* Reset State to Ready */
    set_ready_state();

    cmd_strobe(SPIRIT1_STROBE_FTX); // flush TX FIFO buffer

#ifndef NDEBUG
    debug_if(!(linear_fifo_read_num_elements_tx_fifo() == 0), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif

    pkt_basic_set_payload_length(payload_len); // set desired payload len

    if(use_csma_ca) {
        csma_ca_state(S_ENABLE); // enable CSMA/CA
    }

    /* Init buffer & number of bytes to be send */
    tx_fifo_remaining = payload_len;
    tx_fifo_buffer = (const uint8_t*)payload;

    int8_t fifo_available = SPIRIT_MAX_FIFO_LEN; // fill-up whole fifo
    int8_t to_send = (tx_fifo_remaining > fifo_available) ? fifo_available : tx_fifo_remaining;

    tx_fifo_remaining -= to_send;

    /* Fill FIFO Buffer */
    if(to_send > 0) {
        spi_write_linear_fifo(to_send, (uint8_t*)&tx_fifo_buffer[0]);
    }

    tx_buffer_pos = to_send;
    _spirit_tx_started = true;

    enable_spirit_irq();

    /* Start transmitting */
    cmd_strobe(SPIRIT1_STROBE_TX);

    while(tx_fifo_remaining != 0); // wait until not everything is yet send (evtl. by irq handler)

    BUSYWAIT_UNTIL(!_spirit_tx_started, STATE_TIMEOUT);
#ifdef HEAVY_DEBUG
    debug("\r\n%s (%d): state=%x, _spirit_tx_started=%d\r\n", __func__, __LINE__, SPIRIT1_STATUS()>>1, _spirit_tx_started);
#endif

    if(use_csma_ca) {
        csma_ca_state(S_DISABLE); // disable CSMA/CA
    }

    cmd_strobe(SPIRIT1_STROBE_RX); // Return to RX state

    disable_spirit_irq();
    if(_spirit_tx_started) { // in case of state timeout
        _spirit_tx_started = false;
        enable_spirit_irq();
        return RADIO_TX_ERR;
    } else {
        enable_spirit_irq();
        return RADIO_TX_OK;
    }
}

/** Set Ready State **/
void SimpleSpirit1::set_ready_state(void) {
    uint16_t state;

    disable_spirit_irq();

    _spirit_tx_started = false;
    _is_receiving = false;
    stop_rx_timeout();

    cmd_strobe(SPIRIT1_STROBE_FRX);
    CLEAR_RXBUF();
    CLEAR_TXBUF();

    state = SPIRIT1_STATUS();
    if(state == SPIRIT1_STATE_STANDBY) {
        cmd_strobe(SPIRIT1_STROBE_READY);
    } else if(state == SPIRIT1_STATE_RX) {
        cmd_strobe(SPIRIT1_STROBE_SABORT);
    } else if(state != SPIRIT1_STATE_READY) {
#ifndef NDEBUG
        debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, state>>1);
#endif
    }

    BUSYWAIT_UNTIL((SPIRIT1_STATUS() == SPIRIT1_STATE_READY) && ((last_state & XO_ON) == XO_ON), STATE_TIMEOUT);
    if(last_state != (SPIRIT1_STATE_READY | XO_ON)) {
        error("\r\nSpirit1: failed to become ready (%x) => pls. reset!\r\n", last_state);
        enable_spirit_irq();
        return;
    }

    irq_clear_status();

    enable_spirit_irq();
}

int SimpleSpirit1::off(void) {
    if(spirit_on == ON) {
        /* Disables the mcu to get IRQ from the SPIRIT1 */
        disable_spirit_irq();

        /* first stop rx/tx */
        set_ready_state();

#ifdef USE_STANDBY_STATE
        /* Puts the SPIRIT1 in STANDBY */
        cmd_strobe(SPIRIT1_STROBE_STANDBY);
        BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_STANDBY, STATE_TIMEOUT);
        if((last_state & SPIRIT1_STATE_STATEBITS) != SPIRIT1_STATE_STANDBY) {
            error("\r\nSpirit1: failed to enter standby (%x)\r\n", last_state>>1);
            return 1;
        }
#endif // USE_STANDBY_STATE

        spirit_on = OFF;
        _nr_of_irq_disables = 1;
    }
    return 0;
}

int SimpleSpirit1::on(void) {
    if(spirit_on == OFF) {
        set_ready_state();

        /* now we go to Rx */
        cmd_strobe(SPIRIT1_STROBE_RX);

        BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, STATE_TIMEOUT);
        if((last_state & SPIRIT1_STATE_STATEBITS) != SPIRIT1_STATE_RX) {
            error("\r\nSpirit1: failed to enter rx (%x) => retry\r\n", last_state>>1);
        }

        /* Enables the mcu to get IRQ from the SPIRIT1 */
        spirit_on = ON;
#ifndef NDEBUG
        debug_if(!(_nr_of_irq_disables == 1), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
        enable_spirit_irq();
    }

#ifndef NDEBUG
    if(SPIRIT1_STATUS() != SPIRIT1_STATE_RX) {
        debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
    }
#endif

    return 0;
}

uint8_t SimpleSpirit1::refresh_state(void) {
    uint8_t mcstate;

    SpiritSpiReadRegisters(MC_STATE0_BASE, 1, &mcstate);

    return mcstate;
}

int SimpleSpirit1::read(void *buf, unsigned int bufsize)
{
    disable_spirit_irq();

    /* Checks if the RX buffer is empty */
    if(IS_RXBUF_EMPTY()) {
#ifndef NDEBUG
        debug("\r\nBuffer is empty\r\n");
#endif
        set_ready_state();

        cmd_strobe(SPIRIT1_STROBE_RX);
        BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, STATE_TIMEOUT);
        enable_spirit_irq();
        return 0;
    }

    if(bufsize < spirit_rx_len) {
        enable_spirit_irq();

        /* If buf has the correct size */
#ifndef NDEBUG
        debug("\r\nTOO SMALL BUF\r\n");
#endif
        return 0;
    } else {
        /* Copies the packet received */
        memcpy(buf, spirit_rx_buf, spirit_rx_len);

        bufsize = spirit_rx_len;
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

#ifndef NDEBUG
    if(SPIRIT1_STATUS() != SPIRIT1_STATE_RX) {
        debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
    }
#endif

    disable_spirit_irq();

    /* Reset State to Ready */
    set_ready_state();

    /* Stores the RSSI value */
    rssi_value = qi_get_rssi_dbm();

    enable_spirit_irq();

    /* Puts the SPIRIT1 in its previous state */
    if(spirit_state==OFF) {
        off();
#ifndef NDEBUG
#ifdef USE_STANDBY_STATE
        if(SPIRIT1_STATUS() != SPIRIT1_STATE_STANDBY) {
#else
            if(SPIRIT1_STATUS() != SPIRIT1_STATE_READY) {
#endif
                debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
            }
#endif
        } else {
            disable_spirit_irq();

            set_ready_state();

            cmd_strobe(SPIRIT1_STROBE_RX);
            BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, STATE_TIMEOUT);
            if((last_state & SPIRIT1_STATE_STATEBITS) != SPIRIT1_STATE_RX) {
                error("\r\nSpirit1: (#2) failed to enter rx (%x) => retry\r\n", last_state>>1);
            }

            enable_spirit_irq();

#ifndef NDEBUG
            if(SPIRIT1_STATUS() != SPIRIT1_STATE_RX) {
                debug("\r\nAssert failed in: %s (%d): state=%x\r\n", __func__, __LINE__, last_state>>1);
            }
#endif
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
    /* betzw - TODO: use threaded interrupt handling when `MBED_CONF_RTOS_PRESENT` is defined (see `atmel-rf-driver`) */
    void SimpleSpirit1::IrqHandler() {
        st_lib_spirit_irqs x_irq_status;

        /* get interrupt source from radio */
        irq_get_status(&x_irq_status);

        /* The IRQ_TX_DATA_SENT notifies the packet has been sent. Puts the SPIRIT1 in RX */
        if(x_irq_status.IRQ_TX_DATA_SENT) { /* betzw - NOTE: MUST be handled before `IRQ_RX_DATA_READY` for Nanostack integration!
	                                                     Logically, Nanostack only expects the "DONE" after "SUCCESS" (if it gets
	                                                     DONE before SUCCESS, it assumes you're not going to bother to send SUCCESS).
         */
#ifdef DEBUG_IRQ
            uint32_t *tmp = (uint32_t*)&x_irq_status;
            debug_if(!((*tmp) & IRQ_TX_DATA_SENT_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
            debug_if(tx_fifo_remaining != 0, "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif

            if(_spirit_tx_started) {
                _spirit_tx_started = false;

                /* call user callback */
                if(_current_irq_callback) {
                    _current_irq_callback(TX_DONE);
                }
            }

            /* Disable handling of other TX flags */
            x_irq_status.IRQ_TX_FIFO_ALMOST_EMPTY = S_RESET;
            tx_fifo_buffer = NULL;
        }

#ifndef RX_FIFO_THR_WA
        /* The IRQ_TX_FIFO_ALMOST_EMPTY notifies an nearly empty TX fifo */
        if(x_irq_status.IRQ_TX_FIFO_ALMOST_EMPTY) {
#ifdef DEBUG_IRQ
            uint32_t *tmp = (uint32_t*)&x_irq_status;
            debug_if(!((*tmp) & IRQ_TX_FIFO_ALMOST_EMPTY_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
            debug_if(!_spirit_tx_started, "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
            debug_if(tx_fifo_buffer == NULL, "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif

            int8_t fifo_available = SPIRIT_MAX_FIFO_LEN/2; // fill-up half fifo
            int8_t to_send = (tx_fifo_remaining > fifo_available) ? fifo_available : tx_fifo_remaining;

            tx_fifo_remaining -= to_send;

            /* Fill FIFO Buffer */
            if(to_send > 0) {
                spi_write_linear_fifo(to_send, (uint8_t*)&tx_fifo_buffer[tx_buffer_pos]);
            }
            tx_buffer_pos += to_send;
        }
#endif // !RX_FIFO_THR_WA

        /* TX FIFO underflow/overflow error */
        if(x_irq_status.IRQ_TX_FIFO_ERROR) {
#ifdef DEBUG_IRQ
            uint32_t *tmp = (uint32_t*)&x_irq_status;
            debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
            debug_if(!((*tmp) & IRQ_TX_FIFO_ERROR_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
            if(_spirit_tx_started) {
                _spirit_tx_started = false;
                /* call user callback */
                if(_current_irq_callback) {
                    _current_irq_callback(TX_ERR);
                }
            }

            /* reset data still to be sent */
            tx_fifo_remaining = 0;
        }

        /* The IRQ_RX_DATA_READY notifies a new packet arrived */
        if(x_irq_status.IRQ_RX_DATA_READY) {
#ifdef DEBUG_IRQ
            uint32_t *tmp = (uint32_t*)&x_irq_status;
            debug_if(!((*tmp) & IRQ_RX_DATA_READY_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif

            if(!_is_receiving) { // spurious irq?!? (betzw: see comments on macro 'RX_FIFO_THR_WA'!)
#ifdef HEAVY_DEBUG
                debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif
            } else {
                _is_receiving = false; // Finished receiving
                stop_rx_timeout();

                spirit_rx_len = pkt_basic_get_received_pkt_length();

#ifdef DEBUG_IRQ
                debug_if(!(spirit_rx_len <= MAX_PACKET_LEN), "\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif

                if(spirit_rx_len <= MAX_PACKET_LEN) {
                    uint8_t to_receive = spirit_rx_len - _spirit_rx_pos;
                    if(to_receive > 0) {
                        spi_read_linear_fifo(to_receive, &spirit_rx_buf[_spirit_rx_pos]);
                        _spirit_rx_pos += to_receive;
                    }
                }

                cmd_strobe(SPIRIT1_STROBE_FRX);

                last_rssi = qi_get_rssi(); //MGR
                last_sqi  = qi_get_sqi();  //MGR

                /* call user callback */
                if((_spirit_rx_pos == spirit_rx_len) && _current_irq_callback) {
                    _current_irq_callback(RX_DONE);
                }

                /* Disable handling of other RX flags */
                x_irq_status.IRQ_RX_FIFO_ALMOST_FULL = S_RESET;
            }
        }

#ifndef RX_FIFO_THR_WA
        /* RX FIFO almost full */
        if(x_irq_status.IRQ_RX_FIFO_ALMOST_FULL) {
#ifdef DEBUG_IRQ
            uint32_t *tmp = (uint32_t*)&x_irq_status;
            debug_if(!((*tmp) & IRQ_RX_FIFO_ALMOST_FULL_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
            if(!_is_receiving) { // spurious irq?!?
#ifdef DEBUG_IRQ
                debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif
            } else {
                uint8_t fifo_available = linear_fifo_read_num_elements_rx_fifo();
                if((fifo_available + _spirit_rx_pos) <= MAX_PACKET_LEN) {
                    spi_read_linear_fifo(fifo_available, &spirit_rx_buf[_spirit_rx_pos]);
                    _spirit_rx_pos += fifo_available;
                } else {
#ifdef DEBUG_IRQ
                    debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif
                }
            }
        }
#endif // !RX_FIFO_THR_WA

        /* Reception errors */
        if((x_irq_status.IRQ_RX_FIFO_ERROR) || (x_irq_status.IRQ_RX_DATA_DISC)) {
#ifdef DEBUG_IRQ
            uint32_t *tmp = (uint32_t*)&x_irq_status;
            debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
            debug_if(!((*tmp) & (IRQ_RX_FIFO_ERROR_MASK | IRQ_RX_DATA_DISC_MASK)), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
            rx_timeout_handler();
            if(_spirit_tx_started) {
                _spirit_tx_started = false;
                /* call user callback */
                if(_current_irq_callback) {
                    _current_irq_callback(TX_ERR);
                }
            }
        }

        /* The IRQ_VALID_SYNC is used to notify a new packet is coming */
        if(x_irq_status.IRQ_VALID_SYNC) {
#ifdef DEBUG_IRQ
            uint32_t *tmp = (uint32_t*)&x_irq_status;
            debug_if(!((*tmp) & IRQ_VALID_SYNC_MASK), "\r\nAssert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
            /* betzw - NOTE: there is a race condition between Spirit1 receiving packets and
             *               the MCU trying to send a packet, which gets resolved in favor of
             *               sending.
             */
            if(_spirit_tx_started) {
#ifdef DEBUG_IRQ
                debug("\r\n%s (%d): irq=%x\r\n", __func__, __LINE__, *tmp);
#endif
            } else {
                _is_receiving = true;
                start_rx_timeout();
            }
        }
    }
