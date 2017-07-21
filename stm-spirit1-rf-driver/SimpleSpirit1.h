/*** Mbed Includes ***/
#include "mbed.h"
#include "mbed_debug.h"


/*** Cube Includes ***/
#include "SPIRIT_Radio.h"
#include "SPIRIT_Management.h"
#include "SPIRIT_Commands.h"
#include "MCU_Interface.h"


/*** Contiki Lib Includes ***/
#include "spirit1.h"
#include "spirit1-config.h"
#include "spirit1-const.h"


// betzw: enable beyond macro if you want debug messages also from IRQ handler
// #define DEBUG_IRQ


/*** Macros from Cube Implementation ***/
#define CLEAR_TXBUF()			(spirit_tx_len = 0)
#define IS_RXBUF_EMPTY()        (spirit_rx_len == 0)
#define CLEAR_RXBUF()			do { 					\
									spirit_rx_len = 0;	\
									_spirit_rx_pos = 0; \
								} while(0)


/*** Macros from Cube Implementation ***/
/* transceiver state. */
#define ON     0
#define OFF    1


/*** Macros for Spirit1 API ***/
/* max payload */
#define SPIRIT1_MAX_PAYLOAD     (MAX_PACKET_LEN)


/*** Missing Cube External Declarations ***/
extern "C" void SpiritManagementSetFrequencyBase(uint32_t);


/*** UnlockedSPI for Usage in IRQ context ***/
class UnlockedSPI : public SPI {
public:
    UnlockedSPI(PinName mosi, PinName miso, PinName sclk) :
        SPI(mosi, miso, sclk) { }
    virtual ~UnlockedSPI() {}
    virtual void lock() { }
    virtual void unlock() { }
};


/*** A Simple Spirit1 Class ***/
// NOTE: must be a singleton (due to mix of MBED/CUBE code)!!!
// NOTE: implementation is IRQ-save but (intentionally) NOT thread-safe!!!
/** Simple Spirit1 Class
 *
 * @Note Synchronization level: implementation is IRQ-save but (intentionally) NOT thread-safe!!!
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "SimpleSpirit1.h"
 *
 * static char send_buf[] = "Hello World!";
 *
 * static SimpleSpirit1 &myspirit = SimpleSpirit1::CreateInstance(D11, D12, D3, D9, D10, D2);
 *
 * static volatile bool tx_done_flag = false;
 *
 * static void callback_func(int event)
 * {
 *   if (event == SimpleSpirit1::TX_DONE) {
 *     tx_done_flag = true;
 *   }
 * }
 *
 * int main()
 * {
 *   myspirit.attach_irq_callback(callback_func);
 *   myspirit.on();
 *
 *   while(1)
 *   {
 *     size_t curr_len = strlen((const char*)send_buf);
 *     myspirit.send(send_buf, curr_len);
 *
 *     while(!tx_done_flag);
 *     tx_done_flag = false;
 *   }
 * }
 * @endcode
 */
class SimpleSpirit1 {
 protected:
	static SimpleSpirit1 *_singleton;

    /** Communication Interface Instance Variables **/
	UnlockedSPI _spi; // betzw - NOTE: Morpho/Zio pins are valid only for NUCLEO-F401RE
              	  	  // mosi: PA_7 (D11)
					  // miso: PA_6 (D12)
              	  	  // sclk: PB_3 (D3) or
              	  	  //       PA_5 (D13) (only in case you unmount R4 & mount R7,
              	  	  //                  (note: in this case you may not use LED1 on some platforms)
              	  	  // bits: 8-bit
              	  	  // mode: 0
              	  	  // ordr: MSB
              	  	  // freq: max 10MHz
    InterruptIn _irq; // PC_7 (D9) (falling)
    DigitalOut _chip_select; // PB_6 (D10) ('1' == chip unselected)
    DigitalOut _shut_down; // PA_10 (D2) ('1' == shut_down)
    DigitalOut _led; // PB_4 (D5) (optional)

    Callback<void(int)> _current_irq_callback;
    Timeout _rx_receiving_timeout;

    void rx_timeout_handler(void) {
    	set_ready_state();
	    cmd_strobe(SPIRIT1_STROBE_RX);
#ifdef DEBUG_IRQ
	    debug("\r\n%s (%d)\r\n", __func__, __LINE__);
#endif
    }

    void start_rx_timeout(void) {
    	_rx_receiving_timeout.attach_us(Callback<void()>(this, &SimpleSpirit1::rx_timeout_handler), 100 * 1000); // 100ms
    }

    void stop_rx_timeout(void) {
    	_rx_receiving_timeout.detach();
    }

    /** Static Variables from Cube Implementation **/
    /*
     * The buffers which hold incoming data.
     * The +1 because of the first byte,
     * which will contain the length of the packet.
     */
    volatile uint16_t spirit_tx_len;
    volatile bool _spirit_tx_started;
    volatile uint16_t spirit_rx_len;
    volatile uint16_t _spirit_rx_pos;
    volatile bool _spirit_rx_err;
    uint8_t spirit_rx_buf[MAX_PACKET_LEN];
    volatile bool _is_receiving;

    /** Status Variables from Cube Implementation **/
    unsigned int spirit_on;
    uint8_t last_rssi; //MGR
    uint8_t last_sqi;  //MGR

    /** Low Level Instance Variables **/
    unsigned int _nr_of_irq_disables;

    /** Low Level Instance Methods **/
    void disable_spirit_irq(void) {
    	_irq.disable_irq();
    	_nr_of_irq_disables++;
#ifndef NDEBUG
    	debug_if(_nr_of_irq_disables == 0, "\r\nassert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
    }

    void enable_spirit_irq(void) {
#ifndef NDEBUG
    	debug_if(_nr_of_irq_disables == 0, "\r\nassert failed in: %s (%d)\r\n", __func__, __LINE__);
#endif
    	if(--_nr_of_irq_disables == 0)
    		_irq.enable_irq();
    }

    void chip_select() { _chip_select = 0; }
    void chip_unselect() { _chip_select = 1; }

    void enter_shutdown() {
    	_shut_down = 1;
    	wait_ms(5); // wait 5 milliseconds (to allow Spirit1 to shut down)
    }

    void exit_shutdown() {
    	_shut_down = 0;
    	wait_ms(10); // wait 10 milliseconds (to allow Spirit1 a proper boot-up sequence)
    }

    void cs_to_sclk_delay(void) {
    	wait_us(1); // heuristic value
    }

    /**
     * @brief      Write and read a buffer to/from the SPI peripheral device at the same time
     *             in 8-bit data mode using synchronous SPI communication.
     * @param[in]  pBufferToWrite pointer to the buffer of data to send.
     * @param[out] pBufferToRead pointer to the buffer to read data into.
     * @param[in]  NumBytes number of bytes to read and write.
     * @retval     0 if ok.
     * @retval     -1 if data format error.
     * @note       When using the SPI in Interrupt-mode, remember to disable interrupts
     *             before calling this function and to enable them again after.
     */
    void spi_write_read(uint8_t* pBufferToWrite, uint8_t* pBufferToRead, uint16_t NumBytes)
    {
        /* Read and write data at the same time. */
    	for (int i = 0; i < NumBytes; i++) {
            pBufferToRead[i] = _spi.write(pBufferToWrite[i]);
    	}
    }

    /** Radio Instance Methods **/
    void radio_set_xtal_freq(uint32_t freq) {
    	SpiritRadioSetXtalFrequency(freq);
    }

    void radio_set_pa_level_dbm(uint8_t cIndex, float fPowerdBm) {
    	SpiritRadioSetPALeveldBm(cIndex, fPowerdBm);
    }

    void radio_set_pa_level_max_index(uint8_t cIndex) {
    	SpiritRadioSetPALevelMaxIndex(cIndex);
    }

    uint8_t radio_init(SRadioInit *init_struct) {
    	return SpiritRadioInit(init_struct);
    }

    void radio_persistent_rx(SpiritFunctionalState xNewState) {
    	SpiritRadioPersistenRx(xNewState);
    }

    void radio_afc_freeze_on_sync(SpiritFunctionalState xNewState) {
    	SpiritRadioAFCFreezeOnSync(xNewState);
    }

    /** Packet System Instance Methods **/
    void pkt_basic_init(PktBasicInit* pxPktBasicInit) {
    	SpiritPktBasicInit(pxPktBasicInit);
    }

    void pkt_basic_set_payload_length(uint16_t nPayloadLength) {
    	SpiritPktBasicSetPayloadLength(nPayloadLength);
    }

    uint16_t pkt_basic_get_received_pkt_length(void) {
    	return SpiritPktBasicGetReceivedPktLength();
    }

	/** IRQ Instance Methods **/
    void irq_de_init(SpiritIrqs* pxIrqInit) {
    	SpiritIrqDeInit(pxIrqInit);
    }

    void irq_clear_status(void) {
    	SpiritIrqClearStatus();
    }

    void irq_set_status(IrqList xIrq, SpiritFunctionalState xNewState) {
    	SpiritIrq(xIrq, xNewState);
    }

    void irq_get_status(SpiritIrqs* pxIrqStatus) {
    	SpiritIrqGetStatus(pxIrqStatus);
    }

    /** Management Instance Methods **/
    void mgmt_set_freq_base(uint32_t freq) {
    	SpiritManagementSetFrequencyBase(freq);
    }

    void mgmt_refresh_status(void) {
    	SpiritRefreshStatus();
    }

    /** Spirit GPIO Instance Methods **/
    void spirit_gpio_init(SGpioInit* pxGpioInitStruct) {
    	SpiritGpioInit(pxGpioInitStruct);
    }

	/** Qi Instance Methods **/
    void qi_set_sqi_threshold(SqiThreshold xSqiThr) {
    	SpiritQiSetSqiThreshold(xSqiThr);
    }

    void qi_sqi_check(SpiritFunctionalState xNewState) {
    	SpiritQiSqiCheck(xNewState);
    }

    void qi_set_rssi_threshold_dbm(int nDbmValue) {
    	SpiritQiSetRssiThresholddBm(nDbmValue);
    }

    float qi_get_rssi_dbm() {
    	last_rssi = qi_get_rssi();
    	return get_last_rssi_dbm();
    }

    uint8_t qi_get_rssi() {
    	return SpiritQiGetRssi();
    }

    uint8_t qi_get_sqi() {
    	return SpiritQiGetSqi();
    }

    /** Timer Instance Methods **/
    void timer_set_rx_timeout_stop_condition(RxTimeoutStopCondition xStopCondition) {
    	SpiritTimerSetRxTimeoutStopCondition(xStopCondition);
    }

    void timer_set_rx_timeout_counter(uint8_t cCounter) {
    	SpiritTimerSetRxTimeoutCounter(cCounter);
    }

    void timer_set_infinite_rx_timeout(void) {
    	timer_set_rx_timeout_counter(0);
    }

    /** CSMA/CA Instance Methods **/
    void csma_ca_state(SpiritFunctionalState xNewState) {
    	SpiritCsma(xNewState);
    }

    void csma_ca_init(CsmaInit* pxCsmaInit) {
    	csma_ca_state(S_DISABLE); // Disabled at init
    	SpiritCsmaInit(pxCsmaInit);
    	SpiritCsmaSeedReloadMode(S_DISABLE); // always disable seed reload
    }

    /** Command Instance Methods**/
    void cmd_strobe(uint8_t cmd) {
    	SpiritCmdStrobeCommand((SpiritCmd)cmd);
    }

    void cmd_strobe_flush_rx_fifo() {
    	SpiritCmdStrobeCommand(CMD_FLUSHRXFIFO);
    }

    /** SPI Instance Methods **/
    StatusBytes spi_write_linear_fifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
    	return SdkEvalSpiWriteFifo(cNbBytes, pcBuffer);
    }

    StatusBytes spi_read_linear_fifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
    	return SdkEvalSpiReadFifo(cNbBytes, pcBuffer);
    }

    /** Linear FIFO Instance Methods **/
    uint8_t linear_fifo_read_num_elements_rx_fifo(void) {
    	return SpiritLinearFifoReadNumElementsRxFifo();
    }

    uint8_t linear_fifo_read_num_elements_tx_fifo(void) {
    	return SpiritLinearFifoReadNumElementsTxFifo();
    }

    void linear_fifo_set_almost_full_thr_rx(uint8_t cThrRxFifo) {
    	SpiritLinearFifoSetAlmostFullThresholdRx(cThrRxFifo);
    }

    /** Calibration Instance Methods **/
    void calibration_rco(SpiritFunctionalState xNewState) {
    	SpiritCalibrationRco(xNewState);
    }

    /** Internal Spirit Methods */
    void set_ready_state(void);
    uint8_t refresh_state(void);

    /** Friend Functions **/
    friend StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
    friend StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
    friend StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode);
    friend StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
    friend StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);

    /** Sdk Instance Methods **/
    StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
    StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
    StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode);
    StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
    StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);

    /** Helper Instance Methods **/
    void chip_sync_select() {
    	disable_spirit_irq();
    	chip_select();
    	cs_to_sclk_delay();
    }

    void chip_sync_unselect() {
    	chip_unselect();
    	enable_spirit_irq();
    }

    /** Init Instance Method **/
    void init();

    /** Spirit Irq Callback */
    void IrqHandler();

    /** Constructor **/
    SimpleSpirit1(PinName mosi, PinName miso, PinName sclk,
		  PinName irq, PinName cs, PinName sdn,
		  PinName led);

    /** Destructor **/
    ~SimpleSpirit1(void); // should never be called!

public:
    enum {
    	RX_DONE,
    	TX_DONE,
		TX_ERR
    };

    /** Create singleton instance of 'SimpleSpirit1'
     *
     * @param mosi 'PinName' of mosi pin to use
     * @param miso 'PinName' of miso pin to use
     * @param sclk 'PinName' of clock pin to use
     * @param irq  'PinName' of interrupt pin to use
     * @param cs   'PinName' of chip-select pin pin to use
     * @param sdn  'PinName' of pin to use for device shutdown
     *
     * @returns     reference to singleton instance
     */
    static SimpleSpirit1& CreateInstance(PinName mosi, PinName miso, PinName sclk,
    		PinName irq, PinName cs, PinName sdn,
			PinName led = NC) {

    	if(_singleton == NULL) {
    		_singleton = new SimpleSpirit1(mosi, miso, sclk,
    				irq, cs, sdn, led);
    		_singleton->init();
    	} else {
    		error("SimpleSpirit1 singleton already created!\n");
    	}

    	return *_singleton;
    }

    /** Create singleton instance of 'SimpleSpirit1'
     *
     * @param mosi 'PinName' of mosi pin to use
     * @param miso 'PinName' of miso pin to use
     * @param sclk 'PinName' of clock pin to use
     * @param irq  'PinName' of interrupt pin to use
     * @param cs   'PinName' of chip-select pin pin to use
     * @param sdn  'PinName' of pin to use for device shutdown
     *
     * @returns     reference to singleton instance
     */
    static SimpleSpirit1& Instance() {
    	if(_singleton == NULL) {
    		error("SimpleSpirit1 must be created before used!\n");
    	}

    	return *_singleton;
    }

    /** Attach a function to be called by the Spirit Irq handler when an event has occurred
     *
     *  @param func A void(int) callback, or 0 to set as none
     *
     *  @note  Function 'func' will be executed in interrupt context!
     *  @note  Function 'func' will be call with either 'RX_DONE', 'TX_DONE', or 'TX_ERR' as parameter
     *         to indicate which event has occurred.
     */
    void attach_irq_callback(Callback<void(int)> func) {
    	_current_irq_callback = func;
    }

    /** Switch Radio On
     */
    int on(void);
    /** Switch Radio Off
     */
    int off(void);

    /** Set Channel
     */
    void set_channel(uint8_t channel) {
    	SpiritRadioSetChannel(channel);
    }

    /** Send a Buffer
     *
     * @param payload       pointer to buffer to be send
     * @param payload_len   length of payload buffer in bytes
     * @param use_csma_ca   should CSMA/CA be enabled for transmission
     *
     * @returns             zero in case of success, non-zero error code otherwise
     *
     * @note                the maximum payload size in bytes allowed is defined by macro 'SPIRIT1_MAX_PAYLOAD'
     */
    int send(const void *payload, unsigned int payload_len, bool use_csma_ca = true);

    /** Copy received data into buffer
     *
     * @param buf       pointer to buffer to be filled
     * @param bufsize   size of buffer
     *
     * @returns         number of bytes copied into the buffer
     *
     * @note            the buffer should be (at least) of size 'SPIRIT1_MAX_PAYLOAD' (in bytes).
     */
    int read(void *buf, unsigned int bufsize);

    /** Perform a Clear-Channel Assessment (CCA) to find out if there is a packet in the air or not.
     *
     * @returns  1 if packet has been seen.
     */
    int channel_clear(void);

    /** Check if the radio driver has just received a packet
     */
    int get_pending_packet(void);

    /** Is radio currently receiving
     */
    bool is_receiving(void) {
    	return _is_receiving;
    }

    /** Get latest value of RSSI (in dBm)
     */
    float get_last_rssi_dbm(void) {
    	get_last_rssi_raw();
		return (-120.0+((float)(last_rssi-20))/2);
    }

    /** Get latest value of RSSI (as Spirit1 raw value)
     */
    uint8_t get_last_rssi_raw(void) {
    	if(last_rssi == 0) {
    		last_rssi = qi_get_rssi();
    	}
    	return last_rssi;
    }

    /** Get latest value of LQI (scaled to 8-bit)
     */
    uint8_t get_last_sqi(void) {
    	const uint8_t max_sqi = 8 * ((SYNC_LENGTH>>1)+1);
    	if(last_sqi == 0) {
    		last_sqi = qi_get_sqi();
    	}
    	if(last_sqi > max_sqi) last_sqi = max_sqi;

    	return (last_sqi * 255 / max_sqi);
    }

    /** Reset Board
     */
    void reset_board() {
    	init();
    }
};
