/*** Mbed Includes ***/
#include "mbed.h"


/*** Cube Includes ***/
#include "SPIRIT_Radio.h"
#include "SPIRIT_Management.h"
#include "SPIRIT_Commands.h"
#include "MCU_Interface.h"


/*** Contiki Lib Includes ***/
#include "spirit1.h"
#include "spirit1-config.h"
#include "spirit1-const.h"


/*** Missing Cube External Declarations ***/
extern "C" void SpiritManagementSetFrequencyBase(uint32_t);


/*** A Simple Spirit1 Class ***/
class SimpleSpirit1 { // NOTE: must be a singleton (due to mix of MBED/CUBE code)!!!
 protected:
	static SimpleSpirit1 *_singleton;

    /** Communication Interface Instance Variables **/
	SPI _spi; // betzw - NOTE: Arduino pins are valid only for NUCLEO-F401RE
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

    static Timer _busywait_timer;
    Callback<void()> *_current_irq_callback;

    /** Static Variables from Cube Implementation **/
    /*
     * The buffers which hold incoming data.
     * The +1 because of the first byte,
     * which will contain the length of the packet.
     */
    uint8_t spirit_rxbuf[MAX_PACKET_LEN+1];
    uint8_t spirit_txbuf[MAX_PACKET_LEN+1-SPIRIT_MAX_FIFO_LEN];
    volatile unsigned int spirit_on;
    volatile uint8_t receiving_packet;
    int packet_is_prepared;
    int just_got_an_ack;

    /** Low Level Instance Variables **/
    unsigned int _nr_of_irq_disables;

    /** Low Level Instance Methods **/
    void disable_irq(void) {
    	_irq.disable_irq();
    	_nr_of_irq_disables++;
    	MBED_ASSERT(_nr_of_irq_disables != 0);
    }

    void enable_irq(void) {
    	MBED_ASSERT(_nr_of_irq_disables > 0);
    	if(--_nr_of_irq_disables == 0)
    		_irq.enable_irq();
    }

    void chip_select() { _chip_select = 0; }
    void chip_unselect() { _chip_select = 1; }

    void enter_shutdown() { _shut_down = 1; }
    void exit_shutdown() {
    	_shut_down = 0;
    	wait_ms(2); // wait two milliseconds (to allow Spirit1 a proper boot-up sequence)
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

    void radio_persisten_rx(SpiritFunctionalState xNewState) {
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

    /** Management Instance Methods **/
    void mgmt_set_freq_base(uint32_t freq) {
    	SpiritManagementSetFrequencyBase(freq);
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

    /** Command Instance Methods**/
    void cmd_strobe_command(uint8_t cmd) {
    	SpiritCmdStrobeCommand((SpiritCmd)cmd);
    }

    /** SPI Instance Methods **/
    StatusBytes spi_write_linear_fifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
    	return SdkEvalSpiWriteFifo(cNbBytes, pcBuffer);
    }

    /** Internal Spirit Methods */
    void set_ready_state(void);
    uint16_t arch_refresh_status(void);

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
    	disable_irq();
    	chip_select();
    	cs_to_sclk_delay();
    }

    void chip_sync_unselect() {
    	chip_unselect();
    	enable_irq();
    }

    /** Init Instance Method **/
    void init(void);

    /** Constructor **/
    SimpleSpirit1(PinName mosi, PinName miso, PinName sclk,
		  PinName irq, PinName cs, PinName sdn,
		  PinName led);

    /** Destructor **/
    ~SimpleSpirit1(void); // should never be called!

public:
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

    static SimpleSpirit1& Instance() {
    	if(_singleton == NULL) {
    		error("SimpleSpirit1 must be created before used!\n");
    	}

    	return *_singleton;
    }

    /** Attach a function to be called when a SPI interrupt occurs
     *
     *  @param func A void() callback, or 0 to set as none
     *
     *  @note  Function 'func' will be executed in interrupt context!
     */
    void attach_irq(Callback<void()> func) {
    	_irq.fall(func);
    	_current_irq_callback = &func;
    }

    /** Prepare the radio with a packet to be sent. */
    int prepare(const void *payload, unsigned short payload_len);

    /** Send the packet that has previously been prepared. */
    int transmit(unsigned short payload_len);

    /** Switch Radio On/Off **/
    int radio_on(void);
    int radio_off(void);
};
