#include "SimpleSpirit1.h"
#include "nanostack/platform/arm_hal_phy.h"

static uint8_t mac_address[8] = {
		MBED_CONF_SPIRIT1_MAC_ADDRESS_0,
		MBED_CONF_SPIRIT1_MAC_ADDRESS_1,
		MBED_CONF_SPIRIT1_MAC_ADDRESS_2,
		MBED_CONF_SPIRIT1_MAC_ADDRESS_3,
		MBED_CONF_SPIRIT1_MAC_ADDRESS_4,
		MBED_CONF_SPIRIT1_MAC_ADDRESS_5,
		MBED_CONF_SPIRIT1_MAC_ADDRESS_6,
		MBED_CONF_SPIRIT1_MAC_ADDRESS_7
};
static phy_device_driver_s device_driver;
static int8_t rf_radio_driver_id = -1;

const phy_rf_channel_configuration_s phy_subghz = {868000000, 1000000, 250000, 11, M_GFSK};

static phy_device_channel_page_s phy_channel_pages[] = {
    {CHANNEL_PAGE_2, &phy_subghz},
    {CHANNEL_PAGE_0, NULL}
};

static uint8_t tx_sequence = 0xff;
static uint8_t mac_tx_handle = 0;

static SimpleSpirit1 *rf_device = NULL;
static uint8_t rf_rx_buf[MAX_PACKET_LEN];

static int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol)
{
    /*Check if transmitter is busy*/
    if((rf_device->get_receiving_packet()) || (rf_device->channel_clear() == 0)) {
        /*Return busy*/
        return -1;
    } else {
        /*Store the sequence number for ACK handling*/
        tx_sequence = *(data_ptr + 2);

        /*Store TX handle*/
        mac_tx_handle = tx_handle;

        /*Send the packet*/
        rf_device->send(data_ptr, data_length);
    }

    /*Return success*/
    return 0;
}

static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel)
{
    int8_t ret_val = 0;
    switch (new_state)
    {
        /*Reset PHY driver and set to idle*/
        case PHY_INTERFACE_RESET:
        	rf_device->reset_board();
            break;
        /*Disable PHY Interface driver*/
        case PHY_INTERFACE_DOWN:
            ret_val = rf_device->off();
            if(ret_val != 0) ret_val = -1;
            break;
        /*Enable PHY Interface driver*/
        case PHY_INTERFACE_UP:
        	ret_val = rf_device->on();
        	if(ret_val != 0) {
        		ret_val = -1;
        		break;
        	}
            rf_device->set_channel(rf_channel);
            break;
        /*Enable wireless interface ED scan mode*/
        case PHY_INTERFACE_RX_ENERGY_STATE:
            break;
        /*Enable Sniffer state*/
        case PHY_INTERFACE_SNIFFER_STATE:
            // TODO - if we really need this - WAS: rf_setup_sniffer(rf_channel);
        	ret_val = -1;
            break;
    }
    return ret_val;
}

static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr)
{
    switch (extension_type)
    {
        /*Control MAC pending bit for Indirect data transmission*/
        case PHY_EXTENSION_CTRL_PENDING_BIT:
        /*Return frame pending status*/
        case PHY_EXTENSION_READ_LAST_ACK_PENDING_STATUS:
            // TODO: *data_ptr = rf_if_last_acked_pending();
            break;
        /*Set channel, used for setting channel for energy scan*/
        case PHY_EXTENSION_SET_CHANNEL:
            break;
        /*Read energy on the channel*/
        case PHY_EXTENSION_READ_CHANNEL_ENERGY:
            // TODO: *data_ptr = rf_get_channel_energy();
            break;
        /*Read status of the link*/
        case PHY_EXTENSION_READ_LINK_STATUS:
            // TODO: *data_ptr = rf_get_link_status();
            break;
        default:
        	break;
    }
    return 0;
}

#if 0 // TODO - if we really need this - WAS
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{

    switch (address_type)
    {
        /*Set 48-bit address*/
        case PHY_MAC_48BIT:
            /* Not used in this example */
            break;
        /*Set 64-bit address*/
        case PHY_MAC_64BIT:
            rf_set_mac_address(address_ptr);
            break;
        /*Set 16-bit address*/
        case PHY_MAC_16BIT:
            rf_set_short_adr(address_ptr);
            break;
        /*Set PAN Id*/
        case PHY_MAC_PANID:
            rf_set_pan_id(address_ptr);
            break;
    }

    return 0;
}
#endif // 0

/* Note: we are in IRQ context */
static void rf_handle_ack(uint8_t seq_number, uint8_t data_pending)
{
    phy_link_tx_status_e phy_status;

    // TODO - if we really need this - WAS: rf_if_lock();

    /*Received ACK sequence must be equal with transmitted packet sequence*/
    if(tx_sequence == seq_number)
    {
        /*When data pending bit in ACK frame is set, inform NET library*/
        if(data_pending)
            phy_status = PHY_LINK_TX_DONE_PENDING;
        else
            phy_status = PHY_LINK_TX_DONE;

        /*Call PHY TX Done API*/
        if(device_driver.phy_tx_done_cb){
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, phy_status, 1, 1);
        }
    }

    // TODO - if we really need this - WAS: rf_if_unlock();
}

/* Note: we are in IRQ context */
static inline void rf_handle_rx_end(void)
{
    uint8_t rf_lqi;
    int8_t rf_rssi;
    uint16_t rf_buffer_len;

    /* Get received data */
    rf_buffer_len = rf_device->read(rf_rx_buf, MAX_PACKET_LEN);
    if(!rf_buffer_len)
        return;

    /* If waiting for ACK, check here if the packet is an ACK to a message previously sent */
    if((rf_buffer_len == 3) && ((rf_rx_buf[0] & 0x07) == 0x02)) {
    	uint8_t pending = 0;

    	/*Check if data is pending*/
    	if ((rf_rx_buf[0] & 0x10)) {
    		pending=1;
        }

       	/*Send sequence number in ACK handler*/
       	rf_handle_ack(rf_rx_buf[2], pending);
       	return;
    }

    /* Get link information */
    rf_rssi = (int8_t)rf_device->get_last_rssi_dbm();
    rf_lqi = (uint8_t)rf_device->get_last_lqi();

    /* Note: Checksum of the packet must be checked and removed before entering here */

    /* Send received data and link information to the network stack */
    if( device_driver.phy_rx_cb ){
        device_driver.phy_rx_cb(rf_rx_buf, rf_buffer_len, rf_lqi, rf_rssi, rf_radio_driver_id);
    }
}

/* Note: we are in IRQ context */
static inline void rf_handle_tx_end(void)
{
    phy_link_tx_status_e phy_status = PHY_LINK_TX_SUCCESS;

    /*Call PHY TX Done API*/
    if(device_driver.phy_tx_done_cb){
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, phy_status, 1, 1);
    }
}

/* Note: we are in IRQ context */
static inline void rf_handle_tx_err(void) {
    phy_link_tx_status_e phy_status = PHY_LINK_TX_FAIL;

    /*Call PHY TX Done API*/
    if(device_driver.phy_tx_done_cb){
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, phy_status, 1, 1);
    }
}

/* Note: we are in IRQ context */
static void rf_callback_func(int event) {
	switch(event) {
	case SimpleSpirit1::RX_DONE:
		rf_handle_rx_end();
		break;
	case SimpleSpirit1::TX_DONE:
		rf_handle_tx_end();
		break;
	case SimpleSpirit1::TX_ERR:
		rf_handle_tx_err();
		break;
	}
}

static void rf_init(void) {
	rf_device = &SimpleSpirit1::CreateInstance(D11, D12, D13, D9, D10, D2);
	rf_device->attach_irq_callback(rf_callback_func);
}

int8_t rf_device_register(void)
{
    /* Do some initialization */
    rf_init();

    /* Set pointer to MAC address */
    device_driver.PHY_MAC = mac_address;

    /* Set driver Name */
    device_driver.driver_description = (char*)"Spirit1 Sub-GHz RF";

    /*Type of RF PHY is SubGHz*/
    device_driver.link_type = PHY_LINK_15_4_SUBGHZ_TYPE;

    /*Maximum size of payload is 255*/
    device_driver.phy_MTU = MAX_PACKET_LEN;
    /*No header in PHY*/
    device_driver.phy_header_length = 0;
    /*No tail in PHY*/
    device_driver.phy_tail_length = 0;

    /*Set up driver functions*/
    device_driver.address_write = NULL; // betzw - TODO - if we really need this - WAS: &rf_address_write;
    device_driver.extension = &rf_extension;
    device_driver.state_control = &rf_interface_state_control;
    device_driver.tx = &rf_start_cca;

    /*Set supported channel pages*/
    device_driver.phy_channel_pages = phy_channel_pages;

    //Nullify rx/tx callbacks
    device_driver.phy_rx_cb = NULL;
    device_driver.phy_tx_done_cb = NULL;
    device_driver.arm_net_virtual_rx_cb = NULL;
    device_driver.arm_net_virtual_tx_cb = NULL;

    /*Register device driver*/
    rf_radio_driver_id = arm_net_phy_register(&device_driver);

    return rf_radio_driver_id;
}
