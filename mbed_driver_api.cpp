#include "SimpleSpirit1.h"
#include "nanostack/platform/arm_hal_phy.h"
#include "platform/arm_hal_interrupt.h"

#include "mbed_trace.h"
#define TRACE_GROUP  "SPIRIT"

/*Atmel RF Part Type*/
// betzw - TODO
typedef enum
{
    ATMEL_UNKNOW_DEV = 0,
    ATMEL_AT86RF212,
    ATMEL_AT86RF231,
    ATMEL_AT86RF233
}rf_trx_part_e;

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
static uint8_t rf_rnd_rssi = 0;

const phy_rf_channel_configuration_s phy_subghz = {868000000, 1000000, 250000, 11, M_GFSK};

static phy_device_channel_page_s phy_channel_pages[] = {
    {CHANNEL_PAGE_2, &phy_subghz},
    {CHANNEL_PAGE_0, NULL}
};

static uint8_t need_ack = 0;
static uint8_t tx_sequence = 0xff;
static uint8_t mac_tx_handle = 0;

static SimpleSpirit1 *rf_device = NULL;
static uint8_t rf_rx_buf[MAX_PACKET_LEN];

static uint8_t stored_mac_address[8];
static uint8_t stored_short_adr[2];
static uint8_t stored_pan_id[2];

#define RF_SIG_ACK_NEEDED (1<<0)
static Thread rf_ack_sender(osPriorityRealtime);
static uint8_t rf_rx_sequence;
static uint8_t rf_src_pan_id[2];
static bool rf_ack_sent = false;

/* MAC frame helper macros */
#define MAC_FCF_FRAME_TYPE_MASK         0x0007
#define MAC_FCF_FRAME_TYPE_SHIFT        0
#define MAC_FCF_SECURITY_BIT_MASK       0x0008
#define MAC_FCF_SECURITY_BIT_SHIFT      3
#define MAC_FCF_PENDING_BIT_MASK        0x0010
#define MAC_FCF_PENDING_BIT_SHIFT       4
#define MAC_FCF_ACK_REQ_BIT_MASK        0x0020
#define MAC_FCF_ACK_REQ_BIT_SHIFT       5
#define MAC_FCF_INTRA_PANID_MASK        0x0040
#define MAC_FCF_INTRA_PANID_SHIFT       6
#define MAC_FCF_DST_ADDR_MASK           0x0c00
#define MAC_FCF_DST_ADDR_SHIFT          10
#define MAC_FCF_VERSION_MASK            0x3000
#define MAC_FCF_VERSION_SHIFT           12
#define MAC_FCF_SRC_ADDR_MASK           0xc000
#define MAC_FCF_SRC_ADDR_SHIFT          14

/* MAC supported frame types */
#define FC_BEACON_FRAME         0x00
#define FC_DATA_FRAME           0x01
#define FC_ACK_FRAME            0x02
#define FC_CMD_FRAME            0x03

static void rf_if_lock(void)
{
    platform_enter_critical();
}

static void rf_if_unlock(void)
{
    platform_exit_critical();
}

static int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol)
{
	MBED_ASSERT(data_length >= 3);

    /*Check if transmitter is busy*/
    if((rf_device->get_pending_packet()) || (rf_device->channel_clear() == 0)) {
    	/*Return busy*/
        return -1;
    } else {
    	/* Get Lock */
    	rf_if_lock();

    	/*Check if transmitted data needs to be acked*/
    	if((data_ptr[0] & MAC_FCF_ACK_REQ_BIT_MASK) >> MAC_FCF_ACK_REQ_BIT_SHIFT)
    		need_ack = 1;
    	else
    		need_ack = 0;

    	/*Store the sequence number for ACK handling*/
    	tx_sequence = *(data_ptr + 2);

    	/*Store TX handle*/
    	mac_tx_handle = tx_handle;

        /*Send the packet*/
        rf_device->send(data_ptr, data_length);

    	/* Release Lock */
    	rf_if_unlock();

    	tr_debug("%s (%d), tx_handle=%x, tx_seq=%x", __func__, __LINE__, tx_handle, tx_sequence);
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
        	tr_debug("%s (%d)", __func__, __LINE__);
        	rf_device->reset_board();
            break;
        /*Disable PHY Interface driver*/
        case PHY_INTERFACE_DOWN:
        	tr_debug("%s (%d)", __func__, __LINE__);
            ret_val = rf_device->off();
            if(ret_val != 0) ret_val = -1;
            break;
        /*Enable PHY Interface driver*/
        case PHY_INTERFACE_UP:
        	ret_val = rf_device->on();
        	if(ret_val != 0) {
            	tr_debug("%s (%d)", __func__, __LINE__);
        		ret_val = -1;
        		break;
        	}
        	tr_debug("%s (%d) - channel: %d", __func__, __LINE__, (int)rf_channel);
            rf_device->set_channel(rf_channel);
            break;
        /*Enable wireless interface ED scan mode*/
        case PHY_INTERFACE_RX_ENERGY_STATE:
        	tr_debug("%s (%d)", __func__, __LINE__);
            break;
        /*Enable Sniffer state*/
        case PHY_INTERFACE_SNIFFER_STATE:
            // TODO - if we really need this - WAS: rf_setup_sniffer(rf_channel);
        	tr_debug("%s (%d)", __func__, __LINE__);
        	ret_val = -1;
            break;
        default:
        	tr_debug("%s (%d)", __func__, __LINE__);
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
        	tr_debug("%s (%d)", __func__, __LINE__);
            break;

        /*Return frame pending status*/
        case PHY_EXTENSION_READ_LAST_ACK_PENDING_STATUS:
        	tr_debug("%s (%d), need_ack=%x", __func__, __LINE__, (unsigned int)need_ack);
            *data_ptr = need_ack;
            break;

        /*Set channel, used for setting channel for energy scan*/
        case PHY_EXTENSION_SET_CHANNEL:
        	tr_debug("%s (%d)", __func__, __LINE__);
            break;

        /*Read energy on the channel*/
        case PHY_EXTENSION_READ_CHANNEL_ENERGY:
            // TODO: *data_ptr = rf_get_channel_energy();
        	tr_debug("%s (%d)", __func__, __LINE__);
           	*data_ptr = rf_device->get_last_rssi_dbm();
            break;

        /*Read status of the link*/
        case PHY_EXTENSION_READ_LINK_STATUS:
            // TODO: *data_ptr = rf_get_link_status();
        	tr_debug("%s (%d)", __func__, __LINE__);
        	*data_ptr = rf_device->get_last_lqi()*17;
            break;

        default:
        	tr_debug("%s (%d)", __func__, __LINE__);
        	break;
    }
    return 0;
}

#if 0 // Not used in this example
static inline void rf_set_mac_48bit(uint8_t *ptr) {
	tr_debug("%s (%d), adr0=%x, adr1=%x, adr2=%x, adr3=%x, adr4=%x, adr5=%x",
			__func__, __LINE__,
			ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
}
#endif // 0

static inline void rf_set_mac_address(uint8_t *ptr) {
	tr_debug("%s (%d), adr0=%x, adr1=%x, adr2=%x, adr3=%x, adr4=%x, adr5=%x, adr6=%x, adr7=%x",
			__func__, __LINE__,
			ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7]);
	for(int i = 0; i < 8; i++) {
		stored_mac_address[i] = ptr[i];
	}
}

static inline void rf_set_short_adr(uint8_t *ptr) {
	tr_debug("%s (%d), adr0=%x, adr1=%x",
			__func__, __LINE__,
			ptr[0], ptr[1]);
	stored_short_adr[0] = ptr[0];
	stored_short_adr[1] = ptr[1];
}

static inline void rf_set_pan_id(uint8_t *ptr) {
	tr_debug("%s (%d), adr0=%x, adr1=%x",
			__func__, __LINE__,
			ptr[0], ptr[1]);
	stored_pan_id[0] = ptr[0];
}

static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{

    switch (address_type)
    {
        /*Set 48-bit address*/
        case PHY_MAC_48BIT:
            /* Not used in this example */
        	// betzw - WAS: rf_set_mac_48bit(address_ptr);
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

/* Note: we are in IRQ context */
static void rf_handle_ack(uint8_t seq_number)
{
    /*Received ACK sequence must be equal with transmitted packet sequence*/
    if(tx_sequence == seq_number)
    {
    	/* Reset 'need_ack' */
    	need_ack = 0;

    	/*Call PHY TX Done API*/
        if(device_driver.phy_tx_done_cb){
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_DONE, 1, 1);
        }
    }
}

/* Note: we are in IRQ context */
static inline bool rf_check_mac_address(uint8_t *dest) {
	for(int i = 0; i < 8; i++) {
		if(dest[i] != stored_mac_address[i]) return false;
	}
	return true;
}

/* Note: we are in IRQ context */
/* Returns true if packet should be accepted */
static bool rf_check_destination(int len, uint8_t *ack_requested) {
	uint8_t dst_pan_id[2];
	uint8_t dest_addr[8];
	uint8_t dst_addr_mode = 0x0; /*0x00 = no address 0x01 = reserved 0x02 = 16-bit short address 0x03 = 64-bit extended address */
	uint8_t src_addr_mode = 0x0; /*0x00 = no address 0x01 = reserved 0x02 = 16-bit short address 0x03 = 64-bit extended address */
	uint8_t min_size = 3;
	bool ret = false;

	if(len < min_size) return false;

	(*ack_requested) = ((rf_rx_buf[0] & MAC_FCF_ACK_REQ_BIT_MASK) >> MAC_FCF_ACK_REQ_BIT_SHIFT);
	dst_addr_mode = ((rf_rx_buf[0] & MAC_FCF_DST_ADDR_MASK) >> MAC_FCF_DST_ADDR_SHIFT);
	src_addr_mode = ((rf_rx_buf[0] & MAC_FCF_SRC_ADDR_MASK) >> MAC_FCF_SRC_ADDR_SHIFT);

	rf_rx_sequence = rf_rx_buf[2];

	switch(dst_addr_mode) {
	case 0x00:
		ret = true; // no check & ack possible;
		(*ack_requested) = 0;
		break;
	case 0x02:
		min_size = 7;
		if(len < min_size) return false;
		min_size = 9;
		dst_pan_id[0] = rf_rx_buf[3];
		dst_pan_id[1] = rf_rx_buf[4];
		if((dst_pan_id[0] == 0xFF) && (dst_pan_id[1] == 0xFF)) {
			ret = true;
			break;
		}

		if((dst_pan_id[0] == stored_pan_id[0]) && (dst_pan_id[1] == stored_pan_id[1])) {
			ret = true;
			break;
		}

		dest_addr[0] = rf_rx_buf[5];
		dest_addr[1] = rf_rx_buf[6];
		if((dest_addr[0] == stored_short_adr[0]) && (dest_addr[1] == stored_short_adr[1])) {
			ret = true;
			break;
		}
		break;
	case 0x03:
		min_size = 7;
		if(len < 7) return false;
		min_size = 15;
		dst_pan_id[0] = rf_rx_buf[3];
		dst_pan_id[1] = rf_rx_buf[4];
		if((dst_pan_id[0] == 0xFF) && (dst_pan_id[1] == 0xFF)) {
			ret = true;
			break;
		}

		ret = rf_check_mac_address(&rf_rx_buf[5]);
		break;
	default:
		/* not supported */
		return false;
	}

	if(*ack_requested) {
		if(src_addr_mode == 0x00) {
			*ack_requested = 0; // cannot send acknowledgment
		} else {
			if(len < min_size) {
				*ack_requested = 0; // cannot send acknowledgment
			} else {
				rf_src_pan_id[0] = rf_rx_buf[min_size-2];
				rf_src_pan_id[1] = rf_rx_buf[min_size-1];
			}
		}
	}

	return ret;
}

/* Note: we are in IRQ context */
static inline void rf_send_ack() {
	rf_ack_sender.signal_set(RF_SIG_ACK_NEEDED);
}

/* Note: we are in IRQ context */
static inline void rf_handle_rx_end(void)
{
    uint8_t rf_lqi;
    int8_t rf_rssi;
    uint16_t rf_buffer_len;
    uint8_t ack_requested = 0;

    /* Get received data */
    rf_buffer_len = rf_device->read(rf_rx_buf, MAX_PACKET_LEN);
    if(!rf_buffer_len)
        return;

    /* Check if packet should be accepted */
    if(!rf_check_destination(rf_buffer_len, &ack_requested)) {
    	return;
    }

    /* If waiting for ACK, check here if the packet is an ACK to a message previously sent */
    if((rf_buffer_len == 5) && (((rf_rx_buf[0] & MAC_FCF_FRAME_TYPE_MASK) >> MAC_FCF_FRAME_TYPE_SHIFT) == FC_ACK_FRAME)) {
    	/*Send sequence number in ACK handler*/
    	tr_debug("%s (%d), len=%u", __func__, __LINE__, (unsigned int)rf_buffer_len);
       	rf_handle_ack(rf_rx_buf[2]);
       	return;
    }

    /* Kick off ACK sending */
    if(ack_requested) {
    	rf_send_ack();
    }

    /* Get link information */
    rf_rssi = (int8_t)rf_device->get_last_rssi_dbm();
    rf_lqi = (uint8_t)rf_device->get_last_lqi();
    rf_lqi *= 17; // scale to 8-bit value

    /* Note: Checksum of the packet must be checked and removed before entering here */
    /* TODO - betzw: what to do? */
    // rf_buffer_len -= 2;

    /* Send received data and link information to the network stack */
    if( device_driver.phy_rx_cb ){
        device_driver.phy_rx_cb(rf_rx_buf, rf_buffer_len, rf_lqi, rf_rssi, rf_radio_driver_id);
    }
}

/* Note: we are in IRQ context */
static inline void rf_handle_tx_end(void)
{
    phy_link_tx_status_e phy_status = PHY_LINK_TX_SUCCESS;

    /* Check if this is an ACK sending which is still pending */
	if(rf_ack_sent) {
		rf_ack_sent = false;
		return; // no need to inform stack
	}

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
    	tr_debug("%s (%d)", __func__, __LINE__);
		rf_handle_tx_err();
		break;
	}
}

static void rf_ack_loop(void) {
	static uint16_t buffer[4];

	/* Pre-prepare payload */
	buffer[0] = FC_ACK_FRAME | (0x2 << MAC_FCF_DST_ADDR_SHIFT); // FCF

	do {
		/* Wait for signal */
		rf_ack_sender.signal_wait(RF_SIG_ACK_NEEDED);

		/* Prepare payload */
		uint8_t *ptr = (uint8_t*)&buffer[1];
		ptr[0] = rf_rx_sequence;   // Sequence number
		ptr[1] = rf_src_pan_id[0]; // pan_id_0
		ptr[2] = rf_src_pan_id[1]; // pan_id_1

		/* Get Lock */
		rf_if_lock();

		/* Wait for device not receiving */
		while(rf_device->is_receiving()) {
			wait_us(10);
		}

		/* Set information that we have sent an ACK */
		rf_ack_sent = true;

        /*Send the packet*/
        rf_device->send((uint8_t*)buffer, 5);

		/* Release Lock */
		rf_if_unlock();
	} while(true);
}

static void rf_init(void) {
	rf_device = &SimpleSpirit1::CreateInstance(D11, D12, D13, D9, D10, D2);
	rf_device->attach_irq_callback(rf_callback_func);

	rf_ack_sender.start(rf_ack_loop);
}

extern "C" int8_t rf_device_register(void)
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
    device_driver.address_write = &rf_address_write;
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

	tr_debug("%s (%d)", __func__, __LINE__);
    return rf_radio_driver_id;
}

/*
 * \brief Function reads the MAC address array.
 *
 * \param ptr Pointer to read array
 *
 * \return none
 */
extern "C" void rf_read_mac_address(uint8_t *ptr)
{
	tr_debug("%s (%d)", __func__, __LINE__);
    memcpy(ptr, mac_address, 8);
}

/*
 * \brief Function returns the generated 8-bit random value for seeding Pseudo-random generator. This value was generated by reading noise from RF channel in RF initialisation.
 *
 * \param none
 *
 * \return random RSSI value
 */
extern "C" int8_t rf_read_random(void)
{
	tr_debug("%s (%d)", __func__, __LINE__);
    return rf_rnd_rssi;
}

/*
 * \brief Read connected radio part.
 *
 * This function only return valid information when rf_init() is called
 *
 * \return
 */
extern "C" rf_trx_part_e rf_radio_type_read(void)
{
	tr_debug("%s (%d)", __func__, __LINE__);
	return ATMEL_UNKNOW_DEV;
}
