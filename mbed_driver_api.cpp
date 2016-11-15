#include "SimpleSpirit1.h"
#include "nanostack/platform/arm_hal_phy.h"
#include "platform/arm_hal_interrupt.h"

#include "mbed_trace.h"
#define TRACE_GROUP  "SPIRIT"

/* Define beyond macro if you want to perform heavy debug tracing also in IRQ context */
// #define HEAVY_TRACING

/* Define beyond macro if you want to use acknowledgment frames with only 3 bytes */
#define SHORT_ACK_FRAMES

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

const phy_rf_channel_configuration_s phy_subghz = {868000000, 1000000, 250000, 11, M_GFSK};

static phy_device_channel_page_s phy_channel_pages[] = {
    {CHANNEL_PAGE_2, &phy_subghz},
    {CHANNEL_PAGE_0, NULL}
};

static uint8_t tx_sequence = 0xff;
static uint8_t mac_tx_handle = 0;

static SimpleSpirit1 *rf_device = NULL;
static uint8_t rf_rx_buf[MAX_PACKET_LEN];

static uint8_t stored_mac_address[8];
static uint16_t stored_short_adr;
static uint16_t stored_pan_id;

#define RF_SIG_ACK_NEEDED (1<<0)
static Thread rf_ack_sender(osPriorityRealtime);
static volatile uint8_t rf_rx_sequence;
#ifndef SHORT_ACK_FRAMES
static volatile uint8_t rf_src_adr[8];
static volatile uint8_t rf_src_adr_len = 0;
#endif
static volatile bool rf_ack_sent = false;

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

static inline uint16_t rf_read_16_bit(uint8_t *data_ptr) { // little-endian
	uint16_t ret;

	ret = ((uint16_t)data_ptr[0]) + (((uint16_t)data_ptr[1]) << 8);
	return ret;
}

static int8_t rf_trigger_send(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol)
{
#ifndef NDEBUG
	debug_if(!(data_length >= 3), "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif

	/* Give 'rf_ack_sender' a better chance to run */
	Thread::yield();

	/* Get Lock */
	rf_if_lock();

    /*Check if transmitter is busy*/
    if(rf_device->is_receiving()) { /* betzw - WAS: (rf_device->channel_clear() != 0)), do NOT use this but rather study and enable automatic CCA */
    	tr_debug("%s (%d)", __func__, __LINE__);

    	/* Release Lock */
    	rf_if_unlock();

    	/*Return busy*/
        return -1;
    } else {
    	uint16_t fcf = rf_read_16_bit(data_ptr);

#ifdef HEAVY_TRACING
    	uint16_t need_ack;

    	/*Check if transmitted data needs to be acked*/
    	if((fcf & MAC_FCF_ACK_REQ_BIT_MASK) >> MAC_FCF_ACK_REQ_BIT_SHIFT)
    		need_ack = 1;
    	else
    		need_ack = 0;
#endif

    	/*Store the sequence number for ACK handling*/
    	tx_sequence = *(data_ptr + 2);

    	/*Store TX handle*/
    	mac_tx_handle = tx_handle;

#ifdef HEAVY_TRACING
    	tr_info("%s (%d), len=%d, tx_handle=%x, tx_seq=%x, need_ack=%d (%x:%x, %x:%x, %x:%x, %x:%x)", __func__, __LINE__,
    			data_length, tx_handle, tx_sequence, need_ack,
				data_ptr[3], data_ptr[4], data_ptr[5], data_ptr[6],
				data_ptr[7], data_ptr[8], data_ptr[9], data_ptr[10]);
#endif

    	    	    	/*Send the packet*/
        rf_device->send(data_ptr, data_length);

    	/* Release Lock */
    	rf_if_unlock();
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
        	tr_debug("%s (%d)", __func__, __LINE__);
            *data_ptr = 0;
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
	stored_short_adr = (ptr[0] << 8) + ptr[1]; // big-endian
	tr_debug("%s (%d), adr0=%x, adr1=%x, val=%d",
			__func__, __LINE__,
			ptr[0], ptr[1], stored_short_adr);
}

static inline void rf_set_pan_id(uint8_t *ptr) {
	stored_pan_id = (ptr[0] << 8) + ptr[1]; // big-endian
	tr_debug("%s (%d), adr0=%x, adr1=%x, val=%d",
			__func__, __LINE__,
			ptr[0], ptr[1], stored_pan_id);
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
#ifdef HEAVY_TRACING
    	tr_info("%s (%d)", __func__, __LINE__);
#endif

    			/*Call PHY TX Done API*/
        if(device_driver.phy_tx_done_cb){
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_DONE, 0, 0);
        }
    } else {
#ifdef HEAVY_TRACING
		tr_info("%s (%d)", __func__, __LINE__);
#endif
    }
}

/* Note: we are in IRQ context */
static inline bool rf_check_mac_address(uint8_t *dest) {
	for(int i = 0; i < 8; i++) {
		if(dest[i] != stored_mac_address[7-i]) return false;
	}
	return true;
}

/* Note: we are in IRQ context */
/* Returns true if packet should be accepted */
static bool rf_check_destination(int len, uint8_t *ack_requested) {
	uint8_t frame_type;
	uint16_t dst_pan_id;
	uint16_t dst_short_adr;
	uint8_t dst_addr_mode = 0x0; /*0x00 = no address 0x01 = reserved 0x02 = 16-bit short address 0x03 = 64-bit extended address */
	uint8_t src_addr_mode = 0x0; /*0x00 = no address 0x01 = reserved 0x02 = 16-bit short address 0x03 = 64-bit extended address */
	uint8_t min_size = 3; // FCF & SeqNr
	bool ret = false;
#if !defined(SHORT_ACK_FRAMES) || defined(HEAVY_TRACING)
	bool panid_compr = false;
#endif

	if(len < 3) {
    	tr_debug("%s (%d)", __func__, __LINE__);
		return false;
	}

	uint16_t fcf = rf_read_16_bit(rf_rx_buf);
	frame_type = ((fcf & MAC_FCF_FRAME_TYPE_MASK) >> MAC_FCF_FRAME_TYPE_SHIFT);
	(*ack_requested) = ((fcf & MAC_FCF_ACK_REQ_BIT_MASK) >> MAC_FCF_ACK_REQ_BIT_SHIFT);
	dst_addr_mode = ((fcf & MAC_FCF_DST_ADDR_MASK) >> MAC_FCF_DST_ADDR_SHIFT);
	src_addr_mode = ((fcf & MAC_FCF_SRC_ADDR_MASK) >> MAC_FCF_SRC_ADDR_SHIFT);
#if !defined(SHORT_ACK_FRAMES) || defined(HEAVY_TRACING)
	panid_compr = ((fcf & MAC_FCF_INTRA_PANID_MASK) >> MAC_FCF_INTRA_PANID_SHIFT);
#endif

#ifdef HEAVY_TRACING
	tr_info("%s (%d): len=%d, ftype=%x, snr=%x, ack=%d, dst=%x, src=%x, intra=%d", __func__, __LINE__, len, frame_type,
			rf_rx_buf[2], (*ack_requested), dst_addr_mode, src_addr_mode, panid_compr);
#endif

	if(frame_type == FC_ACK_FRAME) { // betzw: we support up to two different forms of ACK frames!
#ifdef SHORT_ACK_FRAMES
		if((len == 3) && (dst_addr_mode == 0x0) && (src_addr_mode == 0x0)) {
			ret = true;
		}
#else // !SHORT_ACK_FRAMES
		if((dst_addr_mode == 0x3) && (src_addr_mode == 0x3)) {
			if(panid_compr) { // no PAN ID is in the frame
				ret = rf_check_mac_address(&rf_rx_buf[3]);
#ifdef HEAVY_TRACING
				if(!ret) tr_debug("%s (%d)", __func__, __LINE__);
#endif
			} else { // presence of dest pan id (even if indicating src pan id)
				ret = rf_check_mac_address(&rf_rx_buf[5]);
#ifdef HEAVY_TRACING
				if(!ret) tr_debug("%s (%d)", __func__, __LINE__);
#endif
			}
		} else if(dst_addr_mode == 0x2) {
	    	dst_short_adr = rf_read_16_bit(&rf_rx_buf[5]);
			if(dst_short_adr == stored_short_adr) {
		    	tr_debug("%s (%d)", __func__, __LINE__);
				ret = true;
			} else {
#ifdef HEAVY_TRACING
		    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
			}
		}
#endif // !SHORT_ACK_FRAMES

#ifdef HEAVY_TRACING
		tr_info("%s (%d): ret=%d", __func__, __LINE__, ret);
#endif
		(*ack_requested) = 0;  // Never acknowledge ACK frames
		return ret;
	}

	switch(dst_addr_mode) {
	case 0x00:
		ret = true; // no check possible;
		break;
	case 0x02:
		min_size += 4; // pan id + short dest adr

		if(len < 5) {
#ifdef HEAVY_TRACING
	    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
			return false;
		}

		dst_pan_id = rf_read_16_bit(&rf_rx_buf[3]);
		if(dst_pan_id == 0xFFFF) {
#ifdef HEAVY_TRACING
			tr_debug("%s (%d)", __func__, __LINE__);
#endif
			ret = true;
			break;
		}

		if(dst_pan_id == stored_pan_id) {
#ifdef HEAVY_TRACING
			tr_debug("%s (%d)", __func__, __LINE__);
#endif
			ret = true;
			break;
		} else {
#ifdef HEAVY_TRACING
			tr_debug("%s (%d): %d!=%d", __func__, __LINE__, dst_pan_id, stored_pan_id);
#endif
		}

		if(len < 7) {
#ifdef HEAVY_TRACING
	    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
			return false;
		}

    	dst_short_adr = rf_read_16_bit(&rf_rx_buf[5]);
		if(dst_short_adr == stored_short_adr) {
#ifdef HEAVY_TRACING
	    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
			ret = true;
			break;
		} else {
#ifdef HEAVY_TRACING
	    	tr_debug("%s (%d): %d!=%d", __func__, __LINE__, dst_short_adr, stored_short_adr);
#endif
		}
		break;
	case 0x03:
		min_size += 10; // pan id + dest mac addr

		if(len < 5) {
#ifdef HEAVY_TRACING
	    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
			return false;
		}

		dst_pan_id = rf_read_16_bit(&rf_rx_buf[3]);
		if(dst_pan_id == 0xFFFF) {
#ifdef HEAVY_TRACING
			tr_debug("%s (%d)", __func__, __LINE__);
#endif
			ret = true;
			break;
		}

		if(dst_pan_id == stored_pan_id) {
#ifdef HEAVY_TRACING
			tr_debug("%s (%d)", __func__, __LINE__);
#endif
			ret = true;
			break;
		}

		if(len < 13) {
#ifdef HEAVY_TRACING
	    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
			return false;
		}

    	ret = rf_check_mac_address(&rf_rx_buf[5]);
		break;
	default:
		/* not supported */
#ifdef HEAVY_TRACING
    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
		return false;
	}

	if(ret && (*ack_requested)) {
		rf_rx_sequence = rf_rx_buf[2];

#ifndef SHORT_ACK_FRAMES
		if(!panid_compr) { // Src PAN Id is present
			min_size += 2; // src pan id
		}
		switch(src_addr_mode) {
		case 0x00:
			(*ack_requested) = 0; // cannot send acknowledgment
			break;
		case 0x02:
			min_size += 2; // short src adr
			if(len < min_size) {
				(*ack_requested) = 0; // cannot send acknowledgment
			} else {
				rf_src_adr_len = 2;
				rf_src_adr[0] = rf_rx_buf[min_size-2];
				rf_src_adr[1] = rf_rx_buf[min_size-1];
#ifdef HEAVY_TRACING
				tr_debug("%s (%d): %x:%x", __func__, __LINE__, rf_src_adr[0], rf_src_adr[1]);
#endif
			}
			break;
		case 0x03:
			min_size += 8; // src mac adr
			if(len < min_size) {
				(*ack_requested) = 0; // cannot send acknowledgment
			} else {
				rf_src_adr_len = 8;
				memcpy(&rf_src_adr[0], &rf_rx_buf[min_size-8], rf_src_adr_len);
#ifdef HEAVY_TRACING
				tr_debug("%s (%d): %x:%x", __func__, __LINE__, rf_src_adr[0], rf_src_adr[1]);
#endif
			}
			break;
		default:
			/* not supported */
#ifdef HEAVY_TRACING
	    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
			return false;
		}
#endif // !SHORT_ACK_FRAMES
	}

#ifdef HEAVY_TRACING
	tr_info("%s (%d), ret=%d, ack=%d", __func__, __LINE__, ret, (*ack_requested));
#endif
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

#ifdef HEAVY_TRACING
    	tr_debug("%s (%d)", __func__, __LINE__);
#endif

    	/* Check if packet should be accepted */
    if(!rf_check_destination(rf_buffer_len, &ack_requested)) {
#ifdef HEAVY_TRACING
    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
    	return;
    }

    /* If waiting for ACK, check here if the packet is an ACK to a message previously sent */
    uint16_t fcf = rf_read_16_bit(rf_rx_buf);
    if(((fcf & MAC_FCF_FRAME_TYPE_MASK) >> MAC_FCF_FRAME_TYPE_SHIFT) == FC_ACK_FRAME) {
    	/*Send sequence number in ACK handler*/
#ifdef HEAVY_TRACING
    	tr_debug("%s (%d), len=%u", __func__, __LINE__, (unsigned int)rf_buffer_len);
#endif
       	rf_handle_ack(rf_rx_buf[2]);
       	return;
    }

    /* Kick off ACK sending */
    if(ack_requested) {
#ifdef HEAVY_TRACING
    	tr_debug("%s (%d), len=%u", __func__, __LINE__, (unsigned int)rf_buffer_len);
#endif
    	rf_send_ack();
    }

    /* Get link information */
    rf_rssi = (int8_t)rf_device->get_last_rssi_dbm();
    rf_lqi = (uint8_t)rf_device->get_last_lqi();
    rf_lqi *= 17; // scale to 8-bit value

    /* Note: Checksum of the packet must be checked and removed before entering here */
    /* TODO - betzw: what to do? */
    // rf_buffer_len -= 2;

#ifdef HEAVY_TRACING
      	tr_debug("%s (%d)", __func__, __LINE__);
#endif

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
#ifdef HEAVY_TRACING
    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
		return; // no need to inform stack
	}

    /*Call PHY TX Done API*/
    if(device_driver.phy_tx_done_cb){
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, phy_status, 0, 0);
    }
}

/* Note: we are in IRQ context */
static inline void rf_handle_tx_err(void) {
    phy_link_tx_status_e phy_status = PHY_LINK_TX_FAIL;

    /*Call PHY TX Done API*/
    if(device_driver.phy_tx_done_cb){
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, phy_status, 0, 0);
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
#ifdef HEAVY_TRACING
    	tr_debug("%s (%d): TX_ERR!!!", __func__, __LINE__);
#endif
		rf_handle_tx_err();
		break;
	}
}

#ifdef SHORT_ACK_FRAMES
static void rf_ack_loop(void) {
	static uint16_t buffer[2] = {
			(FC_ACK_FRAME << MAC_FCF_FRAME_TYPE_SHIFT),
			0x0
	};

	tr_debug("%s (%d)", __func__, __LINE__);

	do {
		/* Wait for signal */
		rf_ack_sender.signal_wait(RF_SIG_ACK_NEEDED);

#ifdef HEAVY_TRACING
    	tr_debug("%s (%d)", __func__, __LINE__);
#endif

    	/* Get Lock */
		rf_if_lock();

#ifdef HEAVY_TRACING
    	tr_debug("%s (%d)", __func__, __LINE__);
#endif

		/* Prepare payload */
		uint8_t *ptr = (uint8_t*)&buffer[1];
		ptr[0] = rf_rx_sequence;   // Sequence number

		/* Wait for device not receiving */
		while(rf_device->is_receiving()) {
#ifdef HEAVY_TRACING
	    	tr_info("%s (%d)", __func__, __LINE__);
#endif
			wait_us(10);
		}

#ifdef HEAVY_TRACING
    	tr_debug("%s (%d), hdr=%x, nr=%x", __func__, __LINE__, buffer[0], ptr[0]);
#endif

    	/* Set information that we have sent an ACK */
		rf_ack_sent = true;

        /*Send the packet*/
        rf_device->send((uint8_t*)buffer, 3);

    	tr_debug("%s (%d), hdr=%x, nr=%x", __func__, __LINE__, buffer[0], ptr[0]);

    	/* Release Lock */
		rf_if_unlock();

#ifdef HEAVY_TRACING
		tr_debug("%s (%d)", __func__, __LINE__);
#endif
	} while(true);
}
#else // !SHORT_ACK_FRAMES
static void rf_ack_loop(void) {
	static uint16_t buffer[6];
	uint8_t *dest;
	uint8_t msg_len;

	tr_debug("%s (%d)", __func__, __LINE__);

	do {
		/* Wait for signal */
		rf_ack_sender.signal_wait(RF_SIG_ACK_NEEDED);

#ifdef HEAVY_TRACING
    	tr_debug("%s (%d)", __func__, __LINE__);
#endif

    	/* Get Lock */
		rf_if_lock();

#ifdef HEAVY_TRACING
    	tr_debug("%s (%d)", __func__, __LINE__);
#endif

    	/* Prepare header */
		uint8_t *ptr = (uint8_t*)&buffer[1];
		if(rf_src_adr_len == 2) {
			buffer[0] = (FC_ACK_FRAME << MAC_FCF_FRAME_TYPE_SHIFT) | (0x02 << MAC_FCF_DST_ADDR_SHIFT); // dest PAN id present
			dest = &ptr[3];
			msg_len = 7;
		} else {
			buffer[0] =
					(FC_ACK_FRAME << MAC_FCF_FRAME_TYPE_SHIFT) |
					(0x03 << MAC_FCF_DST_ADDR_SHIFT) |
					(0x03 << MAC_FCF_SRC_ADDR_SHIFT) |
					(0x01 << MAC_FCF_INTRA_PANID_SHIFT); // no PAN IDs
			dest = &ptr[1];
			msg_len = 11;
		}

		/* Prepare payload */
		ptr[0] = rf_rx_sequence;   // Sequence number
		memcpy(dest, &rf_src_adr[0], rf_src_adr_len);

		/* Wait for device not receiving */
		while(rf_device->is_receiving()) {
#ifdef HEAVY_TRACING
	    	tr_debug("%s (%d)", __func__, __LINE__);
#endif
			wait_us(10);
		}

#ifdef HEAVY_TRACING
    	tr_debug("%s (%d), hdr=%x, nr=%x, pan0=%x, pan1=%x, adr0=%x, adr1=%x", __func__, __LINE__,
    			buffer[0], ptr[0], ptr[1], ptr[2], ptr[3], ptr[4]);
#endif

    	/* Set information that we have sent an ACK */
		rf_ack_sent = true;

        /*Send the packet*/
        rf_device->send((uint8_t*)buffer, msg_len);

		/* Release Lock */
		rf_if_unlock();

		tr_debug("%s (%d)", __func__, __LINE__);
	} while(true);
}
#endif // !SHORT_ACK_FRAMES

static void rf_init(void) {
#ifndef NDEBUG
	osStatus ret;
#endif

	rf_device = &SimpleSpirit1::CreateInstance(D11, D12, D13, D9, D10, D2);
	rf_device->attach_irq_callback(rf_callback_func);

#ifndef NDEBUG
	ret =
#endif
	rf_ack_sender.start(rf_ack_loop);

#ifndef NDEBUG
	debug_if(!(ret == osOK), "\n\rassert failed in: %s (%d)\n\r", __func__, __LINE__);
#endif
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
    device_driver.tx = &rf_trigger_send;

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
	float tmp;

	rf_device->channel_clear();
	tmp = rf_device->get_last_rssi_dbm();
	tmp *= -10;

	tr_debug("%s (%d): ret=%d", __func__, __LINE__, (uint8_t)tmp);
    return (uint8_t)tmp;
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
