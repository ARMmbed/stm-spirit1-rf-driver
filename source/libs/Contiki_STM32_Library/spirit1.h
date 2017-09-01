/*
 * Copyright (c) 2012, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
#ifndef __SPIRIT_H__
#define __SPIRIT_H__
/*---------------------------------------------------------------------------*/
#include "radio.h"
#include "SPIRIT_Config.h"
#include "spirit1-config.h"
//#include "spirit1_appli.h"
#include "spirit1-const.h"
/*---------------------------------------------------------------------------*/   
extern const struct radio_driver spirit_radio_driver;
void spirit1_interrupt_callback(void);

/* exported from spirit1appli.h */

#include "radio_shield_config.h"
#include "MCU_Interface.h"
#include "SPIRIT_Config.h"
// betzw - WAS: #include "SPIRIT1_Util.h"


#if defined(X_NUCLEO_IDS01A3)
	 #define USE_SPIRIT1_433MHz
#elif defined(X_NUCLEO_IDS01A4)
         #define USE_SPIRIT1_868MHz
#elif defined(X_NUCLEO_IDS01A5)
         #define USE_SPIRIT1_915MHz
#else
#error SPIRIT1 Nucleo Shield undefined or unsupported
#endif

/*  Uncomment the Link Layer features to be used */
// #define USE_AUTO_ACK
// #define USE_AUTO_ACK_PIGGYBACKING
// #define USE_AUTO_RETRANSMISSION

#if defined(USE_AUTO_ACK)&& defined(USE_AUTO_ACK_PIGGYBACKING)&& defined(USE_AUTO_RETRANSMISSION)
#define USE_STack_PROTOCOL

/* LLP configuration parameters */
#define EN_AUTOACK                      S_ENABLE
#define EN_PIGGYBACKING             	S_ENABLE
#define MAX_RETRANSMISSIONS         	PKT_N_RETX_2

#else
#define USE_BASIC_PROTOCOL

#endif

/*  Uncomment the system Operating mode */
//#define USE_LOW_POWER_MODE

#if defined (USE_LOW_POWER_MODE)
#define LPM_ENABLE
#define MCU_STOP_MODE
//#define MCU_SLEEP_MODE
//#define RF_STANDBY
#endif


/* Exported constants --------------------------------------------------------*/

/* Radio configuration parameters  */
/* General Remarks:
 * Two SPSGRF modules will only communicate when both are having same frequency , same channel number,
 * same modulation scheme, same data rate, etc.
 * For example, the SPSGRF-915 module supports frequencies 902 to 928 MHz. User can select any frequency
 * between this band.
 */
#define XTAL_OFFSET_PPM             0
#define INFINITE_TIMEOUT            0.0

#ifdef USE_SPIRIT1_433MHz
#define BASE_FREQUENCY              433.0e6
#endif

#ifdef USE_SPIRIT1_868MHz
#define BASE_FREQUENCY              868.0e6
#endif

#ifdef USE_SPIRIT1_915MHz
#define BASE_FREQUENCY              915.0e6
#endif


/*  Addresses configuration parameters  */
#define EN_FILT_MY_ADDRESS          S_DISABLE
#define MY_ADDRESS                  0x24
#define EN_FILT_MULTICAST_ADDRESS   S_DISABLE
#define MULTICAST_ADDRESS           0xEE
#define EN_FILT_BROADCAST_ADDRESS   S_DISABLE
#define BROADCAST_ADDRESS           0xFF
#define DESTINATION_ADDRESS         0x44
#define EN_FILT_SOURCE_ADDRESS      S_DISABLE
#define SOURCE_ADDR_MASK            0xf0
#define SOURCE_ADDR_REF             0x37

#define APPLI_CMD                       0x11
#define NWK_CMD                         0x22
#define LED_TOGGLE                      0xff
#define ACK_OK                          0x01
#define MAX_BUFFER_LEN                  96
#define TIME_TO_EXIT_RX                 3000
#define DELAY_RX_LED_TOGGLE             200
#define DELAY_TX_LED_GLOW               1000
#define LPM_WAKEUP_TIME                 100
#define DATA_SEND_TIME                  30

#define PREAMBLE_LENGTH             PKT_PREAMBLE_LENGTH_04BYTES
#define SYNC_LENGTH                 PKT_SYNC_LENGTH_4BYTES
#define CONTROL_LENGTH              PKT_CONTROL_LENGTH_0BYTES
#define EN_ADDRESS                  S_DISABLE
#define EN_FEC                      S_DISABLE
#define CHANNEL_NUMBER              1 // betzw - WAS: 0
#define LENGTH_TYPE                 PKT_LENGTH_VAR
#define POWER_INDEX                 7
#define RECEIVE_TIMEOUT             2000.0 /*change the value for required timeout period*/
#define RSSI_THRESHOLD              -120



#define POWER_DBM                   11.6
#define CHANNEL_SPACE               100e3
#define FREQ_DEVIATION              127e3
#define BANDWIDTH                   540.0e3
#define MODULATION_SELECT           GFSK_BT1
#define DATARATE                    250000
#define XTAL_OFFSET_PPM             0
#define SYNC_WORD                   0x88888888
#define LENGTH_WIDTH                8 // betzw - NOTE: only 255 bytes for payload!!!
#define CRC_MODE                    PKT_CRC_MODE_16BITS_2
#define EN_WHITENING                S_DISABLE
#define INFINITE_TIMEOUT            0.0

// extern volatile FlagStatus xRxDoneFlag, xTxDoneFlag;
// extern volatile FlagStatus PushButtonStatusWakeup;
extern uint16_t wakeupCounter;
extern uint16_t dataSendCounter ;
// extern volatile FlagStatus PushButtonStatusData, datasendFlag;

typedef struct
{
  uint8_t Cmdtag;
  uint8_t CmdType;
  uint8_t CmdLen;
  uint8_t Cmd;
  uint8_t DataLen;
  uint8_t* DataBuff;
}AppliFrame_t;

/*---------------------------------------------------------------------------*/
#endif /* __SPIRIT_H__ */
/*---------------------------------------------------------------------------*/
