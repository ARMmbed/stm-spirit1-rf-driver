/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef IP64_CONF_H
#define IP64_CONF_H

/*
#include "ip64-tap-driver.h"
#include "ip64-eth-interface.h"

#define IP64_CONF_UIP_FALLBACK_INTERFACE ip64_eth_interface
#define IP64_CONF_INPUT                  ip64_eth_interface_input

#define IP64_CONF_ETH_DRIVER             ip64_tap_driver


#undef UIP_FALLBACK_INTERFACE
#define UIP_FALLBACK_INTERFACE ip64_uip_fallback_interface
*/
#ifdef MY_DRIVERS
#include <my_wifi_interface.h>
#include <my_wifi_driver.h>

#define IP64_CONF_UIP_FALLBACK_INTERFACE_SLIP 0
#define IP64_CONF_UIP_FALLBACK_INTERFACE my_wifi_interface
#define IP64_CONF_INPUT                  my_wifi_interface_input
#define IP64_CONF_ETH_DRIVER             my_wifi_driver

#undef UIP_CONF_ND6_RA_RDNSS
#define UIP_CONF_ND6_RA_RDNSS 1

#undef UIP_CONF_ND6_SEND_RA
#define UIP_CONF_ND6_SEND_RA  1

#undef UIP_CONF_ROUTER
#define UIP_CONF_ROUTER       1

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM          16
#endif


#else


#include "net/ip64/ip64-slip-interface.h"
#include "net/ip64/ip64-null-driver.h"

#define IP64_CONF_UIP_FALLBACK_INTERFACE_SLIP 1
#define IP64_CONF_UIP_FALLBACK_INTERFACE ip64_slip_interface
#define IP64_CONF_INPUT                  ip64_slip_interface_input
#define IP64_CONF_ETH_DRIVER             ip64_null_driver

#undef UIP_CONF_ND6_RA_RDNSS
#define UIP_CONF_ND6_RA_RDNSS 1

#undef UIP_CONF_ND6_SEND_RA
#define UIP_CONF_ND6_SEND_RA  1

#undef UIP_CONF_ROUTER
#define UIP_CONF_ROUTER       1

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM          16
#endif

#endif//MY_DRIVERS

#endif /* IP64_CONF_H */
