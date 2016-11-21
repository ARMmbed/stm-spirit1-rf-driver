/*
 * Copyright (c) 2014-2015 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DRIVERRFPHY_H_
#define DRIVERRFPHY_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "nanostack/platform/arm_hal_phy.h"
	
/*Atmel RF Part Type*/
typedef enum {
	ATMEL_UNKNOW_DEV = 0,
	ATMEL_AT86RF212,
	ATMEL_AT86RF231,
	ATMEL_AT86RF233
}rf_trx_part_e;
	
extern int8_t rf_device_register(void);
extern rf_trx_part_e rf_radio_type_read(void);
extern void rf_read_mac_address(uint8_t *ptr);
extern int8_t rf_read_random(void);
	
#ifdef __cplusplus
}
#endif
#endif /* DRIVERRFPHY_H_ */
