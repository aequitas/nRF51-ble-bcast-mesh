/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#ifndef _VERSION_HANDLER_H__
#define _VERSION_HANDLER_H__
#include "rbc_mesh.h"
#include "ble_gap.h"
#include "mesh_packet.h"
#include <stdint.h>
#include <stdbool.h>

typedef __packed_armcc struct {
	uint8_t id;
	int16_t rssi;
	uint8_t count; // Used to timeout the rssi
} __packed_gcc rssi_avg_t;

// For each entry in the average rssi list, the timeout is increased with this value
#define VH_RSSI_TIMEOUT_PER_ENTRY 10

uint32_t vh_init(uint32_t min_interval_us,
                 uint32_t access_address,
                 uint8_t channel,
                 rbc_mesh_txpower_t tx_power);

uint32_t vh_min_interval_set(uint32_t min_interval_us);

void vh_tx_power_set(rbc_mesh_txpower_t tx_power);

uint32_t vh_rx(mesh_packet_t* p_packet, uint32_t timestamp, uint8_t rssi);

uint32_t vh_local_update(rbc_mesh_value_handle_t handle, uint8_t* data, uint8_t length);

uint32_t vh_on_timeslot_begin(void);

uint32_t vh_order_update(uint32_t time_now);

/** @brief: Make copy of payload for given handle. */
uint32_t vh_value_get(rbc_mesh_value_handle_t handle, uint8_t* data, uint16_t* length);

uint32_t vh_tx_event_set(rbc_mesh_value_handle_t handle, bool do_tx_event);

uint32_t vh_tx_event_flag_get(rbc_mesh_value_handle_t handle, bool* is_doing_tx_event);

uint32_t vh_value_enable(rbc_mesh_value_handle_t handle);

uint32_t vh_value_disable(rbc_mesh_value_handle_t handle);

uint32_t vh_value_is_enabled(rbc_mesh_value_handle_t handle, bool* p_is_enabled);

uint32_t vh_value_persistence_set(rbc_mesh_value_handle_t handle, bool persistent);

uint32_t vh_value_persistence_get(rbc_mesh_value_handle_t handle, bool* p_persistent);

/** @brief: Averages and stores the rssi, returns the average. */
int8_t   vh_average_rssi(uint8_t id, int8_t rssi);

/** @brief: Increases timeout count, and invalidates entries that timed out. */
void     vh_rssi_check_timeout();

/** @brief: Get the averaged rssi, returns 0 when no value is known. */
int8_t   vh_get_rssi(uint8_t id);

void     vh_clear_rssi_list();

rssi_avg_t* vh_get_rssi_list();

#endif /* _VERSION_HANDLER_H__ */

