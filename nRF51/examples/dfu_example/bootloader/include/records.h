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

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


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

#ifndef __RECORDS_H__
#define __RECORDS_H__

#include "dfu_mesh.h"
#include <stdint.h>
#include <stdbool.h>

#define DFU_RECORD_SIZE     (16)

#define PAGE_INDEX(full_addr)       ((full_addr) >> 10)

typedef struct 
{
    uint16_t seq_num;
    uint16_t short_addr;
    uint8_t data[DFU_RECORD_SIZE];
} dfu_record_t;

uint32_t records_init(uint32_t missing_record_pool_size);
void records_record_add(dfu_record_t* p_record, bool was_missing);
bool records_record_get(uint16_t seq_num, dfu_record_t* p_record);
void records_flash_page(uint32_t page_index);
void records_flash(void);
void records_missing_report(uint16_t seq_num);
bool records_missing_in_range(uint16_t lowest_seq_num, uint16_t highest_seq_num);
bool records_is_missing(uint16_t seq_num);
uint16_t records_missing_get(void);
void records_clear(void);

#endif /* __RECORDS_H__ */
