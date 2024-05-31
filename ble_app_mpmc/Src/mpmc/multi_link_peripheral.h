/**
 *****************************************************************************************
 *
 * @file multi_peripheral.h
 *
 * @brief Header file - multi peripheral Function
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

#ifndef __MULTI_PERIPHERAL_H__
#define __MULTI_PERIPHERAL_H__

#include "multi_link.h"

typedef enum
{
    ML_PERIPHERAL_STATE_IDLE,
    ML_PERIPHERAL_STATE_ADVERTISING,
    ML_PERIPHERAL_STATE_CONNECTED, 
} ml_peripheral_state_t;


typedef void (*ml_peripheral_evt_handler_t)(ml_evt_t *p_evt);

typedef struct
{
    ml_peripheral_state_t   state;
    ml_link_info_t          link_info;
} ml_peripheral_t;

typedef struct
{
    uint8_t                       *adv_data;
    uint8_t                       *scan_data;
    uint8_t                        adv_data_len;
    uint8_t                        scan_data_len;
    ble_gap_adv_param_t            adv_param;
    ble_gap_adv_time_param_t       adv_time_param;
    ml_peripheral_evt_handler_t    evt_handler;
} ml_peripheral_init_t;

sdk_err_t ml_peripheral_create(ml_peripheral_t *p_central, ml_peripheral_init_t *p_init);

sdk_err_t ml_peripheral_adv_start(void);

void ml_peripheral_evt_handler(const ble_evt_t *p_evt);
#endif

