/**
 *****************************************************************************************
 *
 * @file multi_central.h
 *
 * @brief Header file - multi central Function
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

#ifndef __MULTI_CENTRAL_H__
#define __MULTI_CENTRAL_H__

#include "multi_link.h"


#define ML_CENTRAL_NAME_FILTER             (0x01 << 0)          /**< Filter device base on target device name. */
#define ML_CENTRAL_APPEARANCE_FILTER       (0x01 << 1)          /**< Filter device base on target appearance. */
#define ML_CENTRAL_UUID_FILTER             (0x01 << 2)          /**< Filter device base on tagert service UUID. */
#define ML_CENTRAL_ADDR_FILTER             (0x01 << 3)          /**< Filter device base on target address. */

typedef enum
{
    ML_CENTRAL_FILTER_ALL_MATCH,           /**< Only when all type match. */
    ML_CENTRAL_FILTER_ANYONE_MATCH,        /**< Just match any type . */
} ml_central_filter_mode_t;

typedef enum
{
    ML_CENTRAL_STATE_IDLE,
    ML_CENTRAL_STATE_TARGET_FINDING,
    ML_CENTRAL_STATE_TARGET_MATCHED,
    ML_CENTRAL_STATE_TARGET_CONNECTING,
    ML_CENTRAL_STATE_TARGET_CONNECTED, 
} ml_central_state_t;


typedef struct
{
    ble_gap_bdaddr_t    target_addr;  /**< Target device address. */
    uint16_t            appearance;   /**< Target appearance. */
    ml_data_t           svr_uuid;     /**< Target service UUID. */
    ml_data_t           dev_name;     /**< Target device name. */
} ml_central_filter_data_t;

typedef struct
{
    uint16_t                    filter_type;
    ml_central_filter_mode_t    filter_mode;
    ml_central_filter_data_t    filter_data;
} ml_central_target_filter_t;


typedef void (*ml_central_evt_handler_t)(ml_evt_t *p_evt);

typedef struct
{
    ml_central_state_t  state;
    ml_link_info_t      link_info;
} ml_central_t;

typedef struct
{
    ml_central_evt_handler_t    evt_handler;
    ml_central_target_filter_t  target_filter;
} ml_central_init_t;

sdk_err_t ml_central_create(ml_central_t *p_central, ml_central_init_t *p_init);

sdk_err_t ml_central_scan_start(ble_gap_scan_param_t *p_param);

sdk_err_t ml_central_scan_stop(void);

void ml_central_evt_handler(const ble_evt_t *p_evt);
#endif

