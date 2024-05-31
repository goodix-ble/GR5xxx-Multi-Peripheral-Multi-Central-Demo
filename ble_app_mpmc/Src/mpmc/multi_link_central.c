/**
 *****************************************************************************************
 *
 * @file multi_central.c
 *
 * @brief multi central Implementation.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "multi_link_central.h"

#define UUID_16_BIT_BYTES                   2              /**< Length of bytes for 16 bit UUID. */
#define UUID_32_BIT_BYTES                   4              /**< Length of bytes for 32 bit UUID. */
#define UUID_128_BIT_BYTES                  16             /**< Length of bytes for 128 bit UUID. */

#define CONN_INIT_DEFAULT_INTERVAL          6
#define CONN_INIT_DEFAULT_SLAVE_LATENCY     0
#define CONN_INIT_DEFAULT_SUP_TIMEOUT       600
#define CONN_INIT_DEFAULT_CON_TIMEOUT       0

enum
{
    ML_CENTRAL_SCAN_IDEL,
    ML_CENTRAL_SCAN_STARTING,
    ML_CENTRAL_SCAN_STARTED,
    ML_CENTRAL_SCAN_STOPING,
};

typedef struct
{
    ml_central_evt_handler_t    evt_handler;
    ml_central_target_filter_t  target_filter;
    ml_central_t               *instance_ptr;
} ml_central_info_t;

typedef struct
{
    uint8_t                     scan_state;
    ble_gap_scan_param_t        scan_param;
    ble_gap_init_param_t        conn_init_param;
    uint8_t                     central_cnt;
    ml_central_info_t           central_info[ML_CENTRAL_CNT];
} ml_central_context_t;

static ml_central_context_t s_ml_central_context =
{
    .scan_state                     = ML_CENTRAL_SCAN_IDEL,
    .conn_init_param.type           = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST,
    .conn_init_param.interval_min   = CONN_INIT_DEFAULT_INTERVAL,
    .conn_init_param.interval_max   = CONN_INIT_DEFAULT_INTERVAL,
    .conn_init_param.slave_latency  = CONN_INIT_DEFAULT_SLAVE_LATENCY,
    .conn_init_param.sup_timeout    = CONN_INIT_DEFAULT_SUP_TIMEOUT,
    .conn_init_param.conn_timeout   = CONN_INIT_DEFAULT_CON_TIMEOUT,
};

static bool ml_central_filter_data_find(uint8_t *p_data, uint16_t length, uint8_t ad_type, ml_central_filter_data_t *p_filter_data)
{
    if ((p_data == NULL) || (length == 0) || (p_filter_data == NULL))
    {
        return false;
    }

    uint16_t i = 0;

    while ((i + 1) < length && p_data[i + 1] != ad_type)
    {
        i += (p_data[i] + 1);
    }

    if (i >= length)
    {
        return false;
    }

    uint16_t data_offset    = i + 2;
    uint16_t data_length    = p_data[i] ? (p_data[i] - 1) : 0;

    if (data_length == 0 || (data_offset + data_length) > length)
    {
        return false;
    }

    switch (ad_type)
    {
        case BLE_GAP_AD_TYPE_SHORTENED_NAME:
        case BLE_GAP_AD_TYPE_COMPLETE_NAME:
            if (p_filter_data->dev_name.length == data_length &&
                0 == memcmp(p_filter_data->dev_name.p_data, p_data + data_offset, data_length))
            {
                return true;
            }
            break;

        case BLE_GAP_AD_TYPE_APPEARANCE:
            if ((p_data[data_offset] | p_data[data_offset + 1] << 8) == p_filter_data->appearance)
            {
                return true;
            }
            break;

        case BLE_GAP_AD_TYPE_MORE_16_BIT_UUID:
        case BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID:
        {
            uint8_t parse_len = 0;

            while(parse_len < data_length)
            {
                if (0 == memcmp(&p_data[data_offset + parse_len], p_filter_data->svr_uuid.p_data, UUID_16_BIT_BYTES))
                {
                    return true;
                }
                parse_len += UUID_16_BIT_BYTES;
            }
            break;
        }

        case BLE_GAP_AD_TYPE_MORE_32_BIT_UUID:
        case BLE_GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID:
        {
            uint8_t parse_len = 0;

            while(parse_len < data_length)
            {
                if (0 == memcmp(&p_data[data_offset + parse_len], p_filter_data->svr_uuid.p_data, UUID_32_BIT_BYTES))
                {
                    return true;
                }
                parse_len += UUID_32_BIT_BYTES;
            }
            break;
        }


        case BLE_GAP_AD_TYPE_MORE_128_BIT_UUID:
        case BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID:
        {
            uint8_t parse_len = 0;

            while(parse_len < data_length)
            {
                if (0 == memcmp(&p_data[data_offset + parse_len], p_filter_data->svr_uuid.p_data, UUID_128_BIT_BYTES))
                {
                    return true;
                }
                parse_len += UUID_128_BIT_BYTES;
            }
            break;
        }


        default:
            return false;
    }

    return true;
}


static bool ml_central_target_match(ml_central_info_t *p_central_info, const ble_gap_evt_adv_report_t *p_adv_report)
{
    bool dev_name_match   = false;
    bool appearance_match = false;
    bool uuid_match       = false;
    bool addr_match       = false;
    bool is_match_target  = true;

    sdk_err_t ret = SDK_SUCCESS;

    if (NULL == p_central_info || NULL == p_adv_report)
    {
        return false;
    }

    if (p_central_info->target_filter.filter_type & ML_CENTRAL_NAME_FILTER)
    {
        if (ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_SHORTENED_NAME, &p_central_info->target_filter.filter_data) ||
            ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_COMPLETE_NAME, &p_central_info->target_filter.filter_data))
        {
            dev_name_match = true;
        }
    }

    if (p_central_info->target_filter.filter_type & ML_CENTRAL_APPEARANCE_FILTER)
    {
        if (ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_APPEARANCE, &p_central_info->target_filter.filter_data))
        {
            appearance_match = true;
        }
    }

    if (p_central_info->target_filter.filter_type & ML_CENTRAL_UUID_FILTER)
    {
        if (p_central_info->target_filter.filter_data.svr_uuid.length == UUID_16_BIT_BYTES)
        {
            if (ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_MORE_16_BIT_UUID, &p_central_info->target_filter.filter_data) ||
                ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID, &p_central_info->target_filter.filter_data))
            {
                uuid_match = true;
            }
        }

        if (p_central_info->target_filter.filter_data.svr_uuid.length == UUID_32_BIT_BYTES)
        {
            if (ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_MORE_32_BIT_UUID, &p_central_info->target_filter.filter_data) ||
                ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID, &p_central_info->target_filter.filter_data))
            {
                uuid_match = true;
            }
        }

        if (p_central_info->target_filter.filter_data.svr_uuid.length == UUID_128_BIT_BYTES)
        {
            if (ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_MORE_128_BIT_UUID, &p_central_info->target_filter.filter_data) ||
                ml_central_filter_data_find(p_adv_report->data, p_adv_report->length, BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID, &p_central_info->target_filter.filter_data))
            {
                uuid_match = true;
            }
        }
    }

    if (p_central_info->target_filter.filter_type & ML_CENTRAL_ADDR_FILTER)
    {
        if (0 == memcmp(&p_central_info->target_filter.filter_data.target_addr, &p_adv_report->broadcaster_addr, sizeof(ble_gap_bdaddr_t)))
        {
            addr_match = true;
        }
    }

    if (ML_CENTRAL_FILTER_ANYONE_MATCH == p_central_info->target_filter.filter_mode)
    {
        if (!dev_name_match && !appearance_match && !uuid_match && !addr_match)
        {
            is_match_target = false;
        }
    }
    else
    {
        if (p_central_info->target_filter.filter_type & ML_CENTRAL_NAME_FILTER && !dev_name_match)
        {
            is_match_target = false;
        }

        if (p_central_info->target_filter.filter_type & ML_CENTRAL_APPEARANCE_FILTER && !appearance_match)
        {
            is_match_target = false;
        }

        if (p_central_info->target_filter.filter_type & ML_CENTRAL_UUID_FILTER && !uuid_match)
        {
            is_match_target = false;
        } 

        if (p_central_info->target_filter.filter_type & ML_CENTRAL_ADDR_FILTER && !addr_match)
        {
            is_match_target = false;
        }
    }

    if (is_match_target)
    {
        p_central_info->instance_ptr->state = ML_CENTRAL_STATE_TARGET_MATCHED;

        memcpy(&s_ml_central_context.conn_init_param.peer_addr, &p_adv_report->broadcaster_addr, sizeof(ble_gap_bdaddr_t));
        ret = ml_central_scan_stop();
        if (ret)
        {
            p_central_info->instance_ptr->state = ML_CENTRAL_STATE_TARGET_FINDING;

            ml_evt_t event;

            memset(&event, 0x00, sizeof(event));

            event.type   = ML_EVT_SCAN_STOP;
            event.status = ret;

            ML_EVT_REPORT(p_central_info, event);
        }
    }

    return is_match_target;
}


static uint8_t ml_central_slot_find(uint8_t conn_idx)
{
    for (uint8_t i = 0; i < s_ml_central_context.central_cnt; i++)
    {
        if (s_ml_central_context.central_info[i].instance_ptr->link_info.conn_index == conn_idx)
        {
            return i;
        }
    }

    return 0xff;
}



static sdk_err_t ml_central_target_connect(ml_central_t *p_central, ble_gap_init_param_t *p_init_param)
{
    uint16_t ret = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, p_init_param);

    p_central->state = ret ? ML_CENTRAL_STATE_IDLE : ML_CENTRAL_STATE_TARGET_CONNECTING;

    return ret;
}

static void on_scan_start(uint8_t status)
{
    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_SCAN_START;
    event.status = status;

    s_ml_central_context.scan_state = BLE_SUCCESS == status ? ML_CENTRAL_SCAN_STARTED : ML_CENTRAL_SCAN_IDEL;

    for (uint8_t i = 0; i < s_ml_central_context.central_cnt; i++)
    {
        if (s_ml_central_context.central_info[i].instance_ptr->state == ML_CENTRAL_STATE_IDLE)
        {
            ML_EVT_REPORT(&s_ml_central_context.central_info[i], event);

            if (status == BLE_SUCCESS)
            {
                s_ml_central_context.central_info[i].instance_ptr->state =  ML_CENTRAL_STATE_TARGET_FINDING;
            }
        }
    }
}

static void on_adv_report(const ble_gap_evt_adv_report_t *p_adv_report)
{
    for (uint8_t i = 0; i < s_ml_central_context.central_cnt; i++)
    {
        if (s_ml_central_context.central_info[i].instance_ptr->state == ML_CENTRAL_STATE_TARGET_MATCHED)
        {
            return;
        }
    }

    for (uint8_t i = 0; i < s_ml_central_context.central_cnt; i++)
    {
        if (s_ml_central_context.central_info[i].instance_ptr->state == ML_CENTRAL_STATE_TARGET_FINDING)
        {
            if (ml_central_target_match(&s_ml_central_context.central_info[i], p_adv_report))
            {
                return;
            }
        }
    }
}

static void on_scan_stop(uint8_t status, ble_gap_stopped_reason_t reason)
{
    ml_evt_t    event;
    sdk_err_t   ret;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_SCAN_STOP;
    event.status = status;
    event.param.scan_stop_reason = reason;

    s_ml_central_context.scan_state = BLE_SUCCESS == status ? ML_CENTRAL_SCAN_IDEL : ML_CENTRAL_SCAN_STARTING;


    for (uint8_t i = 0; i < s_ml_central_context.central_cnt; i++)
    {
        ML_EVT_REPORT(&s_ml_central_context.central_info[i],event);

        if (BLE_SUCCESS == status)
        {
            if (s_ml_central_context.central_info[i].instance_ptr->state == ML_CENTRAL_STATE_TARGET_MATCHED)
            {
                ret = ml_central_target_connect(s_ml_central_context.central_info[i].instance_ptr, &s_ml_central_context.conn_init_param);
                if (ret)
                {
                    event.type   = ML_EVT_CONNECT;
                    event.status = ret;
                    ML_EVT_REPORT(&s_ml_central_context.central_info[i], event);
                }
            }
            else if (ML_CENTRAL_STATE_TARGET_FINDING == s_ml_central_context.central_info[i].instance_ptr->state)
            {
                s_ml_central_context.central_info[i].instance_ptr->state = ML_CENTRAL_STATE_IDLE;
            }
        }
    }
}

static void  on_connect(uint8_t status, uint8_t conn_idx, const ble_gap_evt_connected_t *p_conn)
{
    if (BLE_GAP_LL_ROLE_MASTER != p_conn->ll_role)
    {
        return;
    }

    ml_evt_t    event;
    uint8_t     central_idx = ML_INVALID_IDX;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_CONNECT;
    event.status = status;

    for (uint8_t i = 0; i < s_ml_central_context.central_cnt; i++)
    {
        if (s_ml_central_context.central_info[i].instance_ptr->state == ML_CENTRAL_STATE_TARGET_CONNECTING)
        {
            central_idx = i;
            break;
        }
    }

    if (central_idx >= s_ml_central_context.central_cnt)
    {
        return;
    }

    s_ml_central_context.central_info[central_idx].instance_ptr->state = BLE_SUCCESS == status ? ML_CENTRAL_STATE_TARGET_CONNECTED : ML_CENTRAL_STATE_IDLE;

    if (BLE_SUCCESS == status)
    {
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.conn_index          = conn_idx;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.conn_interval       = p_conn->conn_interval;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.slave_latency       = p_conn->slave_latency; 
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.sup_timeout         = p_conn->sup_timeout; 
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.peer_addr.addr_type = p_conn->peer_addr_type;
        memcpy(s_ml_central_context.central_info[central_idx].instance_ptr->link_info.peer_addr.gap_addr.addr,  p_conn->peer_addr.addr, 6);

        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.gatt_mtu     = ML_MTU_DEFAULT;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.rx_data_len  = ML_DATA_LEN_DEFAULT;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.tx_data_len  = ML_DATA_LEN_DEFAULT;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.tx_phy       = BLE_GAP_PHY_LE_1MBPS;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.rx_phy       = BLE_GAP_PHY_LE_1MBPS;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.encrypted    = false;
    }

    ML_EVT_REPORT(&s_ml_central_context.central_info[central_idx], event);

    for (uint8_t i = 0; i < s_ml_central_context.central_cnt; i++)
    {
        if (s_ml_central_context.central_info[i].instance_ptr->state != ML_CENTRAL_STATE_TARGET_CONNECTED)
        {
            ml_central_scan_start(&s_ml_central_context.scan_param);
            break;
        }
    } 
}

static void on_disconnect(uint8_t status, uint8_t conn_idx, uint8_t reason)
{
    uint8_t central_idx = ml_central_slot_find(conn_idx);

    if (central_idx >= s_ml_central_context.central_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_DISCONNECT;
    event.status = status;
    event.param.disconn_reason = reason;

    if (status == BLE_SUCCESS)
    {
        s_ml_central_context.central_info[central_idx].instance_ptr->state = ML_CENTRAL_STATE_IDLE;

        ML_LINK_INFO_RESET(s_ml_central_context.central_info[central_idx].instance_ptr->link_info);

        if (ML_CENTRAL_SCAN_STARTED == s_ml_central_context.scan_state)
        {
            s_ml_central_context.central_info[central_idx].instance_ptr->state = ML_CENTRAL_STATE_TARGET_FINDING;
        }
        else
        {
            ml_central_scan_start(&s_ml_central_context.scan_param);
        }
    }

    ML_EVT_REPORT(&s_ml_central_context.central_info[central_idx], event);
}

static void on_mtu_exchange(uint8_t status, uint8_t conn_idx, uint16_t mtu)
{
    uint8_t central_idx = ml_central_slot_find(conn_idx);

    if (central_idx >= s_ml_central_context.central_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_MTU_EXCHANGE;
    event.status = status;

    if (status == BLE_SUCCESS)
    {
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.gatt_mtu = mtu;
    }

    ML_EVT_REPORT(&s_ml_central_context.central_info[central_idx], event);
}

static void on_encrypt_req(uint8_t status, uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    uint8_t central_idx = ml_central_slot_find(conn_idx);

    if (central_idx >= s_ml_central_context.central_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_ENCRYPT_REQ;
    event.status = status;
    memcpy(&event.param.encrypt_req, p_enc_req, sizeof(ble_sec_evt_enc_req_t));

    ML_EVT_REPORT(&s_ml_central_context.central_info[central_idx], event);
}

static void on_encrypt_ind(uint8_t status, uint8_t conn_idx, const ble_sec_evt_enc_ind_t  *p_enc_ind)
{
    uint8_t central_idx = ml_central_slot_find(conn_idx);

    if (central_idx >= s_ml_central_context.central_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_ENCRYPT_IND;
    event.status = status;
    memcpy(&event.param.encrypt_ind, p_enc_ind, sizeof(ble_sec_evt_enc_ind_t));

    if (status == BLE_SUCCESS)
    {
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.encrypted = true;
    }

    ML_EVT_REPORT(&s_ml_central_context.central_info[central_idx], event);
}

static void on_phy_update(uint8_t status, uint8_t conn_idx, const ble_gap_evt_phy_update_t *p_phy_update)
{
    uint8_t central_idx = ml_central_slot_find(conn_idx);

    if (central_idx >= s_ml_central_context.central_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_PHY_UPDATE;
    event.status = status;

    if (status == BLE_SUCCESS)
    {
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.tx_phy = p_phy_update->tx_phy;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.rx_phy = p_phy_update->rx_phy;
    }

    ML_EVT_REPORT(&s_ml_central_context.central_info[central_idx], event);
}

static void on_data_len_update(uint8_t status, uint8_t conn_idx, const ble_gap_evt_data_length_t *data_length)
{
    uint8_t central_idx = ml_central_slot_find(conn_idx);

    if (central_idx >= s_ml_central_context.central_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_DATA_LEN_UPDATE;
    event.status = status;

    if (status == BLE_SUCCESS)
    {
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.tx_data_len = data_length->max_tx_octets;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.rx_data_len = data_length->max_rx_octets;
    }

    ML_EVT_REPORT(&s_ml_central_context.central_info[central_idx], event);
}

static void on_conn_param_update(uint8_t status, uint8_t conn_idx, const ble_gap_evt_conn_param_updated_t *p_conn_param_updated)
{
    uint8_t central_idx = ml_central_slot_find(conn_idx);

    if (central_idx >= s_ml_central_context.central_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_CONN_PARAM_UPDATE;
    event.status = status;

    if (status == BLE_SUCCESS)
    {
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.conn_interval = p_conn_param_updated->conn_interval;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.slave_latency = p_conn_param_updated->slave_latency;
        s_ml_central_context.central_info[central_idx].instance_ptr->link_info.sup_timeout   = p_conn_param_updated->sup_timeout;
    }

    ML_EVT_REPORT(&s_ml_central_context.central_info[central_idx], event);
}

sdk_err_t ml_central_create(ml_central_t *p_central, ml_central_init_t *p_init)
{
    if (NULL == p_central || NULL == p_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if (s_ml_central_context.central_cnt >= ML_CENTRAL_CNT)
    {
        return SDK_ERR_NO_RESOURCES;
    }

    memset(p_central, 0x00, sizeof(ml_central_t));
    ML_LINK_INFO_RESET(p_central->link_info);

    s_ml_central_context.central_info[s_ml_central_context.central_cnt].instance_ptr = p_central;
    s_ml_central_context.central_info[s_ml_central_context.central_cnt].evt_handler  = p_init->evt_handler;
    memcpy(&s_ml_central_context.central_info[s_ml_central_context.central_cnt].target_filter, &p_init->target_filter, sizeof(ml_central_target_filter_t));

    s_ml_central_context.central_cnt++;

    return SDK_SUCCESS;
}

sdk_err_t ml_central_scan_start(ble_gap_scan_param_t *p_param)
{
    sdk_err_t ret = SDK_SUCCESS;

    if (NULL == p_param)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if (s_ml_central_context.scan_state != ML_CENTRAL_SCAN_IDEL)
    {
        return SDK_ERR_DISALLOWED;
    }

    for (uint8_t i = 0; i < ML_CENTRAL_CNT; i++)
    {
        if (s_ml_central_context.central_info[i].instance_ptr->state == ML_CENTRAL_STATE_TARGET_CONNECTING)
        {
            return SDK_ERR_DISALLOWED;
        }
    }

    memcpy(&s_ml_central_context.scan_param, p_param, sizeof(ble_gap_scan_param_t));

    ret = ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, p_param);
    if (ret == SDK_SUCCESS)
    {
        ret = ble_gap_scan_start();
    }

    if (ret == SDK_SUCCESS)
    {
        s_ml_central_context.scan_state = ML_CENTRAL_SCAN_STARTING;
    }

    return ret;
}

sdk_err_t ml_central_scan_stop(void)
{
    sdk_err_t ret = SDK_SUCCESS;

    if (ML_CENTRAL_SCAN_STARTED != s_ml_central_context.scan_state)
    {
        return SDK_ERR_DISALLOWED;
    }

    ret = ble_gap_scan_stop();

    if (ret == SDK_SUCCESS)
    {
        s_ml_central_context.scan_state = ML_CENTRAL_SCAN_STOPING;
    }

    return ret;
}


void ml_central_evt_handler(const ble_evt_t *p_evt)
{
    switch (p_evt->evt_id)
    {
        case BLE_GAPM_EVT_SCAN_START:
            on_scan_start(p_evt->evt_status);
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            on_scan_stop(p_evt->evt_status, p_evt->evt.gapm_evt.params.scan_stop.reason);
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            on_adv_report(&p_evt->evt.gapm_evt.params.adv_report);
            break;

        case BLE_GAPC_EVT_CONNECTED:
            on_connect(p_evt->evt_status, p_evt->evt.gapc_evt.index, &p_evt->evt.gapc_evt.params.connected);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            on_disconnect(p_evt->evt_status, p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            on_mtu_exchange(p_evt->evt_status, p_evt->evt.gatt_common_evt.index, p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            on_encrypt_req(p_evt->evt_status, p_evt->evt.sec_evt.index, &p_evt->evt.sec_evt.params.enc_req);
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            on_encrypt_ind(p_evt->evt_status, p_evt->evt.sec_evt.index, &p_evt->evt.sec_evt.params.enc_ind);
            break;

        case BLE_GAPC_EVT_PHY_UPDATED:
            on_phy_update(p_evt->evt_status, p_evt->evt.gapc_evt.index, &p_evt->evt.gapc_evt.params.phy_update);
            break;

        case BLE_GAPC_EVT_DATA_LENGTH_UPDATED:
            on_data_len_update(p_evt->evt_status, p_evt->evt.gapc_evt.index, &p_evt->evt.gapc_evt.params.data_length);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATED:
            on_conn_param_update(p_evt->evt_status, p_evt->evt.gapc_evt.index, &p_evt->evt.gapc_evt.params.conn_param_updated);
            break;
    }
}


