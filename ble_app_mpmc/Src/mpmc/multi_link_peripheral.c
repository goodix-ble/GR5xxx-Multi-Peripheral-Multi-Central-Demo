/**
 *****************************************************************************************
 *
 * @file multi_peripheral.c
 *
 * @brief multi peripheral Implementation.
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
#include "multi_link_peripheral.h"

enum
{
    ML_PERIPHERAL_ADV_IDEL,
    ML_PERIPHERAL_ADV_STARTING,
    ML_PERIPHERAL_ADV_STARTED,
    ML_PERIPHERAL_ADV_STOPING,
};

typedef struct
{
    uint8_t                        adv_data[31];
    uint8_t                        scan_data[31];
    uint8_t                        adv_data_len;
    uint8_t                        scan_data_len;
    ble_gap_adv_param_t            adv_param;
    ble_gap_adv_time_param_t       adv_time_param;
    ml_peripheral_evt_handler_t    evt_handler;
    ml_peripheral_t               *instance_ptr;
} ml_peripheral_info_t;

typedef struct
{
    uint8_t                     adv_state;
    uint8_t                     adv_peripheral_idx;
    uint8_t                     peripheral_cnt;
    ml_peripheral_info_t        peripheral_info[ML_PERIPHERAL_CNT];
} ml_peripheral_context_t;

static ml_peripheral_context_t s_ml_peripheral_context = 
{
    .adv_state          = ML_PERIPHERAL_ADV_IDEL,
    .adv_peripheral_idx = ML_INVALID_IDX,
};


static uint8_t ml_peripheral_slot_find(uint8_t conn_idx)
{
    for (uint8_t i = 0; i < s_ml_peripheral_context.peripheral_cnt; i++)
    {
        if (s_ml_peripheral_context.peripheral_info[i].instance_ptr->link_info.conn_index == conn_idx)
        {
            return i;
        }
    }

    return 0xff;
}

static uint8_t ml_peripheral_need_adv_get(void)
{
    uint8_t  peripheral_idx = s_ml_peripheral_context.adv_peripheral_idx;

    if (peripheral_idx == ML_INVALID_IDX)
    {
        peripheral_idx = 0;
    }

    for (uint8_t i = 0; i < s_ml_peripheral_context.peripheral_cnt; i++)
    {
        if (s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->state == ML_PERIPHERAL_STATE_CONNECTED)
        {
            peripheral_idx = peripheral_idx + 1 == s_ml_peripheral_context.peripheral_cnt ?  0 : peripheral_idx + 1;
            continue;
        }
        else
        {
            s_ml_peripheral_context.adv_peripheral_idx = peripheral_idx;
            return peripheral_idx;
        }
    }

    return ML_INVALID_IDX;
}

static void on_adv_start(uint8_t index,  uint8_t status)
{
    if (index != ML_PERIPHERAL_ADV_IDX)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type     = ML_EVT_ADV_START;
    event.status   = status;

    s_ml_peripheral_context.adv_state = BLE_SUCCESS == status ? ML_PERIPHERAL_ADV_STARTED : ML_PERIPHERAL_ADV_IDEL;

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.adv_peripheral_idx], event);
}

static void on_adv_stop(uint8_t index, uint8_t status, ble_gap_stopped_reason_t reason)
{
    if (index != ML_PERIPHERAL_ADV_IDX)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type     = ML_EVT_ADV_STOP;
    event.status   = status;
    event.param.adv_stop_reason = reason;

    if (status == BLE_SUCCESS)
    {
        s_ml_peripheral_context.adv_state = ML_PERIPHERAL_ADV_IDEL;
    }

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.adv_peripheral_idx], event);


    if (status == BLE_SUCCESS && reason == BLE_GAP_STOPPED_REASON_CONN_EST)
    {
         s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.adv_peripheral_idx].instance_ptr->state = ML_PERIPHERAL_STATE_CONNECTED;
    }

    if (reason == BLE_GAP_STOPPED_REASON_TIMEOUT || reason == BLE_GAP_STOPPED_REASON_CONN_EST)
    {
        ml_peripheral_adv_start();
    }
}

static void on_connect(uint8_t status, uint8_t conn_idx, const ble_gap_evt_connected_t *p_conn)
{
    if (BLE_GAP_LL_ROLE_SLAVE != p_conn->ll_role)
    {
        return;
    }

    ml_evt_t    event;
    uint8_t     peripheral_idx = s_ml_peripheral_context.adv_peripheral_idx;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_CONNECT;
    event.status = status;

    s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->state = BLE_SUCCESS == status ? ML_PERIPHERAL_STATE_CONNECTED : ML_PERIPHERAL_STATE_IDLE;

    if (BLE_SUCCESS == status)
    {
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.conn_index          = conn_idx;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.conn_interval       = p_conn->conn_interval;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.slave_latency       = p_conn->slave_latency; 
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.sup_timeout         = p_conn->sup_timeout; 
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.peer_addr.addr_type = p_conn->peer_addr_type;
        memcpy(s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.peer_addr.gap_addr.addr,  p_conn->peer_addr.addr, 6);

        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.gatt_mtu     = ML_MTU_DEFAULT;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.tx_data_len  = ML_DATA_LEN_DEFAULT;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.rx_data_len  = ML_DATA_LEN_DEFAULT;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.tx_phy       = BLE_GAP_PHY_LE_1MBPS;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.rx_phy       = BLE_GAP_PHY_LE_1MBPS;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.encrypted    = false;
    }

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[peripheral_idx], event);
}


static void on_disconnect(uint8_t status, uint8_t conn_idx, uint8_t reason)
{
    uint8_t peripheral_idx = ml_peripheral_slot_find(conn_idx);

    if (peripheral_idx >= s_ml_peripheral_context.peripheral_cnt)
    {
        return;
    }

    if (status == BLE_SUCCESS)
    {
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->state = ML_PERIPHERAL_STATE_IDLE;
        ML_LINK_INFO_RESET(s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info);
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_DISCONNECT;
    event.status = status;
    event.param.disconn_reason = reason;

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[peripheral_idx], event);

    ml_peripheral_adv_start();
}

static void on_mtu_exchange(uint8_t status, uint8_t conn_idx, uint16_t mtu)
{
    uint8_t peripheral_idx = ml_peripheral_slot_find(conn_idx);

    if (peripheral_idx >= s_ml_peripheral_context.peripheral_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_MTU_EXCHANGE;
    event.status = status;

    if (status == BLE_SUCCESS)
    {
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.gatt_mtu = mtu;
    }

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[peripheral_idx], event);
}

static void on_encrypt_req(uint8_t status, uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    uint8_t peripheral_idx = ml_peripheral_slot_find(conn_idx);

    if (peripheral_idx >= s_ml_peripheral_context.peripheral_cnt)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_ENCRYPT_REQ;
    event.status = status;
    memcpy(&event.param.encrypt_req, p_enc_req, sizeof(ble_sec_evt_enc_req_t));

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[peripheral_idx], event);
}

static void on_encrypt_ind(uint8_t status, uint8_t conn_idx, const ble_sec_evt_enc_ind_t  *p_enc_ind)
{
    uint8_t peripheral_idx = ml_peripheral_slot_find(conn_idx);

    if (peripheral_idx >= s_ml_peripheral_context.peripheral_cnt)
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
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.encrypted = true;
    }

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[peripheral_idx], event);
} 

static void on_phy_update(uint8_t status, uint8_t conn_idx, const ble_gap_evt_phy_update_t *p_phy_update)
{
    uint8_t peripheral_idx = ml_peripheral_slot_find(conn_idx);

    if (peripheral_idx >= ML_CENTRAL_CNT)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_PHY_UPDATE;
    event.status = status;

    if (status == BLE_SUCCESS)
    {
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.tx_phy = p_phy_update->tx_phy;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.rx_phy = p_phy_update->rx_phy;
    }

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[peripheral_idx], event);
}

static void on_data_len_update(uint8_t status, uint8_t conn_idx, const ble_gap_evt_data_length_t *data_length)
{
    uint8_t peripheral_idx = ml_peripheral_slot_find(conn_idx);

    if (peripheral_idx >= ML_CENTRAL_CNT)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_DATA_LEN_UPDATE;
    event.status = status;

    if (status == BLE_SUCCESS)
    {
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.tx_data_len = data_length->max_tx_octets;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.rx_data_len = data_length->max_rx_octets;
    }

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[peripheral_idx], event);
}

static void on_conn_param_update(uint8_t status, uint8_t conn_idx, const ble_gap_evt_conn_param_updated_t *p_conn_param_updated)
{
    uint8_t peripheral_idx = ml_peripheral_slot_find(conn_idx);

    if (peripheral_idx >= ML_CENTRAL_CNT)
    {
        return;
    }

    ml_evt_t event;

    memset(&event, 0x00, sizeof(event));

    event.type   = ML_EVT_CONN_PARAM_UPDATE;
    event.status = status;

    if (status == BLE_SUCCESS)
    {
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.conn_interval = p_conn_param_updated->conn_interval;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.slave_latency = p_conn_param_updated->slave_latency;
        s_ml_peripheral_context.peripheral_info[peripheral_idx].instance_ptr->link_info.sup_timeout   = p_conn_param_updated->sup_timeout;
    }

    ML_EVT_REPORT(&s_ml_peripheral_context.peripheral_info[peripheral_idx], event);
}

sdk_err_t ml_peripheral_create(ml_peripheral_t *p_peripheral, ml_peripheral_init_t *p_init)
{
    if (NULL == p_peripheral || NULL == p_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    if (s_ml_peripheral_context.peripheral_cnt >= ML_PERIPHERAL_CNT)
    {
        return SDK_ERR_NO_RESOURCES;
    }

    memset(p_peripheral, 0x00, sizeof(ml_peripheral_t));
    ML_LINK_INFO_RESET(p_peripheral->link_info);

    s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.peripheral_cnt].instance_ptr = p_peripheral;
    s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.peripheral_cnt].evt_handler   = p_init->evt_handler;
    s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.peripheral_cnt].adv_data_len  = p_init->adv_data_len;
    s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.peripheral_cnt].scan_data_len = p_init->scan_data_len;
    memcpy(&s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.peripheral_cnt].adv_data, p_init->adv_data, p_init->adv_data_len); 
    memcpy(&s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.peripheral_cnt].scan_data, p_init->scan_data, p_init->scan_data_len);
    memcpy(&s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.peripheral_cnt].adv_param, &p_init->adv_param, sizeof(ble_gap_adv_param_t));
    memcpy(&s_ml_peripheral_context.peripheral_info[s_ml_peripheral_context.peripheral_cnt].adv_time_param, &p_init->adv_time_param, sizeof(ble_gap_adv_time_param_t));

    s_ml_peripheral_context.peripheral_cnt++;

    return SDK_SUCCESS;
}

sdk_err_t ml_peripheral_adv_start(void)
{
    sdk_err_t ret;

    if (s_ml_peripheral_context.adv_state != ML_PERIPHERAL_ADV_IDEL)
    {
        return SDK_ERR_DISALLOWED;
    }

    uint8_t peripheral_idx = ml_peripheral_need_adv_get();

    if (peripheral_idx >= s_ml_peripheral_context.peripheral_cnt)
    {
        return SDK_ERR_DISALLOWED;
    }

    ret = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_ml_peripheral_context.peripheral_info[peripheral_idx].adv_param);
    if (ret == SDK_SUCCESS)
    {
        ret = ble_gap_adv_data_set(0,
                                   BLE_GAP_ADV_DATA_TYPE_DATA,
                                   s_ml_peripheral_context.peripheral_info[peripheral_idx].adv_data, 
                                   s_ml_peripheral_context.peripheral_info[peripheral_idx].adv_data_len);
    }

    
    if (ret == SDK_SUCCESS && s_ml_peripheral_context.peripheral_info[peripheral_idx].scan_data_len)
    {
        ret = ble_gap_adv_data_set(0,
                                   BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                   s_ml_peripheral_context.peripheral_info[peripheral_idx].scan_data, 
                                   s_ml_peripheral_context.peripheral_info[peripheral_idx].scan_data_len);
    }

    if (ret == SDK_SUCCESS)
    {
        ret = ble_gap_adv_start(0, &s_ml_peripheral_context.peripheral_info[peripheral_idx].adv_time_param);
    }

    if (ret == SDK_SUCCESS)
    {
        s_ml_peripheral_context.adv_state = ML_PERIPHERAL_ADV_STARTING;
    }

    return ret;
}


void ml_peripheral_evt_handler(const ble_evt_t *p_evt)
{
    switch (p_evt->evt_id)
    {
        case BLE_GAPM_EVT_ADV_START:
            on_adv_start(p_evt->evt.gapm_evt.index, p_evt->evt_status);
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            on_adv_stop(p_evt->evt.gapm_evt.index, p_evt->evt_status, p_evt->evt.gapm_evt.params.scan_stop.reason);
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
            on_encrypt_ind(p_evt->evt_status, p_evt->evt.gapc_evt.index, &p_evt->evt.sec_evt.params.enc_ind);
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



