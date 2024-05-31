

#ifndef __MULTI_LINK_H__
#define __MULTI_LINK_H__

#include "custom_config.h"
#include "gr_includes.h"


#define ML_PERIPHERAL_CNT               2
#define ML_CENTRAL_CNT                  2

#if (ML_PERIPHERAL_CNT + ML_CENTRAL_CNT) > CFG_MAX_CONNECTIONS
#error "(MPMC_PERIPHERAL_CNT + MPMC_CENTRAL_CNT) > CFG_MAX_CONNECTIONS(in custom_config.h)"
#endif


#define ML_EVT_REPORT(p_inst_info, event)               \
do                                                      \
{                                                       \
    if ((p_inst_info) && (p_inst_info)->evt_handler)    \
    {                                                   \
        (p_inst_info)->evt_handler(&event);             \
    }                                                   \
} while(0)

#define ML_LINK_INFO_RESET(link_info)                   \
do                                                      \
{                                                       \
    memset(&link_info, 0x00, sizeof(link_info));        \
    link_info.conn_index = BLE_GAP_INVALID_CONN_INDEX;  \
} while(0)

#define ML_INVALID_IDX                  0xff
#define ML_MTU_DEFAULT                  23 
#define ML_DATA_LEN_DEFAULT             27
#define ML_PERIPHERAL_ADV_IDX           0

typedef enum
{
    ML_EVT_INVALID,
    ML_EVT_SCAN_START,
    ML_EVT_SCAN_STOP,
    ML_EVT_ADV_START,
    ML_EVT_ADV_STOP,
    ML_EVT_CONNECT,
    ML_EVT_DISCONNECT,
    ML_EVT_MTU_EXCHANGE,
    ML_EVT_ENCRYPT_REQ,
    ML_EVT_ENCRYPT_IND,
    ML_EVT_PHY_UPDATE,
    ML_EVT_DATA_LEN_UPDATE,
    ML_EVT_CONN_PARAM_UPDATE,
} ml_evt_type_t;


typedef struct
{
    const uint8_t *p_data;                  /**< Pointer to the data. */
    uint16_t       length;                  /**< Length of the data. */
} ml_data_t;

typedef struct
{ 
    ml_evt_type_t   type;
    uint8_t         status;
    union
    {
        ble_gap_evt_adv_report_t    rec_adv_report;
        ble_gap_stopped_reason_t    scan_stop_reason;
        ble_gap_stopped_reason_t    adv_stop_reason;
        uint8_t                     disconn_reason;
        ble_sec_evt_enc_req_t       encrypt_req;
        ble_sec_evt_enc_ind_t       encrypt_ind;
    } param;
} ml_evt_t;

typedef struct
{
    uint8_t             conn_index;
    uint16_t            conn_interval;
    uint16_t            slave_latency;
    uint16_t            sup_timeout;
    ble_gap_bdaddr_t    peer_addr;

    uint8_t             tx_phy;
    uint8_t             rx_phy;
    uint16_t            tx_data_len;
    uint16_t            rx_data_len;
    uint16_t            gatt_mtu;

    bool                encrypted;
} ml_link_info_t;




#endif


