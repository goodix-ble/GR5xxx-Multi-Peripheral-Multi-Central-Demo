
#include "user_gus_peripheral.h"

#include "multi_link_peripheral.h"

#include "gus.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"


#define APP_ADV_INTERVAL        48

static ml_peripheral_t  s_gus_peripheral;


static uint8_t s_gus_peripheral_adv_data[] =
{
    0x11, // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    GUS_SERVICE_UUID,

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7, 
    0x04,
    // Goodix specific adv data
    0x02,0x03,
};

static uint8_t s_gus_peripheral_adv_rsp_data[] =
{
    0x0c, // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'U', 'A', 'R', 'T',
};

static void gus_peripheral_evt_handler(ml_evt_t *p_evt)
{
#define AssignEventName(type) \
    case type: \
        event_name = #type; \
        break

    const char* event_name;
    switch (p_evt->type) {
        AssignEventName(ML_EVT_ADV_START);
        AssignEventName(ML_EVT_ADV_STOP);
        AssignEventName(ML_EVT_CONNECT);
        AssignEventName(ML_EVT_DISCONNECT);
        AssignEventName(ML_EVT_MTU_EXCHANGE);
        AssignEventName(ML_EVT_ENCRYPT_REQ);
        AssignEventName(ML_EVT_ENCRYPT_IND);
        AssignEventName(ML_EVT_PHY_UPDATE);
        AssignEventName(ML_EVT_DATA_LEN_UPDATE);
        AssignEventName(ML_EVT_CONN_PARAM_UPDATE);
        default:
            event_name = "Unknown";
            break;
    }
#undef AssignEventName

    if (p_evt->type == ML_EVT_ENCRYPT_REQ)
    {
        ble_sec_cfm_enc_t cfm_enc;

        memset((uint8_t *)&cfm_enc, 0, sizeof(ble_sec_cfm_enc_t));

        cfm_enc.req_type = p_evt->param.encrypt_req.req_type;
        cfm_enc.accept   = false;


        ble_sec_enc_cfm(s_gus_peripheral.link_info.conn_index, &cfm_enc);
    }

    APP_LOG_INFO("##################### GUS peripheral ###################");
    APP_LOG_INFO("event: %s", event_name);
    APP_LOG_INFO(" -state:          %d", s_gus_peripheral.state);
    APP_LOG_INFO(" -conn_index:     %d", s_gus_peripheral.link_info.conn_index);
    APP_LOG_INFO(" -conn_interval:  %d", s_gus_peripheral.link_info.conn_interval);
    APP_LOG_INFO(" -slave_latency:  %d", s_gus_peripheral.link_info.slave_latency);
    APP_LOG_INFO(" -sup_timeout:    %d", s_gus_peripheral.link_info.sup_timeout);
    APP_LOG_INFO(" -peer_addr:      %02X:%02X:%02X:%02X:%02X:%02X", s_gus_peripheral.link_info.peer_addr.gap_addr.addr[5],
                                                                    s_gus_peripheral.link_info.peer_addr.gap_addr.addr[4],
                                                                    s_gus_peripheral.link_info.peer_addr.gap_addr.addr[3],
                                                                    s_gus_peripheral.link_info.peer_addr.gap_addr.addr[2],
                                                                    s_gus_peripheral.link_info.peer_addr.gap_addr.addr[1],
                                                                    s_gus_peripheral.link_info.peer_addr.gap_addr.addr[0]);
    APP_LOG_INFO(" -tx_phy:         %d", s_gus_peripheral.link_info.tx_phy);
    APP_LOG_INFO(" -rx_phy:         %d", s_gus_peripheral.link_info.rx_phy);
    APP_LOG_INFO(" -tx_data_len:    %d", s_gus_peripheral.link_info.tx_data_len);
    APP_LOG_INFO(" -rx_data_len:    %d", s_gus_peripheral.link_info.rx_data_len);
    APP_LOG_INFO(" -gatt_mtu:       %d", s_gus_peripheral.link_info.gatt_mtu);
    APP_LOG_INFO(" -encrypted:      %d", s_gus_peripheral.link_info.encrypted);
    APP_LOG_INFO("########################################################\r\n\r\n");
}

static void gus_service_process_event(gus_evt_t *p_evt)
{
    if (p_evt->conn_idx != s_gus_peripheral.link_info.conn_index)
    {
        return;
    }

    switch (p_evt->evt_type)
    {
        case GUS_EVT_TX_PORT_OPENED:
            APP_LOG_DEBUG("GUS TX Notification is enabled.");
            break;

        case GUS_EVT_TX_PORT_CLOSED:
            APP_LOG_DEBUG("GUS TX Notification is disabled.");
            break;

        case GUS_EVT_RX_DATA_RECEIVED:
            APP_LOG_HEX_DUMP(p_evt->p_data, p_evt->length);
            break;

        default:
            break;
    }
}

static void user_gus_peripheralservices_init(void)
{
    sdk_err_t   error_code;
    gus_init_t gus_init;

    gus_init.evt_handler = gus_service_process_event;

    error_code = gus_service_init(&gus_init);
    APP_ERROR_CHECK(error_code);
}

sdk_err_t user_gus_peripheral_init(void)
{
    ml_peripheral_init_t    gus_peripheral_init;

    memset(&gus_peripheral_init, 0x00, sizeof(ml_peripheral_init_t));

    gus_peripheral_init.evt_handler     = gus_peripheral_evt_handler;
    gus_peripheral_init.adv_data        = s_gus_peripheral_adv_data;
    gus_peripheral_init.scan_data       = s_gus_peripheral_adv_rsp_data;
    gus_peripheral_init.adv_data_len    = sizeof(s_gus_peripheral_adv_data);
    gus_peripheral_init.scan_data_len   = sizeof(s_gus_peripheral_adv_rsp_data);

    gus_peripheral_init.adv_param.adv_intv_max = APP_ADV_INTERVAL;
    gus_peripheral_init.adv_param.adv_intv_min = APP_ADV_INTERVAL;
    gus_peripheral_init.adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    gus_peripheral_init.adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    gus_peripheral_init.adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    gus_peripheral_init.adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    gus_peripheral_init.adv_time_param.duration     = 0;
    gus_peripheral_init.adv_time_param.max_adv_evt  = 0;

    user_gus_peripheralservices_init();

    return  ml_peripheral_create(&s_gus_peripheral, &gus_peripheral_init);
}
