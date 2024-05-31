
#include "user_relay_peripheral.h"

#include "multi_link_peripheral.h"

#include "hrs.h"
#include "rscs.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"


#define APP_ADV_INTERVAL        48

static ml_peripheral_t  s_realy_peripheral;


static uint8_t s_realy_peripheral_adv_data[] =
{
    0x05,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_RUNNING_SPEED_CADENCE),
    HI_U16(BLE_ATT_SVC_RUNNING_SPEED_CADENCE),
    LO_U16(BLE_ATT_SVC_HEART_RATE),
    HI_U16(BLE_ATT_SVC_HEART_RATE),
};

static uint8_t s_realy_peripheral_adv_rsp_data[] =
{
    0x16,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'H', 'R', 'S', '_', 'R', 'S', 'C', 'S', '_', 'R', 'E', 'L', 'A', 'Y',
};

static void realy_peripheral_evt_handler(ml_evt_t *p_evt)
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

        if (p_evt->param.encrypt_req.req_type  == BLE_SEC_PAIR_REQ)
        {
            cfm_enc.req_type = BLE_SEC_PAIR_REQ;
            cfm_enc.accept   = true;
        }

        ble_sec_enc_cfm(s_realy_peripheral.link_info.conn_index, &cfm_enc);
    }

    APP_LOG_INFO("##################### RELAY peripheral ###################");
    APP_LOG_INFO("event: %s", event_name);
    APP_LOG_INFO(" -state:          %d", s_realy_peripheral.state);
    APP_LOG_INFO(" -conn_index:     %d", s_realy_peripheral.link_info.conn_index);
    APP_LOG_INFO(" -conn_interval:  %d", s_realy_peripheral.link_info.conn_interval);
    APP_LOG_INFO(" -slave_latency:  %d", s_realy_peripheral.link_info.slave_latency);
    APP_LOG_INFO(" -sup_timeout:    %d", s_realy_peripheral.link_info.sup_timeout);
    APP_LOG_INFO(" -peer_addr:      %02X:%02X:%02X:%02X:%02X:%02X", s_realy_peripheral.link_info.peer_addr.gap_addr.addr[5],
                                                                    s_realy_peripheral.link_info.peer_addr.gap_addr.addr[4],
                                                                    s_realy_peripheral.link_info.peer_addr.gap_addr.addr[3],
                                                                    s_realy_peripheral.link_info.peer_addr.gap_addr.addr[2],
                                                                    s_realy_peripheral.link_info.peer_addr.gap_addr.addr[1],
                                                                    s_realy_peripheral.link_info.peer_addr.gap_addr.addr[0]);
    APP_LOG_INFO(" -tx_phy:         %d", s_realy_peripheral.link_info.tx_phy);
    APP_LOG_INFO(" -rx_phy:         %d", s_realy_peripheral.link_info.rx_phy);
    APP_LOG_INFO(" -tx_data_len:    %d", s_realy_peripheral.link_info.tx_data_len);
    APP_LOG_INFO(" -rx_data_len:    %d", s_realy_peripheral.link_info.rx_data_len);
    APP_LOG_INFO(" -gatt_mtu:       %d", s_realy_peripheral.link_info.gatt_mtu);
    APP_LOG_INFO(" -encrypted:      %d", s_realy_peripheral.link_info.encrypted);
    APP_LOG_INFO("########################################################\r\n\r\n");
}

static void relay_hrs_service_process_event(hrs_evt_t *p_evt)
{
    if (p_evt->conn_idx != s_realy_peripheral.link_info.conn_index)
    {
        return;
    }

    switch (p_evt->evt_type)
    {
        case HRS_EVT_NOTIFICATION_ENABLED:
            APP_LOG_DEBUG("Heart Rate Measurement Notification is enabled.");
            break;

        case HRS_EVT_NOTIFICATION_DISABLED:
            APP_LOG_DEBUG("Heart Rate Measurement Notification is disabled.");
            break;

        default:
            break;
    }
}

static void relay_rsc_service_process_event(rscs_evt_t *p_evt)
{
    if (p_evt->conn_idx != s_realy_peripheral.link_info.conn_index)
    {
        return;
    }

    switch (p_evt->evt_type)
    {
        case RSCS_EVT_RSC_MEAS_NOTIFICATION_ENABLE:
            APP_LOG_DEBUG("RSC Measurement Notification is enabled.");
            break;

        case RSCS_EVT_RSC_MEAS_NOTIFICATION_DISABLE:
            APP_LOG_DEBUG("RSC Measurement Notification is enabled.");
            break;

        default:
            break;
    }
}

static void user_relay_peripheral_services_init(void)
{
    rscs_init_t rscs_env_init;
    hrs_init_t  hrs_env_init;
    sdk_err_t   error_code;

    /*------------------------------------------------------------------*/
    hrs_env_init.sensor_loc                  = (hrs_sensor_loc_t)0xff;
    hrs_env_init.char_mask                   = HRS_CHAR_MANDATORY | HRS_CHAR_BODY_SENSOR_LOC_SUP;
    hrs_env_init.evt_handler                 = relay_hrs_service_process_event;
    hrs_env_init.is_sensor_contact_supported = true;
    error_code = hrs_service_init(&hrs_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    rscs_env_init.char_mask                  = HRS_CHAR_MANDATORY | RSCS_CHAR_SENSOR_LOC_SUP;
    rscs_env_init.feature                    = RSCS_FEAR_FULL_BIT;
    rscs_env_init.sensor_location            = (rscs_sensor_loc_t)0xff;
    rscs_env_init.evt_handler                = relay_rsc_service_process_event;
    error_code = rscs_service_init(&rscs_env_init);
    APP_ERROR_CHECK(error_code);
}

sdk_err_t user_relay_peripheral_init(void)
{
    ml_peripheral_init_t    realy_peripheral_init;

    memset(&realy_peripheral_init, 0x00, sizeof(ml_peripheral_init_t));

    realy_peripheral_init.evt_handler     = realy_peripheral_evt_handler;
    realy_peripheral_init.adv_data        = s_realy_peripheral_adv_data;
    realy_peripheral_init.scan_data       = s_realy_peripheral_adv_rsp_data;
    realy_peripheral_init.adv_data_len    = sizeof(s_realy_peripheral_adv_data);
    realy_peripheral_init.scan_data_len   = sizeof(s_realy_peripheral_adv_rsp_data);

    realy_peripheral_init.adv_param.adv_intv_max = APP_ADV_INTERVAL;
    realy_peripheral_init.adv_param.adv_intv_min = APP_ADV_INTERVAL;
    realy_peripheral_init.adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    realy_peripheral_init.adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    realy_peripheral_init.adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    realy_peripheral_init.adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    realy_peripheral_init.adv_time_param.duration     = 0;
    realy_peripheral_init.adv_time_param.max_adv_evt  = 0;

    user_relay_peripheral_services_init();

    return  ml_peripheral_create(&s_realy_peripheral, &realy_peripheral_init);
}

void user_relay_peripheral_hrs_measurement_send(uint16_t heart_rate, bool is_energy_updated)
{
    if (s_realy_peripheral.link_info.conn_index != BLE_GAP_INVALID_CONN_INDEX)
    {
        hrs_heart_rate_measurement_send(s_realy_peripheral.link_info.conn_index, heart_rate, is_energy_updated);
    }
}

void user_relay_peripheral_rscs_measurement_send(rscs_meas_val_t *p_meas)
{
    if (s_realy_peripheral.link_info.conn_index != BLE_GAP_INVALID_CONN_INDEX)
    {
        rscs_measurement_send(s_realy_peripheral.link_info.conn_index, p_meas);
    }
}





