
#include "user_rscs_central.h"
#include "multi_link_central.h"

#include "user_relay_peripheral.h"

#include "rscs_c.h"
#include "rscs.h"
#include "app_log.h"
#include "app_error.h"


static ml_central_t     s_rscs_central;
static uint16_t         s_rscs_uuid = BLE_ATT_SVC_RUNNING_SPEED_CADENCE;

static void rscs_central_evt_handler(ml_evt_t *p_evt)
{
    sdk_err_t   error_code;
 
#define AssignEventName(type) \
    case type: \
        event_name = #type; \
        break

    const char* event_name;
    switch (p_evt->type) {
        AssignEventName(ML_EVT_SCAN_START);
        AssignEventName(ML_EVT_SCAN_STOP);
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
        
    }

    APP_LOG_INFO("##################### RSCS central #####################");
    APP_LOG_INFO("event: %s", event_name);
    APP_LOG_INFO(" -state:          %d", s_rscs_central.state);
    APP_LOG_INFO(" -conn_index:     %d", s_rscs_central.link_info.conn_index);
    APP_LOG_INFO(" -conn_interval:  %d", s_rscs_central.link_info.conn_interval);
    APP_LOG_INFO(" -slave_latency:  %d", s_rscs_central.link_info.slave_latency);
    APP_LOG_INFO(" -sup_timeout:    %d", s_rscs_central.link_info.sup_timeout);
    APP_LOG_INFO(" -peer_addr:      %02X:%02X:%02X:%02X:%02X:%02X", s_rscs_central.link_info.peer_addr.gap_addr.addr[5],
                                                                    s_rscs_central.link_info.peer_addr.gap_addr.addr[4],
                                                                    s_rscs_central.link_info.peer_addr.gap_addr.addr[3],
                                                                    s_rscs_central.link_info.peer_addr.gap_addr.addr[2],
                                                                    s_rscs_central.link_info.peer_addr.gap_addr.addr[1],
                                                                    s_rscs_central.link_info.peer_addr.gap_addr.addr[0]);
    APP_LOG_INFO(" -tx_phy:         %d", s_rscs_central.link_info.tx_phy);
    APP_LOG_INFO(" -rx_phy:         %d", s_rscs_central.link_info.rx_phy);
    APP_LOG_INFO(" -tx_data_len:    %d", s_rscs_central.link_info.tx_data_len);
    APP_LOG_INFO(" -rx_data_len:    %d", s_rscs_central.link_info.rx_data_len);
    APP_LOG_INFO(" -gatt_mtu:       %d", s_rscs_central.link_info.gatt_mtu);
    APP_LOG_INFO(" -encrypted:      %d", s_rscs_central.link_info.encrypted);
    APP_LOG_INFO("########################################################\r\n\r\n");

    switch (p_evt->type)
    {
        case ML_EVT_CONNECT:
            APP_LOG_DEBUG("Running Speed and Cadence Service discovering.");
            error_code = rscs_c_disc_srvc_start(s_rscs_central.link_info.conn_index);
            APP_ERROR_CHECK(error_code);
            break;

        default:
            break;
    }
}

static void rscs_c_evt_process(rscs_c_evt_t *p_evt)
{
    sdk_err_t error_code;

    if (p_evt->conn_idx != s_rscs_central.link_info.conn_index)
    {
        return;
    }

    switch (p_evt->evt_type)
    {
        case RSCS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_DEBUG("Running Speed and Cadence Service discovery completely.");
            APP_LOG_DEBUG("Running Speed and Cadence Service measurement notification enable setting."); 
            error_code = rscs_c_rsc_meas_notify_set(s_rscs_central.link_info.conn_index, true);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_C_EVT_DISCOVERY_FAIL:
            APP_LOG_DEBUG("Running Speed and Cadence Service discovery failed.");
            break;

        case RSCS_C_EVT_RSC_MEAS_NTF_SET_SUCCESS:
            APP_LOG_DEBUG("Running Speed and Cadence Service measurement notification enable set.");
            APP_LOG_DEBUG("Running Speed and Cadence Service sensor location reading.");
            error_code = rscs_c_sensor_loc_read(s_rscs_central.link_info.conn_index);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_C_EVT_SENSOR_LOC_RECEIVE:
            APP_LOG_DEBUG("RSCS sensor location is got.");
            rscs_sensor_loc_update((rscs_sensor_loc_t)p_evt->value.rsc_sensor_loc);
            break;

        case RSCS_C_EVT_RSC_MEAS_VAL_RECEIVE:
            user_relay_peripheral_rscs_measurement_send((rscs_meas_val_t *)&p_evt->value.rsc_meas_buff);
            break;

        default:
            break;
    }
}

static void user_rscs_central_services_client_init(void)
{
    sdk_err_t   error_code;

    error_code = rscs_client_init(rscs_c_evt_process);
    APP_ERROR_CHECK(error_code);
}

sdk_err_t user_rscs_central_init(void)
{
    ml_central_init_t    rscs_central_init;

    rscs_central_init.evt_handler = rscs_central_evt_handler;

    rscs_central_init.target_filter.filter_type = ML_CENTRAL_UUID_FILTER | ML_CENTRAL_APPEARANCE_FILTER,
    rscs_central_init.target_filter.filter_mode = ML_CENTRAL_FILTER_ALL_MATCH,

    rscs_central_init.target_filter.filter_data.appearance       = BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR,
    rscs_central_init.target_filter.filter_data.svr_uuid.length  = sizeof(s_rscs_uuid);
    memcpy(&rscs_central_init.target_filter.filter_data.svr_uuid.p_data, (uint8_t *)&s_rscs_uuid, sizeof(s_rscs_uuid));

    user_rscs_central_services_client_init();

    return  ml_central_create(&s_rscs_central, &rscs_central_init);
}
