
#include "user_hrs_central.h"
#include "multi_link_central.h"

#include "user_relay_peripheral.h"

#include "hrs_c.h"
#include "hrs.h"
#include "app_log.h"
#include "app_error.h"


static ml_central_t         s_hrs_central;
static uint16_t             s_hrs_uuid = BLE_ATT_SVC_HEART_RATE;


static void hrs_central_evt_handler(ml_evt_t *p_evt)
{
    sdk_err_t   error_code;
 
#define AssignEventName(type) \
    case type: \
        event_name = #type; \
        break

    const char* event_name;
    switch (p_evt->type)
    {
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

    APP_LOG_INFO("##################### HRS central #####################");
    APP_LOG_INFO("event: %s", event_name);
    APP_LOG_INFO(" -state:          %d", s_hrs_central.state);
    APP_LOG_INFO(" -conn_index:     %d", s_hrs_central.link_info.conn_index);
    APP_LOG_INFO(" -conn_interval:  %d", s_hrs_central.link_info.conn_interval);
    APP_LOG_INFO(" -slave_latency:  %d", s_hrs_central.link_info.slave_latency);
    APP_LOG_INFO(" -sup_timeout:    %d", s_hrs_central.link_info.sup_timeout);
    APP_LOG_INFO(" -peer_addr:      %02X:%02X:%02X:%02X:%02X:%02X", s_hrs_central.link_info.peer_addr.gap_addr.addr[5],
                                                                    s_hrs_central.link_info.peer_addr.gap_addr.addr[4],
                                                                    s_hrs_central.link_info.peer_addr.gap_addr.addr[3],
                                                                    s_hrs_central.link_info.peer_addr.gap_addr.addr[2],
                                                                    s_hrs_central.link_info.peer_addr.gap_addr.addr[1],
                                                                    s_hrs_central.link_info.peer_addr.gap_addr.addr[0]);
    APP_LOG_INFO(" -tx_phy:         %d", s_hrs_central.link_info.tx_phy);
    APP_LOG_INFO(" -rx_phy:         %d", s_hrs_central.link_info.rx_phy);
    APP_LOG_INFO(" -tx_data_len:    %d", s_hrs_central.link_info.tx_data_len);
    APP_LOG_INFO(" -rx_data_len:    %d", s_hrs_central.link_info.rx_data_len);
    APP_LOG_INFO(" -gatt_mtu:       %d", s_hrs_central.link_info.gatt_mtu);
    APP_LOG_INFO(" -encrypted:      %d", s_hrs_central.link_info.encrypted);
    APP_LOG_INFO("########################################################\r\n\r\n");

    switch (p_evt->type)
    {
        case ML_EVT_CONNECT:
            APP_LOG_DEBUG("Heart Rate Service discovering.");
            error_code = hrs_c_disc_srvc_start(s_hrs_central.link_info.conn_index);
            APP_ERROR_CHECK(error_code);
            break;

        default:
            break;
    }
}

static void hrs_c_evt_process(hrs_c_evt_t *p_evt)
{
    sdk_err_t error_code;

    if (p_evt->conn_idx != s_hrs_central.link_info.conn_index)
    {
        return;
    }

    uint8_t rr_intervals_idx = 0;

    switch (p_evt->evt_type)
    {
        case HRS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_DEBUG("Heart Rate Service discovery completely.");
            APP_LOG_DEBUG("Heart Rate Service measurement notification enable setting."); 
            error_code = hrs_c_heart_rate_meas_notify_set(s_hrs_central.link_info.conn_index, true);
            APP_ERROR_CHECK(error_code);
            break;

        case HRS_C_EVT_DISCOVERY_FAIL:
            APP_LOG_DEBUG("Heart Rate Service discovery failed.");
            break;

        case HRS_C_EVT_HR_MEAS_NTF_SET_SUCCESS:
            APP_LOG_DEBUG("Heart Rate Service measurement notification enable set."); 
            APP_LOG_DEBUG("Heart Rate Service sensor location reading.");
            error_code = hrs_c_sensor_loc_read(s_hrs_central.link_info.conn_index);
            APP_ERROR_CHECK(error_code);
            break;

        case HRS_C_EVT_HR_MEAS_VAL_RECEIVE:
            for (rr_intervals_idx = 0; rr_intervals_idx < p_evt->value.hr_meas_buff.rr_intervals_num; rr_intervals_idx++)
            {
                hrs_rr_interval_add(p_evt->value.hr_meas_buff.rr_intervals[rr_intervals_idx]);
            }
            hrs_sensor_contact_detected_update(p_evt->value.hr_meas_buff.is_sensor_contact_detected);

            user_relay_peripheral_hrs_measurement_send(p_evt->value.hr_meas_buff.hr_value, p_evt->value.hr_meas_buff.energy_expended);
            break;

        case HRS_C_EVT_SENSOR_LOC_READ_RSP:
            APP_LOG_DEBUG("HRS sensor location is got.");
            hrs_sensor_location_set((hrs_sensor_loc_t)p_evt->value.sensor_loc);
            break;

        default:
            break;
    }
}

static void user_hrs_central_services_client_init(void)
{
    sdk_err_t   error_code;

    error_code = hrs_client_init(hrs_c_evt_process);
    APP_ERROR_CHECK(error_code);
}

sdk_err_t user_hrs_central_init(void)
{
    ml_central_init_t    hrs_central_init;

    hrs_central_init.evt_handler = hrs_central_evt_handler;

    hrs_central_init.target_filter.filter_type = ML_CENTRAL_UUID_FILTER | ML_CENTRAL_APPEARANCE_FILTER,
    hrs_central_init.target_filter.filter_mode = ML_CENTRAL_FILTER_ALL_MATCH,

    hrs_central_init.target_filter.filter_data.appearance       = BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR,
    hrs_central_init.target_filter.filter_data.svr_uuid.length  = sizeof(s_hrs_uuid);
    memcpy(&hrs_central_init.target_filter.filter_data.svr_uuid.p_data, (uint8_t *)&s_hrs_uuid, sizeof(s_hrs_uuid));

    user_hrs_central_services_client_init();

    return  ml_central_create(&s_hrs_central, &hrs_central_init);
}
