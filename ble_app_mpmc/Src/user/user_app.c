
#include "gr_includes.h"

#include "app_log.h"
#include "app_error.h"

#include "user_hrs_central.h"
#include "user_rscs_central.h"
#include "multi_link_central.h"

#include "user_relay_peripheral.h"
#include "user_gus_peripheral.h"
#include "multi_link_peripheral.h"


#define APP_SCAN_INTERVAL              160
#define APP_SCAN_WINDOW                80
#define APP_SCAN_TIMEOUT               0


static ble_gap_scan_param_t s_scan_param = 
{
   .scan_type     = BLE_GAP_SCAN_ACTIVE,
   .scan_mode     = BLE_GAP_SCAN_GEN_DISC_MODE,
   .scan_dup_filt = BLE_GAP_SCAN_FILT_DUPLIC_DIS,
   .use_whitelist = false,
   .interval      = APP_SCAN_INTERVAL,
   .window        = APP_SCAN_WINDOW,
   .timeout       = APP_SCAN_TIMEOUT,
};

static ble_sec_param_t s_sec_param =
{
    .level     = BLE_SEC_MODE1_LEVEL2,
    .io_cap    = BLE_SEC_IO_NO_INPUT_NO_OUTPUT,
    .oob       = false,
    .auth      = BLE_SEC_AUTH_BOND,
    .key_size  = 16,
    .ikey_dist = BLE_SEC_KDIST_ALL,
    .rkey_dist = BLE_SEC_KDIST_ALL,
};

void ble_app_init(void)
{
    sdk_err_t        error_code;
    ble_gap_bdaddr_t bd_addr;
    sdk_version_t    version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                 version.major, version.minor, version.build, version.commit_id);

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);
    APP_LOG_INFO("Local Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);
    APP_LOG_INFO("BLE Multi Peripheral & Multi Central example started.");

    /*------------------------------------------------------------------*/
    error_code = ble_gap_data_length_set(251, 2120);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_l2cap_params_set(247, 247, 1);
    APP_ERROR_CHECK(error_code);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);

    ble_gap_pair_enable(true);

    error_code = ble_sec_params_set(&s_sec_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_privacy_params_set(900, true);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    error_code = user_hrs_central_init();
    APP_ERROR_CHECK(error_code);

    error_code = user_rscs_central_init();
    APP_ERROR_CHECK(error_code);

    error_code = ml_central_scan_start(&s_scan_param);
    APP_ERROR_CHECK(error_code);


    /*------------------------------------------------------------------*/
    error_code = user_relay_peripheral_init();
    APP_ERROR_CHECK(error_code);

    error_code = user_gus_peripheral_init();
    APP_ERROR_CHECK(error_code);

    error_code = ml_peripheral_adv_start();
    APP_ERROR_CHECK(error_code);
}

void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        default:
            break;
    }

    ml_central_evt_handler(p_evt);
    ml_peripheral_evt_handler(p_evt);
}
