/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 26th 2021
 **/

#include "gatt_table.h"


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATT_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATT_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATT_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATT_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATT_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}


static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATT_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATT_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATT_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, SEN_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATT_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATT_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATT_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATT_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATT_TABLE_TAG, param->write.value, param->write.len);

                // enable/disable characteristic
                enum GattAttr enb_char_idx = IDX_SVC; // use it as default value
                enum GattAttr char_idx = IDX_SVC;
                uint8_t val = 0;

                if (sensoring_handle_table[IDX_CHAR_CO2_ENB] == param->write.handle) {
                    enb_char_idx = IDX_CHAR_CO2_ENB;
                    char_idx = IDX_CHAR_CO2_VAL;
                    val = co2_char_value;
                } 
                else if (sensoring_handle_table[IDX_CHAR_TEMP_ENB] == param->write.handle) {
                    enb_char_idx = IDX_CHAR_TEMP_ENB;
                    char_idx = IDX_CHAR_TEMP_VAL;
                    val = temp_char_value;
                }
                else if (sensoring_handle_table[IDX_CHAR_CAP_ENB] == param->write.handle) {
                    enb_char_idx = IDX_CHAR_CAP_ENB;
                    char_idx = IDX_CHAR_CAP_VAL;
                    val = cap_char_value;
                }

                if (enb_char_idx != IDX_SVC && param->write.len == 1){
                    uint8_t descr_value = param->write.value[0];

                    if (descr_value == 1){
                        ESP_LOGI(GATT_TABLE_TAG, "notify enable");

                        esp_ble_gatts_set_attr_value(sensoring_handle_table[enb_char_idx], sizeof(descr_value), (uint8_t *)descr_value);

                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, sensoring_handle_table[char_idx],
                                                sizeof(val), val, false);
                    }
                    else if (descr_value == 0){
                        ESP_LOGI(GATT_TABLE_TAG, "notify disable ");
                        esp_ble_gatts_set_attr_value(sensoring_handle_table[enb_char_idx], sizeof(descr_value), (uint8_t *)descr_value);
                    }else{
                        ESP_LOGE(GATT_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATT_TABLE_TAG, param->write.value, param->write.len);
                    }
                }
                else if(heart_rate_handle_table[IDX_CHAR_CO2_T_CFG] == param->write.handle && param->write.len == 2) {
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    esp_ble_gatts_set_attr_value(sensoring_handle_table[IDX_CHAR_CO2_T_CFG], sizeof(descr_value), (uint8_t *)descr_value);
                    ESP_LOGI(GATT_TABLE_TAG, "Modified IDX_CHAR_CO2_T_CFG to %d", descr_value);
                }
                else if(heart_rate_handle_table[IDX_CHAR_TEMP_T_CFG] == param->write.handle && param->write.len == 2) {
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    esp_ble_gatts_set_attr_value(sensoring_handle_table[IDX_CHAR_TEMP_T_CFG], sizeof(descr_value), (uint8_t *)descr_value);
                    ESP_LOGI(GATT_TABLE_TAG, "Modified IDX_CHAR_TEMP_T_CFG to %d", descr_value);
                }
                else if(heart_rate_handle_table[IDX_CHAR_CAP_D_CFG] == param->write.handle && param->write.len == 2) {
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    esp_ble_gatts_set_attr_value(sensoring_handle_table[IDX_CHAR_CAP_D_CFG], sizeof(descr_value), (uint8_t *)descr_value);
                    ESP_LOGI(GATT_TABLE_TAG, "Modified IDX_CHAR_CAP_D_CFG to %d", descr_value);
                }

                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT: 
            // the length of gattc prepare write data must be less than GATT_CHAR_VAL_LEN_MAX. 
            ESP_LOGI(GATT_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            //example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATT_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATT_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATT_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATT_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATT_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATT_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            sensoring_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATT_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == sensoring_profile_tab[idx].gatts_if) {
                if (sensoring_profile_tab[idx].gatts_cb) {
                    sensoring_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void gatt_server_task(void *pvparameters)
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATT_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // TODO: check if mode ESP_BT_MODE_BTDM (BLE + BT) is required for capacity sensoring
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATT_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATT_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATT_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATT_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATT_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATT_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATT_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    vTaskDelete(NULL);
}
