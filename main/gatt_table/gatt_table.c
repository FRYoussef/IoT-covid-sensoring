/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 26th 2021
 **/

#include "gatt_table.h"

void function(void *arg) {}


void copy_char(uint8_t *in, uint8_t *out, int n){
    for(int i = 0; i < n; i++)
        out[i] = in[i];
}


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
                ESP_LOGE(CONFIG_LOG_TAG, "advertising start failed");
            }else{
                ESP_LOGI(CONFIG_LOG_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(CONFIG_LOG_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(CONFIG_LOG_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(CONFIG_LOG_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
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


void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            ESP_LOGI(CONFIG_LOG_TAG, "ESP_GATTS_REG_EVT");
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(CONFIG_LOG_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(CONFIG_LOG_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(CONFIG_LOG_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, SEN_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(CONFIG_LOG_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(CONFIG_LOG_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATT_CHAR_VAL_LEN_MAX.
                ESP_LOGI(CONFIG_LOG_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(CONFIG_LOG_TAG, param->write.value, param->write.len);

                // enable/disable characteristic
                uint8_t *char_cfg = NULL;
                enum GattAttr char_idx = IDX_SVC; // use it as default value
                uint8_t *val = NULL;
                int n_elems = 2;
                QueueHandle_t queue = NULL;
                int ev_enb;
                void (* foo)(void *);
                foo = function;

                if (sensoring_handle_table[IDX_CHAR_CO2_ENB] == param->write.handle) {
                    char_cfg = co2_enb;
                    char_idx = IDX_CHAR_CO2_VAL;
                    val = (uint8_t *) &co2_char_value;
                } 
                else if (sensoring_handle_table[IDX_CHAR_TEMP_ENB] == param->write.handle) {
                    char_cfg = temp_enb;
                    char_idx = IDX_CHAR_TEMP_VAL;
                    val = (uint8_t *) &temp_char_value;
                    foo = update_temperature_char;
                    queue = si7021_queue;
                    ev_enb = TEMP_ENABLE;
                }
                else if (sensoring_handle_table[IDX_CHAR_HUM_ENB] == param->write.handle) {
                    char_cfg = hum_enb;
                    char_idx = IDX_CHAR_HUM_VAL;
                    val = (uint8_t *) &hum_char_value;
                    foo = update_humidity_char;
                    queue = si7021_queue;
                    ev_enb = HUM_ENABLE;
                }
                else if (sensoring_handle_table[IDX_CHAR_CAP_ENB] == param->write.handle) {
                    char_cfg = cap_enb;
                    char_idx = IDX_CHAR_CAP_VAL;
                    val = (uint8_t *) &cap_char_value;
                }

                if (char_cfg != NULL && param->write.len == 2){
                    uint8_t descr_value = param->write.value[0];

                    if (descr_value == 1){
                        ESP_LOGI(CONFIG_LOG_TAG, "notify enable");
                        
                        if(char_cfg[0] != descr_value)
                            xQueueSendToFront(queue, (void *) &ev_enb, 100);
                        
                        char_cfg[0] = descr_value;

                        /* Update char value */
                        foo(val);

                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, sensoring_handle_table[char_idx],
                            sizeof(*val)*n_elems, val, false);
                    }
                    else if (descr_value == 0){
                        ESP_LOGI(CONFIG_LOG_TAG, "notify disable ");
                        
                        if(char_cfg[0] != descr_value)
                            xQueueSendToFront(queue, (void *) &ev_enb, 100);

                        char_cfg[0] = descr_value;
                    }else{
                        ESP_LOGE(CONFIG_LOG_TAG, "unknown descr value");
                        esp_log_buffer_hex(CONFIG_LOG_TAG, param->write.value, param->write.len);
                    }
                }
                else if(sensoring_handle_table[IDX_CHAR_CO2_T_CFG] == param->write.handle && param->write.len == 2) {
                    copy_char(param->write.value, co2_ccc, 2);
                    ESP_LOGI(CONFIG_LOG_TAG, "Modified IDX_CHAR_CO2_T_CFG to %d", co2_ccc[1] << 8 | co2_ccc[0]);
                }
                else if(sensoring_handle_table[IDX_CHAR_TEMP_T_CFG] == param->write.handle && param->write.len == 2) {
                    copy_char(param->write.value, temp_ccc, 2);
                    gatt_ev = TEMP_SAMPLE_FREQ;
                    xQueueSendToFront(si7021_queue, (void *) &gatt_ev, 100);
                    ESP_LOGI(CONFIG_LOG_TAG, "Modified IDX_CHAR_TEMP_T_CFG to %d", temp_ccc[1] << 8 | temp_ccc[0]);
                }
                else if(sensoring_handle_table[IDX_CHAR_HUM_T_CFG] == param->write.handle && param->write.len == 2) {
                    copy_char(param->write.value, hum_ccc, 2);
                    gatt_ev = HUM_SAMPLE_FREQ;
                    xQueueSendToFront(si7021_queue, (void *) &gatt_ev, 100);
                    ESP_LOGI(CONFIG_LOG_TAG, "Modified IDX_CHAR_HUM_T_CFG to %d", hum_ccc[1] << 8 | hum_ccc[0]);
                }
                else if(sensoring_handle_table[IDX_CHAR_CAP_D_CFG] == param->write.handle && param->write.len == 2) {
                    copy_char(param->write.value, cap_ccc, 2);
                    ESP_LOGI(CONFIG_LOG_TAG, "Modified IDX_CHAR_CAP_D_CFG to %d", cap_ccc[1] << 8 | cap_ccc[0]);
                }

                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT: 
            // the length of gattc prepare write data must be less than GATT_CHAR_VAL_LEN_MAX. 
            ESP_LOGI(CONFIG_LOG_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(CONFIG_LOG_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(CONFIG_LOG_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(CONFIG_LOG_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(CONFIG_LOG_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(CONFIG_LOG_TAG, param->connect.remote_bda, 6);
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
            ESP_LOGI(CONFIG_LOG_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(CONFIG_LOG_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != SEN_IDX_NB){
                ESP_LOGE(CONFIG_LOG_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to SEN_IDX_NB(%d)", param->add_attr_tab.num_handle, SEN_IDX_NB);
            }
            else {
                ESP_LOGI(CONFIG_LOG_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(sensoring_handle_table, param->add_attr_tab.handles, sizeof(sensoring_handle_table));
                esp_ble_gatts_start_service(sensoring_handle_table[IDX_SVC]);
            }
            break;
        }
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
            ESP_LOGE(CONFIG_LOG_TAG, "reg app failed, app_id %04x, status %d",
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

void configure_gatt_server(QueueHandle_t q1) {
    si7021_queue = q1;
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(CONFIG_LOG_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(CONFIG_LOG_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(CONFIG_LOG_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(CONFIG_LOG_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(CONFIG_LOG_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(CONFIG_LOG_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(CONFIG_LOG_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(CONFIG_LOG_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}
