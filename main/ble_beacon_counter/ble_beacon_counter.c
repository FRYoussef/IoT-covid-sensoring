#include "ble_beacon_counter/ble_beacon_counter.h"

esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static const float MEASURED_POWER = -69.0f;
static const float N = 2.0f;
bool beacon_enb = true;
uint32_t max_distance = 20;


void beacon_init_vars(bool *enb, uint32_t *d){
    enb = &beacon_enb;
    d = &max_distance;
    
    // avoid compiler error
    *d = 20;
    *enb = true;
}


void init_beacon_counter(void) {
    esp_ble_gap_set_scan_params(&ble_scan_params);
}


void beacon_fsm(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if(!beacon_enb)
        return;

    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second, 0 means scan permanently
        esp_ble_gap_start_scanning(0);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(CONFIG_LOG_TAG, "Scan start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
                printf("Msg from device ");
                for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
                    printf("%02x", scan_result->scan_rst.bda[i]);
                }

                int rssi = scan_result->scan_rst.rssi;            
                printf(", RSSI = %i", rssi);
                double d = pow(10, ((MEASURED_POWER - rssi)/(10 * N)));
                printf(", Distance = %f meters\n", d);
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(CONFIG_LOG_TAG, "Scan stop failed: %s", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}