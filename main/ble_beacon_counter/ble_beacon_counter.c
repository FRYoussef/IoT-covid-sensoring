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
static const float BEACON_N = 2.0f;
bool *beacon_enb = NULL;
uint32_t *max_distance = NULL; //meters
device_list_t d_list = {
    .size = DEVICE_LIST_INITIAL_N,
    .counter = 0,
    .devices = NULL,
};


int find_address(uint8_t bda[]){
    bool found = true;

    for(int i = 0; i < d_list.counter; i++){
        found = true;
        for(int j = 0; j < ESP_BD_ADDR_LEN; j++) {
            found = found && (bda[j] == d_list.devices[i].addr[j]);
        }
        if (found)
            return i;
    }

    return -1;
}


void remove_device(int index) {
    for(int i = index; i < d_list.counter-1; i++){
        free_buffer(&d_list.devices[i].buffer);
        d_list.devices[i] = d_list.devices[i+1];
    }
    d_list.counter--;
}


void add_beacon_from(int rssi, uint8_t bda[]) {
    float d = pow(10, ((MEASURED_POWER - rssi)/(10 * BEACON_N)));

    if(d > *max_distance)
        return;

    int index = find_address(bda);
    time_t now;
    time(&now);

    if(index == -1) {
        if(d_list.counter == d_list.size)
            increase_list_size();

        for(int j = 0; j < ESP_BD_ADDR_LEN; j++)
            d_list.devices[d_list.counter].addr[j] = bda[j];

        index = d_list.counter;
        d_list.counter++;
        init_buffer(&d_list.devices[index].buffer, CONFIG_BEACON_COUNTER_WINDOW_SIZE);
    }

    d_list.devices[index].last_timestamp = now;
    add_element(&d_list.devices[index].buffer, d);
}


void increase_list_size(void){
    beacon_device_t *new = (beacon_device_t *) malloc(sizeof(beacon_device_t)*d_list.size*2);

    // copy
    for(int i = 0; i < d_list.counter; i++){
        new[i].last_timestamp = d_list.devices[i].last_timestamp;
        new[i].buffer = d_list.devices[i].buffer;
        for(int j = 0; j < ESP_BD_ADDR_LEN; j++) {
            new[i].addr[j] = d_list.devices[i].addr[j];
        }
    }
    d_list.size *= 2; 
    free(d_list.devices);
    d_list.devices = new;
}


void beacon_init_vars(bool *enb, uint32_t *d){
    beacon_enb = enb;
    max_distance = d;

    d_list.devices = (beacon_device_t *) malloc(sizeof(beacon_device_t)*d_list.size);
}


void free_beacon_mem(void){
    for(int i = 0; i < d_list.counter; i++)
        free_buffer(&d_list.devices[i].buffer);

    free(d_list.devices);
}


void init_beacon_counter(void) {
    esp_ble_gap_set_scan_params(&ble_scan_params);
}


void beacon_fsm(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if(!*beacon_enb)
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
                add_beacon_from(scan_result->scan_rst.rssi, scan_result->scan_rst.bda);
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


int get_number_of_devices(void){
    int devices = 0;
    double mean = 0;

    // iterate over all registered devices
    for(int i = 0; i < d_list.counter; i++){
        // for each device do moving average
        mean = 0;
        for(int j = 0; j < d_list.devices[i].buffer.counter; j++)
            mean += get_element(&d_list.devices[i].buffer);

        mean /= d_list.devices[i].buffer.counter;
        if(mean <= *max_distance)
            devices++;
    }

    return devices;
}


void update_ble_devices_char(void *arg) {
    uint8_t *dev = (uint8_t*) arg;
    int n_devices = get_number_of_devices();

    dev[0] = (uint8_t)((n_devices >> 8) & 0xFF);
    dev[1] = (uint8_t)(n_devices & 0xFF);
}
