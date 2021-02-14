/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 13th 2021
 **/

#ifndef _BEACON_COUNT_
#define _BEACON_COUNT_

#include <time.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"

#include "circular_buffer/circular_buffer.h"

#define DEVICE_LIST_INITIAL_N 5

typedef struct
{
    bool *beacon_enb;
    uint32_t *max_distance;
    void (* init)(void);
    void (* callback)(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
} beacon_control_args_t;


typedef struct
{
    uint8_t addr[ESP_BD_ADDR_LEN];
    uint32_t last_timestamp;
    struct CircularBuffer buffer;
} beacon_device_t;


typedef struct
{
    beacon_device_t *devices;
    uint32_t counter;
    uint32_t size;
} device_list_t;


void beacon_init_vars(bool *enb, uint32_t *d);
void init_beacon_counter(void);
void beacon_fsm(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void free_beacon_mem(void);
int find_address(uint8_t bda[]);
void add_beacon_from(int rssi, uint8_t bda[]);
void increase_list_size(void);
void remove_device(int index);
void update_ble_devices_char(void *arg);
int get_number_of_devices(void);
void chrono_clean_unreached(void);

#endif