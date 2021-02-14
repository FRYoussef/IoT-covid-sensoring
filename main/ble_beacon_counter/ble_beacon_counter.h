/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 13th 2021
 **/

#ifndef _BEACON_COUNT_
#define _BEACON_COUNT_

#include <math.h>
#include "esp_log.h"
#include "esp_gap_ble_api.h"

typedef struct
{
    bool *beacon_enb;
    uint32_t *max_distance;
    void (* init)(void);
    void (* callback)(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
} beacon_control_args_t;


void beacon_init_vars(bool *enb, uint32_t *d);
void init_beacon_counter(void);
void beacon_fsm(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

#endif